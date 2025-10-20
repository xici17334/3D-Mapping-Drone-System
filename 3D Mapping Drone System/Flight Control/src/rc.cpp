/*
 * MIT License
 *
 * Copyright (c) 2024 Kouhei Ito
 * Copyright (c) 2024 M5Stack
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "rc.hpp"
#include "tof.hpp" // 包含 ToF 头文件
#include <WiFi.h>
#include <WiFiUdp.h>
#include "flight_control.hpp"

// WiFi and UDP Settings
const char* ssid = "crf0408";
const char* password = "15371537";
const int localUdpPort = 4210;

WiFiUDP udp;
IPAddress remoteIP;
uint16_t remotePort = 0;
char packetBuffer[255];

// Connection tracking
volatile uint32_t last_packet_time = 0;

volatile uint8_t MyMacAddr[6];
volatile uint8_t Rc_err_flag = 0;

// RC
volatile float Stick[16];
volatile uint8_t ahrs_reset_flag = 0;

// ========== ToF 相关全局变量 ==========
volatile int16_t tof_bottom_distance = 0;
volatile int16_t tof_front_distance = 0;
volatile uint32_t last_tof_read_time = 0;
const uint32_t TOF_READ_INTERVAL = 50; // 后台读取间隔 50ms (20Hz)

// 遥测数据包结构（ToF响应）
struct TelemetryPacket {
    uint8_t header[2];      // 'T', 'F'
    int16_t bottom_distance;
    int16_t front_distance;
    uint32_t timestamp;
    uint8_t checksum;
} __attribute__((packed));


// ========== 立即发送 ToF 数据（响应请求） ==========
void send_tof_telemetry_immediate(void) {
    if (remotePort == 0) return;

    TelemetryPacket packet;
    packet.header[0] = 'T';
    packet.header[1] = 'F';
    packet.bottom_distance = tof_bottom_distance; // 使用后台更新的最新值
    packet.front_distance = tof_front_distance;   // 使用后台更新的最新值
    packet.timestamp = millis();

    // 计算校验和
    uint8_t *data = (uint8_t*)&packet;
    uint8_t checksum = 0;
    for (size_t i = 0; i < sizeof(TelemetryPacket) - 1; i++) {
        checksum += data[i];
    }
    packet.checksum = checksum;

    // 发送
    udp.beginPacket(remoteIP, remotePort);
    udp.write((uint8_t*)&packet, sizeof(TelemetryPacket));
    udp.endPacket();
}

// ========== 优化后的遥控数据解析函数 ==========
void parse_rc_packet(const uint8_t *recv_data, int data_len) {
    last_packet_time = millis();

    // 1. 基本长度校验 (RC基础包为25字节)
    if (data_len < 25) {
        Rc_err_flag = 1;
        return;
    }

    // 2. MAC 地址校验
    if ((recv_data[0] != MyMacAddr[3]) || 
        (recv_data[1] != MyMacAddr[4]) || 
        (recv_data[2] != MyMacAddr[5])) {
        Rc_err_flag = 1;
        return;
    }

    // 3. 校验和
    uint8_t check_sum = 0;
    for (uint8_t i = 0; i < 24; i++) {
        check_sum += recv_data[i];
    }
    if (check_sum != recv_data[24]) {
        Rc_err_flag = 1;
        return;
    }

    // 所有校验通过
    Rc_err_flag = 0;

    // 4. 解析摇杆数据 (使用 union 优化字节到浮点数的转换)
    union {
        uint8_t bytes[4];
        float value;
    } converter;

    // Rudder
    memcpy(converter.bytes, &recv_data[3], 4);
    Stick[RUDDER] = converter.value;

    // Throttle
    memcpy(converter.bytes, &recv_data[7], 4);
    Stick[THROTTLE] = converter.value;

    // Aileron
    memcpy(converter.bytes, &recv_data[11], 4);
    Stick[AILERON] = converter.value;

    // Elevator
    memcpy(converter.bytes, &recv_data[15], 4);
    Stick[ELEVATOR] = converter.value;

    // 5. 解析按钮和模式
    Stick[BUTTON_ARM]     = recv_data[19];
    Stick[BUTTON_FLIP]    = recv_data[20];
    Stick[CONTROLMODE]    = recv_data[21];
    Stick[ALTCONTROLMODE] = recv_data[22];
    ahrs_reset_flag       = recv_data[23];

    // 6. 【关键】检查 ToF 请求标志并响应
    //    RC数据包结构: Byte 25: ToF 请求标志 (0=不请求, 1=请求)
    bool tof_requested = (data_len >= 26 && recv_data[25] == 1);
    if (tof_requested) {
        // 仅在收到请求时发送 ToF 数据
        send_tof_telemetry_immediate();
    }
}


void rc_init(void) {
    for (uint8_t i = 0; i < 16; i++) Stick[i] = 0.0;

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    USBSerial.print("Connecting to WiFi...");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        USBSerial.print(".");
    }
    USBSerial.println("\nConnected to WiFi");
    USBSerial.print("IP Address: ");
    USBSerial.println(WiFi.localIP());

    WiFi.macAddress((uint8_t *)MyMacAddr);
    USBSerial.printf("MAC ADDRESS: %02X:%02X:%02X:%02X:%02X:%02X\r\n", 
                     MyMacAddr[0], MyMacAddr[1], MyMacAddr[2],
                     MyMacAddr[3], MyMacAddr[4], MyMacAddr[5]);

    if (udp.begin(localUdpPort)) {
        USBSerial.printf("UDP listening on port %d\n", localUdpPort);
    } else {
        USBSerial.println("Failed to start UDP listener.");
    }
}


// ========== 后台定时读取 ToF（非阻塞） ==========
void tof_read_background(void) {
    uint32_t current_time = millis();
    if (current_time - last_tof_read_time >= TOF_READ_INTERVAL) {
        last_tof_read_time = current_time;
        
        // 读取传感器并更新全局变量
        // 注意: 这里的 get_range 函数应该是快速、非阻塞的
        tof_bottom_distance = tof_bottom_get_range();
        tof_front_distance = tof_front_get_range();
    }
}

// ========== 优化后的主更新函数 ==========
void rc_update(void) {
    // 1. (高优先级) 检查并处理来自遥控器的UDP数据包
    int packetSize = udp.parsePacket();
    if (packetSize) {
        remoteIP = udp.remoteIP();
        remotePort = udp.remotePort();
        int len = udp.read(packetBuffer, 255);
        if (len > 0) {
            parse_rc_packet((uint8_t*)packetBuffer, len);
        }
    }
    
    // 2. (低优先级) 在后台持续更新ToF传感器数据
    tof_read_background();
}


void rc_end(void) {
    WiFi.disconnect(true);
}

uint8_t rc_isconnected(void) {
    if (millis() - last_packet_time < 500) {
        return 1; // Connected
    }
    return 0; // Disconnected
}

// ========== 已删除旧的 ToF 相关函数 ==========
// void tof_read_sensors(void) { ... }
// void tof_send_telemetry(void) { ... }
// void tof_telemetry_update(void) { ... }
// 这些函数的功能已被 tof_read_background() 和 send_tof_telemetry_immediate() 替代

void rc_demo() {
    // This function is not used, kept empty.
}