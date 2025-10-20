#include "PMW3901_Sensor.hpp"

// ===== 初始化函数 =====
bool PMW3901_Sensor::init(int chip_select) {
    cs_pin = chip_select;
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);
    
    spiSettings = SPISettings(2000000, MSBFIRST, SPI_MODE3); // 2MHz SPI clock
    SPI.begin();
    
    delay(50);
    
    if (!reset()) {
        Serial.println("PMW3901 reset failed!");
        return false;
    }
    
    uint8_t product_id = read_register(PMW3901_PRODUCT_ID);
    uint8_t revision_id = read_register(PMW3901_REVISION_ID);
    
    Serial.print("PMW3901 Product ID: 0x");
    Serial.print(product_id, HEX);
    Serial.print(", Revision: 0x");
    Serial.println(revision_id, HEX);
    
    if (product_id != 0x49) {
        Serial.println("Wrong product ID!");
        return false;
    }
    
    write_register(PMW3901_LEDCTL, 0x1F);
    
    reset_position();
    last_update_time = millis();
    update_scale_factor(1.0);
    
    Serial.println("PMW3901 initialized successfully!");
    return true;
}

// ===== 更新缩放因子 (根据高度) =====
void PMW3901_Sensor::update_scale_factor(float altitude) {
    scale_factor = (PIXEL_SIZE * altitude) / FOCAL_LENGTH;
}

// ===== 主更新函数 =====
bool PMW3901_Sensor::update(float current_altitude) {
    update_scale_factor(current_altitude);
    
    uint32_t now = millis();
    dt = (now - last_update_time) / 1000.0;
    last_update_time = now;
    
    if (!read_motion_data()) {
        debug.valid = false;
        return false;
    }
    
    if (surface_quality < MIN_QUALITY) {
        debug.valid = false;
        return false;
    }
    
    float dx_m = raw_dx * scale_factor;
    float dy_m = raw_dy * scale_factor;
    
    if (dt > 0) {
        velocity_x = dx_m / dt;
        velocity_y = dy_m / dt;
        
        filtered_vx = alpha * filtered_vx + (1 - alpha) * velocity_x;
        filtered_vy = alpha * filtered_vy + (1 - alpha) * velocity_y;
    }
    
    position_x += dx_m;
    position_y += dy_m;
    
    float move_dist = sqrt(dx_m * dx_m + dy_m * dy_m);
    distance += move_dist;
    
    if (fabs(dx_m) > 0.001 || fabs(dy_m) > 0.001) {
        heading = atan2(dy_m, dx_m);
    }
    
    debug.raw_x = raw_dx;
    debug.raw_y = raw_dy;
    debug.quality = surface_quality;
    debug.pos_x = position_x;
    debug.pos_y = position_y;
    debug.vel_x = filtered_vx;
    debug.vel_y = filtered_vy;
    debug.heading_deg = heading * 180.0 / PI;
    debug.distance = distance;
    debug.valid = true;
    
    return true;
}

// ===== 重置位置 =====
void PMW3901_Sensor::reset_position() {
    position_x = 0;
    position_y = 0;
    distance = 0;
    heading = 0;
    velocity_x = 0;
    velocity_y = 0;
    filtered_vx = 0;
    filtered_vy = 0;
}

// ===== 获取当前位置和角度 =====
void PMW3901_Sensor::get_position(float &x, float &y, float &angle_deg) {
    x = position_x;
    y = position_y;
    angle_deg = heading * 180.0 / PI;
}

// ===== 打印调试信息 =====
void PMW3901_Sensor::print_debug() {
    Serial.print("Raw: X=");
    Serial.print(debug.raw_x);
    Serial.print(" Y=");
    Serial.print(debug.raw_y);
    Serial.print(" | Qual=");
    Serial.print(debug.quality);
    Serial.print(" | Pos(m): X=");
    Serial.print(debug.pos_x, 3);
    Serial.print(" Y=");
    Serial.print(debug.pos_y, 3);
    Serial.print(" | Vel(m/s): X=");
    Serial.print(debug.vel_x, 3);
    Serial.print(" Y=");
    Serial.print(debug.vel_y, 3);
    Serial.print(" | Heading=");
    Serial.print(debug.heading_deg, 1);
    Serial.print("° | Dist=");
    Serial.print(debug.distance, 3);
    Serial.print("m | Valid=");
    Serial.println(debug.valid ? "YES" : "NO");
}

// ===== SPI读写函数 =====
uint8_t PMW3901_Sensor::read_register(uint8_t reg) {
    SPI.beginTransaction(spiSettings);
    digitalWrite(cs_pin, LOW);
    
    SPI.transfer(reg & 0x7F);
    delayMicroseconds(50);
    uint8_t value = SPI.transfer(0);
    
    digitalWrite(cs_pin, HIGH);
    SPI.endTransaction();
    
    delayMicroseconds(200);
    return value;
}

void PMW3901_Sensor::write_register(uint8_t reg, uint8_t value) {
    SPI.beginTransaction(spiSettings);
    digitalWrite(cs_pin, LOW);
    
    SPI.transfer(reg | 0x80);
    delayMicroseconds(50);
    SPI.transfer(value);
    
    digitalWrite(cs_pin, HIGH);
    SPI.endTransaction();
    
    delayMicroseconds(200);
}

// ===== 读取运动数据 =====
bool PMW3901_Sensor::read_motion_data() {
    uint8_t motion = read_register(PMW3901_MOTION);
    motion_detected = motion & 0x80;
    
    if (!motion_detected) {
        raw_dx = 0;
        raw_dy = 0;
        return false;
    }
    
    uint8_t dx_l = read_register(PMW3901_DELTA_X_L);
    uint8_t dx_h = read_register(PMW3901_DELTA_X_H);
    uint8_t dy_l = read_register(PMW3901_DELTA_Y_L);
    uint8_t dy_h = read_register(PMW3901_DELTA_Y_H);
    
    raw_dx = (int16_t)((dx_h << 8) | dx_l);
    raw_dy = (int16_t)((dy_h << 8) | dy_l);
    
    surface_quality = read_register(PMW3901_SQUAL);
    
    return true;
}

// ===== 复位传感器 =====
bool PMW3901_Sensor::reset() {
    write_register(PMW3901_POWER_UP_RESET, 0x5A);
    delay(50);
    
    for (int i = 0; i < 10; i++) {
        if (read_register(PMW3901_MOTION) & 0x80) {
            return true;
        }
        delay(10);
    }
    return false;
}