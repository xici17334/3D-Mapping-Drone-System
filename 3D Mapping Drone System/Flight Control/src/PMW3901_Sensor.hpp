#ifndef PMW3901_SENSOR_HPP
#define PMW3901_SENSOR_HPP

#include <Arduino.h>
#include <SPI.h>
#include <math.h>

// ===== PMW3901 寄存器定义 =====
#define PMW3901_PRODUCT_ID      0x00
#define PMW3901_REVISION_ID     0x01
#define PMW3901_MOTION          0x02
#define PMW3901_DELTA_X_L       0x03
#define PMW3901_DELTA_X_H       0x04
#define PMW3901_DELTA_Y_L       0x05
#define PMW3901_DELTA_Y_H       0x06
#define PMW3901_SQUAL           0x07
#define PMW3901_RAWDATA_SUM     0x08
#define PMW3901_MAXIMUM_RAWDATA 0x09
#define PMW3901_MINIMUM_RAWDATA 0x0A
#define PMW3901_SHUTTER_LOWER   0x0B
#define PMW3901_SHUTTER_UPPER   0x0C
#define PMW3901_OBSERVATION     0x15
#define PMW3901_MOTION_BURST    0x16
#define PMW3901_POWER_UP_RESET  0x3A
#define PMW3901_SHUTDOWN        0x3B
#define PMW3901_RES_STEP        0x85
#define PMW3901_LEDCTL          0x7F

class PMW3901_Sensor {
public:
    // 调试信息结构
    struct DebugInfo {
        int16_t raw_x;
        int16_t raw_y;
        uint8_t quality;
        float pos_x;
        float pos_y;
        float vel_x;
        float vel_y;
        float heading_deg;
        float distance;
        bool valid;
    } debug;

    // ===== 公有函数 =====
    bool init(int chip_select);
    void update_scale_factor(float altitude);
    bool update(float current_altitude = 1.0);
    void reset_position();
    void get_position(float &x, float &y, float &angle_deg);
    void print_debug();

private:
    // ===== 私有函数 =====
    uint8_t read_register(uint8_t reg);
    void write_register(uint8_t reg, uint8_t value);
    bool read_motion_data();
    bool reset();

    // ===== 私有变量 =====
    // SPI相关
    SPISettings spiSettings;
    int cs_pin;
    
    // 原始数据
    int16_t raw_dx, raw_dy;
    uint8_t surface_quality;
    uint8_t motion_detected;
    
    // 计算的位置和速度
    float position_x;
    float position_y;
    float velocity_x;
    float velocity_y;
    
    // 方向角度
    float heading;
    float distance;
    
    // 标定参数
    float scale_factor;
    const float FOCAL_LENGTH = 0.0042;
    const float PIXEL_SIZE = 30e-6;
    
    // 时间管理
    uint32_t last_update_time;
    float dt;
    
    // 滤波器参数
    float alpha = 0.8;
    float filtered_vx = 0;
    float filtered_vy = 0;
    
    // 质量阈值
    const uint8_t MIN_QUALITY = 30;
    const uint8_t MIN_FEATURES = 10;
};

#endif // PMW3901_SENSOR_HPP