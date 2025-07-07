// this spits sensortime ticks but not IMU data...
//
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <chrono>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <cmath>
#include <string.h>
#include <thread>
#include <errno.h>
#include <algorithm>
#include <limits>
#include <iomanip>
#include <sstream>

#include "bmi270_driver/bmi2.h"
#include "bmi270_driver/bmi270.h"

using namespace std::chrono_literals;

int8_t bmi2_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);
int8_t bmi2_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);
void bmi2_delay_us(uint32_t period, void*);

class BMI270FIFONode : public rclcpp::Node {
public:
    BMI270FIFONode() : Node("bmi270_fifo_node"), first_sync_(true) {
        publisher_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
        timer_ = create_wall_timer(30ms, std::bind(&BMI270FIFONode::timer_callback, this));

        i2c_fd_ = open("/dev/i2c-3", O_RDWR);
        if (i2c_fd_ < 0 || ioctl(i2c_fd_, I2C_SLAVE, 0x68) < 0) {
            RCLCPP_FATAL(get_logger(), "I2C setup failed: %s", strerror(errno));
            rclcpp::shutdown();
            return;
        }

        dev_.intf = BMI2_I2C_INTF;
        dev_.intf_ptr = &i2c_fd_;
        dev_.read = bmi2_i2c_read;
        dev_.write = bmi2_i2c_write;
        dev_.delay_us = bmi2_delay_us;
        dev_.read_write_len = 32;

        if (bmi270_init(&dev_) != BMI2_OK) {
            RCLCPP_FATAL(get_logger(), "BMI270 init failed");
            rclcpp::shutdown();
            return;
        }

        struct bmi2_sens_config config[2] = {0};
        config[0].type = BMI2_ACCEL;
        config[0].cfg.acc.odr = BMI2_ACC_ODR_200HZ;
        config[0].cfg.acc.range = BMI2_ACC_RANGE_4G;
        config[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
        config[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

        config[1].type = BMI2_GYRO;
        config[1].cfg.gyr.odr = BMI2_GYR_ODR_200HZ;
        config[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
        config[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
        config[1].cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;
        config[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

        if (bmi2_set_sensor_config(config, 2, &dev_) != BMI2_OK) {
            RCLCPP_FATAL(get_logger(), "Sensor config failed");
            rclcpp::shutdown();
            return;
        }

        uint8_t sensors[] = {BMI2_ACCEL, BMI2_GYRO};
        if (bmi2_sensor_enable(sensors, 2, &dev_) != BMI2_OK) {
            RCLCPP_FATAL(get_logger(), "Sensor enabling failed");
            rclcpp::shutdown();
            return;
        }

        uint16_t fifo_config = BMI2_FIFO_HEADER_EN | BMI2_FIFO_TIME_EN | BMI2_FIFO_ACC_EN | BMI2_FIFO_GYR_EN;
        if (bmi2_set_fifo_config(fifo_config, BMI2_ENABLE, &dev_) != BMI2_OK) {
            RCLCPP_FATAL(get_logger(), "Failed to configure FIFO");
            rclcpp::shutdown();
            return;
        }

        uint16_t fifo_cfg_check = 0;
        if (bmi2_get_fifo_config(&fifo_cfg_check, &dev_) == BMI2_OK) {
            RCLCPP_INFO(get_logger(), "FIFO config: 0x%X", fifo_cfg_check);
        }
    }

    ~BMI270FIFONode() {
        if (i2c_fd_ >= 0) close(i2c_fd_);
    }

private:
    void timer_callback() {
        constexpr uint16_t FIFO_SIZE = 2048;
        static uint8_t fifo_buffer[FIFO_SIZE] = {0};

        struct bmi2_fifo_frame fifo = {};
        fifo.data = fifo_buffer;
        fifo.length = FIFO_SIZE;

        if (bmi2_get_fifo_length(&fifo.length, &dev_) != BMI2_OK || fifo.length == 0) {
            RCLCPP_WARN(get_logger(), "FIFO empty or error reading length");
            return;
        }

        if (bmi2_read_fifo_data(&fifo, &dev_) != BMI2_OK) {
            RCLCPP_WARN(get_logger(), "Failed to read FIFO data");
            return;
        }

        // Parse sensortime manually from raw buffer
        for (uint16_t i = 0; i + 3 < fifo.length; ++i) {
            if (fifo_buffer[i] == 0x44) {
                uint32_t ticks = ((uint32_t)fifo_buffer[i+1]) |
                                 ((uint32_t)fifo_buffer[i+2] << 8) |
                                 ((uint32_t)fifo_buffer[i+3] << 16);
                RCLCPP_INFO(get_logger(), "Parsed SensorTime frame: %u ticks", ticks);
                i += 3;
            }
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int i2c_fd_;
    struct bmi2_dev dev_;
    bool first_sync_;
    rclcpp::Time t0_ros_;
    uint32_t t0_imu_ticks_;
};

int8_t bmi2_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr) {
    int fd = *(int *)intf_ptr;
    if (write(fd, &reg_addr, 1) != 1) return BMI2_E_COM_FAIL;
    if (read(fd, data, len) != (int)len) return BMI2_E_COM_FAIL;
    return BMI2_OK;
}

int8_t bmi2_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr) {
    int fd = *(int *)intf_ptr;
    uint8_t buf[len + 1];
    buf[0] = reg_addr;
    memcpy(&buf[1], data, len);
    if (write(fd, buf, len + 1) != (int)(len + 1)) return BMI2_E_COM_FAIL;
    return BMI2_OK;
}

void bmi2_delay_us(uint32_t period, void*) {
    std::this_thread::sleep_for(std::chrono::microseconds(period));
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BMI270FIFONode>());
    rclcpp::shutdown();
    return 0;
}

