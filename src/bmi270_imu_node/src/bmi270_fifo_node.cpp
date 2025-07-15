
// Set and use FIFO mode, purpose: get accurate timestamps
// It seems the only way to get timestamps from the IMU itself
// is when using FIFO (see all the examples in the Bosch API)
// This works to publish imu message but i can't get sensortime to work
// (frame 0x44 never appear)
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
#include <array>

#include "bmi270_driver/bmi2.h"
#include "bmi270_driver/bmi270.h"

using namespace std::chrono_literals;

// already defined in the bosch API
// #define BMI2_SENSORTIME_RESOLUTION (1.0 / 25600.0)
#define ACCEL_RESOLUTION (9.80665 / 8192.0)
#define GYRO_RESOLUTION ((1.0 / 16.4) * (M_PI / 180.0))

int8_t bmi2_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);
int8_t bmi2_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);
void bmi2_delay_us(uint32_t period, void*);

class BMI270FIFONode : public rclcpp::Node {
public:
    BMI270FIFONode() : Node("bmi270_fifo_node"), first_sync_(true), t0_imu_ticks_(0) {
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

        // Disable Advanced Power Save Mode
if (bmi2_set_adv_power_save(BMI2_DISABLE, &dev_) != BMI2_OK) {
    RCLCPP_FATAL(get_logger(), "Failed to disable advanced power save mode");
    rclcpp::shutdown();
    return;
}

// Clear any FIFO configuration before setting new one
if (bmi2_set_fifo_config(BMI2_FIFO_ALL_EN, BMI2_DISABLE, &dev_) != BMI2_OK) {
    RCLCPP_FATAL(get_logger(), "Failed to reset FIFO config");
    rclcpp::shutdown();
    return;
}

// Now enable FIFO with header, accel, gyro, and time
uint16_t fifo_config = BMI2_FIFO_HEADER_EN | BMI2_FIFO_TIME_EN | BMI2_FIFO_ACC_EN | BMI2_FIFO_GYR_EN;
if (bmi2_set_fifo_config(fifo_config, BMI2_ENABLE, &dev_) != BMI2_OK) {
    RCLCPP_FATAL(get_logger(), "Failed to configure FIFO");
    rclcpp::shutdown();
    return;
}


        // uint16_t fifo_config = BMI2_FIFO_HEADER_EN | BMI2_FIFO_TIME_EN | BMI2_FIFO_ACC_EN | BMI2_FIFO_GYR_EN;
        // if (bmi2_set_fifo_config(fifo_config, BMI2_ENABLE, &dev_) != BMI2_OK) {
        //     RCLCPP_FATAL(get_logger(), "Failed to configure FIFO");
        //     rclcpp::shutdown();
        //     return;
        // }

        if (bmi2_set_fifo_wm(512, &dev_) != BMI2_OK) {
            RCLCPP_WARN(get_logger(), "Failed to set FIFO watermark");
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
        fifo.header_enable = BMI2_ENABLE;

        uint16_t fifo_len = 0;
        if (bmi2_get_fifo_length(&fifo_len, &dev_) != BMI2_OK || fifo_len == 0) return;
        fifo.length = std::min<uint16_t>(fifo_len, FIFO_SIZE);

        if (bmi2_read_fifo_data(&fifo, &dev_) != BMI2_OK) return;
        RCLCPP_DEBUG(get_logger(), "FIFO read length: %u", fifo_len);

        std::ostringstream debug_hex;
        debug_hex << "FIFO[0:" << fifo.length << "] = ";
        for (size_t k = 0; k < fifo.length; ++k) {
            debug_hex << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(fifo.data[k]) << " ";
        }
        RCLCPP_DEBUG(get_logger(), "%s", debug_hex.str().c_str());

        uint16_t i = 0;
        uint32_t last_sensor_time = 0;
        imu_accel_buf_.clear();
        imu_gyro_buf_.clear();

        while (i + 6 < fifo.length) {
            uint8_t header = fifo.data[i++];
            if (header == 0x84 && i + 6 <= fifo.length) {
                std::array<int16_t, 3> acc = {
                    static_cast<int16_t>(fifo.data[i] | (fifo.data[i+1] << 8)),
                    static_cast<int16_t>(fifo.data[i+2] | (fifo.data[i+3] << 8)),
                    static_cast<int16_t>(fifo.data[i+4] | (fifo.data[i+5] << 8))
                };
                i += 6;
                imu_accel_buf_.push_back(acc);
            } else if (header == 0x88 && i + 6 <= fifo.length) {
                std::array<int16_t, 3> gyr = {
                    static_cast<int16_t>(fifo.data[i] | (fifo.data[i+1] << 8)),
                    static_cast<int16_t>(fifo.data[i+2] | (fifo.data[i+3] << 8)),
                    static_cast<int16_t>(fifo.data[i+4] | (fifo.data[i+5] << 8))
                };
                i += 6;
                imu_gyro_buf_.push_back(gyr);
            } else if (header == 0x8C && i + 12 <= fifo.length) {
                std::array<int16_t, 3> acc = {
                    static_cast<int16_t>(fifo.data[i] | (fifo.data[i+1] << 8)),
                    static_cast<int16_t>(fifo.data[i+2] | (fifo.data[i+3] << 8)),
                    static_cast<int16_t>(fifo.data[i+4] | (fifo.data[i+5] << 8))
                };
                std::array<int16_t, 3> gyr = {
                    static_cast<int16_t>(fifo.data[i+6] | (fifo.data[i+7] << 8)),
                    static_cast<int16_t>(fifo.data[i+8] | (fifo.data[i+9] << 8)),
                    static_cast<int16_t>(fifo.data[i+10] | (fifo.data[i+11] << 8))
                };
                i += 12;
                imu_accel_buf_.push_back(acc);
                imu_gyro_buf_.push_back(gyr);
            } else if (header == 0x44 && i + 3 <= fifo.length) {
                last_sensor_time = fifo.data[i] | (fifo.data[i+1] << 8) | (fifo.data[i+2] << 16);
                i += 3;
                RCLCPP_DEBUG(get_logger(), "SensorTime frame found: %u", last_sensor_time);
            } else {
                RCLCPP_DEBUG(get_logger(), "Unknown header: 0x%02X at index %u", header, i - 1);
                break;
            }
        }

        if (last_sensor_time > 0) {
            if (first_sync_) {
                t0_ros_ = this->now();
                t0_imu_ticks_ = last_sensor_time;
                RCLCPP_INFO(get_logger(), "Initial sensor time sync: %u ticks", last_sensor_time);
                first_sync_ = false;
            }
            last_imu_ticks_ = last_sensor_time;
        }

        size_t frame_count = std::min(imu_accel_buf_.size(), imu_gyro_buf_.size());
        RCLCPP_DEBUG(get_logger(), "Parsed %zu accel+gyro frames", frame_count);

        for (size_t i = 0; i < frame_count; ++i) {
            const auto& acc = imu_accel_buf_[i];
            const auto& gyr = imu_gyro_buf_[i];

            sensor_msgs::msg::Imu imu;
            imu.header.frame_id = "imu_link";
            if (!first_sync_) {
                double dt = (last_imu_ticks_ - t0_imu_ticks_) * BMI2_SENSORTIME_RESOLUTION;
                imu.header.stamp = t0_ros_ + rclcpp::Duration::from_seconds(dt);
            } else {
                imu.header.stamp = this->now();
            }

            imu.linear_acceleration.x = acc[0] * ACCEL_RESOLUTION;
            imu.linear_acceleration.y = acc[1] * ACCEL_RESOLUTION;
            imu.linear_acceleration.z = acc[2] * ACCEL_RESOLUTION;

            imu.angular_velocity.x = gyr[0] * GYRO_RESOLUTION;
            imu.angular_velocity.y = gyr[1] * GYRO_RESOLUTION;
            imu.angular_velocity.z = gyr[2] * GYRO_RESOLUTION;

            imu.linear_acceleration_covariance[0] = -1.0;
            imu.linear_acceleration_covariance[4] = -1.0;
            imu.linear_acceleration_covariance[8] = -1.0;
            imu.angular_velocity_covariance[0] = -1.0;
            imu.angular_velocity_covariance[4] = -1.0;
            imu.angular_velocity_covariance[8] = -1.0;

            publisher_->publish(imu);
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int i2c_fd_;
    struct bmi2_dev dev_;
    bool first_sync_;
    rclcpp::Time t0_ros_;
    uint32_t t0_imu_ticks_;
    uint32_t last_imu_ticks_ = 0;
    std::vector<std::array<int16_t, 3>> imu_accel_buf_;
    std::vector<std::array<int16_t, 3>> imu_gyro_buf_;
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

