#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <chrono>
#include <fcntl.h>       // For open()
#include <unistd.h>      // For close(), read(), write()
#include <linux/i2c-dev.h> // For I2C_SLAVE
#include <sys/ioctl.h>   // For ioctl()
#include <cmath>         // For M_PI
#include <string.h>      // For memcpy
#include <thread>        // For std::this_thread::sleep_for
#include <errno.h>       // For strerror(errno)
#include <time.h>
#include <sstream>

// Include Bosch BMI270 API headers
#include "bmi270_driver/bmi2.h"
#include "bmi270_driver/bmi270.h"

// Include Eigen headers
// on my RPi eigen is located in /usr/include/eigen3/Eigen
// so need eigen3/Eigen instead of just Eigen
#include <eigen3/Eigen/Dense> // For Matrix and Vector operations

using namespace std::chrono_literals;

// Forward declarations for I2C and delay functions
int8_t bmi2_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);
int8_t bmi2_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);
void bmi2_delay_us(uint32_t period, void*);

class BMI270Node : public rclcpp::Node
{
public:
    BMI270Node() : Node("bmi270_imu_calibtd_node")
    {
        // Declare and get calibration parameters
        // Accelerometer Calibration Parameters
        this->declare_parameter<double>("accel_bias_x", 0.0);
        this->declare_parameter<double>("accel_bias_y", 0.0);
        this->declare_parameter<double>("accel_bias_z", 0.0);
        this->declare_parameter<double>("accel_scale_x", 1.0);
        this->declare_parameter<double>("accel_scale_y", 1.0);
        this->declare_parameter<double>("accel_scale_z", 1.0);
        this->declare_parameter<double>("accel_mis_yz", 0.0);
        this->declare_parameter<double>("accel_mis_zy", 0.0);
        this->declare_parameter<double>("accel_mis_xz", 0.0);
        this->declare_parameter<double>("accel_mis_zx", 0.0);
        this->declare_parameter<double>("accel_mis_xy", 0.0);
        this->declare_parameter<double>("accel_mis_yx", 0.0);

        // Gyroscope Calibration Parameters
        this->declare_parameter<double>("gyro_bias_x", 0.0);
        this->declare_parameter<double>("gyro_bias_y", 0.0);
        this->declare_parameter<double>("gyro_bias_z", 0.0);
        this->declare_parameter<double>("gyro_scale_x", 1.0);
        this->declare_parameter<double>("gyro_scale_y", 1.0);
        this->declare_parameter<double>("gyro_scale_z", 1.0);
        this->declare_parameter<double>("gyro_mis_yz", 0.0);
        this->declare_parameter<double>("gyro_mis_zy", 0.0);
        this->declare_parameter<double>("gyro_mis_xz", 0.0);
        this->declare_parameter<double>("gyro_mis_zx", 0.0);
        this->declare_parameter<double>("gyro_mis_xy", 0.0);
        this->declare_parameter<double>("gyro_mis_yx", 0.0);

        // Populate Eigen objects with parameter values
        accel_bias_ <<
            this->get_parameter("accel_bias_x").as_double(),
            this->get_parameter("accel_bias_y").as_double(),
            this->get_parameter("accel_bias_z").as_double();

        accel_misalignment_matrix_ <<
            1.0, -this->get_parameter("accel_mis_yz").as_double(), this->get_parameter("accel_mis_zy").as_double(),
            this->get_parameter("accel_mis_xz").as_double(), 1.0, -this->get_parameter("accel_mis_zx").as_double(),
            -this->get_parameter("accel_mis_xy").as_double(), this->get_parameter("accel_mis_yx").as_double(), 1.0;

        accel_scale_matrix_ <<
            this->get_parameter("accel_scale_x").as_double(), 0.0, 0.0,
            0.0, this->get_parameter("accel_scale_y").as_double(), 0.0,
            0.0, 0.0, this->get_parameter("accel_scale_z").as_double();

        // Calculate combined accelerometer correction matrix M = K * T
        accel_correction_matrix_ = accel_scale_matrix_ * accel_misalignment_matrix_;

        // debug...
        RCLCPP_INFO(this->get_logger(), "Loaded Accel Bias X: %f", this->get_parameter("accel_bias_x").as_double());
        RCLCPP_INFO(this->get_logger(), "Loaded Accel Bias Y: %f", this->get_parameter("accel_bias_y").as_double());
        RCLCPP_INFO(this->get_logger(), "Loaded Accel Bias Z: %f", this->get_parameter("accel_bias_z").as_double());

        RCLCPP_INFO(this->get_logger(), "Eigen Accel Bias X: %f", accel_bias_.x());
        RCLCPP_INFO(this->get_logger(), "Eigen Accel Bias Y: %f", accel_bias_.y());
        RCLCPP_INFO(this->get_logger(), "Eigen Accel Bias Z: %f", accel_bias_.z());

        // RCLCPP_INFO(this->get_logger(), "Accel Correction Matrix:\n%s", accel_correction_matrix_.format(Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols)).c_str());
         // Use stringstream to format the Eigen matrix (c_str doesn't work)
        std::stringstream ss;
        ss << accel_correction_matrix_.format(Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols));
        RCLCPP_INFO(this->get_logger(), "Accel Correction Matrix:\n%s", ss.str().c_str());
 
      

        gyro_bias_ <<
            this->get_parameter("gyro_bias_x").as_double(),
            this->get_parameter("gyro_bias_y").as_double(),
            this->get_parameter("gyro_bias_z").as_double();

        gyro_misalignment_matrix_ <<
            1.0, -this->get_parameter("gyro_mis_yz").as_double(), this->get_parameter("gyro_mis_zy").as_double(),
            this->get_parameter("gyro_mis_xz").as_double(), 1.0, -this->get_parameter("gyro_mis_zx").as_double(),
            -this->get_parameter("gyro_mis_xy").as_double(), this->get_parameter("gyro_mis_yx").as_double(), 1.0;

        gyro_scale_matrix_ <<
            this->get_parameter("gyro_scale_x").as_double(), 0.0, 0.0,
            0.0, this->get_parameter("gyro_scale_y").as_double(), 0.0,
            0.0, 0.0, this->get_parameter("gyro_scale_z").as_double();

        // Calculate combined gyroscope correction matrix M = K * T
        gyro_correction_matrix_ = gyro_scale_matrix_ * gyro_misalignment_matrix_;


        // Initialize ROS2 publisher and timer
        publisher_ = create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
        timer_ = create_wall_timer(2ms, std::bind(&BMI270Node::timer_callback, this));

        // Open I2C device
        i2c_fd_ = open("/dev/i2c-3", O_RDWR);
        if (i2c_fd_ < 0) {
            RCLCPP_FATAL(get_logger(), "Failed to open I2C device /dev/i2c-1. Error: %s", strerror(errno));
            rclcpp::shutdown();
            return;
        }

        // Set I2C slave address for BMI270 (0x68 or 0x69)
        if (ioctl(i2c_fd_, I2C_SLAVE, 0x68) < 0) {
            RCLCPP_FATAL(get_logger(), "Failed to set I2C address 0x%X. Please check your I2C address. Error: %s", 0x68, strerror(errno));
            close(i2c_fd_);
            rclcpp::shutdown();
            return;
        }

        // Initialize BMI2 sensor device structure
        dev_.intf = BMI2_I2C_INTF;
        dev_.intf_ptr = &i2c_fd_;
        dev_.read = bmi2_i2c_read;
        dev_.write = bmi2_i2c_write;
        dev_.delay_us = bmi2_delay_us;
        dev_.read_write_len = 32;

        // Initialize BMI270 sensor
        if (bmi270_init(&dev_) != BMI2_OK) {
            RCLCPP_FATAL(get_logger(), "BMI270 initialization failed. Check wiring and power.");
            close(i2c_fd_);
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(get_logger(), "BMI270 initialized successfully.");

        // Sensor configuration
        struct bmi2_sens_config config[2] = {0};

        // Accelerometer configuration
        config[0].type = BMI2_ACCEL;
        config[0].cfg.acc.odr = BMI2_ACC_ODR_200HZ;
        config[0].cfg.acc.range = BMI2_ACC_RANGE_4G;
        config[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
        config[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

        // Gyroscope configuration
        config[1].type = BMI2_GYRO;
        config[1].cfg.gyr.odr = BMI2_GYR_ODR_200HZ;
        config[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
        config[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
        config[1].cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;
        config[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

        // Set sensor configurations
        if (bmi2_set_sensor_config(config, 2, &dev_) != BMI2_OK) {
            RCLCPP_FATAL(get_logger(), "Sensor configuration failed.");
            close(i2c_fd_);
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(get_logger(), "Sensor configured for Accel and Gyro.");

        // Enable sensors
        uint8_t sensors[2] = { BMI2_ACCEL, BMI2_GYRO };
        if (bmi2_sensor_enable(sensors, 2, &dev_) != BMI2_OK) {
            RCLCPP_FATAL(get_logger(), "Sensor enabling failed.");
            close(i2c_fd_);
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(get_logger(), "Accel and Gyro enabled.");
    }

    ~BMI270Node()
    {
        if (i2c_fd_ >= 0) {
            close(i2c_fd_);
            RCLCPP_INFO(get_logger(), "I2C device closed.");
        }
    }

private:
    void timer_callback()
    {
        struct bmi2_sens_data sensor_data;

        if ((bmi2_get_sensor_data(&sensor_data, &dev_) == BMI2_OK) && (sensor_data.status & BMI2_DRDY_ACC) && (sensor_data.status & BMI2_DRDY_GYR)) {
            auto msg = sensor_msgs::msg::Imu();
            struct timespec tp;
            clock_gettime(CLOCK_MONOTONIC, &tp);
            msg.header.stamp = rclcpp::Time(tp.tv_sec * 1000000000 + tp.tv_nsec, RCL_STEADY_TIME);
            msg.header.frame_id = "imu_link";

            // Raw accelerometer data as Eigen Vector
            Eigen::Vector3d raw_accel_data(
                static_cast<double>(sensor_data.acc.x),
                static_cast<double>(sensor_data.acc.y),
                static_cast<double>(sensor_data.acc.z)
            );

            // Apply accelerometer calibration: C = M * (R - B)
            Eigen::Vector3d calibrated_accel_data = accel_correction_matrix_ * (raw_accel_data - accel_bias_);

            // Convert calibrated accelerometer data to m/s^2
            float acc_lsb_scale = (4.0f * 9.80665f) / 32768.0f; // Scale factor for +/-4G range (m/s^2 per LSB)
            msg.linear_acceleration.x = calibrated_accel_data.x() * acc_lsb_scale;
            msg.linear_acceleration.y = calibrated_accel_data.y() * acc_lsb_scale;
            msg.linear_acceleration.z = calibrated_accel_data.z() * acc_lsb_scale;

            // Raw gyroscope data as Eigen Vector
            Eigen::Vector3d raw_gyro_data(
                static_cast<double>(sensor_data.gyr.x),
                static_cast<double>(sensor_data.gyr.y),
                static_cast<double>(sensor_data.gyr.z)
            );

            // Apply gyroscope calibration: C = M * (R - B)
            Eigen::Vector3d calibrated_gyro_data = gyro_correction_matrix_ * (raw_gyro_data - gyro_bias_);

            // Convert calibrated gyroscope data to rad/s
            float gyro_lsb_scale = (2000.0f / 32768.0f) * (M_PI / 180.0f); // Scale factor for +/-2000 dps range (rad/s per LSB)
            msg.angular_velocity.x = calibrated_gyro_data.x() * gyro_lsb_scale;
            msg.angular_velocity.y = calibrated_gyro_data.y() * gyro_lsb_scale;
            msg.angular_velocity.z = calibrated_gyro_data.z() * gyro_lsb_scale;

            // Indicate no covariance data available
            msg.angular_velocity_covariance[0] = -1;
            msg.linear_acceleration_covariance[0] = -1;

            publisher_->publish(msg);
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int i2c_fd_;
    struct bmi2_dev dev_;

    // Eigen objects for calibration
    Eigen::Vector3d accel_bias_;
    Eigen::Matrix3d accel_misalignment_matrix_;
    Eigen::Matrix3d accel_scale_matrix_;
    Eigen::Matrix3d accel_correction_matrix_; // K * T

    Eigen::Vector3d gyro_bias_;
    Eigen::Matrix3d gyro_misalignment_matrix_;
    Eigen::Matrix3d gyro_scale_matrix_;
    Eigen::Matrix3d gyro_correction_matrix_; // K * T
};

// ... (bmi2_i2c_read, bmi2_i2c_write, bmi2_delay_us, and main functions remain the same) ...

/**
 * @brief Custom I2C read function for BMI270 API.
 * This function reads a specified number of bytes from the sensor via I2C.
 *
 * @param reg_addr   : Register address to start reading from.
 * @param data       : Pointer to buffer to store read data.
 * @param len        : Number of bytes to read.
 * @param intf_ptr   : Pointer to I2C file descriptor.
 * @return BMI2_OK on success, BMI2_E_COM_FAIL on failure.
 */
int8_t bmi2_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr)
{
    // Cast the void pointer back to an int pointer to get the file descriptor
    int fd = *(int *)intf_ptr;

    // Write the register address to the I2C device
    if (write(fd, &reg_addr, 1) != 1) {
        // If the write operation fails, return communication failure error
        return BMI2_E_COM_FAIL;
    }

    // Read 'len' bytes from the I2C device into the data buffer
    if (read(fd, data, len) != (int)len) {
        // If the read operation fails or doesn't read the expected number of bytes, return communication failure error
        return BMI2_E_COM_FAIL;
    }

    // Return success
    return BMI2_OK;
}

/**
 * @brief Custom I2C write function for BMI270 API.
 * This function writes a specified number of bytes to the sensor via I2C.
 *
 * @param reg_addr   : Register address to start writing to.
 * @param data       : Pointer to data buffer to write.
 * @param len        : Number of bytes to write.
 * @param intf_ptr   : Pointer to I2C file descriptor.
 * @return BMI2_OK on success, BMI2_E_COM_FAIL on failure.
 */
int8_t bmi2_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr)
{
    // Cast the void pointer back to an int pointer to get the file descriptor
    int fd = *(int *)intf_ptr;

    // Create a buffer to hold the register address followed by the data
    uint8_t buf[len + 1];

    // Place the register address at the beginning of the buffer
    buf[0] = reg_addr;
    // Copy the data to be written into the buffer, starting after the register address
    memcpy(&buf[1], data, len);

    // Write the entire buffer (register address + data) to the I2C device
    if (write(fd, buf, len + 1) != (int)(len + 1)) {
        // If the write operation fails or doesn't write the expected number of bytes, return communication failure error
        return BMI2_E_COM_FAIL;
    }

    // Return success
    return BMI2_OK;
}

/**
 * @brief Custom delay function for BMI270 API.
 * This function provides a microsecond delay.
 *
 * @param period   : Delay period in microseconds.
 * @param unused_ptr : Void pointer (unused but required by API signature).
 */
void bmi2_delay_us(uint32_t period, void* unused_ptr)
{
    // Use C++ standard library for sleeping for a specified duration
    std::this_thread::sleep_for(std::chrono::microseconds(period));
}

int main(int argc, char *argv[])
{
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create and spin the BMI270Node
    rclcpp::spin(std::make_shared<BMI270Node>());

    // Shutdown ROS2
    rclcpp::shutdown();
    return 0;
}
