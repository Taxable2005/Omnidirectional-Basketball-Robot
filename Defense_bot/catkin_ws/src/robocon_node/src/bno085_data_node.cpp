#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <string>
#include <stdexcept>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstring>
#include <cmath>

#define CHANNEL_COMMAND 0
#define CHANNEL_CONTROL 1
#define CHANNEL_REPORTS 2

#define REPORTID_ROTATION_VECTOR 0x05

class BNO08x {
public:
    BNO08x(const std::string& i2c_dev, uint8_t address)
        : address_(address) {
        if ((fd_ = open(i2c_dev.c_str(), O_RDWR)) < 0) {
            throw std::runtime_error("Failed to open I2C device");
        }
        if (ioctl(fd_, I2C_SLAVE, address_) < 0) {
            throw std::runtime_error("Failed to set I2C address");
        }
    }

    ~BNO08x() {
        if (fd_ >= 0) close(fd_);
    }

    bool begin() {
        usleep(10000); // Wait after boot

        // Enable Rotation Vector report at 100 Hz (interval in microseconds)
        uint8_t cmd[] = {
            0xFD, // Set Feature Command
            REPORTID_ROTATION_VECTOR, // Report ID
            0, 0, 0, 0, // Feature flags
            0xA0, 0x86, 0x01, 0x00, // Report interval: 10000 us (100 Hz)
            0, 0 // Batch interval, sensor-specific config
        };
        return sendPacket(CHANNEL_CONTROL, cmd, sizeof(cmd));
    }

    bool readImu(sensor_msgs::Imu& msg) {
        uint8_t header[4];
        if (read(fd_, header, 4) != 4) return false;

        uint16_t data_length = header[0] | (header[1] << 8);
        uint8_t channel = header[2] & 0x0F;
        if (channel != CHANNEL_REPORTS) {
            // Drain remaining data
            uint8_t dummy[data_length];
            read(fd_, dummy, data_length);
            return false;
        }

        uint8_t data[data_length];
        if (read(fd_, data, data_length) != data_length) return false;

        if (data[0] != REPORTID_ROTATION_VECTOR) return false;

        // Parse quaternion (Q14)
        int16_t q_i = data[4] | (data[5] << 8);
        int16_t q_j = data[6] | (data[7] << 8);
        int16_t q_k = data[8] | (data[9] << 8);
        int16_t q_real = data[10] | (data[11] << 8);

        const float scale = 1.0f / (1 << 14);

        msg.header.stamp = ros::Time::now();
        msg.orientation.x = q_i * scale;
        msg.orientation.y = q_j * scale;
        msg.orientation.z = q_k * scale;
        msg.orientation.w = q_real * scale;

        msg.orientation_covariance[0] = -1; // unknown
        msg.angular_velocity.x = 0;
        msg.angular_velocity.y = 0;
        msg.angular_velocity.z = 0;
        msg.angular_velocity_covariance[0] = -1;
        msg.linear_acceleration.x = 0;
        msg.linear_acceleration.y = 0;
        msg.linear_acceleration.z = 0;
        msg.linear_acceleration_covariance[0] = -1;

        ROS_INFO_THROTTLE(1, "Published Quaternion: [%.3f, %.3f, %.3f, %.3f]",
            msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
        return true;
    }

private:
    int fd_{-1};
    uint8_t address_;

    bool sendPacket(uint8_t channel, uint8_t* data, size_t length) {
        uint8_t buffer[4 + length];
        buffer[0] = length & 0xFF;
        buffer[1] = (length >> 8) & 0xFF;
        buffer[2] = channel & 0x0F;
        buffer[3] = 0; // Sequence number, ignored

        memcpy(buffer + 4, data, length);

        ssize_t written = write(fd_, buffer, 4 + length);
        return written == (ssize_t)(4 + length);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "bno08x_node");
    ros::NodeHandle nh("~");

    std::string i2c_dev;
    int i2c_addr;
    nh.param<std::string>("i2c_device", i2c_dev, "/dev/i2c-1");
    nh.param<int>("i2c_address", i2c_addr, 0x4A);

    BNO08x sensor(i2c_dev, static_cast<uint8_t>(i2c_addr));
    if (!sensor.begin()) {
        ROS_FATAL("Failed to initialize BNO08x sensor");
        return 1;
    }

    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 10);
    ros::Rate rate(100);

    while (ros::ok()) {
        sensor_msgs::Imu imu_msg;
        if (sensor.readImu(imu_msg)) {
            imu_pub.publish(imu_msg);
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
