#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <array>
#include <cmath>
#include <string>
#include <iostream>

class ImuNoiseEstimator : public rclcpp::Node
{
public:
    ImuNoiseEstimator()
    : Node("imu_noise_estimator"),
      sample_count_(0)
    {
        // Parameters
        this->declare_parameter<std::string>("imu_topic", "/livox/imu");
        this->declare_parameter<int>("min_samples", 5000);     // minimum samples before reporting
        this->declare_parameter<bool>("auto_shutdown", false); // shut down after report?

        imu_topic_     = this->get_parameter("imu_topic").as_string();
        min_samples_   = this->get_parameter("min_samples").as_int();
        auto_shutdown_ = this->get_parameter("auto_shutdown").as_bool();

        RCLCPP_INFO(get_logger(), "IMU Noise Estimator");
        RCLCPP_INFO(get_logger(), "  IMU topic: %s", imu_topic_.c_str());
        RCLCPP_INFO(get_logger(), "  Min samples: %d", min_samples_);
        RCLCPP_INFO(get_logger(), "  Keep sensor completely still while collecting!");

        sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, rclcpp::SensorDataQoS(),
            std::bind(&ImuNoiseEstimator::imuCallback, this, std::placeholders::_1));
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Extract gyro and accel
        double gx = msg->angular_velocity.x;
        double gy = msg->angular_velocity.y;
        double gz = msg->angular_velocity.z;

        double ax = msg->linear_acceleration.x;
        double ay = msg->linear_acceleration.y;
        double az = msg->linear_acceleration.z;

        // Welford online algorithm for mean & variance
        updateStats(0, gx);
        updateStats(1, gy);
        updateStats(2, gz);
        updateStats(3, ax);
        updateStats(4, ay);
        updateStats(5, az);

        sample_count_++;

        if (sample_count_ == min_samples_) {
            printReport();
            if (auto_shutdown_) {
                rclcpp::shutdown();
            }
        } else if (sample_count_ % 1000 == 0) {
            RCLCPP_INFO(get_logger(), "Collected %zu samples so far...", sample_count_);
        }
    }

    void updateStats(size_t idx, double x)
    {
        // idx: 0..5  (0–2 gyro, 3–5 accel)
        double& mean = mean_[idx];
        double& m2   = m2_[idx];

        double delta  = x - mean;
        mean         += delta / static_cast<double>(sample_count_ + 1);
        double delta2 = x - mean;
        m2           += delta * delta2;
    }

    void printReport()
    {
        RCLCPP_INFO(get_logger(), "=========================================");
        RCLCPP_INFO(get_logger(), "      IMU NOISE ESTIMATION REPORT        ");
        RCLCPP_INFO(get_logger(), "=========================================");
        RCLCPP_INFO(get_logger(), "Samples: %zu", sample_count_);
        RCLCPP_INFO(get_logger(), "Assuming sensor was completely static.");

        if (sample_count_ < 2) {
            RCLCPP_WARN(get_logger(), "Not enough samples to compute statistics.");
            return;
        }

        std::array<double, 6> stddev{};
        for (size_t i = 0; i < 6; ++i) {
            double variance = m2_[i] / static_cast<double>(sample_count_ - 1);
            stddev[i] = std::sqrt(variance);
        }

        // Print per-axis
        std::cout << std::fixed;
        std::cout << "\nGyro STD (rad/s):\n";
        std::cout << "  gx: " << stddev[0]
                  << "  gy: " << stddev[1]
                  << "  gz: " << stddev[2] << "\n";

        std::cout << "\nAccel STD (m/s^2):\n";
        std::cout << "  ax: " << stddev[3]
                  << "  ay: " << stddev[4]
                  << "  az: " << stddev[5] << "\n";

        // Average over axes
        double gyro_std_mean  = (stddev[0] + stddev[1] + stddev[2]) / 3.0;
        double accel_std_mean = (stddev[3] + stddev[4] + stddev[5]) / 3.0;

        std::cout << "\nAveraged noise (for LIO-SAM parameters):\n";
        std::cout << "  imuGyrNoise  ≈ " << gyro_std_mean  << "  (rad/s)\n";
        std::cout << "  imuAccNoise  ≈ " << accel_std_mean << "  (m/s^2)\n";

        // Simple heuristic for bias random walk
        double imuGyrBiasN = gyro_std_mean  / 10.0;
        double imuAccBiasN = accel_std_mean / 10.0;

        std::cout << "\nRecommended params.yaml entries (initial guess):\n";
        std::cout << "  imuGyrNoise:  " << gyro_std_mean  << "\n";
        std::cout << "  imuAccNoise:  " << accel_std_mean << "\n";
        std::cout << "  imuGyrBiasN:  " << imuGyrBiasN    << "\n";
        std::cout << "  imuAccBiasN:  " << imuAccBiasN    << "\n\n";

        std::cout << "Paste these into your LIO-SAM params.yaml under IMU settings.\n";
        std::cout << "If you repeat the test and values are similar, you’re good.\n";
        std::cout << "=========================================\n";
    }

    // Parameters
    std::string imu_topic_;
    int         min_samples_;
    bool        auto_shutdown_;

    // Online stats
    size_t sample_count_;
    std::array<double, 6> mean_ {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<double, 6> m2_   {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuNoiseEstimator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
