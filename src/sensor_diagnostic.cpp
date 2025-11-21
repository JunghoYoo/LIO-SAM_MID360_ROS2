/**
 * LIO-SAM + Livox MID-360 Diagnostic Tool (C++)
 * Checks common configuration issues that cause drift
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <map>
#include <deque>
#include <vector>
#include <numeric>
#include <cmath>
#include <iomanip>

class LioSamDiagnostic : public rclcpp::Node
{
public:
    LioSamDiagnostic() : Node("lio_sam_diagnostic")
    {
        // Subscribers
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar", 10,
            std::bind(&LioSamDiagnostic::lidarCallback, this, std::placeholders::_1));
        
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/livox/imu", 10,
            std::bind(&LioSamDiagnostic::imuCallback, this, std::placeholders::_1));
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/lio_sam/mapping/odometry", 10,
            std::bind(&LioSamDiagnostic::odomCallback, this, std::placeholders::_1));
        
        // Timer for periodic reporting (5 seconds)
        timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&LioSamDiagnostic::printDiagnostics, this));
        
        RCLCPP_INFO(this->get_logger(), "LIO-SAM Diagnostic Tool Started");
        RCLCPP_INFO(this->get_logger(), "Collecting data for 5 seconds...");
    }

private:
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        lidar_timestamps_.push_back(timestamp);
        
        // Keep only last 100 timestamps
        if (lidar_timestamps_.size() > 100) {
            lidar_timestamps_.pop_front();
        }
        
        // Find ring/line field
        int ring_offset = -1;
        for (const auto& field : msg->fields) {
            if (field.name == "ring" || field.name == "line") {
                ring_offset = field.offset;
                break;
            }
        }
        
        if (ring_offset >= 0) {
            // Parse point cloud to count rings
            const uint8_t* data_ptr = msg->data.data();
            size_t point_step = msg->point_step;
            size_t num_points = msg->width * msg->height;
            
            for (size_t i = 0; i < num_points; ++i) {
                if (ring_offset < static_cast<int>(point_step)) {
                    uint8_t ring = data_ptr[i * point_step + ring_offset];
                    ring_counts_[ring]++;
                }
            }
        }
    }
    
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        imu_timestamps_.push_back(timestamp);
        
        // Keep only last 1000 timestamps
        if (imu_timestamps_.size() > 1000) {
            imu_timestamps_.pop_front();
        }
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::vector<double> pos = {
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z
        };
        odom_positions_.push_back(pos);
        
        // Keep only last 1000 positions
        if (odom_positions_.size() > 1000) {
            odom_positions_.pop_front();
        }
    }
    
    void printDiagnostics()
    {
        std::cout << "\n" << std::string(70, '=') << "\n";
        std::cout << "LIO-SAM + LIVOX MID-360 DIAGNOSTIC REPORT\n";
        std::cout << std::string(70, '=') << "\n";
        
        // 1. Check LiDAR Ring Distribution
        printLidarRingDistribution();
        
        // 2. Check IMU Frequency
        printIMUFrequency();
        
        // 3. Check LiDAR Frequency
        printLidarFrequency();
        
        // 4. Check Odometry
        printOdometryAnalysis();
        
        // 5. Configuration Recommendations
        printRecommendations();
        
        std::cout << "\n" << std::string(70, '=') << "\n";
        std::cout << "Report updates every 5 seconds. Press Ctrl+C to exit.\n";
        std::cout << std::string(70, '=') << "\n\n";
    }
    
    void printLidarRingDistribution()
    {
        std::cout << "\n1. LIDAR SCAN LINE DISTRIBUTION:\n";
        
        if (ring_counts_.empty()) {
            std::cout << "   ❌ NO LIDAR DATA RECEIVED\n";
            std::cout << "      Check if /livox/lidar topic is publishing\n";
            return;
        }
        
        // Calculate total points
        size_t total_points = 0;
        for (const auto& pair : ring_counts_) {
            total_points += pair.second;
        }
        
        std::cout << "   Total points analyzed: " << total_points << "\n";
        
        // Print ring distribution
        for (const auto& pair : ring_counts_) {
            double percentage = (pair.second * 100.0) / total_points;
            std::cout << "   Ring " << static_cast<int>(pair.first) 
                      << ": " << pair.second << " points (" 
                      << std::fixed << std::setprecision(1) << percentage << "%)\n";
        }
        
        // Check for issues
        uint8_t max_ring = ring_counts_.rbegin()->first;
        
        if (max_ring > 3) {
            std::cout << "   ⚠️  WARNING: Found ring " << static_cast<int>(max_ring) 
                      << ", but MID-360 should only have rings 0-3\n";
        } else if (max_ring < 3) {
            std::cout << "   ❌ CRITICAL: Only found rings 0-" << static_cast<int>(max_ring) << "!\n";
            std::cout << "      MID-360 has 4 scan lines (0-3). Check your N_SCAN parameter!\n";
        } else {
            std::cout << "   ✅ GOOD: Found all 4 scan lines (rings 0-3)\n";
        }
    }
    
    void printIMUFrequency()
    {
        std::cout << "\n2. IMU FREQUENCY:\n";
        
        if (imu_timestamps_.size() < 2) {
            std::cout << "   ❌ NO IMU DATA RECEIVED\n";
            return;
        }
        
        // Calculate average frequency
        std::vector<double> dts;
        for (size_t i = 1; i < imu_timestamps_.size(); ++i) {
            dts.push_back(imu_timestamps_[i] - imu_timestamps_[i-1]);
        }
        
        double avg_dt = std::accumulate(dts.begin(), dts.end(), 0.0) / dts.size();
        double freq = 1.0 / avg_dt;
        
        std::cout << "   Average frequency: " << std::fixed << std::setprecision(1) 
                  << freq << " Hz\n";
        
        if (freq < 150) {
            std::cout << "   ⚠️  WARNING: IMU rate is low. MID-360 should be ~200Hz\n";
        } else if (freq > 250) {
            std::cout << "   ⚠️  WARNING: IMU rate is unusually high\n";
        } else {
            std::cout << "   ✅ GOOD: IMU rate is within expected range\n";
        }
    }
    
    void printLidarFrequency()
    {
        std::cout << "\n3. LIDAR FREQUENCY:\n";
        
        if (lidar_timestamps_.size() < 2) {
            std::cout << "   ❌ NO LIDAR DATA RECEIVED\n";
            return;
        }
        
        // Calculate average frequency
        std::vector<double> dts;
        for (size_t i = 1; i < lidar_timestamps_.size(); ++i) {
            dts.push_back(lidar_timestamps_[i] - lidar_timestamps_[i-1]);
        }
        
        double avg_dt = std::accumulate(dts.begin(), dts.end(), 0.0) / dts.size();
        double freq = 1.0 / avg_dt;
        
        std::cout << "   Average frequency: " << std::fixed << std::setprecision(1) 
                  << freq << " Hz\n";
        
        if (freq < 8) {
            std::cout << "   ⚠️  WARNING: LiDAR rate is low. Expected ~10Hz\n";
        } else if (freq > 12) {
            std::cout << "   ⚠️  WARNING: LiDAR rate is unusually high\n";
        } else {
            std::cout << "   ✅ GOOD: LiDAR rate is within expected range\n";
        }
    }
    
    void printOdometryAnalysis()
    {
        std::cout << "\n4. ODOMETRY ANALYSIS:\n";
        
        if (odom_positions_.size() < 10) {
            std::cout << "   ⚠️  NO ODOMETRY DATA RECEIVED\n";
            return;
        }
        
        // Calculate drift
        auto start_pos = odom_positions_.front();
        auto end_pos = odom_positions_.back();
        
        double drift = std::sqrt(
            std::pow(end_pos[0] - start_pos[0], 2) +
            std::pow(end_pos[1] - start_pos[1], 2) +
            std::pow(end_pos[2] - start_pos[2], 2)
        );
        
        // Calculate variance to detect motion
        std::vector<double> x_vals, y_vals, z_vals;
        for (const auto& pos : odom_positions_) {
            x_vals.push_back(pos[0]);
            y_vals.push_back(pos[1]);
            z_vals.push_back(pos[2]);
        }
        
        double x_var = calculateVariance(x_vals);
        double y_var = calculateVariance(y_vals);
        double z_var = calculateVariance(z_vals);
        
        bool is_stationary = (x_var < 0.1 && y_var < 0.1 && z_var < 0.1);
        
        if (is_stationary) {
            std::cout << "   Robot appears stationary\n";
            std::cout << "   Position drift: " << std::fixed << std::setprecision(3) 
                      << drift << " m over " << odom_positions_.size() << " samples\n";
            
            if (drift > 0.5) {
                std::cout << "   ❌ CRITICAL: Significant drift while stationary!\n";
                std::cout << "      This indicates a serious configuration problem\n";
            } else if (drift > 0.1) {
                std::cout << "   ⚠️  WARNING: Noticeable drift while stationary\n";
            } else {
                std::cout << "   ✅ GOOD: Minimal drift while stationary\n";
            }
        } else {
            double avg_var = (x_var + y_var + z_var) / 3.0;
            std::cout << "   Robot is moving (position variance: " 
                      << std::fixed << std::setprecision(3) << avg_var << ")\n";
            std::cout << "   Total displacement: " << drift << " m\n";
        }
    }
    
    void printRecommendations()
    {
        std::cout << "\n5. CONFIGURATION RECOMMENDATIONS:\n";
        std::cout << "   Based on the above diagnostics:\n";
        
        std::vector<std::string> issues;
        
        // Check N_SCAN issue
        if (!ring_counts_.empty()) {
            uint8_t max_ring = ring_counts_.rbegin()->first;
            if (max_ring < 3) {
                issues.push_back("Set N_SCAN: 4 in params.yaml (CRITICAL)");
            }
        }
        
        // Check LiDAR frequency
        if (lidar_timestamps_.size() > 1) {
            std::vector<double> dts;
            for (size_t i = 1; i < lidar_timestamps_.size(); ++i) {
                dts.push_back(lidar_timestamps_[i] - lidar_timestamps_[i-1]);
            }
            double avg_dt = std::accumulate(dts.begin(), dts.end(), 0.0) / dts.size();
            double freq = 1.0 / avg_dt;
            
            if (freq < 8) {
                issues.push_back("Check LiDAR driver configuration (low frequency)");
            }
        }
        
        // Check IMU frequency
        if (imu_timestamps_.size() > 1) {
            std::vector<double> dts;
            for (size_t i = 1; i < imu_timestamps_.size(); ++i) {
                dts.push_back(imu_timestamps_[i] - imu_timestamps_[i-1]);
            }
            double avg_dt = std::accumulate(dts.begin(), dts.end(), 0.0) / dts.size();
            double freq = 1.0 / avg_dt;
            
            if (freq < 150) {
                issues.push_back("Check IMU rate in driver configuration");
            }
        }
        
        if (!issues.empty()) {
            std::cout << "\n   ❌ ISSUES FOUND:\n";
            for (size_t i = 0; i < issues.size(); ++i) {
                std::cout << "      " << (i+1) << ". " << issues[i] << "\n";
            }
        } else {
            std::cout << "\n   ✅ No critical issues detected\n";
            std::cout << "   If you're still experiencing drift, check:\n";
            std::cout << "      - IMU noise parameters\n";
            std::cout << "      - LiDAR-IMU extrinsic calibration\n";
            std::cout << "      - Feature extraction thresholds\n";
        }
    }
    
    double calculateVariance(const std::vector<double>& values)
    {
        if (values.empty()) return 0.0;
        
        double mean = std::accumulate(values.begin(), values.end(), 0.0) / values.size();
        double sq_sum = 0.0;
        
        for (double val : values) {
            sq_sum += (val - mean) * (val - mean);
        }
        
        return sq_sum / values.size();
    }

    // Member variables
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::map<uint8_t, size_t> ring_counts_;
    std::deque<double> imu_timestamps_;
    std::deque<double> lidar_timestamps_;
    std::deque<std::vector<double>> odom_positions_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<LioSamDiagnostic>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}