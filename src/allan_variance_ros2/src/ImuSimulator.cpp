/**
 * @file   ImuSimulator.cpp
 * @brief  Tool to simulate imu data, ref:
 * https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model.
 * @author Rick Liu
 */

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <ctime>
#include <set>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "allan_variance_ros2/yaml_parsers.hpp"

using Vec3d = Eigen::Vector3d;
using std::placeholders::_1;

Vec3d RandomNormalDistributionVector(double sigma)
{
  static boost::mt19937 rng;
  static boost::normal_distribution<> nd(0, 1);
  return {sigma * nd(rng), sigma * nd(rng), sigma * nd(rng)};
}

template <typename S, typename T>
void FillROSVector3d(const S &from, T &to)
{
  to.x = from.x();
  to.y = from.y();
  to.z = from.z();
}

class ImuSimulator
{
public:
  ImuSimulator(std::string config_file, std::string output_path)
  {
    auto yaml_config = loadYamlFile(config_file);

    get(yaml_config, "accelerometer_noise_density", accelerometer_noise_density_);
    get(yaml_config, "accelerometer_random_walk", accelerometer_random_walk_);
    get(yaml_config, "accelerometer_bias_init", accelerometer_bias_init_);
    get(yaml_config, "gyroscope_noise_density", gyroscope_noise_density_);
    get(yaml_config, "gyroscope_random_walk", gyroscope_random_walk_);
    get(yaml_config, "gyroscope_bias_init", gyroscope_bias_init_);
    get(yaml_config, "rostopic", rostopic_);
    get(yaml_config, "update_rate", update_rate_);
    get(yaml_config, "sequence_duration", sequence_duration_);

    RCLCPP_INFO_STREAM(rclcpp::get_logger("imu_sim"), "rostopic: " << rostopic_);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("imu_sim"), "output bag path: " << output_path);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("imu_sim"),"update_rate: " << update_rate_);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("imu_sim"),"sequence_duration: " << sequence_duration_);

    writer_.open(output_path);
    writer_.create_topic({
      .name = rostopic_,
      .type = "sensor_msgs/msg/Imu",
      .serialization_format = "cdr"
    });
  }

  void run()
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("imu_sim"), "Generating IMU data...");

    double dt = 1.0 / update_rate_;
    // clang-format off
    rclcpp::Time start_time = rclcpp::Clock().now();
    Vec3d acc_bias = Vec3d::Constant(accelerometer_bias_init_);
    Vec3d gyro_bias = Vec3d::Constant(gyroscope_bias_init_);
    Vec3d acc_real = Vec3d::Zero();
    Vec3d gyro_real = Vec3d::Zero();

    for (int64_t i = 0; i < sequence_duration_ * update_rate_; ++i)
    {
      // Break if requested by user
      if (!rclcpp::ok()) break;

      // Reference: https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model
      acc_bias += RandomNormalDistributionVector(accelerometer_random_walk_) * sqrt(dt);
      gyro_bias += RandomNormalDistributionVector(gyroscope_random_walk_) * sqrt(dt);

      Vec3d acc_measure = acc_real + acc_bias + RandomNormalDistributionVector(accelerometer_noise_density_) / sqrt(dt);
      Vec3d gyro_measure = gyro_real + gyro_bias + RandomNormalDistributionVector(gyroscope_noise_density_) / sqrt(dt);

      auto msg = sensor_msgs::msg::Imu();
      msg.header.stamp = start_time + rclcpp::Duration::from_seconds(i * dt);
      msg.header.frame_id = "imu_link";
      FillROSVector3d(acc_measure, msg.linear_acceleration);
      FillROSVector3d(gyro_measure, msg.angular_velocity);

      writer_.write(msg, rostopic_, msg.header.stamp);
    }

    RCLCPP_INFO_STREAM(rclcpp::get_logger("imu_sim"), "Finished generating IMU data.");
  }

private:
  rosbag2_cpp::Writer writer_;

  double accelerometer_noise_density_;
  double accelerometer_random_walk_;
  double accelerometer_bias_init_;

  double gyroscope_noise_density_;
  double gyroscope_random_walk_;
  double gyroscope_bias_init_;

  std::string rostopic_;
  double update_rate_;
  double sequence_duration_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc < 3)
  {
    RCLCPP_ERROR(rclcpp::get_logger("imu_sim"), "Usage: ./imu_simulator <output_bag_path> <config_file.yaml>");
    return 1;
  }

  std::string rosbag_filename = argv[1];
  std::string config_file = argv[2];

  auto node = rclcpp::Node::make_shared("imu_simulator");
  auto start = std::clock();

  ImuSimulator simulator(config_file, rosbag_filename);
  simulator.run();

  double durationTime = (std::clock() - start) / (double)CLOCKS_PER_SEC;
  RCLCPP_INFO_STREAM(rclcpp::get_logger("imu_sim"), "Total computation time: " << durationTime << " seconds");

  rclcpp::shutdown();
  return 0;
}

