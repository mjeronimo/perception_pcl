/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: segment_differences.cpp 35361 2011-01-20 04:34:49Z rusu $
 *
 */

#include "pcl_ros/segmentation/segment_differences.hpp"

#include <pcl/common/io.h>

//////////////////////////////////////////////////////////////////////////////////////////////
pcl_ros::SegmentDifferences::SegmentDifferences(const rclcpp::NodeOptions & options)
: PCLNode("SegmentDifferencesNode", options)
{
  init_parameters();
  subscribe();
  pub_output_ = create_publisher<sensor_msgs::msg::PointCloud2>("output", max_queue_size_);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void pcl_ros::SegmentDifferences::init_parameters()
{
  add_parameter(
    "distance_threshold", rclcpp::ParameterValue(distance_threshold_),
    floating_point_range{0.0, 2.0, 0.0},  // from, to, step
    "The distance tolerance as a measure in the L2 Euclidean space between "
    "corresponding points");

  distance_threshold_ = get_parameter("distance_threshold").as_double();

  // Set the parameters on the underlying implementation
  impl_.setDistanceThreshold(distance_threshold_);

  // Initialize the parameter callback
  set_parameters_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&SegmentDifferences::set_parameters_callback, this, std::placeholders::_1));

  RCLCPP_DEBUG(
    get_logger(),
    "[init_parameters] Node initialized with the following parameters:\n"
    " - distance_threshold: %f",
    distance_threshold_);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void pcl_ros::SegmentDifferences::subscribe()
{
  // Subscribe to the input using a filter
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
  custom_qos_profile.depth = max_queue_size_;
  sub_input_filter_.subscribe(this, "input", custom_qos_profile);
  sub_target_filter_.subscribe(this, "target", custom_qos_profile);

  if (approximate_sync_) {
    sync_input_target_a_ =
      std::make_shared<message_filters::Synchronizer<sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>>>(max_queue_size_);
    sync_input_target_a_->connectInput(sub_input_filter_, sub_target_filter_);
    sync_input_target_a_->registerCallback(bind(
      &SegmentDifferences::input_target_callback, this, std::placeholders::_1,
      std::placeholders::_2));
  } else {
    sync_input_target_e_ = std::make_shared<message_filters::Synchronizer<
      sync_policies::ExactTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>>>(
      max_queue_size_);
    sync_input_target_e_->connectInput(sub_input_filter_, sub_target_filter_);
    sync_input_target_e_->registerCallback(bind(
      &SegmentDifferences::input_target_callback, this, std::placeholders::_1,
      std::placeholders::_2));
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void pcl_ros::SegmentDifferences::unsubscribe()
{
  sub_input_filter_.unsubscribe();
  sub_target_filter_.unsubscribe();
}

//////////////////////////////////////////////////////////////////////////////////////////////
rcl_interfaces::msg::SetParametersResult pcl_ros::SegmentDifferences::set_parameters_callback(
  const std::vector<rclcpp::Parameter> & params)
{
  std::lock_guard<std::mutex> lock(mutex_);

  for (const rclcpp::Parameter & param : params) {
    if (param.get_name() == "distance_threshold") {
      double new_distance_threshold = param.as_double();
      if (distance_threshold_ != new_distance_threshold) {
        distance_threshold_ = new_distance_threshold;
        RCLCPP_DEBUG(
          get_logger(), "[set_parameters_callback] Setting new distance threshold to: %f.",
          distance_threshold_);
        impl_.setDistanceThreshold(distance_threshold_);
      }
    }
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void pcl_ros::SegmentDifferences::input_target_callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_target)
{
  // No subscribers, no work
  if (count_subscribers(pub_output_->get_topic_name()) <= 0) {
    return;
  }

  if (!isValid(cloud) || !isValid(cloud_target, "target")) {
    RCLCPP_ERROR(get_logger(), "[input_indices_callback] Invalid input!");
    sensor_msgs::msg::PointCloud2 output;
    output.header = cloud->header;
    pub_output_->publish(output);
    return;
  }

  RCLCPP_DEBUG(
    get_logger(),
    "[input_indices_callback]\n"
    "  - PointCloud with %d data points (%s), stamp %d.%09d, and frame %s on topic %s "
    "received.\n"
    "  - PointCloud with %d data points (%s), stamp %d.%09d, and frame %s on topic %s "
    "received.",
    cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str(), cloud->header.stamp.sec,
    cloud->header.stamp.nanosec, cloud->header.frame_id.c_str(), "input",
    cloud_target->width * cloud_target->height, pcl::getFieldsList(*cloud_target).c_str(),
    cloud_target->header.stamp.sec, cloud_target->header.stamp.nanosec,
    cloud_target->header.frame_id.c_str(), "target");

  // Acquire the mutex before accessing the underlying implementation */
  std::lock_guard<std::mutex> lock(mutex_);

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcl_cloud =
    boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(*cloud, *pcl_cloud);
  impl_.setInputCloud(pcl_cloud);

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcl_cloud_target =
    boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(*cloud_target, *pcl_cloud_target);
  impl_.setTargetCloud(pcl_cloud_target);

  PointCloud output;
  impl_.segment(output);

  sensor_msgs::msg::PointCloud2 ros_output;
  pcl::toROSMsg(output, ros_output);
  pub_output_->publish(ros_output);

  RCLCPP_DEBUG(
    get_logger(),
    "[segmentAndPublish] Published PointCloud2 with %lu points and stamp %d.%09d on "
    "topic %s",
    output.points.size(), fromPCL(output.header).stamp.sec, fromPCL(output.header).stamp.nanosec,
    "output");
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_ros::SegmentDifferences)
