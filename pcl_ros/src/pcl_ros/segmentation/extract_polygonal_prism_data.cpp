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
 * $Id: extract_polygonal_prism_data.hpp 32996 2010-09-30 23:42:11Z rusu $
 *
 */

#include <vector>

#include <pcl/common/io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/segmentation/extract_polygonal_prism_data.hpp>
#include <pcl_ros/transforms.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>

using pcl_conversions::moveFromPCL;
using pcl_conversions::moveToPCL;

//////////////////////////////////////////////////////////////////////////////////////////////
pcl_ros::ExtractPolygonalPrismData::ExtractPolygonalPrismData(const rclcpp::NodeOptions & options)
: PCLNode("ExtractPolygonalPrismDataNode", options)
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::ExtractPolygonalPrismData::subscribe()
{
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
  custom_qos_profile.depth = max_queue_size_;
  sub_hull_filter_.subscribe(this, "planar_hull", custom_qos_profile);
  sub_input_filter_.subscribe(this, "input", custom_qos_profile);

  // Create the objects here
  if (approximate_sync_) {
    sync_input_hull_indices_a_ =
      std::make_shared<message_filters::Synchronizer<
          sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2, PointIndices>>>(max_queue_size_);
  } else {
    sync_input_hull_indices_e_ =
      std::make_shared<message_filters::Synchronizer<
          sync_policies::ExactTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2, PointIndices>>>(max_queue_size_);
  }

  if (use_indices_) {
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
    custom_qos_profile.depth = max_queue_size_;
    sub_indices_filter_.subscribe(this, "indices", custom_qos_profile);
    if (approximate_sync_) {
      sync_input_hull_indices_a_->connectInput(
        sub_input_filter_, sub_hull_filter_,
        sub_indices_filter_);
    } else {
      sync_input_hull_indices_e_->connectInput(
        sub_input_filter_, sub_hull_filter_,
        sub_indices_filter_);
    }
  } else {
    sub_input_filter_.registerCallback(std::bind(&ExtractPolygonalPrismData::input_callback, this, std::placeholders::_1));

    if (approximate_sync_) {
      sync_input_hull_indices_a_->connectInput(sub_input_filter_, sub_hull_filter_, nf_);
    } else {
      sync_input_hull_indices_e_->connectInput(sub_input_filter_, sub_hull_filter_, nf_);
    }
  }

  // Register callbacks
  if (approximate_sync_) {
    sync_input_hull_indices_a_->registerCallback(std::bind(&ExtractPolygonalPrismData::input_hull_indices_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  } else {
    sync_input_hull_indices_e_->registerCallback(std::bind(&ExtractPolygonalPrismData::input_hull_indices_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::ExtractPolygonalPrismData::unsubscribe()
{
  sub_hull_filter_.unsubscribe();
  sub_input_filter_.unsubscribe();

  if (use_indices_) {
    sub_indices_filter_.unsubscribe();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
rcl_interfaces::msg::SetParametersResult
pcl_ros::ExtractPolygonalPrismData::config_callback(const std::vector<rclcpp::Parameter> & params)
{
  std::lock_guard<std::mutex> lock(mutex_);

  double height_min;
  double height_max;
  impl_.getHeightLimits(height_min, height_max);

  for (const rclcpp::Parameter & param : params) {
    if (param.get_name() == "height_min") {
      height_min = param.as_double();
      RCLCPP_DEBUG(
        get_logger(),
        "[config_callback] Setting new minimum height to the planar model to: %f.",
        height_min);
      impl_.setHeightLimits(height_min, height_max);
    }

    if (param.get_name() == "height_max") {
      height_max = param.as_double();
      RCLCPP_DEBUG(
        get_logger(),
        "[config_callback] Setting new maximum height to the planar model to: %f.",
        height_max);
      impl_.setHeightLimits(height_min, height_max);
    }
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::ExtractPolygonalPrismData::input_hull_indices_callback(
  const PointCloud2::ConstSharedPtr & cloud,
  const PointCloud2::ConstSharedPtr & hull,
  const PointIndices::ConstSharedPtr & indices)
{
  // No subscribers, no work
  if (count_subscribers(pub_output_->get_topic_name()) <= 0) {
    return;
  }

  // Copy the header (stamp + frame_id)
  pcl_msgs::msg::PointIndices inliers;
  inliers.header = cloud->header;

  // If cloud is given, check if it's valid
  if (!isValid(cloud) || !isValid(hull, "planar_hull")) {
    RCLCPP_ERROR(get_logger(), "[input_hull_indices_callback] Invalid input!");
    pub_output_->publish(inliers);
    return;
  }
  // If indices are given, check if they are valid
  if (indices && !isValid(indices)) {
    RCLCPP_ERROR(get_logger(), "[input_hull_indices_callback] Invalid indices!");
    pub_output_->publish(inliers);
    return;
  }

  /// DEBUG
  if (indices) {
    RCLCPP_DEBUG(
      get_logger(),
      "[input_indices_hull_callback]\n"
      "  - PointCloud with %d data points (%s), stamp %d.%09d, and frame %s on topic %s received.\n"
      "  - PointCloud with %d data points (%s), stamp %d.%09d, and frame %s on topic %s received.\n"
      "  - PointIndices with %zu values, stamp %d.%09d, and frame %s on topic %s received.",
      cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str(),
      cloud->header.stamp.sec, cloud->header.stamp.nanosec, cloud->header.frame_id.c_str(), "input",
      hull->width * hull->height, pcl::getFieldsList(*hull).c_str(),
      hull->header.stamp.sec, hull->header.stamp.nanosec, hull->header.frame_id.c_str(), "planar_hull",
      indices->indices.size(), indices->header.stamp.sec, indices->header.stamp.nanosec,
      indices->header.frame_id.c_str(), "indices");
  } else {
    RCLCPP_DEBUG(
      get_logger(), 
      "[input_indices_hull_callback]\n"
      "PointCloud with %d data points and frame %s on topic %s received.",
      "PointCloud with %d data points and frame %s on topic %s received.",
      cloud->width * cloud->height, cloud->header.frame_id.c_str(), "input",
      hull->width * hull->height, hull->header.frame_id.c_str(), "planar_hull");
  }
  ///

  // Check whether the user has given a different input TF frame
  if (cloud->header.frame_id != hull->header.frame_id) {
    RCLCPP_DEBUG(
      get_logger(),
      "[input_hull_callback] Planar hull has a different TF frame (%s) than the input "
      "point cloud (%s)! Using TF to transform.",
      hull->header.frame_id.c_str(), cloud->header.frame_id.c_str());
    PointCloud2 planar_hull;
    if (!pcl_ros::transformPointCloud(cloud->header.frame_id, *hull, planar_hull, tf_buffer_)) {
      // Publish empty before return
      pub_output_->publish(inliers);
      return;
    }

    // Convert from planar_hull to pcl_hull
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcl_hull = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(planar_hull, *pcl_hull);
    impl_.setInputPlanarHull(pcl_hull);

  } else {
    // Convert from hull to pcl_hull
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcl_hull = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(*hull, *pcl_hull);
    impl_.setInputPlanarHull(pcl_hull);
  }

  IndicesPtr indices_ptr;
  if (indices && !indices->header.frame_id.empty()) {
    indices_ptr.reset(new std::vector<int>(indices->indices));
  }

  // Convert from cloud to pcl_cloud
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcl_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(*cloud, *pcl_cloud);

  impl_.setInputCloud(pcl_cloud);
  impl_.setIndices(indices_ptr);

  // Final check if the data is empty
  if (!cloud->width && !cloud->height) {
    pcl::PointIndices pcl_inliers;
    moveToPCL(inliers, pcl_inliers);
    impl_.segment(pcl_inliers);
    moveFromPCL(pcl_inliers, inliers);
  }
  // Enforce that the TF frame and the timestamp are copied
  inliers.header = cloud->header;
  pub_output_->publish(inliers);
  RCLCPP_DEBUG(
    get_logger(),
    "[input_hull_callback] Publishing %zu indices.",
    inliers.indices.size());
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_ros::ExtractPolygonalPrismData)
