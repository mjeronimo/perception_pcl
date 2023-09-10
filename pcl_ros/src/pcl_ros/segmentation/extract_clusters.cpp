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
 * $Id: extract_clusters.hpp 32052 2010-08-27 02:19:30Z rusu $
 *
 */

#include "pcl_ros/segmentation/extract_clusters.hpp"

#include <functional>
#include <vector>

#include "pcl/common/io.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/PointIndices.h"
#include "rclcpp/rclcpp.hpp"

using pcl_conversions::fromPCL;
using pcl_conversions::moveFromPCL;
using pcl_conversions::toPCL;

//////////////////////////////////////////////////////////////////////////////////////////////
pcl_ros::EuclideanClusterExtraction::EuclideanClusterExtraction(const rclcpp::NodeOptions & options)
  : PCLNode("EuclideanClusterExtractionNode", options),
    publish_indices_(false),
    max_clusters_(std::numeric_limits<int>::max())
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::EuclideanClusterExtraction::onInit()
{
  // ---[ Mandatory parameters
  double cluster_tolerance;
  if (!get_parameter("cluster_tolerance", cluster_tolerance)) {
    RCLCPP_ERROR(get_logger(), "[onInit] Need a 'cluster_tolerance' parameter to be set before continuing!");
    return;
  }

  get_parameter("publish_indices", publish_indices_);
  if (publish_indices_) {
    pub_indices = create_publisher<pcl_msgs::msg::PointIndices>("output", max_queue_size_);
  } else {
    pub_output_ = create_publisher<sensor_msgs::msg::PointCloud2>("output", max_queue_size_);
  }

  RCLCPP_DEBUG(
    get_logger(),
    "[onInit] Nodelet successfully created with the following parameters:\n"
    " - max_queue_size    : %d\n"
    " - use_indices       : %s\n"
    " - cluster_tolerance : %f\n",
    max_queue_size_, (use_indices_) ? "true" : "false", cluster_tolerance);

  // Set given parameters here
  impl_.setClusterTolerance(cluster_tolerance);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::EuclideanClusterExtraction::subscribe()
{
  // If we're supposed to look for PointIndices (indices)
  if (use_indices_) {
    // Subscribe to the input using a filter
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
    custom_qos_profile.depth = max_queue_size_;
    sub_input_filter_.subscribe(this, "input", custom_qos_profile);
    sub_indices_filter_.subscribe(this, "indices", custom_qos_profile);

    if (approximate_sync_) {
      sync_input_indices_a_ =
        boost::make_shared<message_filters::Synchronizer<
            message_filters::sync_policies::ApproximateTime<
              sensor_msgs::msg::PointCloud2, pcl_msgs::msg::PointIndices>>>(max_queue_size_);
      sync_input_indices_a_->connectInput(sub_input_filter_, sub_indices_filter_);
      sync_input_indices_a_->registerCallback(
        bind(
          &EuclideanClusterExtraction::
          input_indices_callback, this, std::placeholders::_1, std::placeholders::_2));
    } else {
      sync_input_indices_e_ =
        boost::make_shared<message_filters::Synchronizer<
            message_filters::sync_policies::ExactTime<sensor_msgs::msg::PointCloud2, pcl_msgs::msg::PointIndices>>>(max_queue_size_);
      sync_input_indices_e_->connectInput(sub_input_filter_, sub_indices_filter_);
      sync_input_indices_e_->registerCallback(
        bind(
          &EuclideanClusterExtraction::
          input_indices_callback, this, std::placeholders::_1, std::placeholders::_2));
    }
  } else {
    // Subscribe in an old fashion to input only (no filters)
#if 0
std::function<void(PointCloud2::ConstSharedPtr &, pcl_msgs::msg::PointIndices::ConstSharedPtr &)> callback;
    sub_input_ =
      create_subscription<sensor_msgs::msg::PointCloud2>(
      "input", rclcpp::SensorDataQoS().keep_last(max_queue_size_), callback);
      // std::bind(&EuclideanClusterExtraction::input_indices_callback, this, std::placeholders::_1, std::placeholders::_2));
#endif
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::EuclideanClusterExtraction::unsubscribe()
{
  if (use_indices_) {
    sub_input_filter_.unsubscribe();
    sub_indices_filter_.unsubscribe();
  } else {
    sub_input_.reset();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
rcl_interfaces::msg::SetParametersResult
pcl_ros::EuclideanClusterExtraction::config_callback(const std::vector<rclcpp::Parameter> & params)
{
  std::lock_guard<std::mutex> lock(mutex_);

  auto cluster_tolerance = impl_.getClusterTolerance();
  auto cluster_min_size = impl_.getMinClusterSize();
  auto cluster_max_size = impl_.getMaxClusterSize();

  for (const rclcpp::Parameter & param : params) {
    if (param.get_name() == "cluster_tolerance") {
        // TODO(mjeronimo)
    }
  }


#if 0
  if (impl_.getClusterTolerance() != config.cluster_tolerance) {
    impl_.setClusterTolerance(config.cluster_tolerance);
    RCLCPP_DEBUG(
      get_logger(),
      "[config_callback] Setting new clustering tolerance to: %f.",
      config.cluster_tolerance);
  }
  if (impl_.getMinClusterSize() != config.cluster_min_size) {
    impl_.setMinClusterSize(config.cluster_min_size);
    RCLCPP_DEBUG(
      get_logger(),
      "[config_callback] Setting the minimum cluster size to: %d.",
      config.cluster_min_size);
  }
  if (impl_.getMaxClusterSize() != config.cluster_max_size) {
    impl_.setMaxClusterSize(config.cluster_max_size);
    RCLCPP_DEBUG(
      get_logger(),
      "[config_callback] Setting the maximum cluster size to: %d.",
      config.cluster_max_size);
  }
  if (max_clusters_ != config.max_clusters) {
    max_clusters_ = config.max_clusters;
    RCLCPP_DEBUG(
      get_logger(),
      "[config_callback] Setting the maximum number of clusters to extract to: %d.",
      config.max_clusters);
  }
#endif

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::EuclideanClusterExtraction::input_indices_callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
  const pcl_msgs::msg::PointIndices::ConstSharedPtr & indices)
{
  // No subscribers, no work
  if (count_subscribers(pub_output_->get_topic_name()) <= 0) {
    return;
  }

  // If cloud is given, check if it's valid
  if (!isValid(cloud)) {
    RCLCPP_ERROR(get_logger(), "[input_indices_callback] Invalid input!");
    return;
  }
  // If indices are given, check if they are valid
  if (indices && !isValid(indices)) {
    RCLCPP_ERROR(get_logger(), "[input_indices_callback] Invalid indices!");
    return;
  }

  /// DEBUG
  #if JERONIMO
  if (indices) {
    std_msgs::msg::Header cloud_header = fromPCL(cloud->header);
    std_msgs::msg::Header indices_header = indices->header;
    RCLCPP_DEBUG(
      get_logger(),
      "[input_indices_callback]\n"
      "  - PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.\n"
      "  - PointIndices with %zu values, stamp %f, and frame %s on topic %s received.",
      cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str(),
      cloud_header.stamp.toSec(), cloud_header.frame_id.c_str(), "input",
      indices->indices.size(), indices_header.stamp.seconds(),
      indices_header.frame_id.c_str(), "indices");
  } else {
    RCLCPP_DEBUG(
      get_logger(),
      "[input_callback] PointCloud with %d data points, stamp %f, and frame %s on topic %s received.",
      cloud->width * cloud->height, fromPCL(
      cloud->header).stamp.toSec(),
      cloud->header.frame_id.c_str(), "input");
  }
#endif
  ///

  IndicesPtr indices_ptr;
  if (indices) {
    indices_ptr.reset(new std::vector<int>(indices->indices));
  }

  // impl_.setInputCloud(pcl_ptr(cloud));
#if 0
  impl_.setInputCloud(pcl_input);
#endif

  impl_.setIndices(indices_ptr);

  std::vector<pcl::PointIndices> clusters;
  impl_.extract(clusters);

  if (publish_indices_) {
    for (size_t i = 0; i < clusters.size(); ++i) {
      if (static_cast<int>(i) >= max_clusters_) {
        break;
      }
      // TODO(xxx): HACK!!! We need to change the PointCloud2 message to add for an incremental
      // sequence ID number.
      pcl_msgs::msg::PointIndices ros_pi;
      moveFromPCL(clusters[i], ros_pi);
      ros_pi.header.stamp = rclcpp::Time(ros_pi.header.stamp.sec, ros_pi.header.stamp.nanosec) + rclcpp::Duration::from_seconds(i * 0.001);
      pub_indices->publish(ros_pi);
    }

    RCLCPP_DEBUG(
      get_logger(),
      "[segmentAndPublish] Published %zu clusters (PointIndices) on topic %s",
      clusters.size(), get_node_topics_interface()->resolve_topic_name("output").c_str());
  } else {
    for (size_t i = 0; i < clusters.size(); ++i) {
      if (static_cast<int>(i) >= max_clusters_) {
        break;
      }
      sensor_msgs::msg::PointCloud2 output;

      // TODO(mjeronimo)
      // pcl::copyPointCloud(*cloud, clusters[i].indices, output);

      // PointCloud output_blob;     // Convert from the templated output to the PointCloud blob
      // pcl::toROSMsg (output, output_blob);
      // TODO(xxx): HACK!!! We need to change the PointCloud2 message to add for an incremental
      // sequence ID number.

      // std_msgs::msg::Header header = fromPCL(output.header);
      std_msgs::msg::Header header = output.header;
      header.stamp = rclcpp::Time(header.stamp.sec, header.stamp.nanosec) + rclcpp::Duration::from_seconds(i * 0.001);

      // TODO(mjeronimo)
      // toPCL(header, output.header);

      // Publish a Boost shared ptr const data
      // pub_output_->publish(ros_ptr(output.makeShared()));
      // TODO(MJERONIMO)
      pub_output_->publish(output);

      RCLCPP_DEBUG(
        get_logger(),
        "[segmentAndPublish] Published cluster %zu (with %zu values and stamp %d.%09d) on topic %s",
        i, clusters[i].indices.size(), header.stamp.sec, header.stamp.nanosec, "output");
    }
  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_ros::EuclideanClusterExtraction)
