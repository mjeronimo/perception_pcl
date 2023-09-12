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
  : PCLNode("EuclideanClusterExtractionNode", options)
{
  // TODO(mjeronimo)
  // def add (self, name, paramtype, level, description, default = None, min = None, max = None, edit_method = ""):
  // gen.add ("cluster_tolerance", double_t, 0, "The spatial tolerance as a measure in the L2 Euclidean space", 0.05, 0.0, 2.0)
  // gen.add ("cluster_min_size", int_t, 0, "The minimum number of points that a cluster must contain in order to be accepted", 1, 0, 1000)
  // gen.add ("cluster_max_size", int_t, 0, "The maximum number of points that a cluster must contain in order to be accepted", 2147483647, 0, 2147483647)
  // gen.add ("max_clusters", int_t, 0, "The maximum number of clusters to extract.", 2147483647, 1, 2147483647)
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
    sub_input_ =
      create_subscription<sensor_msgs::msg::PointCloud2>("input", max_queue_size_,
        std::bind(&pcl_ros::EuclideanClusterExtraction::input_callback, this, std::placeholders::_1));
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

  for (const rclcpp::Parameter & param : params) {
    if (param.get_name() == "cluster_tolerance") {
      double cur_cluster_tolerance = impl_.getClusterTolerance();
      double new_cluster_tolerance = param.as_double();
      if (cur_cluster_tolerance != new_cluster_tolerance) {
        impl_.setClusterTolerance(new_cluster_tolerance);
        RCLCPP_DEBUG(
          get_logger(),
          "[config_callback] Setting new clustering tolerance to: %f.",
          new_cluster_tolerance);
      }
    }

    if (param.get_name() == "cluster_min_size")  {
      int cur_cluster_min_size = impl_.getMinClusterSize();
      int new_cluster_min_size = param.as_int();

      if (cur_cluster_min_size != new_cluster_min_size) {
        impl_.setMinClusterSize(new_cluster_min_size);
        RCLCPP_DEBUG(
          get_logger(),
          "[config_callback] Setting the minimum cluster size to: %d.",
          new_cluster_min_size);
      }
    }

    if (param.get_name() == "cluster_min_size")  {
      int cur_cluster_max_size = impl_.getMaxClusterSize();
      int new_cluster_max_size = param.as_int();

      if (cur_cluster_max_size != new_cluster_max_size) {
        impl_.setMaxClusterSize(new_cluster_max_size);
        RCLCPP_DEBUG(
          get_logger(),
          "[config_callback] Setting the maximum cluster size to: %d.",
          new_cluster_max_size);
      }
    }

    if (param.get_name() == "max_clusters")  {
      int new_max_clusters = param.as_int();

      if (max_clusters_ != new_max_clusters) {
        max_clusters_ = new_max_clusters;
        RCLCPP_DEBUG(
          get_logger(),
          "[config_callback] Setting the maximum cluster size to: %d.",
          new_max_clusters);
      }
    }
  }

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
  if (indices) {
    RCLCPP_DEBUG(
      get_logger(),
      "[input_indices_callback]\n"
      "  - PointCloud with %d data points (%s), stamp %d.%09d, and frame %s on topic %s received.\n"
      "  - PointIndices with %zu values, stamp %d.%09d, and frame %s on topic %s received.",
      cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str(),
      cloud->header.stamp.sec, cloud->header.stamp.nanosec, cloud->header.frame_id.c_str(), "input",
      indices->indices.size(), indices->header.stamp.sec, indices->header.stamp.nanosec,
      indices->header.frame_id.c_str(), "indices");
  } else {
    RCLCPP_DEBUG(
      get_logger(),
      "[input_callback] PointCloud with %d data points, stamp %d.%09d, and frame %s on topic %s received.",
      cloud->width * cloud->height, cloud->header.stamp.sec, cloud->header.stamp.nanosec,
      cloud->header.frame_id.c_str(), "input");
  }
  ///

  IndicesPtr indices_ptr;
  if (indices) {
    indices_ptr.reset(new std::vector<int>(indices->indices));
  }

  // Convert from cloud to pcl_cloud
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcl_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(*cloud, *pcl_cloud);

  impl_.setInputCloud(pcl_cloud);
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

      // Copy the clusters to the new PointCloud output
      boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcl_output = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      pcl::copyPointCloud(*pcl_cloud, clusters[i].indices, *pcl_output);

      // Convert to a ROS message
      sensor_msgs::msg::PointCloud2 ros_output;
      pcl::toROSMsg(*pcl_output, ros_output);

      // Update the timestamp
      ros_output.header.stamp = rclcpp::Time(ros_output.header.stamp.sec,
                                             ros_output.header.stamp.nanosec) +
                                rclcpp::Duration::from_seconds(i * 0.001);

      // Publish the output
      pub_output_->publish(ros_output);
      RCLCPP_DEBUG(
          get_logger(),
          "[segmentAndPublish] Published cluster %zu (with %zu values and stamp %d.%09d) on topic %s",
          i, clusters[i].indices.size(), ros_output.header.stamp.sec, ros_output.header.stamp.nanosec, "output");
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::EuclideanClusterExtraction::input_callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
{
  return input_indices_callback(cloud, PointIndicesConstPtr());
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_ros::EuclideanClusterExtraction)
