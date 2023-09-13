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

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/PointIndices.h"
#include "rclcpp/rclcpp.hpp"

using pcl_conversions::moveFromPCL;

//////////////////////////////////////////////////////////////////////////////////////////////
pcl_ros::EuclideanClusterExtraction::EuclideanClusterExtraction(const rclcpp::NodeOptions & options)
  : PCLNode("EuclideanClusterExtractionNode", options)
{
  init_parameters();
  subscribe();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::EuclideanClusterExtraction::init_parameters()
{
  add_parameter(
    "cluster_tolerance",
    rclcpp::ParameterValue(cluster_tolerance_),
    floating_point_range{0.0, 2.0, 0.0},  // from, to, step
    "The spatial tolerance as a measure in the L2 Euclidean space");

  add_parameter(
    "cluster_min_size",
    rclcpp::ParameterValue(cluster_min_size_),
    integer_range{0, 1000, 0},  // from, to, step
    "The minimum number of points that a cluster must contain in order to be accepted");

  add_parameter(
    "cluster_max_size",
    rclcpp::ParameterValue(cluster_max_size_),
    integer_range{0, 2147483647, 0},  // from, to, step
    "The maximum number of points that a cluster must contain in order to be accepted");

  add_parameter(
    "max_clusters",
    rclcpp::ParameterValue(max_clusters_),
    integer_range{1, 2147483647, 0},  // from, to, step
    "The maximum number of clusters to extract");
  
  add_parameter(
    "publish_indices",
    rclcpp::ParameterValue(publish_indices_),
    "Whether to publish the point cloud indices");

  cluster_tolerance_ = get_parameter("cluster_tolerance").as_double();
  cluster_min_size_ = get_parameter("cluster_min_size").as_int();
  cluster_max_size_ = get_parameter("cluster_max_size").as_int();
  max_clusters_ = get_parameter("max_clusters").as_int();
  publish_indices_ = get_parameter("publish_indices").as_bool();

  // Set the parameters on the underlying implementation
  impl_.setClusterTolerance(cluster_tolerance_);
  impl_.setMinClusterSize(cluster_min_size_);
  impl_.setMaxClusterSize(cluster_max_size_);

  // Create the requested output publisher type
  if (publish_indices_) {
    pub_indices = create_publisher<pcl_msgs::msg::PointIndices>("output", max_queue_size_);
  } else {
    pub_output_ = create_publisher<sensor_msgs::msg::PointCloud2>("output", max_queue_size_);
  }

  RCLCPP_DEBUG(
    get_logger(),
    "[init_parameters] Node initialized with the following parameters:\n"
    " - cluster_tolerance : %f\n"
    " - cluster_min_size : %d\n"
    " - cluster_max_size : %d\n"
    " - max_clusters : %d\n"
    " - publish_indices : %s\n", 
    cluster_tolerance_, cluster_min_size_, cluster_max_size_, max_clusters_, publish_indices_? "true" : "false");
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
      double new_cluster_tolerance = param.as_double();
      if (cluster_tolerance_ != new_cluster_tolerance) {
        cluster_tolerance_ = new_cluster_tolerance;
        RCLCPP_DEBUG(
          get_logger(),
          "[config_callback] Setting new clustering tolerance to: %f.",
          cluster_tolerance_);
        impl_.setClusterTolerance(cluster_tolerance_);
      }
    }

    if (param.get_name() == "cluster_min_size")  {
      int new_cluster_min_size = param.as_int();
      if (cluster_min_size_ != new_cluster_min_size) {
        cluster_min_size_ = new_cluster_min_size;
        RCLCPP_DEBUG(
          get_logger(),
          "[config_callback] Setting the minimum cluster size to: %d.",
          cluster_min_size_);
        impl_.setMinClusterSize(cluster_min_size_);
      }
    }

    if (param.get_name() == "cluster_min_size")  {
      int new_cluster_max_size = param.as_int();
      if (cluster_max_size_ != new_cluster_max_size) {
        cluster_max_size_ = new_cluster_max_size;
        RCLCPP_DEBUG(
          get_logger(),
          "[config_callback] Setting the maximum cluster size to: %d.",
          cluster_max_size_);
        impl_.setMaxClusterSize(cluster_max_size_);
      }
    }

    if (param.get_name() == "max_clusters")  {
      int new_max_clusters = param.as_int();
      if (max_clusters_ != new_max_clusters) {
        RCLCPP_DEBUG(
          get_logger(),
          "[config_callback] Setting the maximum cluster size to: %d.",
          max_clusters_);
      }
    }

    // Note: "publish_indices" cannot be dynamically updated. It is only respected
    // at construction, where the output publisher is created.
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

  // Acquire the mutex before accessing the underlying implementation */
  std::lock_guard<std::mutex> lock(mutex_);

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
