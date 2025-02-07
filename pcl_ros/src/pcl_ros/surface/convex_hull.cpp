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
 * $Id: convex_hull.hpp 32993 2010-09-30 23:08:57Z rusu $
 *
 */

#include "pcl_ros/surface/convex_hull.hpp"

//////////////////////////////////////////////////////////////////////////////////////////////
pcl_ros::ConvexHull2D::ConvexHull2D(const rclcpp::NodeOptions & options)
: PCLNode("ConvextHull2DNode", options)
{
  init_parameters();
  subscribe();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void pcl_ros::ConvexHull2D::init_parameters()
{
  pub_output_ = create_publisher<sensor_msgs::msg::PointCloud2>("output", max_queue_size_);
  pub_plane_ =
    create_publisher<geometry_msgs::msg::PolygonStamped>("output_polygon", max_queue_size_);

  RCLCPP_DEBUG(get_logger(), "[init_parameters] Node initialized with no additional parameters");
}

//////////////////////////////////////////////////////////////////////////////////////////////
void pcl_ros::ConvexHull2D::subscribe()
{
  // If we're supposed to look for PointIndices (indices)
  if (use_indices_) {
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
    custom_qos_profile.depth = 1;

    // Subscribe to the input using a filter
    sub_input_filter_.subscribe(this, "input", custom_qos_profile);

    // If indices are enabled, subscribe to the indices
    sub_indices_filter_.subscribe(this, "indices", custom_qos_profile);

    if (approximate_sync_) {
      sync_input_indices_a_ = std::make_shared<message_filters::Synchronizer<
        sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, PointIndices>>>(
        max_queue_size_);

      // Surface not enabled, connect the input-indices duo and register
      sync_input_indices_a_->connectInput(sub_input_filter_, sub_indices_filter_);
      sync_input_indices_a_->registerCallback(std::bind(
        &ConvexHull2D::input_indices_callback, this, std::placeholders::_1, std::placeholders::_2));
    } else {
      sync_input_indices_e_ = std::make_shared<message_filters::Synchronizer<
        sync_policies::ExactTime<sensor_msgs::msg::PointCloud2, PointIndices>>>(max_queue_size_);

      // Surface not enabled, connect the input-indices duo and register
      sync_input_indices_e_->connectInput(sub_input_filter_, sub_indices_filter_);
      sync_input_indices_e_->registerCallback(std::bind(
        &ConvexHull2D::input_indices_callback, this, std::placeholders::_1, std::placeholders::_2));
    }
  } else {
    // Subscribe in an old fashion to input only (no filters)
    sub_input_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "input", 1, std::bind(&ConvexHull2D::input_callback, this, std::placeholders::_1));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void pcl_ros::ConvexHull2D::unsubscribe()
{
  if (use_indices_) {
    sub_input_filter_.unsubscribe();
    sub_indices_filter_.unsubscribe();
  } else {
    sub_input_.reset();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void pcl_ros::ConvexHull2D::input_indices_callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
  const pcl_msgs::msg::PointIndices::ConstSharedPtr & indices)
{
  // No subscribers, no work
  if (
    count_subscribers(pub_output_->get_topic_name()) <= 0 &&
    count_subscribers(pub_plane_->get_topic_name()) <= 0) {
    return;
  }

  sensor_msgs::msg::PointCloud2 output;
  output.header = cloud->header;

  // If cloud is given, check if it's valid
  if (!isValid(cloud)) {
    RCLCPP_ERROR(get_logger(), "[input_indices_callback] Invalid input!");
    // Publish an empty message
    pub_output_->publish(output);
    return;
  }

  // If indices are given, check if they are valid
  if (indices && !isValid(indices, "indices")) {
    RCLCPP_ERROR(get_logger(), "[input_indices_callback] Invalid indices!");
    // Publish an empty message
    pub_output_->publish(output);
    return;
  }

  if (indices) {
    RCLCPP_DEBUG(
      get_logger(),
      "[input_indices_callback]\n"
      "  - PointCloud with %d data points (%s), stamp %d.%09d, and frame %s on topic %s received.\n"
      "  - PointIndices with %zu values, stamp %d.%09d, and frame %s on topic %s received.",
      cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str(), cloud->header.stamp.sec,
      cloud->header.stamp.nanosec, cloud->header.frame_id.c_str(), "input", indices->indices.size(),
      indices->header.stamp.sec, indices->header.stamp.nanosec, indices->header.frame_id.c_str(),
      "indices");
  } else {
    RCLCPP_DEBUG(
      get_logger(),
      "[input_indices_callback] PointCloud with %d data points, stamp %d.%09d, and "
      "frame %s on topic %s received.",
      cloud->width * cloud->height, cloud->header.stamp.sec, cloud->header.stamp.nanosec,
      cloud->header.frame_id.c_str(), "input");
  }

  // Reset the indices and surface pointers
  IndicesPtr indices_ptr;
  if (indices) {
    indices_ptr.reset(new std::vector<int>(indices->indices));
  }
  impl_.setIndices(indices_ptr);

  // Generate the PCL data types
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcl_cloud =
    std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(*cloud, *pcl_cloud);
  impl_.setInputCloud(pcl_cloud);

  // Estimate the feature
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcl_output =
    std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  impl_.reconstruct(*pcl_output);

  // If more than 3 points are present, send a PolygonStamped hull too
  if (pcl_output->points.size() >= 3) {
    geometry_msgs::msg::PolygonStamped poly;
    poly.header = output.header;
    poly.polygon.points.resize(pcl_output->points.size());

    // Get three consecutive points (without copying)
    pcl::Vector4fMap O = pcl_output->points[1].getVector4fMap();
    pcl::Vector4fMap B = pcl_output->points[0].getVector4fMap();
    pcl::Vector4fMap A = pcl_output->points[2].getVector4fMap();

    // Check the direction of points -- polygon must have CCW direction
    Eigen::Vector4f OA = A - O;
    Eigen::Vector4f OB = B - O;
    Eigen::Vector4f N = OA.cross3(OB);
    double theta = N.dot(O);
    bool reversed = false;
    if (theta < (M_PI / 2.0)) {
      reversed = true;
    }

    for (size_t i = 0; i < pcl_output->points.size(); ++i) {
      if (reversed) {
        size_t j = pcl_output->points.size() - i - 1;
        poly.polygon.points[i].x = pcl_output->points[j].x;
        poly.polygon.points[i].y = pcl_output->points[j].y;
        poly.polygon.points[i].z = pcl_output->points[j].z;
      } else {
        poly.polygon.points[i].x = pcl_output->points[i].x;
        poly.polygon.points[i].y = pcl_output->points[i].y;
        poly.polygon.points[i].z = pcl_output->points[i].z;
      }
    }
    pub_plane_->publish(poly);
  }

  // Publish the result
  output.header = cloud->header;
  pub_output_->publish(output);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_ros::ConvexHull2D)
