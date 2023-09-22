/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 *  COPYRIGHT OWNERff OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: normal_3d.cpp 35361 2011-01-20 04:34:49Z rusu $
 *
 */

#include "pcl_ros/features/normal_3d.hpp"

void pcl_ros::NormalEstimation::emptyPublish(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
{
  sensor_msgs::msg::PointCloud2 output;
  output.header = cloud->header;
  pub_output_->publish(output);
}

void pcl_ros::NormalEstimation::computePublish(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & surface, const IndicesPtr & indices)
{
  // Set the parameters
  impl_.setKSearch(k_);
  impl_.setRadiusSearch(search_radius_);

  // Convert from cloud to pcl_cloud
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcl_cloud =
    boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(*cloud, *pcl_cloud);

  // Convert from surface to pcl_surface
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcl_surface =
    boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(*surface, *pcl_surface);

  // Set the inputs
  impl_.setInputCloud(pcl_cloud);
  impl_.setIndices(indices);
  impl_.setSearchSurface(pcl_surface);

  // Compute the normals
  pcl::PointCloud<pcl::Normal> normals;
  impl_.compute(normals);

  // Convert to a ROS message for publishing
  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(normals, output);

  // Enforce that the TF frame and the timestamp are copied
  output.header = cloud->header;

  // Publish
  pub_output_->publish(output);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_ros::NormalEstimation)
