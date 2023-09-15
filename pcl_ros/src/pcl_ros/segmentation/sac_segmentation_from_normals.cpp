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
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: sac_segmentation.hpp 33195 2010-10-10 14:12:19Z marton $
 *
 */

#include "pcl_ros/segmentation/sac_segmentation_from_normals.hpp"

#include <vector>

#include "pcl/common/io.h"
#include "pcl_conversions/pcl_conversions.h"

using pcl_conversions::fromPCL;

//////////////////////////////////////////////////////////////////////////////////////////////
pcl_ros::SACSegmentationFromNormals::SACSegmentationFromNormals(const rclcpp::NodeOptions & options)
  : SACSegmentation(options)
{
  init_parameters();
  subscribe();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::SACSegmentationFromNormals::init_parameters()
{
  add_parameter(
    "normal_distance_weight",
    rclcpp::ParameterValue(normal_distance_weight_),
    floating_point_range{0.0, 1.0, 0.0}, // from, to, step
    "The relative weight (between 0 and 1) to give to the angular distance (0 to pi/2) between point normals and the plane normal");

  double normal_distance_weight_ = get_parameter("normal_distance_weight").as_double();

  impl_.setNormalDistanceWeight(normal_distance_weight_);

  // Initialize the parameter callback
  set_parameters_callback_handle_ = add_on_set_parameters_callback(std::bind(&SACSegmentationFromNormals::set_parameters_callback, this, std::placeholders::_1));

  RCLCPP_DEBUG(
    get_logger(),
    "[init_parameters] Node initialized with the following parameters:\n"
    " - normal_distance_weight : %f\n",
    normal_distance_weight_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::SACSegmentationFromNormals::subscribe()
{
  // Subscribe to the input and normals using filters
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
  custom_qos_profile.depth = max_queue_size_;
  sub_input_filter_.subscribe(this, "input", custom_qos_profile);
  sub_normals_filter_.subscribe(this, "normals", custom_qos_profile);

  // Subscribe to an axis direction along which the model search is to be constrained (the first
  // 3 model coefficients will be checked)
  sub_axis_ = 
    create_subscription<pcl_msgs::msg::ModelCoefficients>("axis", 1, 
      std::bind(&pcl_ros::SACSegmentationFromNormals::axis_callback, this, std::placeholders::_1));

  if (approximate_sync_) {
    sync_input_normals_indices_a_ =
      boost::make_shared<message_filters::Synchronizer<
          sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, PointCloudN, pcl_msgs::msg::PointIndices>>>(max_queue_size_);
  } else {
    sync_input_normals_indices_e_ =
      boost::make_shared<message_filters::Synchronizer<
          sync_policies::ExactTime<sensor_msgs::msg::PointCloud2, PointCloudN, pcl_msgs::msg::PointIndices>>>(max_queue_size_);
  }

  // If we're supposed to look for PointIndices (indices)
  if (use_indices_) {
    // Subscribe to the input using a filter
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
    custom_qos_profile.depth = max_queue_size_;
    sub_indices_filter_.subscribe(this, "indices", custom_qos_profile);

    if (approximate_sync_) {
      sync_input_normals_indices_a_->connectInput(sub_input_filter_, sub_normals_filter_, sub_indices_filter_);
    } else {
      sync_input_normals_indices_e_->connectInput(sub_input_filter_, sub_normals_filter_, sub_indices_filter_);
    }
  } else {
#if 0
    // Create a different callback for copying over the timestamp to fake indices
    sub_input_filter_.registerCallback(bind(&SACSegmentationFromNormals::input_callback, this, _1));

    if (approximate_sync_) {
      sync_input_normals_indices_a_->connectInput(sub_input_filter_, sub_normals_filter_, nf_);
    } else {
      sync_input_normals_indices_e_->connectInput(sub_input_filter_, sub_normals_filter_, nf_);
    }
  }

  if (approximate_sync_) {
    sync_input_normals_indices_a_->registerCallback(
      bind(
        &SACSegmentationFromNormals::
        input_normals_indices_callback, this, _1, _2, _3));
  } else {
    sync_input_normals_indices_e_->registerCallback(
      bind(
        &SACSegmentationFromNormals::
        input_normals_indices_callback, this, _1, _2, _3));
#endif
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::SACSegmentationFromNormals::unsubscribe()
{
  sub_input_filter_.unsubscribe();
  sub_normals_filter_.unsubscribe();

  sub_axis_.reset();

  if (use_indices_) {
    sub_indices_filter_.unsubscribe();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::SACSegmentationFromNormals::axis_callback(
  const pcl_msgs::msg::ModelCoefficients::ConstSharedPtr & model)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (model->values.size() < 3) {
    RCLCPP_ERROR(
      get_logger(),
      "[axis_callback] Invalid axis direction / model coefficients with %zu values sent on %s!",
      model->values.size(), "axis");
    return;
  }

  RCLCPP_DEBUG(
    get_logger(),
    "[axis_callback] Received axis direction: %f %f %f",
    model->values[0], model->values[1], model->values[2]);

  Eigen::Vector3f axis(model->values[0], model->values[1], model->values[2]);
  impl_.setAxis(axis);
}

//////////////////////////////////////////////////////////////////////////////////////////////
rcl_interfaces::msg::SetParametersResult
pcl_ros::SACSegmentationFromNormals::set_parameters_callback(
  const std::vector<rclcpp::Parameter> & params)
{
  std::lock_guard<std::mutex> lock(mutex_);

  for (const rclcpp::Parameter & param : params) {
    if (param.get_name() == "normal_distance_weight") {
      double new_normal_distance_weight = param.as_double();
      if (normal_distance_weight_ != new_normal_distance_weight) {
        normal_distance_weight_ = new_normal_distance_weight;
        RCLCPP_DEBUG(
          get_logger(),
          "[set_parameters_callback] Setting new distance weight to: %f.",
          normal_distance_weight_);
      impl_.setNormalDistanceWeight(normal_distance_weight_);
      }
    }
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::SACSegmentationFromNormals::input_normals_indices_callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
  const PointCloudNConstPtr & cloud_normals,
  const pcl_msgs::msg::PointIndices::ConstSharedPtr & indices
)
{
  std::lock_guard<std::mutex> lock(mutex_);

  pcl_msgs::msg::PointIndices inliers;
  pcl_msgs::msg::ModelCoefficients model;

  // Enforce that the TF frame and the timestamp are copied
  inliers.header = model.header = cloud->header;

  if (model_type_ < 0) {
    RCLCPP_ERROR(get_logger(), "[input_normals_indices_callback] Model type not set!");
    pub_indices_->publish(inliers);
    pub_model_->publish(model);
    return;
  }

  if (!isValid(cloud)) {  // || !isValid (cloud_normals, "normals"))
    RCLCPP_ERROR(get_logger(), "[input_normals_indices_callback] Invalid input!");
    pub_indices_->publish(inliers);
    pub_model_->publish(model);
    return;
  }

  // If indices are given, check if they are valid
  if (indices && !isValid(indices)) {
    RCLCPP_ERROR(get_logger(), "[input_normals_indices_callback] Invalid indices!");
    pub_indices_->publish(inliers);
    pub_model_->publish(model);
    return;
  }

  /// DEBUG
  if (indices && !indices->header.frame_id.empty()) {
#if 0
    RCLCPP_DEBUG(
      get_logger(),
      "[input_normals_indices_callback]\n"
      "  - PointCloud with %d data points (%s), stamp %d.%09d, and frame %s on topic %s received.\n"
      "  - PointCloud with %d data points (%s), stamp %d.%09d, and frame %s on topic %s received.\n"
      "  - PointIndices with %zu values, stamp %d.%09d, and frame %s on topic %s received.",
      cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str(), 
      cloud->header.stamp.sec, cloud->header.stamp.nanosec,
      cloud->header.frame_id.c_str(), "input",
      cloud_normals->width * cloud_normals->height, pcl::getFieldsList(*cloud_normals).c_str(), 
      cloud_normals->header.stamp.sec, cloud_normals->header.stamp.nanosec,
      cloud_normals->header.frame_id.c_str(), "normals",
      indices->indices.size(), 
      indices->header.stamp.sec, indices->header.stamp.nanosec,
      indices->header.frame_id.c_str(), "indices");
#endif
  } else {
#if 0
    RCLCPP_DEBUG(
      get_logger(),
      "[input_normals_indices_callback]\n"
      "  - PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.\n"
      "  - PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.",
      cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str(), 
      cloud->header.stamp.toSec(),
      cloud->header.frame_id.c_str(), "input",
      cloud_normals->width * cloud_normals->height, pcl::getFieldsList(*cloud_normals).c_str(),
      cloud_normals->header).stamp.toSec(),
      cloud_normals->header.frame_id.c_str(), "normals");
#endif
  }
  ///

  // Extra checks for safety
  int cloud_nr_points = cloud->width * cloud->height;
  int cloud_normals_nr_points = cloud_normals->width * cloud_normals->height;

  if (cloud_nr_points != cloud_normals_nr_points) {
    RCLCPP_ERROR(
      get_logger(),
      "[input_normals_indices_callback] Number of points in the input dataset (%d) differs "
      "from the number of points in the normals (%d)!",
      cloud_nr_points, cloud_normals_nr_points);
    pub_indices_->publish(inliers);
    pub_model_->publish(model);
    return;
  }

  // TODO(mjeronimo)
  // impl_.setInputCloud(pcl_ptr(cloud));
  // impl_.setInputNormals(pcl_ptr(cloud_normals));

  IndicesPtr indices_ptr;
  if (indices && !indices->header.frame_id.empty()) {
    indices_ptr.reset(new std::vector<int>(indices->indices));
  }

  impl_.setIndices(indices_ptr);

  // Final check if the data is empty
  // (remember that indices are set to the size of the data -- if indices* = NULL)
#if 0
  if (!cloud->points.empty()) {
    pcl::PointIndices pcl_inliers;
    pcl::ModelCoefficients pcl_model;
    pcl_conversions::moveToPCL(inliers, pcl_inliers);
    pcl_conversions::moveToPCL(model, pcl_model);
    impl_.segment(pcl_inliers, pcl_model);
    pcl_conversions::moveFromPCL(pcl_inliers, inliers);
    pcl_conversions::moveFromPCL(pcl_model, model);
  }
#endif

  // Check if we have enough inliers, clear inliers + model if not
  if (static_cast<int>(inliers.indices.size()) <= min_inliers_) {
    inliers.indices.clear();
    model.values.clear();
  }

  // Publish
  pub_indices_->publish(inliers);
  pub_model_->publish(model);

  RCLCPP_DEBUG(
    get_logger(),
    "[input_normals_callback] Published PointIndices with %zu values on topic %s, and "
    "ModelCoefficients with %zu values on topic %s",
    inliers.indices.size(), "inliers",
    model.values.size(), "model");

  if (inliers.indices.empty()) {
    RCLCPP_WARN(get_logger(), "[input_indices_callback] No inliers found!");
  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_ros::SACSegmentationFromNormals)

