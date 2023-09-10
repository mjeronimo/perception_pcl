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
#include "xmlrpc.h"

using pcl_conversions::fromPCL;

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::SACSegmentationFromNormals::onInit()
{
  // Advertise the output topics
  pub_indices_ = create_publisher<PointIndices>("inliers", max_queue_size_);
  pub_model_ = create_publisher<ModelCoefficients>("model", max_queue_size_);

  // ---[ Mandatory parameters
  int model_type;
  if (!pnh_->getParam("model_type", model_type)) {
    RCLCPP_ERROR(
      get_logger(),
      "[onInit] Need a 'model_type' parameter to be set before continuing!");
    return;
  }
  double threshold;  // unused - set via dynamic reconfigure in the callback
  if (!pnh_->getParam("distance_threshold", threshold)) {
    RCLCPP_ERROR(
      get_logger(),
      "[onInit] Need a 'distance_threshold' parameter to be set before continuing!");
    return;
  }

  // ---[ Optional parameters
  int method_type = 0;
  pnh_->getParam("method_type", method_type);

  XmlRpc::XmlRpcValue axis_param;
  pnh_->getParam("axis", axis_param);
  Eigen::Vector3f axis = Eigen::Vector3f::Zero();

  switch (axis_param.getType()) {
    case XmlRpc::XmlRpcValue::TypeArray:
      {
        if (axis_param.size() != 3) {
          RCLCPP_ERROR(
            get_logger(),
            "[onInit] Parameter 'axis' given but with a different number of values (%d) than "
            "required (3)!",
            axis_param.size());
          return;
        }
        for (int i = 0; i < 3; ++i) {
          if (axis_param[i].getType() != XmlRpc::XmlRpcValue::TypeDouble) {
            RCLCPP_ERROR(
              get_logger(),
              "[onInit] Need floating point values for 'axis' parameter.");
            return;
          }
          double value = axis_param[i]; axis[i] = value;
        }
        break;
      }
    default:
      {
        break;
      }
  }

  // Initialize the random number generator
  srand(time(0));

  RCLCPP_DEBUG(
    get_logger(),
    "[onInit] Nodelet successfully created with the following parameters:\n"
    " - model_type               : %d\n"
    " - method_type              : %d\n"
    " - model_threshold          : %f\n"
    " - axis                     : [%f, %f, %f]\n",
    model_type, method_type, threshold,
    axis[0], axis[1], axis[2]);

  // Set given parameters here
  impl_.setModelType(model_type);
  impl_.setMethodType(method_type);
  impl_.setAxis(axis);

  onInitPostProcess();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::SACSegmentationFromNormals::subscribe()
{
  // Subscribe to the input and normals using filters
  sub_input_filter_.subscribe(this, "input", max_queue_size_);
  sub_normals_filter_.subscribe(this, "normals", max_queue_size_);

  // Subscribe to an axis direction along which the model search is to be constrained (the first
  // 3 model coefficients will be checked)
  sub_axis_ = create_subscription("axis", 1, &SACSegmentationFromNormals::axis_callback, this);

  if (approximate_sync_) {
    sync_input_normals_indices_a_ =
      boost::make_shared<message_filters::Synchronizer<
          sync_policies::ApproximateTime<PointCloud, PointCloudN, PointIndices>>>(max_queue_size_);
  } else {
    sync_input_normals_indices_e_ =
      boost::make_shared<message_filters::Synchronizer<
          sync_policies::ExactTime<PointCloud, PointCloudN, PointIndices>>>(max_queue_size_);
  }

  // If we're supposed to look for PointIndices (indices)
  if (use_indices_) {
    // Subscribe to the input using a filter
    sub_indices_filter_.subscribe(this, "indices", max_queue_size_);

    if (approximate_sync_) {
      sync_input_normals_indices_a_->connectInput(
        sub_input_filter_, sub_normals_filter_,
        sub_indices_filter_);
    } else {
      sync_input_normals_indices_e_->connectInput(
        sub_input_filter_, sub_normals_filter_,
        sub_indices_filter_);
    }
  } else {
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
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::SACSegmentationFromNormals::unsubscribe()
{
  sub_input_filter_.unsubscribe();
  sub_normals_filter_.unsubscribe();

  sub_axis_.shutdown();

  if (use_indices_) {
    sub_indices_filter_.unsubscribe();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::SACSegmentationFromNormals::axis_callback(
  const pcl_msgs::msg::ModelCoefficientsConstPtr & model)
{
  boost::mutex::scoped_lock lock(mutex_);

  if (model->values.size() < 3) {
    RCLCPP_ERROR(
      get_logger(),
      "[axis_callback] Invalid axis direction / model coefficients with %zu values sent on %s!",
      model->values.size(), pnh_->resolveName("axis").c_str());
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
void
pcl_ros::SACSegmentationFromNormals::config_callback(
  SACSegmentationFromNormalsConfig & config,
  uint32_t level)
{
  boost::mutex::scoped_lock lock(mutex_);

  if (impl_.getDistanceThreshold() != config.distance_threshold) {
    impl_.setDistanceThreshold(config.distance_threshold);
    RCLCPP_DEBUG(
      get_logger(),
      "[config_callback] Setting distance to model threshold to: %f.",
      config.distance_threshold);
  }
  // The maximum allowed difference between the model normal and the given axis _in radians_
  if (impl_.getEpsAngle() != config.eps_angle) {
    impl_.setEpsAngle(config.eps_angle);
    RCLCPP_DEBUG(
      get_logger(),
      "[config_callback] Setting new epsilon angle to model threshold to: %f (%f degrees).",
      config.eps_angle, config.eps_angle * 180.0 / M_PI);
  }

  if (impl_.getMaxIterations() != config.max_iterations) {
    impl_.setMaxIterations(config.max_iterations);
    RCLCPP_DEBUG(
      get_logger(),
      "[config_callback] Setting new maximum number of iterations to: %d.",
      config.max_iterations);
  }

  // Number of inliers
  if (min_inliers_ != config.min_inliers) {
    min_inliers_ = config.min_inliers;
    RCLCPP_DEBUG(
      get_logger(),
      "[config_callback] Setting new minimum number of inliers to: %d.",
      min_inliers_);
  }


  if (impl_.getProbability() != config.probability) {
    impl_.setProbability(config.probability);
    RCLCPP_DEBUG(
      get_logger(),
      "[config_callback] Setting new probability to: %f.",
      config.probability);
  }

  if (impl_.getOptimizeCoefficients() != config.optimize_coefficients) {
    impl_.setOptimizeCoefficients(config.optimize_coefficients);
    RCLCPP_DEBUG(
      get_logger(),
      "[config_callback] Setting coefficient optimization to: %s.",
      (config.optimize_coefficients) ? "true" : "false");
  }

  if (impl_.getNormalDistanceWeight() != config.normal_distance_weight) {
    impl_.setNormalDistanceWeight(config.normal_distance_weight);
    RCLCPP_DEBUG(
      get_logger(),
      "[config_callback] Setting new distance weight to: %f.",
      config.normal_distance_weight);
  }

  double radius_min, radius_max;
  impl_.getRadiusLimits(radius_min, radius_max);
  if (radius_min != config.radius_min) {
    radius_min = config.radius_min;
    RCLCPP_DEBUG(
      get_logger(),
      "[config_callback] Setting minimum allowable model radius to: %f.",
      radius_min);
    impl_.setRadiusLimits(radius_min, radius_max);
  }
  if (radius_max != config.radius_max) {
    radius_max = config.radius_max;
    RCLCPP_DEBUG(
      get_logger(),
      "[config_callback] Setting maximum allowable model radius to: %f.",
      radius_max);
    impl_.setRadiusLimits(radius_min, radius_max);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::SACSegmentationFromNormals::input_normals_indices_callback(
  const PointCloudConstPtr & cloud,
  const PointCloudNConstPtr & cloud_normals,
  const PointIndicesConstPtr & indices
)
{
  boost::mutex::scoped_lock lock(mutex_);

  PointIndices inliers;
  ModelCoefficients model;
  // Enforce that the TF frame and the timestamp are copied
  inliers.header = model.header = fromPCL(cloud->header);

  if (impl_.getModelType() < 0) {
    RCLCPP_ERROR(get_logger(), "[input_normals_indices_callback] Model type not set!");
    pub_indices_.publish(boost::make_shared<const PointIndices>(inliers));
    pub_model_.publish(boost::make_shared<const ModelCoefficients>(model));
    return;
  }

  if (!isValid(cloud)) {  // || !isValid (cloud_normals, "normals"))
    RCLCPP_ERROR(get_logger(), "[input_normals_indices_callback] Invalid input!");
    pub_indices_.publish(boost::make_shared<const PointIndices>(inliers));
    pub_model_.publish(boost::make_shared<const ModelCoefficients>(model));
    return;
  }
  // If indices are given, check if they are valid
  if (indices && !isValid(indices)) {
    RCLCPP_ERROR(get_logger(), "[input_normals_indices_callback] Invalid indices!");
    pub_indices_.publish(boost::make_shared<const PointIndices>(inliers));
    pub_model_.publish(boost::make_shared<const ModelCoefficients>(model));
    return;
  }

  /// DEBUG
  if (indices && !indices->header.frame_id.empty()) {
    RCLCPP_DEBUG(
      get_logger(),
      "[input_normals_indices_callback]\n"
      "                                 - PointCloud with %d data points (%s), stamp %f, and "
      "frame %s on topic %s received.\n"
      "                                 - PointCloud with %d data points (%s), stamp %f, and "
      "frame %s on topic %s received.\n"
      "                                 - PointIndices with %zu values, stamp %f, and "
      "frame %s on topic %s received.",
      cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str(), fromPCL(
        cloud->header).stamp.toSec(), cloud->header.frame_id.c_str(), pnh_->resolveName(
        "input").c_str(),
      cloud_normals->width * cloud_normals->height, pcl::getFieldsList(
        *cloud_normals).c_str(), fromPCL(
        cloud_normals->header).stamp.toSec(),
      cloud_normals->header.frame_id.c_str(), pnh_->resolveName("normals").c_str(),
      indices->indices.size(), indices->header.stamp.toSec(),
      indices->header.frame_id.c_str(), pnh_->resolveName("indices").c_str());
  } else {
    RCLCPP_DEBUG(
      get_logger(),
      "[input_normals_indices_callback]\n"
      "                                 - PointCloud with %d data points (%s), stamp %f, and "
      "frame %s on topic %s received.\n"
      "                                 - PointCloud with %d data points (%s), stamp %f, and "
      "frame %s on topic %s received.",
      cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str(), fromPCL(
        cloud->header).stamp.toSec(), cloud->header.frame_id.c_str(), pnh_->resolveName(
        "input").c_str(),
      cloud_normals->width * cloud_normals->height, pcl::getFieldsList(
        *cloud_normals).c_str(), fromPCL(
        cloud_normals->header).stamp.toSec(),
      cloud_normals->header.frame_id.c_str(), pnh_->resolveName("normals").c_str());
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
    pub_indices_.publish(boost::make_shared<const PointIndices>(inliers));
    pub_model_.publish(boost::make_shared<const ModelCoefficients>(model));
    return;
  }

  impl_.setInputCloud(pcl_ptr(cloud));
  impl_.setInputNormals(pcl_ptr(cloud_normals));

  IndicesPtr indices_ptr;
  if (indices && !indices->header.frame_id.empty()) {
    indices_ptr.reset(new std::vector<int>(indices->indices));
  }

  impl_.setIndices(indices_ptr);

  // Final check if the data is empty
  // (remember that indices are set to the size of the data -- if indices* = NULL)
  if (!cloud->points.empty()) {
    pcl::PointIndices pcl_inliers;
    pcl::ModelCoefficients pcl_model;
    pcl_conversions::moveToPCL(inliers, pcl_inliers);
    pcl_conversions::moveToPCL(model, pcl_model);
    impl_.segment(pcl_inliers, pcl_model);
    pcl_conversions::moveFromPCL(pcl_inliers, inliers);
    pcl_conversions::moveFromPCL(pcl_model, model);
  }

  // Check if we have enough inliers, clear inliers + model if not
  if (static_cast<int>(inliers.indices.size()) <= min_inliers_) {
    inliers.indices.clear();
    model.values.clear();
  }

  // Publish
  pub_indices_.publish(boost::make_shared<const PointIndices>(inliers));
  pub_model_.publish(boost::make_shared<const ModelCoefficients>(model));
  RCLCPP_DEBUG(
    get_logger(),
    "[input_normals_callback] Published PointIndices with %zu values on topic %s, and "
    "ModelCoefficients with %zu values on topic %s",
    inliers.indices.size(), pnh_->resolveName("inliers").c_str(),
    model.values.size(), pnh_->resolveName("model").c_str());
  if (inliers.indices.empty()) {
    RCLCPP_WARN(get_logger(), "[input_indices_callback] No inliers found!");
  }
}

// typedef pcl_ros::SACSegmentation SACSegmentation;
// typedef pcl_ros::SACSegmentationFromNormals SACSegmentationFromNormals;
// PLUGINLIB_EXPORT_CLASS(SACSegmentation, nodelet::Nodelet)
// PLUGINLIB_EXPORT_CLASS(SACSegmentationFromNormals, nodelet::Nodelet)
