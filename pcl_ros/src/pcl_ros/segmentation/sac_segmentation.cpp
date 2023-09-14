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

#include "pcl_ros/segmentation/sac_segmentation.hpp"

#include <vector>

#include "pcl/common/io.h"
#include "pcl_conversions/pcl_conversions.h"
#include "xmlrpc.h"

using pcl_conversions::fromPCL;

//////////////////////////////////////////////////////////////////////////////////////////////
pcl_ros::SACSegmentation::SACSegmentation(const rclcpp::NodeOptions & options)
  : PCLNode("SACSegmentationNode", options)
{
  init_parameters();
  subscribe();

  // Create the publishers
  pub_indices_ = create_publisher<PointIndices>("inliers", max_queue_size_);
  pub_model_ = create_publisher<ModelCoefficients>("model", max_queue_size_);

  // Initialize the random number generator
  srand(time(0));
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::SACSegmentation::init_parameters()
{
#if 0
    # add(self, name, paramtype, level, description, default = None, min = None,
    # max = None, edit_method = '')
    gen.add('max_iterations', int_t, 0,
            'The maximum number of iterations the algorithm will run for',
            50, 0, 100000)
    gen.add('probability', double_t, 0,
            'The desired probability of choosing at least one sample free from outliers',
            0.99, 0.5, 0.99)
    gen.add('distance_threshold', double_t, 0,
            'The distance to model threshold',
            0.02, 0, 1.0)
    gen.add('optimize_coefficients', bool_t, 0,
            'Model coefficient refinement',
            True)
    gen.add('radius_min', double_t, 0,
            'The minimum allowed model radius (where applicable)',
            0.0, 0, 1.0)
    gen.add('radius_max', double_t, 0,
            'The maximum allowed model radius (where applicable)',
            0.05, 0, 1.0)
    gen.add('eps_angle', double_t, 0,
            ('The maximum allowed difference between the model normal '
             'and the given axis in radians.'),
            0.17, 0.0, 1.5707)
    gen.add('min_inliers', int_t, 0,
            'The minimum number of inliers a model must have in order to be considered valid.',
            0, 0, 100000)
    gen.add('input_frame', str_t, 0,
            ('The input TF frame the data should be transformed into, '
             'if input.header.frame_id is different.'),
            '')
    gen.add('output_frame', str_t, 0,
            ('The output TF frame the data should be transformed into, '
             'if input.header.frame_id is different.'),
            '')
#endif

  // ---[ Mandatory parameters
  int model_type;
  if (get_parameter("model_type", model_type)) {
    RCLCPP_ERROR(get_logger(), "[init_parameters] Need a 'model_type' parameter to be set before continuing!");
    return;
  }
  double threshold;  // unused - set via dynamic reconfigure in the callback
  if (get_parameter("distance_threshold", threshold)) {
    RCLCPP_ERROR(get_logger(), "[init_parameters] Need a 'distance_threshold' parameter to be set before continuing!");
    return;
  }

  // ---[ Optional parameters
  int method_type = 0;
  get_parameter("method_type", method_type);

#if 1
  std::vector<double> axis_params; //  = double_array_param.as_double_array();
  get_parameter("axis", axis_params);
  Eigen::Vector3f axis = Eigen::Vector3f::Zero();
#else
  XmlRpc::XmlRpcValue axis_param;
  get_parameter("axis", axis_param);

  switch (axis_param.getType()) {
    case XmlRpc::XmlRpcValue::TypeArray:
      {
        if (axis_param.size() != 3) {
          RCLCPP_ERROR(
            get_logger(),
            "[init_parameters] Parameter 'axis' given but with a different number of values (%d) "
            "than required (3)!",
            axis_param.size());
          return;
        }
        for (int i = 0; i < 3; ++i) {
          if (axis_param[i].getType() != XmlRpc::XmlRpcValue::TypeDouble) {
            RCLCPP_ERROR(
              get_logger(),
              "[init_parameters] Need floating point values for 'axis' parameter.");
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
#endif

  // Set given parameters here
  impl_.setModelType(model_type);
  impl_.setMethodType(method_type);
  impl_.setAxis(axis);

   // Initialize the parameter callback
  set_parameters_callback_handle_ = add_on_set_parameters_callback(std::bind(&SACSegmentation::set_parameters_callback, this, std::placeholders::_1));

  RCLCPP_DEBUG(
    get_logger(),
    "[init_parameters] Node initialized with the following parameters:\n"
    " - model_type : %d\n"
    " - method_type : %d\n"
    " - model_threshold : %f\n"
    " - axis : [%f, %f, %f]\n",
    model_type, method_type, threshold,
    axis[0], axis[1], axis[2]);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::SACSegmentation::subscribe()
{
  // If we're supposed to look for PointIndices (indices)
  if (use_indices_) {
    // Subscribe to the input using a filter
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
    custom_qos_profile.depth = max_queue_size_;
    sub_input_filter_.subscribe(this, "input", custom_qos_profile);
    sub_indices_filter_.subscribe(this, "indices", custom_qos_profile);

    // when "use_indices" is set to true, and "latched_indices" is set to true,
    // we'll subscribe and get a separate callback for PointIndices that will
    // save the indices internally, and a PointCloud + PointIndices callback
    // will take care of meshing the new PointClouds with the old saved indices.
    if (latched_indices_) {
      // Subscribe to a callback that saves the indices
      sub_indices_filter_.registerCallback(bind(&SACSegmentation::indices_callback, this, std::placeholders::_1));
      // Subscribe to a callback that sets the header of the saved indices to the cloud header
      sub_input_filter_.registerCallback(bind(&SACSegmentation::input_callback, this, std::placeholders::_1));

      // Synchronize the two topics. No need for an approximate synchronizer here, as we'll
      // match the timestamps exactly
      sync_input_indices_e_ =
        boost::make_shared<message_filters::Synchronizer<
            sync_policies::ExactTime<sensor_msgs::msg::PointCloud2, pcl_msgs::msg::PointIndices>>>(max_queue_size_);
      sync_input_indices_e_->connectInput(sub_input_filter_, nf_pi_);
      sync_input_indices_e_->registerCallback(
        bind(
          &SACSegmentation::input_indices_callback, this,
          std::placeholders::_1, std::placeholders::_2));
    } else {  // "latched_indices" not set, proceed with regular <input,indices> pairs
      if (approximate_sync_) {
        sync_input_indices_a_ =
          boost::make_shared<message_filters::Synchronizer<
              sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, pcl_msgs::msg::PointIndices>>>(max_queue_size_);
        sync_input_indices_a_->connectInput(sub_input_filter_, sub_indices_filter_);
        sync_input_indices_a_->registerCallback(
          bind(
            &SACSegmentation::input_indices_callback, this,
            std::placeholders::_1, std::placeholders::_2));
      } else {
        sync_input_indices_e_ =
          boost::make_shared<message_filters::Synchronizer<
              sync_policies::ExactTime<sensor_msgs::msg::PointCloud2, pcl_msgs::msg::PointIndices>>>(max_queue_size_);
        sync_input_indices_e_->connectInput(sub_input_filter_, sub_indices_filter_);
        sync_input_indices_e_->registerCallback(
          bind(
            &SACSegmentation::input_indices_callback, this,
            std::placeholders::_1, std::placeholders::_2));
      }
    }
  } else {
    // Subscribe in an old fashion to input only (no filters)
#if 0
    // TODO(mjeronimo)
    sub_input_ =
      create_subscription<sensor_msgs::msg::PointCloud2>("input", max_queue_size_,
        std::bind(&pcl_ros::SACSegmentation::input_indices_callback, this, std::placeholders::_1, pcl_msgs::msg::PointIndices::ConstSharedPtr()));
#endif
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::SACSegmentation::unsubscribe()
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
pcl_ros::SACSegmentation::set_parameters_callback(const std::vector<rclcpp::Parameter> & /*params*/)
{
  std::lock_guard<std::mutex> lock(mutex_);

#if 0
  if (impl_.getDistanceThreshold() != config.distance_threshold) {
    // sac_->setDistanceThreshold (threshold_); - done in initSAC
    impl_.setDistanceThreshold(config.distance_threshold);
    RCLCPP_DEBUG(
      get_logger(),
      "[set_parameters_callback] Setting new distance to model threshold to: %f.",
      config.distance_threshold);
  }
  // The maximum allowed difference between the model normal and the given axis _in radians_
  if (impl_.getEpsAngle() != config.eps_angle) {
    impl_.setEpsAngle(config.eps_angle);
    RCLCPP_DEBUG(
      get_logger(),
      "[set_parameters_callback] Setting new epsilon angle to model threshold to: %f (%f degrees).",
      config.eps_angle, config.eps_angle * 180.0 / M_PI);
  }

  // Number of inliers
  if (min_inliers_ != config.min_inliers) {
    min_inliers_ = config.min_inliers;
    RCLCPP_DEBUG(
      get_logger(),
      "[set_parameters_callback] Setting new minimum number of inliers to: %d.",
      min_inliers_);
  }

  if (impl_.getMaxIterations() != config.max_iterations) {
    // sac_->setMaxIterations (max_iterations_); - done in initSAC
    impl_.setMaxIterations(config.max_iterations);
    RCLCPP_DEBUG(
      get_logger(),
      "[set_parameters_callback] Setting new maximum number of iterations to: %d.",
      config.max_iterations);
  }
  if (impl_.getProbability() != config.probability) {
    // sac_->setProbability (probability_); - done in initSAC
    impl_.setProbability(config.probability);
    RCLCPP_DEBUG(
      get_logger(),
      "[set_parameters_callback] Setting new probability to: %f.",
      config.probability);
  }
  if (impl_.getOptimizeCoefficients() != config.optimize_coefficients) {
    impl_.setOptimizeCoefficients(config.optimize_coefficients);
    RCLCPP_DEBUG(
      get_logger(),
      "[set_parameters_callback] Setting coefficient optimization to: %s.",
      (config.optimize_coefficients) ? "true" : "false");
  }

  double radius_min, radius_max;
  impl_.getRadiusLimits(radius_min, radius_max);
  if (radius_min != config.radius_min) {
    radius_min = config.radius_min;
    RCLCPP_DEBUG(get_logger(), "[set_parameters_callback] Setting minimum allowable model radius to: %f.", radius_min);
    impl_.setRadiusLimits(radius_min, radius_max);
  }
  if (radius_max != config.radius_max) {
    radius_max = config.radius_max;
    RCLCPP_DEBUG(get_logger(), "[set_parameters_callback] Setting maximum allowable model radius to: %f.", radius_max);
    impl_.setRadiusLimits(radius_min, radius_max);
  }

  if (tf_input_frame_ != config.input_frame) {
    tf_input_frame_ = config.input_frame;
    RCLCPP_DEBUG(get_logger(), "[set_parameters_callback] Setting the input TF frame to: %s.", tf_input_frame_.c_str());
    RCLCPP_WARN(get_logger(), "input_frame TF not implemented yet!");
  }
  if (tf_output_frame_ != config.output_frame) {
    tf_output_frame_ = config.output_frame;
    RCLCPP_DEBUG(
      get_logger(),
      "[set_parameters_callback] Setting the output TF frame to: %s.",
      tf_output_frame_.c_str());
    RCLCPP_WARN(get_logger(), "output_frame TF not implemented yet!");
  }
#endif

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::SACSegmentation::input_indices_callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
  const pcl_msgs::msg::PointIndices::ConstSharedPtr & indices)
{
  std::lock_guard<std::mutex> lock(mutex_);

  pcl_msgs::msg::PointIndices inliers;
  pcl_msgs::msg::ModelCoefficients model;
  // Enforce that the TF frame and the timestamp are copied
  inliers.header = model.header = cloud->header;

  // If cloud is given, check if it's valid
  if (!isValid(cloud)) {
    RCLCPP_ERROR(get_logger(), "[input_indices_callback] Invalid input!");
    pub_indices_->publish(inliers);
    pub_model_->publish(model);
    return;
  }
  // If indices are given, check if they are valid
  if (indices && !isValid(indices)) {
    RCLCPP_ERROR(get_logger(), "[input_indices_callback] Invalid indices!");
    pub_indices_->publish(inliers);
    pub_model_->publish(model);
    return;
  }

  /// DEBUG
  if (indices && !indices->header.frame_id.empty()) {
    RCLCPP_DEBUG(
      get_logger(),
      "[input_indices_callback]\n"
      "  - PointCloud with %d data points (%s), stamp %d.%09d, and frame %s on topic %s received.\n"
      "  - PointIndices with %zu values, stamp %d.%09d, and frame %s on topic %s received.",
      cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str(), 
      cloud->header.stamp.sec, cloud->header.stamp.nanosec,
      cloud->header.frame_id.c_str(), "input",
      indices->indices.size(), 
      indices->header.stamp.sec, indices->header.stamp.nanosec,
      indices->header.frame_id.c_str(), "indices");
  } else {
    RCLCPP_DEBUG(
      get_logger(),
      "[input_indices_callback] PointCloud with %d data points, stamp %d.%09d, and frame %s on topic %s received.",
      cloud->width * cloud->height, 
      cloud->header.stamp.sec, cloud->header.stamp.nanosec,
      cloud->header.frame_id.c_str(), "input");
  }
  ///

  // Check whether the user has given a different input TF frame
  tf_input_orig_frame_ = cloud->header.frame_id;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_tf;
/*  if (!tf_input_frame_.empty () && cloud->header.frame_id != tf_input_frame_)
  {
    RCLCPP_DEBUG ("[input_callback] Transforming input dataset from %s to %s.",
    // cloud->header.frame_id.c_str (), tf_input_frame_.c_str ());
    // Save the original frame ID
    // Convert the cloud into the different frame
    PointCloud cloud_transformed;
    if (!pcl::transformPointCloud (tf_input_frame_, cloud->header.stamp, *cloud,
      cloud_transformed, tf_listener_))
      return;
    cloud_tf.reset (new PointCloud (cloud_transformed));
  }
  else*/
  cloud_tf = cloud;

  IndicesPtr indices_ptr;
  if (indices && !indices->header.frame_id.empty()) {
    indices_ptr.reset(new std::vector<int>(indices->indices));
  }

  // TODO(mjeronimo)
  //impl_.setInputCloud(pcl_ptr(cloud_tf));
  impl_.setIndices(indices_ptr);

  // Final check if the data is empty
  // (remember that indices are set to the size of the data -- if indices* = NULL)
  if (cloud->width && cloud->height) {
    pcl::PointIndices pcl_inliers;
    pcl::ModelCoefficients pcl_model;
    pcl_conversions::moveToPCL(inliers, pcl_inliers);
    pcl_conversions::moveToPCL(model, pcl_model);
    impl_.segment(pcl_inliers, pcl_model);
    pcl_conversions::moveFromPCL(pcl_inliers, inliers);
    pcl_conversions::moveFromPCL(pcl_model, model);
  }

  // Probably need to transform the model of the plane here

  // Check if we have enough inliers, clear inliers + model if not
  if (static_cast<int>(inliers.indices.size()) <= min_inliers_) {
    inliers.indices.clear();
    model.values.clear();
  }

  // Publish
  // TODO(mjeronimo)
  // pub_indices_->publish(std::make_shared<const PointIndices>(inliers));
  // pub_model_->publish(std::make_shared<const ModelCoefficients>(model));

  RCLCPP_DEBUG(
    get_logger(),
    "[input_indices_callback] Published PointIndices with %zu values on topic %s, "
    "and ModelCoefficients with %zu values on topic %s",
    inliers.indices.size(), "inliers",
    model.values.size(), "model");

  if (inliers.indices.empty()) {
    RCLCPP_WARN(get_logger(), "[input_indices_callback] No inliers found!");
  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_ros::SACSegmentation)
