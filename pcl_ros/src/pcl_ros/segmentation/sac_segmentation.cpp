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
 * $Id: sac_segmentation.cpp 33195 2010-10-10 14:12:19Z marton $
 *
 */

#include "pcl_ros/segmentation/sac_segmentation.hpp"

#include <pcl/common/io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <vector>

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
void pcl_ros::SACSegmentation::init_parameters()
{
  add_parameter(
    "axis", rclcpp::ParameterValue(axis_),
    "The axis along which we need to search for a model perpendicular to");

  add_parameter(
    "distance_threshold", rclcpp::ParameterValue(distance_threshold_),
    floating_point_range{0.0, 1.0, 0.0},  // from, to, step
    "The distance to model threshold");

  add_parameter(
    "eps_angle", rclcpp::ParameterValue(eps_angle_),
    floating_point_range{0.0, 1.5707, 0.0},  // from, to, step
    "The maximum allowed difference between the model normal and the given axis in "
    "radians");

  add_parameter(
    "input_frame", rclcpp::ParameterValue(input_frame_),
    "The input TF frame the data should be transformed into, if input.header.frame_id "
    "is different");

  add_parameter(
    "latched_indices", rclcpp::ParameterValue(latched_indices_),
    "Whether to latch the indices values");

  add_parameter(
    "max_iterations", rclcpp::ParameterValue(max_iterations_),
    integer_range{0, 100000, 0},  // from, to, step
    "The maximum number of iterations the algorithm will run for");

  add_parameter(
    "method_type", rclcpp::ParameterValue(method_type_),
    "One of the values from pcl/sample_consensus/method_types.h");

  add_parameter(
    "min_inliers", rclcpp::ParameterValue(min_inliers_),
    integer_range{0, 100000, 0},  // from, to, step
    "The minimum number of inliers a model must have in order to be considered valid");

  add_parameter(
    "model_type", rclcpp::ParameterValue(model_type_),
    "One of the values from the SacModel enumeration in "
    "pcl/sample_consensus/model_types.h");

  add_parameter(
    "optimize_coefficients", rclcpp::ParameterValue(optimize_coefficients_),
    "Model coefficient refinement");

  add_parameter(
    "output_frame", rclcpp::ParameterValue(output_frame_),
    "The output TF frame the data should be transformed into, if input.header.frame_id "
    "is different");

  add_parameter(
    "probability", rclcpp::ParameterValue(probability_),
    floating_point_range{0.5, 0.99, 0.0},  // from, to, step
    "The desired probability of choosing at least one sample free from outliers");

  add_parameter(
    "radius_max", rclcpp::ParameterValue(radius_max_),
    floating_point_range{0.0, 1.0, 0.0},  // from, to, step
    "The maximum allowed model radius (where applicable)");

  add_parameter(
    "radius_min", rclcpp::ParameterValue(radius_min_),
    floating_point_range{0.0, 1.0, 0.0},  // from, to, step
    "The minimum allowed model radius (where applicable)");

  // Get the current parameter values
  axis_ = get_parameter("axis").as_double_array();
  distance_threshold_ = get_parameter("distance_threshold").as_double();
  eps_angle_ = get_parameter("eps_angle").as_double();
  input_frame_ = get_parameter("input_frame").as_string();
  latched_indices_ = get_parameter("latched_indices").as_bool();
  max_iterations_ = get_parameter("max_iterations").as_int();
  method_type_ = get_parameter("method_type").as_int();
  min_inliers_ = get_parameter("min_inliers").as_int();
  model_type_ = get_parameter("model_type").as_int();
  optimize_coefficients_ = get_parameter("optimize_coefficients").as_bool();
  output_frame_ = get_parameter("outut_frame").as_string();
  probability_ = get_parameter("probability").as_double();
  radius_max_ = get_parameter("radius_max").as_double();
  radius_min_ = get_parameter("radius_min").as_double();

  // Verify the parameter values
  if (axis_.size() != 3) {
    RCLCPP_ERROR(
      get_logger(),
      "[init_parameters] Parameter 'axis' given but with a different number of values "
      "(%ld) than required (3)!",
      axis_.size());
    return;
  }
  Eigen::Vector3f eigen_axis(axis_[0], axis_[1], axis_[2]);

  // TODO(mjeronimo): verify model_type and method_type

  // Set the parameters on the underlying model
  impl_.setAxis(eigen_axis);
  impl_.setMethodType(method_type_);
  impl_.setModelType(model_type_);

  // Initialize the parameter callback
  set_parameters_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&SACSegmentation::set_parameters_callback, this, std::placeholders::_1));

  RCLCPP_DEBUG(
    get_logger(),
    "[init_parameters] Node initialized with the following parameters:\n"
    " - axis : [%f, %f, %f]\n"
    " - distance_threshold : %f\n"
    " - eps_angle : %f\n"
    " - input_frame : %s\n"
    " - latched_indices : %s\n"
    " - max_iterations : %d\n"
    " - method_type : %d\n"
    " - min_inliers : %d\n"
    " - model_type : %d\n"
    " - optimize_coefficients : %s\n"
    " - ouput_frame : %s\n"
    " - probability : %f\n"
    " - radius_max : %f\n"
    " - radius_min : %f\n",
    axis_[0], axis_[1], axis_[2], distance_threshold_, eps_angle_, input_frame_.c_str(),
    latched_indices_ ? "true" : "false", max_iterations_, method_type_, min_inliers_, model_type_,
    optimize_coefficients_ ? "true" : "false", output_frame_.c_str(), probability_, radius_max_,
    radius_min_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void pcl_ros::SACSegmentation::subscribe()
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
      sub_indices_filter_.registerCallback(
        bind(&SACSegmentation::latched_indices_callback, this, std::placeholders::_1));
      // Subscribe to a callback that sets the header of the saved indices to the cloud header
      sub_input_filter_.registerCallback(
        bind(&SACSegmentation::latched_input_callback, this, std::placeholders::_1));

      // Synchronize the two topics. No need for an approximate synchronizer here, as we'll
      // match the timestamps exactly
      sync_input_indices_e_ = boost::make_shared<message_filters::Synchronizer<
        sync_policies::ExactTime<sensor_msgs::msg::PointCloud2, pcl_msgs::msg::PointIndices>>>(
        max_queue_size_);
      sync_input_indices_e_->connectInput(sub_input_filter_, nf_pi_);
      sync_input_indices_e_->registerCallback(bind(
        &SACSegmentation::input_indices_callback, this, std::placeholders::_1,
        std::placeholders::_2));
    } else {  // "latched_indices" not set, proceed with regular <input,indices> pairs
      if (approximate_sync_) {
        sync_input_indices_a_ =
          boost::make_shared<message_filters::Synchronizer<sync_policies::ApproximateTime<
            sensor_msgs::msg::PointCloud2, pcl_msgs::msg::PointIndices>>>(max_queue_size_);
        sync_input_indices_a_->connectInput(sub_input_filter_, sub_indices_filter_);
        sync_input_indices_a_->registerCallback(bind(
          &SACSegmentation::input_indices_callback, this, std::placeholders::_1,
          std::placeholders::_2));
      } else {
        sync_input_indices_e_ = boost::make_shared<message_filters::Synchronizer<
          sync_policies::ExactTime<sensor_msgs::msg::PointCloud2, pcl_msgs::msg::PointIndices>>>(
          max_queue_size_);
        sync_input_indices_e_->connectInput(sub_input_filter_, sub_indices_filter_);
        sync_input_indices_e_->registerCallback(bind(
          &SACSegmentation::input_indices_callback, this, std::placeholders::_1,
          std::placeholders::_2));
      }
    }
  } else {
    // Subscribe in an old fashion to input only (no filters)
    sub_input_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "input", max_queue_size_,
      std::bind(&pcl_ros::SACSegmentation::input_callback, this, std::placeholders::_1));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void pcl_ros::SACSegmentation::unsubscribe()
{
  if (use_indices_) {
    sub_input_filter_.unsubscribe();
    sub_indices_filter_.unsubscribe();
  } else {
    sub_input_.reset();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
rcl_interfaces::msg::SetParametersResult pcl_ros::SACSegmentation::set_parameters_callback(
  const std::vector<rclcpp::Parameter> & params)
{
  std::lock_guard<std::mutex> lock(mutex_);

  for (const rclcpp::Parameter & param : params) {
    if (param.get_name() == "distance_threshold") {
      double new_distance_threshold = param.as_double();
      if (distance_threshold_ != new_distance_threshold) {
        distance_threshold_ = new_distance_threshold;
        RCLCPP_DEBUG(
          get_logger(), "[set_parameters_callback] Setting new distance to model threshold to: %f",
          distance_threshold_);
        impl_.setDistanceThreshold(distance_threshold_);
      }
    }

    if (param.get_name() == "eps_angle") {
      double new_eps_angle = param.as_double();
      if (eps_angle_ != new_eps_angle) {
        eps_angle_ = new_eps_angle;
        RCLCPP_DEBUG(
          get_logger(),
          "[set_parameters_callback] Setting new epsilon angle to model threshold to: "
          "%f (%f degrees)",
          eps_angle_, eps_angle_ * 180.0 / M_PI);
        impl_.setEpsAngle(eps_angle_);
      }
    }

    if (param.get_name() == "input_frame") {
      std::string new_input_frame = param.as_string();
      if (input_frame_ != new_input_frame) {
        RCLCPP_DEBUG(
          get_logger(), "[set_parameters_callback] Setting the input TF frame to: %s",
          input_frame_.c_str());
        RCLCPP_WARN(get_logger(), "[set_parameters_callback] input_frame TF not implemented yet!");
      }
    }

    if (param.get_name() == "max_iterations") {
      int new_max_iterations = param.as_int();
      if (max_iterations_ != new_max_iterations) {
        max_iterations_ = new_max_iterations;
        RCLCPP_DEBUG(
          get_logger(), "[set_parameters_callback] Setting new maximum number of iterations to: %d",
          max_iterations_);
        impl_.setMaxIterations(max_iterations_);
      }
    }

    if (param.get_name() == "min_inliers") {
      int new_min_inliers = param.as_int();
      if (min_inliers_ != new_min_inliers) {
        min_inliers_ = new_min_inliers;
        RCLCPP_DEBUG(
          get_logger(), "[set_parameters_callback] Setting new minimum number of inliers to: %d",
          min_inliers_);
      }
    }

    if (param.get_name() == "probability") {
      double new_probability = param.as_double();
      if (probability_ != new_probability) {
        probability_ = new_probability;
        RCLCPP_DEBUG(
          get_logger(), "[set_parameters_callback] Setting new probability to: %f", probability_);
        impl_.setProbability(probability_);
      }
    }

    if (param.get_name() == "optimize_coefficients") {
      bool new_optimize_coefficients = param.as_bool();
      if (optimize_coefficients_ != new_optimize_coefficients) {
        optimize_coefficients_ = new_optimize_coefficients;
        RCLCPP_DEBUG(
          get_logger(), "[set_parameters_callback] Setting coefficient optimization to: %s",
          optimize_coefficients_ ? "true" : "false");
        impl_.setOptimizeCoefficients(optimize_coefficients_);
      }
    }

    if (param.get_name() == "radius_max") {
      double new_radius_max = param.as_double();
      if (radius_max_ != new_radius_max) {
        radius_max_ = new_radius_max;
        RCLCPP_DEBUG(
          get_logger(), "[set_parameters_callback] Setting maximum allowable model radius to: %f",
          radius_max_);
        impl_.setRadiusLimits(radius_min_, radius_max_);
      }
    }

    if (param.get_name() == "output_frame") {
      std::string new_output_frame = param.as_string();
      if (output_frame_ != new_output_frame) {
        RCLCPP_DEBUG(
          get_logger(), "[set_parameters_callback] Setting the output TF frame to: %s",
          output_frame_.c_str());
        RCLCPP_WARN(get_logger(), "[set_parameters_callback] output_frame TF not implemented yet!");
      }
    }

    if (param.get_name() == "radius_min") {
      double new_radius_min = param.as_double();
      if (radius_min_ != new_radius_min) {
        radius_min_ = new_radius_min;
        RCLCPP_DEBUG(
          get_logger(), "[set_parameters_callback] Setting minimum allowable model radius to: %f",
          radius_min_);
        impl_.setRadiusLimits(radius_min_, radius_max_);
      }
    }
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void pcl_ros::SACSegmentation::input_indices_callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
  const pcl_msgs::msg::PointIndices::ConstSharedPtr & indices)
{
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

  if (indices && !indices->header.frame_id.empty()) {
    RCLCPP_DEBUG(
      get_logger(),
      "[input_indices_callback]\n"
      "  - PointCloud with %d data points (%s), stamp %d.%09d, and frame %s on topic %s "
      "received.\n"
      "  - PointIndices with %zu values, stamp %d.%09d, and frame %s on topic %s "
      "received",
      cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str(), cloud->header.stamp.sec,
      cloud->header.stamp.nanosec, cloud->header.frame_id.c_str(), "input", indices->indices.size(),
      indices->header.stamp.sec, indices->header.stamp.nanosec, indices->header.frame_id.c_str(),
      "indices");
  } else {
    RCLCPP_DEBUG(
      get_logger(),
      "[input_indices_callback] PointCloud with %d data points, stamp %d.%09d, and "
      "frame %s on topic %s received",
      cloud->width * cloud->height, cloud->header.stamp.sec, cloud->header.stamp.nanosec,
      cloud->header.frame_id.c_str(), "input");
  }

  // Check whether the user has given a different input TF frame
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_tf;
  /*  if (!input_frame_.empty () && cloud->header.frame_id != input_frame_)
    {
      RCLCPP_DEBUG ("[input_callback] Transforming input dataset from %s to %s",
      // cloud->header.frame_id.c_str (), input_frame_.c_str ());
      // Save the original frame ID
      // Convert the cloud into the different frame
      PointCloud cloud_transformed;
      if (!pcl::transformPointCloud (input_frame_, cloud->header.stamp, *cloud,
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

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcl_cloud_tf =
    boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(*cloud_tf, *pcl_cloud_tf);

  // Acquire the mutex before accessing the underlying implementation */
  std::lock_guard<std::mutex> lock(mutex_);

  impl_.setInputCloud(pcl_cloud_tf);
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
  pub_indices_->publish(inliers);
  pub_model_->publish(model);

  RCLCPP_DEBUG(
    get_logger(),
    "[input_indices_callback] Published PointIndices with %zu values on topic %s, "
    "and ModelCoefficients with %zu values on topic %s",
    inliers.indices.size(), "inliers", model.values.size(), "model");

  if (inliers.indices.empty()) {
    RCLCPP_WARN(get_logger(), "[input_indices_callback] No inliers found!");
  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_ros::SACSegmentation)
