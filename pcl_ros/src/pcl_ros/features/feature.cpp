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
 * $Id: feature.cpp 35422 2011-01-24 20:04:44Z rusu $
 *
 */

#include "pcl_ros/features/feature.hpp"

#include <vector>

////////////////////////////////////////////////////////////////////////////////////////////
pcl_ros::Feature::Feature(const std::string & node_name, const rclcpp::NodeOptions & options)
: PCLNode(node_name, options)
{
  RCLCPP_DEBUG(get_logger(), "Feature: constructor");
  init_parameters();
  subscribe();
}

////////////////////////////////////////////////////////////////////////////////////////////
void pcl_ros::Feature::init_parameters()
{
  RCLCPP_DEBUG(get_logger(), "Feature: init_parameters");

  add_parameter(
    "k", rclcpp::ParameterValue(k_), integer_range{0, 1000, 0},  // from, to, step
    "Number of k-nearest neighbors to search for");

  add_parameter(
    "search_radius", rclcpp::ParameterValue(search_radius_),
    floating_point_range{0.0, 0.5, 0.0},  // from, to, step
    "Sphere radius for nearest neighbor search");

  add_parameter(
    "use_surface", rclcpp::ParameterValue(use_surface_),
    "Whether to listen for incoming point clouds representing the search surface");

  k_ = get_parameter("k").as_int();
  search_radius_ = get_parameter("search_radius").as_double();
  use_surface_ = get_parameter("use_surface").as_bool();

  // Initialize the parameter callback
  set_parameters_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&Feature::set_parameters_callback, this, std::placeholders::_1));

  RCLCPP_DEBUG(
    get_logger(),
    "[init_parameters] Node initialized with the following parameters:\n"
    " - k              : %d\n"
    " - search_radius  : %f\n"
    " - use_surface    : %s\n",
    k_, search_radius_, use_surface_ ? "true" : "false");
}

////////////////////////////////////////////////////////////////////////////////////////////
void pcl_ros::Feature::subscribe()
{
  RCLCPP_DEBUG(get_logger(), "Feature: subscribe");

  RCLCPP_DEBUG(get_logger(), "Feature: subscribe: use_indices_: %d", use_indices_);
  RCLCPP_DEBUG(get_logger(), "Feature: subscribe: use_surface_: %d", use_surface_);

  // If we're supposed to look for PointIndices (indices) or PointCloud (surface) messages
  if (use_indices_ || use_surface_) {
    // Create the objects here
    if (approximate_sync_) {
      sync_input_surface_indices_a_ =
        boost::make_shared<message_filters::Synchronizer<sync_policies::ApproximateTime<
          sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2, PointIndices>>>(
          max_queue_size_);
    } else {
      sync_input_surface_indices_e_ =
        boost::make_shared<message_filters::Synchronizer<sync_policies::ExactTime<
          sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2, PointIndices>>>(
          max_queue_size_);
    }

    // Subscribe to the input using a filter
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
    custom_qos_profile.depth = max_queue_size_;
    sub_input_filter_.subscribe(this, "input", custom_qos_profile);

    if (use_indices_) {
      // If indices are enabled, subscribe to the indices
      sub_indices_filter_.subscribe(this, "indices", custom_qos_profile);
      if (use_surface_) {  // Use both indices and surface
        // If surface is enabled, subscribe to the surface,
        // connect the input-indices-surface trio and register
        sub_surface_filter_.subscribe(this, "surface", custom_qos_profile);
        if (approximate_sync_) {
          sync_input_surface_indices_a_->connectInput(
            sub_input_filter_, sub_surface_filter_, sub_indices_filter_);
        } else {
          sync_input_surface_indices_e_->connectInput(
            sub_input_filter_, sub_surface_filter_, sub_indices_filter_);
        }
      } else {  // Use only indices
        sub_input_filter_.registerCallback(
          bind(&Feature::input_callback, this, std::placeholders::_1));
        // surface not enabled, connect the input-indices duo and register
        if (approximate_sync_) {
          sync_input_surface_indices_a_->connectInput(
            sub_input_filter_, nf_pc_, sub_indices_filter_);
        } else {
          sync_input_surface_indices_e_->connectInput(
            sub_input_filter_, nf_pc_, sub_indices_filter_);
        }
      }
    } else {  // Use only surface
      sub_input_filter_.registerCallback(
        bind(&Feature::input_callback, this, std::placeholders::_1));
      // indices not enabled, connect the input-surface duo and register
      sub_surface_filter_.subscribe(this, "surface", custom_qos_profile);
      if (approximate_sync_) {
        sync_input_surface_indices_a_->connectInput(sub_input_filter_, sub_surface_filter_, nf_pi_);
      } else {
        sync_input_surface_indices_e_->connectInput(sub_input_filter_, sub_surface_filter_, nf_pi_);
      }
    }

    // Register callbacks
    if (approximate_sync_) {
      sync_input_surface_indices_a_->registerCallback(bind(
        &Feature::input_surface_indices_callback, this, std::placeholders::_1,
        std::placeholders::_2, std::placeholders::_3));
    } else {
      sync_input_surface_indices_e_->registerCallback(bind(
        &Feature::input_surface_indices_callback, this, std::placeholders::_1,
        std::placeholders::_2, std::placeholders::_3));
    }
  } else {
    // Subscribe in an old fashion to input only (no filters)
    sub_input_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "input", max_queue_size_,
      bind(&Feature::input_no_filters_callback, this, std::placeholders::_1));
  }
}

////////////////////////////////////////////////////////////////////////////////////////////
void pcl_ros::Feature::unsubscribe()
{
  RCLCPP_DEBUG(get_logger(), "Feature: unsubscribe");

  if (use_indices_ || use_surface_) {
    sub_input_filter_.unsubscribe();
    if (use_indices_) {
      sub_indices_filter_.unsubscribe();
      if (use_surface_) {
        sub_surface_filter_.unsubscribe();
      }
    } else {
      sub_surface_filter_.unsubscribe();
    }
  } else {
    sub_input_.reset();
  }
}

////////////////////////////////////////////////////////////////////////////////////////////
rcl_interfaces::msg::SetParametersResult pcl_ros::Feature::set_parameters_callback(
  const std::vector<rclcpp::Parameter> & params)
{
  RCLCPP_DEBUG(get_logger(), "Feature: set_parameters_callback");

  std::lock_guard<std::mutex> lock(mutex_);

  for (const rclcpp::Parameter & param : params) {
    if (param.get_name() == "k") {
      int new_k = param.as_int();
      if (k_ != new_k) {
        k_ = new_k;
        RCLCPP_DEBUG(
          get_logger(),
          "[set_parameters_callback] Setting the number of K nearest neighbors to use for each "
          "point: %d.",
          k_);
      }
    }

    if (param.get_name() == "search_radius") {
      double new_search_radius = param.as_double();
      if (search_radius_ != new_search_radius) {
        search_radius_ = new_search_radius;
        RCLCPP_DEBUG(
          get_logger(),
          "[set_parameters_callback] Setting the nearest neighbors search radius for each point: "
          "%f",
          search_radius_);
      }
    }
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

////////////////////////////////////////////////////////////////////////////////////////////
void pcl_ros::Feature::input_surface_indices_callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_surface,
  const PointIndices::ConstSharedPtr & indices)
{
  RCLCPP_DEBUG(get_logger(), "Feature: input_surface_indices_callback");

  // No subscribers, no work
  if (count_subscribers(pub_output_->get_topic_name()) <= 0) {
    return;
  }

  // If cloud is given, check if it's valid
  if (!isValid(cloud)) {
    RCLCPP_ERROR(get_logger(), "[input_surface_indices_callback] Invalid input!");
    emptyPublish(cloud);
    return;
  }

  // If surface is given, check if it's valid
  if (cloud_surface && !isValid(cloud_surface, "surface")) {
    RCLCPP_ERROR(get_logger(), "[input_surface_indices_callback] Invalid input surface!");
    emptyPublish(cloud);
    return;
  }

  // If indices are given, check if they are valid
  if (indices && !isValid(indices)) {
    RCLCPP_ERROR(get_logger(), "[input_surface_indices_callback] Invalid input indices!");
    emptyPublish(cloud);
    return;
  }

  if (cloud_surface) {
    if (indices) {
      RCLCPP_DEBUG(
        get_logger(),
        "[input_surface_indices_callback]\n"
        "  - PointCloud with %d data points (%s), stamp %d.%09d, and frame %s on topic %s "
        "received.\n"
        "  - PointCloud with %d data points (%s), stamp %d.%09d, and frame %s on topic %s "
        "received.\n"
        "  - PointIndices with %zu values, stamp %d.%09d, and frame %s on topic %s received.",
        cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str(), cloud->header.stamp.sec,
        cloud->header.stamp.nanosec, cloud->header.frame_id.c_str(), "input",
        cloud_surface->width * cloud_surface->height, pcl::getFieldsList(*cloud_surface).c_str(),
        cloud_surface->header.stamp.sec, cloud_surface->header.stamp.nanosec,
        cloud_surface->header.frame_id.c_str(), "surface", indices->indices.size(),
        indices->header.stamp.sec, indices->header.stamp.nanosec, indices->header.frame_id.c_str(),
        "indices");
    } else {
      RCLCPP_DEBUG(
        get_logger(),
        "[input_surface_indices_callback]\n"
        "  - PointCloud with %d data points (%s), stamp %d.%09d, and frame %s on topic %s "
        "received.\n"
        "  - PointCloud with %d data points (%s), stamp %d.%09d, and frame %s on topic %s "
        "received.",
        cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str(), cloud->header.stamp.sec,
        cloud->header.stamp.nanosec, cloud->header.frame_id.c_str(), "input",
        cloud_surface->width * cloud_surface->height, pcl::getFieldsList(*cloud_surface).c_str(),
        cloud_surface->header.stamp.sec, cloud_surface->header.stamp.nanosec,
        cloud_surface->header.frame_id.c_str(), "surface");
    }
  } else if (indices) {
    RCLCPP_DEBUG(
      get_logger(),
      "[input_surface_indices_callback]\n"
      "  - PointCloud with %d data points (%s), stamp %d.%09d, and frame %s on topic %s received.\n"
      "  - PointIndices with %zu values, stamp %d.%09d, and frame %s on topic %s received.",
      cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str(), cloud->header.stamp.sec,
      cloud->header.stamp.nanosec, cloud->header.frame_id.c_str(), "input", indices->indices.size(),
      indices->header.stamp.sec, indices->header.stamp.nanosec, indices->header.frame_id.c_str(),
      "indices");
  } else {
    RCLCPP_DEBUG(
      get_logger(),
      "[input_surface_indices_callback] PointCloud with %d data points, stamp %d.%09d, and frame "
      "%s on "
      "topic %s received.",
      cloud->width * cloud->height, cloud->header.stamp.sec, cloud->header.stamp.nanosec,
      cloud->header.frame_id.c_str(), "input");
  }

  if (static_cast<int>(cloud->width * cloud->height) < k_) {
    RCLCPP_ERROR(
      get_logger(),
      "[input_surface_indices_callback] Requested number of k-nearest neighbors (%d) is larger "
      "than the PointCloud size (%d)!",
      k_, (int)(cloud->width * cloud->height));
    emptyPublish(cloud);
    return;
  }

  // If indices given...
  IndicesPtr vindices;
  if (indices && !indices->header.frame_id.empty()) {
    vindices.reset(new std::vector<int>(indices->indices));
  }

  computePublish(cloud, cloud_surface, vindices);
}

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

void initializeEmptyPointCloud2(sensor_msgs::msg::PointCloud2 & msg)
{
  // Fill in the size of the cloud
  msg.height = 0;
  msg.width = 0;

  // Create the modifier to setup the fields and memory
  sensor_msgs::PointCloud2Modifier mod(msg);

  // Set the fields that our cloud will have
  mod.setPointCloud2FieldsByString(2, "xyz", "rgb");

  // Set up memory for our points
  mod.resize(msg.height * msg.width);

  // Now create iterators for fields
  sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(msg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(msg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(msg, "b");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b)
  {
    *iter_x = 0.0;
    *iter_y = 0.0;
    *iter_z = 0.0;
    *iter_r = 0;
    *iter_g = 255;
    *iter_b = 0;
  }
}

  ////////////////////////////////////////////////////////////////////////////////////////////
void pcl_ros::Feature::input_no_filters_callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input)
{
  RCLCPP_DEBUG(get_logger(), "Feature: input_no_filters_callback");

  // TODO(mjeronimo)
  // const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
  // const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_surface,
  // const PointIndicesConstPtr & indices);

  auto surface = nullptr; // std::make_shared<sensor_msgs::msg::PointCloud2>();
  auto indices = nullptr; // std::make_shared<PointIndices>();

  // initializeEmptyPointCloud2(*surface);

  // TODO(mjeronimo) copy the timestamp or use now()?
  // surface->header.stamp = input->header.stamp;
  //indices->header.stamp = input->header.stamp;

  input_surface_indices_callback(
    //input, sensor_msgs::msg::PointCloud2::ConstSharedPtr(),
    //pcl_msgs::msg::PointIndices::ConstSharedPtr());
    input, surface, indices);
    //pcl_msgs::msg::PointIndices::ConstSharedPtr());
}

////////////////////////////////////////////////////////////////////////////////////////////
void pcl_ros::Feature::input_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input)
{
  RCLCPP_DEBUG(get_logger(), "Feature: input_callback");

  pcl_msgs::msg::PointIndices indices;
  indices.header.stamp = input->header.stamp;

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = input->header.stamp;

  nf_pi_.add(std::make_shared<PointIndices>(indices));
  nf_pc_.add(std::make_shared<sensor_msgs::msg::PointCloud2>(cloud));

  RCLCPP_DEBUG(get_logger(), "Feature::input_callback: return");
}

////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////
pcl_ros::FeatureFromNormals::FeatureFromNormals(
  const std::string & node_name, const rclcpp::NodeOptions & options)
: PCLNode(node_name, options)
{
  init_parameters();
  subscribe();
}

////////////////////////////////////////////////////////////////////////////////////////////
void pcl_ros::FeatureFromNormals::init_parameters()
{
  add_parameter(
    "k", rclcpp::ParameterValue(k_), integer_range{0, 1000, 0},  // from, to, step
    "Number of k-nearest neighbors to search for");

  add_parameter(
    "search_radius", rclcpp::ParameterValue(search_radius_),
    floating_point_range{0.0, 0.5, 0.0},  // from, to, step
    "Sphere radius for nearest neighbor search");

  add_parameter(
    "use_surface", rclcpp::ParameterValue(use_surface_),
    "Whether to listen for incoming point clouds representing the search surface");

  k_ = get_parameter("k").as_int();
  search_radius_ = get_parameter("search_radius").as_double();
  use_surface_ = get_parameter("use_surface").as_bool();

  // Initialize the parameter callback
  set_parameters_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&FeatureFromNormals::set_parameters_callback, this, std::placeholders::_1));

  RCLCPP_DEBUG(
    get_logger(),
    "[init_parameters] Node initialized with the following parameters:\n"
    " - k              : %d\n"
    " - search_radius  : %f\n"
    " - use_surface    : %s\n",
    k_, search_radius_, use_surface_ ? "true" : "false");
}

//////////////////////////////////////////////////////////////////////////////////////////////
void pcl_ros::FeatureFromNormals::subscribe()
{
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
  custom_qos_profile.depth = max_queue_size_;
  sub_input_filter_.subscribe(this, "input", custom_qos_profile);
  sub_normals_filter_.subscribe(this, "normals", custom_qos_profile);

  // Create the objects here
  if (approximate_sync_) {
    sync_input_normals_surface_indices_a_ =
      boost::make_shared<message_filters::Synchronizer<sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2,
        PointIndices>>>(max_queue_size_);
  } else {
    sync_input_normals_surface_indices_e_ =
      boost::make_shared<message_filters::Synchronizer<sync_policies::ExactTime<
        sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2,
        PointIndices>>>(max_queue_size_);
  }

  // If we're supposed to look for PointIndices (indices) or PointCloud (surface) messages
  if (use_indices_ || use_surface_) {
    if (use_indices_) {
      // If indices are enabled, subscribe to the indices
      sub_indices_filter_.subscribe(this, "indices", custom_qos_profile);
      if (use_surface_) {  // Use both indices and surface
        // If surface is enabled, subscribe to the surface, connect the input-indices-surface trio
        // and register
        sub_surface_filter_.subscribe(this, "surface", custom_qos_profile);
        if (approximate_sync_) {
          sync_input_normals_surface_indices_a_->connectInput(
            sub_input_filter_, sub_normals_filter_, sub_surface_filter_, sub_indices_filter_);
        } else {
          sync_input_normals_surface_indices_e_->connectInput(
            sub_input_filter_, sub_normals_filter_, sub_surface_filter_, sub_indices_filter_);
        }
      } else {  // Use only indices
        sub_input_filter_.registerCallback(
          bind(&FeatureFromNormals::input_callback, this, std::placeholders::_1));
        if (approximate_sync_) {
          // surface not enabled, connect the input-indices duo and register
          sync_input_normals_surface_indices_a_->connectInput(
            sub_input_filter_, sub_normals_filter_, nf_pc_, sub_indices_filter_);
        } else {
          // surface not enabled, connect the input-indices duo and register
          sync_input_normals_surface_indices_e_->connectInput(
            sub_input_filter_, sub_normals_filter_, nf_pc_, sub_indices_filter_);
        }
      }
    } else {  // Use only surface
      // indices not enabled, connect the input-surface duo and register
      sub_surface_filter_.subscribe(this, "surface", custom_qos_profile);

      sub_input_filter_.registerCallback(
        bind(&FeatureFromNormals::input_callback, this, std::placeholders::_1));
      if (approximate_sync_) {
        sync_input_normals_surface_indices_a_->connectInput(
          sub_input_filter_, sub_normals_filter_, sub_surface_filter_, nf_pi_);
      } else {
        sync_input_normals_surface_indices_e_->connectInput(
          sub_input_filter_, sub_normals_filter_, sub_surface_filter_, nf_pi_);
      }
    }
  } else {
    sub_input_filter_.registerCallback(
      bind(&FeatureFromNormals::input_callback, this, std::placeholders::_1));

    if (approximate_sync_) {
      sync_input_normals_surface_indices_a_->connectInput(
        sub_input_filter_, sub_normals_filter_, nf_pc_, nf_pi_);
    } else {
      sync_input_normals_surface_indices_e_->connectInput(
        sub_input_filter_, sub_normals_filter_, nf_pc_, nf_pi_);
    }
  }

  // Register callbacks
  if (approximate_sync_) {
    sync_input_normals_surface_indices_a_->registerCallback(bind(
      &FeatureFromNormals::input_normals_surface_indices_callback, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
  } else {
    sync_input_normals_surface_indices_e_->registerCallback(bind(
      &FeatureFromNormals::input_normals_surface_indices_callback, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void pcl_ros::FeatureFromNormals::unsubscribe()
{
  sub_input_filter_.unsubscribe();
  sub_normals_filter_.unsubscribe();
  if (use_indices_ || use_surface_) {
    if (use_indices_) {
      sub_indices_filter_.unsubscribe();
      if (use_surface_) {
        sub_surface_filter_.unsubscribe();
      }
    } else {
      sub_surface_filter_.unsubscribe();
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////
rcl_interfaces::msg::SetParametersResult pcl_ros::FeatureFromNormals::set_parameters_callback(
  const std::vector<rclcpp::Parameter> & params)
{
  std::lock_guard<std::mutex> lock(mutex_);

  for (const rclcpp::Parameter & param : params) {
    if (param.get_name() == "k") {
      int new_k = param.as_int();
      if (k_ != new_k) {
        k_ = new_k;
        RCLCPP_DEBUG(
          get_logger(),
          "[set_parameters_callback] Setting the number of K nearest neighbors to use for each "
          "point: %d.",
          k_);
      }
    }

    if (param.get_name() == "search_radius") {
      double new_search_radius = param.as_double();
      if (search_radius_ != new_search_radius) {
        search_radius_ = new_search_radius;
        RCLCPP_DEBUG(
          get_logger(),
          "[set_parameters_callback] Setting the nearest neighbors search radius for each point: "
          "%f",
          search_radius_);
      }
    }
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void pcl_ros::FeatureFromNormals::input_normals_surface_indices_callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_normals,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_surface,
  const pcl_msgs::msg::PointIndices::ConstSharedPtr & indices)
{
  // No subscribers, no work
  if (count_subscribers(pub_output_->get_topic_name()) <= 0) {
    return;
  }

  // If cloud_normals is given, check if it's valid
  if (!isValid(cloud)) {  // || !isValid (cloud_normals, "normals"))
    RCLCPP_ERROR(get_logger(), "[input_normals_surface_indices_callback] Invalid input!");
    emptyPublish(cloud);
    return;
  }

  // If surface is given, check if it's valid
  if (cloud_surface && !isValid(cloud_surface, "surface")) {
    RCLCPP_ERROR(get_logger(), "[input_normals_surface_indices_callback] Invalid input surface!");
    emptyPublish(cloud);
    return;
  }

  // If indices are given, check if they are valid
  if (indices && !isValid(indices)) {
    RCLCPP_ERROR(get_logger(), "[input_normals_surface_indices_callback] Invalid input indices!");
    emptyPublish(cloud);
    return;
  }

  /// DEBUG
  if (cloud_surface) {
    if (indices) {
      RCLCPP_DEBUG(
        get_logger(),
        "[input_normals_surface_indices_callback]\n"
        "  - PointCloud with %d data points (%s), stamp %d.%09d, and frame %s on topic %s "
        "received.\n"
        "  - PointCloud with %d data points (%s), stamp %d.%09d, and frame %s on topic %s "
        "received.\n"
        "  - PointCloud with %d data points (%s), stamp %d.%09d, and frame %s on topic %s "
        "received.\n"
        "  - PointIndices with %zu values, stamp %d.%09d, and frame %s on topic %s received.",
        cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str(), cloud->header.stamp.sec,
        cloud->header.stamp.nanosec, cloud->header.frame_id.c_str(), "input",
        cloud_surface->width * cloud_surface->height, pcl::getFieldsList(*cloud_surface).c_str(),
        cloud_surface->header.stamp.sec, cloud_surface->header.stamp.nanosec,
        cloud_surface->header.frame_id.c_str(), "surface",
        cloud_normals->width * cloud_normals->height, pcl::getFieldsList(*cloud_normals).c_str(),
        cloud_normals->header.stamp.sec, cloud_normals->header.stamp.nanosec,
        cloud_normals->header.frame_id.c_str(), "normals", indices->indices.size(),
        indices->header.stamp.sec, indices->header.stamp.nanosec, indices->header.frame_id.c_str(),
        "indices");
    } else {
      RCLCPP_DEBUG(
        get_logger(),
        "[input_normals_surface_indices_callback]\n"
        "  - PointCloud with %d data points (%s), stamp %d.%09d, and frame %s on topic %s "
        "received.\n"
        "  - PointCloud with %d data points (%s), stamp %d.%09d, and frame %s on topic %s "
        "received.\n"
        "  - PointCloud with %d data points (%s), stamp %d.%09d, and frame %s on topic %s "
        "received.",
        cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str(), cloud->header.stamp.sec,
        cloud->header.stamp.nanosec, cloud->header.frame_id.c_str(), "input",
        cloud_surface->width * cloud_surface->height, pcl::getFieldsList(*cloud_surface).c_str(),
        cloud_surface->header.stamp.sec, cloud_surface->header.stamp.nanosec,
        cloud_surface->header.frame_id.c_str(), "surface",
        cloud_normals->width * cloud_normals->height, pcl::getFieldsList(*cloud_normals).c_str(),
        cloud_normals->header.stamp.sec, cloud_normals->header.stamp.nanosec,
        cloud_normals->header.frame_id.c_str(), "normals");
    }
  } else if (indices) {
    RCLCPP_DEBUG(
      get_logger(),
      "[input_normals_surface_indices_callback]\n"
      "  - PointCloud with %d data points (%s), stamp %d.%09d, and frame %s on topic %s received.\n"
      "  - PointCloud with %d data points (%s), stamp %d.%09d, and frame %s on topic %s received.\n"
      "  - PointIndices with %zu values, stamp %d.%09d, and frame %s on topic %s received.",
      cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str(), cloud->header.stamp.sec,
      cloud->header.stamp.nanosec, cloud->header.frame_id.c_str(), "input",
      cloud_normals->width * cloud_normals->height, pcl::getFieldsList(*cloud_normals).c_str(),
      cloud_normals->header.stamp.sec, cloud_normals->header.stamp.nanosec,
      cloud_normals->header.frame_id.c_str(), "normals", indices->indices.size(),
      indices->header.stamp.sec, indices->header.stamp.nanosec, indices->header.frame_id.c_str(),
      "indices");
  } else {
    RCLCPP_DEBUG(
      get_logger(),
      "[input_normals_surface_indices_callback]\n"
      "  - PointCloud with %d data points (%s), stamp %d.%09d, and frame %s on topic %s received.\n"
      "  - PointCloud with %d data points (%s), stamp %d.%09d, and frame %s on topic %s received.",
      cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str(), cloud->header.stamp.sec,
      cloud->header.stamp.nanosec, cloud->header.frame_id.c_str(), "input",
      cloud_normals->width * cloud_normals->height, pcl::getFieldsList(*cloud_normals).c_str(),
      cloud_normals->header.stamp.sec, cloud_normals->header.stamp.nanosec,
      cloud_normals->header.frame_id.c_str(), "normals");
  }
  ///

  if (static_cast<int>(cloud->width * cloud->height) < k_) {
    RCLCPP_ERROR(
      get_logger(),
      "[input_normals_surface_indices_callback] Requested number of k-nearest neighbors (%d) "
      "is larger than the PointCloud size (%d)!",
      k_, (int)(cloud->width * cloud->height));
    emptyPublish(cloud);
    return;
  }

  // If indices given...
  IndicesPtr vindices;
  if (indices && !indices->header.frame_id.empty()) {
    vindices.reset(new std::vector<int>(indices->indices));
  }

  computePublish(cloud, cloud_normals, cloud_surface, vindices);
}

////////////////////////////////////////////////////////////////////////////////////////////
void pcl_ros::FeatureFromNormals::input_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input)
{
  RCLCPP_DEBUG(get_logger(), "FeatureFromNormals::input_callback");

  pcl_msgs::msg::PointIndices indices;
  indices.header.stamp = input->header.stamp;

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = input->header.stamp;

  nf_pi_.add(std::make_shared<PointIndices>(indices));
  nf_pc_.add(std::make_shared<sensor_msgs::msg::PointCloud2>(cloud));

  RCLCPP_DEBUG(get_logger(), "FeatureFromNormals::input_callback: return");
}

