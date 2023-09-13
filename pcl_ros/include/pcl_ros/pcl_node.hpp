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
 * $Id: pcl_node.h 33238 2010-03-11 00:46:58Z rusu $
 *
 */

/**

\author Radu Bogdan Rusu

**/

#ifndef PCL_ROS__PCL_NODE_HPP_
#define PCL_ROS__PCL_NODE_HPP_

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
// #include <nodelet_topic_tools/nodelet_lazy.h>  // TODO(daisukes): lazy subscription

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_msgs/msg/point_indices.hpp>
#include <pcl_msgs/msg/model_coefficients.hpp>

// #include "pcl_ros/point_cloud.hpp"

using pcl_conversions::fromPCL;

namespace pcl_ros
{
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
/** \brief @b PCLNode represents the base PCL Node class. All PCL nodes should inherit from
 *  this class. */
template<typename T, typename PublisherT = rclcpp::Publisher<T>>
class PCLNode : public rclcpp::Node
{
public:
  typedef sensor_msgs::msg::PointCloud2 PointCloud2;

  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
  typedef PointCloud::Ptr PointCloudPtr;
  typedef PointCloud::ConstPtr PointCloudConstPtr;

  typedef pcl_msgs::msg::PointIndices PointIndices;
  typedef PointIndices::SharedPtr PointIndicesPtr;
  typedef PointIndices::ConstSharedPtr PointIndicesConstPtr;

  typedef pcl_msgs::msg::ModelCoefficients ModelCoefficients;
  typedef ModelCoefficients::SharedPtr ModelCoefficientsPtr;
  typedef ModelCoefficients::ConstSharedPtr ModelCoefficientsConstPtr;

  typedef pcl::IndicesPtr IndicesPtr;
  typedef pcl::IndicesConstPtr IndicesConstPtr;

  /** \brief Empty constructor. */
  PCLNode(std::string node_name, const rclcpp::NodeOptions & options)
  : rclcpp::Node(node_name, options),
    use_indices_(false), transient_local_indices_(false),
    max_queue_size_(3), approximate_sync_(false),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.name = "max_queue_size";
      desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
      desc.description = "QoS History depth";
      desc.read_only = true;
      max_queue_size_ = declare_parameter(desc.name, max_queue_size_, desc);
    }

    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.name = "use_indices";
      desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
      desc.description = "Only process a subset of the point cloud from an indices topic";
      desc.read_only = true;
      use_indices_ = declare_parameter(desc.name, use_indices_, desc);
    }

    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.name = "transient_local_indices";
      desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
      desc.description = "Does indices topic use transient local documentation";
      desc.read_only = true;
      transient_local_indices_ = declare_parameter(desc.name, transient_local_indices_, desc);
    }

    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.name = "approximate_sync";
      desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
      desc.description =
        "Match indices and point cloud messages if time stamps are approximatly the same.";
      desc.read_only = true;
      approximate_sync_ = declare_parameter(desc.name, approximate_sync_, desc);
    }

    RCLCPP_DEBUG(
      this->get_logger(), "PCL Node successfully created with the following parameters:\n"
      " - approximate_sync          : %s\n"
      " - use_indices               : %s\n"
      " - transient_local_indices_  : %s\n"
      " - max_queue_size            : %d",
      (approximate_sync_) ? "true" : "false",
      (use_indices_) ? "true" : "false",
      (transient_local_indices_) ? "true" : "false",
      max_queue_size_);
  }

protected:
  /** \brief Set to true if point indices are used.
   *
   * When receiving a point cloud, if use_indices_ is false, the entire
   * point cloud is processed for the given operation. If use_indices_ is
   * true, then the ~indices topic is read to get the vector of point
   * indices specifying the subset of the point cloud that will be used for
   * the operation. In the case where use_indices_ is true, the ~input and
   * ~indices topics must be synchronised in time, either exact or within a
   * specified jitter. See also @ref transient_local_indices_ and approximate_sync.
   **/
  bool use_indices_;
  /** \brief Set to true if the indices topic has transient_local durability.
   *
   * If use_indices_ is true, the ~input and ~indices topics generally must
   * be synchronised in time. By setting this flag to true, the most recent
   * value from ~indices can be used instead of requiring a synchronised
   * message.
   **/
  bool transient_local_indices_;

  /** \brief The message filter subscriber for PointCloud2. */
  message_filters::Subscriber<PointCloud2> sub_input_filter_;

  /** \brief The message filter subscriber for PointIndices. */
  message_filters::Subscriber<PointIndices> sub_indices_filter_;

  /** \brief The output PointCloud publisher. Allow each individual class that inherits from this
    *  to declare the Publisher with their type.
    */
  std::shared_ptr<PublisherT> pub_output_;

  /** \brief The maximum queue size (default: 3). */
  int max_queue_size_;

  /** \brief True if we use an approximate time synchronizer
    * versus an exact one (false by default).
    **/
  bool approximate_sync_;

  /** \brief TF listener object. */
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  /** \brief Test whether a given PointCloud message is "valid" (i.e., has points, and width and height are non-zero).
    * \param cloud the point cloud to test
    * \param topic_name an optional topic name (only used for printing, defaults to "input")
    */
  inline bool
  isValid(const PointCloud2::ConstSharedPtr & cloud, const std::string & topic_name = "input")
  {
    if (cloud->width * cloud->height * cloud->point_step != cloud->data.size()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Invalid PointCloud (data = %zu, width = %d, height = %d, step = %d) "
        "with stamp %d.%09d, and frame %s on topic %s received!",
        cloud->data.size(), cloud->width, cloud->height, cloud->point_step,
        cloud->header.stamp.sec, cloud->header.stamp.nanosec,
        cloud->header.frame_id.c_str(), topic_name.c_str());

      return false;
    }
    return true;
  }

  /** \brief Test whether a given PointCloud message is "valid" (i.e., has points, and width and height are non-zero).
    * \param cloud the point cloud to test
    * \param topic_name an optional topic name (only used for printing, defaults to "input")
    */
  inline bool
  isValid(const PointCloudConstPtr & cloud, const std::string & topic_name = "input")
  {
    if (cloud->width * cloud->height != cloud->points.size()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Invalid PointCloud (points = %zu, width = %d, height = %d) "
        "with stamp %d.%09d, and frame %s on topic %s received!",
        cloud->points.size(), cloud->width, cloud->height,
        fromPCL(cloud->header).stamp.sec, fromPCL(cloud->header).stamp.nanosec,
        cloud->header.frame_id.c_str(), topic_name.c_str());

      return false;
    }
    return true;
  }

  /** \brief Test whether a given PointIndices message is "valid" (i.e., has values).
    * \param indices the point indices message to test
    * \param topic_name an optional topic name (only used for printing, defaults to "indices")
    */
  inline bool
  isValid(
    const PointIndicesConstPtr & /*indices*/,
    const std::string & /*topic_name*/ = "indices")
  {
    /*if (indices->indices.empty ())
    {
      RCLCPP_WARN(
        this->get_logger(), "Empty indices (values = %zu) with stamp %d.%09d, "
        "and frame %s on topic %s received!",
        indices->indices.size(), indices->header.stamp.sec, indices->header.stamp.nanosec,
        indices->header.frame_id.c_str(), topic_name.c_str());
      //return (true);  // looks like it should be false
      return false;
    }*/
    return true;
  }

  /** \brief Test whether a given ModelCoefficients message is "valid" (i.e., has values).
    * \param model the model coefficients to test
    * \param topic_name an optional topic name (only used for printing, defaults to "model")
    */
  inline bool
  isValid(
    const ModelCoefficientsConstPtr & /*model*/,
    const std::string & /*topic_name*/ = "model")
  {
    /*if (model->values.empty ())
    {
      RCLCPP_WARN(
        this->get_logger(), "Empty model (values = %zu) with stamp %d.%09d, "
        "and frame %s on topic %s received!",
        model->values.size(), model->header.stamp.sec, model->header.stamp.nanosec,
        model->header.frame_id.c_str(), topic_name.c_str());
      return (false);
    }*/
    return true;
  }

  /** \brief Lazy transport subscribe/unsubscribe routine.
    * It is optional for backward compatibility.
    **/
  virtual void subscribe() {}
  virtual void unsubscribe() {}

  typedef struct
  {
    double from_value;
    double to_value;
    double step;
  } floating_point_range;

  typedef struct
  {
    int from_value;
    int to_value;
    int step;
  } integer_range;

  /**
   * @brief Declare a parameter that has no integer or floating point range constraints
   * @param node_name Name of parameter
   * @param default_value Default node value to add
   * @param description Node description
   * @param additional_constraints Any additional constraints on the parameters to list
   * @param read_only Whether this param should be considered read only
   */
  void add_parameter(
    const std::string & name, const rclcpp::ParameterValue & default_value,
    const std::string & description = "", const std::string & additional_constraints = "",
    bool read_only = false)
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    descriptor.name = name;
    descriptor.description = description;
    descriptor.additional_constraints = additional_constraints;
    descriptor.read_only = read_only;

    declare_parameter(descriptor.name, default_value, descriptor);
  }

  /**
   * @brief Declare a parameter that has a floating point range constraint
   * @param node_name Name of parameter
   * @param default_value Default node value to add
   * @param fp_range floating point range
   * @param description Node description
   * @param additional_constraints Any additional constraints on the parameters to list
   * @param read_only Whether this param should be considered read only
   */
  void add_parameter(
    const std::string & name, const rclcpp::ParameterValue & default_value,
    const floating_point_range fp_range,
    const std::string & description = "", const std::string & additional_constraints = "",
    bool read_only = false)
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    descriptor.name = name;
    descriptor.description = description;
    descriptor.additional_constraints = additional_constraints;
    descriptor.read_only = read_only;
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = fp_range.from_value;
    descriptor.floating_point_range[0].to_value = fp_range.to_value;
    descriptor.floating_point_range[0].step = fp_range.step;

    declare_parameter(descriptor.name, default_value, descriptor);
  }

  /**
   * @brief Declare a parameter that has an integer range constraint
   * @param node_name Name of parameter
   * @param default_value Default node value to add
   * @param integer_range Integer range
   * @param description Node description
   * @param additional_constraints Any additional constraints on the parameters to list
   * @param read_only Whether this param should be considered read only
   */
  void add_parameter(
    const std::string & name, const rclcpp::ParameterValue & default_value,
    const integer_range int_range,
    const std::string & description = "", const std::string & additional_constraints = "",
    bool read_only = false)
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    descriptor.name = name;
    descriptor.description = description;
    descriptor.additional_constraints = additional_constraints;
    descriptor.read_only = read_only;
    descriptor.integer_range.resize(1);
    descriptor.integer_range[0].from_value = int_range.from_value;
    descriptor.integer_range[0].to_value = int_range.to_value;
    descriptor.integer_range[0].step = int_range.step;

    declare_parameter(descriptor.name, default_value, descriptor);
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace pcl_ros

#endif  // PCL_ROS__PCL_NODE_HPP_
