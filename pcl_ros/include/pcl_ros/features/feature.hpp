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
 * $Id: feature.h 35422 2011-01-24 20:04:44Z rusu $
 *
 */

#ifndef PCL_ROS__FEATURES__FEATURE_HPP_
#define PCL_ROS__FEATURES__FEATURE_HPP_

#include "message_filters/pass_through.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/features/feature.h"
#include "pcl_ros/pcl_node.hpp"

namespace pcl_ros
{
namespace sync_policies = message_filters::sync_policies;

///////////////////////////////////////////////////////////////////////////////////////////
/** \brief @b Feature represents the base feature class. Some generic 3D operations that
  * are applicable to all features are defined here as static methods.
  * \author Radu Bogdan Rusu
  */
class Feature : public PCLNode<sensor_msgs::msg::PointCloud2>
{
public:
  typedef pcl::KdTree<pcl::PointXYZ> KdTree;
  typedef pcl::KdTree<pcl::PointXYZ>::Ptr KdTreePtr;

  // typedef pcl::IndicesPtr IndicesPtr;
  // typedef pcl::IndicesConstPtr IndicesConstPtr;

  /** \brief Disallow the empty constructor. */
  Feature() = delete;

  /** \brief Feature constructor
    * \param node_name node name
    * \param options node options
    */
  explicit Feature(const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  /** \brief Initialize the node's parameters. */
  void init_parameters();

  /** \brief Lazy transport subscribe routine. */
  void subscribe();

  /** \brief Lazy transport unsubscribe routine. */
  void unsubscribe();

  /** \brief Publish an empty point cloud of the feature output type. */
  virtual void emptyPublish(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud) = 0;

  /** \brief Compute the feature and publish it. Internal method. */
  virtual void computePublish(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & surface,
    const IndicesPtr & indices) = 0;

  /** \brief Input point cloud callback. Used when \a use_indices and \a use_surface are set.
    * \param cloud the pointer to the input point cloud
    * \param cloud_surface the pointer to the surface point cloud
    * \param indices the pointer to the input point cloud indices
    */
  void input_surface_indices_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_surface,
    const PointIndicesConstPtr & indices);

  /** \brief Input point cloud callback without using message filters */
  void input_no_filters_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input);

  /** \brief Input point cloud callback.
    * Because we want to use the same synchronizer object, we push back
    * empty elements with the same timestamp.
    */
  void input_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input);

  /** \brief Parameter callback
  * \param params parameter values to set
  */
  rcl_interfaces::msg::SetParametersResult
  set_parameters_callback(const std::vector<rclcpp::Parameter> & params);

    /** \brief Pointer to parameters callback handle. */
  OnSetParametersCallbackHandle::SharedPtr set_parameters_callback_handle_;

  /** \brief Internal mutex. */
  std::mutex mutex_;

  /** \brief The number of K nearest neighbors to use for each point. */
  int k_{10};

  /** \brief The nearest neighbors search radius for each point. */
  double search_radius_{0.0};

  /** \brief Set to true if the node needs to listen for incoming point
   * clouds representing the search surface.
   */
  bool use_surface_{false};

  /** \brief The surface PointCloud subscriber filter. */
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_surface_filter_;

  /** \brief The input PointCloud subscriber. */
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_input_;

  /** \brief Null passthrough filter, used for pushing empty elements in the
    * synchronizer */
  message_filters::PassThrough<pcl_msgs::msg::PointIndices> nf_pi_;
  message_filters::PassThrough<sensor_msgs::msg::PointCloud2> nf_pc_;

  /** \brief Synchronized input, surface, and point indices.*/
  boost::shared_ptr<message_filters::Synchronizer<sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2,
    sensor_msgs::msg::PointCloud2, pcl_msgs::msg::PointIndices>>> sync_input_surface_indices_a_;
  boost::shared_ptr<message_filters::Synchronizer<sync_policies::ExactTime<sensor_msgs::msg::PointCloud2,
    sensor_msgs::msg::PointCloud2, pcl_msgs::msg::PointIndices>>> sync_input_surface_indices_e_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


//////////////////////////////////////////////////////////////////////////////////////////
class FeatureFromNormals : public PCLNode<sensor_msgs::msg::PointCloud2>
{
public:
  // typedef pcl::PointCloud<pcl::Normal> PointCloudN;
  // typedef boost::shared_ptr<PointCloudN> PointCloudNPtr;
  // typedef boost::shared_ptr<const PointCloudN> PointCloudNConstPtr;

  /** \brief Disallow the empty constructor. */
  FeatureFromNormals() = delete;

  /** \brief Empty constructor. */
  explicit FeatureFromNormals(const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  /** \brief Initialize the node's parameters. */
  void init_parameters();

  /** \brief Lazy transport subscribe routine. */
  void subscribe();

  /** \brief Lazy transport unsubscribe routine. */
  void unsubscribe();

  /** \brief Publish an empty point cloud of the feature output type. */
  virtual void emptyPublish(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud) = 0;

  /** \brief Compute the feature and publish it. */
  virtual void computePublish(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & normals,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & surface,
    const IndicesPtr & indices) = 0;

  /** \brief Input point cloud callback. Used when \a use_indices and \a use_surface are set.
    * \param cloud the pointer to the input point cloud
    * \param cloud_normals the pointer to the input point cloud normals
    * \param cloud_surface the pointer to the surface point cloud
    * \param indices the pointer to the input point cloud indices
    */
  void input_normals_surface_indices_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_normals,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_surface,
    const pcl_msgs::msg::PointIndices::ConstSharedPtr & indices);

  /** \brief Input point cloud callback.
    * Because we want to use the same synchronizer object, we push back
    * empty elements with the same timestamp.
    */
  void input_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input);

  /** \brief Parameter callback
  * \param params parameter values to set
  */
  rcl_interfaces::msg::SetParametersResult
  set_parameters_callback(const std::vector<rclcpp::Parameter> & params);

    /** \brief Pointer to parameters callback handle. */
  OnSetParametersCallbackHandle::SharedPtr set_parameters_callback_handle_;

  /** \brief Internal mutex. */
  std::mutex mutex_;

  /** \brief The number of K nearest neighbors to use for each point. */
  int k_{10};

  /** \brief The nearest neighbors search radius for each point. */
  double search_radius_{0.0};

  /** \brief Set to true if the node needs to listen for incoming point
   * clouds representing the search surface.
   */
  bool use_surface_{false};

    /** \brief The surface PointCloud subscriber filter. */
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_surface_filter_;

  /** \brief Null passthrough filter, used for pushing empty elements in the
    * synchronizer */
  message_filters::PassThrough<pcl_msgs::msg::PointIndices> nf_pi_;
  message_filters::PassThrough<sensor_msgs::msg::PointCloud2> nf_pc_;

  /** \brief A pointer to the input dataset that contains the point normals of the XYZ dataset. */
  sensor_msgs::msg::PointCloud2::ConstSharedPtr normals_;

  /** \brief The normals PointCloud subscriber filter. */
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_normals_filter_;

  /** \brief Synchronized input, normals, surface, and point indices.*/
  boost::shared_ptr<message_filters::Synchronizer<sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2,
    sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2, pcl_msgs::msg::PointIndices>>> sync_input_normals_surface_indices_a_;

  boost::shared_ptr<message_filters::Synchronizer<sync_policies::ExactTime<sensor_msgs::msg::PointCloud2,
    sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2, pcl_msgs::msg::PointIndices>>> sync_input_normals_surface_indices_e_;

  /** \brief Internal method. */
  void computePublish(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &,
    const IndicesPtr &) {}                        // This should never be called

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace pcl_ros

#endif  // PCL_ROS__FEATURES__FEATURE_HPP_
