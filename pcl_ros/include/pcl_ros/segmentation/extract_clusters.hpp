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
 * $Id: extract_clusters.h 35361 2011-01-20 04:34:49Z rusu $
 *
 */

#ifndef PCL_ROS__SEGMENTATION__EXTRACT_CLUSTERS_HPP_
#define PCL_ROS__SEGMENTATION__EXTRACT_CLUSTERS_HPP_

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <pcl/segmentation/extract_clusters.h>

#include <limits>
#include <vector>

#include "pcl_ros/pcl_node.hpp"
#include <rclcpp/rclcpp.hpp>

namespace pcl_ros
{
namespace sync_policies = message_filters::sync_policies;

////////////////////////////////////////////////////////////////////////////////////////////
/** \brief @b EuclideanClusterExtraction represents a segmentation class for cluster extraction in
 * an Euclidean sense. \author Radu Bogdan Rusu
 */
class EuclideanClusterExtraction : public PCLNode<sensor_msgs::msg::PointCloud2>
{
public:
  /** \brief Disallow the empty constructor. */
  EuclideanClusterExtraction() = delete;

  /** \brief EuclideanClusterExtraction constructor
   * \param options node options
   */
  explicit EuclideanClusterExtraction(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  /** \brief Initialize the node's parameters. */
  void init_parameters();

  /** \brief Lazy transport subscribe routine. */
  void subscribe();

  /** \brief Lazy transport unsubscribe routine. */
  void unsubscribe();

  /** \brief Input point cloud callback.
   * \param cloud the pointer to the input point cloud
   * \param indices the pointer to the input point cloud indices
   */
  void input_indices_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
    const pcl_msgs::msg::PointIndices::ConstSharedPtr & indices);

  /** \brief Input point cloud callback.
   * \param cloud the pointer to the input point cloud
   */
  void input_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud);

  /** \brief Parameter callback
   * \param params parameter values to set
   */
  rcl_interfaces::msg::SetParametersResult set_parameters_callback(
    const std::vector<rclcpp::Parameter> & params);

  /** \brief Pointer to parameters callback handle. */
  OnSetParametersCallbackHandle::SharedPtr set_parameters_callback_handle_;

  /** \brief Internal mutex. */
  std::mutex mutex_;

  /** The maximum number of points that a cluster must contain in order to be accepted. */
  int cluster_max_size_{2147483647};

  /** \brief The minimum number of points that a cluster must contain in order to be accepted. */
  int cluster_min_size_{1};

  /** \brief The spatial tolerance as a measure in the L2 Euclidean space. */
  double cluster_tolerance_{0.05};

  /** \brief Maximum number of clusters to extract */
  int max_clusters_{std::numeric_limits<int>::max()};

  /** \brief Publish indices or convert to PointCloud clusters. */
  bool publish_indices_{false};

  /** \brief A publisher for the optional PointIndices output. */
  rclcpp::Publisher<pcl_msgs::msg::PointIndices>::SharedPtr pub_indices;

  /** \brief The input PointCloud subscriber. */
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_input_;

  /** \brief Synchronized input, and indices.*/
  boost::shared_ptr<message_filters::Synchronizer<
    sync_policies::ExactTime<sensor_msgs::msg::PointCloud2, pcl_msgs::msg::PointIndices>>>
    sync_input_indices_e_;
  boost::shared_ptr<message_filters::Synchronizer<
    sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, pcl_msgs::msg::PointIndices>>>
    sync_input_indices_a_;

  /** \brief The underlying PCL implementation used. */
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> impl_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace pcl_ros

#endif  // PCL_ROS__SEGMENTATION__EXTRACT_CLUSTERS_HPP_
