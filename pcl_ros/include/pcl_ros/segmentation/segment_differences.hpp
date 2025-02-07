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
 * $Id: segment_differences.hpp 35361 2011-01-20 04:34:49Z rusu $
 *
 */

#ifndef PCL_ROS__SEGMENTATION__SEGMENT_DIFFERENCES_HPP_
#define PCL_ROS__SEGMENTATION__SEGMENT_DIFFERENCES_HPP_

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <pcl/segmentation/segment_differences.h>

#include <memory>
#include <vector>

#include "pcl_ros/pcl_node.hpp"
#include <rclcpp/rclcpp.hpp>

namespace pcl_ros
{
namespace sync_policies = message_filters::sync_policies;

/** \brief @b SegmentDifferences obtains the difference between two spatially aligned point clouds
 * and returns the difference between them for a maximum given distance threshold. \author Radu
 * Bogdan Rusu
 */
class SegmentDifferences : public PCLNode<sensor_msgs::msg::PointCloud2>
{
public:
  /** \brief Disallow the empty constructor. */
  SegmentDifferences() = delete;

  /** \brief SegmentDifferences constructor
   * \param options node options
   */
  explicit SegmentDifferences(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  /** \brief Initialize the node's parameters. */
  void init_parameters();

  /** \brief Lazy transport subscribe routine. */
  void subscribe();

  /** \brief Lazy transport unsubscribe routine. */
  void unsubscribe();

  /** \brief Input point cloud callback.
   * \param cloud the pointer to the input point cloud
   * \param cloud_target the pointcloud that we want to segment \a cloud from
   */
  void input_target_callback(
    const PointCloud2::ConstSharedPtr & cloud, const PointCloud2::ConstSharedPtr & cloud_target);

  /** \brief Parameter callback
   * \param params parameter values to set
   */
  rcl_interfaces::msg::SetParametersResult set_parameters_callback(
    const std::vector<rclcpp::Parameter> & params);

  /** \brief Pointer to parameters callback handle. */
  OnSetParametersCallbackHandle::SharedPtr set_parameters_callback_handle_;

  /** \brief Internal mutex. */
  std::mutex mutex_;

  /** \brief The distance tolerance as a measure in the L2 Euclidean space between corresponding
   * points. */
  double distance_threshold_{0.0};

  /** \brief The message filter subscriber for PointCloud2. */
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_target_filter_;

  /** \brief Synchronized input and planar hull.*/
  std::shared_ptr<message_filters::Synchronizer<
    sync_policies::ExactTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>>>
    sync_input_target_e_;
  std::shared_ptr<message_filters::Synchronizer<
    sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>>>
    sync_input_target_a_;

  /** \brief The underlying PCL implementation used. */
  pcl::SegmentDifferences<pcl::PointXYZ> impl_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace pcl_ros

#endif  // PCL_ROS__SEGMENTATION__SEGMENT_DIFFERENCES_HPP_
