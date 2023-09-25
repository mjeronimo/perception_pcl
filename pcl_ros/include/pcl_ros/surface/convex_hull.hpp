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
 * $Id: convex_hull.hpp 36116 2011-02-22 00:05:23Z rusu $
 *
 */

#ifndef PCL_ROS__SURFACE__CONVEX_HULL_HPP_
#define PCL_ROS__SURFACE__CONVEX_HULL_HPP_

#include <pcl/surface/convex_hull.h>

#include <geometry_msgs/msg/polygon_stamped.hpp>

#include "pcl_ros/pcl_node.hpp"

namespace pcl_ros
{
namespace sync_policies = message_filters::sync_policies;

/** \brief @b ConvexHull2D represents a 2D ConvexHull implementation.
  * \author Radu Bogdan Rusu
  */
class ConvexHull2D : public PCLNode<sensor_msgs::msg::PointCloud2>
{
public:
 /** \brief ConvexHull2D constructor
   * \param options node options
   */
  explicit ConvexHull2D(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

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
  void input_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
  {
    input_indices_callback(cloud, nullptr);
  }

  /** \brief The input PointCloud subscriber. */
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_input_;

  /** \brief Publisher for PolygonStamped. */
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub_plane_;

  /** \brief Synchronized input, and indices.*/
  std::shared_ptr<message_filters::Synchronizer<
    sync_policies::ExactTime<sensor_msgs::msg::PointCloud2, pcl_msgs::msg::PointIndices>>>
    sync_input_indices_e_;
  std::shared_ptr<message_filters::Synchronizer<
    sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, pcl_msgs::msg::PointIndices>>>
    sync_input_indices_a_;

  /** \brief The PCL implementation used. */
  pcl::ConvexHull<pcl::PointXYZ> impl_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace pcl_ros

#endif  // PCL_ROS__SURFACE__CONVEX_HULL_HPP_
