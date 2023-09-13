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
 * $Id: extract_polygonal_prism_data.h 35361 2011-01-20 04:34:49Z rusu $
 *
 */

#ifndef PCL_ROS__SEGMENTATION__EXTRACT_POLYGONAL_PRISM_DATA_HPP_
#define PCL_ROS__SEGMENTATION__EXTRACT_POLYGONAL_PRISM_DATA_HPP_

#include <message_filters/pass_through.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <pcl_ros/pcl_node.hpp>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <rclcpp/rclcpp.hpp>

namespace pcl_ros
{
namespace sync_policies = message_filters::sync_policies;

/** \brief @b ExtractPolygonalPrismData uses a set of point indices that represent a planar model, and together with
  * a given height, generates a 3D polygonal prism. The polygonal prism is then used to segment all points lying
  * inside it.
  *
  * An example of its usage is to extract the data lying within a set of 3D boundaries (e.g., objects supported by a plane).
  * \author Radu Bogdan Rusu
  */
class ExtractPolygonalPrismData : public PCLNode<pcl_msgs::msg::PointIndices>
{
public:
    /** \brief Disallow the empty constructor. */
  ExtractPolygonalPrismData() = delete;

  /** \brief ExtractPolygonalPrismData constructor
    * \param options node options
    */
  explicit ExtractPolygonalPrismData(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  /** \brief Initialize the node's parameters. */
  void init_parameters();

  /** \brief Lazy transport subscribe routine. */
  void subscribe();

  /** \brief Lazy transport subscribe routine. */
  void unsubscribe();

  /** \brief Input point cloud callback.
    * Because we want to use the same synchronizer object, we push back
    * empty elements with the same timestamp.
    */
  void input_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input);

  /** \brief The minimum allowed distance to the plane model value a point will be considered from. */
  double height_min_{0.0};

  /** \brief The maximum allowed distance to the plane model value a point will be considered from. */
  double height_max_{0.5};

  /** \brief The message filter subscriber for PointCloud2. */
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_hull_filter_;

  /** \brief Synchronized input, planar hull, and indices.*/
  std::shared_ptr<message_filters::Synchronizer<sync_policies::ExactTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2,
    pcl_msgs::msg::PointIndices>>> sync_input_hull_indices_e_;
  std::shared_ptr<message_filters::Synchronizer<sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2,
    sensor_msgs::msg::PointCloud2, pcl_msgs::msg::PointIndices>>> sync_input_hull_indices_a_;

  /** \brief Null passthrough filter, used for pushing empty elements in the
    * synchronizer */
  message_filters::PassThrough<pcl_msgs::msg::PointIndices> null_filter_;

  /** \brief Input point cloud callback. Used when \a use_indices is set.
    * \param cloud the pointer to the input point cloud
    * \param hull the pointer to the planar hull point cloud
    * \param indices the pointer to the input point cloud indices
    */
  void input_hull_indices_callback(
    const PointCloud2::ConstSharedPtr & cloud,
    const PointCloud2::ConstSharedPtr & hull,
    const PointIndices::ConstSharedPtr & indices);

private:
  /** \brief The PCL implementation used. */
  pcl::ExtractPolygonalPrismData<pcl::PointXYZ> impl_;

  /** \brief Internal mutex. */
  std::mutex mutex_;

  /** \brief Pointer to parameters callback handle. */
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;

  /** \brief Parameter callback
    * \param params parameter values to set
    */
  rcl_interfaces::msg::SetParametersResult
  config_callback(const std::vector<rclcpp::Parameter> & params);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace pcl_ros

#endif  // PCL_ROS__SEGMENTATION__EXTRACT_POLYGONAL_PRISM_DATA_HPP_
