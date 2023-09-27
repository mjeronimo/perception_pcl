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
 *  $Id: sac_segmentation_from_normals.hpp 35564 2011-01-27 07:32:12Z rusu $
 *
 */

#ifndef PCL_ROS__SEGMENTATION__SAC_SEGMENTATION_FROM_NORMALS_HPP_
#define PCL_ROS__SEGMENTATION__SAC_SEGMENTATION_FROM_NORMALS_HPP_

#include <memory>
#include <vector>

#include "pcl_ros/segmentation/sac_segmentation.hpp"

namespace pcl_ros
{
namespace sync_policies = message_filters::sync_policies;

////////////////////////////////////////////////////////////////////////////////////////////
/** \brief @b SACSegmentationFromNormals represents the PCL nodelet segmentation class for
 * Sample Consensus methods and models that require the use of surface normals for estimation.
 */
class SACSegmentationFromNormals : public SACSegmentation
{
public:
  /** \brief Disallow the empty constructor. */
  SACSegmentationFromNormals() = delete;

  /** \brief SACSegmentationFromNormals constructor
   * \param options node options
   */
  explicit SACSegmentationFromNormals(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  // typedef pcl::PointCloud<pcl::Normal> PointCloudN;
  // typedef boost::shared_ptr<PointCloudN> PointCloudNPtr;
  // typedef boost::shared_ptr<const PointCloudN> PointCloudNConstPtr;

  /** \brief Initialize the node's parameters. */
  void init_parameters();

  /** \brief Lazy transport subscribe routine. */
  void subscribe();

  /** \brief Lazy transport unsubscribe routine. */
  void unsubscribe();

  /** \brief Input point cloud callback.
   * \param cloud the pointer to the input point cloud
   * \param cloud_normals the pointer to the input point cloud normals
   * \param indices the pointer to the input point cloud indices
   */
  void input_normals_indices_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_normals,
    const pcl_msgs::msg::PointIndices::ConstSharedPtr & indices);

  /** \brief Input point cloud callback.
   * Because we want to use the same synchronizer object, we push back
   * empty elements with the same timestamp.
   */
  inline void input_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
  {
    pcl_msgs::msg::PointIndices indices;
    indices.header.stamp = cloud->header.stamp;
    nf_.add(std::make_shared<pcl_msgs::msg::PointIndices>(indices));
  }

  /** \brief Model callback
   * \param model the sample consensus model found
   */
  void axis_callback(const pcl_msgs::msg::ModelCoefficients::ConstSharedPtr & model);

  /** \brief Parameter callback
   * \param params parameter values to set
   */
  rcl_interfaces::msg::SetParametersResult set_parameters_callback(
    const std::vector<rclcpp::Parameter> & params);

  /** \brief Pointer to parameters callback handle. */
  OnSetParametersCallbackHandle::SharedPtr set_parameters_callback_handle_;

  /** \brief The relative weight (between 0 and 1) to give to the angular distance (0 to pi/2)
   *  between point normals and the plane normal.
   */
  double normal_distance_weight_{0.1};

  /** \brief The normals PointCloud subscriber filter. */
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_normals_filter_;

  /** \brief The input PointCloud subscriber. */
  rclcpp::Subscription<pcl_msgs::msg::ModelCoefficients>::SharedPtr sub_axis_;

  /** \brief Null passthrough filter, used for pushing empty elements in the
   * synchronizer */
  message_filters::PassThrough<pcl_msgs::msg::PointIndices> nf_;

  /** \brief Synchronized input, normals, and indices.*/
  std::shared_ptr<message_filters::Synchronizer<sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2, pcl_msgs::msg::PointIndices>>>
    sync_input_normals_indices_a_;
  std::shared_ptr<message_filters::Synchronizer<sync_policies::ExactTime<
    sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2, pcl_msgs::msg::PointIndices>>>
    sync_input_normals_indices_e_;

  /** \brief The PCL implementation used. */
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> impl_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace pcl_ros

#endif  // PCL_ROS__SEGMENTATION__SAC_SEGMENTATION_FROM_NORMALS_HPP_
