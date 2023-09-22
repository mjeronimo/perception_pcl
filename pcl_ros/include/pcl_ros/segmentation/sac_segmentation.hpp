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
 * $Id: sac_segmentation.h 35564 2011-01-27 07:32:12Z rusu $
 *
 */

#ifndef PCL_ROS__SEGMENTATION__SAC_SEGMENTATION_HPP_
#define PCL_ROS__SEGMENTATION__SAC_SEGMENTATION_HPP_

#include <message_filters/pass_through.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <string>
#include <vector>

#include "pcl_ros/pcl_node.hpp"

namespace pcl_ros
{
namespace sync_policies = message_filters::sync_policies;

////////////////////////////////////////////////////////////////////////////////////////////
/** \brief @b SACSegmentation represents the Nodelet segmentation class for Sample Consensus
 * methods and models, in the sense that it just creates a Nodelet wrapper for generic-purpose
 * SAC-based segmentation.
 * \author Radu Bogdan Rusu
 */
class SACSegmentation : public PCLNode<pcl_msgs::msg::PointIndices>
{
public:
  /** \brief Disallow the empty constructor. */
  SACSegmentation() = delete;

  /** \brief SACSegmentation constructor
   * \param options node options
   */
  explicit SACSegmentation(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  /** \brief Initialize the node's parameters. */
  void init_parameters();

  /** \brief Lazy transport subscribe routine. */
  void subscribe();

  /** \brief Lazy transport unsubscribe routine. */
  void unsubscribe();

  /** \brief Input point cloud callback. Used when \a use_indices is set.
   * \param cloud the pointer to the input point cloud
   * \param indices the pointer to the input point cloud indices
   */
  void input_indices_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
    const pcl_msgs::msg::PointIndices::ConstSharedPtr & indices);

  /** \brief Input point cloud callback. Used when \a use_indices is not set.
   * \param cloud the pointer to the input point cloud
   */
  void input_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
  {
    input_indices_callback(cloud, pcl_msgs::msg::PointIndices::ConstSharedPtr());
  }

  /** \brief Indices callback. Used when \a latched_indices_ is set.
   * \param indices the pointer to the input point cloud indices
   */
  inline void latched_indices_callback(const pcl_msgs::msg::PointIndices::ConstSharedPtr & indices)
  {
    indices_ = *indices;
  }

  /** \brief Input callback. Used when \a latched_indices_ is set.
   * \param input the pointer to the input point cloud
   */
  inline void latched_input_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input)
  {
    indices_.header = input->header;
    pcl_msgs::msg::PointIndices::ConstSharedPtr indices;
    indices.reset(new PointIndices(indices_));
    nf_pi_.add(indices);
  }

  /** \brief Parameter callback
   * \param params parameter values to set
   */
  rcl_interfaces::msg::SetParametersResult set_parameters_callback(
    const std::vector<rclcpp::Parameter> & params);

  /** \brief Pointer to parameters callback handle. */
  OnSetParametersCallbackHandle::SharedPtr set_parameters_callback_handle_;

  /** \brief Internal mutex. */
  std::mutex mutex_;

  /** \brief The axis along which we need to search for a model perpendicular to. */
  std::vector<double> axis_{0.0, 0.0, 0.0};

  /** \brief Distance to the model threshold (user given parameter). */
  double distance_threshold_{0.02};

  /** \brief The maximum allowed difference between the model normal and the given axis, _in
   * radians_. */
  double eps_angle_{0.17};

  /** \brief The input TF frame the data should be transformed into,
   * if input.header.frame_id is different.
   */
  std::string input_frame_;

  /** \brief Set to true if the indices topic is latched.
   *
   * If use_indices_ is true, the ~input and ~indices topics generally must
   * be synchronised in time. By setting this flag to true, the most recent
   * value from ~indices can be used instead of requiring a synchronised
   * message.
   **/
  bool latched_indices_{false};

  /** \brief Maximum number of iterations before giving up. */
  int max_iterations_{50};

  /** \brief The type of sample consensus method to use. */
  int method_type_{pcl::SAC_RANSAC};

  /** \brief The minimum number of inliers a model must have in order to be considered valid. */
  int min_inliers_{0};

  /** \brief The type of model to use. */
  int model_type_{pcl::SacModel::SACMODEL_PLANE};

  /** \brief Set to true if a coefficient refinement is required. */
  bool optimize_coefficients_{true};

  /** \brief The output TF frame the data should be transformed into,
   * if input.header.frame_id is different.
   */
  std::string output_frame_;

  /** \brief Desired probability of choosing at least one sample free from outliers. */
  double probability_{0.99};

  /** \brief The minimum radius limits for the model. Applicable to all models that estimate a
   * radius. */
  double radius_min_{0.0};

  /** \brief The maximum radius limits for the model. Applicable to all models that estimate a
   * radius. */
  double radius_max_{0.05};

  /** \brief The output PointIndices publisher. */
  rclcpp::Publisher<pcl_msgs::msg::PointIndices>::SharedPtr pub_indices_;

  /** \brief The output ModelCoefficients publisher. */
  rclcpp::Publisher<ModelCoefficients>::SharedPtr pub_model_;

  /** \brief The input PointCloud subscriber. */
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_input_;

  /** \brief Null passthrough filter, used for pushing empty elements in the
   * synchronizer */
  message_filters::PassThrough<pcl_msgs::msg::PointIndices> nf_pi_;

  /** \brief Pointer to a set of indices stored internally.
   * (used when \a latched_indices_ is set).
   */
  pcl_msgs::msg::PointIndices indices_;

  /** \brief Synchronized input, and indices.*/
  boost::shared_ptr<message_filters::Synchronizer<
    sync_policies::ExactTime<sensor_msgs::msg::PointCloud2, pcl_msgs::msg::PointIndices>>>
    sync_input_indices_e_;
  boost::shared_ptr<message_filters::Synchronizer<
    sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, pcl_msgs::msg::PointIndices>>>
    sync_input_indices_a_;

  /** \brief The PCL implementation used. */
  pcl::SACSegmentation<pcl::PointXYZ> impl_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace pcl_ros

#endif  // PCL_ROS__SEGMENTATION__SAC_SEGMENTATION_HPP_
