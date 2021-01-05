// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK Lab
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
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
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
 *********************************************************************/

// #include <cv_bridge/cv_bridge.h>
#include <jsk_recognition_utils/pcl_ros_util.h>
// #include <jsk_topic_tools/rosparam_utils.h>
// #include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <pcl/filters/frustum_culling.h>
#include <jsk_pcl_ros/frustum_culling.h>

#define M_PI 3.14159265358979

void jsk_pcl_ros::FrustumCulling::onInit () {

  NODELET_INFO("[%s::onInit]", getName().c_str());
  ConnectionBasedNodelet::onInit();
  tf_listener_ = TfListenerSingleton::getInstance();

  pnh_->param("use_asynchronous", use_asynchronous, false);
  ROS_INFO("use_asynchronous : %d", use_asynchronous);

  pnh_->param("use_approximate", use_approximate, false);
  pnh_->param("negative", negative_, false);
  ROS_INFO("use_approximate : %d", use_approximate);

  pnh_->param("info_throttle", info_throttle_, 0);
  info_counter_ = 0;
  pnh_->param("max_queue_size", max_queue_size_, 3);

  // maybe below queue_size can always be 1,
  // but we set max_queue_size_ for backward compatibility.
  pnh_->param("max_pub_queue_size", max_pub_queue_size_, max_queue_size_);
  pnh_->param("max_sub_queue_size", max_sub_queue_size_, max_queue_size_);

  pnh_->param("near_plane_distance", near_plane_distance_, 0.0001);
  pnh_->param("far_plane_distance" , far_plane_distance_, 100.0);
  pnh_->param("custom_fov_h" , custom_fov_h_, -1.0);
  pnh_->param("custom_fov_v" , custom_fov_v_, -1.0);
  pnh_->param("camera_link_frame" , camera_link_frame_, std::string("camera_link"));

  pub_cloud_ = advertise<PointCloud>(*pnh_, "output_cloud", max_pub_queue_size_);

  srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
  dynamic_reconfigure::Server<Config>::CallbackType f =
    boost::bind (
      &FrustumCulling::configCallback, this, _1, _2);
  srv_->setCallback (f);

  onInitPostProcess();
}

void jsk_pcl_ros::FrustumCulling::subscribe() {
  if (use_asynchronous) {
    sub_as_info_ = pnh_->subscribe<sensor_msgs::CameraInfo> ("info", max_sub_queue_size_,
                                                             &FrustumCulling::callback_info, this);
    sub_as_cloud_ = pnh_->subscribe<sensor_msgs::PointCloud2> ("input", max_sub_queue_size_,
                                                               &FrustumCulling::callback_cloud, this);
  } else {
    sub_info_.subscribe(*pnh_, "info", max_sub_queue_size_);
    sub_cloud_.subscribe(*pnh_, "input", max_sub_queue_size_);

    if (use_approximate) {
      sync_inputs_a_ = boost::make_shared <message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> > > (max_queue_size_);
      sync_inputs_a_->connectInput (sub_info_, sub_cloud_);
      sync_inputs_a_->registerCallback (bind (&FrustumCulling::callback_sync, this, _1, _2));
    } else {
      sync_inputs_e_ = boost::make_shared <message_filters::Synchronizer<message_filters::sync_policies::ExactTime<sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> > > (max_queue_size_);
      sync_inputs_e_->connectInput (sub_info_, sub_cloud_);
      sync_inputs_e_->registerCallback (bind (&FrustumCulling::callback_sync, this, _1, _2));
    }
  }
}

void jsk_pcl_ros::FrustumCulling::unsubscribe() {
  if (use_asynchronous) {
    sub_as_info_.shutdown();
    sub_as_cloud_.shutdown();
  }
  else {
    sub_info_.unsubscribe();
    sub_cloud_.unsubscribe();
  }
}

void jsk_pcl_ros::FrustumCulling::configCallback(Config &config, uint32_t level)
{
  boost::mutex::scoped_lock lock(mutex_);
  near_plane_distance_ = config.near_plane_distance;
  far_plane_distance_  = config.far_plane_distance;
  custom_fov_h_        = config.custom_fov_h;
  custom_fov_v_        = config.custom_fov_v;
}

void jsk_pcl_ros::FrustumCulling::callback_sync(const sensor_msgs::CameraInfoConstPtr& info,
                                                   const sensor_msgs::PointCloud2ConstPtr& pcloud2) {
  ROS_DEBUG("FrustumCulling::callback_sync");
  publish_points(info, pcloud2);
}

void jsk_pcl_ros::FrustumCulling::callback_cloud(const sensor_msgs::PointCloud2ConstPtr& pcloud2) {
  ROS_DEBUG("FrustumCulling::callback_cloud");
  boost::mutex::scoped_lock lock(this->mutex_points);
  points_ptr_ = pcloud2;
}

void jsk_pcl_ros::FrustumCulling::callback_info(const sensor_msgs::CameraInfoConstPtr& info) {
  ROS_DEBUG("FrustumCulling::callback_info");
  boost::mutex::scoped_lock lock(this->mutex_points);
  if( info_counter_++ >= info_throttle_ ) {
    info_counter_ = 0;
  } else {
    return;
  }
  if (points_ptr_) {
    publish_points(info, points_ptr_);
  }
}

void jsk_pcl_ros::FrustumCulling::publish_points(const sensor_msgs::CameraInfoConstPtr& info,
                                                    const sensor_msgs::PointCloud2ConstPtr& pcloud2) {
  sensor_msgs::PointCloud2Ptr pcloud2_ptr(new sensor_msgs::PointCloud2(*pcloud2));

  ROS_DEBUG("FrustumCulling::publish_points");
  if (!pcloud2_ptr)  return;

  int width = info->width;
  int height = info->height;
  float fx = info->P[0];
  float cx = info->P[2];
  float tx = info->P[3];
  float fy = info->P[5];
  float cy = info->P[6];

  double fov_h = (2 * atan2(0.5 * width,  fx)) / M_PI * 180;
  double fov_v = (2 * atan2(0.5 * height, fy)) / M_PI * 180;

  if (custom_fov_h_ > 0) fov_h = custom_fov_h_;
  if (custom_fov_v_ > 0) fov_v = custom_fov_v_;

  ROS_DEBUG("Horizontal FOV: %.2lf", fov_h);
  ROS_DEBUG("Vertical FOV:   %.2lf", fov_v);

  Eigen::Affine3f sensorPose;
  {
    tf::StampedTransform transform;
    try {
      // FrustumCulling's coords are X front, Y left, Z up
      // camera optical frame (X right, Y down, Z front) cannot be used

      // tf_listener_->waitForTransform(pcloud2_ptr->header.frame_id,
      //                                info->header.frame_id,
      //                                info->header.stamp,
      //                                ros::Duration(0.001));
      // tf_listener_->lookupTransform(pcloud2_ptr->header.frame_id,
      //                               info->header.frame_id,
      //                               info->header.stamp, transform);
      tf_listener_->waitForTransform(pcloud2_ptr->header.frame_id,
                                     camera_link_frame_,
                                     info->header.stamp,
                                     ros::Duration(1.0));
      tf_listener_->lookupTransform(pcloud2_ptr->header.frame_id,
                                    camera_link_frame_,
                                    info->header.stamp, transform);

    }
    catch ( std::runtime_error e ) {
      ROS_ERROR("%s",e.what());
      return;
    }

    tf::Vector3 p = transform.getOrigin();
    tf::Quaternion q = transform.getRotation();
    sensorPose = (Eigen::Affine3f)Eigen::Translation3f(p.getX(), p.getY(), p.getZ());
    Eigen::Quaternion<float> rot(q.getW(), q.getX(), q.getY(), q.getZ());
    sensorPose = sensorPose * rot;

    if (tx != 0.0) {
      Eigen::Affine3f trans = (Eigen::Affine3f)Eigen::Translation3f(-tx/fx , 0, 0);
      sensorPose = sensorPose * trans;
    }
    ROS_INFO("%f %f %f %f %f %f %f %f %f, %f %f %f",
             sensorPose(0,0), sensorPose(0,1), sensorPose(0,2),
             sensorPose(1,0), sensorPose(1,1), sensorPose(1,2),
             sensorPose(2,0), sensorPose(2,1), sensorPose(2,2),
             sensorPose(0,3), sensorPose(1,3), sensorPose(2,3));
  }

  PointCloud::Ptr cloud_in (new PointCloud);
  pcl::fromROSMsg(*pcloud2, *cloud_in);

  pcl::FrustumCulling<Point> fc;
  fc.setInputCloud (cloud_in);
  fc.setVerticalFOV (fov_v);
  fc.setHorizontalFOV (fov_h);
  fc.setNearPlaneDistance (near_plane_distance_);
  fc.setFarPlaneDistance (far_plane_distance_);

  // fc.setCameraPose (Eigen::Matrix4f::Identity());
  fc.setCameraPose (sensorPose.matrix());
  fc.setNegative(negative_);

  PointCloud cloud_out;
  fc.filter (cloud_out);

  cloud_out.header = cloud_in->header;
  cloud_out.width = cloud_out.points.size();
  cloud_out.height = 1;
  cloud_out.is_dense = true;
  pub_cloud_.publish(boost::make_shared<PointCloud>(cloud_out));

}  // namespace jsk_pcl_ros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::FrustumCulling, nodelet::Nodelet);
