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

#ifndef JSK_PCL_ROS_FRUSTUM_CULLING_H_
#define JSK_PCL_ROS_FRUSTUM_CULLING_H_

#include <pcl_ros/pcl_nodelet.h>
#include <pcl_ros/transforms.h>

#include <pluginlib/class_list_macros.h>
#include <dynamic_reconfigure/server.h>
#include <jsk_pcl_ros/FrustumCullingConfig.h>

#if ( PCL_MAJOR_VERSION >= 1 && PCL_MINOR_VERSION >= 5 )
#include <pcl/common/transforms.h>
#else
#include <pcl/common/transform.h>
#endif

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/thread/mutex.hpp>

#include <jsk_topic_tools/connection_based_nodelet.h>
#include "jsk_pcl_ros/tf_listener_singleton.h"
#include <jsk_recognition_utils/geo_util.h>

namespace jsk_pcl_ros
{
  class FrustumCulling : public jsk_topic_tools::ConnectionBasedNodelet
  {
  protected:
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cloud_;
    ros::Subscriber sub_as_info_;
    ros::Subscriber sub_as_cloud_;
    ros::Publisher pub_cloud_;
    typedef jsk_pcl_ros::FrustumCullingConfig Config;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    boost::mutex mutex_;

    sensor_msgs::PointCloud2ConstPtr points_ptr_;

    boost::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ExactTime<sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> > > sync_inputs_e_;
    boost::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> > > sync_inputs_a_;
    boost::mutex mutex_points;
    bool use_asynchronous;
    bool use_approximate;
    bool negative_;
    int info_throttle_;
    int info_counter_;
    int max_queue_size_;
    int max_pub_queue_size_;
    int max_sub_queue_size_;
    std::string camera_link_frame_;
    tf::TransformListener* tf_listener_;
    double near_plane_distance_;
    double far_plane_distance_;
    double custom_fov_h_;
    double custom_fov_v_;
    typedef pcl::PointXYZ Point;
    typedef pcl::PointCloud< Point > PointCloud;

    void onInit();

    void callback_sync(const sensor_msgs::CameraInfoConstPtr& info,
                       const sensor_msgs::PointCloud2ConstPtr& pcloud2);

    void callback_cloud(const sensor_msgs::PointCloud2ConstPtr& pcloud2);

    void callback_info(const sensor_msgs::CameraInfoConstPtr& info);

    void publish_points(const sensor_msgs::CameraInfoConstPtr& info,
                        const sensor_msgs::PointCloud2ConstPtr& pcloud2);

    void subscribe();
    void unsubscribe();
    void configCallback (Config &config, uint32_t level);
  public:
  };
}

#endif
