/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/CompressedImage.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/Config.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <nodelet_topic_tools/nodelet_throttle.h>
namespace camera_throttle_nodelets{
typedef nodelet_topic_tools::NodeletThrottle<sensor_msgs::CompressedImage> NodeletThrottleCompressedImage;
typedef nodelet_topic_tools::NodeletThrottle<dynamic_reconfigure::ConfigDescription> NodeletThrottleParameterDescription;
typedef nodelet_topic_tools::NodeletThrottle<dynamic_reconfigure::Config> NodeletThrottleParameterUpdates;
typedef nodelet_topic_tools::NodeletThrottle<sensor_msgs::CameraInfo> NodeletThrottleCameraInfo;
typedef nodelet_topic_tools::NodeletThrottle<sensor_msgs::Image> NodeletThrottleImage;
}

PLUGINLIB_EXPORT_CLASS(camera_throttle_nodelets::NodeletThrottleCompressedImage, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(camera_throttle_nodelets::NodeletThrottleParameterDescription, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(camera_throttle_nodelets::NodeletThrottleParameterUpdates, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(camera_throttle_nodelets::NodeletThrottleImage, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(camera_throttle_nodelets::NodeletThrottleCameraInfo, nodelet::Nodelet);
