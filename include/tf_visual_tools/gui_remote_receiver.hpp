/****************************************************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Copyright 2017, Andy McEvoy
 *
 *  Redistribution and use in source and binary forms, with or without modification, are permitted
 *  provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this list of conditions
 *  and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice, this list of
 *  conditions and the following disclaimer in the documentation and/or other materials provided
 *  with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its contributors may be used to
 *  endorse or promote products derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 *  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 *  FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 *  IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 *  OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************************************/
/*
 * Author(s) : Andy McEvoy ( mcevoy.andy@gmail.com )
 * Desc      : implementation of TFRemoteReceiver. See H file for documentation.
 * Created   : 09 - May - 2017
 */

/*
 * Copyright (c) 2025 Fraunhofer Institute for Integrated Circuits
 * Changes to project: Ported from ROS 1 to ROS 2
 * Author(s) : Gokhul Raj Ravikumar (gokhulraj6200@gmail.com)
 * Maintainer(s) : Sebastian Zarnack (sebastian.zarnack@iis.fraunhofer.de)
 * Created   : 2025 - April - 29
 */

#ifndef TF_REMOTE_RECEIVER_H
#define TF_REMOTE_RECEIVER_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <vector>
#include <string>

namespace tf_visual_tools
{

  class TFRemoteReceiver : public rclcpp::Node
  {
  public:
    // singleton
    static TFRemoteReceiver &getInstance()
    {
      static TFRemoteReceiver instance;
      return instance;
    }

    void createTF(const geometry_msgs::msg::TransformStamped &create_tf_msg);
    void removeTF(const geometry_msgs::msg::TransformStamped &remove_tf_msg);
    void updateTF(const geometry_msgs::msg::TransformStamped &update_tf_msg);
    void addIMarkerMenuPub(int menu_index, const std::string &menu_name);
    void publishIMarkerMenuSelection(int menu_index);

    std::vector<std::string> getTFNames();

  private:
    TFRemoteReceiver();

    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr create_tf_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr remove_tf_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr update_tf_pub_;

    std::vector<std::string> tf_names_;
    std::vector<std::pair<int, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr>> menu_pubs_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  }; // end class TFRemoteReceiver

} // end namespace tf_visual_tools

#endif
