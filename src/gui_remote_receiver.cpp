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

#include <tf_visual_tools/gui_remote_receiver.hpp>

namespace tf_visual_tools
{

  TFRemoteReceiver::TFRemoteReceiver()
      : Node("tf_remote_receiver")
  {
    // Initialize tf2_ros::Buffer properly
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock()); // Using the node's clock

    // Create a TransformListener to listen to TF updates
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(this->get_logger(), "Remote receiver initialized");

    create_tf_pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("/rviz_tf_create", 10);
    remove_tf_pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("/rviz_tf_remove", 10);
    update_tf_pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("/rviz_tf_update", 10);
  }

  void TFRemoteReceiver::createTF(const geometry_msgs::msg::TransformStamped &create_tf_msg)
  {
    create_tf_pub_->publish(create_tf_msg);
  }

  void TFRemoteReceiver::removeTF(const geometry_msgs::msg::TransformStamped &remove_tf_msg)
  {
    remove_tf_pub_->publish(remove_tf_msg);
  }

  void TFRemoteReceiver::updateTF(const geometry_msgs::msg::TransformStamped &update_tf_msg)
  {
    update_tf_pub_->publish(update_tf_msg);
  }

  std::vector<std::string> TFRemoteReceiver::getTFNames()
  {
    tf_buffer_->_getFrameStrings(tf_names_);
    return tf_names_;
  }

  void TFRemoteReceiver::addIMarkerMenuPub(int menu_index, const std::string &menu_name)
  {
    // replace ' ' with '_'
    std::string modified_menu_name = menu_name;
    for (std::size_t i = 0; i < modified_menu_name.size(); i++)
    {
      if (modified_menu_name[i] == ' ')
        modified_menu_name[i] = '_';
    }

    std::string new_topic_name = "/imarker/" + modified_menu_name;
    auto new_pub = this->create_publisher<std_msgs::msg::Bool>(new_topic_name, 1);

    std::pair<int, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> new_menu_pub(menu_index, new_pub);
    menu_pubs_.push_back(new_menu_pub);
  }

  void TFRemoteReceiver::publishIMarkerMenuSelection(int menu_index)
  {
    std_msgs::msg::Bool msg;
    msg.data = true;

    for (std::size_t i = 0; i < menu_pubs_.size(); i++)
    {
      if (menu_pubs_[i].first == menu_index)
      {
        menu_pubs_[i].second->publish(msg);
        break;
      }
    }
  }

} // end namespace tf_visual_tools