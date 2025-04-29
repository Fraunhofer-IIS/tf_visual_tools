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

#ifndef TF_KEYBOARD_CAL_GUI_H
#define TF_KEYBOARD_CAL_GUI_H

#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <tf_visual_tools/gui_remote_receiver.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/interactive_marker_update.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <Eigen/Dense>
#include <QLabel>
#include <QPushButton>
#include <QTabWidget>
#include <QtGui>
#include <QCheckBox>
#include <QLineEdit>
#include <QComboBox>
#include <QFrame>
#include <QDoubleValidator>
#include <QDir>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QFileDialog>
#include <QObject>
#include <QWidget>
#include <QKeyEvent>
#include <stdio.h>
#include <fstream>
#include <boost/algorithm/string/trim.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <memory>

namespace tf_visual_tools
{
  struct tf_data
  {
    std::size_t id_;
    std::string from_;
    std::string to_;
    bool imarker_;
    QString name_;
    double values_[6];

    geometry_msgs::msg::TransformStamped getTFMsg();
  };
  /**
   * Tab for manipulating TFs
   */
  class manipulateTFTab : public QWidget
  {
    Q_OBJECT

  public:
    explicit manipulateTFTab(QWidget *parent = 0);
    void updateTFList();
    static void updateTFValues(int list_index, geometry_msgs::msg::Pose pose);

  protected Q_SLOTS:
    void setQLineValues(int item_id);
    void editTFTextValue(QString text);

    void incrementDOF();
    void incrementDOF(int dof, double sign);

    void setXYZDelta(QString text);
    void setXYZDelta(double xyz_delta);

    void setRPYDelta(QString text);
    void setRPYDelta(double rpy_delta);

    void keyPressEvent(QKeyEvent *);

  private:
    void updateTFValues(int dof, double value);

    static constexpr double MAX_XYZ_DELTA = 100.0;
    static constexpr double MAX_RPY_DELTA = 360.0;

    double xyz_delta_;
    double rpy_delta_;

    QComboBox *active_tfs_;
    QLineEdit *xyz_delta_box_;
    QLineEdit *rpy_delta_box_;

    std::vector<QLineEdit *> dof_qline_edits_;

    TFRemoteReceiver *remote_receiver_;
  };
  /**
   * Tab for creating & deleting TFs
   */
  class createTFTab : public QWidget
  {
    Q_OBJECT

  public:
    explicit createTFTab(QWidget *parent = 0);
    void updateFromList();
    manipulateTFTab *manipulate_tab_;
    void createNewIMarker(tf_data new_tf, bool has_menu);

  protected Q_SLOTS:
    void createNewTF();
    void removeTF();

    void fromTextChanged(QString text);
    void toTextChanged(QString text);

  private:
    void processIMarkerFeedback(
        const std::shared_ptr<const visualization_msgs::msg::InteractiveMarkerFeedback> &feedback);

    bool menu_handler_set_;

    std::string from_tf_name_;
    std::string to_tf_name_;

    std::size_t id_;

    QComboBox *from_;
    QLineEdit *to_;

    QCheckBox *add_imarker_;
    QCheckBox *add_imarker_menu_;

    QPushButton *create_tf_btn_;
    QPushButton *remove_tf_btn_;

    QComboBox *active_tfs_;

    TFRemoteReceiver *remote_receiver_;
  };
  /**
   * Tab for saving and loading TFs
   */
  class saveLoadTFTab : public QWidget
  {
    Q_OBJECT

  public:
    explicit saveLoadTFTab(QWidget *parent = 0);
    createTFTab *create_tab_;

  protected Q_SLOTS:
    void load();
    void save();

  private:
    QPushButton *load_btn_;
    QPushButton *save_btn_;

    std::string full_save_path_;
    std::string full_load_path_;

    TFRemoteReceiver *remote_receiver_;
  };
  /**
   * Main class
   */
  class TFVisualTools
      : public rviz_common::Panel
  {
    Q_OBJECT
  public:
    explicit TFVisualTools(QWidget *parent = 0);
    ~TFVisualTools() override;
    void onInitialize() override;
  protected Q_SLOTS:
    void updateTabData(int);

  protected:
    std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_ptr_;

  private:
    QTabWidget *tab_widget_;

    createTFTab *new_create_tab_;
    manipulateTFTab *new_manipulate_tab_;
    saveLoadTFTab *new_save_load_tab_;
  };
} // namespace tf_visual_tools

#endif // TF_KEYBOARD_CAL_GUI_H