//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_MAGNETIC_H
#define HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_MAGNETIC_H

#include <gazebo/common/Plugin.hh>

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/MagneticField.h>
#include <hector_gazebo_plugins/sensor_model.h>
#include <hector_gazebo_plugins/update_timer.h>

#include <dynamic_reconfigure/server.h>

namespace gazebo
{

class GazeboRosMagnetic : public ModelPlugin
{
public:
  GazeboRosMagnetic();
  virtual ~GazeboRosMagnetic();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Reset();
  virtual void Update();

private:
  /// \brief The parent World
  physics::WorldPtr world;

  /// \brief The link referred to by this plugin
  physics::LinkPtr link;

  ros::NodeHandle* node_handle_;
  ros::Publisher publisher_;

  //geometry_msgs::Vector3Stamped magnetic_field_;
  sensor_msgs::MagneticField magnetic_field_;
#if (GAZEBO_MAJOR_VERSION >= 8)
  ignition::math::Vector3d magnetic_field_world_;
#else
  gazebo::math::Vector3 magnetic_field_world_;
#endif

  std::string namespace_;
  std::string topic_;
  std::string link_name_;
  std::string frame_id_;

  double magnitude_;
  double reference_heading_;
  double declination_;
  double inclination_;

  SensorModel3 sensor_model_;

  UpdateTimer updateTimer;
  event::ConnectionPtr updateConnection;

  boost::shared_ptr<dynamic_reconfigure::Server<SensorModelConfig> > dynamic_reconfigure_server_;
};

} // namespace gazebo

#endif // HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_MAGNETIC_H
