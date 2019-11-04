/*
 * Copyright (C) 2012-2013 Simon Lynen, ASL, ETH Zurich, Switzerland
 * You can contact the author at <slynen at ethz dot ch>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef ATTITUDE_SENSOR_H

#include <queue>

#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Imu.h>
#include <msf_core/msf_sensormanagerROS.h>

#include <msf_updates/attitude_sensor_handler/attitude_measurement.h>

namespace msf_attitude_sensor {
class AttitudeSensorHandler : public msf_core::SensorHandler<
    typename msf_updates::EKFState> {
 private:
  Eigen::Quaternion<double> z_q_;  ///< attitude measurement.
  double n_q_;  ///< attitude measurement noise.
  ros::Subscriber subIMU_;
  void MeasurementCallback(const sensor_msgs::ImuConstPtr & msg);
 public:
  AttitudeSensorHandler(
      msf_core::MSF_SensorManager<msf_updates::EKFState>& meas,
      std::string topic_namespace, std::string parameternamespace);
  // Used for the init.
  Eigen::Quaternion<double> GetAttitudeMeasurement() {
    return z_q_;
  }
  // Setters for configure values.
  void SetNoises(double n_q);
};
}  // namespace msf_attitude_sensor
#include "implementation/attitude_sensorhandler.hpp"
#endif  // POSE_SENSOR_H
