/*
 * Copyright (C) 2012-2013 Simon Lynen, ASL, ETH Zurich, Switzerland
 * You can contact the author at <slynen at ethz dot ch>
 * Copyright (C) 2011-2012 Stephan Weiss, ASL, ETH Zurich, Switzerland
 * You can contact the author at <stephan dot weiss at ieee dot org>
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
#include <msf_core/eigen_utils.h>
#ifndef ATTITUDE_SENSORHANDLER_HPP_
#define ATTITUDE_SENSORHANDLER_HPP_

namespace msf_attitude_sensor {
AttitudeSensorHandler::AttitudeSensorHandler(
        msf_core::MSF_SensorManager<msf_updates::EKFState>& meas,
        std::string topic_namespace, std::string parameternamespace)
    : SensorHandler<msf_updates::EKFState>(meas, topic_namespace,
                                           parameternamespace),
      n_q_(1e-6) {
    ros::NodeHandle pnh("~/attitude_sensor");
    ros::NodeHandle nh("msf_updates");

    pnh.param("enable_mah_outlier_rejection", enable_mah_outlier_rejection_, true);
    pnh.param("mah_threshold", mah_threshold_, msf_core::kDefaultMahThreshold_);

    subIMU_ =
            nh.subscribe<sensor_msgs::Imu>
            ("attitude", 20, &AttitudeSensorHandler::MeasurementCallback, this);

}

void AttitudeSensorHandler::SetNoises(double n_q) {
    n_q_ = n_q;
}

void AttitudeSensorHandler::MeasurementCallback(
        const sensor_msgs::ImuConstPtr & msg) {

    received_first_measurement_ = true;

    this->SequenceWatchDog(msg->header.seq, subIMU_.getTopic());
    MSF_INFO_STREAM_ONCE(
                "*** attitude sensor got first measurement from topic "
                << this->topic_namespace_ << "/" << subIMU_.getTopic()
                << " ***");

    if (msg->header.seq%5!=0)
    {
        return;
    }
    shared_ptr<attitude_measurement::AttitudeMeasurement> meas(
                new attitude_measurement::AttitudeMeasurement(
                    n_q_, true, this->sensorID, enable_mah_outlier_rejection_,
                    mah_threshold_));
    meas->MakeFromSensorReading(msg, msg->header.stamp.toSec());

    z_q_ = meas->z_q_;  // Store this for the init procedure.

    this->manager_.msf_core_->AddMeasurement(meas);
}
}  // namespace msf_attitude_sensor
#endif  // ATTITUDE_SENSORHANDLER_HPP_
