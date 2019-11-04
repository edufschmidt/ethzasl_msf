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
#ifndef ATTITUDE_MEASUREMENT_HPP_
#define ATTITUDE_MEASUREMENT_HPP_

#include <msf_core/msf_measurement.h>
#include <msf_core/msf_core.h>

namespace attitude_measurement {
enum {
    nMeasurements = 3
};
/**
 * \brief A measurement as provided by a q sensor.
 */
typedef msf_core::MSF_Measurement<sensor_msgs::Imu,
Eigen::Matrix<double, nMeasurements, nMeasurements>, msf_updates::EKFState> AttitudeMeasurementBase;
struct AttitudeMeasurement : public AttitudeMeasurementBase {
private:
    typedef AttitudeMeasurementBase Measurement_t;
    typedef Measurement_t::Measurement_ptr measptr_t;

    virtual void MakeFromSensorReadingImpl(measptr_t msg) {
        Eigen::Matrix<double, nMeasurements,
                msf_core::MSF_Core<msf_updates::EKFState>::nErrorStatesAtCompileTime> H_old;
        Eigen::Matrix<double, nMeasurements, 1> r_old;

        H_old.setZero();

        // Get measurements.
        z_q_ = Eigen::Quaternion<double>(msg->orientation.w,
                                         msg->orientation.x,
                                         msg->orientation.y,
                                         msg->orientation.z);

        const double s_zp = n_q_ * n_q_;
        R_ = (Eigen::Matrix<double, nMeasurements, 1>() << s_zp).finished()
                .asDiagonal();
    }
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Quaternion<double> z_q_;  /// q measurement.
    double n_q_;  /// q measurement noise.

    typedef msf_updates::EKFState EKFState_T;
    typedef EKFState_T::StateDefinition_T StateDefinition_T;
    virtual ~AttitudeMeasurement() {}
    AttitudeMeasurement(double n_q, bool isabsoluteMeasurement, int sensorID,
                        bool enable_mah_outlier_rejection, double mah_threshold)
        : AttitudeMeasurementBase(isabsoluteMeasurement, sensorID,
                                  enable_mah_outlier_rejection, mah_threshold),
          n_q_(n_q) {}
    virtual std::string Type() { return "attitude"; }
    /**
   * The method called by the msf_core to apply the measurement represented by
   * this object.
   */
    virtual void Apply(shared_ptr<EKFState_T> non_const_state,
                       msf_core::MSF_Core<EKFState_T>& core) {
        // Init variables.
        Eigen::Matrix<double, nMeasurements,
                msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime> H_old;
        Eigen::Matrix<double, nMeasurements, 1> r_old;

        H_old.setZero();

        if (non_const_state->time == msf_core::constants::INVALID_TIME) {
            MSF_WARN_STREAM(
                        "Apply attitude update was called with an invalid state.");
            return;  // Early abort.
        }

        const EKFState_T& state = *non_const_state;

        enum {
            idx_q = msf_tmp::GetStartIndex<EKFState_T::StateSequence_T,
            typename msf_tmp::GetEnumStateType<EKFState_T::StateSequence_T,
            StateDefinition_T::q>::value,
            msf_tmp::CorrectionStateLengthForType>::value,

        };

        // Construct H matrix.
        // Attitude:
        Eigen::Matrix<double, 3, 3> C_q = state.Get<StateDefinition_T::q>().toRotationMatrix();

        H_old.block<3, 3>(0, idx_q) = Eigen::Matrix<double, 3, 3>::Identity().eval();  // q

        // Construct residuals.
        // Attitude.
        Eigen::Quaternion<double> q_err;
        q_err = (state.Get<StateDefinition_T::q>()).conjugate() * z_q_;
        r_old.block<3, 1>(0, 0) = q_err.vec() / q_err.w() * 2;
//        r_old(3, 0) = -2 * (q_err.w() * q_err.z() + q_err.x() * q_err.y())
//                / (1 - 2 * (q_err.y() * q_err.y() + q_err.z() * q_err.z()));


        // Call update step in base class.
        this->CalculateAndApplyCorrection(non_const_state, core, H_old, r_old, R_);
    }
};
}  // namespace attitude_measurement
#endif  // ATTITUDE_MEASUREMENT_HPP_
