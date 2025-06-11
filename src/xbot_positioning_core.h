//
// Created by Clemens Elflein on 27.10.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//

#ifndef SRC_XBOT_POSITIONING_CORE_H
#define SRC_XBOT_POSITIONING_CORE_H

#include "SystemModel.hpp"
#include "PositionMeasurementModel.hpp"
#include "OrientationMeasurementModel.hpp"
#include "OrientationMeasurementModel2.hpp"
#include "SpeedMeasurementModel.hpp"

#include <kalman/ExtendedKalmanFilter.hpp>
#include <kalman/UnscentedKalmanFilter.hpp>

namespace xbot {
    namespace positioning {
        typedef double T;

        typedef xbot::positioning::State<T> StateT;
        typedef xbot::positioning::Control<T> ControlT;
        typedef xbot::positioning::SystemModel<T> SystemModelT;

        typedef xbot::positioning::PositionMeasurement<T> PositionMeasurementT;
        typedef xbot::positioning::OrientationMeasurement<T> OrientationMeasurementT;
        typedef xbot::positioning::OrientationMeasurement2<T> OrientationMeasurement2T;
        typedef xbot::positioning::SpeedMeasurement<T> SpeedMeasurementT;
        typedef xbot::positioning::PositionMeasurementModel<T> PositionModelT;
        typedef xbot::positioning::OrientationMeasurementModel<T> OrientationModelT;
        typedef xbot::positioning::OrientationMeasurementModel2<T> OrientationModel2T;
        typedef xbot::positioning::SpeedMeasurementModel<T> SpeedModelT;

        class xbot_positioning_core {
        //private:
        //    void logIfChanged(const xbot::positioning::StateT &x0, const xbot::positioning::StateT &x1, const std::string &msg);

        public:
            xbot_positioning_core();

            const StateT &predict(double linearVelocity, double vroll, double vpitch, double vyaw, double dt);
            const StateT &updatePosition(double x, double y, double z, double covariance);
            const StateT &updateOrientation(double roll, double pitch, /*double yaw, */double covariance);
            const StateT &updateOrientation2(double vx, double vy, double covariance);
            const StateT &updateSpeed(double linearVelocity, double angularVelocity, double covariance);
            const StateT &getState();
            void setState(double px, double py, double pz, double roll, double pitch, double yaw, double linearVelocity, double angularVelocity);
            const Kalman::Covariance<StateT> &getCovariance();
            void setAntennaOffset(tf2::Vector3 antenna_offset);

        public:
            Kalman::ExtendedKalmanFilter<StateT> kf{};
            SystemModelT sys{};
            PositionModelT p_model{};
            OrientationModelT o_model{};
            OrientationModel2T o2_model{};
            SpeedModelT s_model{};

            ControlT u{};
            PositionMeasurementT pos_meas{};
            OrientationMeasurementT orient_meas{};
            OrientationMeasurement2T orient2_meas{};
            SpeedMeasurementT speed_meas{};
        };
    }
}

#endif //SRC_XBOT_POSITIONING_CORE_H
