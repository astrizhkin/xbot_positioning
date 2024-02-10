//
// Created by Clemens Elflein on 27.10.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//

#include "xbot_positioning_core.h"

const xbot::positioning::StateT &
xbot::positioning::xbot_positioning_core::predict(double linearVelocity, double vroll, double vpitch, double vyaw, double dt) {
    sys.setDt(dt);
    u.v() = linearVelocity;
    u.droll() = vroll;
    u.dpitch() = vpitch;
    u.dyaw() = vyaw;
    return ekf.predict(sys, u);
}

const xbot::positioning::StateT &
xbot::positioning::xbot_positioning_core::updatePosition(double x, double y, double z, double covariance) {
    pos_meas.x_pos() = x;
    pos_meas.y_pos() = y;
    pos_meas.z_pos() = z;

    Kalman::Covariance<PositionMeasurementT> c;
    c.setIdentity();
    c *= covariance;

    p_model.setCovariance(c);

    return ekf.update(p_model, pos_meas);
}

const xbot::positioning::StateT &
xbot::positioning::xbot_positioning_core::updateOrientation(double roll, double pitch, /*double yaw, */double covariance) {
    orient_meas.roll() = roll;
    orient_meas.pitch() = pitch;
    //orient_meas.yaw() = yaw;
    Kalman::Covariance<OrientationMeasurementT> c;
    c.setIdentity();
    c *= covariance;

    o_model.setCovariance(c);

    return ekf.update(o_model, orient_meas);
}

const xbot::positioning::StateT &
xbot::positioning::xbot_positioning_core::updateOrientation2(double vx, double vy, double vz, double covariance) {
    orient2_meas.vx() = vx;
    orient2_meas.vy() = vy;
    orient2_meas.vz() = vz;

    Kalman::Covariance<OrientationMeasurement2T> c;
    c.setIdentity();
    c *= covariance;

    o2_model.setCovariance(c);

    return ekf.update(o2_model, orient2_meas);
}

const xbot::positioning::StateT &
xbot::positioning::xbot_positioning_core::updateSpeed(double linearVelocity, double angularVelocity, double covariance) {
    speed_meas.sl() = linearVelocity;
    speed_meas.sa() = angularVelocity;

//    Kalman::Covariance<SpeedMeasurementT> c;
//    c.setIdentity();
//    c *= covariance;

//    s_model.setCovariance(c);

    return ekf.update(s_model, speed_meas);
}

const xbot::positioning::StateT &xbot::positioning::xbot_positioning_core::getState() {
    return ekf.getState();
}

const Kalman::Covariance<xbot::positioning::StateT> &xbot::positioning::xbot_positioning_core::getCovariance() {
    return ekf.getCovariance();
}

xbot::positioning::xbot_positioning_core::xbot_positioning_core() {
//    Kalman::Covariance<StateT> c;
//    c.setIdentity();
//    c *= 0.001;
//    sys.setCovariance(c);
    setState(0,0,0,0,0,0,0,0);
}

void xbot::positioning::xbot_positioning_core::setState(double px, double py, double pz, double roll, double pitch, double yaw, double linearVelocity, double angularVelocity) {
    StateT x;
    x.setZero();
    x.x() = px;
    x.y() = py;
    x.z() = pz;
    x.roll() = roll;
    x.pitch() = pitch;
    x.yaw() = yaw;
    x.sl() = linearVelocity;
    x.sa() = angularVelocity;
    this->ekf.init(x);
    Kalman::Covariance<StateT> c;
    c.setIdentity();
    this->ekf.setCovariance(c);
}

void xbot::positioning::xbot_positioning_core::setAntennaOffset(double offset_x, double offset_y,double offset_z) {
    p_model.antenna_offset_x = o2_model.antenna_offset_x = offset_x;
    p_model.antenna_offset_y = o2_model.antenna_offset_y = offset_y;
    p_model.antenna_offset_z = o2_model.antenna_offset_z = offset_z;
}

