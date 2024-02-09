//
// Created by Clemens Elflein on 27.10.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//

#include "xbot_positioning_core.h"


const xbot::positioning::StateT &xbot::positioning::xbot_positioning_core::predict(double vx, double vr, double dt) {
    sys.setDt(dt);
    u.v() = vx;
    u.dtheta() = vr;
    return ekf.predict(sys, u);
}

const xbot::positioning::StateT &xbot::positioning::xbot_positioning_core::updatePosition(double x, double y, double covariance) {
    pos_meas.x_pos() = x;
    pos_meas.y_pos() = y;

    Kalman::Covariance<PositionMeasurementT> c;
    c.setIdentity();
    c *= covariance;

    p_model.setCovariance(c);

    return ekf.update(p_model, pos_meas);
}

const xbot::positioning::StateT &
xbot::positioning::xbot_positioning_core::updateOrientation(double theta, double covariance) {
    orient_meas.theta() = theta;
    Kalman::Covariance<OrientationMeasurementT> c;
    c.setIdentity();
    c *= covariance;

    o_model.setCovariance(c);

    return ekf.update(o_model, orient_meas);
}

const xbot::positioning::StateT &
xbot::positioning::xbot_positioning_core::updateOrientation2(double vx, double vy, double covariance) {
    orient2_meas.vx() = vx;
    orient2_meas.vy() = vy;

    Kalman::Covariance<OrientationMeasurementT2> c;
    c.setIdentity();
    c *= covariance;

    o2_model.setCovariance(c);

    return ekf.update(o2_model, orient2_meas);
}
const xbot::positioning::StateT &
xbot::positioning::xbot_positioning_core::updateSpeed(double vx, double vr, double covariance) {
    speed_meas.vx() = vx;
    speed_meas.vr() = vr;
//
//    Kalman::Covariance<SpeedMeasurementT> c;
//    c.setIdentity();
//    c *= covariance;
//
//    sm.setCovariance(c);

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
    setState(0,0,0,0,0);
}

void xbot::positioning::xbot_positioning_core::setState(double px, double py, double theta, double vx, double vr) {
    StateT x;
    x.setZero();
    x.x() = px;
    x.y() = py;
    x.theta() = theta;
    x.vx() = vx;
    x.vr() = vr;
    this->ekf.init(x);
    Kalman::Covariance<StateT> c;
    c.setIdentity();
    this->ekf.setCovariance(c);
}

void xbot::positioning::xbot_positioning_core::setAntennaOffset(double offset_x, double offset_y) {
    p_model.antenna_offset_x = o2_model.antenna_offset_x = offset_x;
    p_model.antenna_offset_y = o2_model.antenna_offset_y = offset_y;
}

