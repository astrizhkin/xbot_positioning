//
// Created by Clemens Elflein on 27.10.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//

#include "xbot_positioning_core.h"


//void xbot::positioning::xbot_positioning_core::logIfChanged(const xbot::positioning::StateT &x0, const xbot::positioning::StateT &x1, const std::string &msg){
//    double d = 0.05;
//    double dx = x1.x_pos()-x0.x_pos();
//    double dy = x1.y_pos()- x0.y_pos();
//    double droll = x1.roll()-x0.roll();
//    double dpitch = x1.pitch()-x0.pitch();
//    double dyaw = x1.yaw()-x0.yaw();
//    
//    if (abs(dx)>d || abs(dy)>d) {
//        ROS_INFO_STREAM(msg<<" dx "<<dx<<" dy "<<dy);
//    } 
//    if (abs(droll)>d || abs(dpitch)>d || abs(dyaw)>d) {
//        ROS_INFO_STREAM(msg<<" dRPY "<<droll<<", "<<dpitch<<", "<<dyaw);
//    } 
//}

const xbot::positioning::StateT &
xbot::positioning::xbot_positioning_core::predict(double linearVelocity, double vroll, double vpitch, double vyaw, double dt) {
    sys.setDt(dt);
    u.v() = linearVelocity;
    u.droll() = vroll;
    u.dpitch() = vpitch;
    u.dyaw() = vyaw;
//    auto s1 = getState();
    auto &s2 = kf.predict(sys, u);
//    logIfChanged(s1,s2,"predict");
    return s2;
}

const xbot::positioning::StateT &
xbot::positioning::xbot_positioning_core::updatePosition(double x, double y, double z, double covariance) {
    pos_meas.gps_x() = x;
    pos_meas.gps_y() = y;
    pos_meas.gps_z() = z;

    Kalman::Covariance<PositionMeasurementT> c;
    c.setIdentity();
    c *= covariance;

    p_model.setCovariance(c);

//    auto s1 = getState();
    auto &s2 = kf.update(p_model, pos_meas);
//    logIfChanged(s1,s2,"updatePosition");
    return s2;
}

const xbot::positioning::StateT &
xbot::positioning::xbot_positioning_core::updateOrientation(double roll, double pitch, double covariance) {
    orient_meas.roll() = roll;
    orient_meas.pitch() = pitch;
    Kalman::Covariance<OrientationMeasurementT> c;
    c.setIdentity();
    c *= covariance;

    o_model.setCovariance(c);
//    auto s1 = getState(); 
    auto &s2 = kf.update(o_model, orient_meas);
//    logIfChanged(s1,s2, "updateOrientation");
    return s2;
}

const xbot::positioning::StateT &
xbot::positioning::xbot_positioning_core::updateOrientation2(double vx, double vy, double covariance) {
    orient2_meas.gps_vx() = vx;
    orient2_meas.gps_vy() = vy;

    Kalman::Covariance<OrientationMeasurement2T> c;
    c.setIdentity();
    c *= covariance;

    o2_model.setCovariance(c);

//    auto s1 = getState(); 
    auto &s2 = kf.update(o2_model, orient2_meas);
//    logIfChanged(s1,s2, "updateOrientation2");
    return s2;
}

const xbot::positioning::StateT &
xbot::positioning::xbot_positioning_core::updateSpeed(double linearVelocity, double angularVelocity, double covariance) {
    speed_meas.sl() = linearVelocity;
    speed_meas.sa() = angularVelocity;

//    Kalman::Covariance<SpeedMeasurementT> c;
//    c.setIdentity();
//    c *= covariance;

//    s_model.setCovariance(c);

//    auto s1 = getState(); 
    auto &s2 = kf.update(s_model, speed_meas);
//    logIfChanged(s1,s2, "updateSpeed");
    return s2;
}

const xbot::positioning::StateT &xbot::positioning::xbot_positioning_core::getState() {
    return kf.getState();
}

const Kalman::Covariance<xbot::positioning::StateT> &xbot::positioning::xbot_positioning_core::getCovariance() {
    return kf.getCovariance();
}

xbot::positioning::xbot_positioning_core::xbot_positioning_core() /* : kf(1)*/ {
//    Kalman::Covariance<StateT> c;
//    c.setIdentity();
//    c *= 0.001;
//    sys.setCovariance(c);
    setState(0,0,0,0,0,0,0,0);
}

void xbot::positioning::xbot_positioning_core::setState(double px, double py, double pz, double roll, double pitch, double yaw, double linearVelocity, double angularVelocity) {
    StateT x;
    x.setZero();
    x.x_pos() = px;
    x.y_pos() = py;
    x.z_pos() = pz;
    x.roll() = roll;
    x.pitch() = pitch;
    x.yaw() = yaw;
    x.sl() = linearVelocity;
    x.sa() = angularVelocity;
    this->kf.init(x);
    Kalman::Covariance<StateT> c;
    c.setIdentity();
    this->kf.setCovariance(c);
}

void xbot::positioning::xbot_positioning_core::setAntennaOffset(tf2::Vector3 anttena_offset) {
    p_model.antenna_offset = anttena_offset;
    o2_model.antenna_offset = anttena_offset;
}

