//
// Created by Clemens Elflein on 27.10.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//

#include "SystemModel.hpp"

#include "ros/ros.h"

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include "xbot_msgs/AbsolutePose.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "xbot_positioning_core.h"
#include "xbot_msgs/WheelTick.h"
#include "xbot_positioning/KalmanState.h"
#include "xbot_positioning/GPSControlSrv.h"
#include "xbot_positioning/SetPoseSrv.h"

ros::Publisher odometry_pub;
ros::Publisher xbot_absolute_pose_pub;

// Debug Publishers
ros::Publisher kalman_state;
ros::Publisher dbg_expected_motion_vector;

// The kalman filters
xbot::positioning::xbot_positioning_core core{};

// True, if we don't want to do gyro calibration on launch
bool skip_gyro_calibration;

// True, if we have wheel ticks (i.e. last_ticks is valid)
bool has_ticks;
xbot_msgs::WheelTick last_ticks;
bool has_gps;
xbot_msgs::AbsolutePose last_gps;

// True, if last_imu is valid and gyro_offset is valid
bool has_gyro;
sensor_msgs::Imu last_imu;
ros::Time gyro_calibration_start;
//double gyro_offset_x;
//double gyro_offset_y;
//double gyro_offset_z;

//double accel_offset_x;
//double accel_offset_y;
//double accel_offset_z;

// Current speed calculated by wheel ticks
double vx = 0.0;

// Min speed for motion vector to be fed into kalman filter
double min_speed = 0.0;

// Max position accuracy to allow for GPS updates
double max_gps_accuracy;

// True, if we should publish debug topics (expected motion vector and kalman state)
bool publish_debug;

// Antenna offset (offset between point of rotation and antenna)
double antenna_offset_x, antenna_offset_y;

nav_msgs::Odometry odometry;
xbot_positioning::KalmanState state_msg;
xbot_msgs::AbsolutePose xb_absolute_pose_msg;

bool gps_enabled = true;
int gps_outlier_count = 0;
int valid_gps_samples = 0;

ros::Time last_gps_time(0.0);

tf2::Vector3 normal_gravity_vector(0.0, 0.0, 9.81);
tf2::Vector3 accel_bias(0.0, 0.0, 0.0);
tf2::Vector3 gyro_bias(0.0, 0.0, 0.0);
int bias_samples_count;

/*void OdomPredictor::integrateIMUData(const sensor_msgs::Imu& msg) {
  if (!has_imu_meas) {
    estimate_timestamp_ = msg.header.stamp;
    has_imu_meas = true;
    return;
  }

  const double delta_time = (msg.header.stamp - estimate_timestamp_).toSec();

  Vector3 imu_linear_acceleration, imu_angular_velocity;

  const Vector3 final_angular_velocity =
      (imu_angular_velocity - imu_angular_velocity_bias_);
  const Vector3 delta_angle =
      delta_time * (final_angular_velocity + angular_velocity_) / 2.0;
  angular_velocity_ = final_angular_velocity;

  // apply half of the rotation delta
  const Rotation half_delta_rotation = Rotation::exp(delta_angle / 2.0);

  if (!have_orientation_) {
    transform_.getRotation() = transform_.getRotation() * half_delta_rotation;
  }

  // find changes in linear velocity and position
  const Vector3 delta_linear_velocity =
      delta_time * (imu_linear_acceleration +
                    transform_.getRotation().inverse().rotate(kGravity) -
                    imu_linear_acceleration_bias_);
  transform_.getPosition() =
      transform_.getPosition() +
      transform_.getRotation().rotate(
          delta_time * (linear_velocity_ + delta_linear_velocity / 2.0));
  linear_velocity_ += delta_linear_velocity;

  if (!have_orientation_) {
  // apply the other half of the rotation delta
    transform_.getRotation() = transform_.getRotation() * half_delta_rotation;
  }

  estimate_timestamp_ = msg.header.stamp;
}*/

double gyro_pitch = 0.0;
double gyro_roll = 0.0;

void onImu(const sensor_msgs::Imu::ConstPtr &msg) {
    tf2::Vector3 imu_accel(msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z);
    tf2::Vector3 imu_gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    double dt = (msg->header.stamp - last_imu.header.stamp).toSec();

    if(!has_gyro) {
        if(!skip_gyro_calibration) {
            if (bias_samples_count == 0) {
                ROS_INFO_STREAM("[xbot_positioning] Started IMU calibration. Robot must be in horizontal position.");
                gyro_calibration_start = msg->header.stamp;
                gyro_bias.setValue(0.0,0.0,0.0);
                accel_bias.setValue(0.0,0.0,0.0);
            }
            gyro_bias += imu_gyro;
            accel_bias += imu_accel;
            bias_samples_count++;
            if ((msg->header.stamp - gyro_calibration_start).toSec() < 7) {
                last_imu = *msg;
                return;
            }
            has_gyro = true;
            if (bias_samples_count > 0) {
                gyro_bias /= bias_samples_count;
                accel_bias /= bias_samples_count;
                accel_bias -= normal_gravity_vector;
            } else {
                gyro_bias.setValue(0.0,0.0,0.0);
                accel_bias.setValue(0.0,0.0,0.0);
            }
            bias_samples_count = 0;
            ROS_INFO_STREAM("[xbot_positioning] Calibrated IMU bias gyro " << gyro_bias.x() << ", " << gyro_bias.y() << ", " << gyro_bias.z() << " accel " << accel_bias.x() << ", " << accel_bias.y() << ", " << accel_bias.z());
        } else {
            ROS_WARN("[xbot_positioning] Skipped IMU calibration");
            has_gyro = true;
            return;
        }
    }

    //substract bias
    imu_accel -= accel_bias;
    imu_gyro -= gyro_bias;

    //compute pitch and roll (it includes acceleration component unfortunately)
    //pitch angle around y axis
    tf2::Vector3 pitch_vector(imu_accel.x(),0.0,imu_accel.z());
    tf2::Vector3 pitch_cross = normal_gravity_vector.cross(pitch_vector);
    double pitch_angle = normal_gravity_vector.angle(pitch_vector);
    if (pitch_cross.y() < 0) {
        pitch_angle = -pitch_angle;
    }
    //roll angle around x axis 
    tf2::Vector3 roll_vector(0.0,imu_accel.y(),imu_accel.z());
    tf2::Vector3 roll_cross = normal_gravity_vector.cross(roll_vector);
    double roll_angle = normal_gravity_vector.angle(roll_vector);
    //it works but why we check here for the positive to reverse? 
    if (roll_cross.x() > 0) {
        roll_angle = -roll_angle;
    }

    gyro_pitch+= imu_gyro.y()*dt;
    gyro_roll+=imu_gyro.x()*dt;

    //debug section
    //first calculate angle to the normal calibrated gravity
    double angle_to_normal_gravity = imu_accel.angle(normal_gravity_vector);
    if(angle_to_normal_gravity > 0.05) {
        //ROS_INFO_STREAM("[xbot_positioning] pitch cross " << pitch_cross.x() << ", " << pitch_cross.y() << ", " << pitch_cross.z());
        //ROS_INFO_STREAM("[xbot_positioning] roll cross " << roll_cross.x() << ", " << roll_cross.y() << ", " << roll_cross.z());
        //ROS_INFO_STREAM("[xbot_positioning] normal gravity angle " << angle_to_normal_gravity << " pitch " << pitch_angle << " roll " << roll_angle);
        //ROS_INFO_STREAM("[xbot_positioning] calibrated IMU accel " << imu_accel.x() << ", " << imu_accel.y() << ", " << imu_accel.z());

        if( abs(pitch_angle) > 0.2 ) {
            ROS_INFO_STREAM("[xbot_positioning] pitch " << pitch_angle << " gyro " << gyro_pitch);
        }
        if( abs(roll_angle) > 0.2 ) {
            ROS_INFO_STREAM("[xbot_positioning] roll " << roll_angle << " gyro " << gyro_roll);
        }
    }
    //TODO add pitch and roll to EKF
    //pitch_angle = core.updatePitch(pitch_angle);
    //roll_angle = core.updateRoll(roll_angle);

    core.predict(vx, imu_gyro.z(), dt);
    auto x = core.updateSpeed(vx, imu_gyro.z(),0.01);

    odometry.header.stamp = ros::Time::now();
    odometry.header.seq++;
    odometry.header.frame_id = "map";
    odometry.child_frame_id = "base_link";
    odometry.pose.pose.position.x = x.x_pos();
    odometry.pose.pose.position.y = x.y_pos();
    //TODO calucalte Z from GPS z and pitch * velocity
    odometry.pose.pose.position.z = 0;
    //TODO calucalate pitch and roll from EKF accelerometer and gyroscope
    tf2::Quaternion q(roll_angle, pitch_angle, x.theta());
    odometry.pose.pose.orientation = tf2::toMsg(q);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header = odometry.header;
    odom_trans.child_frame_id = odometry.child_frame_id;
    odom_trans.transform.translation.x = odometry.pose.pose.position.x;
    odom_trans.transform.translation.y = odometry.pose.pose.position.y;
    odom_trans.transform.translation.z = odometry.pose.pose.position.z;
    odom_trans.transform.rotation = odometry.pose.pose.orientation;

    static tf2_ros::TransformBroadcaster transform_broadcaster;
    transform_broadcaster.sendTransform(odom_trans);

    if(publish_debug) {
        auto state = core.getState();
        state_msg.x = state.x();
        state_msg.y = state.y();
        state_msg.theta = state.theta();
        state_msg.vx = state.vx();
        state_msg.vr = state.vr();

        kalman_state.publish(state_msg);
    }

    odometry_pub.publish(odometry);

    xb_absolute_pose_msg.header = odometry.header;
    xb_absolute_pose_msg.sensor_stamp = 0;
    xb_absolute_pose_msg.received_stamp = 0;
    xb_absolute_pose_msg.source = xbot_msgs::AbsolutePose::SOURCE_SENSOR_FUSION;
    xb_absolute_pose_msg.flags = xbot_msgs::AbsolutePose::FLAG_SENSOR_FUSION_DEAD_RECKONING;

    xb_absolute_pose_msg.orientation_valid = true;
    // TODO: send motion vector
    xb_absolute_pose_msg.motion_vector_valid = false;
    // TODO: set real value from kalman filter, not the one from the GPS.
    if(has_gps) {
        xb_absolute_pose_msg.position_accuracy = last_gps.position_accuracy;
    } else {
        xb_absolute_pose_msg.position_accuracy = 999;
    }
    if((ros::Time::now() - last_gps_time).toSec() < 10.0) {
        xb_absolute_pose_msg.flags |= xbot_msgs::AbsolutePose::FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE;
    } else {
        // on GPS timeout, we set accuracy to 0.
        xb_absolute_pose_msg.position_accuracy = 999;
    }
    // TODO: set real value
    xb_absolute_pose_msg.orientation_accuracy = 0.01;
    xb_absolute_pose_msg.pose = odometry.pose;
    xb_absolute_pose_msg.vehicle_heading = x.theta();
    xb_absolute_pose_msg.motion_heading = x.theta();

    xbot_absolute_pose_pub.publish(xb_absolute_pose_msg);

    last_imu = *msg;
}

void onWheelTicks(const xbot_msgs::WheelTick::ConstPtr &msg) {
    if(!has_ticks) {
        last_ticks = *msg;
        has_ticks = true;
        return;
    }
    double dt = (msg->stamp - last_ticks.stamp).toSec();

    double rl_delta = msg->wheel_pos_rl - last_ticks.wheel_pos_rl;
    double fl_delta = msg->wheel_pos_fl - last_ticks.wheel_pos_fl;
    double rr_delta = msg->wheel_pos_rr - last_ticks.wheel_pos_rr;
    double fr_delta = msg->wheel_pos_fr - last_ticks.wheel_pos_fr;

    double d_wheel_l = (rl_delta + fl_delta) * msg->wheel_radius / 2;
    double d_wheel_r = (rr_delta + fr_delta) * msg->wheel_radius / 2;

    //if(msg->wheel_direction_rl) {
    //    d_wheel_l *= -1.0;
    //}
    //if(msg->wheel_direction_rr) {
    //    d_wheel_r *= -1.0;
    //}

    double d_center = (d_wheel_l + d_wheel_r) / 2.0;
    vx = d_center / dt;
    //ROS_INFO("vx %f dist %f",vx,d_center);

    last_ticks = *msg;
}

bool setGpsState(xbot_positioning::GPSControlSrvRequest &req, xbot_positioning::GPSControlSrvResponse &res) {
    ROS_INFO_STREAM("[xbot_positioning] set gps_enabled = " << (int)req.gps_enabled << " with reason [" << req.reason <<+"]");
    gps_enabled = req.gps_enabled;
    return true;
}

bool setPose(xbot_positioning::SetPoseSrvRequest &req, xbot_positioning::SetPoseSrvResponse &res) {
    ROS_INFO_STREAM("[xbot_positioning] set pose with reason [" << req.reason << "]");
    tf2::Quaternion q;
    tf2::fromMsg(req.robot_pose.orientation, q);


    tf2::Matrix3x3 m(q);
    double unused1, unused2, yaw;

    m.getRPY(unused1, unused2, yaw);
    core.setState(req.robot_pose.position.x, req.robot_pose.position.y, yaw,0,0);
    return true;
}

void onPose(const xbot_msgs::AbsolutePose::ConstPtr &msg) {
    if(!gps_enabled) {
        ROS_INFO_STREAM_THROTTLE(1, "[xbot_positioning] dropping GPS update, since gps_enabled = false.");
        return;
    }
    // TODO fuse with high covariance?

    if((msg->flags & xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FIXED) == 0 && 
	(msg->flags & xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FLOAT) == 0 ) {
        ROS_INFO_STREAM_THROTTLE(1, "[xbot_positioning] Dropped GPS update, since it's not RTK Fixed nor Float");
        return;
    }

    if(msg->position_accuracy > max_gps_accuracy) {
        ROS_INFO_STREAM_THROTTLE(1, "[xbot_positioning] Dropped GPS update, since it's not accurate enough. Accuracy was: " << msg->position_accuracy << ", limit is:" << max_gps_accuracy);
        return;
    }

    double time_since_last_gps = (ros::Time::now() - last_gps_time).toSec();
    if (time_since_last_gps > 5.0) {
        ROS_WARN_STREAM("[xbot_positioning] Last GPS was " << time_since_last_gps << " seconds ago.");
        has_gps = false;
        valid_gps_samples = 0;
        gps_outlier_count = 0;
        last_gps = *msg;
        // we have GPS for next time
        last_gps_time = ros::Time::now();
        return;
    }

    tf2::Vector3 gps_pos(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
    tf2::Vector3 last_gps_pos(last_gps.pose.pose.position.x,last_gps.pose.pose.position.y,last_gps.pose.pose.position.z);

    double distance_to_last_gps = (last_gps_pos - gps_pos).length();

    if (distance_to_last_gps < 5.0) {
        // inlier, we treat it normally
        // store the gps as last
        last_gps = *msg;
        last_gps_time = ros::Time::now();

        gps_outlier_count = 0;
        valid_gps_samples++;

        if (!has_gps && valid_gps_samples > 10) {
            //ROS_INFO_STREAM("GPS data now valid");
            ROS_INFO_STREAM("[xbot_positioning] First GPS data, moving kalman filter to " << msg->pose.pose.position.x << ", " << msg->pose.pose.position.y);
            // we don't even have gps yet, set odometry to first estimate
            core.updatePosition(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.001);

            has_gps = true;
        } else if (has_gps) {
            // gps was valid before, we apply the filter
            //ROS_INFO_STREAM("[xbot_positioning] Next GPS data, update position " << msg->pose.pose.position.x << ", " << msg->pose.pose.position.y);
            core.updatePosition(msg->pose.pose.position.x, msg->pose.pose.position.y, 500.0);
            if(publish_debug) {
                auto m = core.o2_model.h(core.ekf.getState());
                geometry_msgs::Vector3 dbg;
                dbg.x = m.vx();
                dbg.y = m.vy();
                dbg_expected_motion_vector.publish(dbg);
            }
            if(std::sqrt(std::pow(msg->motion_vector.x, 2)+std::pow(msg->motion_vector.y, 2)) >= min_speed) {
                core.updateOrientation2(msg->motion_vector.x, msg->motion_vector.y, 10000.0);
            }
        }
    } else {
        ROS_WARN_STREAM("[xbot_positioning] GPS outlier found. Distance was: " << distance_to_last_gps);
        gps_outlier_count++;
        // ~10 sec
        if (gps_outlier_count > 10) {
            ROS_ERROR_STREAM("[xbot_positioning] too many outliers, assuming that the current gps value is valid.");
            // store the gps as last
            last_gps = *msg;
            last_gps_time = ros::Time::now();
            has_gps = false;

            valid_gps_samples = 0;
            gps_outlier_count = 0;
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "xbot_positioning");

    has_gps = false;
    gps_enabled = true;
    vx = 0.0;
    has_gyro = false;
    has_ticks = false;
    gyro_bias.setValue(0.0,0.0,0.0);
    accel_bias.setValue(0.0,0.0,0.0);

    bias_samples_count = 0;

    valid_gps_samples = 0;
    gps_outlier_count = 0;

    antenna_offset_x = antenna_offset_y = 0;

    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");

    ros::ServiceServer gps_service = n.advertiseService("xbot_positioning/set_gps_state", setGpsState);
    ros::ServiceServer pose_service = n.advertiseService("xbot_positioning/set_robot_pose", setPose);

    paramNh.param("skip_gyro_calibration", skip_gyro_calibration, false);
    double gyro_bias_z;
    paramNh.param("gyro_offset", gyro_bias_z, 0.0);
    paramNh.param("min_speed", min_speed, 0.01);
    paramNh.param("max_gps_accuracy", max_gps_accuracy, 0.1);
    paramNh.param("debug", publish_debug, false);
    paramNh.param("antenna_offset_x", antenna_offset_x, 0.0);
    paramNh.param("antenna_offset_y", antenna_offset_y, 0.0);

    core.setAntennaOffset(antenna_offset_x, antenna_offset_y);

    ROS_INFO_STREAM("[xbot_positioning] Antenna offset: " << antenna_offset_x << ", " << antenna_offset_y);

    if(gyro_bias_z != 0.0 && skip_gyro_calibration) {
        gyro_bias.setZ(gyro_bias_z);
        ROS_WARN_STREAM("[xbot_positioning] Using gyro z offset of: " << gyro_bias.z());
    }

    odometry_pub = paramNh.advertise<nav_msgs::Odometry>("odom_out", 50);
    xbot_absolute_pose_pub = paramNh.advertise<xbot_msgs::AbsolutePose>("xb_pose_out", 50);
    if(publish_debug) {
        dbg_expected_motion_vector = paramNh.advertise<geometry_msgs::Vector3>("debug_expected_motion_vector", 50);
        kalman_state = paramNh.advertise<xbot_positioning::KalmanState>("kalman_state", 50);
    }

    ros::Subscriber imu_sub = paramNh.subscribe("imu_in", 10, onImu);
    ros::Subscriber pose_sub = paramNh.subscribe("xb_pose_in", 10, onPose);
    ros::Subscriber wheel_tick_sub = paramNh.subscribe("wheel_ticks_in", 10, onWheelTicks);

    ros::spin();
    return 0;
}
