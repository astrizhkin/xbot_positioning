//
// Created by Clemens Elflein on 27.10.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//

//#define WHEEL_TICKS_MSG

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "SystemModel.hpp"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "ros/ros.h"
#include "xbot_msgs/AbsolutePose.h"
#ifdef WHEEL_TICKS_MSG
    #include "xbot_msgs/wheel_WheelTick.h"
#endif
#include "xbot_positioning/GPSControlSrv.h"
#include "xbot_positioning/KalmanState.h"
#include "xbot_positioning/SetPoseSrv.h"
#include "xbot_positioning_core.h"

ros::Publisher odometry_3d_pub;
ros::Publisher odometry_2d_pub;
ros::Publisher xbot_absolute_pose_pub;

// Debug Publishers
ros::Publisher kalman_state;
ros::Publisher dbg_expected_motion_vector;
ros::Publisher gps_velocity_pub;
ros::Publisher gps_baselink_pub;

// The kalman filters
xbot::positioning::xbot_positioning_core core{};

// True, if we don't want to do gyro calibration on launch
bool skip_gyro_calibration;

// True, if we have wheel ticks (i.e. last_ticks is valid)
#ifdef WHEEL_TICKS_MSG
    bool has_ticks;
    xbot_msgs::WheelTick last_ticks;
#endif
bool has_gps;
xbot_msgs::AbsolutePose last_gps;
bool gps_enabled = true;
int gps_outlier_count = 0;
int valid_gps_samples = 0;
ros::Time last_gps_time(0.0);
uint64_t last_pose_update_epoch;
uint64_t last_orientation_update_epoch;

// True, if last_imu is valid and gyro_offset is valid
bool has_gyro;
sensor_msgs::Imu last_imu;
ros::Time gyro_calibration_start;

// Current speed calculated by wheel ticks
double odom_linear_velocity = 0.0;
double odom_angular_velocity = 0.0;

// Min speed for motion vector to be fed into kalman filter
double min_speed = 0.0;

// Max position accuracy to allow for GPS updates
double max_gps_accuracy;

// True, if we should publish debug topics (expected motion vector and kalman state)
bool publish_debug, publish_2d_odom;

// Antenna offset (offset between point of rotation and antenna)
tf2::Vector3 antenna_offset;

xbot_positioning::KalmanState state_msg;

tf2::Vector3 normal_gravity_vector(0.0, 0.0, 9.81);
tf2::Vector3 accel_bias(0.0, 0.0, 0.0);
tf2::Vector3 gyro_bias(0.0, 0.0, 0.0);
int bias_samples_count;
bool skip_accelermoter_calibration = false;

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

void onImu(const sensor_msgs::Imu::ConstPtr &msg) {
    tf2::Vector3 imu_accel(msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z);
    tf2::Vector3 imu_gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    double dt = (msg->header.stamp - last_imu.header.stamp).toSec();

    if (!has_gyro) {
        if (!skip_gyro_calibration) {
            if (bias_samples_count == 0) {
                ROS_WARN_STREAM("[xbot_positioning] Started IMU calibration. Robot must be in horizontal position.");
                gyro_calibration_start = msg->header.stamp;
                gyro_bias.setValue(0.0,0.0,0.0);
                if(!skip_accelermoter_calibration) {
                    accel_bias.setValue(0.0,0.0,0.0);
                } else {
                    ROS_WARN_STREAM("[xbot_positioning] Skipping acelermoter calibration.");
                }
            }
            gyro_bias += imu_gyro;
            if(!skip_accelermoter_calibration) {
                accel_bias += imu_accel;
            }
            bias_samples_count++;
            if ((msg->header.stamp - gyro_calibration_start).toSec() < 7) {
                last_imu = *msg;
                return;
            }
            has_gyro = true;
            if (bias_samples_count > 0) {
                gyro_bias /= bias_samples_count;
                if(!skip_accelermoter_calibration) {
                    accel_bias /= bias_samples_count;
                    accel_bias -= normal_gravity_vector;
                }
            } else {
                gyro_bias.setValue(0.0,0.0,0.0);
                if(!skip_accelermoter_calibration) {
                    accel_bias.setValue(0.0,0.0,0.0);
                }
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
    
    //pitch angle around y axis based on accellerometer
    tf2::Vector3 pitch_vector(imu_accel.x(),0.0,imu_accel.z());
    tf2::Vector3 pitch_cross = pitch_vector.cross(normal_gravity_vector);
    double pitch_angle = pitch_vector.angle(normal_gravity_vector);
    if (pitch_cross.y() < 0) pitch_angle = -pitch_angle;
    
    //roll angle around x axis based on accellerometer
    tf2::Vector3 roll_vector(0.0,imu_accel.y(),imu_accel.z());
    tf2::Vector3 roll_cross = roll_vector.cross(normal_gravity_vector);
    double roll_angle = roll_vector.angle(normal_gravity_vector);
    if (roll_cross.x() < 0) roll_angle = -roll_angle;
    
    //update EKF state
    core.predict(odom_linear_velocity, imu_gyro.x(), imu_gyro.y(), imu_gyro.z(), dt);
    //covariance less than 100 will significantly affect roll/pitch readings on linear horizontal acceleration (x,y)
    core.updateOrientation(roll_angle, pitch_angle, 5000.0);
    xbot::positioning::StateT x = core.updateSpeed(odom_linear_velocity, imu_gyro.z(),0.01);
    
    //ROS_INFO("[xbot_positioning] RPY %+3.2f %+3.2f %+3.2f Input RP %+3.2f(x%+3.2f) %+3.2f(y%+3.2f) GYRO XYZ %+3.2f %+3.2f %+3.2f ACCEL XYZ %+4.2f %+4.2f %+4.2f",
    //    x.roll(),x.pitch(),x.yaw(),
    //    roll_angle,roll_cross.x(),pitch_angle,pitch_cross.y(),
    //    imu_gyro.x(),imu_gyro.y(),imu_gyro.z(),
    //    imu_accel.x(),imu_accel.y(),imu_accel.z());

    //get result quaternions
    tf2::Quaternion q_3d;
    q_3d.setRPY(x.roll(),x.pitch(),x.yaw());//same as setEylerZYZ
    tf2::Quaternion q_2d;
    q_2d.setRPY(0, 0, x.yaw());//same as setEylerZYZ
    tf2::Quaternion q_2d_to_3d;
    q_2d_to_3d.setRPY(x.roll(), x.pitch(),0);//same as setEylerZYZ
    //Two quaternions from the same frame, q_2d and q_3d. 
    //to find the relative rotation, q_r, to go from q_2d to q_3d:
    //q_3d = q_r*q_2d
    //solve for q_r similarly to solving a matrix equation. 
    //Invert q_1 and right-multiply both sides. Again, the order of multiplication is important:
    //q_r = q_3d*q_2d_inverse
    //DOESN'T WORK!!!
    //tf2::Quaternion q_2d_to_3d = q_3d*q_2d.inverse();

    // This represents an estimate of a position and velocity in free space.  
    // The pose in this message should be specified in the coordinate frame given by header.frame_id.
    // The twist in this message should be specified in the coordinate frame given by the child_frame_id
    nav_msgs::Odometry odometry_2d;
    odometry_2d.header.stamp = ros::Time::now();
    odometry_2d.header.seq++;
    odometry_2d.header.frame_id = "map";
    odometry_2d.child_frame_id = "base_footprint";
    odometry_2d.pose.pose.position.x = x.x_pos();
    odometry_2d.pose.pose.position.y = x.y_pos();
    odometry_2d.pose.pose.position.z = 0;
    odometry_2d.pose.pose.orientation = tf2::toMsg(q_2d);
    odometry_2d.twist.twist.linear.x = odom_linear_velocity * cos(x.pitch());//projected to 2d linearVelocityWheels
    odometry_2d.twist.twist.angular.z = odom_angular_velocity;//correct. angularVelocityWheels 3d = 2d
    //build messages

    // This expresses a transform from coordinate frame header.frame_id
    // to the coordinate frame child_frame_id
    geometry_msgs::TransformStamped base_footprint_transform_2d;
    base_footprint_transform_2d.header = odometry_2d.header;
    base_footprint_transform_2d.child_frame_id = odometry_2d.child_frame_id;
    base_footprint_transform_2d.transform.translation.x = odometry_2d.pose.pose.position.x;
    base_footprint_transform_2d.transform.translation.y = odometry_2d.pose.pose.position.y;
    base_footprint_transform_2d.transform.translation.z = 0;
    base_footprint_transform_2d.transform.rotation = tf2::toMsg(q_2d);//odometry.pose.pose.orientation;

    // This represents an estimate of a position and velocity in free space.  
    // The pose in this message should be specified in the coordinate frame given by header.frame_id.
    // The twist in this message should be specified in the coordinate frame given by the child_frame_id
    nav_msgs::Odometry odometry_3d;
    odometry_3d.header.stamp = odometry_2d.header.stamp;
    odometry_3d.header.seq++;
    odometry_3d.header.frame_id = "map";
    odometry_3d.child_frame_id = "base_link";
    odometry_3d.pose.pose.position.x = x.x_pos();
    odometry_3d.pose.pose.position.y = x.y_pos();
    odometry_3d.pose.pose.position.z = x.z_pos();
    odometry_3d.pose.pose.orientation = tf2::toMsg(q_3d);
    odometry_3d.twist.twist.linear.x = odom_linear_velocity;
    odometry_3d.twist.twist.angular.z = odom_angular_velocity;

    // This expresses a transform from coordinate frame header.frame_id
    // to the coordinate frame child_frame_id
    geometry_msgs::TransformStamped base_link_transform_3d;
    base_link_transform_3d.header.stamp = odometry_3d.header.stamp;
    base_link_transform_3d.header.frame_id = "base_footprint";
    base_link_transform_3d.child_frame_id = "base_link";
    base_link_transform_3d.transform.translation.x = 0;
    base_link_transform_3d.transform.translation.y = 0;
    base_link_transform_3d.transform.translation.z = x.z_pos();
    base_link_transform_3d.transform.rotation = tf2::toMsg(q_2d_to_3d);

    static tf2_ros::TransformBroadcaster transform_broadcaster;
    transform_broadcaster.sendTransform(base_footprint_transform_2d);//it should be transform from map to base_footprint
    transform_broadcaster.sendTransform(base_link_transform_3d);//it should be transform base_footprint to base_link

    if (publish_debug) {
        auto state = core.getState();
        state_msg.x = state.x();
        state_msg.y = state.y();
        state_msg.z = state.z();
        state_msg.roll = state.roll();
        state_msg.pitch = state.pitch();
        state_msg.yaw = state.yaw();
        state_msg.sl = state.sl();
        state_msg.sa = state.sa();

        kalman_state.publish(state_msg);
    }

    odometry_3d_pub.publish(odometry_3d);
    if(publish_2d_odom) {
        odometry_2d_pub.publish(odometry_2d);
    }

    xbot_msgs::AbsolutePose xb_absolute_pose_msg_2d;
    xb_absolute_pose_msg_2d.header = odometry_2d.header;
    xb_absolute_pose_msg_2d.epoch_ms = 0;
    xb_absolute_pose_msg_2d.received_stamp = 0;
    xb_absolute_pose_msg_2d.source = xbot_msgs::AbsolutePose::SOURCE_SENSOR_FUSION;
    xb_absolute_pose_msg_2d.flags = xbot_msgs::AbsolutePose::FLAG_SENSOR_FUSION_DEAD_RECKONING;

    xb_absolute_pose_msg_2d.orientation_valid = true;
    // TODO: send motion vector
    xb_absolute_pose_msg_2d.motion_vector_valid = false;
    // TODO: set real value from kalman filter, not the one from the GPS.
    if (has_gps) {
        xb_absolute_pose_msg_2d.position_accuracy = last_gps.position_accuracy;
    } else {
        xb_absolute_pose_msg_2d.position_accuracy = 999;
    }
    if ((ros::Time::now() - last_gps_time).toSec() < 10.0) {
        xb_absolute_pose_msg_2d.flags |= xbot_msgs::AbsolutePose::FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE;
    } else {
        // on GPS timeout, we set accuracy to 0.
        xb_absolute_pose_msg_2d.position_accuracy = 999;
    }
    // TODO: set real value
    xb_absolute_pose_msg_2d.orientation_accuracy = 0.01;
    xb_absolute_pose_msg_2d.pose.pose.position.x = odometry_2d.pose.pose.position.x;
    xb_absolute_pose_msg_2d.pose.pose.position.y = odometry_2d.pose.pose.position.y;
    xb_absolute_pose_msg_2d.pose.pose.position.z = 0;
    xb_absolute_pose_msg_2d.pose.pose.orientation = tf2::toMsg(q_2d);
    xb_absolute_pose_msg_2d.vehicle_heading = x.yaw();
    xb_absolute_pose_msg_2d.motion_heading = x.yaw();

    xbot_absolute_pose_pub.publish(xb_absolute_pose_msg_2d);

    last_imu = *msg;
}

#ifdef WHEEL_TICKS_MSG
void onWheelTicks(const xbot_msgs::WheelTick::ConstPtr &msg) {
    if (!has_ticks) {
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

    double d_linear = (d_wheel_r + d_wheel_l) / 2.0;
    double d_angular = (d_wheel_r - d_wheel_l) / msg->wheel_separation;
    
    odom_linear_velocity = d_linear / dt;
    odom_angular_velocity = d_angular / dt;

    //ROS_INFO("vx %f dist %f",vx,d_center);

    last_ticks = *msg;
}
#endif

void onTwistIn(const geometry_msgs::TwistStamped::ConstPtr &msg) {
    odom_linear_velocity = msg->twist.linear.x;
    odom_angular_velocity = msg->twist.angular.z;
    //ROS_INFO_STREAM_THROTTLE(0.2,"[xbot_positioning] twist in lin=" << odom_linear_velocity << " ang=" << odom_angular_velocity);
}

bool setGpsState(xbot_positioning::GPSControlSrvRequest &req, xbot_positioning::GPSControlSrvResponse &res) {
    ROS_INFO_STREAM("[xbot_positioning] set gps_enabled = " << (int)req.gps_enabled << " with reason [" << req.reason <<+"]");
    gps_enabled = req.gps_enabled;
    return true;
}

bool setPose(const geometry_msgs::Pose &pose) {
    tf2::Quaternion q;
    tf2::fromMsg(pose.orientation, q);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    core.setState(pose.position.x, pose.position.y, pose.position.z, 0, 0, yaw, 0, 0);
    return true;
}

bool setPose(xbot_positioning::SetPoseSrvRequest &req, xbot_positioning::SetPoseSrvResponse &res) {
    ROS_INFO_STREAM("[xbot_positioning] set pose with reason [" << req.reason << "]");
    return setPose(req.robot_pose);
}

void onInitialPose(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    ROS_INFO_STREAM_THROTTLE(1, "[xbot_positioning] set initial pose from topic");
    setPose(msg->pose);
}

void onGpsPose(const xbot_msgs::AbsolutePose::ConstPtr &msg) {
    if (!gps_enabled) {
        ROS_INFO_STREAM_THROTTLE(1, "[xbot_positioning] dropping GPS update, since gps_enabled = false.");
        return;
    }
    // TODO fuse with high covariance?

    if ((msg->flags & xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FIXED) == 0 && 
	(msg->flags & xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FLOAT) == 0 ) {
        ROS_INFO_STREAM_THROTTLE(1, "[xbot_positioning] Dropped GPS update, since it's not RTK Fixed nor Float");
        return;
    }

    if (msg->position_accuracy > max_gps_accuracy) {
        ROS_INFO_STREAM_THROTTLE(1, "[xbot_positioning] Dropped GPS update, since it's not accurate enough. Accuracy was: " << msg->position_accuracy << ", limit is:" << max_gps_accuracy);
        return;
    }

    if (msg->pose_valid || msg->motion_vector_valid || msg->position_accuracy_valid) {
        //ROS_INFO_STREAM("[xbot_positioning] Accept GPS update: " << msg->epoch_ms << ": P="<<(int)msg->pose_valid << " M=" << (int)msg->motion_vector_valid << " A="<<(int)msg->position_accuracy_valid);
    } else {
        ROS_INFO_STREAM("[xbot_positioning] Dropped GPS update without valid data : " << msg->epoch_ms);
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

    tf2::Vector3 gps_pos(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    tf2::Vector3 last_gps_pos(last_gps.pose.pose.position.x, last_gps.pose.pose.position.y, last_gps.pose.pose.position.z);

    double distance_to_last_gps = (last_gps_pos - gps_pos).length();

    if (distance_to_last_gps < 2.0) {
        // inlier, we treat it normally
        // store the gps as last
        last_gps = *msg;
        last_gps_time = ros::Time::now();

        gps_outlier_count = 0;
        valid_gps_samples++;

        if (!has_gps && valid_gps_samples > 10) {
            ROS_INFO_STREAM("[xbot_positioning] First GPS data, moving kalman filter to " << msg->pose.pose.position.x << ", " << msg->pose.pose.position.y);
            // we don't even have gps yet, set odometry to first estimate
            if(msg->pose_valid && msg->epoch_ms != last_pose_update_epoch) {
                core.updatePosition(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z, 0.001);
                last_pose_update_epoch = msg->epoch_ms;
            } 
            //else {
            //    ROS_INFO_STREAM("[xbot_positioning] Skip same epoch or invald position update: last epoch="<<last_pose_update_epoch<< " valid="<<(int)msg->pose_valid);
            //}
            has_gps = true;
        } else if (has_gps) {
            // gps was valid before, we apply the filter
            //ROS_INFO_STREAM("[xbot_positioning] Next GPS data, update position " << msg->pose.pose.position.x << ", " << msg->pose.pose.position.y);
            if(msg->pose_valid && msg->epoch_ms != last_pose_update_epoch) {
                auto &x = core.updatePosition(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z, 500.0);
                last_pose_update_epoch = msg->epoch_ms;
                if (publish_debug) {
                    tf2::Quaternion q;
                    q.setRPY(x.roll(),x.pitch(),x.yaw());
                    tf2::Vector3 antennaRotated = tf2::quatRotate(q, antenna_offset);
                    geometry_msgs::Vector3Stamped gps_baselink;
                    gps_baselink.header.stamp = msg->header.stamp;
                    gps_baselink.header.frame_id = msg->header.frame_id;
                    gps_baselink.vector.x = msg->pose.pose.position.x - antennaRotated.x();
                    gps_baselink.vector.y = msg->pose.pose.position.y - antennaRotated.y();
                    gps_baselink.vector.z = msg->pose.pose.position.y - antennaRotated.z();
                    gps_baselink_pub.publish(gps_baselink);
                }
            } 
            //else {
            //    ROS_INFO_STREAM("[xbot_positioning] Skip same epoch or invald position update: last epoch="<<last_pose_update_epoch<< " valid="<<(int)msg->pose_valid);
            //}


            if(msg->motion_vector_valid && msg->epoch_ms != last_orientation_update_epoch){
                double gps_3d_linear_velocity = std::sqrt(msg->motion_vector.x * msg->motion_vector.x +
                                                    msg->motion_vector.y * msg->motion_vector.y +
                                                    msg->motion_vector.z * msg->motion_vector.z);

                if (gps_3d_linear_velocity >= min_speed) {
                    //ROS_INFO_STREAM("[xbot_positioning] Calc heading dx=" << msg->motion_vector.x << " dy=" << msg->motion_vector.y << " head="<<best_heading_yaw<< " yaw="<<core.getState().yaw());
                    core.updateOrientation2(msg->motion_vector.x, msg->motion_vector.y, 5000.0);
                    last_orientation_update_epoch = msg->epoch_ms;
                }
                if (publish_debug) {
                    geometry_msgs::TwistStamped gps_vel;
                    gps_vel.header.stamp = msg->header.stamp;
                    gps_vel.header.frame_id = msg->header.frame_id;
                    gps_vel.twist.linear.x = gps_3d_linear_velocity;
                    gps_velocity_pub.publish(gps_vel);

                    auto m = core.o2_model.h(core.kf.getState());
                    geometry_msgs::Vector3Stamped dbg;
                    dbg.header.stamp = msg->header.stamp;
                    dbg.header.frame_id = msg->header.frame_id;
                    dbg.vector.x = m.gps_vx();
                    dbg.vector.y = m.gps_vy();
                    dbg.vector.z = 0;
                    dbg_expected_motion_vector.publish(dbg);
                }
            } 
            //else {
            //    ROS_INFO_STREAM("[xbot_positioning] Skip same epoch or invald orientation update: last epoch="<<last_orientation_update_epoch<< " valid="<<(int)msg->motion_vector_valid);
            //}
        }
    } else {
        ROS_WARN_STREAM("[xbot_positioning] GPS outlier found. Distance was: " << distance_to_last_gps);
        gps_outlier_count++;
        // ~5 sec
        if (gps_outlier_count > 25) {
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

bool findStaticTransform(const std::string& targetFrame, const std::string& sourceFrame, tf2::Vector3 &vec,const ros::NodeHandle &nh) {
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer,nh,true);
    try{
        geometry_msgs::TransformStamped transform = tfBuffer.lookupTransform(targetFrame, sourceFrame,ros::Time::now(),ros::Duration(20));
        tf2::fromMsg(transform.transform.translation,vec);
        ROS_INFO_STREAM("[xbot_positioning] Found transform from "<<sourceFrame<<" to "<<targetFrame);
        return true;
    } catch (tf2::TransformException &ex) {
        ROS_ERROR_STREAM("[xbot_positioning] Unable to get transfrom from "<<sourceFrame<<" to "<<targetFrame<<": "<<ex.what());
        return false;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "xbot_positioning");

    has_gps = false;
    gps_enabled = true;
    odom_linear_velocity = 0.0;
    odom_angular_velocity = 0.0;
    has_gyro = false;
    #ifdef WHEEL_TICKS_MSG
        has_ticks = false;
    #endif
    gyro_bias.setValue(0.0,0.0,0.0);
    accel_bias.setValue(0.0,0.0,0.0);

    bias_samples_count = 0;

    valid_gps_samples = 0;
    gps_outlier_count = 0;

    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");

    ros::ServiceServer gps_service = n.advertiseService("xbot_positioning/set_gps_state", setGpsState);
    ros::ServiceServer pose_service = n.advertiseService("xbot_positioning/set_robot_pose", setPose);

    paramNh.param("skip_gyro_calibration", skip_gyro_calibration, false);
    double gyro_bias_z;
    paramNh.param("gyro_offset", gyro_bias_z, 0.0);
    paramNh.param("min_speed", min_speed, 0.1);//10cm/sec
    paramNh.param("max_gps_accuracy", max_gps_accuracy, 0.1);
    paramNh.param("debug", publish_debug, false);
    paramNh.param("publish_2d_odom", publish_2d_odom, false);

    double accel_bias_x = 0.0, accel_bias_y = 0.0, accel_bias_z = 0.0;
    paramNh.param("accel_bias_x", accel_bias_x, 0.0);
    paramNh.param("accel_bias_y", accel_bias_y, 0.0);
    paramNh.param("accel_bias_z", accel_bias_z, 0.0);
    if(accel_bias_x!=0.0 || accel_bias_y!=0.0 || accel_bias_z!=0.0){
        ROS_INFO_STREAM("[xbot_positioning] Will use configured accelerometer bias: " << accel_bias_x << ", " << accel_bias_y << ", " << accel_bias_z);
        accel_bias.setValue(accel_bias_x,accel_bias_y,accel_bias_z);
        skip_accelermoter_calibration = true;
    }
    if(!findStaticTransform("base_link", "gps", antenna_offset, n)){
        return 1;
    }
    
    core.setAntennaOffset(antenna_offset);

    ROS_INFO_STREAM("[xbot_positioning] Antenna offset: " << antenna_offset.x() << ", " << antenna_offset.y() << ", " << antenna_offset.z());

    if (gyro_bias_z != 0.0 && skip_gyro_calibration) {
        gyro_bias.setZ(gyro_bias_z);
        ROS_WARN_STREAM("[xbot_positioning] Using gyro z offset of: " << gyro_bias.z());
    }

    odometry_3d_pub = paramNh.advertise<nav_msgs::Odometry>("odom_3d_out", 50);

    if(publish_2d_odom) {
        odometry_2d_pub = paramNh.advertise<nav_msgs::Odometry>("odom_2d_out", 50);
    }

    xbot_absolute_pose_pub = paramNh.advertise<xbot_msgs::AbsolutePose>("xb_pose_out", 50);

    if (publish_debug) {
        dbg_expected_motion_vector = paramNh.advertise<geometry_msgs::Vector3Stamped>("debug_expected_motion_vector", 50);
        kalman_state = paramNh.advertise<xbot_positioning::KalmanState>("kalman_state", 50);
        gps_velocity_pub = paramNh.advertise<geometry_msgs::TwistStamped>("gps_vel", 50);
        gps_baselink_pub = paramNh.advertise<geometry_msgs::Vector3Stamped>("gps_baselink", 50);
    }

    ros::Subscriber imu_sub = paramNh.subscribe("imu_in", 10, onImu);
    ros::Subscriber twist_sub = paramNh.subscribe("twist_in", 10, onTwistIn);
    ros::Subscriber gps_pose_sub = paramNh.subscribe("xb_pose_in", 10, onGpsPose);
    ros::Subscriber initial_pose_sub = paramNh.subscribe("/initialpose", 10, onInitialPose);
    #ifdef WHEEL_TICKS_MSG
        ros::Subscriber wheel_tick_sub = paramNh.subscribe("wheel_ticks_in", 10, onWheelTicks);
    #endif
    //gps_enabled = false;

    ros::spin();
    return 0;
}
