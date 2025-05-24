#ifndef KALMAN_EXAMPLES_ROBOT1_ORIENTATIONMEASUREMENTMODEL2_HPP_
#define KALMAN_EXAMPLES_ROBOT1_ORIENTATIONMEASUREMENTMODEL2_HPP_

#include <kalman/LinearizedMeasurementModel.hpp>
#include "SystemModel.hpp"

namespace xbot
{
    namespace positioning
    {

/**
 * @brief Measurement vector measuring an orientation (i.e. by using a compass)
 *
 * @param T Numeric scalar type
 */
template<typename T>
class OrientationMeasurement2 : public Kalman::Vector<T, 2>
{
public:
    KALMAN_VECTOR(OrientationMeasurement2, T, 2)
    
    //! Orientation
    static constexpr size_t GVX = 0;
    static constexpr size_t GVY = 1;

    T gps_vx()  const { return (*this)[ GVX ]; }
    T& gps_vx() { return (*this)[ GVX ]; }
    T gps_vy()  const { return (*this)[ GVY ]; }
    T& gps_vy() { return (*this)[ GVY ]; }
};

/**
 * @brief Measurement model for measuring orientation of a 3DOF robot
 *
 * This is the measurement model for measuring the orientation of our
 * planar robot. This could be realized by a compass / magnetometer-sensor.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class OrientationMeasurementModel2 : public Kalman::LinearizedMeasurementModel<State<T>, OrientationMeasurement2<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef xbot::positioning::State<T> S;
    
    //! Measurement type shortcut definition
    typedef  xbot::positioning::OrientationMeasurement2<T> M;
    
    OrientationMeasurementModel2()
    {
        // Setup jacobians. As these are static, we can define them once
        // and do not need to update them dynamically
        this->H.setIdentity();
        this->V.setIdentity();
    }
    
    /**
     * @brief Definition of (possibly non-linear) measurement function
     *
     * This function maps the system state to the measurement that is expected
     * to be received from the sensor assuming the system is currently in the
     * estimated state.
     *
     * @param [in] x The system state in current time-step
     * @returns The (predicted) sensor measurement for the system state
     */
    M h(const S& x) const
    {
        // Measurement is given by the actual robot orientation
        M measurement;
        
        // Convert Euler angles to quaternion for consistent rotation
        tf2::Quaternion q;
        q.setRPY(x.roll(), x.pitch(), x.yaw());
        
        // The linear velocity is already in the robot's body frame
        // We need to transform it to the global frame
        tf2::Vector3 velocity_body(x.sl(), 0.0, 0.0); // Linear velocity along robot's x-axis
        
        // Transform to global frame using quaternion rotation
        // quatRotate rotates a vector by a quaternion: result = q * v * q^(-1)
        tf2::Vector3 velocity_global = tf2::quatRotate(q, velocity_body);
        
        // GNSS measures velocity at the antenna position, not at the robot's center
        // We need to account for the rotational effect on the antenna
        // Angular velocity in body frame
        tf2::Vector3 angular_velocity_body(0.0, 0.0, x.sa()); // Assuming rotation around body z-axis
        
        // Rotational effect is in the body frame - we need to transform the antenna offset to body frame
        // and then transform the result back to global frame
        tf2::Vector3 antenna_offset_body = antenna_offset; // Antenna offset is already in body frame
        tf2::Vector3 rot_effect_body = angular_velocity_body.cross(antenna_offset_body);
        tf2::Vector3 rot_effect_global = tf2::quatRotate(q, rot_effect_body);
        
        // Total velocity at GNSS antenna = linear velocity + rotational effect
        measurement.gps_vx() = velocity_global.x() + rot_effect_global.x();
        measurement.gps_vy() = velocity_global.y() + rot_effect_global.y();

        return measurement;
    }

    /*double alignMotionHeading(double coreYaw, const xbot_msgs::AbsolutePose::ConstPtr &msg) {
        double motion_heading = msg->motion_heading;
        //TODO: motion heading is not populated in old gps driver versions
        if(motion_heading==0) {
            motion_heading = std::atan2(msg->motion_vector.y, msg->motion_vector.x);
        }
    
        double current_yaw = coreYaw;
        double best_yaw_diff = current_yaw - motion_heading;
        double best_heading_yaw = motion_heading;
        while(true) {
            double diff = current_yaw - motion_heading;
            if(abs(diff) > abs(best_yaw_diff)) {
                break;
            }else{
                best_yaw_diff = diff;
                best_heading_yaw = motion_heading;
            }
            if(motion_heading > current_yaw){
                motion_heading -= M_PI * 2;
            } else {
                motion_heading += M_PI * 2;
            }
        }
    
        return best_heading_yaw;
    }*/

    void updateJacobians( const S& x )
    {
        // Initialize H matrix to zeros
        this->H.setZero();
        
        // Use numerical differentiation for accurate Jacobian computation
        const double delta = 1e-6; // Small perturbation
        
        // Base measurement at current state
        M base_measurement = h(x);
        
        // For each state variable, compute partial derivatives
        S perturbed_state = x;
        
        // Partial derivatives with respect to orientation (roll, pitch, yaw)
        // For roll
        //perturbed_state = x;
        //perturbed_state.roll() = x.roll() + delta;
        //M perturbed_measurement = h(perturbed_state);
        //this->H(M::GVX, S::ROLL) = (perturbed_measurement.gps_vx() - base_measurement.gps_vx()) / delta;
        //this->H(M::GVY, S::ROLL) = (perturbed_measurement.gps_vy() - base_measurement.gps_vy()) / delta;
        
        // For pitch
        //perturbed_state = x;
        //perturbed_state.pitch() = x.pitch() + delta;
        //perturbed_measurement = h(perturbed_state);
        //this->H(M::GVX, S::PITCH) = (perturbed_measurement.gps_vx() - base_measurement.gps_vx()) / delta;
        //this->H(M::GVY, S::PITCH) = (perturbed_measurement.gps_vy() - base_measurement.gps_vy()) / delta;
        
        // For yaw (MOST IMPORTANT COMPONENT REQUIRED FOR INDIRECT YAW UPDATES)
        perturbed_state = x;
        perturbed_state.yaw() = x.yaw() + delta;
        M perturbed_measurement = h(perturbed_state);
        this->H(M::GVX, S::YAW) = (perturbed_measurement.gps_vx() - base_measurement.gps_vx()) / delta;
        this->H(M::GVY, S::YAW) = (perturbed_measurement.gps_vy() - base_measurement.gps_vy()) / delta;
        
        // For linear speed (sl)
        perturbed_state = x;
        perturbed_state.sl() = x.sl() + delta;
        perturbed_measurement = h(perturbed_state);
        this->H(M::GVX, S::SL) = (perturbed_measurement.gps_vx() - base_measurement.gps_vx()) / delta;
        this->H(M::GVY, S::SL) = (perturbed_measurement.gps_vy() - base_measurement.gps_vy()) / delta;
        
        // For angular speed (sa)
        //perturbed_state = x;
        //perturbed_state.sa() = x.sa() + delta;
        //perturbed_measurement = h(perturbed_state);
        //this->H(M::GVX, S::SA) = (perturbed_measurement.gps_vx() - base_measurement.gps_vx()) / delta;
        //this->H(M::GVY, S::SA) = (perturbed_measurement.gps_vy() - base_measurement.gps_vy()) / delta;
    }


    tf2::Vector3 antenna_offset;
};

} // namespace Robot
} // namespace KalmanExamples

#endif