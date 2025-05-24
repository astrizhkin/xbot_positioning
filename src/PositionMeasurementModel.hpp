#ifndef KALMAN_EXAMPLES_ROBOT1_POSITIONMEASUREMENTMODEL_HPP_
#define KALMAN_EXAMPLES_ROBOT1_POSITIONMEASUREMENTMODEL_HPP_

#include <kalman/LinearizedMeasurementModel.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "SystemModel.hpp"

namespace xbot {
namespace positioning {

/**
 * @brief Measurement vector measuring the robot position
 *
 * @param T Numeric scalar type
 */
template<typename T>
class PositionMeasurement : public Kalman::Vector<T, 3> {
public:
    KALMAN_VECTOR(PositionMeasurement, T, 3)
    
    //! X position of the GPS antenna
    static constexpr size_t GX = 0;
    
    //! Y position of the GPS antenna
    static constexpr size_t GY = 1;

    //! Z position of the GPS antenna
    static constexpr size_t GZ = 2;

    T gps_x()       const { return (*this)[ GX ]; }
    T gps_y()       const { return (*this)[ GY ]; }
    T gps_z()       const { return (*this)[ GZ ]; }
    
    T& gps_x()      { return (*this)[ GX ]; }
    T& gps_y()      { return (*this)[ GY ]; }
    T& gps_z()      { return (*this)[ GZ ]; }
};

/**
 * @brief Measurement model for measuring the position of the robot
 *        using two beacon-landmarks
 *
 * This is the measurement model for measuring the position of the robot.
 * The measurement is given by two landmarks in the space, whose positions are known.
 * The robot can measure the direct distance to both the landmarks, for instance
 * through visual localization techniques.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class PositionMeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, PositionMeasurement<T>, CovarianceBase> {
public:
    //! State type shortcut definition
    typedef  xbot::positioning::State<T> S;
    
    //! Measurement type shortcut definition
    typedef  xbot::positioning::PositionMeasurement<T> M;
    
    /**
     * @brief Constructor
     *
     * @param landmark1x The x-position of landmark 1
     * @param landmark1y The y-position of landmark 1
     * @param landmark2x The x-position of landmark 2
     * @param landmark2y The y-position of landmark 2
     */
    PositionMeasurementModel() {
        // Setup noise jacobian. As this one is static, we can define it once
        // and do not need to update it dynamically
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
    M h(const S& x) const {
        M measurement;

        // Calculate the GPS antenna position given the current system state.
        tf2::Quaternion q;
        q.setRPY(x.roll(),x.pitch(),x.yaw());
        tf2::Vector3 antennaRotated = tf2::quatRotate(q, antenna_offset);
        measurement.gps_x() = x.x_pos() + antennaRotated.x();
        measurement.gps_y() = x.y_pos() + antennaRotated.y();
        measurement.gps_z() = x.z_pos() + antennaRotated.z();

        //old implementation
        //measurement.gps_x() = x.x_pos() + std::cos(x.yaw()) * antenna_offset_x - std::sin(x.yaw()) * antenna_offset_y;
        //measurement.gps_y() = x.y_pos() + std::sin(x.yaw()) * antenna_offset_x + std::cos(x.yaw()) * antenna_offset_y;
        //measurement.gps_z() = x.z_pos() + antenna_offset_z;
        return measurement;
    }

    tf2::Vector3 antenna_offset;

protected:

    /**
     * @brief Update jacobian matrices for the system state transition function using current state
     *
     * This will re-compute the (state-dependent) elements of the jacobian matrices
     * to linearize the non-linear measurement function \f$h(x)\f$ around the
     * current state \f$x\f$.
     *
     * @note This is only needed when implementing a LinearizedSystemModel,
     *       for usage with an ExtendedKalmanFilter or SquareRootExtendedKalmanFilter.
     *       When using a fully non-linear filter such as the UnscentedKalmanFilter
     *       or its square-root form then this is not needed.
     *
     * @param x The current system state around which to linearize
     * @param u The current system control input
     */


    /**
     * @brief Update jacobian matrices for the measurement function using current state
     *
     * This will compute the elements of the jacobian matrix H to linearize
     * the non-linear measurement function h(x) around the current state x.
     * The H matrix represents how changes in each state variable affect the expected measurements.
     *
     * @param x The current system state around which to linearize
     */
    /* Mathematically correct but problematic full implementaion. It has effect on RPY based on GPS XYZ readings
    void updateJacobians(const S& x)
    {
        // Initialize the H matrix to zeros
        this->H.setZero();

        // The first 3 columns of H represent how position measurements change with respect to position states
        // These are simply identity because a change in position directly translates to a change in measurement
        this->H(M::GX, S::X) = 1.0;
        this->H(M::GY, S::Y) = 1.0;
        this->H(M::GZ, S::Z) = 1.0;

        We don't want position updates to affect roll, pitch and yaw. Do not initialize jacobian elemnts 
        // The next columns represent how position measurements change with respect to orientation states (roll, pitch, yaw)
        // These are more complex because they involve derivatives of the rotation function

        // First, calculate the rotated antenna offset at the current orientation
        tf2::Quaternion q;
        q.setRPY(x.roll(), x.pitch(), x.yaw());
        tf2::Vector3 antennaRotated = tf2::quatRotate(q, antenna_offset);
    
        // Small angle perturbations for numerical differentiation
        const double delta = 1e-6;
        
        // Calculate derivatives with respect to roll
        {
            tf2::Quaternion q_delta;
            q_delta.setRPY(x.roll() + delta, x.pitch(), x.yaw());
            tf2::Vector3 antennaRotated_plus = tf2::quatRotate(q_delta, antenna_offset);
            
            // Approximate derivatives using finite differences
            this->H(M::GX, S::ROLL) = (antennaRotated_plus.x() - antennaRotated.x()) / delta;
            this->H(M::GY, S::ROLL) = (antennaRotated_plus.y() - antennaRotated.y()) / delta;
            this->H(M::GZ, S::ROLL) = (antennaRotated_plus.z() - antennaRotated.z()) / delta;
        }
        
        // Calculate derivatives with respect to pitch
        {
            tf2::Quaternion q_delta;
            q_delta.setRPY(x.roll(), x.pitch() + delta, x.yaw());
            tf2::Vector3 antennaRotated_plus = tf2::quatRotate(q_delta, antenna_offset);
            
            // Approximate derivatives using finite differences
            this->H(M::GX, S::PITCH) = (antennaRotated_plus.x() - antennaRotated.x()) / delta;
            this->H(M::GY, S::PITCH) = (antennaRotated_plus.y() - antennaRotated.y()) / delta;
            this->H(M::GZ, S::PITCH) = (antennaRotated_plus.z() - antennaRotated.z()) / delta;
        }
        
        // Calculate derivatives with respect to yaw
        {
            tf2::Quaternion q_delta;
            q_delta.setRPY(x.roll(), x.pitch(), x.yaw() + delta);
            tf2::Vector3 antennaRotated_plus = tf2::quatRotate(q_delta, antenna_offset);
            
            // Approximate derivatives using finite differences
            this->H(M::GX, S::YAW) = (antennaRotated_plus.x() - antennaRotated.x()) / delta;
            this->H(M::GY, S::YAW) = (antennaRotated_plus.y() - antennaRotated.y()) / delta;
            this->H(M::GZ, S::YAW) = (antennaRotated_plus.z() - antennaRotated.z()) / delta;
        }
        // The remaining columns represent how position measurements change with respect to speeds
        // For a pure position measurement, changes in speed don't directly affect the measurement
        // So these elements remain zero
    }
        */    


    void updateJacobians( const S& x ) {
      this->H.setIdentity();
    }
};

} // namespace Robot
} // namespace KalmanExamples

#endif