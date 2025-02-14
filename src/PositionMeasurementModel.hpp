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
    static constexpr size_t X = 0;
    
    //! Y position of the GPS antenna
    static constexpr size_t Y = 1;

    //! Z position of the GPS antenna
    static constexpr size_t Z = 2;

    T x_pos()       const { return (*this)[ X ]; }
    T y_pos()       const { return (*this)[ Y ]; }
    T z_pos()       const { return (*this)[ Z ]; }
    
    T& x_pos()      { return (*this)[ X ]; }
    T& y_pos()      { return (*this)[ Y ]; }
    T& z_pos()      { return (*this)[ Z ]; }
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
        tf2::Vector3 antenna(antenna_offset_x, antenna_offset_y, antenna_offset_z);
        tf2::Quaternion q;
        q.setRPY(x.roll(),x.pitch(),x.yaw());
        tf2::Vector3 antennaRotated = tf2::quatRotate(q,antenna);
        measurement.x_pos() = x.x_pos() + antennaRotated.x();
        measurement.y_pos() = x.y_pos() + antennaRotated.y();
        measurement.z_pos() = x.z_pos() + antennaRotated.z();

        //old implementation
        //TODO: implement 3d shift using all 3 offsets
        //TODO: should we use pitch and roll here?
        //measurement.x_pos() = x.x_pos() + std::cos(x.yaw()) * antenna_offset_x - std::sin(x.yaw()) * antenna_offset_y;
        //measurement.y_pos() = x.y_pos() + std::sin(x.yaw()) * antenna_offset_x + std::cos(x.yaw()) * antenna_offset_y;
        //TODO: plus or minus????
        //measurement.z_pos() = x.z_pos() + antenna_offset_z;
        return measurement;
    }

    double antenna_offset_x = 0;
    double antenna_offset_y = 0;
    double antenna_offset_z = 0;

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
    void updateJacobians( const S& x ) {
      this->H.setIdentity();
    }
};

} // namespace Robot
} // namespace KalmanExamples

#endif