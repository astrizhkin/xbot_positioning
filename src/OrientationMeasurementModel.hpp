#ifndef KALMAN_EXAMPLES_ROBOT1_ORIENTATIONMEASUREMENTMODEL_HPP_
#define KALMAN_EXAMPLES_ROBOT1_ORIENTATIONMEASUREMENTMODEL_HPP_

#include <kalman/LinearizedMeasurementModel.hpp>
#include "SystemModel.hpp"

namespace xbot {
    namespace positioning {

/**
 * @brief Measurement vector measuring an orientation (i.e. by using a compass)
 *
 * @param T Numeric scalar type
 */
template<typename T>
class OrientationMeasurement : public Kalman::Vector<T, 2> {
public:
    KALMAN_VECTOR(OrientationMeasurement, T, 2)
    
    //! Orientation
    static constexpr size_t ROLL = 0;
    static constexpr size_t PITCH = 1;
    //static constexpr size_t YAW = 2;
    
    T roll()  const { return (*this)[ ROLL ]; }
    T pitch()  const { return (*this)[ PITCH ]; }
    //T yaw()  const { return (*this)[ YAW ]; }

    T& roll() { return (*this)[ ROLL ]; }
    T& pitch() { return (*this)[ PITCH ]; }
    //T& yaw() { return (*this)[ YAW ]; }
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
class OrientationMeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, OrientationMeasurement<T>, CovarianceBase> {
public:
    //! State type shortcut definition
    typedef xbot::positioning::State<T> S;
    
    //! Measurement type shortcut definition
    typedef  xbot::positioning::OrientationMeasurement<T> M;
    
    OrientationMeasurementModel() {
        // Setup jacobians. As these are static, we can define them once
        // and do not need to update them dynamically
        this->H.setZero();
        this->V.setIdentity();

        this->H( M::ROLL, S::ROLL) = 1;
        this->H( M::PITCH, S::PITCH ) = 1;
        //this->H( M::YAW, S::YAW ) = 1;
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
        
        // Measurement is given by the actual robot orientation
        measurement.roll() = x.roll();
        measurement.pitch() = x.pitch();
        //measurement.yaw() = x.yaw();
        
        return measurement;
    }
};

} // namespace Robot
} // namespace KalmanExamples

#endif