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
class OrientationMeasurement2 : public Kalman::Vector<T, 3>
{
public:
    KALMAN_VECTOR(OrientationMeasurement2, T, 3)
    
    //! Orientation
    static constexpr size_t VX = 0;
    static constexpr size_t VY = 1;
    static constexpr size_t VZ = 2;

    T vx()  const { return (*this)[ VX ]; }
    T& vx() { return (*this)[ VX ]; }
    T vy()  const { return (*this)[ VY ]; }
    T& vy() { return (*this)[ VY ]; }
    T vz()  const { return (*this)[ VZ ]; }
    T& vz() { return (*this)[ VZ ]; }

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
        
        double cosy = std::cos(x.yaw());
        double siny = std::sin(x.yaw());
        double cosp = std::cos(x.pitch());
        double sinp = std::sin(x.pitch());

        //it should use all antenna offsets
        measurement.vx() = x.sl() * cosy * cosp - siny * antenna_offset.x() * x.sa() - cosy * antenna_offset.y() * x.sa();
        measurement.vy() = x.sl() * siny * cosp + cosy * antenna_offset.x() * x.sa() - siny * antenna_offset.y() * x.sa();
        measurement.vz() = - x.sl() * sinp;
        return measurement;
    }

    void updateJacobians( const S& x )
    {
        this->H.setZero();

        // partial derivative of meas A w.r.t. B
        double cosy = std::cos(x.yaw());
        double siny = std::sin(x.yaw());
        double cosp = std::cos(x.pitch());
        double sinp = std::sin(x.pitch());

        //it should use all antenna offsets
        this->H( M::VX, S::YAW ) = -x.sl() * siny * cosp - antenna_offset.x() * x.sa() * cosy + siny * antenna_offset.y() * x.sa();
        this->H( M::VY, S::YAW ) = x.sl() * cosy * cosp - antenna_offset.x() * x.sa() * siny - cosy * antenna_offset.y() * x.sa();
        //this->H( M::VZ, S::YAW ) = 1;

        //this->H( M::VX, S::PITCH ) = 
        //this->H( M::VY, S::PITCH ) = 
        this->H( M::VZ, S::PITCH ) = - x.sl() * sinp
    }


    tf2::Vector3 antenna_offset;
};

} // namespace Robot
} // namespace KalmanExamples

#endif