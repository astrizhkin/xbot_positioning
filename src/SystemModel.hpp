//
// Created by Clemens Elflein on 27.10.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//

#ifndef SRC_SYSTEMMODEL_HPP
#define SRC_SYSTEMMODEL_HPP

#include <kalman/LinearizedSystemModel.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "ros/ros.h"


namespace xbot {
    namespace positioning {

/**
 * @brief System state vector-type for a 3DOF planar robot
 *
 * This is a system state for a very simple planar robot that
 * is characterized by its (x,y)-Position and angular orientation.
 *
 * @param T Numeric scalar type
 */
        template<typename T>
        class State : public Kalman::Vector<T, 8> {
        public:
            KALMAN_VECTOR(State, T, 8)

            //! X-position
            static constexpr size_t X = 0;
            //! Y-Position
            static constexpr size_t Y = 1;
            //! Z-Position
            static constexpr size_t Z = 2;
            //! Orientation
            static constexpr size_t ROLL = 3;
            //! Orientation
            static constexpr size_t PITCH = 4;
            //! Orientation
            static constexpr size_t YAW = 5;
            //! Speed lenaar
            static constexpr size_t SL = 6;
            //! Speed angular
            static constexpr size_t SA = 7;

            T x_pos() const { return (*this)[X]; }
            T y_pos() const { return (*this)[Y]; }
            T z_pos() const { return (*this)[Z]; }

            T roll() const { return (*this)[ROLL]; }
            T pitch() const { return (*this)[PITCH]; }
            T yaw() const { return (*this)[YAW]; }

            T sl() const { return (*this)[SL]; }
            T sa() const { return (*this)[SA]; }

            T &x_pos() { return (*this)[X]; }
            T &y_pos() { return (*this)[Y]; }
            T &z_pos() { return (*this)[Z]; }

            T &roll() { return (*this)[ROLL]; }
            T &pitch() { return (*this)[PITCH]; }
            T &yaw() { return (*this)[YAW]; }

            T &sl() { return (*this)[SL]; }
            T &sa() { return (*this)[SA]; }

        };

/**
 * @brief System control-input vector-type for a 3DOF planar robot
 *
 * This is the system control-input of a very simple planar robot that
 * can control the velocity in its current direction as well as the
 * change in direction.
 *
 * @param T Numeric scalar type
 */
        template<typename T>
        class Control : public Kalman::Vector<T, 4> {
        public:
            KALMAN_VECTOR(Control, T, 4)

            //! Velocity
            static constexpr size_t V = 0;
            //! Angular Rate (Orientation-change)
            static constexpr size_t DROLL = 1;
            //! Angular Rate (Orientation-change)
            static constexpr size_t DPITCH = 2;
            //! Angular Rate (Orientation-change)
            static constexpr size_t DYAW = 3;

            T v() const { return (*this)[V]; }

            T droll() const { return (*this)[DROLL]; }
            T dpitch() const { return (*this)[DPITCH]; }
            T dyaw() const { return (*this)[DYAW]; }

            T &v() { return (*this)[V]; }

            T &droll() { return (*this)[DROLL]; }
            T &dpitch() { return (*this)[DPITCH]; }
            T &dyaw() { return (*this)[DYAW]; }
        };

/**
 * @brief System model for a simple planar 3DOF robot
 *
 * This is the system model defining how our robot moves from one
 * time-step to the next, i.e. how the system state evolves over time.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
        template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
        class SystemModel : public Kalman::LinearizedSystemModel<State<T>, Control<T>, CovarianceBase> {
        public:
            //! State type shortcut definition
            typedef xbot::positioning::State <T> S;

            //! Control type shortcut definition
            typedef xbot::positioning::Control <T> C;

            void setDt(double dt) {
                this->dt = dt;
            }

            /**
             * @brief Definition of (non-linear) state transition function
             *
             * This function defines how the system state is propagated through time,
             * i.e. it defines in which state \f$\hat{x}_{k+1}\f$ is system is expected to
             * be in time-step \f$k+1\f$ given the current state \f$x_k\f$ in step \f$k\f$ and
             * the system control input \f$u\f$.
             *
             * @param [in] x The system state in current time-step
             * @param [in] u The control vector input
             * @returns The (predicted) system state in the next time-step
             */
            S f(const S &x, const C &u) const {
                //! Predicted state vector after transition
                S x_;

                // New orientation given by old orientation plus orientation change
                //old simplified implementation
                //auto newRoll = x.roll() + u.droll() * dt;
                //auto newPitch = x.pitch() + u.dpitch() * dt;
                //auto newYaw = x.yaw() + u.dyaw() * dt;

                // Represent current orientation as quaternion (assuming we store or convert from Euler)
                tf2::Quaternion q_current;
                q_current.setRPY(x.roll(), x.pitch(), x.yaw()); // Convert Euler to quaternion

                // Create a quaternion representing the angular change
                tf2::Quaternion q_control;
                q_control.setRPY(u.droll()*dt, u.dpitch()*dt, u.dyaw()*dt);

                // Calculate new quaternion
                tf2::Quaternion q_new = q_current * q_control;

                tf2::Matrix3x3 wm(q_new);
                double newRoll,newPitch,newYaw;
                wm.getRPY(newRoll, newPitch, newYaw);

                //ROS_INFO_STREAM("wfRPY:"<<x.roll()<<","<<x.pitch()<<","<<x.yaw()<< 
                //                " rfvelRPY:"<<u.droll()<<","<<u.dpitch()<<","<<u.dyaw()<<
                //                " wfdRPY:"<<world_dRoll<<","<<world_dPitch<<","<<world_dYaw);
                
                // Re-scale orientation to [-pi to +pi]
                while(newYaw > M_PI){
                    newYaw-= 2 * M_PI;
                }
                while(newYaw < -M_PI){
                    newYaw+= 2 * M_PI; 
                }
                
                x_.roll() = newRoll;
                x_.pitch() = newPitch;
                x_.yaw() = newYaw;

                double cosy = std::cos(newYaw);
                double siny = std::sin(newYaw);
                double cosp = std::cos(newPitch);
                double sinp = std::sin(newPitch);

                // New x-position given by old x-position plus change in x-direction
                // Change in x-direction is given by the cosine of the (new) orientation
                // times the velocity
                x_.x_pos() = x.x_pos() + cosy * cosp * u.v() * dt;
                x_.y_pos() = x.y_pos() + siny * cosp * u.v() * dt;
                x_.z_pos() = x.z_pos() - sinp * u.v() * dt;

//                x_.sl() = x.sl();
//                x_.sa() = x.sa();
                x_.sl() = u.v();
                x_.sa() = u.dyaw();


                // Return transitioned state vector
                return x_;
            }

        protected:
            double dt = 0;
            /**
             * @brief Update jacobian matrices for the system state transition function using current state
             *
             * This will re-compute the (state-dependent) elements of the jacobian matrices
             * to linearize the non-linear state transition function \f$f(x,u)\f$ around the
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
            void updateJacobians(const S &x, const C &u) {
                // F = df/dx (Jacobian of state transition w.r.t. the state)
                this->F.setZero();

                tf2::Quaternion q_current;
                q_current.setRPY(x.roll(), x.pitch(), x.yaw()); // Convert Euler to quaternion

                // Create a quaternion representing the angular change
                tf2::Quaternion q_control;
                q_control.setRPY(u.droll()*dt, u.dpitch()*dt, u.dyaw()*dt);

                // Calculate new quaternion
                tf2::Quaternion q_new = q_current * q_control;

                tf2::Matrix3x3 wm(q_new);
                double newRoll,newPitch,newYaw;
                wm.getRPY(newRoll, newPitch, newYaw);

                double cosy = std::cos(newYaw);
                double siny = std::sin(newYaw);
                double cosp = std::cos(newPitch);
                double sinp = std::sin(newPitch);

                //d sinx / dx = cosx
                //d cosx / dx = -sinx
                this->F(S::X, S::X) = 1;
                // partial differential of (x.x() + cosy * cosp * u.v() * dt)
                this->F(S::X, S::YAW) =   -siny * cosp * u.v() * dt;
                this->F(S::X, S::PITCH) = -cosy * sinp * u.v() * dt;

                this->F(S::Y, S::Y) = 1;
                // partial differential of (x.y() + siny * cosp * u.v() * dt)
                this->F(S::Y, S::YAW) =    cosy * cosp * u.v() * dt;
                this->F(S::Y, S::PITCH) = -siny * sinp * u.v() * dt;

                this->F(S::Z, S::Z) = 1;
                // partial differential of (x.z() - sinp * u.v() * dt)
                this->F(S::Z, S::PITCH) =  - cosp * u.v() * dt;

                this->F(S::ROLL, S::ROLL) = 1;
                this->F(S::PITCH, S::PITCH) = 1;
                this->F(S::YAW, S::YAW) = 1;

                this->F(S::SL, S::SL) = 1;
                this->F(S::SA, S::SA) = 1;

                // W = df/dw (Jacobian of state transition w.r.t. the noise)
                this->W.setIdentity();
                // TODO: more sophisticated noise modelling
                //       i.e. The noise affects the the direction in which we move as
                //       well as the velocity (i.e. the distance we move)
            }
        };

    } // namespace Robot
}

#endif //SRC_SYSTEMMODEL_HPP
