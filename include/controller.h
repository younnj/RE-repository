#define _USE_MATH_DEFINES
#include <cmath>

#ifndef CROBOTICS_CONTROLLER_H
#define CROBOTICS_CONTROLLER_H

#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/joint/joint-collection.hpp>
#include <fstream>
#include <iostream>
#include "math_type_define.h"

using namespace Eigen;

class cRoboticsController
{
    public:
        cRoboticsController(const std::string& urdf_path, 
                            const std::string& manipulator_control_mode, 
                            const double& dt);

        std::string manipulator_control_mode_; // position or torque
        double dt_;
            
        // pinocchio data
        pinocchio::Model model_;
        pinocchio::Data data_;

        std::ofstream logging_file_1;
        std::ofstream logging_file_2;
        std::ofstream logging_file_3;

        // =============================================================================
        // ================================= User data =================================
        // =============================================================================
        // control mode state
        bool is_mode_changed_{false};
        enum CTRL_MODE {
            joint_ctrl_home,
            joint_ctrl_init,
            
            // -------------------------------------
            // TODO 1: Add your control modes here
            // Example:
            hw2_Jacobian,
            hw2_CLIK,
            hw2_CLIK_WPI,
            torque_ctrl_dynamic,
            // -------------------------------------
            
            DEFAULT
        };
        CTRL_MODE control_mode_{DEFAULT};

        // time state
        double play_time_;
        double control_start_time_;
        double q_t_time;

        // Current Joint space state
        VectorXd q_;           // joint angle (7,1)
        VectorXd qdot_;        // joint velocity (7,1)
        VectorXd tau_;         // joint torque (7,1)

        // Cntrol Input 
        VectorXd q_desired_;   // desired joint angle (7,1)  -> using for position mode
        VectorXd tau_desired_; // desired joint torque (7,1) -> using for torque mode

        // Initial joint space state
        VectorXd q_init_;      // initial joint angle (7,1)
        VectorXd qdot_init_;   // initial joint velocity (7,1)

        // Task space state
        std::vector<std::string> link_names_{"fr3_link0", 
                                             "fr3_link1", 
                                             "fr3_link2",
                                             "fr3_link3",
                                             "fr3_link4",
                                             "fr3_link5",
                                             "fr3_link6",
                                             "fr3_link7"};
        std::string ee_name_{"attachment_site"};
        Matrix4d x_;                            // Homogeneous matrix; pose of EE (4,4)
        VectorXd xdot_;                         // velocity of link4 (6,1); linear + angular
        MatrixXd J_;                            // jacobian of EE (6,7)
        Matrix4d x_init_;                       // Homogeneous matrix; initial pose of EE (4,4)
        Matrix4d x2_;                           // Homogeneous matrix; pose of link4 (4,4)
        VectorXd x2dot_;                        // velocity of link4 (6,1); linear + angular
        MatrixXd J2_;                           // jacobian of link4 (6,7)
        Matrix4d x2_init_;                      // Homogeneous matrix; initial pose of link4 (4,4)
        VectorXd q_t;
        
        // Joint space Dynamics
        MatrixXd M_;     // mass matrix (7,7)
        VectorXd g_;     // gravity torques (7,1)
        VectorXd c_;     // centrifugal and coriolis forces (7,1)

        // =============================================================================
        // =============================== User functions ==============================
        // =============================================================================
        // Control functions
        void moveJointPosition(const VectorXd& target_position, double duration);
        void moveJointPositionTorque(const VectorXd& target_position, double duration);

        // ----------------------------------------------
        // TODO 2: Add your control function here
        // Example:
        void HW2_Jacobian();
        void HW2_CLIK();
        void HW2_CLIK_WPI();
        void torqueCtrlDynamic();
        // ----------------------------------------------

        // ============================================================================
        // Utils functions
        Matrix4d getLinkPose(const VectorXd& q, const std::string& link_name);
        MatrixXd getLinkJac(const VectorXd& q, const std::string& link_name);
        Matrix4d getEEPose(const VectorXd& q);
        MatrixXd getEEJac(const VectorXd& q);
        MatrixXd getMassMatrix(const VectorXd& q);
        VectorXd getGravityVector(const VectorXd& q);

        // Core functions
        void keyMapping(const int &key);
        void compute(const double& play_time);
        void setMode(const CTRL_MODE& control_mode);
        void printState();
        bool updateModel(const VectorXd& q, const VectorXd& qdot, const VectorXd& tau);
        bool updateKinematics(const VectorXd& q, const VectorXd& qdot);
        bool updateDynamics(const VectorXd& q, const VectorXd& qdot);
        VectorXd getCtrlInput();

};

#endif // CROBOTICS_CONTROLLER_H