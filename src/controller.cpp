#include "controller.h"

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>

cRoboticsController::cRoboticsController(const std::string& urdf_path, 
                                         const std::string& manipulator_control_mode, 
                                         const double& dt)
: manipulator_control_mode_(manipulator_control_mode),
  dt_(dt)
{
    pinocchio::urdf::buildModel(urdf_path, model_);
    data_ = pinocchio::Data(model_);

    // Initialize joint space state
    q_ = VectorXd::Zero(model_.nq);
    qdot_ = VectorXd::Zero(model_.nq);
    tau_ = VectorXd::Zero(model_.nq);
    q_desired_ = VectorXd::Zero(model_.nq);
    tau_desired_ = VectorXd::Zero(model_.nq);
    q_init_ = VectorXd::Zero(model_.nq);
    qdot_init_ = VectorXd::Zero(model_.nq);

    // Initialize task space state
    x_ = Matrix4d::Identity();
    xdot_ = VectorXd::Zero(6);
    J_ = MatrixXd::Zero(6, model_.nv);
    x2_ = Matrix4d::Identity();
    x2dot_ = VectorXd::Zero(6);
    J2_ = MatrixXd::Zero(6, model_.nv);
    q_t = VectorXd::Zero(7);

    // Initialize joint space dynamics
    M_ = MatrixXd::Zero(model_.nq, model_.nv);
    g_ = VectorXd::Zero(model_.nq);       
    c_ = VectorXd::Zero(model_.nq);       

    logging_file_1.open("log_jac.txt");
    logging_file_2.open("log_clik.txt");
    logging_file_3.open("log_wpi.txt");
}

void cRoboticsController::keyMapping(const int &key)
{
    switch (key)
    {
    // Implement with user input
    case '1':
       setMode(joint_ctrl_init);
        break;
    case '2':
       setMode(joint_ctrl_home);
        break;
    
    // --------------------------------------------------------------------------------------
    // TODO 3: Add your keyboard mapping here (using the control modes from TODO 1)
    // Example:
    case '3':
       setMode(hw2_Jacobian);
        break;
    case '4':
       setMode(hw2_CLIK);
        break;
    case '5':
       setMode(hw2_CLIK_WPI);
        break;
    case '6':
       setMode(torque_ctrl_dynamic);
        break;
    // --------------------------------------------------------------------------------------
    
    default:
        break;
    }
}

void cRoboticsController::compute(const double& play_time)
{
    play_time_ = play_time;
    if(is_mode_changed_)
    {
        is_mode_changed_ = false;
        control_start_time_ = play_time_;
        q_t_time = play_time_;
        q_init_ = q_;
        qdot_init_ = qdot_;
        x_init_ = x_;
        x2_init_ = x2_;
        q_t = q_;
    }

    switch (control_mode_)
    {
        case joint_ctrl_init:
        {
            Vector7d target_position;
            target_position << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, M_PI / 4.;
            if(manipulator_control_mode_ == "position") moveJointPosition(target_position, 2.0);
            else moveJointPositionTorque(target_position, 2.0);
            break;
        }
        case joint_ctrl_home:
        {
            Vector7d target_position;
            target_position << 0.477, 0.286, 0.384, -0.679, 0.197, -0.742, 0.000;
            //target_position << 0.0, 0.0, 0.0, -M_PI/2., 0.0, M_PI/2., M_PI / 4.;
            if(manipulator_control_mode_ == "position") moveJointPosition(target_position, 2.0);
            else moveJointPositionTorque(target_position, 2.0);
            break;
        }
        // ----------------------------------------------------------------------------
        // TODO 4: Add your control logic here (using the control functions from TODO 5)
        // Example:
        case hw2_Jacobian:
        {
            HW2_Jacobian();
            break;
        }
        case hw2_CLIK:
        {
            HW2_CLIK();
            break;
        }
        case hw2_CLIK_WPI:
        {
            HW2_CLIK_WPI();
            break;
        }
        case torque_ctrl_dynamic:
        {
            torqueCtrlDynamic();
        }
        // ----------------------------------------------------------------------------
        
        default:
            if(manipulator_control_mode_ == "torque") tau_desired_ = g_;
            break;
    }
}

// =============================================================================
// =============================== User functions ==============================
// =============================================================================
void cRoboticsController::moveJointPosition(const VectorXd& target_position, double duration)
{
	Vector7d zero_vector;
	zero_vector.setZero();
	q_desired_ = DyrosMath::cubicVector<7>(play_time_,
		                                   control_start_time_,
		                                   control_start_time_ + duration, 
                                           q_init_, 
                                           target_position, 
                                           zero_vector, 
                                           zero_vector);
}

void cRoboticsController::moveJointPositionTorque(const VectorXd& target_position, double duration)
{
	Matrix7d kp, kv;
	Vector7d q_cubic, qd_cubic;
	
	kp = Matrix7d::Identity() * 500.0;
	kv = Matrix7d::Identity() * 20.0;

	for (int i = 0; i < 7; i++)
	{
		qd_cubic(i) = DyrosMath::cubicDot(play_time_, 
                                          control_start_time_,
			                              control_start_time_ + duration, q_init_(i), 
                                          target_position(i), 
                                          0, 
                                          0);
		q_cubic(i) = DyrosMath::cubic(play_time_,
                                      control_start_time_,
			                          control_start_time_ + duration, 
                                      q_init_(i), 
                                      target_position(i), 
                                      0, 
                                      0);
	}


	tau_desired_ = M_ * (kp*(q_cubic - q_) + kv*(qd_cubic - qdot_)) + g_;
}

// -------------------------------------
// TODO 5: Add your control functions here
// Example:
void cRoboticsController::HW2_Jacobian()
{
    Matrix4d EEPos_init_Matrix = getEEPose(q_init_);
    Vector3d EEPos_init;
    EEPos_init.setZero();
    EEPos_init(0) = EEPos_init_Matrix(0,3);
    EEPos_init(1) = EEPos_init_Matrix(1,3);
    EEPos_init(2) = EEPos_init_Matrix(2,3);

    Vector3d EEPos_target;
    EEPos_target.setZero();
    EEPos_target(0) = 0.25;
    EEPos_target(1) = 0.28;
    EEPos_target(2) = 0.65;

    Vector3d zero_vector;
	zero_vector.setZero();
    
	Vector3d x_desired_ = DyrosMath::cubicVector<3>(play_time_,
		                                   control_start_time_,
		                                   control_start_time_ + 2, 
                                           EEPos_init, 
                                           EEPos_target, 
                                           zero_vector, 
                                           zero_vector);

    Vector3d x_desired_dot_ = DyrosMath::cubicDotVector<3>(play_time_,
		                                   control_start_time_,
		                                   control_start_time_ + 2, 
                                           EEPos_init, 
                                           EEPos_target, 
                                           zero_vector, 
                                           zero_vector);

    MatrixXd EEJac = getEEJac(q_);
    MatrixXd CurrentEEJac(3, 7);
    CurrentEEJac = EEJac.topRows(3);
    

    double lambda = 0.01; 
    MatrixXd J_JT = CurrentEEJac * CurrentEEJac.transpose(); 
    MatrixXd I_damped = J_JT + lambda * lambda * MatrixXd::Identity(CurrentEEJac.rows(), CurrentEEJac.rows());
    MatrixXd J_pinv = CurrentEEJac.transpose() * I_damped.inverse(); 
    //MatrixXd J_pinv = CurrentEEJac.transpose() * J_JT.inverse();

    Vector7d target_position;
    target_position = q_t + J_pinv * x_desired_dot_ * (play_time_ - q_t_time);
    q_t = target_position;
    q_t_time = play_time_;
    
    q_desired_ = target_position;
    
    Vector3d x_current;
    x_current.setZero();
    x_current(0) = x_(0,3);
    x_current(1) = x_(1,3);
    x_current(2) = x_(2,3);
    
    logging_file_1 << std::fixed << std::setprecision(6) 
                  << play_time_ << " "                 // 1. Time
                  
                  << x_desired_.transpose() << " "    // 2. x_desired (3개 원소)
                  
                  << x_current.transpose() << " "    // 3. x_current (3개 원소)

                  << q_desired_.transpose()           // 4. q_desired (7개 원소)
                  << std::endl;
}

void cRoboticsController::HW2_CLIK()
{
    Matrix4d EEPos_init_Matrix = getEEPose(q_init_);
    Vector3d EEPos_init;
    EEPos_init.setZero();
    EEPos_init(0) = EEPos_init_Matrix(0,3);
    EEPos_init(1) = EEPos_init_Matrix(1,3);
    EEPos_init(2) = EEPos_init_Matrix(2,3);

    Vector3d EEPos_target;
    EEPos_target.setZero();
    EEPos_target(0) = 0.25;
    EEPos_target(1) = 0.28;
    EEPos_target(2) = 0.65;

    Vector3d zero_vector;
	zero_vector.setZero();
    
	Vector3d x_desired_ = DyrosMath::cubicVector<3>(play_time_,
		                                   control_start_time_,
		                                   control_start_time_ + 2.0, 
                                           EEPos_init, 
                                           EEPos_target, 
                                           zero_vector, 
                                           zero_vector);

    Vector3d x_desired_dot_ = DyrosMath::cubicDotVector<3>(play_time_,
		                                   control_start_time_,
		                                   control_start_time_ + 2.0, 
                                           EEPos_init, 
                                           EEPos_target, 
                                           zero_vector, 
                                           zero_vector);

    MatrixXd EEJac = getEEJac(q_);
    MatrixXd CurrentEEJac(3, 7);
    CurrentEEJac = EEJac.topRows(3);
    
    Matrix4d EEPos_cur_Matrix = getEEPose(q_);
    Vector3d EEPos_cur;
    EEPos_cur.setZero();
    EEPos_cur(0) = EEPos_cur_Matrix(0,3);
    EEPos_cur(1) = EEPos_cur_Matrix(1,3);
    EEPos_cur(2) = EEPos_cur_Matrix(2,3);

    Matrix3d Gain;
    Gain.setZero();
    Gain(0,0) = 5;
    Gain(1,1) = 5;
    Gain(2,2) = 5;

    double lambda = 0.01; 
    MatrixXd J_JT = CurrentEEJac * CurrentEEJac.transpose(); 
    MatrixXd I_damped = J_JT + lambda * lambda * MatrixXd::Identity(CurrentEEJac.rows(), CurrentEEJac.rows());
    MatrixXd J_pinv = CurrentEEJac.transpose() * I_damped.inverse(); 

    Vector7d target_position;
    target_position = q_t + J_pinv * (x_desired_dot_ + Gain*(x_desired_ - EEPos_cur))* (play_time_ - q_t_time);
    q_t = target_position;
    q_t_time = play_time_;
    
    q_desired_ = target_position;

    Vector3d x_current;
    x_current.setZero();
    x_current(0) = x_(0,3);
    x_current(1) = x_(1,3);
    x_current(2) = x_(2,3);
    
    logging_file_2 << std::fixed << std::setprecision(6) 
                  << play_time_ << " "                 // 1. Time
                  
                  << x_desired_.transpose() << " "    // 2. x_desired (3개 원소)
                  
                  << x_current.transpose() << " "    // 3. x_current (3개 원소)

                  << q_desired_.transpose()           // 4. q_desired (7개 원소)
                  << std::endl;
}

void cRoboticsController::HW2_CLIK_WPI()
{
    Matrix4d EEPos_init_Matrix = getEEPose(q_init_);
    Vector3d EEPos_init;
    EEPos_init.setZero();
    EEPos_init(0) = EEPos_init_Matrix(0,3);
    EEPos_init(1) = EEPos_init_Matrix(1,3);
    EEPos_init(2) = EEPos_init_Matrix(2,3);

    Vector3d EEPos_target;
    EEPos_target.setZero();
    EEPos_target(0) = 0.25;
    EEPos_target(1) = 0.28;
    EEPos_target(2) = 0.65;

    Vector3d zero_vector;
	zero_vector.setZero();
    
	Vector3d x_desired_ = DyrosMath::cubicVector<3>(play_time_,
		                                   control_start_time_,
		                                   control_start_time_ + 2.0, 
                                           EEPos_init, 
                                           EEPos_target, 
                                           zero_vector, 
                                           zero_vector);
    
    Vector3d x_desired_dot_ = DyrosMath::cubicDotVector<3>(play_time_,
		                                   control_start_time_,
		                                   control_start_time_ + 2.0, 
                                           EEPos_init, 
                                           EEPos_target, 
                                           zero_vector, 
                                           zero_vector);

    MatrixXd EEJac = getEEJac(q_);
    MatrixXd CurrentEEJac(3, 7);
    CurrentEEJac = EEJac.topRows(3);

    Matrix4d EEPos_cur_Matrix = getEEPose(q_);
    Vector3d EEPos_cur;
    EEPos_cur.setZero();
    EEPos_cur(0) = EEPos_cur_Matrix(0,3);
    EEPos_cur(1) = EEPos_cur_Matrix(1,3);
    EEPos_cur(2) = EEPos_cur_Matrix(2,3);
    
    Matrix3d Gain;
    Gain.setZero();
    Gain(0,0) = 5;
    Gain(1,1) = 5;
    Gain(2,2) = 5;

    Matrix7d Weight;
    Weight.setZero();
    Weight(0,0) = 1;
    Weight(1,1) = 1;
    Weight(2,2) = 1;
    Weight(3,3) = 0.001;
    Weight(4,4) = 1;
    Weight(5,5) = 1;
    Weight(6,6) = 1;

    double lambda = 0.01; 
    MatrixXd J_JT = CurrentEEJac * Weight * CurrentEEJac.transpose(); 
    MatrixXd I_damped = J_JT + lambda * lambda * MatrixXd::Identity(CurrentEEJac.rows(), CurrentEEJac.rows());
    MatrixXd J_pinv = Weight * CurrentEEJac.transpose() * I_damped.inverse(); 

    Vector7d target_position;
    target_position = q_t + J_pinv * (x_desired_dot_ + Gain*(x_desired_ - EEPos_cur))* (play_time_ - q_t_time);
    q_t = target_position;
    q_t_time = play_time_;
    
    q_desired_ = target_position;

    Vector3d x_current;
    x_current.setZero();
    x_current(0) = x_(0,3);
    x_current(1) = x_(1,3);
    x_current(2) = x_(2,3);
    
    logging_file_3 << std::fixed << std::setprecision(6) 
                  << play_time_ << " "                 // 1. Time
                  
                  << x_desired_.transpose() << " "    // 2. x_desired (3개 원소)
                  
                  << x_current.transpose() << " "    // 3. x_current (3개 원소)

                  << q_desired_.transpose()           // 4. q_desired (7개 원소)
                  << std::endl;
}

void cRoboticsController::torqueCtrlDynamic()
{
    // logging_file_ << ... << std::endl;
    // torque_desired_ = 
}
// -------------------------------------
// =============================================================================

void cRoboticsController::setMode(const CTRL_MODE& control_mode)
{
    is_mode_changed_ = true;
    control_mode_ = control_mode;
    std::cout << "Control mode change: " << control_mode_ << std::endl;
}

void cRoboticsController::printState()
{
    // TODO 6: Extend or modify this for debugging your controller
    std::cout << "\n\n------------------------------------------------------------------" << std::endl;
    std::cout << "time     : " << std::fixed << std::setprecision(3) << play_time_ << std::endl;
    std::cout << "q now    :\t";
    std::cout << std::fixed << std::setprecision(3) << q_.transpose() << std::endl;
    std::cout << "q desired:\t";
    std::cout << std::fixed << std::setprecision(3) << q_desired_.transpose() << std::endl;
    std::cout << "x        :\n";
    std::cout << std::fixed << std::setprecision(3) << x_ << std::endl;
    std::cout << "x dot    :\t";
    std::cout << std::fixed << std::setprecision(3) << xdot_.transpose() << std::endl;
    std::cout << "J        :\n";
    std::cout << std::fixed << std::setprecision(3) << J_ << std::endl;
}

Matrix4d cRoboticsController::getLinkPose(const VectorXd& q, const std::string& link_name)
{
    if(q.size() != model_.nq)
    {
        std::cerr << "getEEPose Error: size of q " << q.size() << " is not equal to model.nq size: " << model_.nq << std::endl;
        return Matrix4d::Identity();
    }
    pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
    if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
    {
        std::cerr << "Error: Link name " << link_name << " not found in URDF." << std::endl;
        return Matrix4d::Identity();
    }

    pinocchio::Data data_tmp(model_);
    pinocchio::framesForwardKinematics(model_, data_tmp, q);
    return data_tmp.oMf[link_index].toHomogeneousMatrix();
}

MatrixXd cRoboticsController::getLinkJac(const VectorXd& q, const std::string& link_name)
{
    if(q.size() != model_.nq)
    {
        std::cerr << "getEEJac Error: size of q " << q.size() << " is not equal to model.nq size: " << model_.nq << std::endl;
        return MatrixXd::Zero(6, model_.nv);
    }
    pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
    if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
    {
        std::cerr << "Error: Link name " << link_name << " not found in URDF." << std::endl;
        return MatrixXd::Zero(6, model_.nv);
    }

    MatrixXd J;
    J.setZero(6, model_.nv);
    pinocchio::Data data_tmp(model_);
    pinocchio::computeJointJacobians(model_, data_tmp, q);
    pinocchio::getFrameJacobian(model_, data_tmp, link_index, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);

    return J;
}

Matrix4d cRoboticsController::getEEPose(const VectorXd& q)
{
    if(q.size() != model_.nq)
    {
        std::cerr << "getEEPose Error: size of q " << q.size() << " is not equal to model.nq size: " << model_.nq << std::endl;
        return Matrix4d::Identity();
    }
    pinocchio::FrameIndex ee_index = model_.getFrameId(ee_name_);
    if (ee_index == static_cast<pinocchio::FrameIndex>(-1))  
    {
        std::cerr << "Error: Link name " << ee_name_ << " not found in URDF." << std::endl;
        return Matrix4d::Identity();
    }

    pinocchio::Data data_tmp(model_);
    pinocchio::framesForwardKinematics(model_, data_tmp, q);
    return data_tmp.oMf[ee_index].toHomogeneousMatrix();
}

MatrixXd cRoboticsController::getEEJac(const VectorXd& q)
{
    if(q.size() != model_.nq)
    {
        std::cerr << "getEEJac Error: size of q " << q.size() << " is not equal to model.nq size: " << model_.nq << std::endl;
        return MatrixXd::Zero(6, model_.nv);
    }
    pinocchio::FrameIndex ee_index = model_.getFrameId(ee_name_);
    if (ee_index == static_cast<pinocchio::FrameIndex>(-1))  
    {
        std::cerr << "Error: Link name " << ee_name_ << " not found in URDF." << std::endl;
        return MatrixXd::Zero(6, model_.nv);
    }

    MatrixXd J;
    J.setZero(6, model_.nv);
    pinocchio::Data data_tmp(model_);
    pinocchio::computeJointJacobians(model_, data_tmp, q);
    pinocchio::getFrameJacobian(model_, data_tmp, ee_index, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);

    return J;
}

MatrixXd cRoboticsController::getMassMatrix(const VectorXd& q)
{
    if(q.size() != model_.nq)
    {
        std::cerr << "getMassMatrix Error: size of q " << q.size() << " is not equal to model.nq size: " << model_.nq << std::endl;
        return MatrixXd::Zero(model_.nq, model_.nq);
    }
    pinocchio::Data data_tmp(model_);
    pinocchio::crba(model_, data_tmp, q);

    return data_tmp.M.selfadjointView<Upper>();  // Only upper triangular part of M_ is computed by pinocchio::crba
}

VectorXd cRoboticsController::getGravityVector(const VectorXd& q)
{
    if(q.size() != model_.nq)
    {
        std::cerr << "getGravityVector Error: size of q " << q.size() << " is not equal to model.nq size: " << model_.nq << std::endl;
        return VectorXd::Zero(model_.nq);
    }
    pinocchio::Data data_tmp(model_);
    pinocchio::computeGeneralizedGravity(model_, data_tmp, q);

    return data_tmp.g;
}

bool cRoboticsController::updateModel(const VectorXd& q, const VectorXd& qdot, const VectorXd& tau)
{
    q_ = q;
    qdot_ = qdot;
    tau_ = tau;
    if(!updateKinematics(q_, qdot_)) return false;
    if(!updateDynamics(q_, qdot_)) return false;

    return true;
}

bool cRoboticsController::updateKinematics(const VectorXd& q, const VectorXd& qdot)
{
    if(q.size() != model_.nq)
    {
        std::cerr << "updateKinematics Error: size of q " << q.size() << " is not equal to model.nq size: " << model_.nq << std::endl;
        return false;
    }
    if(qdot.size() != model_.nv)
    {
        std::cerr << "updateKinematics Error: size of qdot " << qdot.size() << " is not equal to model.nv size: " << model_.nv << std::endl;
        return false;
    }
    pinocchio::FrameIndex ee_index = model_.getFrameId(ee_name_);
    if (ee_index == static_cast<pinocchio::FrameIndex>(-1))  
    {
        std::cerr << "Error: Link name " << ee_name_ << " not found in URDF." << std::endl;
        return false;
    }

    pinocchio::FrameIndex link4_index = model_.getFrameId(link_names_[4]);
    if (link4_index == static_cast<pinocchio::FrameIndex>(-1))  
    {
        std::cerr << "Error: Link name " << link_names_[4] << " not found in URDF." << std::endl;
        return false;
    }

    pinocchio::computeJointJacobians(model_, data_, q);

    pinocchio::getFrameJacobian(model_, data_, ee_index, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_);
    x_ = data_.oMf[ee_index].toHomogeneousMatrix();
    xdot_ = J_ * qdot;

    pinocchio::getFrameJacobian(model_, data_, link4_index, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J2_);
    x2_ = data_.oMf[link4_index].toHomogeneousMatrix();
    x2dot_ = J2_ * qdot;

    return true;
}

bool cRoboticsController::updateDynamics(const VectorXd& q, const VectorXd& qdot)
{
    if(q.size() != model_.nq)
    {
        std::cerr << "updateDynamics Error: size of q " << q.size() << " is not equal to model.nq size: " << model_.nq << std::endl;
        return false;
    }
    if(qdot.size() != model_.nv)
    {
        std::cerr << "updateDynamics Error: size of qdot " << qdot.size() << " is not equal to model.nv size: " << model_.nv << std::endl;
        return false;
    }
    pinocchio::crba(model_, data_, q);
    pinocchio::computeGeneralizedGravity(model_, data_, q);
    pinocchio::computeCoriolisMatrix(model_, data_, q, qdot);

    // update joint space dynamics
    M_ = data_.M;
    M_ = M_.selfadjointView<Upper>();  // Only upper triangular part of M_ is computed by pinocchio::crba
    g_ = data_.g;
    c_ = data_.C * qdot_;

    return true;
}

VectorXd cRoboticsController::getCtrlInput()
{
    if(manipulator_control_mode_ == "position")
    {
        return q_desired_;
    }
    else
    {
        return tau_desired_;
    }
}