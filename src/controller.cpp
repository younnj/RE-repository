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

    logging_file_1.open("PD_Step.txt");
    logging_file_2.open("PDwGravityComp_Step.txt");
    logging_file_3.open("PDwGravityComp_Spline.txt");
    logging_file_4.open("DynamicComp_Step.txt");
    logging_file_5.open("DynamicComp_Spline.txt");
    //logging_file_6.open("log_tasktrans.txt");
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
       setMode(hw4_GravityComp);
        break;
    case '4':
       setMode(hw4_PD);
        break;
    case '5':
       setMode(hw4_PDwGravityComp_Step);
        break;
    case '6':
       setMode(hw4_PDwGravityComp_Spline);
        break;
    case '7':
       setMode(hw4_DynamicComp_Step);
        break;
    case '8':
       setMode(hw4_DynamicComp_Spline);
        break;
    case '9':
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
            target_position << 0.0, 0.0, 0.0, -M_PI/6, 0, M_PI / 2, 0;
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
        case hw4_GravityComp:
        {
            HW4_GravityComp();
            break;
        }
        case hw4_PD:
        {
            HW4_PD();
            break;
        }
        case hw4_PDwGravityComp_Step:
        {
            HW4_PDwGravityComp_Step();
            break;
        }
        case hw4_PDwGravityComp_Spline:
        {
            HW4_PDwGravityComp_Spline();
            break;
        }
        case hw4_DynamicComp_Step:
        {
            HW4_DynamicComp_Step();
            break;
        }
        case hw4_DynamicComp_Spline:
        {
            HW4_DynamicComp_Spline();
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
void cRoboticsController::HW4_GravityComp()
{
    tau_desired_ = getGravityVector(q_);
}

void cRoboticsController::HW4_PD()
{
    VectorXd p_gains(7);
    p_gains << 300.0, 250.0, 250.0, 250.0, 250.0, 250.0, 150.0;
    VectorXd v_gains(7);
    v_gains << 100.0, 80.0, 80.0, 80.0, 80.0, 80.0, 60.0;
    
    Matrix7d Kp = p_gains.asDiagonal();
    Matrix7d Kv = v_gains.asDiagonal();

    VectorXd qd(7);
    qd << 0.0, 0.0, 0.0, -M_PI*5/36, 0, M_PI / 2, 0;

    tau_desired_ = Kp * (qd - q_) - Kv * qdot_;
    
    logging_file_1 << std::fixed << std::setprecision(6) 
                  << play_time_ << " "                 
                  
                  << qd(3) << " "    
                  
                  << q_(3)

                  << std::endl;
}

void cRoboticsController::HW4_PDwGravityComp_Step()
{
    VectorXd p_gains(7);
    p_gains << 300.0, 250.0, 250.0, 250.0, 250.0, 250.0, 150.0;
    VectorXd v_gains(7);
    v_gains << 100.0, 80.0, 80.0, 80.0, 80.0, 80.0, 60.0;
    
    Matrix7d Kp = p_gains.asDiagonal();
    Matrix7d Kv = v_gains.asDiagonal();

    VectorXd qd(7);
    qd << 0.0, 0.0, 0.0, -M_PI*5/36, 0, M_PI / 2, 0;

    tau_desired_ = Kp * (qd - q_) - Kv * qdot_ + getGravityVector(q_);
    
    
    logging_file_2 << std::fixed << std::setprecision(6) 
                  << play_time_ << " "                 // 1. Time
                  
                  << qd(3) << " "    
                  
                  << q_(3)

                  << std::endl;
}

void cRoboticsController::HW4_PDwGravityComp_Spline()
{
    VectorXd p_gains(7);
    p_gains << 300.0, 250.0, 250.0, 250.0, 250.0, 250.0, 150.0;
    VectorXd v_gains(7);
    v_gains << 100.0, 80.0, 80.0, 80.0, 80.0, 80.0, 60.0;
    
    Matrix7d Kp = p_gains.asDiagonal();
    Matrix7d Kv = v_gains.asDiagonal();

    VectorXd qd_spline_init(7);
    VectorXd qd_spline_target(7);
    qd_spline_init << 0.0, 0.0, 0.0, -M_PI/6, 0, M_PI / 2, 0;
    qd_spline_target << 0.0, 0.0, 0.0, -M_PI/3, 0, M_PI / 2, 0;

    VectorXd zero_vector(7);
    zero_vector.setZero();

    Vector7d qd_spline = DyrosMath::cubicVector<7>(play_time_,
		                                   control_start_time_,
		                                   control_start_time_ + 2.0, 
                                           qd_spline_init, 
                                           qd_spline_target,
                                           zero_vector, 
                                           zero_vector);

    tau_desired_ = Kp * (qd_spline - q_) - Kv * qdot_ + getGravityVector(q_);
    
    
    logging_file_3 << std::fixed << std::setprecision(6) 
                  << play_time_ << " "                 // 1. Time
                  
                  << qd_spline(3) << " "    
                  
                  << q_(3)

                  << std::endl;
}

void cRoboticsController::HW4_DynamicComp_Step()
{
    VectorXd p_gains(7);
    p_gains << 300.0, 250.0, 250.0, 250.0, 250.0, 250.0, 150.0;
    VectorXd v_gains(7);
    v_gains << 100.0, 80.0, 80.0, 80.0, 80.0, 80.0, 60.0;
    
    Matrix7d Kp = p_gains.asDiagonal();
    Matrix7d Kv = v_gains.asDiagonal();

    VectorXd qd(7);
    qd << 0.0, 0.0, 0.0, -M_PI*5/36, 0, M_PI / 2, 0;

    tau_desired_ = getMassMatrix(q_) * (Kp * (qd - q_) - Kv * qdot_) + getGravityVector(q_);
    
    logging_file_4 << std::fixed << std::setprecision(6) 
                  << play_time_ << " "                 // 1. Time
                  
                  << qd(3) << " "    
                  
                  << q_(3)

                  << std::endl;
}

void cRoboticsController::HW4_DynamicComp_Spline()
{
    VectorXd p_gains(7);
    p_gains << 300.0, 250.0, 250.0, 250.0, 250.0, 250.0, 150.0;
    VectorXd v_gains(7);
    v_gains << 50.0, 20.0, 20.0, 20.0, 20.0, 20.0, 15.0;
    
    Matrix7d Kp = p_gains.asDiagonal();
    Matrix7d Kv = v_gains.asDiagonal();

    VectorXd qd_spline_init(7);
    VectorXd qd_spline_target(7);
    qd_spline_init << 0.0, 0.0, 0.0, -M_PI/6, 0, M_PI / 2, 0;
    qd_spline_target << 0.0, 0.0, 0.0, -M_PI/3, 0, M_PI / 2, 0;

    VectorXd zero_vector(7);
    zero_vector.setZero();

    Vector7d qd_spline = DyrosMath::cubicVector<7>(play_time_,
		                                   control_start_time_,
		                                   control_start_time_ + 2.0, 
                                           qd_spline_init, 
                                           qd_spline_target,
                                           zero_vector, 
                                           zero_vector);

    tau_desired_ = getMassMatrix(q_) * (Kp * (qd_spline - q_) - Kv * qdot_) + getGravityVector(q_);
    
    logging_file_5 << std::fixed << std::setprecision(6) 
                  << play_time_ << " "                 // 1. Time
                  
                  << qd_spline(3) << " "    
                  
                  << q_(3)
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
    std::cout << "Link4x        :\n";
    std::cout << std::fixed << std::setprecision(3) << getLinkPose(q_, "fr3_link4") << std::endl;
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