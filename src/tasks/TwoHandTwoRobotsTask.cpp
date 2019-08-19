/*
 * TwoHandTwoRobotsTask.cpp
 *
 *      Author: Mikael Jorda
 */

#include "TwoHandTwoRobotsTask.h"

#include <stdexcept>


namespace Sai2Primitives
{


TwoHandTwoRobotsTask::TwoHandTwoRobotsTask(Sai2Model::Sai2Model* robot_arm_1,
						 Sai2Model::Sai2Model* robot_arm_2, 
		            const std::string link_name_1, 
					const std::string link_name_2, 
		            const Eigen::Affine3d control_frame_1,
		            const Eigen::Affine3d control_frame_2,
		            const double loop_time) :
	TwoHandTwoRobotsTask(robot_arm_1, robot_arm_2, link_name_1, link_name_2, 
		control_frame_1.translation(), control_frame_2.translation(),
		control_frame_1.linear(), control_frame_2.linear(),
		loop_time) {}

TwoHandTwoRobotsTask::TwoHandTwoRobotsTask(Sai2Model::Sai2Model* robot_arm_1,
						 Sai2Model::Sai2Model* robot_arm_2, 
		            const std::string link_name_1, 
		            const std::string link_name_2, 
		            const Eigen::Vector3d pos_in_link_1, 
		            const Eigen::Vector3d pos_in_link_2, 
		            const Eigen::Matrix3d rot_in_link_1,
		            const Eigen::Matrix3d rot_in_link_2,
		            const double loop_time)
{

	Eigen::Affine3d control_frame_1 = Eigen::Affine3d::Identity();
	Eigen::Affine3d control_frame_2 = Eigen::Affine3d::Identity();
	control_frame_1.linear() = rot_in_link_1;
	control_frame_1.translation() = pos_in_link_1;
	control_frame_2.linear() = rot_in_link_2;
	control_frame_2.translation() = pos_in_link_2;

	_robot_arm_1 = robot_arm_1;
	_robot_arm_2 = robot_arm_2;
	_link_name_1 = link_name_1;
	_link_name_2 = link_name_2;
	_control_frame_1 = control_frame_1;
	_control_frame_2 = control_frame_2;

	int dof_1 = _robot_arm_1->dof();
	int dof_2 = _robot_arm_2->dof();

	_N_prec_1.setIdentity(dof_1, dof_1);
	_N_prec_2.setIdentity(dof_2, dof_2);


	if(dof_1 < 6 || dof_2 < 6)
	{
		throw std::invalid_argument("robots must have at least 6 DoF each for a TwoHandTwoRobotsTask\n");
	}

	_grasp_matrix.setZero(12,12);
	_R_grasp_matrix.setIdentity();

	Eigen::Vector3d robot_1_contact = Eigen::Vector3d::Zero();
	Eigen::Vector3d robot_2_contact = Eigen::Vector3d::Zero();
	_robot_arm_1->positionInWorld(robot_1_contact, _link_name_1, _control_frame_1.translation());
	_robot_arm_2->positionInWorld(robot_2_contact, _link_name_2, _control_frame_2.translation());

	_contact_locations.push_back(robot_1_contact);
	_contact_locations.push_back(robot_2_contact);
	_contact_constrained_rotations.push_back(3);
	_contact_constrained_rotations.push_back(3);

	// object position and orientation
	_current_object_position = (_contact_locations[0] + _contact_locations[1]) / 2.0;
	_current_object_orientation.setIdentity();

	//gains
	_kp_pos = 100.0;
	_kv_pos = 20.0;
	_ki_pos = 0.0;
	_kp_ori = 100.0;
	_kv_ori = 20.0;
	_ki_ori = 0.0;

	_kp_force = 1.0;
	_ki_force = 1.7;
	_kv_force = 10.0;
	_kp_moment = 1.0;
	_ki_moment = 1.7;
	_kv_moment = 10.0;

	_object_gravity.setZero(6);

	// velocity saturation
	_use_velocity_saturation_flag = false;
	_linear_saturation_velocity = 0.3;
	_angular_saturation_velocity = M_PI/3;

#ifdef USING_OTG
	_use_interpolation_pos_flag = true;
	_use_interpolation_ori_flag = true;

	_loop_time = loop_time;
	_otg_pos = new OTG(_current_object_position, _loop_time);
	_otg_ori = new OTG_ori(_current_object_orientation, _loop_time);

	_otg_pos->setMaxVelocity(0.3);
	_otg_pos->setMaxAcceleration(0.6);
	_otg_pos->setMaxJerk(1.2);

	_otg_ori->setMaxVelocity(M_PI/4);
	_otg_ori->setMaxAcceleration(M_PI/2);
	_otg_ori->setMaxJerk(M_PI);

#endif

	reInitializeTask();
}


void TwoHandTwoRobotsTask::updateTaskModel(const Eigen::MatrixXd N_prec_1, const Eigen::MatrixXd N_prec_2)
{
	if(N_prec_1.rows() != N_prec_1.cols())
	{
		throw std::invalid_argument("N_prec_1 matrix not square in TwoHandTwoRobotsTask::updateTaskModel\n");
	}
	if(N_prec_1.rows() != _robot_arm_1->_dof)
	{
		throw std::invalid_argument("N_prec_1 matrix size not consistent with robot dof in TwoHandTwoRobotsTask::updateTaskModel\n");
	}
	if(N_prec_2.rows() != N_prec_2.cols())
	{
		throw std::invalid_argument("N_prec_2 matrix not square in TwoHandTwoRobotsTask::updateTaskModel\n");
	}
	if(N_prec_2.rows() != _robot_arm_2->_dof)
	{
		throw std::invalid_argument("N_prec_2 matrix size not consistent with robot dof in TwoHandTwoRobotsTask::updateTaskModel\n");
	}

	_N_prec_1 = N_prec_1;
	_N_prec_2 = N_prec_2;

	Eigen::MatrixXd augmented_R1 = Eigen::MatrixXd::Zero(6,6);
	Eigen::MatrixXd augmented_R2 = Eigen::MatrixXd::Zero(6,6);
	augmented_R1.block<3,3>(0,0) = _robot_arm_1->_T_world_robot.linear();
	augmented_R1.block<3,3>(3,3) = _robot_arm_1->_T_world_robot.linear();
	augmented_R2.block<3,3>(0,0) = _robot_arm_2->_T_world_robot.linear();
	augmented_R2.block<3,3>(3,3) = _robot_arm_2->_T_world_robot.linear();

	Eigen::Affine3d T_world_controllinkrobot1 = Eigen::Affine3d::Identity();
	Eigen::Affine3d T_world_controllinkrobot2 = Eigen::Affine3d::Identity();
	_robot_arm_1->transformInWorld(T_world_controllinkrobot1, _link_name_1);
	_robot_arm_2->transformInWorld(T_world_controllinkrobot2, _link_name_2);

	_T_hand1_object = T_world_controllinkrobot1.inverse() * _T_world_object;
	_T_hand2_object = T_world_controllinkrobot2.inverse() * _T_world_object;

	Eigen::MatrixXd J1_at_object_center;
	_robot_arm_1->J_0(J1_at_object_center, _link_name_1, _T_hand1_object.translation());
	J1_at_object_center = augmented_R1 * J1_at_object_center;
	_robot_arm_1->taskInertiaMatrix(_robot_1_effective_inertia, J1_at_object_center);
	Eigen::MatrixXd J2_at_object_center;
	_robot_arm_2->J_0(J2_at_object_center, _link_name_2, _T_hand2_object.translation());
	J2_at_object_center = augmented_R2 * J2_at_object_center;
	_robot_arm_2->taskInertiaMatrix(_robot_2_effective_inertia, J2_at_object_center);

	_object_effective_inertia.block<3,3>(3,3) = _current_object_orientation * _object_inertia_in_control_frame * _current_object_orientation.transpose();

	_Lambda_tot = _object_effective_inertia + _robot_1_effective_inertia + _robot_2_effective_inertia;

	_robot_arm_1->J_0(_jacobian_1, _link_name_1, _control_frame_1.translation());
	_projected_jacobian_1 = augmented_R1 * _jacobian_1 * _N_prec_1;
	_robot_arm_1->nullspaceMatrix(_N_1, _projected_jacobian_1, _N_prec_1);
	_robot_arm_2->J_0(_jacobian_2, _link_name_2, _control_frame_2.translation());
	_projected_jacobian_2 = augmented_R2 * _jacobian_2 * _N_prec_2;
	_robot_arm_2->nullspaceMatrix(_N_2, _projected_jacobian_2, _N_prec_2);
}


void TwoHandTwoRobotsTask::computeTorques(Eigen::VectorXd& task_joint_torques_1, Eigen::VectorXd& task_joint_torques_2)
{

	// get time since last call for the I term
	_t_curr = std::chrono::high_resolution_clock::now();
	if(_first_iteration)
	{
		_t_prev = std::chrono::high_resolution_clock::now();
		_first_iteration = false;
	}
	_t_diff = _t_curr - _t_prev;

	Eigen::Vector3d position_related_force = Eigen::Vector3d::Zero();
	Eigen::Vector3d orientation_related_force = Eigen::Vector3d::Zero();
	Eigen::Vector3d force_related_force = Eigen::Vector3d::Zero();
	Eigen::Vector3d moment_related_force = Eigen::Vector3d::Zero();

	// update controller state
	_robot_arm_1->positionInWorld(_contact_locations[0], _link_name_1, _control_frame_1.translation());
	_robot_arm_2->positionInWorld(_contact_locations[1], _link_name_2, _control_frame_2.translation());

	Eigen::Matrix3d rot_robot1 = Eigen::Matrix3d::Identity();
	Eigen::Matrix3d rot_robot2 = Eigen::Matrix3d::Identity();

	_robot_arm_1->rotationInWorld(rot_robot1, _link_name_1);
	_robot_arm_2->rotationInWorld(rot_robot2, _link_name_2);

	// compute object frame and grasp matrix
	Eigen::Vector3d position_increment_r1 = _contact_locations[0] - _previous_position_r1;
	Eigen::Vector3d position_increment_r2 = _contact_locations[1] - _previous_position_r2;

	Eigen::Vector3d object_position_increment = (position_increment_r1 + position_increment_r2) / 2.0;
	_current_object_position += object_position_increment;

	Eigen::Matrix3d orientation_increment_r1 = rot_robot1 * _previous_orientation_r1.transpose();
	Eigen::Matrix3d orientation_increment_r2 = rot_robot2 * _previous_orientation_r2.transpose();

	Eigen::Matrix3d added_orientation_increment = orientation_increment_r1 * orientation_increment_r2; // small rotations so we assume they commute
	Eigen::AngleAxisd ori_increment_aa = Eigen::AngleAxisd(added_orientation_increment);
	Eigen::Matrix3d object_orientation_increment = Eigen::AngleAxisd(ori_increment_aa.angle()/2.0, ori_increment_aa.axis()).toRotationMatrix();

	_current_object_orientation = object_orientation_increment * _current_object_orientation;

	Sai2Model::graspMatrix(_grasp_matrix, _R_grasp_matrix, _current_object_position,
				_contact_locations, _contact_constrained_rotations);

	// // compute grasp matrix and object frame
	// Sai2Model::graspMatrixAtGeometricCenter(_grasp_matrix, _R_grasp_matrix, _current_object_position,
	// 			_contact_locations, _contact_constrained_rotations);

	// _x_object_frame = _contact_locations[0] - _contact_locations[1];
	// _x_object_frame.normalize();

	// Eigen::Matrix3d projector_orthogonal_nullspace_x_object = Eigen::Matrix3d::Identity() - _x_object_frame * _x_object_frame.transpose();
	// Eigen::Vector3d projected_arbitrary_direction_hand1_in_world = projector_orthogonal_nullspace_x_object * rot_robot1 * _arbitrary_direction_hand1;
	// Eigen::Vector3d projected_arbitrary_direction_hand2_in_world = projector_orthogonal_nullspace_x_object * rot_robot2 * _arbitrary_direction_hand2;

	// if(projected_arbitrary_direction_hand1_in_world.cross(projected_arbitrary_direction_hand2_in_world).norm() < 1e-2)
	// {
	// 	_y_object_frame = projected_arbitrary_direction_hand1_in_world.normalized();
	// }
	// else
	// {
	// 	double angle = acos(projected_arbitrary_direction_hand1_in_world.dot(projected_arbitrary_direction_hand2_in_world));
	// 	if (projected_arbitrary_direction_hand1_in_world.cross(projected_arbitrary_direction_hand2_in_world).dot(_x_object_frame) < 0)
	// 	{
	// 		angle = -angle;
	// 	}
	// 	Eigen::Matrix3d half_rotation = Eigen::AngleAxisd(angle/2.0, _x_object_frame).toRotationMatrix(); 
	// 	_y_object_frame = half_rotation * projected_arbitrary_direction_hand1_in_world;
	// 	_y_object_frame.normalize();
	// }

	// _z_object_frame = _x_object_frame.cross(_y_object_frame);
	// _z_object_frame.normalize();

	// _current_object_orientation.col(0) = _x_object_frame;
	// _current_object_orientation.col(1) = _y_object_frame;
	// _current_object_orientation.col(2) = _z_object_frame;

	_T_world_object.translation() = _current_object_position;
	_T_world_object.linear() = _current_object_orientation;

	// compute object velocities from robot velocities and grasp matrix
	Eigen::VectorXd r1_velocity = _projected_jacobian_1 * _robot_arm_1->_dq;
	Eigen::VectorXd r2_velocity = _projected_jacobian_2 * _robot_arm_2->_dq;
	Eigen::VectorXd contact_velocities = Eigen::VectorXd::Zero(12);
	contact_velocities << r1_velocity.head(3), r2_velocity.head(3), r1_velocity.tail(3), r2_velocity.tail(3);
	Eigen::VectorXd object_full_velocities = _grasp_matrix.transpose().colPivHouseholderQr().solve(contact_velocities);
	_current_object_velocity = object_full_velocities.segment<3>(0);
	_current_object_angular_velocity = object_full_velocities.segment<3>(3);

	Sai2Model::orientationError(_object_orientation_error, _desired_object_orientation, _current_object_orientation);

	_step_desired_object_position = _desired_object_position;
	_step_desired_object_velocity = _desired_object_velocity;
	_step_desired_object_orientation = _desired_object_orientation;
	_step_object_orientation_error = _object_orientation_error;
	_step_desired_object_angular_velocity = _desired_object_angular_velocity;

	// force related terms
	if(_closed_loop_force_control)
	{
		// update the integrated error
		_integrated_object_force_error += (_object_sensed_force - _object_desired_force) * _t_diff.count();

		// compute the feedback term
		Eigen::Vector3d force_feedback_term = - _kp_force * (_object_sensed_force - _object_desired_force) - _ki_force * _integrated_object_force_error - _kv_force * _current_object_velocity;

		// compute the final contribution
		force_related_force = _sigma_force * (_object_desired_force + force_feedback_term);
	}
	else
	{
		force_related_force = _sigma_force * _object_desired_force;
	}

	// moment related terms
	if(_closed_loop_moment_control)
	{
		// update the integrated error
		_integrated_object_moment_error += (_object_sensed_moment - _object_desired_moment) * _t_diff.count();

		// compute the feedback term
		Eigen::Vector3d moment_feedback_term = - _kp_moment * (_object_sensed_moment - _object_desired_moment) - _ki_moment * _integrated_object_moment_error - _kv_moment * _current_object_angular_velocity;

		// compute the final contribution
		moment_related_force = _sigma_moment * (_object_desired_moment + moment_feedback_term);
	}
	else
	{
		moment_related_force = _sigma_moment * _object_desired_moment;
	}

	// linear motion related terms
	// compute next state from trajectory generation
#ifdef USING_OTG
	if(_use_interpolation_pos_flag)
	{
		_otg_pos->setGoalPositionAndVelocity(_desired_object_position, _desired_object_velocity);
		_otg_pos->computeNextState(_step_desired_object_position, _step_desired_object_velocity);
	}
#endif
	
	// update integrated error for I term
	_integrated_object_position_error += (_current_object_position - _step_desired_object_position) * _t_diff.count();

	// final contribution
	if(_use_velocity_saturation_flag)
	{
		_step_desired_object_velocity = -_kp_pos / _kv_pos * (_current_object_position - _step_desired_object_position) - _ki_pos / _kv_pos * _integrated_object_position_error;
		if(_step_desired_object_velocity.norm() > _linear_saturation_velocity)
		{
			_step_desired_object_velocity *= _linear_saturation_velocity/_step_desired_object_velocity.norm();
		}
		position_related_force = -_kv_pos*(_current_object_velocity - _step_desired_object_velocity);
	}
	else
	{
		position_related_force = _sigma_position * ( -_kp_pos*(_current_object_position - _step_desired_object_position) - 
			_kv_pos*(_current_object_velocity - _step_desired_object_velocity ) - _ki_pos * _integrated_object_position_error);
	}


	// angular motion related terms
	// compute next state from trajectory generation
#ifdef USING_OTG
	if(_use_interpolation_ori_flag)
	{
		_otg_ori->setGoalPositionAndVelocity(_desired_object_orientation, _current_object_orientation, _desired_object_angular_velocity);
		_otg_ori->computeNextState(_step_desired_object_orientation, _step_desired_object_angular_velocity);
		Sai2Model::orientationError(_step_object_orientation_error, _step_desired_object_orientation, _current_object_orientation);
	}
#endif

	// update integrated error for I term
	_integrated_object_orientation_error += _step_object_orientation_error * _t_diff.count();

	// final contribution
	if(_use_velocity_saturation_flag)
	{
		_step_desired_object_angular_velocity = -_kp_ori / _kv_ori * _step_object_orientation_error - _ki_ori / _kv_ori * _integrated_object_position_error;
		if(_step_desired_object_angular_velocity.norm() > _angular_saturation_velocity)
		{
			_step_desired_object_angular_velocity *= _angular_saturation_velocity / _step_desired_object_angular_velocity.norm();
		}
		orientation_related_force = -_kv_ori*(_current_object_angular_velocity - _step_desired_object_angular_velocity);
	}
	else
	{
		orientation_related_force = _sigma_orientation * ( -_kp_ori*_step_object_orientation_error - _kv_ori*(_current_object_angular_velocity - _step_desired_object_angular_velocity) - 
			_ki_ori*_integrated_object_orientation_error);
	}

	// object gravity compensation
	Eigen::Affine3d T_controlpoint_com = _T_com_controlpoint.inverse();
	Eigen::Vector3d p_controlpoint_com_world_frame = _current_object_orientation*T_controlpoint_com.translation();

	Eigen::Vector3d object_linear_gravity = Eigen::Vector3d::Zero(3);
	object_linear_gravity(2) = -9.81 * _object_mass;

	_object_gravity.head(3) = object_linear_gravity;
	_object_gravity.tail(3) = p_controlpoint_com_world_frame.cross(object_linear_gravity);
	// object_gravity << 0, 0, _object_mass * 9.81, 0, 0, 0;

	// internal forces contribution
	Eigen::VectorXd internal_task_force = Eigen::VectorXd::Zero(6);
	internal_task_force << _desired_internal_tension, _desired_internal_moments;
	internal_task_force = internal_task_force - 10.0 * object_full_velocities.tail(6);

	// compute task force
	Eigen::VectorXd position_orientation_contribution(6);
	position_orientation_contribution.head(3) = position_related_force;
	position_orientation_contribution.tail(3) = orientation_related_force;

	Eigen::VectorXd force_moment_contribution(6);
	force_moment_contribution.head(3) = force_related_force;
	force_moment_contribution.tail(3) = moment_related_force;

	Eigen::VectorXd object_task_force = _Lambda_tot * position_orientation_contribution + force_moment_contribution - _object_gravity;
	// Eigen::VectorXd object_task_force = position_orientation_contribution + object_gravity;

	_task_force << object_task_force, internal_task_force;

	// compute task torques
	Eigen::VectorXd robots_task_forces = _grasp_matrix.colPivHouseholderQr().solve(_task_force);
	Eigen::VectorXd robot_1_task_force = Eigen::VectorXd::Zero(6);
	Eigen::VectorXd robot_2_task_force = Eigen::VectorXd::Zero(6);
	robot_1_task_force << robots_task_forces.segment<3>(0), robots_task_forces.segment<3>(6);
	robot_2_task_force << robots_task_forces.segment<3>(3), robots_task_forces.segment<3>(9);

	task_joint_torques_1 = _projected_jacobian_1.transpose()*robot_1_task_force;
	task_joint_torques_2 = _projected_jacobian_2.transpose()*robot_2_task_force;

	// update previous time and robot positions and orientations
	_t_prev = _t_curr;

	_previous_position_r1 = _contact_locations[0];
	_previous_position_r2 = _contact_locations[1];
	_previous_orientation_r1 = rot_robot1;
	_previous_orientation_r2 = rot_robot2;
}

void TwoHandTwoRobotsTask::reInitializeTask()
{
	int dof_1 = _robot_arm_1->dof();
	int dof_2 = _robot_arm_2->dof();

	// contact locations for grasp matrix computation
	_robot_arm_1->positionInWorld(_contact_locations[0], _link_name_1, _control_frame_1.translation());
	_robot_arm_2->positionInWorld(_contact_locations[1], _link_name_2, _control_frame_2.translation());

	// Pose of robots hands
	Eigen::Affine3d T_world_hand1 = Eigen::Affine3d::Identity();
	Eigen::Affine3d T_world_hand2 = Eigen::Affine3d::Identity();
	_robot_arm_1->transformInWorld(T_world_hand1, _link_name_1);
	_robot_arm_2->transformInWorld(T_world_hand2, _link_name_2);

	// grasp matrix and object position
	Eigen::Matrix3d rot_robot1 = Eigen::Matrix3d::Identity();
	Eigen::Matrix3d rot_robot2 = Eigen::Matrix3d::Identity();
	_robot_arm_1->rotationInWorld(rot_robot1, _link_name_1);
	_robot_arm_2->rotationInWorld(rot_robot2, _link_name_2);

	_previous_position_r1 = _contact_locations[0];
	_previous_position_r2 = _contact_locations[1];
	_previous_orientation_r1 = rot_robot1;
	_previous_orientation_r2 = rot_robot2;

	// _current_object_position.setZero();
	_current_object_position = (_contact_locations[0] + _contact_locations[1]) / 2.0;
	_current_object_orientation.setIdentity();

	// object inertial properties
	_object_mass = 0;
	_object_inertia_in_control_frame = Eigen::Matrix3d::Zero();
	_object_effective_inertia.setZero(6,6);
	_T_com_controlpoint = Eigen::Affine3d::Identity();


	Sai2Model::graspMatrix(_grasp_matrix, _R_grasp_matrix, _current_object_position,
				_contact_locations, _contact_constrained_rotations);

	_desired_object_position = _current_object_position;
	_desired_object_orientation = _current_object_orientation;

	// // object orientation
	// _x_object_frame = robot_1_contact - robot_2_contact;
	// _x_object_frame.normalize();

	// if(_x_object_frame.dot(Eigen::Vector3d::UnitZ()) < 0.1)
	// {
	// 	_arbitrary_direction_hand1 = T_world_hand1.linear().transpose() * Eigen::Vector3d::UnitZ();
	// 	_arbitrary_direction_hand2 = T_world_hand2.linear().transpose() * Eigen::Vector3d::UnitZ();
	// }
	// else
	// {
	// 	_arbitrary_direction_hand1 = T_world_hand1.linear().transpose() * Eigen::Vector3d::UnitX();
	// 	_arbitrary_direction_hand2 = T_world_hand2.linear().transpose() * Eigen::Vector3d::UnitX();
	// }

	// Eigen::Matrix3d projector_orthogonal_nullspace_x_object = Eigen::Matrix3d::Identity() - _x_object_frame * _x_object_frame.transpose();
	// Eigen::Vector3d projected_arbitrary_direction_hand1_in_world = projector_orthogonal_nullspace_x_object * T_world_hand1.linear() * _arbitrary_direction_hand1;
	// Eigen::Vector3d projected_arbitrary_direction_hand2_in_world = projector_orthogonal_nullspace_x_object * T_world_hand2.linear() * _arbitrary_direction_hand2;

	// if(projected_arbitrary_direction_hand1_in_world.cross(projected_arbitrary_direction_hand2_in_world).norm() < 1e-2)
	// {
	// 	_y_object_frame = projected_arbitrary_direction_hand1_in_world.normalized();
	// }
	// else
	// {
	// 	double angle = acos(projected_arbitrary_direction_hand1_in_world.dot(projected_arbitrary_direction_hand2_in_world));
	// 	if (projected_arbitrary_direction_hand1_in_world.cross(projected_arbitrary_direction_hand2_in_world).dot(_x_object_frame) < 0)
	// 	{
	// 		angle = -angle;
	// 	}
	// 	Eigen::Matrix3d half_rotation = Eigen::AngleAxisd(angle/2.0, _x_object_frame).toRotationMatrix(); 
	// 	_y_object_frame = half_rotation * projected_arbitrary_direction_hand1_in_world;
	// 	_y_object_frame.normalize();
	// }

	// _z_object_frame = _x_object_frame.cross(_y_object_frame);
	// _z_object_frame.normalize();

	// _current_object_orientation.col(0) = _x_object_frame;
	// _current_object_orientation.col(1) = _y_object_frame;
	// _current_object_orientation.col(2) = _z_object_frame;
	// _desired_object_orientation = _current_object_orientation;

	// update object pose in world
	_T_world_object.translation() = _current_object_position;
	_T_world_object.linear() = _current_object_orientation;

	// compute inertial properties
	_T_hand1_object = T_world_hand1.inverse() * _T_world_object;
	_T_hand2_object = T_world_hand2.inverse() * _T_world_object;

	// _object_effective_inertia.setIdentity(6,6);

	Eigen::MatrixXd augmented_R1 = Eigen::MatrixXd::Zero(6,6);
	Eigen::MatrixXd augmented_R2 = Eigen::MatrixXd::Zero(6,6);
	augmented_R1.block<3,3>(0,0) = _robot_arm_1->_T_world_robot.linear();
	augmented_R1.block<3,3>(3,3) = _robot_arm_1->_T_world_robot.linear();
	augmented_R2.block<3,3>(0,0) = _robot_arm_2->_T_world_robot.linear();
	augmented_R2.block<3,3>(3,3) = _robot_arm_2->_T_world_robot.linear();

	_robot_1_effective_inertia.setZero(6,6);
	Eigen::MatrixXd J1_at_object_center;
	_robot_arm_1->J_0(J1_at_object_center, _link_name_1, _T_hand1_object.translation());
	J1_at_object_center = augmented_R1 * J1_at_object_center;
	_robot_arm_1->taskInertiaMatrix(_robot_1_effective_inertia, J1_at_object_center);

	_robot_2_effective_inertia.setZero(6,6);
	Eigen::MatrixXd J2_at_object_center;
	_robot_arm_2->J_0(J2_at_object_center, _link_name_2, _T_hand2_object.translation());
	J2_at_object_center = augmented_R2 * J2_at_object_center;
	_robot_arm_2->taskInertiaMatrix(_robot_2_effective_inertia, J2_at_object_center);

	_Lambda_tot = _object_effective_inertia + _robot_1_effective_inertia + _robot_2_effective_inertia;

	// nullspace matrices
	_jacobian_1.setZero(6, dof_1);
	_projected_jacobian_1.setZero(6, dof_1);
	_N_1.setZero(dof_1, dof_1);
	_jacobian_2.setZero(6, dof_2);
	_projected_jacobian_2.setZero(6, dof_2);
	_N_2.setZero(dof_2, dof_2);

	// velocities
	_current_object_velocity.setZero();        
	_desired_object_velocity.setZero();        
	_current_object_angular_velocity.setZero();
	_desired_object_angular_velocity.setZero();

	// orientation error and I terms
	_object_orientation_error.setZero();
	_integrated_object_orientation_error.setZero();
	_integrated_object_position_error.setZero();

	// object force quantities
	_sigma_position.setIdentity();
	_sigma_orientation.setIdentity();
	_sigma_force.setZero();
	_sigma_moment.setZero();

	_sensed_force_r1.setZero();
	_sensed_force_r2.setZero();
	_sensed_moment_r1.setZero();
	_sensed_moment_r2.setZero();

	_object_sensed_force.setZero();
	_object_sensed_moment.setZero();
	_object_desired_force.setZero();
	_object_desired_moment.setZero();

	_closed_loop_force_control = false;
	_closed_loop_moment_control = false;

	// internal forces quantities
	_desired_internal_tension = 0;
	_desired_internal_moments.setZero(5);
	_sensed_internal_tension = 0;
	_sensed_internal_moments.setZero(5);

	// task force
	_task_force.setZero(12);

	_first_iteration = true;

	_step_desired_object_position = _desired_object_position;
	_step_desired_object_velocity = _desired_object_velocity;
	_step_desired_object_orientation = _desired_object_orientation;
	_step_object_orientation_error.setZero(3);
	_step_desired_object_angular_velocity.setZero(3);

#ifdef USING_OTG
	_otg_pos->reInitialize(_current_object_position);
	_otg_ori->reInitialize(_current_object_orientation);

#endif
}

void TwoHandTwoRobotsTask::setObjectMassPropertiesAndInitialInertialFrameLocation(double object_mass, Eigen::Affine3d T_world_com, Eigen::Matrix3d object_inertia)
{
	_object_mass = object_mass;

	// compute the transformation between the control point and the inertial frame
	Eigen::Affine3d T_world_controlpoint = Eigen::Affine3d::Identity();
	T_world_controlpoint.translation() = _current_object_position;
	T_world_controlpoint.linear() = _current_object_orientation;

	_T_com_controlpoint = T_world_com.inverse() * T_world_controlpoint;

	// compute the object inertia tensor in the control frame
	Eigen::Vector3d p_com_cp = _T_com_controlpoint.translation();
	Eigen::Matrix3d object_inertia_wrt_control_point = object_inertia +     // parallel axis theorem
		object_mass * (p_com_cp.transpose()*p_com_cp * Eigen::Matrix3d::Identity() - p_com_cp*p_com_cp.transpose()); 
	Eigen::Matrix3d _object_inertia_in_control_frame = _T_com_controlpoint.linear().transpose() * object_inertia_wrt_control_point * _T_com_controlpoint.linear();  // we rotate the tensor in the control frame

	// update the object effective inertia in the control frame and the total effective inertia in world frame
	_object_effective_inertia.setZero(6,6);
	_object_effective_inertia.block<3,3>(0,0) = _object_mass * Eigen::Matrix3d::Identity();
	_object_effective_inertia.block<3,3>(3,3) = _current_object_orientation * _object_inertia_in_control_frame * _current_object_orientation.transpose();

	_Lambda_tot = _object_effective_inertia + _robot_1_effective_inertia + _robot_2_effective_inertia;
}

void TwoHandTwoRobotsTask::setControlFrameLocationInitial(Eigen::Affine3d T_world_controlpoint)
{
	_current_object_position = T_world_controlpoint.translation();
	_current_object_orientation = T_world_controlpoint.linear();
}

void TwoHandTwoRobotsTask::setForceSensorFrames(const std::string link_name_1, const Eigen::Affine3d sensor_in_link_r1, 
									const std::string link_name_2, const Eigen::Affine3d sensor_in_link_r2)
{
	if(link_name_1 != _link_name_1  ||  link_name_2 != _link_name_2)
	{
		throw std::invalid_argument("The link to which is attached the sensor should be the same as the link to which is attached the control frame in TwoHandTwoRobotsTask::setForceSensorFrames\n");
	}

	_T_contact_fsensor_r1 = _control_frame_1.inverse() * sensor_in_link_r1;
	_T_contact_fsensor_r2 = _control_frame_2.inverse() * sensor_in_link_r2;
}


void TwoHandTwoRobotsTask::updateSensedForcesAndMoments(const Eigen::Vector3d sensed_force_sensor_frame_r1,
									const Eigen::Vector3d sensed_moment_sensor_frame_r1,
									const Eigen::Vector3d sensed_force_sensor_frame_r2,
									const Eigen::Vector3d sensed_moment_sensor_frame_r2)
{

	// find forces at the contact points in world frame
	// first, the transform frem world frame to each arm contact frame
	Eigen::Affine3d T_world_hand_1;
	Eigen::Affine3d T_world_hand_2;
	_robot_arm_1->transformInWorld(T_world_hand_1, _link_name_1);
	_robot_arm_2->transformInWorld(T_world_hand_2, _link_name_2);
	Eigen::Affine3d T_world_contact_1 = T_world_hand_1 * _control_frame_1;
	Eigen::Affine3d T_world_contact_2 = T_world_hand_2 * _control_frame_2;

	// find the resolved sensed force and moment in control frame
	_sensed_force_r1 = _T_contact_fsensor_r1.rotation() * sensed_force_sensor_frame_r1;
	_sensed_force_r2 = _T_contact_fsensor_r2.rotation() * sensed_force_sensor_frame_r2;
	_sensed_moment_r1 = _T_contact_fsensor_r1.translation().cross(_sensed_force_r1) + _T_contact_fsensor_r1.rotation() * sensed_moment_sensor_frame_r1;
	_sensed_moment_r2 = _T_contact_fsensor_r2.translation().cross(_sensed_force_r2) + _T_contact_fsensor_r2.rotation() * sensed_moment_sensor_frame_r2;

	// rotate the quantities in world frame
	_sensed_force_r1 = T_world_contact_1.rotation() * _sensed_force_r1;
	_sensed_force_r2 = T_world_contact_2.rotation() * _sensed_force_r2;
	_sensed_moment_r1 = T_world_contact_1.rotation() * _sensed_moment_r1;
	_sensed_moment_r2 = T_world_contact_2.rotation() * _sensed_moment_r2;

	// compute the object forces and internal froces from the grasp matrix
	Eigen::VectorXd full_contact_forces = Eigen::VectorXd::Zero(12);
	full_contact_forces << _sensed_force_r1, _sensed_force_r2, _sensed_moment_r1, _sensed_moment_r2;

	Eigen::VectorXd resultant_and_internal_forces = _grasp_matrix * full_contact_forces;

	_object_sensed_force = resultant_and_internal_forces.segment<3>(0) + _object_gravity.head(3);
	_object_sensed_moment = resultant_and_internal_forces.segment<3>(3) + _object_gravity.tail(3);
	_sensed_internal_tension = resultant_and_internal_forces(6);
	_sensed_internal_moments = resultant_and_internal_forces.segment<5>(7);
}

void TwoHandTwoRobotsTask::setForceAxis(const Eigen::Vector3d force_axis)
{
	Eigen::Vector3d normalized_axis = force_axis.normalized();

	_sigma_force = normalized_axis*normalized_axis.transpose();
	_sigma_position = Eigen::Matrix3d::Identity() - _sigma_force;
}

void TwoHandTwoRobotsTask::setLinearMotionAxis(const Eigen::Vector3d linear_motion_axis)
{
	Eigen::Vector3d normalized_axis = linear_motion_axis.normalized();

	_sigma_position = normalized_axis*normalized_axis.transpose();
	_sigma_force = Eigen::Matrix3d::Identity() - _sigma_position;
}

void TwoHandTwoRobotsTask::setMomentAxis(const Eigen::Vector3d moment_axis)
{
	Eigen::Vector3d normalized_axis = moment_axis.normalized();

	_sigma_moment = normalized_axis*normalized_axis.transpose();
	_sigma_orientation = Eigen::Matrix3d::Identity() - _sigma_moment;
}

void TwoHandTwoRobotsTask::setAngularMotionAxis(const Eigen::Vector3d angular_motion_axis)
{
	Eigen::Vector3d normalized_axis = angular_motion_axis.normalized();

	_sigma_orientation = normalized_axis*normalized_axis.transpose();
	_sigma_moment = Eigen::Matrix3d::Identity() - _sigma_orientation;
}

void TwoHandTwoRobotsTask::setFullLinearMotionControl()
{
	_sigma_position = Eigen::Matrix3d::Identity();
	_sigma_force.setZero();
}

void TwoHandTwoRobotsTask::setFullForceControl()
{
	_sigma_force = Eigen::Matrix3d::Identity();
	_sigma_position.setZero();
}

void TwoHandTwoRobotsTask::setFullAngularMotionControl()
{
	_sigma_orientation = Eigen::Matrix3d::Identity();
	_sigma_moment.setZero();
}

void TwoHandTwoRobotsTask::setFullMomentControl()
{
	_sigma_moment = Eigen::Matrix3d::Identity();
	_sigma_orientation.setZero();
}

void TwoHandTwoRobotsTask::setClosedLoopForceControl()
{
	_closed_loop_force_control = true;
	_integrated_object_force_error.setZero();
}


void TwoHandTwoRobotsTask::setClosedLoopMomentControl()
{
	_closed_loop_moment_control = true;
	_integrated_object_moment_error.setZero();
}


} /* namespace Sai2Primitives */

