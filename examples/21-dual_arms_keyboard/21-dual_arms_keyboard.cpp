#include <signal.h>

#include <iostream>
#include <mutex>
#include <string>

#include "Sai2Graphics.h"
#include "Sai2Primitives.h"
#include "Sai2Simulation.h"
#include "redis/RedisClient.h"
#include "redis/keys/chai_haptic_devices_driver.h"
#include "timer/LoopTimer.h"
bool fSimulationRunning = false;
void sighandler(int) { fSimulationRunning = false; }

using namespace std;
using namespace Eigen;
using namespace Sai2Common::ChaiHapticDriverKeys;

namespace {
const string world_file = "${EXAMPLE_21_FOLDER}/world_with_gripper.urdf";
const string robot_file = "${EXAMPLE_21_FOLDER}/panda/panda_arm_with_gripper.urdf";

const string robot_name_1 = "PANDA";
const string robot_name_2 = "PANDA2";
// const string link_name = "end-effector";
const string link_name = "link7";

// mutex for control torques
mutex mtx;
// mutex for key press checks
mutex key_mtx;

// map of flags for key presses
map<int, bool> key_pressed = {
	{GLFW_KEY_B, false}, // key to swith robots
	{GLFW_KEY_G, false},	// key to change grasping status

	// Translation Keys
	{GLFW_KEY_Q, false},	// MOVING up (Z-positive direction)   up
	{GLFW_KEY_W, false},	// MOVING down (Z-negative direction)  down

	{GLFW_KEY_E, false},	// MOVING in Y-positive direction   right 
	{GLFW_KEY_R, false},	// MOVING in Y-negative direction   left 

	{GLFW_KEY_X, false},	// MOVING in X-positive direction   front 
	{GLFW_KEY_C, false},	// MOVING in X-negative direction   back 

	// Rotation Keys
	{GLFW_KEY_J, false},	// Rot CCW about X-axis
	{GLFW_KEY_L, false},	// Rot CW about X-axis
	
	{GLFW_KEY_I, false},	// Rot CCW about Y-axis
	{GLFW_KEY_K, false},	// Rot CW about Y-axis

	{GLFW_KEY_N, false},	// Rot CCW about Z-axis
	{GLFW_KEY_M, false},	// Rot CW about Z-axis
};

map<int, bool> key_was_pressed = key_pressed;

}  // namespace

// Create simulation and control function
void runSim(shared_ptr<Sai2Simulation::Sai2Simulation> sim);
void runControl(shared_ptr<Sai2Simulation::Sai2Simulation> sim, 
				shared_ptr<Sai2Model::Sai2Model> robot,
				const string robot_name
);

VectorXd robot_control_torques = Eigen::VectorXd::Zero(9);
VectorXd robot_control_torques_2 = Eigen::VectorXd::Zero(9);

VectorXd delta_xyz_1 = Eigen::VectorXd::Zero(3);
VectorXd delta_xyz_2 = Eigen::VectorXd::Zero(3);

VectorXd delta_rot_1 = Eigen::VectorXd::Zero(3);
VectorXd delta_rot_2 = Eigen::VectorXd::Zero(3);

bool gripper_1_is_open = true;
bool gripper_2_is_open = true;

Vector2d gripper_goal_open = Vector2d(0.08, -0.08);

// Vector2d gripper_goal_closed = Vector2d(-0.05, +0.05);
Vector2d gripper_goal_closed = Vector2d(0.0, 0.0);

double kp_gripper = 1000.0;	// 1000
double kv_gripper = 20.0;	// 200

// How to switch between the two robots
bool robot_1_is_under_control = true;
bool robot_1_was_under_control = false;

bool key_board_only = true;
// bool key_board_only = false;

// nearest object
string nearest_obj_name_1 = "Box1";
string nearest_obj_name_2 = "Box1";
const Vector3d control_point = Vector3d(0, 0, 0.27);

// initial kinematic data
Matrix3d initial_orientation;

// for synchronizing between threads
int main_thread_itr = 0;

float delta_xyz_norm = 0.03;

// UI torques
// Vector6d UI_torques = Eigen::VectorXd::Zero(6);

int main() {
	Sai2Model::URDF_FOLDERS["EXAMPLE_21_FOLDER"] =
		string(EXAMPLES_FOLDER) + "/21-dual_arms_keyboard";
	
	cout << "Loading URDF world model file: "
		 << Sai2Model::ReplaceUrdfPathPrefix(world_file) << endl;
	cout << string(EXAMPLES_FOLDER) + "/21-dual_arms_keyboard" << endl;
	
	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load simulation world
	auto sim = make_shared<Sai2Simulation::Sai2Simulation>(world_file);
	sim->addSimulatedForceSensor(robot_name_1, link_name, Affine3d::Identity(),
								 55.0);
	sim->addSimulatedForceSensor(robot_name_2, link_name, Affine3d::Identity(),
								55.0);
	sim->setCoeffFrictionStatic(1.0);
	// sim->setCoeffFrictionDynamic(5.0);
	sim->setCoeffFrictionDynamic(1.0);

	sim->setCollisionRestitution(0.2);
	// sim->setCollisionRestitution(0.0);


	// load graphics scene
	auto graphics = make_shared<Sai2Graphics::Sai2Graphics>(world_file);
	graphics->addForceSensorDisplay(sim->getAllForceSensorData()[0]);
	for (const auto& object_name : sim->getObjectNames()) {
		graphics->addUIForceInteraction(object_name);
	}

	// load robot
	// [1] for robot_name_1
	const Affine3d T_world_robot = sim->getRobotBaseTransform(robot_name_1);
	auto robot = make_shared<Sai2Model::Sai2Model>(robot_file);
	robot->setTRobotBase(T_world_robot);
	robot->setQ(sim->getJointPositions(robot_name_1));
	robot->setDq(sim->getJointVelocities(robot_name_1));
	robot->updateModel();

	initial_orientation = robot->rotation("link7");

	// [2] for robot_name_2
	const Affine3d T_world_robot_2 = sim->getRobotBaseTransform(robot_name_2);
	auto robot_2 = make_shared<Sai2Model::Sai2Model>(robot_file);
	robot_2->setTRobotBase(T_world_robot_2);
	robot_2->setQ(sim->getJointPositions(robot_name_2));
	robot_2->setDq(sim->getJointVelocities(robot_name_2));
	robot_2->updateModel();


	// Run simulation for both robot arms
	thread sim_thread(runSim, sim);

	// control threads for each individual robot
	thread control_thread_1(runControl, sim, robot, robot_name_1);
	thread control_thread_2(runControl, sim, robot_2, robot_name_2);

	// graphics timer
	Sai2Common::LoopTimer graphicsTimer(90.0, 1e6);
	// Sai2Common::LoopTimer graphicsTimer(30.0, 1e6);

	// local boolean to store key press status 
	bool key_Q_pressed, key_W_pressed, key_E_pressed, key_R_pressed;
	bool key_X_pressed, key_C_pressed, key_J_pressed, key_L_pressed;
	bool key_I_pressed, key_K_pressed, key_N_pressed, key_M_pressed;
	bool key_G_pressed, key_B_pressed, key_B_was_pressed, key_G_was_pressed;

	VectorXd local_delta_xyz = Eigen::VectorXd::Zero(3);
	VectorXd local_delta_rot = Eigen::VectorXd::Zero(3);

	while (graphics->isWindowOpen()) {
		graphicsTimer.waitForNextLoop();

		{
			lock_guard<mutex> lock(key_mtx);
			// std::atomic<int> key_pressed;
			for (auto& key : key_pressed) {
				key_pressed[key.first] = graphics->isKeyPressed(key.first);
			}
		}

		// Mutex block to safely read the key states
		key_Q_pressed = key_pressed.at(GLFW_KEY_Q) && !key_was_pressed.at(GLFW_KEY_Q);
		key_W_pressed = key_pressed.at(GLFW_KEY_W) && !key_was_pressed.at(GLFW_KEY_W);
		key_E_pressed = key_pressed.at(GLFW_KEY_E) && !key_was_pressed.at(GLFW_KEY_E);
		key_R_pressed = key_pressed.at(GLFW_KEY_R) && !key_was_pressed.at(GLFW_KEY_R);
		key_X_pressed = key_pressed.at(GLFW_KEY_X) && !key_was_pressed.at(GLFW_KEY_X);
		key_C_pressed = key_pressed.at(GLFW_KEY_C) && !key_was_pressed.at(GLFW_KEY_C);
		key_J_pressed = key_pressed.at(GLFW_KEY_J) && !key_was_pressed.at(GLFW_KEY_J);
		key_L_pressed = key_pressed.at(GLFW_KEY_L) && !key_was_pressed.at(GLFW_KEY_L);
		key_I_pressed = key_pressed.at(GLFW_KEY_I) && !key_was_pressed.at(GLFW_KEY_I);
		key_K_pressed = key_pressed.at(GLFW_KEY_K) && !key_was_pressed.at(GLFW_KEY_K);
		key_N_pressed = key_pressed.at(GLFW_KEY_N) && !key_was_pressed.at(GLFW_KEY_N);
		key_M_pressed = key_pressed.at(GLFW_KEY_M) && !key_was_pressed.at(GLFW_KEY_M);
		key_G_pressed = key_pressed.at(GLFW_KEY_G);
		key_B_pressed = key_pressed.at(GLFW_KEY_B);
		key_B_was_pressed = key_was_pressed.at(GLFW_KEY_B);
		key_G_was_pressed = key_was_pressed.at(GLFW_KEY_G);

		local_delta_xyz.setZero();
		local_delta_rot.setZero();

		if (key_board_only) {
			// Move in Z direction --  Q (UP) and W (DOWN)
			if (key_Q_pressed) {local_delta_xyz = delta_xyz_norm * Vector3d::UnitZ();}
			else if (key_W_pressed) {local_delta_xyz = -delta_xyz_norm * Vector3d::UnitZ();}
			// Move in Y direction -- E (Y+) and R (Y-)
			if (key_E_pressed) {local_delta_xyz = delta_xyz_norm * Vector3d::UnitY();}
			else if (key_R_pressed) {local_delta_xyz = -delta_xyz_norm * Vector3d::UnitY();}
			// Move in X direction -- X (X+) and C (X-)
			if (key_X_pressed) {local_delta_xyz = delta_xyz_norm * Vector3d::UnitX();}
			else if (key_C_pressed) {local_delta_xyz = -delta_xyz_norm * Vector3d::UnitX();}
		}
		else {
			// dummy non-zero delta xyz to be used as a signal for 
			local_delta_xyz.setZero(); 
		}
		
		// rotate about X-axis -- J (+) and L (-)
		if (key_J_pressed) {local_delta_rot(0) = + M_PI / 30.0;}
		else if (key_L_pressed) {local_delta_rot(0) = - M_PI / 30.0;}
		// rotate about Y-axis -- I (+) and K (-)
		if (key_I_pressed) {local_delta_rot(1) = + M_PI / 30.0;}
		else if (key_K_pressed) {local_delta_rot(1) = - M_PI / 30.0;}
		// rotate about Z-axis -- N (+) and M (-)
		if (key_N_pressed) {local_delta_rot(2) = + M_PI / 30.0;}
		else if (key_M_pressed) {local_delta_rot(2) = - M_PI / 30.0;}


		{
			lock_guard<mutex> lock(key_mtx);

			// Which robot is under control
			robot_1_was_under_control = robot_1_is_under_control;
			if (key_B_pressed && !key_B_was_pressed) {
				robot_1_is_under_control = !robot_1_is_under_control;
				cout << "Key B is pressed - switching Robot, robot_1_is_under_control=" << robot_1_is_under_control << endl;
			}

			// assign local_delta_xyz and local_delta_rot to delta_xyz_1 and delta_rot_1
			// assign local_delta_xyz and local_delta_rot to delta_xyz_2 and delta_rot_2
			// Update grasping status

			delta_xyz_1.setZero();
			delta_rot_1.setZero();
			delta_xyz_2.setZero();
			delta_rot_2.setZero();
			if (robot_1_is_under_control) {
				// assign local_delta_xyz and local_delta_rot to delta_xyz_1 and delta_rot_1
				delta_xyz_1 = local_delta_xyz;
				delta_rot_1 = local_delta_rot;

				// // Update grasping status
				if (key_G_pressed && !key_G_was_pressed) {
					cout << "Key G is pressed - changing grasping status for Robot 1" << endl;
					gripper_1_is_open = !gripper_1_is_open;
				}

				// if (key_G_pressed) {gripper_1_is_open=false;}
				// else {gripper_1_is_open=true;}
			} else {
				// assign local_delta_xyz and local_delta_rot to delta_xyz_2 and delta_rot_2
				delta_xyz_2 = local_delta_xyz;
				delta_rot_2 = local_delta_rot;

				// Update grasping status
				if (key_G_pressed && !key_G_was_pressed) {
					cout << "Key G is pressed - changing grasping status for Robot 2" << endl;
					gripper_2_is_open = !gripper_2_is_open;
				}
				// if (key_G_pressed) {gripper_2_is_open=false;}
				// else {gripper_2_is_open=true;}
			}
		}

		main_thread_itr += 1;
		key_was_pressed = key_pressed;

		map<double, string> distance_1;
		for (const auto& object_name : sim->getObjectNames()) {
			Eigen::Affine3d obj_pose_in_world = sim->getObjectPose(object_name);
			Eigen::Affine3d obj_pose_in_robot_base = T_world_robot.inverse() * obj_pose_in_world;
			Eigen::Affine3d T_link_base = robot->transform(link_name).inverse();
			Eigen::Vector3d obj_pos_in_link = T_link_base * obj_pose_in_robot_base.translation() - control_point;
			double dist = obj_pos_in_link.norm();
			distance_1.insert({dist, object_name});
		}
		auto it_1 = distance_1.begin();
		nearest_obj_name_1 = it_1->second;

		map<double, string> distance_2;
		for (const auto& object_name : sim->getObjectNames()) {
			Eigen::Affine3d obj_pose_in_world = sim->getObjectPose(object_name);
			Eigen::Affine3d obj_pose_in_robot_base = T_world_robot_2.inverse() * obj_pose_in_world;
			Eigen::Affine3d T_link_base = robot_2->transform(link_name).inverse();
			Eigen::Vector3d obj_pos_in_link = T_link_base * obj_pose_in_robot_base.translation() - control_point;
			double dist = obj_pos_in_link.norm();
			distance_2.insert({dist, object_name});
		}
		auto it_2 = distance_2.begin();
		nearest_obj_name_2 = it_2->second;

		graphics->updateRobotGraphics(robot_name_2, sim->getJointPositions(robot_name_2));
		graphics->updateRobotGraphics(robot_name_1, sim->getJointPositions(robot_name_1));	


		// UI_torques = graphics->getUITorques("Box1");
		// graphics->updateObjectGraphics("Box1",
		// 								   sim->getObjectPose("Box1"),
		// 								   sim->getObjectVelocity("Box1"));
		for (const auto& object_name : sim->getObjectNames()) {
			graphics->updateObjectGraphics(object_name,
										   sim->getObjectPose(object_name),
										   sim->getObjectVelocity(object_name));
		}

		graphics->updateDisplayedForceSensor(sim->getAllForceSensorData()[0]);
		graphics->renderGraphicsWorld();
	}

	// stop simulation and control threads
	fSimulationRunning = false;
	sim_thread.join();
	control_thread_1.join();
	control_thread_2.join();

	return 0;
}

//------------------------------------------------------------------------------
////// Simulation thread //////
//------------------------------------------------------------------------------
void runSim(shared_ptr<Sai2Simulation::Sai2Simulation> sim) {

	// create a timer
	Sai2Common::LoopTimer simTimer(1.0 / sim->timestep(), 1e6);

	sim->enableGravityCompensation(true);

	fSimulationRunning = true;

	while (fSimulationRunning) {

		simTimer.waitForNextLoop();

		{
			lock_guard<mutex> lock(mtx);
			sim->setJointTorques(robot_name_1, robot_control_torques);
		}

		{
			lock_guard<mutex> lock(mtx);
			sim->setJointTorques(robot_name_2, robot_control_torques_2);
		}

		// sim->setObjectForceTorque("Box1",
		// 							  UI_torques);

		sim->integrate();

		
		
	}

	cout << "simulation timer stats:" << endl;
	simTimer.printInfoPostRun();
}

//------------------------------------------------------------------------------
////// Control thread //////
//------------------------------------------------------------------------------
void runControl(shared_ptr<Sai2Simulation::Sai2Simulation> sim, 
				shared_ptr<Sai2Model::Sai2Model> robot, 
				const string robot_name
				) {
	// redis client
	auto redis_client = Sai2Common::RedisClient();
	redis_client.connect();

	// create robot controller -- 6Dof Action
	// const Vector3d control_point = Vector3d(0, 0, 0.17);
	const Vector3d control_point = Vector3d(0, 0, 0.27);

	// const Vector3d control_point = Vector3d(0, 0, 0.07);
	Affine3d compliant_frame = Affine3d::Identity();
	compliant_frame.translation() = control_point;
	auto motion_force_task = make_shared<Sai2Primitives::MotionForceTask>(
		robot, 
		link_name, 
		compliant_frame
	);

	motion_force_task->disableInternalOtg();
	motion_force_task->enableVelocitySaturation(0.9, M_PI);
	motion_force_task->setPosControlGains(400, 40, 0);
	motion_force_task->setOriControlGains(200.0, 25.0);

	// create gripper task -- 2Dof Action
	MatrixXd gripper_selection_matrix = MatrixXd::Zero(2, robot->dof());
	gripper_selection_matrix(0, 7) = 1;
	gripper_selection_matrix(1, 8) = 1;
	auto gripper_task = make_shared<Sai2Primitives::JointTask>(robot, gripper_selection_matrix);
	gripper_task->setDynamicDecouplingType(Sai2Primitives::DynamicDecouplingType::IMPEDANCE);
	gripper_task->setGains(kp_gripper, kv_gripper, 0);
	gripper_task->setGoalPosition(gripper_goal_open);

	// arm posture task
	MatrixXd joint_selection_matrix = MatrixXd::Zero(7, robot->dof());
	joint_selection_matrix.block(0, 0, 7, 7).setIdentity();
	auto arm_posture_task = std::make_shared<Sai2Primitives::JointTask>(robot, joint_selection_matrix);

	// joint task
	int dof = robot->dof();
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);
	// auto joint_task = std::make_shared<Sai2Primitives::JointTask>(robot);
	// joint_task->setGains(400, 40, 0);
	// VectorXd q_desired(robot->dof());
	// q_desired.head(7) << -30.0, -15.0, -15.0, -105.0, 0.0, 90.0, 45.0;
	// q_desired.head(7) *= M_PI / 180.0;
	// q_desired.tail(2) << 0.04, -0.04;
	// joint_task->setGoalPosition(q_desired);

	// Include joint_task in the task list as well?
	vector<shared_ptr<Sai2Primitives::TemplateTask>> task_list = {
		motion_force_task};
	auto robot_controller =
		make_unique<Sai2Primitives::RobotController>(robot, task_list);


	// create haptic controller
	Sai2Primitives::HapticDeviceController::DeviceLimits device_limits(
		redis_client.getEigen(createRedisKey(MAX_STIFFNESS_KEY_SUFFIX, 0)),
		redis_client.getEigen(createRedisKey(MAX_DAMPING_KEY_SUFFIX, 0)),
		redis_client.getEigen(createRedisKey(MAX_FORCE_KEY_SUFFIX, 0)));
	auto haptic_controller =
		make_shared<Sai2Primitives::HapticDeviceController>(
			device_limits, robot->transformInWorld(link_name));
	haptic_controller->setScalingFactors(3.5);
	haptic_controller->setReductionFactorForce(0.7);
	haptic_controller->setVariableDampingGainsPos(vector<double>{0.25, 0.35},
												  vector<double>{0, 20});
	haptic_controller->setHapticControlType(
		Sai2Primitives::HapticControlType::HOMING);
	haptic_controller->disableOrientationTeleop();

	Sai2Primitives::HapticControllerInput haptic_input;
	Sai2Primitives::HapticControllerOtuput haptic_output;
	bool haptic_button_was_pressed = false;
	int haptic_button_is_pressed = 0;
	redis_client.setInt(createRedisKey(SWITCH_PRESSED_KEY_SUFFIX, 0),
						haptic_button_is_pressed);
	redis_client.setInt(createRedisKey(USE_GRIPPER_AS_SWITCH_KEY_SUFFIX, 0), 1);

	// create bilateral teleop POPC
	auto POPC_teleop = make_shared<Sai2Primitives::POPCBilateralTeleoperation>(
		motion_force_task, haptic_controller, 0.001);

	// setup redis communication
	redis_client.addToSendGroup(createRedisKey(COMMANDED_FORCE_KEY_SUFFIX, 0),
								haptic_output.device_command_force);
	redis_client.addToSendGroup(createRedisKey(COMMANDED_TORQUE_KEY_SUFFIX, 0),
								haptic_output.device_command_moment);

	redis_client.addToReceiveGroup(createRedisKey(POSITION_KEY_SUFFIX, 0),
								   haptic_input.device_position);
	redis_client.addToReceiveGroup(createRedisKey(ROTATION_KEY_SUFFIX, 0),
								   haptic_input.device_orientation);
	redis_client.addToReceiveGroup(
		createRedisKey(LINEAR_VELOCITY_KEY_SUFFIX, 0),
		haptic_input.device_linear_velocity);
	redis_client.addToReceiveGroup(
		createRedisKey(ANGULAR_VELOCITY_KEY_SUFFIX, 0),
		haptic_input.device_angular_velocity);
	redis_client.addToReceiveGroup(createRedisKey(SWITCH_PRESSED_KEY_SUFFIX, 0),
								   haptic_button_is_pressed);

	// create a timer
	Sai2Common::LoopTimer controlTimer(1000.0, 1e6);

	// computing transformations
 	const Affine3d T_world_robot = sim->getRobotBaseTransform(robot_name);
 	const Affine3d T_robot_world = T_world_robot.inverse();

	// Only for the current robot
	bool gripper_hold_obj = false;
	MatrixXd Jv = MatrixXd::Zero(3,dof);
	Vector3d gravity = Vector3d(0, 0, -9.8);
	float mass = 0.2;
	Vector3d force = gravity * mass;
	VectorXd obj_projected_torques = Eigen::VectorXd::Zero(9);

	Matrix3d init_rot = robot->rotationInWorld(link_name);
	// Matrix3d init_rot = robot->rotationInWorld(link_name, control_point);
	Matrix3d goal_rot = init_rot;
	// cout << goal_rot << endl;
	Matrix3d delta_rot = Matrix3d::Identity();

	Vector3d init_xyz = robot->positionInWorld(link_name, control_point);
	Vector3d goal_xyz = init_xyz;

	VectorXd local_delta_xyz = Eigen::VectorXd::Zero(3);
	VectorXd local_delta_rot = Eigen::VectorXd::Zero(3);

	Matrix3d orientation_after_transition = robot->rotation("link7");
	int local_main_thread_itr = -1;

	bool local_gripper_is_open = true;

	string local_nearest_obj_name = "Box1";

	bool local_is_under_control = false;
	bool local_was_under_control = true;

	while (fSimulationRunning) {
		// wait for next scheduled loop
		controlTimer.waitForNextLoop();

		// read robot data from simulation thread
		robot->setQ(sim->getJointPositions(robot_name));
		robot->setDq(sim->getJointVelocities(robot_name));
		robot->updateModel();

		robot_controller->updateControllerTaskModels();
		arm_posture_task->updateTaskModel(motion_force_task->getTaskAndPreviousNullspace());

		// read haptic device state from redis
		redis_client.receiveAllFromGroup();

		// compute robot control
		// TODO: Do I need to include the control point here?
		motion_force_task->updateSensedForceAndMoment(
			sim->getSensedForce(robot_name, link_name),
			sim->getSensedMoment(robot_name, link_name));
		
		// state machine for button presses
		if (haptic_controller->getHapticControlType() ==
				Sai2Primitives::HapticControlType::HOMING &&
			haptic_controller->getHomed() && haptic_button_is_pressed) {
			haptic_controller->setHapticControlType(
				Sai2Primitives::HapticControlType::MOTION_MOTION);
			// haptic_controller->setHapticControlType(
			// 	Sai2Primitives::HapticControlType::CLUTCH);
			haptic_controller->setDeviceControlGains(200.0, 15.0);
			cout << "haptic device homed" << endl;
		}

		local_delta_xyz.setZero();
		local_delta_rot.setZero();

		// Read the local_delta_xyz and local_delta_rot from global variables
		if ((robot_name == robot_name_1) && (local_main_thread_itr != main_thread_itr)) {
			local_delta_xyz = delta_xyz_1;
			local_delta_rot = delta_rot_1;
			local_gripper_is_open = gripper_1_is_open;
			local_nearest_obj_name = nearest_obj_name_1;

			// local_was_under_control = local_is_under_control;
			local_is_under_control = robot_1_is_under_control;
		} else if (robot_name == robot_name_2 && (local_main_thread_itr != main_thread_itr)) {
			local_delta_xyz = delta_xyz_2;
			local_delta_rot = delta_rot_2;
			local_gripper_is_open = gripper_2_is_open;
			local_nearest_obj_name = nearest_obj_name_2;

			// local_was_under_control = local_is_under_control;
			local_is_under_control = !robot_1_is_under_control;
		}
		local_main_thread_itr = main_thread_itr;

		if (haptic_controller->getHapticControlType() ==
			Sai2Primitives::HapticControlType::MOTION_MOTION &&
			((haptic_button_is_pressed && !haptic_button_was_pressed))) {
			haptic_controller->setHapticControlType(
				Sai2Primitives::HapticControlType::CLUTCH);
		} else if (haptic_controller->getHapticControlType() ==
					Sai2Primitives::HapticControlType::CLUTCH &&
				!haptic_button_is_pressed && haptic_button_was_pressed) {
			haptic_controller->setHapticControlType(
				Sai2Primitives::HapticControlType::MOTION_MOTION);
		} 

		haptic_button_was_pressed = haptic_button_is_pressed;

		// Update the goal position and orientation
		// goal_xyz = robot->positionInWorld(link_name, control_point);
		// goal_rot = robot->rotationInWorld(link_name);

		goal_xyz = goal_xyz + local_delta_xyz;
		// goal_rot = 
		// 		AngleAxisd( local_delta_rot(0) , Vector3d::UnitX()).toRotationMatrix() *
		// 		goal_rot;
		// goal_rot = 
		// 		AngleAxisd( local_delta_rot(1) , Vector3d::UnitY()).toRotationMatrix() *
		// 		goal_rot;
		// goal_rot = 
		// 		AngleAxisd( local_delta_rot(2) , Vector3d::UnitZ()).toRotationMatrix() *
		// 		goal_rot;
		delta_rot = AngleAxisd( local_delta_rot(2) , Vector3d::UnitZ()).toRotationMatrix() * \
					AngleAxisd( local_delta_rot(1) , Vector3d::UnitY()).toRotationMatrix() * \
					AngleAxisd( local_delta_rot(0) , Vector3d::UnitX()).toRotationMatrix();
		// delta_rot = temp_delta_rot * delta_rot;
		goal_rot = delta_rot * goal_rot;

		// if ((robot_name == robot_name_1) && (delta_rot != Matrix3d::Identity())){
		// 	std::cout << "delta rotation\n" << delta_rot << "\n";
		// 	// std::cout << "init rotation\n" << init_rot << "\n";
		// 	std::cout << "goal rotation\n" << goal_rot << "\n";
		// 	// cout << "temp delta rot" << temp_delta_rot << endl;
		// }
		

		// if (actively_controlled) {	
		if (local_is_under_control) {
			if (key_board_only) {
				motion_force_task->setGoalPosition(goal_xyz);
				motion_force_task->setGoalOrientation(goal_rot);
			} else {
				// compute haptic control
				haptic_input.robot_position = robot->positionInWorld(link_name, control_point);
				haptic_input.robot_orientation = robot->rotationInWorld(link_name);
				haptic_input.robot_linear_velocity =
					robot->linearVelocityInWorld(link_name);
				haptic_input.robot_angular_velocity =
					robot->angularVelocityInWorld(link_name);
				haptic_input.robot_sensed_force =
					motion_force_task->getSensedForceControlWorldFrame();
				haptic_input.robot_sensed_moment =
					motion_force_task->getSensedMomentControlWorldFrame();

				haptic_output = haptic_controller->computeHapticControl(haptic_input);

				if (local_is_under_control) {
					motion_force_task->setGoalPosition(haptic_output.robot_goal_position);
				}
				else {
					motion_force_task->setGoalPosition(goal_xyz);
				}
				motion_force_task->setGoalOrientation(goal_rot);
			}
		}		
		else if (local_was_under_control) {
			motion_force_task->reInitializeTask();
		}

		// gripper control
		if (local_gripper_is_open) {gripper_task->setGoalPosition(gripper_goal_open);}
		else {gripper_task->setGoalPosition(gripper_goal_closed);}
		
		// additional torques for object holding
		gripper_hold_obj = false;
		obj_projected_torques.setZero();
		if (robot->q()(7) > 0.015) {
			// Jv = robot->Jv(link_name, control_point);
			
			Eigen::Affine3d obj_pose_in_world = sim->getObjectPose(local_nearest_obj_name);
			Eigen::Affine3d obj_pose_in_robot_base = T_robot_world * obj_pose_in_world;
			Eigen::Affine3d T_link_base = robot->transform(link_name).inverse();
			Vector3d obj_pos_in_link = T_link_base * obj_pose_in_robot_base.translation();
			Jv = robot->Jv(link_name, obj_pos_in_link);

			if ( (robot_name == robot_name_1) && (!gripper_1_is_open) ) {
				gripper_hold_obj = true;
				obj_projected_torques = -1 * Jv.transpose() * force;
			} else if ( (robot_name == robot_name_2) && (!gripper_2_is_open) ) {
				gripper_hold_obj = true;
				obj_projected_torques = -1 * Jv.transpose() * force;
			} else {
				gripper_hold_obj = false;
				obj_projected_torques.setZero();
			}
		}


		if (!gripper_hold_obj) {obj_projected_torques.setZero();}

		if (robot_name == robot_name_1) {

			// cout << "robot->q()(7) " << robot->q()(7) << " gripper_hold_obj " << gripper_hold_obj << " obj_projected_torques: " << obj_projected_torques << endl;

		}



		// Communicate with the simulation thread
		redis_client.sendAllFromGroup();

		// lockers
		{
			lock_guard<mutex> lock(mtx);
			if (robot_name == robot_name_1) {
				robot_control_torques = robot_controller->computeControlTorques() + gripper_task->computeTorques() + obj_projected_torques + arm_posture_task->computeTorques();
			}

			if (robot_name == robot_name_2) {
				robot_control_torques_2 = robot_controller->computeControlTorques() + gripper_task->computeTorques() + obj_projected_torques + arm_posture_task->computeTorques();
			}
			
		}

		// compute POPC
		auto POPC_force_moment =
			POPC_teleop->computeAdditionalHapticDampingForce();
		haptic_output.device_command_force += POPC_force_moment.first;

		// if (haptic_controller->getHapticControlType() ==
		// 		Sai2Primitives::HapticControlType::MOTION_MOTION &&
		// 	haptic_button_is_pressed && !haptic_button_was_pressed) {
		// 	haptic_controller->setHapticControlType(
		// 		Sai2Primitives::HapticControlType::CLUTCH);
		// } else if (haptic_controller->getHapticControlType() ==
		// 			Sai2Primitives::HapticControlType::CLUTCH &&
		// 		!haptic_button_is_pressed && haptic_button_was_pressed) {
		// 	haptic_controller->setHapticControlType(
		// 		Sai2Primitives::HapticControlType::MOTION_MOTION);
		// } 

		// haptic_button_was_pressed = haptic_button_is_pressed;

		local_was_under_control = local_is_under_control;

	}

	redis_client.setEigen(createRedisKey(COMMANDED_FORCE_KEY_SUFFIX, 0),
						  Vector3d::Zero());
	redis_client.setEigen(createRedisKey(COMMANDED_TORQUE_KEY_SUFFIX, 0),
						  Vector3d::Zero());
	redis_client.setInt(createRedisKey(USE_GRIPPER_AS_SWITCH_KEY_SUFFIX, 0), 0);

	cout << "control timer stats:" << endl;
	controlTimer.printInfoPostRun();
}