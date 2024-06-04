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

// Vector2d gripper_goal_open = Vector2d(0.02, -0.02);
// Vector2d gripper_goal_open = Vector2d(0.05, -0.05);
Vector2d gripper_goal_open = Vector2d(0.1, -0.1);


Vector2d gripper_goal_closed = Vector2d(0.0, 0.0);


double kp_gripper = 100.0;
double kv_gripper = 20.0;

// double kp_gripper = 1e3;
// double kv_gripper = 1e2;

// double kp_gripper = 5e3;
// double kv_gripper = 1e2;

bool gripper_1_is_open = true;
bool gripper_2_is_open = true;

bool robot_1_is_under_control = true;

bool key_board_only = true;
// bool key_board_only = false;

// How to switch between the two robots
// bool keep_pressing_B_to_switch = true;
bool keep_pressing_B_to_switch = false;
Vector6d UI_torques = Eigen::VectorXd::Zero(6);

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
	sim->setCoeffFrictionStatic(10.0);
	// sim->setCoeffFrictionDynamic(5.0);
	sim->setCoeffFrictionDynamic(10.0);

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
	Sai2Common::LoopTimer graphicsTimer(30.0, 1e6);

	while (graphics->isWindowOpen()) {
		graphicsTimer.waitForNextLoop();

		{
			lock_guard<mutex> lock(key_mtx);
			for (auto& key : key_pressed) {
				key_pressed[key.first] = graphics->isKeyPressed(key.first);
			}
		}

		// if (key_pressed.at(GLFW_KEY_B)) {
		// 	graphics->updateRobotGraphics(robot_name_2, sim->getJointPositions(robot_name_2));
		// } else {
		// 	graphics->updateRobotGraphics(robot_name_1, sim->getJointPositions(robot_name_1));	
		// }

		graphics->updateRobotGraphics(robot_name_2, sim->getJointPositions(robot_name_2));
		graphics->updateRobotGraphics(robot_name_1, sim->getJointPositions(robot_name_1));	


		UI_torques = graphics->getUITorques("Box1");
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

		// for (const auto& object_name : sim->getObjectNames()) {
		// 	sim->setObjectForceTorque(object_name,
		// 							  graphics->getUITorques(object_name));
		// }
		// // sim->integrate();

		{
			lock_guard<mutex> lock(mtx);
			sim->setJointTorques(robot_name_1, robot_control_torques);
		}

		{
			lock_guard<mutex> lock(mtx);
			sim->setJointTorques(robot_name_2, robot_control_torques_2);
		}

		sim->setObjectForceTorque("Box1",
									  UI_torques);

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

	// instructions
	cout
		<< "\nexmaple of a motion-motion controller to control a simulated "
		   "robot "
		   "with a haptic device, using direct force feedback with POPC "
		   "bilateral teleoperation to stabilize the contact force. The "
		   "controller will first bring the haptic device to its home pose and "
		   "then a press of the gripper or button is required to start "
		   "cotnrolling the robot."
		<< endl;
	cout << "Provided options:" << endl;
	cout << "1. press the device gripper/button to clutch the device (move the "
			"device without moving the robot) and release to get back the "
			"control of the robot."
		 << endl;

	// create robot controller -- 6Dof Action
	const Vector3d control_point = Vector3d(0, 0, 0.07);
	Affine3d compliant_frame = Affine3d::Identity();
	compliant_frame.translation() = control_point;
	auto motion_force_task = make_shared<Sai2Primitives::MotionForceTask>(
		robot, 
		link_name, 
		compliant_frame
	);


	// auto motion_force_task = std::make_shared<Sai2Primitives::MotionForceTask>(robot, control_link, compliant_frame);
	// motion_force_task->setPosControlGains(400, 40, 0);
	// motion_force_task->setOriControlGains(400, 40, 0);

	// create gripper task -- 2Dof Action
	MatrixXd gripper_selection_matrix = MatrixXd::Zero(2, robot->dof());
	gripper_selection_matrix(0, 7) = 1;
	gripper_selection_matrix(1, 8) = 1;
	auto gripper_task = make_shared<Sai2Primitives::JointTask>(robot, gripper_selection_matrix);
	gripper_task->setDynamicDecouplingType(Sai2Primitives::DynamicDecouplingType::IMPEDANCE);
	
	gripper_task->setGains(kp_gripper, kv_gripper, 0);
	gripper_task->setGoalPosition(gripper_goal_open);
	if (robot_name == robot_name_1) {
		gripper_1_is_open = true;
	} else if (robot_name == robot_name_2) {
		gripper_2_is_open = true;
	}

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


	motion_force_task->disableInternalOtg();
	motion_force_task->enableVelocitySaturation(0.9, M_PI);
	motion_force_task->setPosControlGains(400, 40, 0);
	motion_force_task->setOriControlGains(200.0, 25.0);
	Vector3d prev_sensed_force = Vector3d::Zero();

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

	while (fSimulationRunning) {
		// wait for next scheduled loop
		controlTimer.waitForNextLoop();

		// read robot data from simulation thread
		robot->setQ(sim->getJointPositions(robot_name));
		robot->setDq(sim->getJointVelocities(robot_name));
		robot->updateModel();

		robot_controller->updateControllerTaskModels();

		// read haptic device state from redis
		redis_client.receiveAllFromGroup();

		// compute robot control
		motion_force_task->updateSensedForceAndMoment(
			sim->getSensedForce(robot_name, link_name),
			sim->getSensedMoment(robot_name, link_name));
		
		// state machine for button presses
		if (haptic_controller->getHapticControlType() ==
				Sai2Primitives::HapticControlType::HOMING &&
			haptic_controller->getHomed() && haptic_button_is_pressed) {
			haptic_controller->setHapticControlType(
				Sai2Primitives::HapticControlType::MOTION_MOTION);
			haptic_controller->setDeviceControlGains(200.0, 15.0);
			cout << "haptic device homed" << endl;
		}

		Vector3d delta_xyz = Vector3d::Zero();

		// initial orientation
		Matrix3d cur_orientation = robot->rotationInWorld(link_name);
		Matrix3d goal_orientation = cur_orientation;

		// local boolean to store key press status 
		bool key_Q_pressed, key_W_pressed, key_E_pressed, key_R_pressed;
		bool key_X_pressed, key_C_pressed, key_J_pressed, key_L_pressed;
		bool key_I_pressed, key_K_pressed, key_N_pressed, key_M_pressed;
		bool key_G_pressed, key_B_pressed, key_B_was_pressed;

		// Mutex block to safely read the key states
		{
			lock_guard<mutex> lock(key_mtx);
			key_Q_pressed = key_pressed.at(GLFW_KEY_Q);
			key_W_pressed = key_pressed.at(GLFW_KEY_W);
			key_E_pressed = key_pressed.at(GLFW_KEY_E);
			key_R_pressed = key_pressed.at(GLFW_KEY_R);
			key_X_pressed = key_pressed.at(GLFW_KEY_X);
			key_C_pressed = key_pressed.at(GLFW_KEY_C);
			key_J_pressed = key_pressed.at(GLFW_KEY_J);
			key_L_pressed = key_pressed.at(GLFW_KEY_L);
			key_I_pressed = key_pressed.at(GLFW_KEY_I);
			key_K_pressed = key_pressed.at(GLFW_KEY_K);
			key_N_pressed = key_pressed.at(GLFW_KEY_N);
			key_M_pressed = key_pressed.at(GLFW_KEY_M);

			key_G_pressed = key_pressed.at(GLFW_KEY_G);
			key_B_pressed = key_pressed.at(GLFW_KEY_B);
			key_B_was_pressed = key_was_pressed.at(GLFW_KEY_B);
		}


		// Move in Z direction
		if (key_Q_pressed) 
		{
			// cout << "Key Q is pressed -- MOVING up (Z-positive direction)" << endl;
			delta_xyz = 0.01 * Vector3d::UnitZ();
		} else if (key_W_pressed)
		{
			// cout << "Key W is pressed -- MOVING down (Z-negative direction)" << endl;
			delta_xyz = -0.01 * Vector3d::UnitZ();
		}

		// Move in Y direction
		if (key_E_pressed)
		{
			// cout << "Key E is pressed -- MOVING in Y-positive direction" << endl;
			delta_xyz = 0.01 * Vector3d::UnitY();
		} else if (key_R_pressed)
		{
			// cout << "Key R is pressed -- MOVING in Y-negative direction" << endl;
			delta_xyz = -0.01 * Vector3d::UnitY();
		}

		// Move in Y direction
		if (key_X_pressed) 
		{
			// cout << "Key X is pressed -- MOVING in X-positive direction" << endl;
			delta_xyz = 0.01 * Vector3d::UnitX();
		} else if (key_C_pressed)
		{
			// cout << "Key C is pressed -- MOVING in X-negative direction" << endl;
			delta_xyz = -0.01 * Vector3d::UnitX();
		}


		// rotate about X-axis
		if (key_J_pressed) 
		{
			// cout << "Key J is pressed -- Rot CCW about X-axis " << endl;
			goal_orientation =
				AngleAxisd( + M_PI / 6.0, Vector3d::UnitX()).toRotationMatrix() *
				cur_orientation;
		} else if (key_L_pressed)
		{
			// cout << "Key L is pressed -- Rot CW about X-axis " << endl;
			goal_orientation =
				AngleAxisd( - M_PI / 6.0, Vector3d::UnitX()).toRotationMatrix() *
				cur_orientation;
		}

		// rotate about Y-axis
		if (key_I_pressed) 
		{
			// cout << "Key I is pressed -- Rot CCW about Y-axis " << endl;
			goal_orientation =
				AngleAxisd( + M_PI / 3.0, Vector3d::UnitY()).toRotationMatrix() *
				cur_orientation;
		} else if (key_K_pressed)
		{
			// cout << "Key K is pressed -- Rot CW about Y-axis " << endl;

			goal_orientation =
				AngleAxisd( - M_PI / 3.0, Vector3d::UnitY()).toRotationMatrix() *
				cur_orientation;
		}

		// rotate about Z-axis
		if (key_N_pressed) 
		{
			// cout << "Key N is pressed -- Rot CCW about Z-axis " << endl;
			goal_orientation =
				AngleAxisd( + M_PI / 3.0, Vector3d::UnitZ()).toRotationMatrix() *
				cur_orientation;
		} else if (key_M_pressed)
		{
			// cout << "Key M is pressed -- Rot CW about Z-axis " << endl;

			goal_orientation =
				AngleAxisd( - M_PI / 3.0, Vector3d::UnitZ()).toRotationMatrix() *
				cur_orientation;
		}

		// Change grasping status
		if (robot_name == robot_name_1) {
			if (key_G_pressed) {gripper_1_is_open=false;}
			else {gripper_1_is_open=true;}
		}

		if (robot_name == robot_name_2) {
			if (key_G_pressed) {gripper_2_is_open=false;}
			else {gripper_2_is_open=true;}
		}

		// Switch between the two robots
		if (robot_name == robot_name_1 && robot_1_is_under_control) {
			if (key_B_pressed && !key_B_was_pressed){
				cout << "Key B is pressed - switching Robot " << endl;
				robot_1_is_under_control = !robot_1_is_under_control;
			} 
		} 
		if (robot_name == robot_name_2 && !robot_1_is_under_control) {
			if (key_B_pressed && !key_B_was_pressed){
				cout << "Key B is pressed - changing Robot" << endl;
				robot_1_is_under_control = !robot_1_is_under_control;
			}
		}

		
//    update task model
// 		N_prec.setIdentity();
// 		motion_force_task->updateTaskModel(N_prec);
// 		gripper_task->updateTaskModel(motion_force_task->getTaskAndPreviousNullspace());
// 		joint_task->updateTaskModel(gripper_task->getTaskAndPreviousNullspace());


// 		if (robot_name == robot_name_1) {
if (robot_name == robot_name_1 && robot_1_is_under_control) {

			if (!robot_1_is_under_control) {
				motion_force_task->reInitializeTask();
			}
			else {
				// Grasping control
				if (gripper_1_is_open) {gripper_task->setGoalPosition(gripper_goal_open);}
				else {gripper_task->setGoalPosition(gripper_goal_closed);}

				// 6Dof Control 
				if (key_board_only) {
					motion_force_task->setGoalPosition(
						robot->positionInWorld(link_name, control_point) + delta_xyz
						);
					motion_force_task->setGoalOrientation(
						goal_orientation
						);
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

					motion_force_task->setGoalPosition(haptic_output.robot_goal_position);
					// motion_force_task->setGoalOrientation(
					// 	haptic_output.robot_goal_orientation);
					motion_force_task->setGoalOrientation(
						goal_orientation);
				}

				redis_client.sendAllFromGroup();

				// cout << "robot_control_torques=" << robot_control_torques << endl;
			}
			
			// lockers
			{
				lock_guard<mutex> lock(mtx);
				robot_control_torques = robot_controller->computeControlTorques() + gripper_task->computeTorques();
			}
			if (!gripper_1_is_open){
				// cout << "Gripper 1 is closed" << endl;
				MatrixXd Jv = MatrixXd::Zero(3,dof);
				Jv = robot->Jv(link_name, control_point);
				Vector3d gravity = Vector3d(0, 0, -9.8);
				float mass = 1;
				Vector3d force = gravity * mass;
				robot_control_torques = robot_control_torques - Jv.transpose() *force;
			}

		} else if (robot_name == robot_name_2 && !robot_1_is_under_control) {

			if (robot_1_is_under_control) {
				motion_force_task->reInitializeTask();
			}
			else {
				
				// Grasping control
				if (gripper_2_is_open) {gripper_task->setGoalPosition(gripper_goal_open);}
				else {gripper_task->setGoalPosition(gripper_goal_closed);}

				// 6Dof Control 
				if (key_board_only) {
					motion_force_task->setGoalPosition(
						robot->positionInWorld(link_name, control_point) + delta_xyz
						);
					motion_force_task->setGoalOrientation(
						goal_orientation
						);
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

					motion_force_task->setGoalPosition(haptic_output.robot_goal_position);
					// motion_force_task->setGoalOrientation(
					// 	haptic_output.robot_goal_orientation);
					motion_force_task->setGoalOrientation(
						goal_orientation);
				}

				redis_client.sendAllFromGroup();

				// cout << "robot_control_torques_2=" << robot_control_torques_2 << endl;
			}

			// lockers
			{
				lock_guard<mutex> lock(mtx);
				robot_control_torques_2 = robot_controller->computeControlTorques() + gripper_task->computeTorques();
			}
			if (!gripper_2_is_open){
				// cout << "Gripper 2 is closed" << endl;
				MatrixXd Jv = MatrixXd::Zero(3,dof);
				Jv = robot->Jv(link_name, control_point);
				Vector3d gravity = Vector3d(0, 0, -9.8);
				float mass = 1;
				Vector3d force = gravity * mass;
				robot_control_torques_2 = robot_control_torques_2 - Jv.transpose() *force;
			}
		}

		// cout << "haptic_input.robot_position " << haptic_input.robot_position.transpose() << endl;

		// compute POPC
		auto POPC_force_moment =
			POPC_teleop->computeAdditionalHapticDampingForce();
		haptic_output.device_command_force += POPC_force_moment.first;


		if (haptic_controller->getHapticControlType() ==
				Sai2Primitives::HapticControlType::MOTION_MOTION &&
			haptic_button_is_pressed && !haptic_button_was_pressed) {
			haptic_controller->setHapticControlType(
				Sai2Primitives::HapticControlType::CLUTCH);
		} else if (haptic_controller->getHapticControlType() ==
					Sai2Primitives::HapticControlType::CLUTCH &&
				!haptic_button_is_pressed && haptic_button_was_pressed) {
			haptic_controller->setHapticControlType(
				Sai2Primitives::HapticControlType::MOTION_MOTION);
		} 

		haptic_button_was_pressed = haptic_button_is_pressed;

		{
			lock_guard<mutex> lock(key_mtx);
			key_was_pressed = key_pressed;
		}
	}

	redis_client.setEigen(createRedisKey(COMMANDED_FORCE_KEY_SUFFIX, 0),
						  Vector3d::Zero());
	redis_client.setEigen(createRedisKey(COMMANDED_TORQUE_KEY_SUFFIX, 0),
						  Vector3d::Zero());
	redis_client.setInt(createRedisKey(USE_GRIPPER_AS_SWITCH_KEY_SUFFIX, 0), 0);

	cout << "control timer stats:" << endl;
	controlTimer.printInfoPostRun();
}