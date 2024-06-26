#include <signal.h>

#include <iostream>
#include <mutex>
#include <string>

#include "Sai2Model.h"
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
const string world_file = "/home/harrison/stanford/OpenSai/cs225a/project_starter/22-dual_arms_keyboard_gripper/world.urdf";
const string robot_file = "/home/harrison/stanford/OpenSai/cs225a/urdf_models/panda/panda_arm_hand.urdf";
const string robot_name_1 = "PANDA";
const string robot_name_2 = "PANDA2";
const string link_name = "link7";

// mutex for control torques
mutex mtx;

// map of flags for key presses
map<int, bool> key_pressed = {
	{GLFW_KEY_P, false},
	{GLFW_KEY_L, false},
	{GLFW_KEY_W, false},
	{GLFW_KEY_O, false},
	{GLFW_KEY_J, false}, // key added 
	{GLFW_KEY_X, false},
	{GLFW_KEY_C, false},
	{GLFW_KEY_G, false},	// key added fore grasping!
	{GLFW_KEY_V, false}
};
map<int, bool> key_was_pressed = key_pressed;

}  // namespace

// Create simulation and control function
void runSim(shared_ptr<Sai2Simulation::Sai2Simulation> sim);
void runControl(shared_ptr<Sai2Simulation::Sai2Simulation> sim, 
				shared_ptr<Sai2Model::Sai2Model> robot,
				const string robot_name
);

// robot joint data
VectorXd robot_control_torques = Eigen::VectorXd::Zero(9);
VectorXd robot_control_torques_2 = Eigen::VectorXd::Zero(9);
bool robot_1_is_under_control = true;

bool key_board_only = true;


int main() {
	Sai2Model::URDF_FOLDERS["EXAMPLE_20_FOLDER"] =
		string(EXAMPLES_FOLDER) + "/20-dual_arms";
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
	sim->setCoeffFrictionStatic(0.0);

	// load graphics scene
	auto graphics = make_shared<Sai2Graphics::Sai2Graphics>(world_file);
	graphics->addForceSensorDisplay(sim->getAllForceSensorData()[0]);

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

		for (auto& key : key_pressed) {
			key_pressed[key.first] = graphics->isKeyPressed(key.first);
		}

		// if (key_pressed.at(GLFW_KEY_J)) {
		// 	graphics->updateRobotGraphics(robot_name_2, sim->getJointPositions(robot_name_2));
		// } else {
		// 	graphics->updateRobotGraphics(robot_name_1, sim->getJointPositions(robot_name_1));	
		// }

		graphics->updateRobotGraphics(robot_name_2, sim->getJointPositions(robot_name_2));
		graphics->updateRobotGraphics(robot_name_1, sim->getJointPositions(robot_name_1));	
		

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

	// create robot controller
	Affine3d compliant_frame = Affine3d::Identity();
	auto motion_force_task = make_shared<Sai2Primitives::MotionForceTask>(
		robot, link_name, compliant_frame);
	motion_force_task->disableInternalOtg();
	motion_force_task->enableVelocitySaturation(0.9, M_PI);
	motion_force_task->setOriControlGains(200.0, 25.0);
	Vector3d prev_sensed_force = Vector3d::Zero();

	vector<shared_ptr<Sai2Primitives::TemplateTask>> task_list = {
		motion_force_task};
	auto robot_controller =
		make_unique<Sai2Primitives::RobotController>(robot, task_list);

	// new
	MatrixXd gripper_selection_matrix = MatrixXd::Zero(2, robot->dof());
	gripper_selection_matrix(0, 7) = 1;
	gripper_selection_matrix(1, 8) = 1;
	auto gripper_task = make_shared<Sai2Primitives::JointTask>(robot, gripper_selection_matrix);
	gripper_task->setDynamicDecouplingType(Sai2Primitives::DynamicDecouplingType::IMPEDANCE);
	double kp_gripper = 5e3;
	double kv_gripper = 1e2;
	gripper_task->setGains(kp_gripper, kv_gripper, 0);
	Vector2d gripper_goal_open = Vector2d(0.02, -0.02);
	Vector2d gripper_goal_closed = Vector2d(0.0, 0.0);
	gripper_task->setGoalPosition(gripper_goal_open);

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
		if (key_pressed.at(GLFW_KEY_X)) 
		{
			cout << "Key X is pressed -- MOVING up" << endl;
			delta_xyz = Vector3d(0.,0.,+0.01);
		} else if (key_pressed.at(GLFW_KEY_C))
		{
			cout << "Key C is pressed -- MOVING in y-negative direction" << endl;
			delta_xyz = Vector3d(-0.01, 0., 0.);
		}
		else {
			// cout << "No key is pressed -- MOVING down, Consider to move up by pressing X, or C to move in negative x-axis" << endl;
			delta_xyz = Vector3d(0., 0., -0.01);
		}

		// new
		if (key_pressed.at(GLFW_KEY_G) && !key_was_pressed.at(GLFW_KEY_G)) {
    		cout << "Key G is pressed -- Changing gripper status" << endl;
    		if (robot_name == robot_name_1 && robot_1_is_under_control) {
        		if (gripper_task->getGoalPosition() == gripper_goal_open) {
            	gripper_task->setGoalPosition(gripper_goal_closed);
        		} else {
            		gripper_task->setGoalPosition(gripper_goal_open);
        		}
    		} else if (robot_name == robot_name_2 && !robot_1_is_under_control) {
        		if (gripper_task->getGoalPosition() == gripper_goal_open) {
            		gripper_task->setGoalPosition(gripper_goal_closed);
        		} else {
            	gripper_task->setGoalPosition(gripper_goal_open);
        		}
    		}
		}

 
		// else {
		// 	cout << "If you press Key G -- You will be able to change the gripper status" << endl;
		// }


		if (robot_name == robot_name_1) {
			if (key_pressed.at(GLFW_KEY_J) && !key_was_pressed.at(GLFW_KEY_J)) 
			{
				robot_1_is_under_control = !robot_1_is_under_control;
				cout << "robot_1_is_under_control=" << robot_1_is_under_control << endl;
			}
		}

		if (robot_name == robot_name_1) {

			if (!robot_1_is_under_control) {
				motion_force_task->setGoalPosition(
						robot->positionInWorld(link_name)
						);
				motion_force_task->setGoalOrientation(robot->rotationInWorld(link_name));
			}
			else {
				if (key_board_only) {
					motion_force_task->setGoalPosition(
						robot->positionInWorld(link_name) + delta_xyz
						);
					motion_force_task->setGoalOrientation(robot->rotationInWorld(link_name));
				} else {
					// compute haptic control
					haptic_input.robot_position = robot->positionInWorld(link_name);
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
					motion_force_task->setGoalOrientation(
						haptic_output.robot_goal_orientation);
				}

				redis_client.sendAllFromGroup();

			}
			
			// lockers
			{
				lock_guard<mutex> lock(mtx);
				robot_control_torques = robot_controller->computeControlTorques() + gripper_task->computeTorques();
			}

		} else if (robot_name == robot_name_2) {

			if (robot_1_is_under_control) {
				motion_force_task->setGoalPosition(
						robot->positionInWorld(link_name)
						);
				motion_force_task->setGoalOrientation(robot->rotationInWorld(link_name));
			}
			else {

				if (key_board_only) {
					motion_force_task->setGoalPosition(
						robot->positionInWorld(link_name) + delta_xyz
						);
				motion_force_task->setGoalOrientation(robot->rotationInWorld(link_name));
				} else {
					// compute haptic control
					haptic_input.robot_position = robot->positionInWorld(link_name);
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
					motion_force_task->setGoalOrientation(
						haptic_output.robot_goal_orientation);
				}

				redis_client.sendAllFromGroup();

			}

			// lockers
			{
				lock_guard<mutex> lock(mtx);
				robot_control_torques_2 = robot_controller->computeControlTorques() + gripper_task->computeTorques();;
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
		key_was_pressed = key_pressed;
	}

	redis_client.setEigen(createRedisKey(COMMANDED_FORCE_KEY_SUFFIX, 0),
						  Vector3d::Zero());
	redis_client.setEigen(createRedisKey(COMMANDED_TORQUE_KEY_SUFFIX, 0),
						  Vector3d::Zero());
	redis_client.setInt(createRedisKey(USE_GRIPPER_AS_SWITCH_KEY_SUFFIX, 0), 0);

	cout << "control timer stats:" << endl;
	controlTimer.printInfoPostRun();
}