#include <ros/ros.h>

#include <gantry_robot/Info.h>
#include <gantry_robot/Location.h>
#include <gantry_robot/Command.h>

#define SRV_SUCCESS	1
#define NOT_DONE	-1

#define MAIN_HZ		1

gantry_robot::Info info_;
void infoCallback(const gantry_robot::Info& info) {
	info_ = info;
}

int g_done_srv;
bool serviceDoneCallback(gantry_robot::Command::Request &req, gantry_robot::Command::Response &res) {
    ros::Time time = ros::Time::now();

	g_done_srv = req.command;
	ROS_INFO("service done [command:%d][ts:%lf]", g_done_srv, ros::Time::now().toSec());
	res.success = SRV_SUCCESS;

    return true;
}

enum class ActionState {
	INIT, LOCATION1, LOCATION2, LOCATION3, LOCATION4, HOME, IDLE
};

enum class CommandState {
	INIT, HOME, LOCATION, POSITION, JOG, STOP, IDLE, ERROR, NOT_SET=-1
};

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "test_gantry_robot");
	ros::NodeHandle nh("~");

    ros::ServiceClient client_command = nh.serviceClient<gantry_robot::Command>("/gantry_robot/gantry_robot_command");
    ros::ServiceClient client_location = nh.serviceClient<gantry_robot::Location>("/gantry_robot/gantry_robot_location");

    ros::ServiceServer service_done = nh.advertiseService("gantry_robot_done", serviceDoneCallback);

    ros::Subscriber sub_info = nh.subscribe("/gantry_robot/gantry_robot_info", 10, infoCallback);

    int main_hz = MAIN_HZ;

	#define STEP_TIME 1.0
	double ts_run;
	double ts_cur;
	double ts_pre;
	double ts_diff;

	ros::Rate r(main_hz);

	ActionState actionState = ActionState::INIT;

#define LOC_ERROR_GANTRY	0.0001
#define LOC_ERROR_SERIAL	0.1
#define LOC_ERROR			LOC_ERROR_SERIAL
	gantry_robot::Location location_srv;
	gantry_robot::Command command_srv;
	
	// 초기화 요청
	g_done_srv = NOT_DONE;
	command_srv.request.command = (int32_t)CommandState::INIT;
	client_command.call(command_srv);

	while (ros::ok()) {
		switch (actionState) {
			case ActionState::INIT:
				if (g_done_srv == (int32_t)CommandState::INIT) {
					actionState = ActionState::LOCATION1;
					ROS_INFO("INIT done");

					g_done_srv = NOT_DONE;
					location_srv.request.x = 0.0;
					location_srv.request.y = 0.0;
					location_srv.request.z = 0.0;
					client_location.call(location_srv);
				}
			break;
			case ActionState::LOCATION1:
				if (g_done_srv == (int32_t)CommandState::LOCATION) {
					actionState = ActionState::LOCATION2;
					ROS_INFO("LOCATION1 done");

					g_done_srv = NOT_DONE;
					location_srv.request.x = 0.0;
					location_srv.request.y = 0.0;
					location_srv.request.z = 0.0;
					client_location.call(location_srv);
				}
			break;
			case ActionState::LOCATION2:
				if (g_done_srv == (int32_t)CommandState::LOCATION) {
					actionState = ActionState::LOCATION3;
					ROS_INFO("LOCATION2 done");

					g_done_srv = NOT_DONE;
					location_srv.request.x = 0.0;
					location_srv.request.y = 0.0;
					location_srv.request.z = 0.0;
					client_location.call(location_srv);
				}
			break;
			case ActionState::LOCATION3:
				if (g_done_srv == (int32_t)CommandState::LOCATION) {
					actionState = ActionState::LOCATION4;
					ROS_INFO("LOCATION3 done");

					g_done_srv = NOT_DONE;
					location_srv.request.x = 0.0;
					location_srv.request.y = 0.0;
					location_srv.request.z = 0.0;
					client_location.call(location_srv);
				}
			break;
			case ActionState::LOCATION4:
				if (g_done_srv == (int32_t)CommandState::LOCATION) {
					actionState = ActionState::HOME;
					ROS_INFO("LOCATION4 done");

					g_done_srv = NOT_DONE;
					command_srv.request.command = (int32_t)CommandState::HOME;
					client_command.call(command_srv);
				}
			break;
			case ActionState::HOME:
				if (g_done_srv == (int32_t)CommandState::HOME) {
					actionState = ActionState::IDLE;
					ROS_INFO("HOME done");
				}
			break;
			case ActionState::IDLE:
			break;
		}

		ros::spinOnce();
		r.sleep();
	}

	return 0;
}