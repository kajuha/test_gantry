#include <ros/ros.h>

#include <gantry_robot/Info.h>
#include <gantry_robot/Location.h>
#include <gantry_robot/Command.h>

gantry_robot::Info info_;
void infoCallback(const gantry_robot::Info& info) {
	info_ = info;
}

int g_done_srv;
bool serviceDoneCallback(gantry_robot::Command::Request &req, gantry_robot::Command::Response &res) {
    ros::Time time = ros::Time::now();

	g_done_srv = req.command;
	ROS_INFO("service done [command:%d][ts:%lf]", g_done_srv, ros::Time::now().toSec());
	res.success = 1;

    return true;
}

enum class ActionState {
	INIT, LOCATION1, LOCATION2, LOCATION3, LOCATION4, HOME, IDLE
};

enum class CommandState {
	INIT, HOME, LOCATION, POSITION, JOG, STOP, IDLE, ERROR
};

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "test_gantry_robot");
	ros::NodeHandle nh("~");

    ros::ServiceClient client_command = nh.serviceClient<gantry_robot::Command>("/gantry_robot/gantry_robot_command");
    ros::ServiceClient client_location = nh.serviceClient<gantry_robot::Location>("/gantry_robot/gantry_robot_location");

    ros::ServiceServer service_done = nh.advertiseService("gantry_robot_done", serviceDoneCallback);

    ros::Subscriber sub_info = nh.subscribe("/gantry_robot/gantry_robot_info", 10, infoCallback);

    int main_hz = 1000;

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

	while (ros::ok()) {
		#if 1
		switch (actionState) {
			case ActionState::INIT:
				if (info_.axisX.org && info_.axisY.org && info_.axisZ.org) {
					actionState = ActionState::LOCATION1;
					printf("INIT done\n");
					location_srv.request.x = 0.0;
					location_srv.request.y = 0.0;
					location_srv.request.z = 0.0;
					client_location.call(location_srv);
				}
			break;
			case ActionState::LOCATION1:
#if 0			
				if (info_.axisX.eos && info_.axisY.eos && info_.axisZ.eos &&
					std::abs(info_.axisX.location-location_srv.request.x) < LOC_ERROR &&
#else					
				if (std::abs(info_.axisX.location-location_srv.request.x) < LOC_ERROR &&
#endif					
					std::abs(info_.axisY.location-location_srv.request.y) < LOC_ERROR &&
					std::abs(info_.axisZ.location-location_srv.request.z) < LOC_ERROR) {
					actionState = ActionState::LOCATION2;
					printf("LOCATION1 done\n");
					location_srv.request.x = 0.0;
					location_srv.request.y = 0.0;
					location_srv.request.z = 0.0;
					client_location.call(location_srv);
				}
			break;
			case ActionState::LOCATION2:
#if 0			
				if (info_.axisX.eos && info_.axisY.eos && info_.axisZ.eos &&
					std::abs(info_.axisX.location-location_srv.request.x) < LOC_ERROR &&
#else					
				if (std::abs(info_.axisX.location-location_srv.request.x) < LOC_ERROR &&
#endif					
					std::abs(info_.axisY.location-location_srv.request.y) < LOC_ERROR &&
					std::abs(info_.axisZ.location-location_srv.request.z) < LOC_ERROR) {
					actionState = ActionState::LOCATION3;
					printf("LOCATION2 done\n");
					location_srv.request.x = 0.0;
					location_srv.request.y = 0.0;
					location_srv.request.z = 0.0;
					client_location.call(location_srv);
				}
			break;
			case ActionState::LOCATION3:
#if 0			
				if (info_.axisX.eos && info_.axisY.eos && info_.axisZ.eos &&
					std::abs(info_.axisX.location-location_srv.request.x) < LOC_ERROR &&
#else					
				if (std::abs(info_.axisX.location-location_srv.request.x) < LOC_ERROR &&
#endif					
					std::abs(info_.axisY.location-location_srv.request.y) < LOC_ERROR &&
					std::abs(info_.axisZ.location-location_srv.request.z) < LOC_ERROR) {
					actionState = ActionState::LOCATION4;
					printf("LOCATION3 done\n");
					location_srv.request.x = 0.0;
					location_srv.request.y = 0.0;
					location_srv.request.z = 0.0;
					client_location.call(location_srv);
				}
			break;
			case ActionState::LOCATION4:
#if 0			
				if (info_.axisX.eos && info_.axisY.eos && info_.axisZ.eos &&
					std::abs(info_.axisX.location-location_srv.request.x) < LOC_ERROR &&
#else					
				if (std::abs(info_.axisX.location-location_srv.request.x) < LOC_ERROR &&
#endif					
					std::abs(info_.axisY.location-location_srv.request.y) < LOC_ERROR &&
					std::abs(info_.axisZ.location-location_srv.request.z) < LOC_ERROR) {
					actionState = ActionState::HOME;
					printf("LOCATION4 done\n");
					command_srv.request.command = 1;
					client_command.call(command_srv);
				}
			break;
			case ActionState::HOME:
				if (info_.axisX.org && info_.axisY.org && info_.axisZ.org &&
					std::abs(info_.axisX.location-0.0) < LOC_ERROR &&
					std::abs(info_.axisY.location-0.0) < LOC_ERROR &&
					std::abs(info_.axisZ.location-0.0) < LOC_ERROR) {
					actionState = ActionState::IDLE;
					printf("HOME done\n");
				}
			break;
			case ActionState::IDLE:
			break;
		}
		#else
		location_srv.request.x = 0.0;
		location_srv.request.y = 0.0;
		location_srv.request.z = 0.0;
		client_location.call(location_srv);
		sleep(3);
		location_srv.request.x = 0.6;
		location_srv.request.y = 0.0;
		location_srv.request.z = 0.2;
		client_location.call(location_srv);
		sleep(3);
		location_srv.request.x = 0.6;
		location_srv.request.y = 0.4;
		location_srv.request.z = 0.0;
		client_location.call(location_srv);
		sleep(3);
		location_srv.request.x = 0.0;
		location_srv.request.y = 0.4;
		location_srv.request.z = 0.2;
		client_location.call(location_srv);
		sleep(3);
		#endif

		ros::spinOnce();
		r.sleep();
	}

	return 0;
}