#include <ros/ros.h>

#include <gantry_robot/Info.h>
#include <gantry_robot/Location.h>

gantry_robot::Info info_;

class Test {
    int i32;
    double float64;
};

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "test_cpp");
	ros::NodeHandle nh("~");

	gantry_robot::Location location_srv;

#include <typeinfo>
    std::cout << typeid(info_.axisX.location).name() << std::endl;
    std::cout << typeid(location_srv.request.x).name() << std::endl;
    std::cout << typeid(info_.axisX.location-location_srv.request.x).name() << std::endl;
    std::cout << typeid(abs(info_.axisX.location-location_srv.request.x)).name() << std::endl;
    std::cout << typeid(fabs(info_.axisX.location-location_srv.request.x)).name() << std::endl;
    std::cout << typeid(std::abs(info_.axisX.location-location_srv.request.x)).name() << std::endl;
    std::cout << typeid(std::fabs(info_.axisX.location-location_srv.request.x)).name() << std::endl;

	info_.axisX.location = 15.0;
	location_srv.request.x = 13.0;
	printf("%lf\n", abs(info_.axisX.location-location_srv.request.x));
	printf("%lf\n", fabs(info_.axisX.location-location_srv.request.x));
	printf("%lf\n", std::abs(info_.axisX.location-location_srv.request.x));
	printf("%lf\n", std::fabs(info_.axisX.location-location_srv.request.x));

    return 0;
}