#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include "rover/navigation_data.h"
#include <cmath>

struct Line{
    double slope;
    double intercept;
};

class navigation {

private:
    ros::NodeHandle nh;
    ros::Subscriber gpsSubscriber;
    ros::Subscriber imuSubscriber;
    ros::Publisher goalPublisher;
    sensor_msgs::NavSatFix gpsData;
    sensor_msgs::Imu imuData;
    double Latitude,Longitude;
    bool HasReached,gotGoal;
    Line line; 
    
    const double R = 6371000.0;  //radius of Earth in meters
    rover::navigation_data navData;

    void gpsCallBack(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        gpsData = *msg;
    }

    void imuCallBack(const sensor_msgs::Imu::ConstPtr& msg) {
        imuData = *msg;
    }

    void calculateData(){
        double bearing,distance;
        double latt1 = gpsData.latitude * M_PI / 180.0;
        double long1 = gpsData.longitude * M_PI / 180.0;
        double latt2 = Latitude * M_PI / 180.0;
        double long2 = Longitude * M_PI / 180.0;

        double dlat = latt2 - latt1;
        double dlon = long2 - long1;
        
        // Haversine formula
        double a = sin(dlat / 2.0) * sin(dlat / 2.0) +
                   cos(latt1) * cos(latt2) *
                   sin(dlon / 2.0) * sin(dlon / 2.0);
        double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
        distance = R * c;

        // Calculate bearing
        double x = sin(dlon) * cos(latt2);
        double y = cos(latt1) * sin(latt2) - sin(latt1) * cos(latt2) * cos(dlon);
        bearing = atan2(y, x);
        bearing = fmod((bearing + 2.0 * M_PI), (2.0 * M_PI));
        bearing *= 180.0 / M_PI;

        navData.bearing = bearing;
        navData.distance = distance;
    }
    
    void calculateYaw() {
        double x = imuData.orientation.x;
        double y = imuData.orientation.y;
        double z = imuData.orientation.z;
        double w = imuData.orientation.w;
        double sinr_cosp = +2.0 * (w * z + x * y);
        double cosr_cosp = +1.0 - 2.0 * (y * y + z * z);
        double yaw = atan2(sinr_cosp, cosr_cosp);
        yaw *= 180.0 / M_PI; 
        yaw = fmod(yaw + 360.0, 360.0);
        navData.orientation = yaw;
    }

    void setLine(){
        line.slope = (Latitude - gpsData.latitude)/(Longitude - gpsData.longitude);
        line.intercept = gpsData.latitude - line.slope*gpsData.longitude;
    }
    void checkIfOnLine(){
        double distance = (fabs (gpsData.latitude - line.slope * gpsData.longitude - line.intercept)) / (sqrt (1 + line.slope * line.slope));
        if(distance< 0.000002)
            navData.isOnLine = true;
        else
            navData.isOnLine = false;
    }

public:
    navigation(){        
        gpsSubscriber = nh.subscribe<sensor_msgs::NavSatFix>("/fix", 10,&navigation::gpsCallBack,this);
        imuSubscriber = nh.subscribe<sensor_msgs::Imu>("/imu/data",10,&navigation::imuCallBack,this);
        goalPublisher = nh.advertise<rover::navigation_data>("/navigation_data", 10);
        HasReached = true;
        gotGoal = false;
        ros::Rate loop_rate(50);
        while (ros::ok()){
            ros::spinOnce();
            if(HasReached)
                getGoal();
            if(gotGoal){
                calculateData();
                calculateYaw();
                checkIfOnLine();
                ROS_INFO("Distance:%lf Bearing:%lf Orientation:%lf isOnline:%d",navData.distance,navData.bearing,navData.orientation,navData.isOnLine);
                if(navData.distance<0.2){
                    HasReached = true;
                    gotGoal = false;
                }
                goalPublisher.publish(navData);
            }
            
            loop_rate.sleep();
        }
    }

    void getGoal(){
        ROS_INFO("Input goal (Latitude and Longitude)");
        std::cin>>Latitude>>Longitude;
        HasReached = false;
        gotGoal = true;
        setLine();
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "navigation");
    navigation nav;
    return 0;
}
