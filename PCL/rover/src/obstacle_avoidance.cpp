#include <ros/ros.h>
#include "rover/navigation_data.h"
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Twist.h> 
#include <sensor_msgs/LaserScan.h>
#include <cmath>

struct obstacleLocationStruct
{
    bool left;
    bool right;
    bool center;
};

class obstacle_avoidance
{
private:
    ros::NodeHandle nh;
    rover::navigation_data navData;
    ros::Subscriber navDataSubscriber;  
    ros::Subscriber lidarDataSubscriber;  
    ros::Publisher cmd_velPublisher; 
    geometry_msgs::Twist output; 

    const float proximityThreshold = 1;
    const float frontproximityThreshold = proximityThreshold + 0.4;
    const float tooCloseThreshold = 0.6;
    ros::Duration interval;

    obstacleLocationStruct obstacleLocation;
    bool obstacleInPath;
    enum state { goalReached, calibrating, moving }; 
    std::string currentStateString; 
    state currentState;
    double angularDifference;
    bool firstEncounter;
    bool isTooClose;

    float rightMin;
    float centerMin;
    float leftMin;
    double hitpointDistance,leavePointDistance;


    void navDataCallBack(const rover::navigation_data::ConstPtr& msg){
        navData = *msg;
    }

    bool isClockWiseCloser(){
        double clockwiseDistance = fmod(navData.bearing - navData.orientation + 360.0, 360.0);
        double counterclockwiseDistance = fmod(navData.orientation - navData.bearing + 360.0, 360.0);
        if(clockwiseDistance>175 && counterclockwiseDistance> 175 && abs(clockwiseDistance - counterclockwiseDistance)<1)
            return true;
        if (clockwiseDistance < counterclockwiseDistance) 
            return true;
        return false;
    }
    void getCurrentState(){
        switch (currentState)
        {
        case goalReached:
            currentStateString = "GoalReached";
            break;
        case calibrating:
            currentStateString = "Calibrating";
            break;
        case moving:
            currentStateString = "Moving To Goal";
            break;
        default:
            currentStateString = "Error";
        }
    }

    float minOfArray(float arr[] , int n){
        float min = 100;
        for(int i=0; i<n; i++) 
        {
          if(arr[i]<min) 
          {
             min=arr[i];
          }
       }
       return min;
    }

    void LidarDataCallBack(const sensor_msgs::LaserScan::ConstPtr& msg){
        int length = msg->ranges.size();
        int lendiv3 = length/5;
        int r,m,l;
        float right[2*length/5] , center[length/5] , left[2*length/3];
        r=m=l=0;
        
        for( int i=0 ; i < length ; i++){
            if(i < 2*lendiv3)
            {
                right[r++] = msg->ranges[i];
            }
            else if (i<3*lendiv3)
            {
                center[m++] = msg->ranges[i];
            }
            else
            {
                left[l++] = msg->ranges[i];
            }        
        }
        rightMin = minOfArray(right , 2*length/5);
        centerMin = minOfArray(center , length/5);
        leftMin = minOfArray(left , 2*length/5);

        if ( rightMin >proximityThreshold  && centerMin > proximityThreshold && leftMin > proximityThreshold){
            obstacleLocation.center = obstacleLocation.left = obstacleLocation.right = false;
        }    
        else if ( rightMin > proximityThreshold  && centerMin < proximityThreshold && leftMin > proximityThreshold ){
            obstacleLocation.left = obstacleLocation.right = false;
            obstacleLocation.center = true;
        }
        else if ( rightMin < proximityThreshold  && centerMin > proximityThreshold && leftMin > proximityThreshold ){
            obstacleLocation.left = obstacleLocation.center = false;
            obstacleLocation.right = true;
        }
        else if ( rightMin > proximityThreshold && centerMin > proximityThreshold && leftMin < proximityThreshold ){
            obstacleLocation.right = obstacleLocation.center = false;
            obstacleLocation.left = true;
        }
        else if ( rightMin < proximityThreshold  && centerMin > proximityThreshold && leftMin < proximityThreshold ){
            obstacleLocation.left = obstacleLocation.right = true;
            obstacleLocation.center = false;
        }
        else if ( rightMin > proximityThreshold  && centerMin < frontproximityThreshold && leftMin < proximityThreshold ){
            obstacleLocation.left = obstacleLocation.center = true;
            obstacleLocation.right = false;
        }
        else if ( rightMin < proximityThreshold  && centerMin < frontproximityThreshold && leftMin > proximityThreshold ){
            obstacleLocation.right = obstacleLocation.center = true;
            obstacleLocation.left = false;
        }
        else if ( rightMin < proximityThreshold  && centerMin < frontproximityThreshold && leftMin < proximityThreshold ){
            obstacleLocation.center = obstacleLocation.left = obstacleLocation.right = true;
        }

        //ROS_INFO("%d %d %d\n",obstacleLocation.left,obstacleLocation.center,obstacleLocation.right);

    }

    void normalNav(){
        firstEncounter = true;
        currentState = moving;
        ros::Rate loop_rate(50);
        while (ros::ok()){
            ros::spinOnce();
            getCurrentState();
            //ROS_INFO("Current State = %s , Distance = %lf", currentStateString.c_str(),navData.distance);

            if(navData.distance<0.2){
                currentState = goalReached;
                output.linear.x = output.angular.z = 0;
                cmd_velPublisher.publish(output);
                return;
            }

            if(currentState == moving && obstacleLocation.center == 1 && centerMin < frontproximityThreshold - 0.2){
                hitpointDistance = navData.distance;
                obstacleInPath = true;
                return;
            }
                
            else if(currentState == goalReached)
                currentState = calibrating;
            if(currentState!=goalReached){
                angularDifference = navData.bearing-navData.orientation;
                if(angularDifference>5||angularDifference<-5)
                    currentState = calibrating;
                else
                    currentState = moving;
            }
            switch (currentState)
            {
            case goalReached: output.linear.x = output.angular.z = 0;
                break;
            case calibrating:
                output.linear.x=0;
                if (isClockWiseCloser()) 
                    output.angular.z = 1.5;
                else
                    output.angular.z = -1.5;
                break;

            case moving:
                output.angular.z = 0;
                if(angularDifference>0.5||angularDifference<-0.5){
                    if (isClockWiseCloser()) 
                        output.angular.z = 0.8;
                    else
                        output.angular.z = -0.8;
                }
                output.linear.x = 1;
                break;
            default:
                break;
            }
            cmd_velPublisher.publish(output);
            loop_rate.sleep();
        }
    }


    void update(){

        if(navData.isOnLine){
            leavePointDistance = navData.distance;
            if(leavePointDistance<hitpointDistance && fabs(leavePointDistance - hitpointDistance) > 0.5){
                obstacleInPath = false;
            }
        }
    }

    bool isTooCloseFn(){
        if(rightMin < tooCloseThreshold || leftMin < tooCloseThreshold || rightMin < tooCloseThreshold)
            return true;
        return false;
    }

public:
    obstacle_avoidance(){
        navDataSubscriber = nh.subscribe<rover::navigation_data>("/navigation_data",10,&obstacle_avoidance::navDataCallBack,this);
        lidarDataSubscriber = nh.subscribe<sensor_msgs::LaserScan>("/scan",10,&obstacle_avoidance::LidarDataCallBack,this);
        cmd_velPublisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        interval = ros::Duration(2.5);
        obstacleInPath = true;
        navData.distance = 0;
        hitpointDistance = 9999999999;
        output.linear.x = output.linear.y = output.linear.z = output.angular.x = output.angular.y = output.angular.z = 0;

        ros::Rate loop_rate(50);
        while (ros::ok()){
            ros::spinOnce();     

            isTooClose = isTooCloseFn();

            if(obstacleInPath){
                update();
                if(firstEncounter){
                    ROS_INFO("First Encounter");
                    output.angular.z = -2;
                    output.linear.x = 0;
                    cmd_velPublisher.publish(output);
                    firstEncounter = false;
                    ros::Duration(1.0).sleep();
                }

                if(navData.distance>0.2){
                    if(obstacleLocation.left == 0 ){
                        //turn left
                        //ROS_INFO("Turning Left");
                        
                        if(isTooClose){
                            output.angular.z = 3;
                            output.linear.x = 0;
                        }
                        else{
                            output.angular.z = 4.5;
                            output.linear.x = 1;
                        }
                            
                    }
                    else if(obstacleLocation.left == 1 && obstacleLocation.center == 0){
                        //continue forward
                        //ROS_INFO("Moving Forward");
                        output.angular.z = 0;
                        output.linear.x = 1;
                    }
                    else {
                        //turn right
                        //ROS_INFO("Turning Right");
                        if(isTooClose){
                            output.angular.z = -3;
                            output.linear.x = 0;
                        }
                        else{
                            output.angular.z = -4.5;
                            output.linear.x = 1;
                        }
                    }
                }else{
                    ROS_INFO("Goal Reached");
                    output.angular.z = 0;
                    output.linear.x = 0;
                }
                cmd_velPublisher.publish(output);
            }else
                normalNav();
            
            loop_rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_avoidance");
    obstacle_avoidance ob;
    return 0;
}
