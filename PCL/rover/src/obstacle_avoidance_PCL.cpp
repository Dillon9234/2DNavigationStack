#include <ros/ros.h>
#include "rover/navigation_data.h"
#include "rover/pcl_obstacle_data.h"
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Twist.h> 
#include <sensor_msgs/PointCloud2.h>
#include <cmath>

struct obstacleLocationStruct
{
    bool left;
    bool right;
    bool center;
};

class obstacle_avoidance_PCL
{
private:
    ros::NodeHandle nh;
    ros::Subscriber navDataSubscriber,PCLDataSubscriber;  
    ros::Publisher cmd_velPublisher; 
    geometry_msgs::Twist output; 
    ros::Duration interval;
    rover::navigation_data navData;
    rover::pcl_obstacle_data pclData;
    bool obstacleInPath;
    enum state { goalReached, calibrating, moving }; 
    std::string currentStateString; 
    state currentState;
    double angularDifference;
    bool firstEncounter;
    bool front = false;
    bool left = false;
    bool right = false;

    double hitpointDistance,leavePointDistance;

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

    void normalNav(){
        firstEncounter = true;
        currentState = calibrating;
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

            if(currentState == moving && front == 1 ){
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

    void pclCallBack(const rover::pcl_obstacle_data::ConstPtr& msg){
        pclData = *msg;
        front = pclData.front;
        left = pclData.left;
        right = pclData.right;
    }

    void navDataCallBack(const rover::navigation_data::ConstPtr& msg){
        navData = *msg;
    }


    void update(){

        if(navData.isOnLine){
            leavePointDistance = navData.distance;
            if(leavePointDistance<hitpointDistance && fabs(leavePointDistance - hitpointDistance) > 0.5){
                obstacleInPath = false;
                hitpointDistance = navData.distance;
            }
        }
    }


public:
    obstacle_avoidance_PCL(){
        cmd_velPublisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        PCLDataSubscriber = nh.subscribe<rover::pcl_obstacle_data>("/pcl_data",10,&obstacle_avoidance_PCL::pclCallBack,this);
        navDataSubscriber = nh.subscribe<rover::navigation_data>("/navigation_data",10,&obstacle_avoidance_PCL::navDataCallBack,this);

        interval = ros::Duration(2.5);
        obstacleInPath = true;
        navData.distance = 0;
        hitpointDistance = 9999999999;
        output.linear.x = output.linear.y = output.linear.z = output.angular.x = output.angular.y = output.angular.z = 0;

        ros::Rate loop_rate(50);
        while (ros::ok()){
            ros::spinOnce();     
                        
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
                    if(left == 0 && right == 0 && front ==0 ){
                        //turn left
                        //ROS_INFO("Turning Left");

                        output.angular.z = 2.75;
                        output.linear.x = 1;
                            
                    }
                    else if(left == 1 && front == 0){
                        //continue forward
                        //ROS_INFO("Moving Forward");
                        output.angular.z = 0;
                        output.linear.x = 1;
                    }
                    else {
                        //turn right
                        //ROS_INFO("Turning Right");

                        output.angular.z = -3;
                        output.linear.x = 0;

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
    obstacle_avoidance_PCL ob;
    return 0;
}
