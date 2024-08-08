#include <ros/ros.h>
#include "rover/navigation_data.h"
#include <geometry_msgs/Twist.h> 
#include <cmath>

class movement_controller
{
private:
    ros::NodeHandle nh;
    rover::navigation_data navData;
    ros::Subscriber navDataSubscriber;  
    ros::Publisher cmd_velPublisher; 
    geometry_msgs::Twist output;
    enum state { goalReached, calibrating, moving }; 
    std::string currentStateString; 
    state currentState;
    double angularDifference;

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

public:
    movement_controller(){
        navDataSubscriber = nh.subscribe<rover::navigation_data>("/navigation_data",10,&movement_controller::navDataCallBack,this);
        cmd_velPublisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        currentStateString = "GoalReached";
        currentState = goalReached;
        angularDifference = 0;
        output.linear.x = output.linear.y = output.linear.z = output.angular.x = output.angular.y = output.angular.z = 0;
        ros::Rate loop_rate(50);
        while (ros::ok()){
            ros::spinOnce();
            getCurrentState();
            ROS_INFO("Current State = %s , Distance = %lf", currentStateString.c_str(),navData.distance);
            if(navData.distance<0.2)
                currentState = goalReached;
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
                    output.angular.z = 0.5;
                else
                    output.angular.z = -0.5;
                break;
                
            case moving:
                output.angular.z = 0;
                if(angularDifference>0.5||angularDifference<-0.5){
                    if (isClockWiseCloser()) 
                        output.angular.z = 0.2;
                    else
                        output.angular.z = -0.2;
                }
                output.linear.x = 0.5;
                break;

            default:
                break;
            }
            cmd_velPublisher.publish(output);
            loop_rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "movement_controller");
    movement_controller controller;
    return 0;
}
