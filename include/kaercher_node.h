#ifndef KAERCHER_H_
#define KAERCHER_H_

#include <jsoncpp/json/value.h>

#include "ros/ros.h"
#include <ros/console.h>


class cleaning_robot{

    public:
        cleaning_robot();
        ~cleaning_robot();

        Json::Value robot_info;
        double calculatePath(Json::Value robot_path);
        double calculateCleaningArea(Json::Value lengthCleaningGadget, Json::Value path);
        double calculateTimeOfPath();
        Json::Value getRobotInfo(std::string info) const;

    private:
        std::string jsonfile;
        Json::Value readJsonFIle(std::string jsonfile);
        double calculateDistance(double x1, double y1, double x2, double y2);
        double calculateVelocity(float Kappa);
};

cleaning_robot::cleaning_robot(){
    ros::NodeHandle private_nh("~");
    
    // Check if json available?
    if(private_nh.getParam("jsonfile", jsonfile)){
        ROS_INFO("Read Json-File");
        robot_info = readJsonFIle(jsonfile);
    }
    else{
        // aus Programm springen
        ROS_ERROR("JSON-File is not available"); 
    }

}

cleaning_robot::~cleaning_robot(){  

}


#endif