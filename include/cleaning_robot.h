#ifndef KAERCHER_H_
#define KAERCHER_H_

#include <jsoncpp/json/value.h>
#include <jsoncpp/json/reader.h>
#include <string>
#include <cmath>
#include <vector>
#include <numeric>
#include <iostream>
#include <fstream>

#include "ros/ros.h"
#include <ros/console.h>

struct Point {
    double x;
    double y;
};

class cleaning_robot{

    public:
        cleaning_robot();
        ~cleaning_robot();

        double calculateCleaningArea(Json::Value lengthCleaningGadget, std::vector<double> distance);
        double calculateTimeOfPath(std::vector<double> distance);
        Json::Value getRobotInfo(std::string info) const;
        double calculatePath(std::vector<double> distance);
        std::vector<double> calculateDistance(Json::Value robot_path);

    private:
        std::string jsonfile;
        Json::Value robot_info;

        Json::Value readJsonFile(std::string jsonfile);
        double calculateVelocity(double Kappa);
        std::vector<Point> createVector(Json::Value robot_path);
        double calculateLength(const Point start, const Point end);
};

cleaning_robot::cleaning_robot(){
    ros::NodeHandle private_nh("~");
    
    // Check if json available?
    if(private_nh.getParam("jsonfile", jsonfile)){
        ROS_INFO("Read Json-File");
        robot_info = readJsonFile(jsonfile);
    }
    else{
        // aus Programm springen
        ROS_ERROR("JSON-File is not available");
    }

}

cleaning_robot::~cleaning_robot(){  

}


#endif