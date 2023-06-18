#include <iostream>
#include <fstream>
#include <jsoncpp/json/reader.h>
#include <string>
#include <cmath>

#include "ros/ros.h"
#include <ros/console.h>

#include "kaercher_node.h"


Json::Value cleaning_robot::readJsonFIle(std::string jsonfile){
    std::ifstream robot_file(jsonfile);
    Json::Value robot_information;
    Json::Reader reader;
    reader.parse(robot_file, robot_information);
    //checking if the json file has robot, cleaning gadget and path --> if not throw an fail
    return robot_information;
}

double cleaning_robot::calculateDistance(double x1, double y1, double x2, double y2){
    return std::sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
} 

double cleaning_robot::calculatePath(Json::Value robot_path){
    
    double total_distance = 0.0;

    // Calculating not the exact path 
    for(Json::ArrayIndex i = 0; i < robot_path.size() - 1; i++){
        double x1 = robot_path[i][0].asDouble();
        double y1 = robot_path[i][1].asDouble();
        double x2 = robot_path[i+1][0].asDouble();
        double y2 = robot_path[i+1][1].asDouble();

        double distance = calculateDistance(x1, y1, x2, y2);

        total_distance += distance;
 
    }
    return total_distance;
}

double cleaning_robot::calculateVelocity(float kappa){
    /*
    Kappa is be calculated from the path --> Curvature
    possile three velocity outcomes:
    1. straights and flat curves            --> max Velocity
    2. kappa is given                       --> calculate the velocity
    3. kappa is higher then kappa_max       --> min. Velocity
    */  

    double velRobot;
    float kappa_krit = 0.5;                 // Unit 1/m
    float kappa_max = 10;                   // Unit 1/m
    float vmin = 0.15;                      // Unit m/s
    float vmax = 1.1;                       // Unit m/s
    float v_kappa;                          // Unit m/s

    // calculate Velocity of the robot, if kappa is given
    v_kappa = vmax - ((vmax-vmin)/(kappa_max - kappa_krit))*(kappa-kappa_krit);
    
    // Check if kappa bigger then kappa_max 
    if(kappa > kappa_max){
        velRobot = vmin;
    }
    else{
        velRobot = v_kappa;
    }
    return velRobot;
}

double cleaning_robot::calculateCleaningArea(Json::Value robot, Json::Value lengthCleaningGadget, Json::Value path){
    /*
    Need the lenghts of the cleaning Gadget from the Json
    Need the complete Path of the Robot
    */

    double cleaningArea=0.0;

    // driving straight is the cleaningArea lengthCleaningGadget * path 
    // drinving curve is a little bit complicated --> M_PI * cleaning_gadget * curvature
    
    return cleaningArea;
}

// Need Velocity and Path to get the time
double cleaning_robot::calculateTimeOfPath(){
    return 0.0;
}

Json::Value cleaning_robot::getRobotInfo(std::string info) const {
    // check if info in the json file 
    return robot_info[info];
}

int main(int argc, char** argv){
    ros::init(argc, argv, "kaercher");
    cleaning_robot cleaning_robot;

    // getting the information from json-file
    Json::Value robot_path = cleaning_robot.getRobotInfo("path");
    Json::Value robot_cleaning_gadget = cleaning_robot.getRobotInfo("cleaning_gadget");
    Json::Value robot = cleaning_robot.getRobotInfo("robot");

    // calculating the total_distance and printing out
    double total_distance = cleaning_robot.calculatePath(robot_path);
    std::cout << "The total_distance is: " << total_distance << std::endl;

    //Calculate the cleaning Area 
    //ouble cleaning_area = cleaning_robot.calculateCleaningArea(robot, robot_cleaning_gadget, robot_path);
    //std::cout << "CleaningArea: " << cleaning_area << std::endl;

    ros::spin();
}