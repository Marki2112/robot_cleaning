#include <iostream>
#include <fstream>
#include <jsoncpp/json/json.h>
#include <string>
#include <cmath>
#include <vector>

#include "ros/ros.h"
#include <ros/console.h>

class cleaning_robot{

    public:
        cleaning_robot();
        ~cleaning_robot();


    private:
        std::string jsonfile;
        Json::Value robot_info;
        Json::Value readJsonFIle(std::string jsonfile);
        double calculateDistance(double x1, double y1, double x2, double y2);
        double calculatePath(std::vector<double>& x_points, std::vector<double>& y_points);
        double calculateVelocity(float Kappa);
        double calculateCleaningArea(double lengthCleaningGadget, double path);
        double calculateTimeOfPath();
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

Json::Value cleaning_robot::readJsonFIle(std::string jsonfile){
    std::ifstream robot_file(jsonfile);
    Json::Value robot_information;
    Json::Reader reader;
    reader.parse(robot_file, robot_information);
    return robot_information;
}

double cleaning_robot::calculateDistance(double x1, double y1, double x2, double y2){
    return std::sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
} 
// Changing Datetype to make it more flexible
double cleaning_robot::calculatePath(std::vector<double>& x_points, std::vector<double>& y_points){
    double total_distance = 0.0;

    for(size_t i = 0; i < x_points.size() - 1; i++){
        double x1 = x_points[i];
        double y1 = y_points[i];
        double x2 = x_points[i + 1];
        double y2 = y_points[i + 1];

        total_distance = calculateDistance(x1, y1, x2, y2);
        total_distance += total_distance;
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

double cleaning_robot::calculateCleaningArea(double lengthCleaningGadget, double path){
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

int main(int argc, char** argv){
    ros::init(argc, argv, "kaercher");
    cleaning_robot cleaning_robot;

    ros::spin();
}