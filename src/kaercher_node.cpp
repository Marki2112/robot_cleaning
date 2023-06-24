/*
name: Markus Joos
Date: 18.06.2023
*/

#include "cleaning_robot.h"

// reading the jsonfile, which is needed

int main(int argc, char** argv){
    ros::init(argc, argv, "kaercher");
    cleaning_robot cleaning_robot;

    // getting the information from json-file
    Json::Value robot_path = cleaning_robot.getRobotInfo("path");
    Json::Value robot_cleaning_gadget = cleaning_robot.getRobotInfo("cleaning_gadget");

    // calculating the total_distance and printing out
    std::vector<double> distance = cleaning_robot.calculateDistance(robot_path);
    double total_distance = cleaning_robot.calculatePath(distance);
    std::cout << "The total_distance is: " << total_distance << std::endl;

    //Calculate the cleaning Area 
    double cleaning_area = cleaning_robot.calculateCleaningArea(robot_cleaning_gadget, distance);
    std::cout << "CleaningArea: " << cleaning_area << std::endl;

    // Calculate the time for the complete path depends on the velocity from the robot
    double total_time = cleaning_robot.calculateTimeOfPath(distance);
    std::cout << "The total travel time is: " << total_time << std::endl;

    ros::spin();
}