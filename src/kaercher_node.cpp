/*
name: Markus Joos
Date: 18.06.2023
*/

#include "cleaning_robot.h"

// reading the jsonfile, which is needed

int main(int argc, char** argv){
    ros::init(argc, argv, "kaercher");

    ros::NodeHandle private_nh("~");

    cleaning_robot cleaning_robot;
    std::string jsonfile;
    Json::Value robot_info;

    // Check if json available?
    if(private_nh.getParam("jsonfile", jsonfile)){
        ROS_INFO("Read Json-File ");
        robot_info = cleaning_robot.readJsonFile(jsonfile);
    }
    else{
        ROS_ERROR("JSON-File is not available");
        return -1;
    }
    
    // getting the information from json-file
    Json::Value robot_path = robot_info["path"];
    Json::Value robot_cleaning_gadget = robot_info["cleaning_gadget"];

    // Checking if the Json-File is empty
    if (robot_cleaning_gadget.empty() or robot_path.empty())
    {
        return -1;
    }

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
    
    return 0;
}