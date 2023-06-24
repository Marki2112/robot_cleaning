/*
name: Markus Joos
Date: 18.06.2023
*/

#include "kaercher_node.h"

// reading the jsonfile, which is needed
Json::Value cleaning_robot::readJsonFile(std::string jsonfile){
    std::ifstream robot_file(jsonfile);
    Json::Value robot_information;
    Json::Reader reader;
    reader.parse(robot_file, robot_information);
    //checking if the json file has robot, cleaning gadget and path --> if not throw an fail
    return robot_information;
}

// calculating the distance from the robotspaths
std::vector<double> cleaning_robot::calculateDistance(Json::Value robot_path){
    
    std::vector<double> distance_vec;
    // creating vector from the json_file
    std::pair<std::vector<double>, std::vector<double>> points = createVector(robot_path);
    
    std::vector<double> x_points = points.first;
    std::vector<double> y_points = points.second;

    // Calculating the exact path 
    for(size_t i = 0; i < x_points.size() - 1; i++){
        double x1 = x_points[i];
        double y1 = y_points[i];
        double x2 = x_points[i+1];
        double y2 = y_points[i+1];

        double dist = std::sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
        distance_vec.push_back(dist);
    }

    return distance_vec;
}

// Creating from json_file an vector;
std::pair<std::vector<double>, std::vector<double>> cleaning_robot::createVector(Json::Value robot_path){
    
    std::vector<double> x_points, y_points;
    
    for(Json::ArrayIndex i = 0; i < robot_path.size(); i++){
        double x = robot_path[i][0].asDouble();  
        double y = robot_path[i][1].asDouble();
        x_points.push_back(x);
        y_points.push_back(y);
    }

    return std::make_pair(x_points, y_points);
}

// Calculating the total_distance from an distance vector
double cleaning_robot::calculatePath(std::vector<double> distance){
    double total_distance = 0.0;

    for(size_t i = 0; i < distance.size(); i++){
        total_distance += distance[i];
    }
    return total_distance;
}

// Calculating the velocity depends on the curvature of the distance
double cleaning_robot::calculateVelocity(double curvature){

    double velRobot;
    double k_krit = 0.5;                        // Unit 1/m
    double k_max = 10.0;                        // Unit 1/m
    double vmin = 0.15;                         // Unit m/s
    double vmax = 1.1;                          // Unit m/s

    // Check if kappa bigger then kappa_max 
    // For Debug Purpose
    /*bool test1 = curvature <= k_krit;
    bool test2 = 0.5 <= curvature && curvature < 10.0;
    bool test3 = k_max <= curvature;*/

    /*
    Kappa is be calculated from the path --> Curvature kappa 1/distance approximately
    possile three velocity outcomes:
    1. straights and flat curves            --> max Velocity curvature --> k_krit
    2. kappa is given                       --> calculate the velocity 
    3. kappa is higher then kappa_max       --> min. Velocity --> k_krit <= curvature && curvature < k_max
    */  

    if(curvature <= k_krit){
        velRobot = vmax;
    }
    else if(k_max <= curvature){
        velRobot = vmin;
    }
    else if(k_krit <= curvature && curvature < k_max){
        velRobot = vmax - ((vmax-vmin)/(k_max - k_krit))*(curvature-k_krit);
    }

    //std::cout << curvature << " " << velRobot << " " << test1 << " " << test3 << " " << test2 << std::endl;

    return velRobot;
}

// Function to check if two line segments intersect
bool doLineSegmentsIntersect(const Point& p1, const Point& p2, const Point& p3, const Point& p4) {
    // Code to check for line segment intersection (e.g., using the Cohen-Sutherland algorithm)
    // Return true if the line segments intersect, false otherwise
}

double cleaning_robot::calculateCleaningArea(Json::Value CleaningGadget, std::vector<double> distance){
    
    double resCleaningArea = 0.0;
    double cleaningArea = 0.0;
    double lenghtOfCleanGadget = 0.0;
    double dist = 0.0;
    
    // calculating the lenght of the cleaning device
    lenghtOfCleanGadget = fabs(CleaningGadget[0][1].asDouble()) + fabs(CleaningGadget[1][1].asDouble());
    
    // Calculating the Area of Cleaning    
    for(size_t i = 0; i <= distance.size(); i++)
    {   
        dist = distance[i];
        cleaningArea = lenghtOfCleanGadget * dist;
        std::cout << dist << " " << lenghtOfCleanGadget << " " << cleaningArea << std::endl;
        resCleaningArea += cleaningArea;
    }


    return resCleaningArea;
}

// Need Velocity and Path to get the time
double cleaning_robot::calculateTimeOfPath(std::vector<double> distance){
    double curvature, total_time, time, velocity;

    for(size_t i = 0; i <= distance.size(); i++){
        double dist = distance[i];
        curvature = 1/dist;
        velocity = calculateVelocity(curvature);
        time = dist/velocity;
        total_time += time;
    }

    return total_time;
}

Json::Value cleaning_robot::getRobotInfo(std::string info) const {
    // check if info in the json file (key)
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
    std::vector<double> distance = cleaning_robot.calculateDistance(robot_path);
    double total_distance = cleaning_robot.calculatePath(distance);
    std::cout << "The total_distance is: " << total_distance << std::endl;

    //Calculate the cleaning Area 
    double cleaning_area = cleaning_robot.calculateCleaningArea(robot, robot_cleaning_gadget, distance);
    std::cout << "CleaningArea: " << cleaning_area << std::endl;

    // Calculate the time for the complete path depends on the velocity from the robot
    double total_time = cleaning_robot.calculateTimeOfPath(distance);
    std::cout << "The total travel time is: " << total_time << std::endl;

    ros::spin();
}