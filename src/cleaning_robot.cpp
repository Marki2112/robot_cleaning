#include "cleaning_robot.h"

Json::Value cleaning_robot::readJsonFile(std::string jsonfile){
    
    Json::Value robot_information;
    Json::Reader reader;
    
    std::ifstream robot_file(jsonfile);
    reader.parse(robot_file, robot_information);

    return robot_information;
}

// Creating from json_file an vector;
std::vector<Point> cleaning_robot::createVector(Json::Value robot_path){
    
    std::vector<Point> waypoints;
    Point point;
    
    for(Json::ArrayIndex i = 0; i < robot_path.size(); i++){
        point.x = robot_path[i][0].asDouble();  
        point.y = robot_path[i][1].asDouble();
        waypoints.push_back(point);
    }

    return waypoints;
}

double cleaning_robot::calculateLength(const Point& start, const Point& end){
    double length; 
    
    length = std::sqrt(pow(end.x - start.x, 2) + pow(end.y - start.y, 2));
    return length;
}

// Calculating the total_distance from an distance vector
double cleaning_robot::calculatePath(std::vector<double> distance){
    double total_distance = 0.0;

    for(size_t i = 0; i < distance.size(); i++){
        total_distance += distance[i];
    }
    return total_distance;
}

// calculating the distance from the robot paths
std::vector<double> cleaning_robot::calculateDistance(Json::Value robot_path){
    
    std::vector<double> distance_vec;

    // creating vector from the json_file
    std::vector<Point> points = createVector(robot_path);

    // Calculating the exact path 
    for(size_t i = 0; i < points.size(); i++){
        //Calculating of the length outside
        Point start = points[i];
        Point end = points[i+1];
        double dist = calculateLength(start, end);
        distance_vec.push_back(dist);
    }

    return distance_vec;
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

double cleaning_robot::calculateCleaningArea(Json::Value CleaningGadget, std::vector<double> distance){
    
    double resCleaningArea = 0.0;
    double cleaningArea = 0.0;
    double lenghtOfCleanGadget = 0.0;
    double dist = 0.0;
    
    // calculating the lenght of the cleaning device
    lenghtOfCleanGadget = fabs(CleaningGadget[0][1].asDouble()) + fabs(CleaningGadget[1][1].asDouble());
    
    // Calculating the Area of Cleaning    
    for(size_t i = 0; i <=distance.size(); i++)
    {   
        dist = distance[i];
        cleaningArea = lenghtOfCleanGadget * dist;
        //std::cout << dist << " " << lenghtOfCleanGadget << " " << cleaningArea << std::endl;
        resCleaningArea += cleaningArea;
    }


    return resCleaningArea;
}

// Need Velocity and Path to get the time
double cleaning_robot::calculateTimeOfPath(std::vector<double> distance){
    double curvature, total_time, time, velocity;

    for(size_t i = 0; i < distance.size(); i++){
        double dist = distance[i];
        curvature = 1/dist;
        velocity = calculateVelocity(curvature);
        time = dist/velocity;
        total_time += time;
    }

    return total_time;
}
