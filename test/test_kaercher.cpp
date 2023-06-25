#include "cleaning_robot.h"
#include <string>

int main(){

  
    const double total_dist_test = 3.0;
    const double cleaning_area_test = 1.8;
    const double total_time_test = 2.85714; 

    Json::Value test_clean_gadget;
    cleaning_robot TestCleaningRobot;
    bool calculatePathTest, calculateCleaningAreaTest, calculateTimeOfPathTest; 

    std::vector<double> test_dist = {1.0, 1.0, 1.0};
    
    if((TestCleaningRobot.calculatePath(test_dist) - total_dist_test) < 0.05) 
    {   
        calculatePathTest = true;
    } 
    else{
        calculatePathTest = false;
    }
    std::cout << "the function calculatePath works " << calculatePathTest << std::endl;


    if(test_clean_gadget.empty())
    {
        calculateCleaningAreaTest = false;
    }
    else{
        if((TestCleaningRobot.calculateCleaningArea(test_clean_gadget, test_dist) - total_dist_test) < 0.05) 
        {   
            calculateCleaningAreaTest = true;
        } 
        else{
            calculateCleaningAreaTest = false;
        }
    }
    
    std::cout << "the function calculateCleaningArea works " << calculateCleaningAreaTest << std::endl;

     if((TestCleaningRobot.calculateTimeOfPath(test_dist) - total_time_test) < 0.05) 
    {   
        calculateTimeOfPathTest = true;
    } 
    else{
        calculateTimeOfPathTest = false;
    }
    std::cout << "the function calculateTimeOfPath works " << calculateTimeOfPathTest << std::endl;

    if(calculateCleaningAreaTest && calculatePathTest && calculateTimeOfPathTest){
        return 0;
    }
    else{
        return -1;
    }
}