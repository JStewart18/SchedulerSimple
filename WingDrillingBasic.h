#pragma once
#include <Eigen/Core>
#include <vector>
#include <iostream>
#include <fstream>
#include <istream>

extern const double WINGLENGTH;		//The length of the wing in feet
extern const double WINGWIDTH;		//The width of the wing in feet
extern const int ROBOTCOUNT;
extern const int HOLESPACING;		
extern const int RIBSPACING;

struct node {
	double x;							//Indicates the x and y positions of the given location in a grid, 
	double y;							//this grid is not the same as the wing and includes the robot locations
	bool wasDrilled;				//Indicates whether the node location has already been drilled through
	double drillTime;				//Indicates how long it will take to drill at the node location in seconds
};
struct botArm {
	node baseLocation;				//Node position at which the base of the robot sits
	node* position;					//Node position at which the end of the robot's arm currently is
	double reach;					//The maximum reach distance from base to tip that the robot is capable of drilling at
	node* startingPosition;			//The first node the Robot arm will drill and where it will begin work for the current wing
};


double distance(node node1, node node2);
std::vector<int> allocateRegions(std::vector<botArm> robots, Eigen::Matrix<node, 80, Eigen::Dynamic> nodeGrid);
//double drillTime(botArm driller);
Eigen::Matrix<node, 4, Eigen::Dynamic> scheduleRobots(std::vector<int> splitIntervals);
std::vector<botArm> establishRobotLocations();


