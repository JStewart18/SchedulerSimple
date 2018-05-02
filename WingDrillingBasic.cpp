
#include <Eigen/Core>
#include <vector>
#include <iostream>
#include <fstream>
#include <istream>
#include "WingDrillingBasic.h"


const double WINGLENGTH = 80;		//Length of the wing from root to tip in feet
const double WINGWIDTH = 20;		//Width of the wing root in feet
const int ROBOTCOUNT = 4;
//const int TIMEGRADIENTROOT = 30;	//Length of time to drill a hole at a node location closest to the root of the wing
//const int TIMEGRADIENTTIP = 10;		//Length of time to drill a hole at a node location closest to the tip of the wing
const int HOLESPACING = 1 / 6;			//Spacing between the holes drilled into a given rib in feet
const int RIBSPACING = 2;			//Spacing between the ribs of the wing in feet

double distance(node node1, node node2) {
	return sqrt(pow((node1.x - node2.x), 2) + pow((node1.y - node2.y), 2)); //uses basic pathagorean dist formula for direct line
}

//Sets up th elist of holes in a matrix format, may need ot be changed later
/*Eigen::Matrix<node, 80, Eigen::Dynamic> readCSV() {
	std::ifstream holeList;
	holeList.open("input.csv", std::ifstream::in);

}*/

std::vector<int> allocateRegions(std::vector<botArm> robots, Eigen::Matrix<node, 80, Eigen::Dynamic> nodeGrid) {
	double span = WINGLENGTH / RIBSPACING * (30 + 10) / 2;				//Assuming linearly changing hole drill time between 30 and 10 seconds, states the span of time needed to drill down the length of the wing
	double splitSpan = 0;													//Span for the next split, where the zone assigned to the current pair of robots will end and the next begin. 
	int zoneDivider = 0;												//Specifies the rib boundaries at which one Robot's workspace becomes another's
	std::vector<int> splitIntervals;									//The locations at which x-axis splits occur
	for (int i = 0; i < WINGLENGTH / RIBSPACING; i++) {					//will eventually change to process multiple splits	if this approach is judged appropriate
		span = nodeGrid.coeff(i, 1).drillTime;
		if (splitSpan < span / (ROBOTCOUNT / 2)) {							//Division by 2 assumes there are equal an number of robots on either side of the wing
			splitIntervals[zoneDivider]++;
			splitSpan += nodeGrid.coeff(splitIntervals[zoneDivider], 1).drillTime;		//why splitspan for coord?
		}
		if (splitSpan == span / (ROBOTCOUNT / 2)) {						//If workspace is of the appripriate size, begin sizing the next
			zoneDivider++;
		}
	}
	return splitIntervals;												//This is the interval after which the scheduler will start assigning to the next robot, will change with if above does
}

/*double drillTime(botArm driller) {
if (distance(driller.baseLocation, *driller.position) <= driller.reach) {
driller.position -> wasDrilled = true;
//return (*driller.position).x / WINGLENGTH*(TIMEGRADIENTROOT - TIMEGRADIENTTIP) + TIMEGRADIENTTIP;
return (*driller.position).x
} else {

}

}*/

//Sets the schedule for each arm based upon the CSV file
Eigen::Matrix<node, ROBOTCOUNT, Eigen::Dynamic> scheduleRobots(std::vector<int> splitIntervals) {
	Eigen::Matrix<node, ROBOTCOUNT, Eigen::Dynamic> scheduleByRobot;
	node nextHole;
	std::string line;
	int positionY = 0;
	int robotNumber = 0;
	std::ifstream holeSpecList;
	holeSpecList.open("input.csv", std::ifstream::in);
	//holeSpecList.getline(line, 256, ',') && 
	int	j = 1;										//Will be the interator splitIntervals once more than 1 split exists
	while (holeSpecList.good()) {
		holeSpecList >> nextHole.x, nextHole.y, nextHole.drillTime;			//verify how this interacts with .csv format

																			/*nextHole.x = line;
																			line = holeSpecList.getline(holeSpecList, line, ',');					//verify the correct stuff is being input as "line"
																			nextHole.y = line;
																			holeSpecList.getline(holeSpecList, line, ',');
																			nextHole.drillTime = line;
																			holeSpecList.getline(holeSpecList, line, ',');*/

		if (nextHole.y > WINGWIDTH / 2) {
			positionY = 1; //Top side of wing
		}
		if (positionY == 1) {									//Will obviously need updating for when there are more than two robots per side of the wing
			if (nextHole.x < splitIntervals[j]) {
				robotNumber = 1;
			}
			else {
				robotNumber = 3;
			}
		}
		else {
			if (nextHole.x < splitIntervals[j]) {
				robotNumber = 2;
			}
			else {
				robotNumber = 4;
			}
		}
		/*if (robotNumber == 1) {
		robotNumber = 1;
		} else if (robotNumber == 2){
		robotNumber = 3;
		} else if (nextHole.x < splitInterval) {
		robotNumber = 1;
		} else {
		robotNumber = 3;
		}*/
		//hole = holeSpecList.get();


		scheduleByRobot(robotNumber, Eigen::Dynamic) = nextHole;			//assigns the hole to the appropriate robot -- does this work?
	}
	std::cout << scheduleByRobot;
}


std::vector<botArm> establishRobotLocations() {
	std::vector<botArm> robots(ROBOTCOUNT);
	for (int i = 0; i < ROBOTCOUNT; i++) {
		if (i % 2 == 1) {
			robots[i].position->x = 1 + WINGLENGTH / ROBOTCOUNT * i;		//Robots are falsely assumed to be evenly placed along the wing for simplicity's sake
			robots[i + 1].position->x = 1 + WINGLENGTH / ROBOTCOUNT * i;
			robots[i].position->y = 1;										//starts in corner
			robots[i + 1].position->y = 1 + WINGWIDTH;						//starts in same corner, next quadrant
			robots[i].startingPosition = robots[i].position;				//Arm starts at the starting position, which is tracked to be the starting position for every wing
			robots[i + 1].startingPosition = robots[i].position;
			robots[i].baseLocation = *robots[i].position;					//incorrect because this puts the bot on the wing, but done for now for simplicity's sake
			robots[i + 1].baseLocation = *robots[i].position;
		}
	}
	return robots;
}

int main() {
	std::ofstream outputSchedule;
	//std::ifstream holeSpecList;
	
	std::vector<int> horizontalBarriers;

	std::vector<botArm> robots = establishRobotLocations();
	Eigen::Matrix<node, 80, Eigen::Dynamic> nodeGrid;
	//nodeGrid = readCSV();
	horizontalBarriers = allocateRegions(robots, nodeGrid);
	int verticialBarrier = 4;//change to go off csv list
	establishRobotLocations();
	outputSchedule.open("Scheduler_basicVers.txt");
	outputSchedule << scheduleRobots(horizontalBarriers);
	outputSchedule.close();
	return 0;
}


