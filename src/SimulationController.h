/*
 * SimulationController.h
 *
 *  Created on: Apr 14, 2020
 *      Author: antonio
 */

#ifndef SRC_SIMULATIONCONTROLLER_H_
#define SRC_SIMULATIONCONTROLLER_H_

#include "SimulationState.h"
#include "ArloDriver.h"
#include "NeuroControllerDriver.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Empty.h>
#include "arlo_nn_controller/EvaluateDriver.h"
#include "arlo_gp_controller/EvaluateTree.h"
#include <string>
#include "std_msgs/Float32MultiArray.h"
#include <iostream>

#define NUM_RAYS 32
#define NUM_SONARS 3
#define NUM_ACTUATORS 2  // Velocity linear and angular

using namespace std;


class SimulationController {
public:
	SimulationController(double maxSTime = 300, int tRate = 1);
	virtual ~SimulationController();
	void setDriver(ArloDriver* driver);
	SimulationState startSimulation(int maxtime, int treeIndex);
	SimulationState startSimulation(ArloDriver* driver, int maxtime);
	void checkSonarLeftValues(const sensor_msgs::LaserScan::ConstPtr& msg);
   void checkSonarCenterValues(const sensor_msgs::LaserScan::ConstPtr& msg);
   void checkSonarRightValues(const sensor_msgs::LaserScan::ConstPtr& msg);
	void checkModelPosition(const nav_msgs::Odometry::ConstPtr& msg);
	void checkSimulationTime(const rosgraph_msgs::Clock::ConstPtr& msg);
	int getNumSensors();
	int getNumActuators();
	void actuatorCallback(const std_msgs::Float32MultiArray::ConstPtr& array);
	bool evaluateDriver(arlo_nn_controller::EvaluateDriver::Request  &req,
			            arlo_nn_controller::EvaluateDriver::Response &res);

private:
   double dist2Go(double x, double y);
   double distance(double x1, double y1, double x2, double y2);
   SimulationState arloState;
   double prev_x, prev_y;
   long int stuckCounter;
   bool stuck;
   string inputFile;
	string outputFile;
	double maxSimTime;  /* Maximum time allowed for the robot to get the goal */
   double goalDistance;


	ArloDriver* aDriver;
	int ticsRate;     /* How often the driver is ask for a decision */
	double linear_;   /* Linear velocity to send to the robot */
	double angular_;  /* Angular velocity to send to the robot */
	double l_scale_;  /* Factor to the scale the linear velocity value */
	double a_scale_;  /* Factor to the scale the angular velocity value */
	ros::NodeHandle nh_;
	ros::Publisher vel_pub_;
	ros::Subscriber odom_sub_;
	ros::Subscriber clock_sub_;
	ros::Subscriber sonar_l_sub_;
  ros::Subscriber sonar_c_sub_;
  ros::Subscriber sonar_r_sub_;
	ros::Subscriber actuatorValues_sub;
	ros::ServiceServer service;
	ros::ServiceClient actuatorClient;
	vector<double> actuatorValues;
	vector<double> sensorValues;
	//double currentTime;
	//bool hasTimeRunOut;
	//bool finishLineCrossed;
};

#endif /* SRC_SIMULATIONCONTROLLER_H_ */
