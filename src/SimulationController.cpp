/*
 * SimulationController.cpp
 *
 *  Created on: Apr 14, 2020
 *      Author: antonio
 */

#include "SimulationController.h"


SimulationController::SimulationController(double maxSTime, int tRate) :
	linear_{0},
	angular_{0},
	l_scale_{1.0},
	a_scale_{1.0},
	maxSimTime{maxSTime},
	ticsRate{tRate},
	actuatorValues{NUM_ACTUATORS, 0.0}
{
	sensorValues.resize(NUM_RAYS * NUM_SONARS);

   prev_x = 0;
   prev_y = 0;
   stuck = false;
   stuckCounter = 0;
	nh_.param("scale_angular", a_scale_, a_scale_);
	nh_.param("scale_linear", l_scale_, l_scale_);

	vel_pub_     = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	odom_sub_    = nh_.subscribe("odom", 1000, &SimulationController::checkModelPosition, this);
	clock_sub_   = nh_.subscribe("clock", 1000, &SimulationController::checkSimulationTime, this);
   sonar_l_sub_ = nh_.subscribe("arlo/laser/scan_left", 1000, &SimulationController::checkSonarLeftValues, this);
   sonar_c_sub_ = nh_.subscribe("arlo/laser/scan_center", 1000, &SimulationController::checkSonarCenterValues, this);
   sonar_r_sub_ = nh_.subscribe("arlo/laser/scan_right", 1000, &SimulationController::checkSonarRightValues, this);
   //actuatorValues_sub = nh_.subscribe("gp_test",1000,&SimulationController::actuatorCallback, this);
	service = nh_.advertiseService("evaluate_driver", &SimulationController::evaluateDriver, this);
}

void SimulationController::actuatorCallback(const std_msgs::Float32MultiArray::ConstPtr& array)
{

	int i = 0;
	// print all the remaining numbers
    ROS_INFO("I heard: [");

	for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
	{
		actuatorValues[i] = (float) *it;
    ROS_INFO("%lf, ", actuatorValues[i]);
		cout << actuatorValues[i]<<endl;
		i++;
	}
  ROS_INFO("]\n");

	return;
}

SimulationController::~SimulationController() {
}

void SimulationController::setDriver(ArloDriver* driver) {
	aDriver = driver;
}


bool SimulationController::evaluateDriver(arlo_nn_controller::EvaluateDriver::Request  &req,
		                                    arlo_nn_controller::EvaluateDriver::Response &res)
{
   //TODO: Revisar que Gazebo este corriendo

   startSimulation(req.maxtime);

	res.time = arloState.finishTime;
   res.dist2go = arloState.distanceToGo;
	res.damage = arloState.robotDamage ;
	res.energy = arloState.distanceTravelled;

   // res.time = 1;
   // res.dist2go = 2;
	// res.damage = 3 ;
	// res.energy = 4;

	return true;
}


SimulationState SimulationController::startSimulation(int maxtime)
{
   //string pesos(req.weightsfile);
   //aDriver->setParameters(pesos.c_str());

   puts("Starting the simulation of a new driver...");
	puts("---------------------------");

   std_srvs::Empty gazeboParams;
   ros::service::call("/gazebo/reset_simulation", gazeboParams);
	maxSimTime = maxtime;

	ros::Rate loop_rate(50); // Frecuencia Hz con la que le robot debe tomar una decisión.

	linear_ = angular_ = 0;

   arloState.resetState();
   stuckCounter = 0;

	while (ros::ok() && !arloState.hasTimeRunOut && !arloState.finishLineCrossed)
	{
		
		// Send sensor values to the driver and get its answer to move the robot.
      
		//aDriver->driveArlo(sensorValues, actuatorValues);
      cout << "lineal= " << actuatorValues[0]
          << ", angular= " << actuatorValues[1] << "\n" << endl;
      //publish sensorValues
			

		linear_  = actuatorValues[0];
		angular_ = actuatorValues[1];

		// Set values in the twist object to send the actuator values to the robot in Gazebo.
		geometry_msgs::Twist twist;
		twist.angular.z = a_scale_*angular_;
		twist.linear.x = l_scale_*linear_;
		vel_pub_.publish(twist); // Publish the event for the twist plugin.

		ros::spinOnce();
		loop_rate.sleep();
	}

   cout << "hasTimeRunOut= " << arloState.hasTimeRunOut << "\n";
   cout << "finishLineCrossed= " << arloState.finishLineCrossed << "\n";

	if (arloState.hasTimeRunOut == true) {
      arloState.finishTime = 2*maxSimTime;
      //arloState.distanceToGo = goalDistance - arloState.currentPosition;
      cout << "currentPosition= " << arloState.currentPosition << "\n";
      if (arloState.stuck == true) {
         cout << " ---->>> ATASCADO  <<<-----" << endl;
         cout << " \t Counter = " << stuckCounter << endl;
      }
   }
	else { // The robot reached the goal.
      arloState.finishTime = arloState.currentTime;
      //arloState.distanceToGo = 0.0;
      arloState.robotEnergy = 100;
      cout << "finishTime= " << arloState.finishTime << "\n";
   }

   cout << "x = " << arloState.position[0] << ", y = " << arloState.position[1] << endl;
   cout << "d2Go= " << arloState.distanceToGo << endl;
   cout << "gas= " << arloState.distanceTravelled << endl;

//	res.time = arloState.finishTime;
//   res.dist2go = arloState.distanceToGo;
//	res.damage = arloState.robotDamage ;
//	res.energy = arloState.distanceTravelled;

   ros::service::call("/gazebo/reset_simulation", gazeboParams);
	std::cout << "Bye" << std::endl;

	return arloState;
}

SimulationState SimulationController::startSimulation(ArloDriver* aDriver, int maxtime) {
   //string pesos(req.weightsfile);
   //aDriver->setParameters(pesos.c_str());

	puts("Starting the simulation of a new driver...");
	puts("---------------------------");

   std_srvs::Empty gazeboParams;
   ros::service::call("/gazebo/reset_simulation", gazeboParams);
	maxSimTime = maxtime;

	ros::Rate loop_rate(50); // Frecuencia Hz con la que le robot debe tomar una decisión.

	linear_ = angular_ = 0;

   arloState.resetState();
   stuckCounter = 0;

	while (ros::ok() && !arloState.hasTimeRunOut && !arloState.finishLineCrossed)
	{
		ros::spinOnce();
		// Send sensor values to the driver and get its answer to move the robot.
      
		//aDriver->driveArlo(sensorValues, actuatorValues);
      cout << "lineal= " << actuatorValues[0]
          << ", angular= " << actuatorValues[1] << "\n" << endl;
			

		linear_  = actuatorValues[0];
		angular_ = actuatorValues[1];

		// Set values in the twist object to send the actuator values to the robot in Gazebo.
		geometry_msgs::Twist twist;
		twist.angular.z = a_scale_*angular_;
		twist.linear.x = l_scale_*linear_;
		vel_pub_.publish(twist); // Publish the event for the twist plugin.

		
		loop_rate.sleep();
	}

   cout << "hasTimeRunOut= " << arloState.hasTimeRunOut << "\n";
   cout << "finishLineCrossed= " << arloState.finishLineCrossed << "\n";

	if (arloState.hasTimeRunOut == true) {
      arloState.finishTime = 2*maxSimTime;
      //arloState.distanceToGo = goalDistance - arloState.currentPosition;
      cout << "currentPosition= " << arloState.currentPosition << "\n";
      if (arloState.stuck == true) {
         cout << " ---->>> ATASCADO  <<<-----" << endl;
         cout << " \t Counter = " << stuckCounter << endl;
      }
   }
	else { // The robot reached the goal.
      arloState.finishTime = arloState.currentTime;
      //arloState.distanceToGo = 0.0;
      arloState.robotEnergy = 100;
      cout << "finishTime= " << arloState.finishTime << "\n";
   }

   cout << "x = " << arloState.position[0] << ", y = " << arloState.position[1] << endl;
   cout << "d2Go= " << arloState.distanceToGo << endl;
   cout << "gas= " << arloState.distanceTravelled << endl;

//	res.time = arloState.finishTime;
//   res.dist2go = arloState.distanceToGo;
//	res.damage = arloState.robotDamage ;
//	res.energy = arloState.distanceTravelled;

   ros::service::call("/gazebo/reset_simulation", gazeboParams);
	std::cout << "Bye" << std::endl;

	return arloState;
}

void SimulationController::checkSonarLeftValues(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	//ROS_INFO("Tamano ranges left= %lu", msg->ranges.size());

	for (int i = 0; i < msg->ranges.size(); ++i) {
		sensorValues[i + 0*NUM_RAYS] = msg->ranges[i];  // 0 para el sensor izq.
	}
}

void SimulationController::checkSonarCenterValues(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	//ROS_INFO("Tamano ranges center= %lu", msg->ranges.size());

	for (int i = 0; i < msg->ranges.size(); ++i) {
		sensorValues[i + 1*NUM_RAYS] = msg->ranges[i]; // 1 para el sensor central.
	}
}

void SimulationController::checkSonarRightValues(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	//ROS_INFO("Tamano ranges right= %lu", msg->ranges.size());

	for (int i = 0; i < msg->ranges.size(); ++i) {
		sensorValues[i + 2*NUM_RAYS] = msg->ranges[i]; // 2 para el sensor der.
	}
}

double SimulationController::dist2Go(double x, double y) {
   double distToGo;
   if ( y < 0.8 && x < 4.7) { // Va en la primera recta
         distToGo = 18 - x;
   }
   else if (y < 6.61) {  // Va en la segunda recta
      distToGo = 18 - (5.25 + y);
   }
   else {   // Va en la recta final
      if (x > 0.0)
         distToGo = x;
      else
         distToGo = 0.0;
   }
   return distToGo;
}

double SimulationController::distance(double x1, double y1, double x2, double y2) {
   double sum = (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2);
   return sqrt(sum);
}

void SimulationController::checkModelPosition(const nav_msgs::Odometry::ConstPtr &msg)
{
	//ROS_INFO("Seq: [%d]", msg->header.seq);
	//ROS_INFO("Position-> x: [%f]", msg->pose.pose.position.x);
	//ROS_INFO("Vel-> Linear: [%f], Angular: [%f]",
	//		msg->twist.twist.linear.x,
	//		msg->twist.twist.angular.z);

   // arloState.currentPosition = msg->pose.pose.position.x;
	// if (arloState.currentPosition >= goalDistance)   // Esta es la distancia del pasillo en Gazebo.
	// 	arloState.finishLineCrossed = true;

   double distanceBefore = arloState.distanceTravelled;
   arloState.distanceTravelled += distance(prev_x, prev_y, msg->pose.pose.position.x, msg->pose.pose.position.y);

   if (abs(distanceBefore-arloState.distanceTravelled) < 0.01) {
      stuckCounter++;
      //cout << "Stuck counter: " << stuckCounter << "\n";
      if (stuckCounter > 80) {
         arloState.stuck = true;
         arloState.hasTimeRunOut = true;
      }
   }
   else
      stuckCounter = 0;

   prev_x = msg->pose.pose.position.x;
   prev_y = msg->pose.pose.position.y;

   arloState.currentPosition = msg->pose.pose.position.x;
   arloState.position[0] = msg->pose.pose.position.x;
   arloState.position[1] = msg->pose.pose.position.y;
   arloState.distanceToGo = dist2Go(msg->pose.pose.position.x, msg->pose.pose.position.y);
	if (arloState.distanceToGo <= 0.0)   // Esta es la distancia del pasillo en Gazebo.
		arloState.finishLineCrossed = true;
}

void SimulationController::checkSimulationTime(const rosgraph_msgs::Clock::ConstPtr &msg)
{
	arloState.currentTime = msg->clock.toSec();
	//std::cout << "Tiempo simulacion: " <<  msg->clock << std::endl;

	if (arloState.currentTime >= maxSimTime) {
		arloState.hasTimeRunOut = true;
	}
}

int SimulationController::getNumSensors() {
	return NUM_RAYS * NUM_SONARS;
}

int SimulationController::getNumActuators() {
	return NUM_ACTUATORS;
}
