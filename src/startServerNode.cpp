/*
 * startNode.cpp
 *
 *  Created on: Apr 15, 2020
 *      Author: antonio
 */

#include "SimulationController.h"
#include "ArloDriver.h"
#include "NeuroControllerDriver.h"
#include <ros/ros.h>
#include <vector>
#include <utility>
#include <iostream>

int main(int argc, char **argv)
{
   ros::init(argc, argv, "neurocontroller_node");


   SimulationController sim;

   int numRays = sim.getNumSensors();
   int numActuators = sim.getNumActuators();

   vector<pair<double, double> > outputRanges;
   outputRanges.push_back( make_pair(-0.25, 1) );
   outputRanges.push_back( make_pair(-0.5, 0.5) );

   // ArloDriver *nc1 = new NeuroControllerDriver(numRays,  numActuators, outputRanges);
   // ArloDriver *nc2 = new NeuroControllerDriver(numRays,  numActuators, outputRanges);
   // ArloDriver *nc3 = new NeuroControllerDriver(numRays,  numActuators, outputRanges);


   // sim.setDrivers(nc1, nc2, nc3);

   //sim.startSimulation(nc);
   ROS_INFO("Ready to evaluate Xolobot Drivers...");
   ros::spin();

   return(0);
}
