#include "Example1.h"
#include "Example2.h"
#include <iostream>

int main()
{
	// Round outputs to console.
	std::cout.precision(4);

	// Run example simulation.
	//MotionPlanner::Example1 example1;
	//example1.runSimulation();
	//example1.displayPlan();

	MotionPlanner::Example2 example2;
	example2.runSimulation();
	example2.displayPlan();

	return 0;
}
