#include "Example1.h"
#include <iostream>

int main()
{
	// Round outputs to console.
	std::cout.precision(4);

	// Run example simulation.
	MotionPlanner::Example1 example;
	example.runSimulation();
	example.displayPlan();

	return 0;
}
