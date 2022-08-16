#pragma once

#include <vector>
#include "Sphere.h"
#include "Capsule.h"
#include "Box.h"

class CollisionAggregate
{
private:
	std::vector<Sphere> m_spheres;
	std::vector<Capsule> m_capsules;
	std::vector<Box> m_boxes;

public:
	// Default constructor.
	CollisionAggregate();

	// Add shapes.
	void addShape(const Sphere& sphere);
	void addShape(const Capsule& capsule);
	void addShape(const Box& box);

	// Get shapes.
	std::vector<Sphere> getSpheres() const;
	std::vector<Capsule> getCapsules() const;
	std::vector<Box> getBoxes() const;

};
