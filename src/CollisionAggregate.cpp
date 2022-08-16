#include "CollisionAggregate.h"
#include <vector>
#include "Sphere.h"
#include "Capsule.h"
#include "Box.h"

// Default constructor.
CollisionAggregate::CollisionAggregate()
	: m_spheres(std::vector<Sphere> {}), m_capsules(std::vector<Capsule> {}), m_boxes(std::vector<Box> {}) { }


// Add sphere.
void CollisionAggregate::addShape(const Sphere& sphere)
{
	m_spheres.emplace_back(sphere);
}

// Add capsule.
void CollisionAggregate::addShape(const Capsule& capsule)
{
	m_capsules.emplace_back(capsule);
}

// Add box.
void CollisionAggregate::addShape(const Box& box)
{
	m_boxes.emplace_back(box);
}

// Get spheres.
std::vector<Sphere> CollisionAggregate::getSpheres() const
{
	return m_spheres;
}

// Get capsules.
std::vector<Capsule> CollisionAggregate::getCapsules() const
{
	return m_capsules;
}

// Get boxes.
std::vector<Box> CollisionAggregate::getBoxes() const
{
	return m_boxes;
}
