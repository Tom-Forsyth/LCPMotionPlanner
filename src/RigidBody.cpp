#include "RigidBody.h"
#include <Eigen/Dense>
#include "CollisionAggregate.h"
#include "Sphere.h"
#include "Capsule.h"
#include "Box.h"
#include <vector>
#include <iostream>

// Constructor.
RigidBody::RigidBody(const Joint& joint, const Eigen::Matrix4d& referenceSpatialTransform)
	: m_joint(joint), m_referenceSpatialTransform(referenceSpatialTransform), m_collisionAggregate(CollisionAggregate()),
	  m_currentSpatialTransform(Eigen::Matrix4d::Identity()), m_currentWorldTransform(Eigen::Matrix4d::Identity()) { }

// Add sphere collider.
void RigidBody::addCollider(const Sphere& sphere)
{
	m_collisionAggregate.addShape(sphere);
}


// Add capsule collider.
void RigidBody::addCollider(const Capsule& capsule)
{
	m_collisionAggregate.addShape(capsule);
}

// Add box collider.
void RigidBody::addCollider(const Box& box)
{
	m_collisionAggregate.addShape(box);
}

// Get relative transformation from the joint.
Eigen::Matrix4d RigidBody::getRelativeTransformation() const
{
	return m_joint.getRelativeTransformation();
}

// Get reference spatial transform.
Eigen::Matrix4d RigidBody::getReferenceSpatialTransform() const
{
	return m_referenceSpatialTransform;
}

// Set current spatial transform.
void RigidBody::setCurrentSpatialTransform(const Eigen::Matrix4d& spatialTransform)
{
	m_currentSpatialTransform = spatialTransform;
}

// Set current world transform.
void RigidBody::setCurrentWorldTransform(const Eigen::Matrix4d& worldTransform)
{
	m_currentWorldTransform = worldTransform;
}

// Set joint displacement.
void RigidBody::setJointDisplacement(const double& displacement)
{
	m_joint.setDisplacement(displacement);
}

// Get reference to collision aggregate.
const CollisionAggregate& RigidBody::getCollisionAggregate() const
{
	return m_collisionAggregate;
}

// Get current world transform.
Eigen::Matrix4d RigidBody::getCurrentWorldTransform() const
{
	return m_currentWorldTransform;
}

// Get joint type.
Joint::Type RigidBody::getJointType() const
{
	return m_joint.getType();
}

// Take closest contact in collision aggregate as the body's contact.
void RigidBody::condenseContacts()
{
	// Get colliders in aggregate.
	std::vector<Sphere> spheres = m_collisionAggregate.getSpheres();
	std::vector<Capsule> capsules = m_collisionAggregate.getCapsules();
	std::vector<Box> boxes = m_collisionAggregate.getBoxes();

	// Get all the distances of the colliders.
	double defaultDist = 1000;
	size_t collisionCount = 0;

	double minSphereDist = defaultDist;
	size_t minSphereIndex = -1;
	size_t sphereIndex = 0;
	for (const Sphere& sphere : spheres)
	{
		if (sphere.m_contactPoint->m_isActive)
		{
			collisionCount++;
			double dist = sphere.m_contactPoint->m_distance;
			if (dist < minSphereDist)
			{
				minSphereDist = dist;
				minSphereIndex = sphereIndex;
			}
		}
		sphereIndex++;
	}

	double minCapsuleDist = defaultDist;
	size_t minCapsuleIndex = -1;
	size_t capsuleIndex = 0;
	for (const Capsule& capsule : capsules)
	{
		if (capsule.m_contactPoint->m_isActive)
		{
			collisionCount++;
			double dist = capsule.m_contactPoint->m_distance;
			if (dist < minCapsuleDist)
			{
				minCapsuleDist = dist;
				minCapsuleIndex = capsuleIndex;
			}
		}
		capsuleIndex++;
	}

	double minBoxDist = defaultDist;
	size_t minBoxIndex = -1;
	size_t boxIndex = 0;
	for (const Box& box : boxes)
	{
		if (box.m_contactPoint->m_isActive)
		{
			collisionCount++;
			double dist = box.m_contactPoint->m_distance;
			if (dist < minBoxDist)
			{
				minBoxDist = dist;
				minBoxIndex = boxIndex;
			}
		}
		boxIndex++;
	}

	// Find min of everything.
	if (collisionCount > 0)
	{
		m_contactPoint.m_isActive = true;
		if ((minSphereDist < minCapsuleDist) && (minSphereDist < minBoxDist))
		{
			m_contactPoint.m_distance = spheres[minSphereIndex].m_contactPoint->m_distance;
			m_contactPoint.m_normal = spheres[minSphereIndex].m_contactPoint->m_normal;
			m_contactPoint.m_point = spheres[minSphereIndex].m_contactPoint->m_point;
		}
		else if ((minCapsuleDist < minSphereDist) && (minCapsuleDist < minBoxDist))
		{
			m_contactPoint.m_distance = capsules[minCapsuleIndex].m_contactPoint->m_distance;
			m_contactPoint.m_normal = capsules[minCapsuleIndex].m_contactPoint->m_normal;
			m_contactPoint.m_point = capsules[minCapsuleIndex].m_contactPoint->m_point;
		}
		else
		{
			m_contactPoint.m_distance = boxes[minBoxIndex].m_contactPoint->m_distance;
			m_contactPoint.m_normal = boxes[minBoxIndex].m_contactPoint->m_normal;
			m_contactPoint.m_point = boxes[minBoxIndex].m_contactPoint->m_point;
		}
	}
	else
	{
		m_contactPoint.m_isActive = false;
	}

}

// Get contact point.
const ContactPoint& RigidBody::getContactPoint() const
{
	return m_contactPoint;
}
