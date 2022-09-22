#include "RigidBody.h"
#include <Eigen/Dense>
#include "CollisionAggregate.h"
#include "Sphere.h"
#include "Capsule.h"
#include "Box.h"
#include <vector>
#include "Kinematics.h"

namespace CollisionAvoidance
{
	// Constructor.
	RigidBody::RigidBody(const Joint& joint, const Eigen::Matrix4d& referenceSpatialTransform)
		: m_joint(joint), m_referenceSpatialTransform(referenceSpatialTransform), m_collisionAggregate(CollisionAggregate()),
		m_currentSpatialTransform(Eigen::Matrix4d::Identity()), m_currentWorldTransform(Eigen::Matrix4d::Identity()), m_contactPoint()
	{
		if (joint.getType() == JointType::Fixed)
		{
			m_isMovableBody = false;
		}
		else
		{
			m_isMovableBody = true;
		}
	}

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
	JointType RigidBody::getJointType() const
	{
		return m_joint.getType();
	}

	/*
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
	*/

	// Get contact point.
	const ContactPoint& RigidBody::getContactPoint() const
	{
		return m_contactPoint;
	}

	// Update spatial jacobian.
	void RigidBody::setSpatialJacobian(const Eigen::MatrixXd& spatialJacobian)
	{
		m_spatialJacobian = spatialJacobian;
	}

	// Get joint twist coordinate.
	Eigen::Vector<double, 6> RigidBody::getJointTwistCoord() const
	{
		return m_joint.getTwistCoord();
	}

	// Determine if the body is movable.
	bool RigidBody::isMovable() const
	{
		return m_isMovableBody;
	}

	// Set the contact jacobian.
	void RigidBody::setContactJacobian(const Eigen::MatrixXd& contactJacobian)
	{
		m_contactJacobian = contactJacobian;
	}

	// Get the spatial jacobian.
	Eigen::MatrixXd RigidBody::getSpatialJacobian() const
	{
		return m_spatialJacobian;
	}

	// Get the contact jacobian.
	Eigen::MatrixXd RigidBody::getContactJacobian() const
	{
		return m_contactJacobian;
	}

	// Update the contact jacobian.
	void RigidBody::updateContactJacobian()
	{
		// Extract contact point in world frame and convert to spatial frame.
		const Eigen::Vector3d& contactPointWorld = m_contactPoint.m_point;
		Eigen::Vector4d contactPointWorldHomo{ contactPointWorld[0], contactPointWorld[1], contactPointWorld[2], 1 };
		Eigen::Vector3d contactPointSpatial = (m_currentSpatialTransform * contactPointWorldHomo).head(3);

		// Get analytic jacobian of the contact point (contact jacobian) using the point and this frames spatial jacobian.
		m_contactJacobian = Kinematics::spatialToAnalyticJacobian(m_spatialJacobian, contactPointSpatial);
	}

	// Get the current spatial transform.
	Eigen::Matrix4d RigidBody::getCurrentSpatialTransform() const
	{
		return m_currentSpatialTransform;
	}

	// Get joint displacement.
	double RigidBody::getJointDisplacement() const
	{
		return m_joint.getDisplacement();
	}

	void RigidBody::updateColliderTransforms()
	{
		m_collisionAggregate.updateColliderTransforms(m_currentWorldTransform);
	}
}
