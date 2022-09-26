#pragma once

#include "ObjectType.h"
#include <Eigen/Dense>
#include <string>

namespace CollisionAvoidance
{
	struct ContactPoint;

	enum class ShapeType
	{
		Unassigned,
		Sphere,
		Capsule,
		Box
	};

	class Shape
	{
	private:
		Eigen::Matrix4d m_transform;
		Eigen::Vector3d m_origin;
		Eigen::Vector3d m_rollPitchYaw;
		std::string m_name;
		ObjectType m_objectType = ObjectType::Unassigned;
		ShapeType m_shapeType = ShapeType::Unassigned;

		// Name of the parent body.
		// TODO: Create derived class "Collider" to implement this.
		std::string m_parentBodyName = "None";

	public:
		// Origin + RPY constructor.
		Shape(const Eigen::Vector3d& origin, const Eigen::Vector3d& rollPitchYaw, const std::string& name, ObjectType objectType, ShapeType shapeType);

		// Destructor.
		virtual ~Shape();

		// Compute transform from origin + RPY.
		void computeTransform();

		// Get current transform.
		Eigen::Matrix4d getTransform() const;

		// Get name.
		std::string getName() const;

		// Get the object's collision filtering type.
		ObjectType getObjectType() const;

		// Get shape type.
		ShapeType getShapeType() const;

		// Set the transform.
		void setTransform(const Eigen::Matrix4d& transform);

		// Set the parent body name.
		void setParentBodyName(const std::string& parentBodyName);

		// Get the parent body name.
		std::string getParentBodyName() const;
	};
}
