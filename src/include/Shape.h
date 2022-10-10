#pragma once

#include "ObjectType.h"
#include <Eigen/Dense>
#include <string>

namespace MotionPlanner
{
	struct ContactPoint;

	/// @brief Primitive geometry type enum.
	enum class ShapeType
	{
		Unassigned,
		Sphere,
		Capsule,
		Box
	};

	/// @brief Base class for primitive geometry actors.
	class Shape
	{
	private:
		/// @brief Actor transform.
		Eigen::Matrix4d m_transform;

		/// @brief Actor origin.
		Eigen::Vector3d m_origin;

		/// @brief Actor roll, pitch, and yaw used to initialize PhysX shapes.
		/// @bug Does not update when setTransform is called.
		Eigen::Vector3d m_rollPitchYaw;

		/// @brief Name of shape actor.
		std::string m_name;

		/// @brief Classification of the object for collision filtering (Robot, Obstacle, ...).
		ObjectType m_objectType = ObjectType::Unassigned;

		/// @brief Primitive geometry type classification (Sphere, Capsule, Box...).
		ShapeType m_shapeType = ShapeType::Unassigned;

		/// @brief Name of parent RigidBody if the geometry is a robot geometry.
		std::string m_parentBodyName = "None";

	public:
		/// @brief Constructor.
		/// @param origin Position of actor.
		/// @param rollPitchYaw Orientation of actor.
		/// @param name Name of actor.
		/// @param objectType Object classification of actor.
		/// @param shapeType Shape geometry classification of actor.
		Shape(const Eigen::Vector3d& origin, const Eigen::Vector3d& rollPitchYaw, const std::string& name, ObjectType objectType, ShapeType shapeType);

		/// @brief Destructor.
		/// @bug Does this need to be virtual?
		virtual ~Shape();

		/// @brief Update transform based on origin and roll pitch yaw.
		void computeTransform();

		/// @brief Get the current transform.
		/// @return Transform.
		const Eigen::Matrix4d& getTransform() const;

		/// @brief Get the actor's name.
		/// @return Name.
		const std::string& getName() const;

		/// @brief Get the object type classification.
		/// @return Object type.
		ObjectType getObjectType() const;

		/// @brief Get the primitive shape type.
		/// @return Shape type.
		ShapeType getShapeType() const;

		/// @brief Set the transform of the actor.
		/// @param transform Transform to be set.
		/// @bug Does not update rollPitchYaw member.
		void setTransform(const Eigen::Matrix4d& transform);

		/// @brief Set the name of the parent rigid body.
		/// @param parentBodyName Name of parent body.
		/// @bug Should only allow this for robot geometry actors.
		void setParentBodyName(const std::string& parentBodyName);

		/// @brief Get the name of the parent rigid body.
		/// @return Name.
		const std::string& getParentBodyName() const;
	};
}
