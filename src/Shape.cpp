#include "Shape.h"
#include "ObjectType.h"
#include "ContactPoint.h"
#include "RigidBody.h"
#include <Eigen/Dense>
#include <string>

namespace CollisionAvoidance
{
	// Origin + RPY constructor.
	Shape::Shape(const Eigen::Vector3d& origin, const Eigen::Vector3d& rollPitchYaw, const std::string& name, ObjectType objectType, ShapeType shapeType)
		: m_origin(origin), m_rollPitchYaw(rollPitchYaw), m_transform(Eigen::Matrix4d::Identity()),
		m_name(name), m_objectType(objectType), m_shapeType(shapeType)
	{
		computeTransform();
	}

	// Destructor.
	Shape::~Shape()
	{
		// delete m_contactPoint;
	}

	// Compute transform from origin + RPY.
	void Shape::computeTransform()
	{
		// Create angle-axis representations of roll, pitch, and yaw.
		Eigen::AngleAxisd rollAngleAxis(m_rollPitchYaw[0], Eigen::Vector3d::UnitX());
		Eigen::AngleAxisd pitchAngleAxis(m_rollPitchYaw[1], Eigen::Vector3d::UnitY());
		Eigen::AngleAxisd yawAngleAxis(m_rollPitchYaw[2], Eigen::Vector3d::UnitZ());

		// Create quaternion and convert into rotation matrix.
		Eigen::Quaterniond q = yawAngleAxis * pitchAngleAxis * rollAngleAxis;
		Eigen::Matrix3d R = q.matrix();

		// Create transformation matrix.
		Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
		T.block(0, 0, 3, 3) = R;
		T.block(0, 3, 3, 1) = m_origin;
		m_transform = T;
	}

	// Get current transform.
	Eigen::Matrix4d Shape::getTransform() const
	{
		return m_transform;
	}

	// Get name.
	std::string Shape::getName() const
	{
		return m_name;
	}

	// Get the object's collision filtering type.
	ObjectType Shape::getObjectType() const
	{
		return m_objectType;
	}

	// Get shape type.
	ShapeType Shape::getShapeType() const
	{
		return m_shapeType;
	}

	void Shape::setTransform(const Eigen::Matrix4d& transform)
	{
		m_transform = transform;
	}

	void Shape::setParentBodyName(const std::string& parentBodyName)
	{
		m_parentBodyName = parentBodyName;
	}

	std::string Shape::getParentBodyName() const
	{
		return m_parentBodyName;
	}
}
