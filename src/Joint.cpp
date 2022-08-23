#include "Joint.h"
#include "Kinematics.h"
#include <Eigen/Dense>

// Constructor.
Joint::Joint(const Joint::Type& type, const Eigen::Vector3d& axis, const Eigen::Vector3d& point)
    : m_type(type), m_axis(axis), m_point(point), m_displacement(0),
    m_twistCoord(Eigen::Vector<double, 6>::Zero()), m_twist(Eigen::Matrix4d::Zero()),
    m_relativeTransformation(Eigen::Matrix4d::Identity())
{
    computeTwistCoord();
    computeTwist();
}

// Compute twist coordinate.
void Joint::computeTwistCoord()
{
    if (m_type == REVOLUTE)
    {
        m_twistCoord.segment(0, 3) = -m_axis.cross(m_point);
        m_twistCoord.segment(3, 3) = m_axis;
    }
    else if (m_type == PRISMATIC)
    {
        m_twistCoord.segment(0, 3) = m_axis;
    }
}

// Compute twist.
void Joint::computeTwist()
{
    Eigen::Vector3d v = m_twistCoord.segment(0, 3);
    Eigen::Matrix3d wHat = Kinematics::skew(m_twistCoord.segment(3, 3));
    m_twist.topLeftCorner(3, 3) = wHat;
    m_twist.topRightCorner(3, 1) = v;
}

// Compute displacement transformation.
void Joint::computeRelativeTransformation()
{
    Eigen::Matrix3d R = Kinematics::AxisAngletoRot(m_axis, m_displacement);
    if (m_type == REVOLUTE)
    {
        m_relativeTransformation.block(0, 0, 3, 3) = R;
        m_relativeTransformation.block(0, 3, 3, 1) = (Eigen::Matrix3d::Identity() - R) * m_point;
    }
    else if (m_type == PRISMATIC)
    {
        m_relativeTransformation.block(0, 3, 3, 1) = m_twistCoord.segment(3, 3) * m_displacement;
    }
}

// Get relative transformation.
Eigen::Matrix4d Joint::getRelativeTransformation() const
{
    return m_relativeTransformation;
}

// Set joint displacement.
void Joint::setDisplacement(const double& displacement)
{
    m_displacement = displacement;
    computeRelativeTransformation();
}

// Get type.
Joint::Type Joint::getType() const
{
    return m_type;
}

// Get twist coordinate.
Eigen::Vector<double, 6> Joint::getTwistCoord() const
{
    return m_twistCoord;
}

// Get displacement.
double Joint::getDisplacement() const
{
    return m_displacement;
}
