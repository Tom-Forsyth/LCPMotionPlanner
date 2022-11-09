#include "Joint.h"
#include "Kinematics.h"
#include <Eigen/Dense>

namespace MotionPlanner
{
    Joint::Joint(const JointType& type, const Eigen::Vector3d& axis, const Eigen::Vector3d& point, const std::pair<double, double>& jointLimits)
        : m_type(type), m_axis(axis), m_point(point), m_jointLimits(jointLimits)
    {
        computeTwistCoord();
        computeTwist();
    }

    double Joint::getDisplacement() const
    {
        return m_displacement;
    }

    bool Joint::setDisplacement(const double& displacement)
    {
        if (!wouldViolateLimits(displacement) && std::isfinite(displacement))
        {
            m_displacement = displacement;
            computeRelativeTransformation();
            return true;
        }
        return false;
    }

    Eigen::Matrix4d Joint::getRelativeTransformation() const
    {
        return m_relativeTransformation;
    }

    JointType Joint::getType() const
    {
        return m_type;
    }

    Eigen::Vector<double, 6> Joint::getTwistCoord() const
    {
        return m_twistCoord;
    }

    void Joint::computeTwistCoord()
    {
        if (m_type == JointType::Revolute)
        {
            m_twistCoord.segment(0, 3) = -m_axis.cross(m_point);
            m_twistCoord.segment(3, 3) = m_axis;
        }
        else if (m_type == JointType::Prismatic)
        {
            m_twistCoord.segment(0, 3) = m_axis;
        }
    }

    void Joint::computeTwist()
    {
        Eigen::Vector3d v = m_twistCoord.segment(0, 3);
        Eigen::Matrix3d wHat = Kinematics::skew(m_twistCoord.segment(3, 3));
        m_twist.topLeftCorner(3, 3) = wHat;
        m_twist.topRightCorner(3, 1) = v;
    }

    void Joint::computeRelativeTransformation()
    {
        Eigen::Matrix3d R = Kinematics::AxisAngletoRot(m_axis, m_displacement);
        if (m_type == JointType::Revolute)
        {
            m_relativeTransformation.block(0, 0, 3, 3) = R;
            m_relativeTransformation.block(0, 3, 3, 1) = (Eigen::Matrix3d::Identity() - R) * m_point;
        }
        else if (m_type == JointType::Prismatic)
        {
            m_relativeTransformation.block(0, 3, 3, 1) = m_twistCoord.segment(3, 3) * m_displacement;
        }
    }

    bool Joint::wouldViolateLimits(double displacement) const 
    {
        // If the joint is not fixed, check for joint limit violation.
        if (m_type != JointType::Fixed)
        {
            if ((displacement < m_jointLimits.first) || (displacement > m_jointLimits.second))
            {
                return true;
            }
        }
        return false;
    }

    std::pair<double, double> Joint::getLimits() const
    {
        return m_jointLimits;
    }
}
