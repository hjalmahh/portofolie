#include "app/implementations/motionplannerimpl.h"

#include <iostream>

#include "app/examples/ptplinetrajectorygeneratorexample.h"

#include <utility/vectors.h>

#include <spdlog/spdlog.h>

#include <iostream>

#include "utility/math.h"

using namespace AIS4104;

//TASK: Remove the std::cout prints as the functions have been implemented. (e.g., std::cout << "MotionPlannerImpl::FUNCTION_NAME" << std::endl;)

MotionPlannerImpl::MotionPlannerImpl(const Simulation::Robot &robot)
: m_robot(robot)
{
}

Eigen::VectorXd MotionPlannerImpl::task_space_pose(const Eigen::Matrix4d &pose)
{
    return m_robot.ik_solve_pose(pose, m_robot.joint_positions());
}

//TASK: Implement a function that calculates the pose of the given screw, and solves the IK to obtain the joint positions
Eigen::VectorXd MotionPlannerImpl::task_space_screw(const Eigen::Vector3d &w, const Eigen::Vector3d &q, double theta, double h)
{
    std::cout << "MotionPlannerImpl::task_space_screw:" << std::endl << w.transpose() << std::endl << q.transpose() << std::endl << theta << std::endl << h << std::endl << std::endl;
    return m_robot.joint_positions();
}

//TASK: Implement jogging in tool frame from the start pose along the displacement and rotation
Eigen::VectorXd MotionPlannerImpl::tool_frame_displace(const Eigen::Matrix4d &tw_pose, const Eigen::Vector3d &tf_offset, const Eigen::Vector3d &tf_zyx)
{
    std::cout << "MotionPlannerImpl::tool_frame_displace:" << std::endl << tw_pose << std::endl << tf_offset.transpose() << std::endl << tf_zyx.transpose() << std::endl << std::endl;
    return m_robot.joint_positions();
}

//TASK: Implement a P2P trajectory generator from the current configuration to the target.
std::shared_ptr<Simulation::TrajectoryGenerator> MotionPlannerImpl::task_space_ptp_trajectory(const Eigen::Vector3d &pos, const Eigen::Vector3d &euler_zyx)
{
    std::cout << "MotionPlannerImpl::task_space_ptp_trajectory:" << std::endl << pos.transpose() << std::endl << euler_zyx.transpose() << std::endl << std::endl;
    return nullptr;
}

//TASK: Implement a LIN trajectory generator from the current configuration to the target.
std::shared_ptr<Simulation::TrajectoryGenerator> MotionPlannerImpl::task_space_lin_trajectory(const Eigen::Vector3d &pos, const Eigen::Vector3d &euler_zyx)
{
    std::cout << "MotionPlannerImpl::task_space_lin_trajectory:" << std::endl << pos.transpose() << std::endl << euler_zyx.transpose() << std::endl << std::endl;
    return nullptr;
}

//TASK: Implement a screw trajectory generator from the current configuration to the target.
std::shared_ptr<Simulation::TrajectoryGenerator> MotionPlannerImpl::task_space_screw_trajectory(const Eigen::Vector3d &w, const Eigen::Vector3d &q, double theta, double h)
{
    std::cout << "MotionPlannerImpl::task_space_screw_trajectory:" << std::endl << w.transpose() << std::endl << q.transpose() << std::endl << theta << std::endl << h << std::endl << std::endl;
    return nullptr;
}

std::shared_ptr<Simulation::TrajectoryGenerator> MotionPlannerImpl::task_space_trajectory(const std::vector<Eigen::Matrix4d> &waypoints)
{
    std::vector<Eigen::VectorXd> joint_positions;
    auto idx = 0u;
    if(waypoints.empty())
    {
        spdlog::error("[MotionPlannerImpl::task_space_trajectory] No waypoints given.");
        return nullptr;
    }
    if(waypoints.size() == 1u)
        joint_positions.push_back(m_robot.joint_positions());
    else
        joint_positions.push_back(m_robot.ik_solve_pose(waypoints[idx++], m_robot.joint_positions()));

    for(auto i = idx; i < waypoints.size(); i++)
        joint_positions.push_back(m_robot.ik_solve_pose(waypoints.at(i), joint_positions.at(i)));
    return joint_space_trajectory(joint_positions);
}

//TASK: Implement joint space trajectory generators.
// a) Implement a point to point generator which does not have infinite acceleration (e.g., polynomial time scaling).
// b) Change the use of the standard PTPLineTrajectoryGeneratorExample to your custom trajectory generator.
// c) Implement a multipoint trajectory generator for trajectories with multiple waypoints.
std::shared_ptr<Simulation::TrajectoryGenerator> MotionPlannerImpl::joint_space_trajectory(const std::vector<Eigen::VectorXd> &waypoints)
{
    std::shared_ptr<Simulation::TrajectoryGenerator> trajectory_generator;
    if(waypoints.size() == 2u)
        trajectory_generator = std::make_shared<PTPLineTrajectoryGeneratorExample>(waypoints.front(), waypoints.back());
    else if(waypoints.size() == 1u)
        trajectory_generator = std::make_shared<PTPLineTrajectoryGeneratorExample>(m_robot.joint_positions(), waypoints.front());
    else if(waypoints.empty())
    {
        spdlog::error("[MotionPlannerImpl::joint_space_trajectory] No waypoints given.");
    }
    else
    {
        spdlog::error("[MotionPlannerImpl::joint_space_trajectory] Multipoint trajectories are not implemented.");
    }
    return trajectory_generator;
}
