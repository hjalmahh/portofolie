#include "app/robotwrapper.h"

#include <utility/math.h>
#include <utility/vectors.h>

using namespace AIS4104;

RobotWrapper::RobotWrapper(std::shared_ptr<threepp::Robot> robot, std::shared_ptr<Simulation::KinematicsSolver> solver)
: m_tool_transform(Eigen::Matrix4d::Identity())
, m_robot(std::move(robot))
, m_solver(std::move(solver))
{
}

threepp::Robot& RobotWrapper::threepp_robot()
{
    return *m_robot;
}

std::shared_ptr<threepp::Robot> RobotWrapper::threepp_robot_ptr()
{
    return m_robot;
}

const threepp::Robot& RobotWrapper::threepp_robot() const
{
    return *m_robot;
}

uint8_t RobotWrapper::joint_count() const
{
    return m_solver->joint_count();
}
//source franschesko
//TASK: Implement the function to calculate the joint positions for the desired tool pose
// a) Use m_tool_transform to calculate the flange pose required by m_solver.ik_solve()
// b) Use the m_solver.ik_solve() overload with the solution selector lambda to choose the most desirable IK solution.
Eigen::VectorXd RobotWrapper::ik_solve_pose(const Eigen::Matrix4d &desired_tool_pose, const Eigen::VectorXd &j0) const
{

        //Eigen::Matrix4d pose_d = desired_tool_pose * m_tool_transform.inverse();
        //Eigen::VectorXd joint_positions = m_solver->ik_solve(pose_d, j0, [&](const std::vector<Eigen::VectorXd> &) { return 0u; });

return joint_positions();
}
//source franschesko
//TASK: Implement the function to calculate the joint positions for the desired flange pose
// a) Use m_tool_transform to calculate the flange pose required by m_solver.ik_solve()
// b) Use the m_solver.ik_solve() overload with the solution selector lambda to choose the most desirable IK solution.
Eigen::VectorXd RobotWrapper::ik_solve_flange_pose(const Eigen::Matrix4d &desired_flange_pose, const Eigen::VectorXd &j0) const
{
    Eigen::Matrix4d pose_d = desired_flange_pose * m_tool_transform.inverse();
    Eigen::VectorXd joint_positions = m_solver->ik_solve(pose_d, j0);

    return joint_positions;
}

Eigen::Matrix4d RobotWrapper::tool_transform() const
{
    return m_tool_transform;
}

void RobotWrapper::set_tool_transform(Eigen::Matrix4d transform)
{
    m_tool_transform = std::move(transform);
}

//TASK: Calculate the pose of the end effector using forward kinematics;
// Relevant variables are m_solver and m_tool_transform.
Eigen::Matrix4d RobotWrapper::current_pose() const
{

    /*Eigen::Matrix4d current_tool_pose = m_solver->fk_solve(joint_positions());
    Eigen::Matrix4d base = m_tool_transform.inverse() * current_tool_pose;
    return base;*/
    return Eigen::Matrix4d::Identity();
}

//TASK: Calculate the position of the end effector using forward kinematics.
// Relevant variables are m_solver and m_tool_transform (or possibly another function of RobotWrapper?).
Eigen::Vector3d RobotWrapper::current_position() const
{/*
    Eigen::Matrix4d pose_cur = Eigen::Matrix4d::Identity()*m_tool_transform.inverse();
    Eigen::Matrix4d joints = m_solver->fk_solve(joint_positions());
    Eigen::Matrix4d current_tool_pose = m_tool_transform.inverse() * joints;
    Eigen::Vector3d position = current_tool_pose.block(0,3, 3, 0);
    return position;
    */
    return Eigen::Vector3d::Zero();

}


//TASK: Calculate the orientation of the end effector using forward kinematics and m_solver (or another function of RobotWrapper?).
Eigen::Vector3d RobotWrapper::current_orientation_zyx() const
{/*
    Eigen::Matrix4d pose_cur = Eigen::Matrix4d::Identity()*m_tool_transform.inverse();
    Eigen::Matrix4d joints = m_solver->fk_solve(joint_positions());
    Eigen::Matrix4d current_tool_pose = m_tool_transform.inverse() * joints;

    return Eigen::Vector3d::Zero();
    */
    return Eigen::Vector3d::Zero();
}

//TASK: Calculate the pose of the end effector using forward kinematics and m_solver.
Eigen::Matrix4d RobotWrapper::current_flange_pose() const
{
    Eigen::Matrix4d cur_fp = m_solver->fk_solve(joint_positions());
    return cur_fp;
}

//TASK: Based on the flange pose, return its linear position.
Eigen::Vector3d RobotWrapper::current_flange_position() const
{
    Eigen::Matrix4d cur_fp = current_flange_pose();
    Eigen::Vector3d position = cur_fp.block<3,1>(0,3);

    return position;
}

//TASK: Based on the flange pose, return its orientation in the Euler ZYX representation.
Eigen::Vector3d RobotWrapper::current_flange_orientation_zyx() const
{
    Eigen::Matrix3d rot_f = current_flange_pose().block<3,3>(0,0);
   Eigen::Vector3d orientation = utility::euler_zyx_from_rotation_matrix(rot_f);

    return orientation;
}

const Simulation::JointLimits& RobotWrapper::joint_limits() const
{
    return m_solver->joint_limits();
}

Eigen::VectorXd RobotWrapper::joint_positions() const
{
    return utility::to_eigen_vectord(m_robot->jointValues());
}

void RobotWrapper::set_joint_positions(const Eigen::VectorXd &joint_positions)
{
    m_robot->setJointValues(utility::to_std_vectorf(joint_positions));
}

jajajajajajaj