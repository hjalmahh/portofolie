#include "app/implementations/screwskinematicssolver.h"

#include <utility/math.h>

using namespace AIS4104;

ScrewsKinematicsSolver::ScrewsKinematicsSolver(Eigen::Matrix4d m, std::vector<Eigen::VectorXd> screws, Simulation::JointLimits limits)
    : ScrewsKinematicsSolver(std::move(m), std::move(screws), 4.e-3, 4.e-3, std::move(limits))
{
}

ScrewsKinematicsSolver::ScrewsKinematicsSolver(Eigen::Matrix4d m, std::vector<Eigen::VectorXd> space_screws, double v_e, double w_e, Simulation::JointLimits limits)
    : KinematicsSolver(std::move(limits))
    , m_ve(v_e)
    , m_we(w_e)
    , m_m(std::move(m))
    , m_screws(std::move(space_screws))
{
}

void ScrewsKinematicsSolver::set_epsilons(double v_e, double w_e)
{
    m_ve = v_e;
    m_we = w_e;
}

uint32_t ScrewsKinematicsSolver::joint_count() const
{
    return m_screws.size();
}

//TASK: Implement fk_solve using screws.
Eigen::Matrix4d ScrewsKinematicsSolver::fk_solve(const Eigen::VectorXd &joint_positions)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    for (uint32_t i = 0; i < joint_count(); ++i)
    {
        Eigen::VectorXd s = m_screws[i];
        Eigen::Matrix4d T_n= utility::matrix_exponential(s, joint_positions[i]);

        T= T*T_n;
    }


    return T * m_m;
}

Eigen::VectorXd ScrewsKinematicsSolver::ik_solve(const Eigen::Matrix4d &t_sd, const Eigen::VectorXd &j0)
{
    return ik_solve(t_sd, j0, [&](const std::vector<Eigen::VectorXd> &) { return 0u; });
}

//TASK: Implement ik_solve using screws.
//algorithm page 228 MR 2019
Eigen::VectorXd ScrewsKinematicsSolver::ik_solve(const Eigen::Matrix4d &t_sd, const Eigen::VectorXd &j0, const std::function<uint32_t(const std::vector<Eigen::VectorXd> &)> &solution_selector)
{
    int max_iter = 10000; //max iterations it wil run
    int iter_count = 0; //counts iterations
    double theta;
    Eigen::VectorXd v_b(6);
    Eigen::VectorXd cu_jp = j0;

    Eigen::Matrix4d t_sb = fk_solve(cu_jp);
    Eigen::Matrix4d t_bd = (t_sb).inverse() * t_sd;
    std::tie(v_b, theta) = utility::matrix_logarithm(t_bd);
    v_b *= theta;
    Eigen::MatrixXd jac_body;



    std::pair<Eigen::VectorXd, double> p;
    while ((v_b.head(3).norm() > m_ve) || (v_b.tail(3).norm() > m_we))
    {
        Eigen::MatrixXd jac = body_jacobian(j0);
        cu_jp *= jac.completeOrthogonalDecomposition().pseudoInverse() * v_b;

        Eigen::Matrix4d t_sb = fk_solve(cu_jp);
        Eigen::Matrix4d t_bd = t_sb.inverse() * t_sd;

        std::tie(v_b, theta) = utility::matrix_logarithm(t_bd);
        v_b *= theta;

        /*if (iter_count > max_iter)
        {
            return std::make_pair(iter_count, cu_jp);
        }

        iter_count++;*/

    }
    return cu_jp;
}

std::pair<Eigen::Matrix4d, std::vector<Eigen::VectorXd>> ScrewsKinematicsSolver::space_chain()
{
    return {m_m, m_screws};
}

//TASK: Implement body_chain() using space_chain().
//page 183/184 MR 2019
std::pair<Eigen::Matrix4d, std::vector<Eigen::VectorXd>> ScrewsKinematicsSolver::body_chain()
{
    auto[m, space_screw] = space_chain();
    Eigen::Matrix4d m_inv = m.inverse();

    std::vector<Eigen::VectorXd> body_screws;
    for(const auto& screw : space_screw)
        body_screws.emplace_back(utility::adjoint_matrix(m_inv)*screw);

    return std::make_pair(m, body_screws);
}

//TASK: Implement space_jacobian() using space_chain()
//page 177 MR 2019
Eigen::MatrixXd ScrewsKinematicsSolver::space_jacobian(const Eigen::VectorXd &current_joint_positions)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    auto [m, screws] = space_chain();
    const int num_joints = screws.size(); //basere DOF på fuknsjonen som brukes

    //6x6 jacobian matrix
    Eigen::MatrixXd jac_space(6, num_joints);
    jac_space.setZero();

    for (int i = 0; i < num_joints; i++)
    {
        jac_space.col(i) = utility::adjoint_matrix(T) * screws[i];

        T = T * utility::matrix_exponential(screws[i], current_joint_positions[i]);
    }
    return jac_space;
}

//TASK: Implement body_jacobian() using body_chain()
//page 182 MR 2019
Eigen::MatrixXd ScrewsKinematicsSolver::body_jacobian(const Eigen::VectorXd &current_joint_positions)
{
    {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        auto [m, screws] = body_chain();
        const int num_joints = screws.size(); //basere DOF på fuknsjonen som brukes

        //6x6 jacobian matrix
        Eigen::MatrixXd jac_body(6, num_joints);
        jac_body.setZero();

        for (int i = num_joints - 1; i != -1; i--)
        {
            //std::cout << "Screw " << i << ": " << screws[i].transpose() << std::endl;
            jac_body.col(i) = utility::adjoint_matrix(T) * screws[i];

            T = T * utility::matrix_exponential(screws[i], current_joint_positions[i]).inverse();
        }
        return jac_body;
    }
}
