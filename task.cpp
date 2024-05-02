#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

typedef Eigen::VectorXd vector_t;
typedef Eigen::MatrixXd matrix_t;
typedef Eigen::Transform<double,2,Eigen::Affine> trafo2d_t;

/**************************************************
 * A function to compute the forward kinematics of
 * a planar 3-link robot.
 *************************************************/
trafo2d_t forward_kinematics(vector_t const & q ) {
    // check that the joint angle vector has the correct size
    assert( q.size() == 3 );

    // define a constant offset between two joints
    trafo2d_t link_offset = trafo2d_t::Identity();
    link_offset.translation()(0) = 1.;

    // define the start pose
    trafo2d_t trafo = trafo2d_t::Identity();

    for(int joint_idx = 0; joint_idx < 3 ; joint_idx++ ) {
        // add the rotation contributed by this joint
        trafo *= Eigen::Rotation2D<double>(q(joint_idx));
        // add the link offset to this position
        trafo = trafo * link_offset;
    }
    return trafo;
}


/*************************************************
 * Task:
 * Complete this inverse kinematics function for the robot defined by
 * the forward kinematics function defined above.
 * It should return the joint angles q for the given goal specified in the
 * corresponding parameter.
 * Only the translation (not rotation) part of the goal has to match.
 * 
 *
 * Hints:
 * - This is an non-linear optimization problem which can be solved by using
 *   an iterative algorithm.
 * - To obtain the jacobian, use numerical differentiation
 * - To invert the jacobian use Eigen::JacobiSVD
 * - The algorithm should stop when norm of the error is smaller than 1e-3
 * - The algorithm should also stop when 200 iterations are reached
 ************************************************/

 /**************************************************
  * A function to compute the jacobian of
  * a planar 3-link robot.
  *************************************************/
matrix_t calculate_jacobian(vector_t const& q) {
    matrix_t  jacobian_matrix(2, 3);

    double c0 = cos(q(0));
    double s0 = sin(q(0));
    double c01 = cos(q(0) + q(1));
    double s01 = sin(q(0) + q(1));
    double c012 = cos(q(0) + q(1) + q(2));
    double s012 = sin(q(0) + q(1) + q(2));

    // link lengths (offset), assumed as unit length
    vector_t link(3);
    link.setConstant(1);

    // calculate jacobian matrix
    jacobian_matrix << -link(0) * s0 - link(1) * s01 - link(2) * s012, -link(1) * s01 - link(2) * s012, -link(2) * s012,
        link(0)* c0 + link(1) * c01 + link(2) * c012, link(1)* c01 + link(2) * c012, link(2)* c012;

    return jacobian_matrix;
}


 /**************************************************
  * A function to compute the inverse kinematics of
  * a planar 3-link robot.
  *************************************************/
vector_t inverse_kinematics(vector_t const & q_start, trafo2d_t const & goal ) {
    // set parameters
    const int MAX_ITERATIONS = 200;
    const double ERROR_ALLOWED = 1e-3;
    const double ALPHA = 0.01; // slowly increase the delta angle to reach goal position

    // Initialize the iterator
    int iterations = 0;
    vector_t q_current = q_start;

    while (iterations < MAX_ITERATIONS) {

        // Compute FK for current configuration of joints
        auto current_trafo = forward_kinematics(q_current);

        // Compute error between current configuration and goal configuration of joints
        vector_t error = goal.translation() - current_trafo.translation();

        // Check if error is within allowed tolerance
        if (error.norm() < ERROR_ALLOWED) { 
            std::cout << "Robot is reached to goal position, configuration is: " << std::endl;
            break; 
        }

        // Compute jacobian for current configuration
        auto jacobian_matrix = calculate_jacobian(q_current);

        // Compute right pseudo inverse
        /*auto mat_product = jacobian_matrix * jacobian_matrix.transpose();
        auto j_pseudo = jacobian_matrix.transpose()* mat_product.inverse();
        vector_t q_delta = ALPHA * j_pseudo * error;*/

        // Compute right pseudo inverse using JacobiSVD
        Eigen::JacobiSVD<matrix_t>jacobian_inverse(jacobian_matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
        vector_t q_delta = ALPHA * jacobian_inverse.matrixV() * error;


        // Add difference of configuration to current configuration
        q_current += q_delta;

        iterations++;
    }

    if (iterations > MAX_ITERATIONS) {
        std::cout << "Solution is not converged, possible nearby configuration is: " << std::endl;
    }

    return q_current;
}



/**
 * An example how the inverse kinematics can be used.
 * It should not be required to change this code.
 */
int main(){

    vector_t q_start(3);
    q_start.setConstant(-0.1);

    trafo2d_t goal = trafo2d_t::Identity();
    goal.translation()(0) = 1.;

    vector_t result = inverse_kinematics(q_start,goal);
    std::cout << result << std::endl;

    return 0;
}
