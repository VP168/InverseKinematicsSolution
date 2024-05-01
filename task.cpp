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
    link_offset.translation()(1) = 1.;

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
vector_t inverse_kinematics(vector_t const & q_start, trafo2d_t const & goal ) {
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
