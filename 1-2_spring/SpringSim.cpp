#include "SpringSim.h"

/////////////////////////////////////
//////// EX 1 - PROBLEM 2 ///////////
/////////////////////////////////////
bool SpringSim::advance() {
    // perform time integration with different integrators

	// useful functions: normalized(), norm()
    Eigen::Vector3d spring_dir =
        (m_spring.end - m_spring.start).normalized();
	float spring_norm = (m_spring.start - m_spring.end).norm();

	// TODO: compute external forces to get the acceleartion,
	// which is commonly used by the following methods

	// HINT: use p_cube, m_spring, m_dt, m_gravity

	Eigen::Vector3d v = p_cube->getLinearVelocity();
	Eigen::Vector3d p = p_cube->getPosition();

	// note that it is required to update both m_spring.end and p_cube's position
    switch (m_method) {
        case 0:
            // TODO: analytical solution
            break;

        case 1:
            // TODO: explicit euler
            break;

        case 2:
            // TODO: symplectic euler
            break;

        case 3:
            // TODO: explicit midpoint
            break;

        case 4:
            // TODO: implicit euler
            break;

        default:
            std::cerr << m_method << " is not a valid integrator method."
                        << std::endl;
    }

	// update spring end position
	m_spring.end = p_cube->getPosition();


    // advance m_time
    m_time += m_dt;
    m_step++;

    // log
    if ((m_step % m_log_frequency) == 0) {
        m_trajectories.back().push_back(p_cube->getPosition());
    }

    return false;
}