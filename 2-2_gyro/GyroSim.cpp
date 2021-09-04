#include "GyroSim.h"

/////////////////////////////////////////
//////// EX 2 - PROBLEM 3 & 4 ///////////
/////////////////////////////////////////
bool GyroSim::advance() {
	// TODO: update angular velocity
	Eigen::Vector3d w = p_body->getAngularVelocity();
	switch (m_method) {
		case 0: {
			// semi-implicit
			break;
		}
		case 1: {
			// solve gyroscopic
			break;
		}
		default:{
			std::cerr << m_method << " is not a valid rotation method."
						<< std::endl;
		}
	}

	// TODO: update orientation

	// advance time
	m_time += m_dt;
	m_step++;

	return false;
}