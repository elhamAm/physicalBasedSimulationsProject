#include "MSSSim.h"

/////////////////////////////////////
/////////////// EX 5 ////////////////
/////////////////////////////////////

bool MSSSim::advance() {
    Eigen::MatrixXd V, V_new;
    Eigen::MatrixXi F;
    p_cube->getMesh(V, F);
    V_new = V;

    std::vector<Eigen::Vector3d> f(V.rows(), Eigen::Vector3d::Zero());

	////
	// TODO: update V_new (position) with f (force)
	// use m_edges and m_lengths to get info. about springs as below
	// e.g., int a = m_edges(i, 0); int b = m_edges(i, 1); Eigen::Vector3d v0 = V.row(a);
	// when each vertex is lower than 0 (i.e., floor), compute penalty force with m_floor_stiffness
	// ...
	////

    p_cube->setMesh(V_new, F);

    // advance m_time
    m_time += m_dt;
    m_step++;

    return false;
}