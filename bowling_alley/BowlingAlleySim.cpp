#include "BowlingAlleySim.h"

using namespace std;
/*
 * Bowling alley simulation.
 */

Eigen::Matrix3d skew(const Eigen::Vector3d& a) {
  Eigen::Matrix3d s;
  s << 0, -a.z(), a.y(), a.z(), 0, -a.x(), -a.y(), a.x(), 0;
  return s;
}

bool BowlingAlleySim::advance(){
    // compute the collision detection
    m_collisionDetection.computeCollisionDetection(m_broadPhaseMethod, m_narrowPhaseMethod, m_eps);

    // apply forces (only gravity in this case)
    for (auto &o : m_objects) {
        o.applyForceToCOM(m_gravity);
    }

    for (auto &o : m_objects) {        
        // integrate velocities
        o.setLinearMomentum(o.getLinearMomentum() + m_dt * o.getForce());
        o.setAngularMomentum(o.getAngularMomentum() + m_dt * o.getTorque());
        o.resetForce();
        o.resetTorque();

        // integrate position
        o.setPosition(o.getPosition() + m_dt * o.getLinearVelocity());

        // integrate rotation
        // angular velocity
        Eigen::Vector3d w = o.getAngularVelocity();

        Eigen::Quaterniond q = o.getRotation();
        Eigen::Matrix3d Ib = o.getInertia();
        double h = m_dt;

        // Convert to body coordinates
        Eigen::Vector3d omegab = q.inverse() * w;

        // Residual vector
        Eigen::Vector3d f = h * omegab.cross(Ib * omegab);

        // Jacobian
        Eigen::Matrix3d J = Ib + h * (skew(omegab) * Ib - skew(Ib * omegab));

        // Single Newton-Raphson update
        Eigen::Vector3d res = J.colPivHouseholderQr().solve(f);
        omegab = omegab - res;

        // Back to world coordinates
        w = q * omegab;

        o.setAngularVelocity(w);

        // update orientation
        switch (m_method) {
        case 0: {
			// quaternion-based
            Eigen::Quaterniond wq;
            wq.w() = 0;
            wq.vec() = w;

            Eigen::Quaterniond q = o.getRotation();
            Eigen::Quaterniond dq = wq * q;
            Eigen::Quaterniond new_q;
            new_q.w() = q.w() + 0.5 * m_dt * dq.w();
            new_q.vec() = q.vec() + 0.5 * m_dt * dq.vec();
            o.setRotation(new_q.normalized());
            break;
/*
            // matrix-based angular velocity
            Eigen::Matrix3d r = o.getRotationMatrix();
            Eigen::Matrix3d W;

            // skew-matrix (row-wise)
            W << 0, -w.z(), w.y(),
                w.z(), 0, -w.x(),
                -w.y(), w.x(), 0;

            r = r + m_dt * W * r;

            // orthogonalize rotation matrix to show issue
            // https://math.stackexchange.com/questions/3292034/normalizing-a-rotation-matrix
            // https://en.wikipedia.org/wiki/Orthogonal_Procrustes_problem
            // idea is to find the nearest orthogonral matrix by SVD
            Eigen::JacobiSVD<Eigen::Matrix3d> svd(r, Eigen::ComputeFullU | Eigen::ComputeFullV);
            r = svd.matrixU() * svd.matrixV().transpose();

            o.setRotation(r);
            break;
*/
        }
        default: {
            // quaternion-based
            Eigen::Quaterniond wq;
            wq.w() = 0;
            wq.vec() = w;

            Eigen::Quaterniond q = o.getRotation();
            Eigen::Quaterniond dq = wq * q;
            Eigen::Quaterniond new_q;
            new_q.w() = q.w() + 0.5 * m_dt * dq.w();
            new_q.vec() = q.vec() + 0.5 * m_dt * dq.vec();
            o.setRotation(new_q.normalized());
            break;
        }
        }
    }

    // advance time
    m_time += m_dt;
    m_step++;

    return false;
}
