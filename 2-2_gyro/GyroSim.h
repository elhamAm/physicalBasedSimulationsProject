#include "Simulation.h"

using namespace std;

/*
 * Simulation of a rotating cube. Forces are applied in opposite directions on
 * opposite corners to make a cube rotate.
 */
class GyroSim : public Simulation {
   public:
    GyroSim() : Simulation() { init(); }

    virtual void init() override {
        std::string file = "t.off";

        m_objects.clear();
        m_objects.push_back(RigidObject(file));
        p_body = &m_objects.back();
        p_body->recomputeCOM();

        m_dt = 1e-3;

        reset();
    }

    virtual void resetMembers() override {
        p_body->reset();
        p_body->setMass(1.0);

        Eigen::Matrix3d In;
        double low = m_diag(0);
        double mid = m_diag(1);
        double high = m_diag(2);
        In << low, 0, 0, 0, mid, 0, 0, 0, high;
        p_body->setInertia(In);

        p_body->setAngularVelocity(m_w);
    }

    virtual void updateRenderGeometry() override {
        p_body->getMesh(m_renderV, m_renderF);
    }

    Eigen::Matrix3d skew(const Eigen::Vector3d &a) {
        Eigen::Matrix3d s;
        s << 0, -a.z(), a.y(), a.z(), 0, -a.x(), -a.y(), a.x(), 0;
        return s;
    }

	virtual bool advance() override;

    virtual void renderRenderGeometry(
        igl::opengl::glfw::Viewer &viewer) override {
        viewer.data().set_mesh(m_renderV, m_renderF);
        viewer.data().compute_normals();
    }

    void setAngularVelocity(const Eigen::Vector3d &w) { m_w = w; }

    void setDiagonal(const Eigen::Vector3d &d) { m_diag = d; }

    void setMethod(int m) { m_method = m; }

    Eigen::Vector3d getRotationalEnergy() {
        return 0.5 * p_body->getInertia().diagonal().cwiseProduct(
                         p_body->getAngularVelocity());
    }

   private:
    Eigen::Vector3d m_w;
    Eigen::Vector3d m_diag;
    int m_method = 0;

    RigidObject *p_body;

    Eigen::MatrixXd m_renderV;  // vertex positions for rendering
    Eigen::MatrixXi m_renderF;  // face indices for rendering
};