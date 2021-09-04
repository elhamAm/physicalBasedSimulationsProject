#include <igl/edges.h>
#include "Simulation.h"

using namespace std;

struct Spring {
   public:
    float stiffness;
    float damping;
};

/*
 * Simulation of a string with an attached mass.
 */
class MSSSim : public Simulation {
   public:
    MSSSim() : Simulation() { init(); }

    virtual void init() override {
        setMesh(0);

        m_dt = 1e-2;
        m_mass = 1.0;
        m_floor_stiffness = 1000.0;
        m_gravity << 0, -9.81, 0;

        reset();
    }

    virtual void resetMembers() override {
        p_cube->reset();
        Eigen::Vector3d starting_pos;
        starting_pos << 0, 4, 0;
        p_cube->setMass(m_mass);
        p_cube->setMesh(m_Vorig.rowwise() + starting_pos.transpose(), m_Forig);
        m_velocities = std::vector<Eigen::Vector3d>(m_Vorig.rows(), Eigen::Vector3d::Zero());
    }

    virtual void updateRenderGeometry() override {
        p_cube->getMesh(m_renderV, m_renderF);
    }

	virtual bool advance() override;

    virtual void renderRenderGeometry(
        igl::opengl::glfw::Viewer &viewer) override {
        viewer.data().set_mesh(m_renderV, m_renderF);
    }

#pragma region SettersAndGetters
    void setMass(double m) { m_mass = m; }

    void setSpring(const Spring &s) {
        m_spring = s;
    }

    void setMesh(int mesh) {
        std::string path;
        switch(mesh){
            case 0:
				path = "cube.off";
				break;
            case 1:
                path = "cube_opposite.off";
                break;
            case 2:
				path = "cube_cross.off";
				break;
        }
        m_objects.clear();
        m_objects.push_back(RigidObject(path));
        p_cube = &m_objects.back();

        p_cube->getMesh(m_Vorig, m_Forig);

		// update spring info.
        igl::edges(m_Forig, m_edges);
        m_lengths.resize(m_edges.size());
        for (int i = 0; i < m_edges.rows(); i++) {
            m_lengths[i] =
                (m_Vorig.row(m_edges(i, 0)) - m_Vorig.row(m_edges(i, 1)))
                    .norm();
        }
    }

    void setFloorStiffness(double s){
        m_floor_stiffness = s;
    }
#pragma endregion SettersAndGetters

   private:
    int m_method;  // id of integrator to be used (0: analytical, 1: explicit
                   // euler, 2: semi-implicit euler, 3: explicit midpoint, 4:
                   // implicit euler)
    double m_mass;
    double m_floor_stiffness;

    Eigen::Vector3d m_gravity;

    Spring m_spring;
    RigidObject *p_cube;
    std::vector<double> m_lengths;
    std::vector<Eigen::Vector3d> m_velocities;
    Eigen::MatrixXi m_edges;

    Eigen::MatrixXd m_renderV, m_Vorig;  // vertex positions for rendering
    Eigen::MatrixXi m_renderF, m_Forig;  // face indices for rendering
};