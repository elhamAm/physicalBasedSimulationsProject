#include <igl/writeOFF.h>
#include <thread>
#include "Gui.h"
#include "Simulator.h"
#include "MSSSim.h"

/*
 * GUI for the spring simulation. This time we need additional paramters,
 * e.g. which integrator to use for the simulation and the force applied to the
 * canonball, and we also add some more visualizations (trajectories).
 */
class MSSGui : public Gui {
   public:
    // simulation parameters
    float m_dt = 1e-2;
    float m_mass = 1.0;
    float m_floor_stiffness = 1000;

    Spring m_spring;  // stores properties of a spring

    MSSSim *p_MSSSim = NULL;

    const vector<char const *> m_mesh_types = {"Regular", "Opposite", "Cross"};
    int m_selected_mesh = 0;

    MSSGui() {
        // initialize the spring to be used
        m_spring.stiffness = 100.0;
        m_spring.damping = 0.1;

        p_MSSSim = new MSSSim();
        p_MSSSim->setSpring(m_spring);
        setSimulation(p_MSSSim);

        // show vertex velocity instead of normal
        callback_clicked_vertex =
            [&](int clickedVertexIndex, int clickedObjectIndex,
                Eigen::Vector3d &pos, Eigen::Vector3d &dir) {
                RigidObject &o = p_MSSSim->getObjects()[clickedObjectIndex];
                pos = o.getVertexPosition(clickedVertexIndex);
                dir = o.getVelocity(pos);
            };
        start();
    }

    virtual void updateSimulationParameters() override {
        // change all parameters of the simulation to the values that are set in
        // the GUI
        p_MSSSim->setTimestep(m_dt);
        p_MSSSim->setMass(m_mass);
        p_MSSSim->setSpring(m_spring);
        p_MSSSim->setMesh(m_selected_mesh);
        p_MSSSim->setFloorStiffness(m_floor_stiffness);
    }

    virtual void clearSimulation() override {
    }

    virtual void drawSimulationParameterMenu() override {
        ImGui::Text("Object");
        ImGui::Combo("Mesh", &m_selected_mesh, m_mesh_types.data(), m_mesh_types.size());
        ImGui::InputFloat("Spring Stiffness", &m_spring.stiffness, 0, 0);
        ImGui::InputFloat("Mass", &m_mass, 0, 0);
        ImGui::InputFloat("Damping", &m_spring.damping, 0, 0);
        ImGui::InputFloat("dt", &m_dt, 0, 0);
        ImGui::Text("Floor");
        ImGui::InputFloat("Floor Stiffness", &m_floor_stiffness, 0, 0);
    }
};

int main(int argc, char *argv[]) {
    // create a new instance of the GUI for the spring simulation
    new MSSGui();

    return 0;
}