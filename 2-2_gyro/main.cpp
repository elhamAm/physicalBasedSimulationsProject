#include <deque>
#include "Gui.h"
#include "GyroSim.h"
#include "Simulator.h"

class GyroGui : public Gui {
   public:
    // simulation parameters
    Eigen::Vector3f m_w;
    Eigen::Vector3f m_diag;
    float m_mass = 1.0;
    float m_dt = 0.03;
    int m_maxHistory = 200;
    std::vector<float> m_energy_history;

    const vector<char const *> m_integrators = {
        "Semi Implicit", "Implicit"};
    int m_selected_integrator = 0;

    GyroSim *p_gyroSim = NULL;

    bool use_gyro = true;

    GyroGui() {
        m_w << 0.1, 10, 0;
        m_diag << 24.1449, 28.436, 118.812;
        m_energy_history.clear();

        p_gyroSim = new GyroSim();

        setSimulation(p_gyroSim);

        // show vertex velocity instead of normal
        callback_clicked_vertex =
            [&](int clickedVertexIndex, int clickedObjectIndex,
                Eigen::Vector3d &pos, Eigen::Vector3d &dir) {
                RigidObject &o = p_gyroSim->getObjects()[clickedObjectIndex];
                pos = o.getVertexPosition(clickedVertexIndex);
                dir = o.getVelocity(pos);
            };
        start();
    }

    virtual void updateSimulationParameters() override {
        // change all parameters of the simulation to the values that are set in
        // the GUI
        p_gyroSim->setTimestep(m_dt);
        p_gyroSim->setAngularVelocity(m_w.cast<double>());
        p_gyroSim->setDiagonal(m_diag.cast<double>());
        p_gyroSim->setMethod(m_selected_integrator);
    }

    virtual void drawSimulationParameterMenu() override {
        ImGui::Combo("Integrator", &m_selected_integrator, m_integrators.data(),
                     m_integrators.size());
        ImGui::InputFloat3("Angular Velocity", m_w.data(), 4);
        ImGui::InputFloat3("I_diag", m_diag.data(), 4);
        ImGui::InputFloat("dt", &m_dt, 0, 0);
    }

    virtual void drawSimulationStats() override {
        Eigen::Vector3d E = p_gyroSim->getRotationalEnergy();
        m_energy_history.push_back(E.cast<float>().cwiseAbs().sum());
        if (m_energy_history.size() > m_maxHistory)
            m_energy_history.erase(m_energy_history.begin(),
                                   m_energy_history.begin() + 1);
        ImGui::Text("E_x: %.3f", E(0));
        ImGui::Text("E_y: %.3f", E(1));
        ImGui::Text("E_z: %.3f", E(2));
        ImGui::PlotLines("Total Energy", &m_energy_history[0],
                         m_energy_history.size(), 0, NULL, 0, 300,
                         ImVec2(0, 100));
    }
};

int main(int argc, char *argv[]) {
    // create a new instance of the GUI for the spinning simulation
    new GyroGui();

    return 0;
}