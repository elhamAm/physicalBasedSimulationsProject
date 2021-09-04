#include <igl/writeOFF.h>
#include "BowlingAlleySim.h"
#include "Simulator.h"
//#include "Gui.h"

class CollisionGui {
   public:
    float m_angle = 1.047f;
    float m_force = 1000.0f;//you're sending the ball off with much more force
    float m_dt = 1e-3;
    float m_mass = 1.0;
    bool m_showContacts = false;
    float m_eps = 1.0;

    const vector<char const*> m_methods = { "Matrix", "Quaternion" };
    int m_selected_method = 0;//so as to use quaternoid

    int m_maxHistory = 200;
	int m_simSpeed = 150;
    std::vector<float> m_energy_history;
	Simulator *p_simulator = NULL;

    int m_selectedBroadPhase = 1;
    const std::vector<char const *> m_broadphases = {"None", "AABB", "Own"};
    int m_selectedNarrowPhase = 0;
    const std::vector<char const *> m_narrowphases = {"Exhaustive", "Own"};

    BowlingAlleySim *p_BowlingAlleySim = NULL;
	virtual void updateSimulationParameters() {
		
        p_BowlingAlleySim->setMethod(m_selected_method);
        p_BowlingAlleySim->setForce(m_force);
        p_BowlingAlleySim->setAngle(m_angle);
        p_BowlingAlleySim->setTimestep(m_dt);
        p_BowlingAlleySim->setMass(m_mass);
        p_BowlingAlleySim->setBroadPhaseMethod(m_selectedBroadPhase);
        p_BowlingAlleySim->setNarrowPhaseMethod(m_selectedNarrowPhase);
        p_BowlingAlleySim->setEps(m_eps);
		
		p_BowlingAlleySim->setTimestep(m_dt);


    }
	void setSimulation(Simulation *sim) {
		p_simulator = new Simulator(sim);
		p_simulator->reset();
		p_simulator->setSimulationSpeed(m_simSpeed);
	}

    CollisionGui(int argc, char *argv[]) {
        p_BowlingAlleySim = new BowlingAlleySim(argc, argv);
        setSimulation(p_BowlingAlleySim);
		updateSimulationParameters();
		p_BowlingAlleySim->resetMembers();

        //setFastForward(true); // no delay

        // show vertex velocity instead of normal
		/*
        callback_clicked_vertex = [&](int clickedVertexIndex,
                                      int clickedObjectIndex,
                                      Eigen::Vector3d &pos,
                                      Eigen::Vector3d &dir) {
            RigidObject &o = p_BowlingAlleySim->getObjects()[clickedObjectIndex];
            pos = o.getVertexPosition(clickedVertexIndex);
            dir = o.getVelocity(pos);
        };*/
        //start();
    }



/*
    virtual void clearSimulation() override {
        p_BowlingAlleySim->showContacts(false);
        p_BowlingAlleySim->showContacts(m_showContacts);
    }

    virtual void drawSimulationParameterMenu() override {
        ImGui::Combo("Method", &m_selected_method, m_methods.data(),
            m_methods.size());
        ImGui::SliderAngle("Angle", &m_angle, -180.0f, 180.0f);
        ImGui::InputFloat("Force", &m_force, 0, 0);
        ImGui::InputFloat("Mass", &m_mass, 0, 0);
        ImGui::InputFloat("dt", &m_dt, 0, 0);
        if (ImGui::Checkbox("Show contacts", &m_showContacts)) {
            p_BowlingAlleySim->showContacts(m_showContacts);
        }
        if (ImGui::Combo("Broadphase", &m_selectedBroadPhase,
                         m_broadphases.data(), m_broadphases.size())) {
            p_BowlingAlleySim->setBroadPhaseMethod(m_selectedBroadPhase);
        }
        if (ImGui::Combo("Narrowphase", &m_selectedNarrowPhase,
                         m_narrowphases.data(), m_narrowphases.size())) {
            p_BowlingAlleySim->setNarrowPhaseMethod(m_selectedNarrowPhase);
        }
        ImGui::InputFloat("eps", &m_eps, 0, 0);
    }

    virtual void drawSimulationStats() override {
        Eigen::Vector3d E = p_BowlingAlleySim->getKineticEnergy();
        m_energy_history.push_back(E.cast<float>().cwiseAbs().sum());
        if (m_energy_history.size() > m_maxHistory)
            m_energy_history.erase(m_energy_history.begin(),
                                   m_energy_history.begin() + 1);
        ImGui::Text("E: %.3f, %.3f, %.3f", E(0), E(1), E(2));
        ImGui::PlotLines("Total Energy", &m_energy_history[0],
                         m_energy_history.size(), 0, NULL, 0, 1000,
                         ImVec2(0, 200));
        Eigen::Vector3d p = p_BowlingAlleySim->getLinearMomentum();
        ImGui::Text("M: %.3f, %.3f, %.3f", p(0), p(1), p(2));
        Eigen::Vector3d l = p_BowlingAlleySim->getAngularMomentum();
        ImGui::Text("L: %.3f, %.3f, %.3f", l(0), l(1), l(2));
    }
	*/
	//private:
        //double m_dt = 1e-3;           // timestep
        //double m_eps = 1;             // should be one, we guess
        //double m_impactVelocity = 10; // impact velocity
        //double m_massFactor = 1.0;    // mass factor

};

int main(int argc, char *argv[]) {

	int numberOfRecordings = std::stoi(argv[1]);
	int numberBeforeRec = std::stoi(argv[2]);

	cout << "number of recordings: "<< numberOfRecordings << endl;

    CollisionGui *GUI = new CollisionGui(argc, argv);
	GUI->updateSimulationParameters();
	GUI->p_BowlingAlleySim->resetMembers();
	int ind = 0;
	for(int i = 0; i < numberBeforeRec + numberOfRecordings; i++){
		std::cout << "time step: " << i << endl;
		GUI->p_BowlingAlleySim->advance();
		if(i >= numberBeforeRec){
			GUI->p_simulator->storeRecord(ind);
			ind++;
		}
	}




    return 0;
}