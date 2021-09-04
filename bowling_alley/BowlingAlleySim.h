
#include "CollisionDetection.h"
#include "InitialConfiguration.h"
#include "Simulation.h"
#include <string.h>

#include <deque>

using namespace std;

/*
 * Simulation of bowling alley.
 */

class BowlingAlleySim : public Simulation{
   public:
   BowlingAlleySim(int argc, char *argv[]) : Simulation(), m_collisionDetection(m_objects) { init(); }

	double offset = 1;
	int NUM_PINS = 0;
	bool planeExists = true;
	std::string configPara = "2rows";
	std::string pinType = "ovoid";
	InitialConfiguration configuration = InitialConfiguration();

    virtual void init() override {
		std::string pathPin;
			/****   pin type   ****/
		if(pinType == "ovoid"){
			pathPin = "data_ovoid.obj";
		}
		else if(pinType == "sphere"){
			pathPin = "sphere.off";
		}
		else if(pinType == "cube"){
			pathPin = "cube.off";
		}
		else if(pinType == "cuboid"){
			pathPin = "data_cuboid.obj";
		}
		
		std::string pathPlane = "plane.obj";
		std::string pathSphere = "sphere.off";

		cout << "before" << endl;
		configuration = InitialConfiguration(configPara, NUM_PINS, offset);
		cout << "pin number: " << NUM_PINS << endl;
		cout << "after" << endl;
		

		m_objects.push_back(RigidObject(pathSphere));
		for (int i = 1; i <= NUM_PINS; i++) {
			m_objects.push_back(RigidObject(pathPin));
		}
		configuration.scale(m_objects, pinType);

		if (planeExists) m_objects.push_back(RigidObject(pathPlane));


 		/********************************/
		/***           SCALE          ***/
		/********************************/

		cout << "scaling" << endl;
		if (planeExists) {
			m_objects[m_objects.size() - 1].setScale(10);
			m_objects[m_objects.size() - 1].setType(ObjType::STATIC);
		}


		m_collisionDetection.setObjects(m_objects);

		m_dt = 1e-3 * 3;
		m_gravity << 0, -9.8, 0;
		m_pin_mass = 1.0;
		m_ball_mass = 10.0;
		m_showContacts = false;
		m_broadPhaseMethod = 0;
		m_narrowPhaseMethod = 0;
		m_eps = 1.0;

		reset();

    }

    virtual void resetMembers() override {
        for (auto &o : m_objects) {
            o.reset();
        }

        /********************************/
        /***  Rotation and position   ***/
        /********************************/
		// ball
        m_objects[0].setRotation(Eigen::Quaterniond(0, -0.3444844, -0.3444844, -0.8733046));
		m_objects[0].setPosition(Eigen::Vector3d(-8, 0.6+offset, 2));

		// pins
        //for(int i = 1; i <= NUM_PINS; i++){
        //    m_objects[i].setRotation(Eigen::Quaterniond(0.1, -0.1, 0, 0));
        //}

        configuration.fillPositions(m_objects);

		/********************************/
        /***           COLORS         ***/
        /********************************/
		
		Eigen::RowVector3d c0(204.0 / 255.0, 0, 0);
		Eigen::RowVector3d c1(0, 128.0 / 255.0, 204.0 / 255.0);
		for (int i = 1; i <= NUM_PINS; i++) {
            double a = double(i+1) / NUM_PINS;
            m_objects[i].setColors(c0*a + c1*(1 - a));
		}

        if(planeExists)
            m_objects[m_objects.size()-1].setColors(c0*3/10+ c1*(1 - 3/10));

        /********************************/
        /***           MASS          ***/
        /********************************/

        //mass of ball
        m_objects[0].setMass(1);
        //mass of pins
        
        for (size_t i = 1; i <= NUM_PINS; i++) {
            m_objects[i].setMass(m_pin_mass);
        }
        //mass of plane
        if(planeExists)
            m_objects[m_objects.size()-1].setMass(10);

        updateVars();
    }

    virtual void updateRenderGeometry() override {
        for (size_t i = 0; i < m_objects.size(); i++) {
            RigidObject &o = m_objects[i];
            if (o.getID() < 0) {
                m_renderVs.emplace_back();
                m_renderFs.emplace_back();
            }

            m_objects[i].getMesh(m_renderVs[i], m_renderFs[i]);
        }
    }

    virtual bool advance() override;

    virtual void renderRenderGeometry(
        igl::opengl::glfw::Viewer &viewer) override {
        for (size_t i = 0; i < m_objects.size(); i++) {
            RigidObject &o = m_objects[i];
            if (o.getID() < 0) {
                int new_id = 0;
                if (i > 0) {
                    new_id = viewer.append_mesh();
                    o.setID(new_id);
                } else {
                    o.setID(new_id);
                }

                size_t meshIndex = viewer.mesh_index(o.getID());
				if (i == 0) {
					viewer.data_list[meshIndex].show_lines = true;
					viewer.data_list[meshIndex].show_faces = false;
				}
				else {
					viewer.data_list[meshIndex].show_lines = false;
				}
                viewer.data_list[meshIndex].set_face_based(true);
                viewer.data_list[meshIndex].point_size = 2.0f;
                viewer.data_list[meshIndex].clear();
            }
            size_t meshIndex = viewer.mesh_index(o.getID());

            viewer.data_list[meshIndex].set_mesh(m_renderVs[i], m_renderFs[i]);
            viewer.data_list[meshIndex].compute_normals();

            Eigen::MatrixXd color;
            o.getColors(color);
            viewer.data_list[meshIndex].set_colors(color);
        }

        if (m_showContacts) {
            // number of timesteps to keep showing collision
            int delay = 10;

            // clear old points
            viewer.data_list[1].points = Eigen::MatrixXd(0, 6);
            viewer.data_list[1].point_size = 10.0f;

            // remove expired points
            while (m_contactMemory.size() > 0 &&
                   m_contactMemory.front().second + delay < m_step) {
                m_contactMemory.pop_front();
            }

            // get new points and add them to memory
            auto contacts = m_collisionDetection.getContacts();
            for (auto &contact : contacts) {
                m_contactMemory.push_back(std::make_pair(contact, m_step));
            }

            // show points
            for (auto &contact_int_p : m_contactMemory) {
                viewer.data_list[1].add_points(
                    contact_int_p.first.p.transpose(),
                    (contact_int_p.first.type == ContactType::EDGEEDGE)
                        ? Eigen::RowVector3d(0, 1, 0)
                        : Eigen::RowVector3d(0, 0, 1));
            }
        }
    }

#pragma region SettersAndGetters
    void setMethod(int m) { m_method = m; }
    /*
     * Compute magnitude and direction of momentum and apply it to o
     */
    void updateVars() {
        Eigen::Vector3d momentum;
        //momentum << std::sin(m_angle), std::cos(m_angle), 0;
        momentum << std::sin(90), 0, 0;
        momentum *= m_force;
        m_objects[0].setLinearMomentum(momentum);
    }

    void setAngle(double a) {
        m_angle = a;
        updateVars();
    }

    void setForce(double f) {
        m_force = f;
        updateVars();
    }

    void setMass(double m) { m_pin_mass = m; }

    void showContacts(bool s) {
        if (!s) {
            m_contactMemory.clear();
        }
        m_showContacts = s;
    }

    void setBroadPhaseMethod(int m) { m_broadPhaseMethod = m; }
    void setNarrowPhaseMethod(int m) { m_narrowPhaseMethod = m; }

    void setEps(double eps) { m_eps = eps; }

    Eigen::Vector3d getKineticEnergy() {
        Eigen::Vector3d res;
        res.setZero();
        for (auto o : m_objects) {
            if (o.getType() == ObjType::STATIC) continue;
            Eigen::Vector3d rotE = 0.5 * o.getInertia().diagonal().cwiseProduct(
                                             o.getAngularVelocity());
            Eigen::Vector3d kinE =
                0.5 * o.getMass() * o.getLinearVelocity().array().square();
            res += rotE + kinE;
        }
        return res;
    }

    Eigen::Vector3d getLinearMomentum() {
        Eigen::Vector3d res;
        res.setZero();
        for (auto o : m_objects) {
            if (o.getType() == ObjType::STATIC) continue;
            res += o.getLinearMomentum();
        }
        return res;
    }

    Eigen::Vector3d getAngularMomentum() {
        Eigen::Vector3d res;
        res.setZero();
        for (auto o : m_objects) {
            if (o.getType() == ObjType::STATIC) continue;
            res += o.getAngularMomentum();
        }
        return res;
    }


    Eigen::Vector3d getCurrentCOM();

    Eigen::Vector3d getVariousMagnitudes();

#pragma endregion SettersAndGetters

   private:
    int m_method;
    double m_angle;
    double m_force;
    double m_pin_mass;
	double m_ball_mass;

	

    Eigen::Vector3d m_gravity;

    int m_broadPhaseMethod;
    int m_narrowPhaseMethod;
	/**/
	std::vector<Eigen::MatrixXd> m_renderVs;  // vertex positions for rendering
    std::vector<Eigen::MatrixXi> m_renderFs;  // face indices for rendering

    double m_centerMass;
    double m_impactVelocity;
    Eigen::Vector3d m_currentCOM;

    double m_eps;


    CollisionDetection m_collisionDetection;
    bool m_showContacts;
    std::deque<std::pair<Contact, int>> m_contactMemory;

    std::string m_filePath;


};