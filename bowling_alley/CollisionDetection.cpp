#include "CollisionDetection.h"
#include<set>
#include<iostream>
#include "GJK.h"
#include "eigen-qp.h"

using namespace std;
using namespace Eigen;


bool cmp(std::tuple<double, string, int> &first, std::tuple<double, string, int> &sec)
{
    return get<0>(first) < get<0>(sec);
}

void CollisionDetection::computeBroadPhase(int broadPhaseMethod) {
    // compute possible collisions
    m_overlappingBodys.clear();
    
	switch (broadPhaseMethod) {
	case 0: { // none
		for (size_t i = 0; i < m_objects.size(); i++) {
			for (size_t j = i + 1; j < m_objects.size(); j++) {
				m_overlappingBodys.push_back(std::make_pair(i, j));
			}
		}
		break;
	}
     
	case 1: {  // AABB
        // compute bounding boxes
        std::vector<AABB> aabbs(m_objects.size());
        for (size_t i = 0; i < aabbs.size(); i++) {
            aabbs[i].computeAABB(m_objects[i]);
        }
        for (size_t i = 0; i < m_objects.size(); i++) {
            for (size_t j = i + 1; j < m_objects.size(); j++) {
                // add pair of objects to possible collision if their
                // bounding boxes overlap
                if (aabbs[i].testCollision(aabbs[j])) {
                    m_overlappingBodys.push_back(std::make_pair(i, j));
                }
            }
        }
        break;
    }
	
	case 2: {
		// TODO: implement other broad phases algorithm
        // sweep and prune
        //initialize
        vector<AABB> aabbs(m_objects.size());
        vector<std::tuple<double, string, int>> xMAXMIN, yMAXMIN, zMAXMIN;
        vector<vector<bool>> xOverlap = vector<vector<bool>>(m_objects.size(), vector<bool>(m_objects.size(), false));
        vector<vector<bool>> yOverlap = vector<vector<bool>>(m_objects.size(), vector<bool>(m_objects.size(), false));
        vector<vector<bool>> zOverlap = vector<vector<bool>>(m_objects.size(), vector<bool>(m_objects.size(), false));
        set<int> openIntervalsz;
        set<int> openIntervalsy;
        set<int> openIntervalsx;

        for (size_t i = 0; i < aabbs.size(); i++) {
            aabbs[i].computeAABB(m_objects[i]);
        }

        //take all endings and beginnings of intervals
        for (size_t i = 0; i < aabbs.size(); i++) {

            float x = aabbs[i].getMinCoord().x();
            xMAXMIN.push_back({x, "start", i});
            float y = aabbs[i].getMinCoord().y();
            yMAXMIN.push_back({y, "start", i});
            float z = aabbs[i].getMinCoord().z();
            zMAXMIN.push_back({z, "start", i});


            x = aabbs[i].getMaxCoord().x();
            xMAXMIN.push_back({x, "end", i});
            y = aabbs[i].getMaxCoord().y();
            yMAXMIN.push_back({y, "end", i});
            z = aabbs[i].getMaxCoord().z();
            zMAXMIN.push_back({z, "end", i});
        }

        //sort the x 
        sort(xMAXMIN.begin(), xMAXMIN.end(), cmp); 
        //put in set the beginning as long as not closed, the next open intervals overlap with it
        // store all overlaps   
        for (size_t i=0; i<xMAXMIN.size(); i++) {
            if (get<1>(xMAXMIN[i]) == "start") {   
                for (int openInd : openIntervalsx) {
                    xOverlap[get<2>(xMAXMIN[i])][openInd] = true;
                    xOverlap[openInd][get<2>(xMAXMIN[i])] = true;
                }
                openIntervalsx.insert(get<2>(xMAXMIN[i]));
            }
            else if(get<1>(xMAXMIN[i]) == "end"){
                 openIntervalsx.erase(get<2>(xMAXMIN[i]));
            }
        }

        // do the same for y
        sort(yMAXMIN.begin(), yMAXMIN.end(), cmp);
        for (size_t i=0; i<yMAXMIN.size(); i++) {
            if (get<1>(yMAXMIN[i]) == "start") {   
                for (int openInd : openIntervalsy) {
                    yOverlap[get<2>(yMAXMIN[i])][openInd] = true;
                    yOverlap[openInd][get<2>(yMAXMIN[i])] = true;
                }
                openIntervalsy.insert(get<2>(yMAXMIN[i]));
            }
            else if(get<1>(xMAXMIN[i]) == "end"){
                openIntervalsy.erase(get<2>(yMAXMIN[i]));
            }  
        }

        //do the same for z
        sort(zMAXMIN.begin(), zMAXMIN.end(), cmp);
        for (size_t i=0; i<zMAXMIN.size(); i++) {
            if (get<1>(zMAXMIN[i]) == "start") {   
                for (int openInd : openIntervalsz) {
                    zOverlap[get<2>(zMAXMIN[i])][openInd] = true;
                    zOverlap[openInd][get<2>(zMAXMIN[i])] = true;
                }
                openIntervalsz.insert(get<2>(zMAXMIN[i]));
            }
            else if(get<1>(xMAXMIN[i]) == "end"){
                openIntervalsz.erase(get<2>(zMAXMIN[i]));
            }
        }

        //if they overlap in all dimensions then they are colliding
        for (size_t i = 0; i < m_objects.size(); i++) {
            for (size_t j = i + 1; j < m_objects.size(); j++) {
                if (xOverlap[i][j] && yOverlap[i][j] && zOverlap[i][j]) 
                    m_overlappingBodys.push_back(make_pair(i, j));
            }
        }

		break;
	}
	}
}



void CollisionDetection::computeNarrowPhase(int narrowPhaseMethod) {
    switch (narrowPhaseMethod) {
    case 0: {
        // exhaustive
        // iterate through all pairs of possible collisions
        for (auto overlap : m_overlappingBodys) {
            std::vector<Contact> temp_contacts[2];
            // compute intersection of a with b first and intersectino
            // of b with a and store results in temp_contacts
            for (int switcher = 0; switcher < 2; switcher++) {
                RigidObject* a =
                    &m_objects[(!switcher) ? overlap.first
                                            : overlap.second];
                RigidObject* b =
                    &m_objects[(!switcher) ? overlap.second
                                            : overlap.first];

                Eigen::MatrixXd Va, Vb;
                Eigen::MatrixXi Fa, Fb;
                a->getMesh(Va, Fa);
                b->getMesh(Vb, Fb);

                // iterate through all faces of first object
                for (int face = 0; face < Fa.rows(); face++) {
                    // iterate through all edges of given face
                    for (size_t j = 0; j < 3; j++) {
                        int start = Fa(face, j);
                        int end = Fa(face, (j + 1) % 3);

                        // check if there is a collision
                        ContactType ct = isColliding(
                            Va.row(start), Va.row(end), Vb, Fb);

                        // find collision and check for duplicates
                        switch (ct) {
                            case ContactType::VERTEXFACE: {
                                auto ret = m_penetratingVertices.insert(
                                    Fa(face, j));
                                // if not already in set
                                if (ret.second) {
                                    Contact temp_col =
                                        findVertexFaceCollision(
                                            Va.row(Fa(face, j)), Vb,
                                            Fb);
                                    temp_col.a = a;
                                    temp_col.b = b;
                                    temp_col.type =
                                        ContactType::VERTEXFACE;
                                    temp_contacts[switcher].push_back(
                                        temp_col);
                                }
                                break;
                            }
                            case ContactType::EDGEEDGE: {
                                int orderedStart = std::min(start, end);
                                int orderedEnd = std::max(start, end);
                                auto ret = m_penetratingEdges.insert(
                                    std::make_pair(orderedStart,
                                                    orderedEnd));
                                // if not already in set
                                if (ret.second) {
                                    Contact temp_col =
                                        findEdgeEdgeCollision(
                                            Va.row(orderedStart),
                                            Va.row(orderedEnd), Vb, Fb);
                                    temp_col.a = a;
                                    temp_col.b = b;
                                    temp_col.type =
                                        ContactType::EDGEEDGE;
                                    temp_contacts[switcher].push_back(
                                        temp_col);
                                }
                                break;
                            }
                            case ContactType::NONE: {
                                break;
                            }
                        }
                    }
                }
            }

            // look for vertexFace
            bool found = false;
            for (int i = 0; i < 2; i++) {
                for (auto cont : temp_contacts[i]) {
                    if (cont.type == ContactType::VERTEXFACE) {
                        m_contacts.push_back(cont);
                        found = true;
                        break;
                    }
                }
                if (found) {
                    break;
                }
            }
            if (found) {
                continue;
            }

            // take single edgeedge if possible
            if (temp_contacts[0].size() > 0 &&
                temp_contacts[0].size() < temp_contacts[1].size()) {
                for (int i = 0; i < temp_contacts[0].size(); i++) {
                    m_contacts.push_back(temp_contacts[0][i]);
                }
            } else if (temp_contacts[1].size() > 0 &&
                        temp_contacts[0].size() >
                            temp_contacts[1].size()) {
                for (int i = 0; i < temp_contacts[1].size(); i++) {
                    m_contacts.push_back(temp_contacts[1][i]);
                }
            } else if (temp_contacts[0].size() > 0) {
                for (int i = 0; i < temp_contacts[0].size(); i++) {
                    m_contacts.push_back(temp_contacts[0][i]);
                }
            } else if (temp_contacts[1].size() > 0) {
                for (int i = 0; i < temp_contacts[1].size(); i++) {
                    m_contacts.push_back(temp_contacts[1][i]);
                }
            }
        }
        break;
    }

		case 1: {
        // iterate through all pairs of possible collisions
        for (auto overlap : m_overlappingBodys) {
            RigidObject* a = &m_objects[overlap.first];
            RigidObject* b = &m_objects[overlap.second];
        
            GJK gjkClass(*a, *b);
            // initial search direction
            gjkClass.setDirection({0,0,-1});
  
            Contact contact;
            contact.a = a;
            contact.b = b;
            if(gjkClass.GJKDetect(contact.p, contact.n)) {
                m_contacts.push_back(contact);
            }
        }
		break;
	}
    }
}

void CollisionDetection::compute_a_matrix(Eigen::MatrixXd &ABig){

        for(int i = 0; i < m_contacts.size(); i++){
            for(int j = 0; j < m_contacts.size(); j++){
                ABig(i, j)= compute_aij(m_contacts[i], m_contacts[j]);
            }
        }

}

double CollisionDetection::compute_aij(Contact ci, Contact cj){


    if (ci.a != cj.a &&
        ci.b != cj.b &&
        ci.a != cj.b &&
        ci.b != cj.a)
    {
        return 0;
    }
    else{

        RigidObject *A = ci.a;
        RigidObject *B = ci.b;
        Eigen::Vector3d ni = ci.n,
            nj = cj.n,
            pi = ci.p,
            pj = cj.p;
        Eigen::Vector3d ra = pi - A->getPosition(),
                        rb = pi - B->getPosition();
        Eigen::Vector3d force_on_a = Eigen::Vector3d::Zero(),
                torque_on_a = Eigen::Vector3d::Zero();

        if (cj.a == ci.a){
            force_on_a = nj;
            torque_on_a = (pj - A->getPosition()).cross(nj);
        }
        else if (cj.b == ci.a){
            force_on_a = -nj;
            torque_on_a = (pj - A->getPosition()).cross(nj);
        }
        Eigen::Vector3d force_on_b = Eigen::Vector3d::Zero(),
        torque_on_b = Eigen::Vector3d::Zero();
        if (cj.a == ci.b){
            force_on_b = nj;
            torque_on_b = (pj - B->getPosition()).cross(nj);
        }
        else if (cj.b == ci.b){
            force_on_b = -nj;
            torque_on_b = (pj - B->getPosition()).cross(nj);
        }

        Eigen::Vector3d a_linear = force_on_a / A->getMass(),
            a_angular = (A->getInertiaInvWorld() * torque_on_a).cross(ra);
        Eigen::Vector3d b_linear = force_on_b / B->getMass(),
                    b_angular = (B->getInertiaInvWorld() * torque_on_b).cross(rb);
        return ni.dot((a_linear + a_angular) - (b_linear + b_angular));
    }    
}

Eigen::Vector3d CollisionDetection::compute_ndot(Contact *c)
{
    if (c->type == ContactType::VERTEXFACE)
    {
        return c->b->getAngularVelocity().cross(c->n);
    }
    else
    {
        Eigen::Vector3d eadot = c->a->getAngularVelocity().cross(c->ea);
        Eigen::Vector3d ebdot = c->b->getAngularVelocity().cross(c->eb);
        Eigen::Vector3d n1 = c->ea.cross(c->eb);
        Eigen::Vector3d z = eadot.cross(c->eb) + c->ea.cross(ebdot);
        double l = n1.norm();
        n1.normalize();
        return (z - (z.dot(n1)) * n1) / l;
    }
}

inline Eigen::Vector3d CollisionDetection::pt_velocity(RigidObject* body, Eigen::Vector3d p){
    return body->getLinearVelocity() + ((body->getAngularVelocity()).cross(p - body->getPosition()));
}


void CollisionDetection::compute_b_vector(VectorXd &bvec){
    /* for (int i = 0; i < m_contacts.size(); i++){
        b(i) = (m_contacts[i].n).dot(m_contacts[i].a->getVelocity(m_contacts[i].p) - m_contacts[i].b->getVelocity(m_contacts[i].p));
        b(i) = (1+FLT_EPSILON)* b(i);

    }*/
        for (size_t i = 0; i < m_contacts.size(); i++)
    {
        Contact *c = &m_contacts[i];
        RigidObject *A = c->a;
        RigidObject *B = c->b;

        Eigen::Vector3d n = c->n;
        Eigen::Vector3d ra = c->p - A->getPosition(),
                        rb = c->p - B->getPosition();

        Eigen::Vector3d f_ext_a = A->getForce(),
                        f_ext_b = B->getForce(),
                        t_ext_a = A->getTorque(),
                        t_ext_b = B->getTorque();
        Eigen::Vector3d a_ext_part, a_vel_part,
            b_ext_part, b_vel_part;

        a_ext_part = f_ext_a / A->getMass() +
                     ((A->getInertiaInvWorld() * t_ext_a).cross(ra));
        b_ext_part = f_ext_b / B->getMass() +
                     ((B->getInertiaInvWorld() * t_ext_b).cross(rb));

        a_vel_part = (A->getAngularVelocity().cross(A->getAngularVelocity().cross(ra))) + ((A->getInertiaInvWorld() * (A->getAngularMomentum().cross(A->getAngularVelocity()))).cross(ra));
        b_vel_part = (B->getAngularVelocity().cross(B->getAngularVelocity().cross(rb))) + ((B->getInertiaInvWorld() * (B->getAngularMomentum().cross(B->getAngularVelocity()))).cross(rb));
        double k1 = n.dot((a_ext_part + a_vel_part) - (b_ext_part + b_vel_part));
        Eigen::Vector3d ndot = compute_ndot(c);
        double k2 = 2. * ndot.dot(pt_velocity(A, c->p) - pt_velocity(B, c->p));
        bvec(i) = k1 + k2;
    }



}

void CollisionDetection::findSimultaneousJ(Eigen::MatrixXd &A, VectorXd &b, VectorXd &J){
    compute_a_matrix(A);
    compute_b_vector(b);
    int n = m_contacts.size();

    EigenQP::QPIneqSolver<double, -1, -1> LPCSolver(n, n);

    Eigen::MatrixXd Q(m_contacts.size(), m_contacts.size());
    Q = 2 * A;

    A= -A;
    LPCSolver.solve(Q, b, A, b, J);

}




void CollisionDetection::applyImpulse(double eps) {

    
    // compute impulse for all simultaneous contacts
    Eigen::MatrixXd A(m_contacts.size(), m_contacts.size());
    VectorXd b(m_contacts.size());

    VectorXd J(m_contacts.size());

    //finds the impulses
    findSimultaneousJ(A, b, J);

    int i = 0;
    for (auto contact : m_contacts) {
        Eigen::Vector3d vrel_vec = contact.a->getVelocity(contact.p) - contact.b->getVelocity(contact.p);
        double vrel = contact.n.dot(vrel_vec);
        Eigen::Vector3d r_b = (contact.p) - contact.a->getPosition();
        Eigen::Vector3d r_a = (contact.p) - contact.b->getPosition();
        // moving away
        if (vrel > 1.e-12) {
            continue;
        }

        //resting contacts
        else if (vrel > -1 * 1.e-12){
            contact.a->setLinearMomentum(contact.a->getLinearMomentum() + J(i) * contact.n);
            contact.b->setLinearMomentum(contact.a->getLinearMomentum() - J(i) * contact.n);
            contact.a->setAngularMomentum(contact.a->getAngularMomentum() + r_a.cross(J(i) * contact.n));
            contact.b->setAngularMomentum(contact.b->getAngularMomentum() - r_b.cross(J(i) * contact.n));
            continue;
        }
        //collision response
        Eigen::Vector3d numerator = -1 * (1 + eps) * vrel_vec;
        double term1 = contact.a->getMassInv();
        double term2 = contact.b->getMassInv();
        double term3 = contact.n.dot((contact.a->getInertiaInvWorld() * (r_a.cross(contact.n))).cross(r_a));
        double term4 = contact.n.dot((contact.b->getInertiaInvWorld() * (r_b.cross(contact.n))).cross(r_b));

        Eigen::Vector3d j = numerator / (term1 + term2 + term3 + term4);
        Eigen::Vector3d force = j.dot(contact.n) * contact.n;
        contact.a->setLinearMomentum(contact.a->getLinearMomentum() + force);
        contact.b->setLinearMomentum(contact.a->getLinearMomentum() - force);

        contact.a->setAngularMomentum(contact.a->getAngularMomentum() + r_a.cross(force));
        contact.b->setAngularMomentum(contact.b->getAngularMomentum() - r_b.cross(force));

        i++;

        
    }
    /*

    
        
    for (auto contact : m_contacts) {
        Eigen::Vector3d vrel_vec = contact.a->getVelocity(contact.p) -
        contact.b->getVelocity(contact.p);
        double vrel = contact.n.dot(vrel_vec);
        if (vrel > 0) {
            // bodies are moving apart
            continue;
        }

        // Computing Impulse
        Eigen::Vector3d numerator = -1.0 * (1.0 + eps) * vrel * contact.n;


        double masses = contact.a->getMassInv() + contact.b->getMassInv();

        Eigen::Vector3d r_a = (contact.p) - contact.a->getPosition();
        Eigen::Vector3d interproda = contact.a->getInertiaInv() * r_a.cross(contact.n);
        Eigen::Vector3d crossproda = interproda.cross(r_a);
        double val_proda = contact.n.dot(crossproda);


        Eigen::Vector3d r_b = (contact.p) - contact.a->getPosition();
        Eigen::Vector3d interprodb = contact.b->getInertiaInv() * r_b.cross(contact.n);
        Eigen::Vector3d crossprodb = interprodb.cross(r_b);
        double val_prodb = contact.n.dot(crossprodb);


        Eigen::Vector3d impulse = numerator / (masses + val_proda + val_prodb);

        contact.a->setLinearMomentum(contact.a->getLinearMomentum() + impulse);
        contact.b->setLinearMomentum(contact.b->getLinearMomentum() - impulse);
        contact.a->setAngularMomentum(contact.a->getAngularMomentum() + r_a.cross(impulse));
        contact.b->setAngularMomentum(contact.b->getAngularMomentum() + r_b.cross(-impulse));
    }
    */
    

}
