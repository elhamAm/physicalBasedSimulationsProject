#ifndef COLLISIONDETECTION_H
#define COLLISIONDETECTION_H

#include <igl/ray_mesh_intersect.h>
#include <set>
#include <utility>
#include <vector>
#include "AABB.h"
#include "RigidObject.h"
#include <float.h>

using namespace std;
using namespace Eigen;
enum class ContactType { VERTEXFACE, EDGEEDGE, NONE };

struct Contact {
    RigidObject* a;      // body containing vertex
    RigidObject* b;      // body containing face
    Eigen::Vector3d n;   // world-space vertex location
    Eigen::Vector3d p;   // outwards pointing normal of face
    Eigen::Vector3d ea;  // edge direction for A
    Eigen::Vector3d eb;  // edge direction for B

    ContactType type;  // type of contact
};

class CollisionDetection {
   public:
    CollisionDetection(std::vector<RigidObject>& world) : m_objects(world) {}

    // pass objects in scene to collision detection
    void setObjects(std::vector<RigidObject>& world) { m_objects = world; }

	void computeBroadPhase(int broadPhaseMethod);

    // test if ray from start towards end intersects object with vertices V and
    // faces F
    ContactType isColliding(const Eigen::Vector3d& start,
                            const Eigen::Vector3d& end,
                            const Eigen::MatrixXd& V,
                            const Eigen::MatrixXi& F) {
        std::vector<igl::Hit> hits;
        igl::ray_mesh_intersect(start, end - start, V, F, hits);

        // count number of intersections with object that happen between start
        // and end (so along the edge)
        int cntr = 0;
        for (auto hit : hits) {
            if (hit.t > 0 && hit.t <= 1) {
                cntr++;
            }
        }

        // if hits is odd then the ray enters through one face and
        // does not leave again through another, hence the starting point is
        // inside the object, if the number of intersections between start and
        // end of edge are even, then the edge entering through one and leaving
        // through another face, hence the edge is intersection the object
        ContactType ret = ContactType::NONE;
        if (hits.size() % 2 == 1) {
            ret = ContactType::VERTEXFACE;
        }
        if (cntr % 2 == 0 && cntr > 0) {
            ret = ContactType::EDGEEDGE;
        }
        return ret;
    }

    // compute normal for all faces and use it to find closesest face to
    // given vertex
    Contact findVertexFaceCollision(const Eigen::Vector3d& vertex,
                                    const Eigen::MatrixXd& V,
                                    const Eigen::MatrixXi& F) {
        double minDist = std::numeric_limits<double>::infinity();
        Eigen::Vector3d minNormal;
        for (int i = 0; i < F.rows(); i++) {
            Eigen::Vector3d a = V.row(F(i, 0));
            Eigen::Vector3d b = V.row(F(i, 1));
            Eigen::Vector3d c = V.row(F(i, 2));

            Eigen::Vector3d n = -(b - a).cross(c - a).normalized();

            Eigen::Vector3d v = vertex - a;

            double distance = v.dot(n);
            // if vertex inside
            if (distance >= 0) {
                if (distance < minDist) {
                    minDist = distance;
                    minNormal = -n;
                }
            }
        }
        Contact ret;
        ret.n = minNormal;
        ret.p = vertex + minDist * minNormal;
        return ret;
    }

        Eigen::Vector3d support(RigidObject A, RigidObject B, Eigen::Vector3d d) {
       //cout << "in support" << endl;   
      Eigen::Vector3d p1 = getFarthestPointInDirection(A, d);
      Eigen::Vector3d p2 = getFarthestPointInDirection(B, d);

      return p1 - p2;
    }


    bool containsOrigin(std::vector<Eigen::Vector3d>& simplex, Eigen::Vector3d& d) {
      Eigen::Vector3d a = simplex.back();
      //cout << "a: " << a << endl;
      //cout << "size of simplex: " << simplex.size() << endl;
      Eigen::Vector3d ao = -a;

      if (simplex.size() == 3) {
        //cout << "triangle case" << endl;
        // triangle case
        Eigen::Vector3d b = simplex[1];
        Eigen::Vector3d c = simplex[0];
        //cout << "b: " << b << endl;
        //cout << "c: " << c << endl;

        // edges
        Eigen::Vector3d ab = b - a;
        Eigen::Vector3d ac = c - a;

        // normals
        Eigen::Vector3d abPerp = ac.cross(ab).cross(ab);
        Eigen::Vector3d acPerp = ab.cross(ac).cross(ac);

        if (abPerp.dot(ao) > 0.3) {
          // remove point c
          //cout << "size of simplex before: " << simplex.size() << endl;
          simplex.erase(simplex.begin());
          //cout << "size of simplex after: " << simplex.size() << endl;
          d = abPerp;
        }
        else if (acPerp.dot(ao) > 0.3) {
          //cout << "size of simplex before: " << simplex.size() << endl;
          simplex.erase(simplex.begin() + 1);
          //cout << "size of simplex after: " << simplex.size() << endl;
          d = acPerp;
        }
        else {
          return true;
        }
      }
      else {
        //cout << "line case" << endl;
        // line segment case
        Eigen::Vector3d b = simplex[1];
        Eigen::Vector3d ab = b - a;

        Eigen::Vector3d abPerp = ab.cross(ao).cross(ab);
        d = abPerp;
      }
      return false;
    }

    Eigen::Vector3d getFarthestPointInDirection(RigidObject object, Eigen::Vector3d direction) {
      float max = 0;
      int index = 0;

      Eigen::MatrixXd vertices;
      Eigen::MatrixXi F;
      object.getMesh(vertices, F);


      for (int i = 0; i < vertices.rows(); i++) {
        Eigen::VectorXd vertex(3);
        for (int j = 0; j < vertices.cols(); j++) {
          vertex(j) = vertices(i, j);
        }

        float dot = vertex.dot(direction);
        if (dot > max) {
          max = dot;
          index = i;
        }
      }

      Eigen::VectorXd farthestVertex(3);
      for (int j = 0; j < vertices.cols(); j++) {
        farthestVertex(j) = vertices(index, j);
      }
      return farthestVertex;
    }





Eigen::Vector3d getFarthestPointInDirectionElham(RigidObject object, Eigen::Vector3d direction) {

        //Vector3[] vertices;
        Eigen::MatrixXd vertices;
        Eigen::MatrixXi F;
        //Vector3 farthestPoint;
        Eigen::VectorXd farthestPoint;
        float farDistance;

        object.getMesh(vertices, F);
        farDistance=0;
       // for (int i = 0; i < vertices.rows(); i++){
          //float temp = direction.dot(vertices(i));
          //if(temp > farDistance){
           // farDistance = temp;
            //farthestPoint = vertices(i);
          //} 
       // }
        return farthestPoint;
}

    // loop over edges of faces and compute closest to given edge
    Contact findEdgeEdgeCollision(const Eigen::Vector3d& start,
                                  const Eigen::Vector3d& end,
                                  const Eigen::MatrixXd& V,
                                  const Eigen::MatrixXi& F) {
        double minDist = std::numeric_limits<double>::infinity();
        Contact ret;
        for (int i = 0; i < F.rows(); i++) {
            Eigen::Vector3d a = V.row(F(i, 0));
            Eigen::Vector3d b = V.row(F(i, 1));
            Eigen::Vector3d c = V.row(F(i, 2));

            Eigen::Vector3d n_face = -(b - a).cross(c - a).normalized();

            for (int j = 0; j < 3; j++) {
                Eigen::Vector3d s = V.row(F(i, j));
                Eigen::Vector3d e = V.row(F(i, (j + 1) % 3));

                Eigen::Vector3d ea = end - start;
                Eigen::Vector3d eb = e - s;
                Eigen::Vector3d n =
                    (ea).cross(eb);  // direction of shortest distance
                double distance = n.dot(start - s) / n.norm();

                Eigen::Vector3d plane_normal = n.cross(eb).normalized();
                double t =
                    (s - start).dot(plane_normal) / (ea.dot(plane_normal));
                if (n_face.dot(n) < 0 && distance < 0 && -distance < minDist &&
                    t >= 0 && t <= 1) {
                    ret.ea = ea;
                    ret.eb = eb;
                    ret.n = n;
                    ret.p = start + t * ea;
                    minDist = -distance;
                }
            }
        }
        return ret;
    }

	void computeNarrowPhase(int narrowPhaseMethod);

  void findSimultaneousJ(MatrixXd &A, VectorXd &b, VectorXd &J);
  void fillA(MatrixXd &A);
  void compute_b_vector(VectorXd &b);
  void compute_a_matrix(Eigen::MatrixXd &ABig);
  double compute_aij(Contact ci, Contact cj);

	void applyImpulse(double eps = 1.0);

    void clearDataStructures() {
        m_penetratingEdges.clear();
        m_penetratingVertices.clear();
        m_overlappingBodys.clear();
        m_contacts.clear();
    }

    void computeCollisionDetection(int broadPhaseMethod = 0,
                                   int narrowPhaseMethod = 0,
                                   double eps = 1.0) {
        clearDataStructures();

        computeBroadPhase(broadPhaseMethod);

        computeNarrowPhase(narrowPhaseMethod);

        applyImpulse(eps);
    }

    inline Eigen::Vector3d pt_velocity(RigidObject* body, Eigen::Vector3d p);
    Eigen::Vector3d compute_ndot(Contact* c);

    std::vector<Contact> getContacts() { return m_contacts; }

    std::vector<RigidObject>& m_objects;  // all objects in scene
    // result of broadphase, pairs of objects with possible collisions
    std::vector<std::pair<size_t, size_t>> m_overlappingBodys;

    // set of vertex indices that penetrate a face, used to avoid duplicates
    std::set<int> m_penetratingVertices;
    // set of pairs of vertex indices that represent a penetrating edge, used to
    // avoid duplicates
    std::set<std::pair<int, int>> m_penetratingEdges;

    // computed contact points
    std::vector<Contact> m_contacts;
};

#endif
