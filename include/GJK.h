#ifndef GJK_H
#define GJK_H

#include <tuple>
#include <functional>
#include <Eigen/Core>
#include "RigidObject.h"
#include "igl/barycentric_coordinates.h"

typedef  std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>> Simplex;

class GJK{
public:
	RigidObject A;
	RigidObject B;
	Simplex simplex;
	Eigen::Vector3d direction;

	GJK(RigidObject A_, RigidObject B_){
		A = A_;
		B = B_;
	}

	void setDirection(const Eigen::Vector3d& d){
		direction = d;
	}


	Eigen::Vector3d getFarthestPointInDirection(Eigen::MatrixXd& vertices, Eigen::Vector3d direction){
        float max = -FLT_MAX;
        Eigen::Vector3d farthestVertex;
        for(int i = 0; i < vertices.rows(); i++){
        	Eigen::Vector3d v = vertices.row(i);
            float dot = v.dot(direction);
            if(dot > max){
                max = dot;
                farthestVertex = v;
            }
                        
        }
        return farthestVertex;
	}


	 std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> support(Eigen::Vector3d&  d){
		    Eigen::MatrixXd VA, VB;
            Eigen::MatrixXi FA, FB;
            A.getMesh(VA, FA);
            B.getMesh(VB, FB);
		   Eigen::Vector3d p1 = getFarthestPointInDirection(VA, d);
           Eigen::Vector3d p2 = getFarthestPointInDirection(VB, -d);
		   return {p1 - p2, p1, p2};
	}

	bool GJKDetect(Eigen::Vector3d& point, Eigen::Vector3d& normal){
		// first Minkowski Difference point
		simplex.push_back(support(direction));
		// invert direction
		direction = -direction;

		int counter = 0;
		while(true){
			// add a new point to the simplex because we haven't terminated yet
			simplex.push_back(support(direction));
			 // check that last point added was past the origin
			if(std::get<0>(simplex.back()).dot(direction) <= 0){
				return false;
			}
			else{
				// determine if origin is in current simplex
				if(containsOrigin()) {
					return epa(point, normal);
					//return true;
				}
			}
			counter++;
			if(counter > 200) return false;

		}
	}

	bool containsOrigin() {
		Eigen::Vector3d a = std::get<0>(simplex.back());
		Eigen::Vector3d ao = -1 * a;
        //cout << "the size of simplex: " << simplex.size() << endl;
	
		// the different shapes we can have around the origin
		if(simplex.size() == 4) {
			// tetrahedron
			Eigen::Vector3d ab = std::get<0>(simplex[2]) - a;
			Eigen::Vector3d ac = std::get<0>(simplex[1]) - a;
			Eigen::Vector3d ad = std::get<0>(simplex[0]) - a;

			Eigen::Vector3d abcperp = ab.cross(ac).normalized();
			Eigen::Vector3d abdperp = ab.cross(ad).normalized();
			Eigen::Vector3d acdperp = ac.cross(ad).normalized();

			if(abcperp.dot(ao) > 0) {	
				direction = abcperp.normalized();
				simplex.erase(simplex.begin());
			} 
			else if(abdperp.dot(ao) > 0){
				direction = abdperp.normalized();
				simplex.erase(simplex.begin()+1);
			}
            else if(acdperp.dot(ao) > 0){				
				direction = acdperp.normalized();
				simplex.erase(simplex.begin()+2);
			} 
			else {
				// in the tetrahedron
				return true;
			}
		}
		else if(simplex.size() == 3){
			// triangle
			Eigen::Vector3d ab = std::get<0>(simplex[1]) - a;
			Eigen::Vector3d ac = std::get<0>(simplex[0]) - a;

			Eigen::Vector3d abperp = ac.cross(ab).cross(ab);
			Eigen::Vector3d acperp = ab.cross(ac).cross(ac);
			Eigen::Vector3d abcperp = ab.cross(ac);

			if(abperp.dot(ao) > 0){
				// simplex.erase(simplex.begin());
				direction = abperp.normalized();
			} 
			else if(acperp.dot(ao) > 0){		
				direction = acperp.normalized();
			} 
            else if(abcperp.dot(ao) > 0){
				direction = abcperp.normalized();
			} 
            else{
				direction = -abcperp.normalized();
			}
		}
		else{
			// line segment
			Eigen::Vector3d ab = std::get<0>(simplex[0]) - a;
			Eigen::Vector3d abperp = ab.cross(ao).cross(ab);
			direction = abperp.normalized();
		}
		return false;
	}

	//EPA 
	bool epa(Eigen::Vector3d& point, Eigen::Vector3d& normal){
        //std::cout << "this is the size of the simplex in the very very beginning: " << simplex.size() << endl;

		Eigen::MatrixXi faces(4,3);

		faces.row(0) << 0, 1, 2;
		faces.row(1) << 0, 2, 3;
		faces.row(2) << 1, 3, 2;
		faces.row(3) << 0, 3, 1;
		while(true) {
			double distValue = std::numeric_limits<double>::max();
			
			int face = findClosestFace(faces, distValue);
			Eigen::Vector3d firstEdge = std::get<0>(simplex[faces(face, 1)]) - std::get<0>(simplex[faces(face, 0)]);
			Eigen::Vector3d secEdge = std::get<0>(simplex[faces(face, 2)]) - std::get<0>(simplex[faces(face, 0)]);
			Eigen::Vector3d n = firstEdge.cross(secEdge).normalized();
			std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>  p = support(n);
			double distance = std::get<0>(p).dot(n);
			if(distance - distValue < std::numeric_limits<double>::epsilon()){
				normal = -n;
				point = 1.0/3 * (std::get<1>(p) + std::get<1>(simplex[faces(face,1)]) + std::get<1>(simplex[faces(face,2)]));
				return true;
			} 
			else{
				//std::cout << distance - distValue << std::endl;
				simplex.push_back(p);
				Eigen::Vector3d last(3);
				last[0] = faces(face, 0);
				last[1] = faces(face, 1);
				last[2] = faces(face, 2);

				int newPoint = simplex.size()-1;

				int rowSize = faces.rows();
				int colSize = faces.cols();

				faces.row(face) << newPoint, last[0], last[1];

				Eigen::MatrixXi tmp(faces);
				faces.resize(rowSize+2, colSize);

				for(int i = 0; i < tmp.rows(); i++) {
					faces.row(i) = tmp.row(i);
				}
				faces.row(rowSize) << newPoint, last[1], last[2];
				faces.row(rowSize+1) << newPoint, last[2], last[0];
			}
		}
	}

	int findClosestFace(Eigen::MatrixXi& faces, double& distValue) {
		int face = -1;
		Eigen::Vector3d point;
		Eigen::Vector3d firstEdge;
		Eigen::Vector3d secEdge;
		Eigen::Vector3d n;
		for(int i = 0; i < faces.rows(); i++) {
			point = std::get<0>(simplex[faces(i, 0)]);
			firstEdge = std::get<0>(simplex[faces(i, 1)]) - std::get<0>(simplex[faces(i, 0)]);
			secEdge = std::get<0>(simplex[faces(i, 2)]) - std::get<0>(simplex[faces(i, 0)]);
			n = secEdge.cross(firstEdge);
			n.normalize();
			double dist = n.dot(-point);

			if(dist < distValue) {
				distValue = dist;
				face = i;
			}
		}
		return face;
	}


};

#endif // GJK_H
