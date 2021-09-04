#include <iostream>
#include <string.h>
#include <vector>
#include "InitialConfiguration.h"
#include "Simulation.h"

using namespace std;

/*
 * Simulation the initial configuration.
 */
    InitialConfiguration::InitialConfiguration(){
  }
    InitialConfiguration::InitialConfiguration( string config, int& _pinNumbers, double _offset){
        configInit = config;
        if(configInit == "2rows"){
            _pinNumbers = 6;
        }
        else if(configInit == "3rows"){
          _pinNumbers = 9;
        }
        else if(configInit == "4rows"){
          _pinNumbers = 12;
        }
       
        offset = _offset;
        

  }
    void InitialConfiguration::scale(vector<RigidObject> &objects, string _pinType){
       /****   pin type   ****/
        pinType = _pinType;
        if(pinType == "ovoid"){
          //objects[0].setScale(1);
          objects[0].setScale(0.08);
          objects[0].setType(ObjType::DYNAMIC);

          for (int i = 1; i <= objects.size()-1; i++) {
            objects[i].setScale(0.7);
            objects[i].setType(ObjType::DYNAMIC);
          }
        }
        else if(pinType == "sphere"){
          objects[0].setScale(1);
          objects[0].setType(ObjType::DYNAMIC);

          for (int i = 1; i <= objects.size()-1; i++) {
            objects[i].setScale(0.1);
            objects[i].setType(ObjType::DYNAMIC);
          }

        }
        else if(pinType == "cube"){
          objects[0].setScale(1);
          objects[0].setType(ObjType::DYNAMIC);

          for (int i = 1; i <= objects.size()-1; i++) {
            objects[i].setScale(0.6);
            objects[i].setType(ObjType::DYNAMIC);
          }
        }
        else if(pinType == "cuboid"){
          objects[0].setScale(1);
          objects[0].setType(ObjType::DYNAMIC);

          for (int i = 1; i <= objects.size()-1; i++) {
            objects[i].setScale(0.1);
            objects[i].setType(ObjType::DYNAMIC);
          }
        }
        
    }
    void InitialConfiguration::fillPositions(vector<RigidObject> &objects){
        if(pinType == "ovoid"){
          // ball
          objects[0].setPosition(Eigen::Vector3d(-10, 1.5 + offset, 0));
        }
        else if(pinType == "sphere"){
          // ball
          objects[0].setPosition(Eigen::Vector3d(-10, 1.5 + offset, 0));
        }
        else if(pinType == "cube"){
          // ball
          objects[0].setPosition(Eigen::Vector3d(-10, 1.5 + offset, 0));
        }
        else if(pinType == "cuboid"){
          // ball
          objects[0].setPosition(Eigen::Vector3d(-10, 1.5 + offset, 0));

        }

        // 4 in a row
        offset += 0.6;
        if(configInit == "2rows"){
          objects[1].setPosition(Eigen::Vector3d(0, 0+offset, -2));
          objects[2].setPosition(Eigen::Vector3d(0, 0+offset, 0));
          objects[3].setPosition(Eigen::Vector3d(0, 0+offset, 2));

          objects[4].setPosition(Eigen::Vector3d(2, 0+offset, -2));
          objects[5].setPosition(Eigen::Vector3d(2, 0+offset, 0));
          objects[6].setPosition(Eigen::Vector3d(2, 0+offset, 2));
          
        }
        else if(configInit == "3rows"){
          objects[1].setPosition(Eigen::Vector3d(0, 0+offset, -2));
          objects[2].setPosition(Eigen::Vector3d(0, 0+offset, 0));
          objects[3].setPosition(Eigen::Vector3d(0, 0+offset, 2));

          objects[4].setPosition(Eigen::Vector3d(2, 0+offset, -2));
          objects[5].setPosition(Eigen::Vector3d(2, 0+offset, 0));
          objects[6].setPosition(Eigen::Vector3d(2, 0+offset, 2));

          objects[7].setPosition(Eigen::Vector3d(-2, 0+offset, -2));
          objects[8].setPosition(Eigen::Vector3d(-2, 0+offset, 0));
          objects[9].setPosition(Eigen::Vector3d(-2, 0+offset, 2));
        }
        else if(configInit == "4rows"){
          objects[1].setPosition(Eigen::Vector3d(0, 0+offset, -2));
          objects[2].setPosition(Eigen::Vector3d(0, 0+offset, 0));
          objects[3].setPosition(Eigen::Vector3d(0, 0+offset, 2));

          objects[4].setPosition(Eigen::Vector3d(2, 0+offset, -2));
          objects[5].setPosition(Eigen::Vector3d(2, 0+offset, 0));
          objects[6].setPosition(Eigen::Vector3d(2, 0+offset, 2));

          objects[7].setPosition(Eigen::Vector3d(-2, 0+offset, -2));
          objects[8].setPosition(Eigen::Vector3d(-2, 0+offset, 0));
          objects[9].setPosition(Eigen::Vector3d(-2, 0+offset, 2));

          objects[10].setPosition(Eigen::Vector3d(4, 0+offset, -2));
          objects[11].setPosition(Eigen::Vector3d(4, 0+offset, 0));
          objects[12].setPosition(Eigen::Vector3d(4, 0+offset, 2));
        }
    }
  

