#include <iostream>
#include <string.h>
#include <vector>
#include "Simulation.h"

using namespace std;

/*
 * Simulation the initial configuration.
 */

class InitialConfiguration{
    std::string configInit;
    std::string pinType;
    int pinNumbers;
    double offset;

    public:
      InitialConfiguration(string config, int& _pinNumbers, double offset);
      InitialConfiguration();
  
      void fillPositions(vector<RigidObject> &objects);
      void scale(vector<RigidObject> &objects, string pinType);
  
};
