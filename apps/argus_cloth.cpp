//
// Created by lijie on 9/6/18.
//

#include <iostream>
#include "applications/arcsim_application.h"
#include "simulators/arc_simulator.h"
#include "integrators/bogus_integrator.h"

int main(int argc, char const* argv[]) {

    applications::ArcsimApplication(argc, argv);
//    System<ARCSimulator, BogusIntegrator>(argc, argv).start();
    return 0;
}