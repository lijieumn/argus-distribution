//
// Created by lijie on 9/6/18.
//

#ifndef ARGUS_DISTRIBUTION_APPLICATION_H
#define ARGUS_DISTRIBUTION_APPLICATION_H

#include "simulators/simulator.h"
#include "display/interface.h"

namespace applications {

    class Application {
    protected:
    public:
//    Application(int argc, char const *argv[]) {
////        mSimulator = std::make_shared<Simulator>(std::make_unique<Integrator>(), "meshes/sphere.obj");
//    }
        virtual void init() = 0;

    };

}

#endif //ARGUS_DISTRIBUTION_APPLICATION_H
