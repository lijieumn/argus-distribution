//
// Created by lijie on 9/6/18.
//

#include "simulator.h"

namespace simulators {

    Simulator::Simulator(std::unique_ptr<Integrator> integrator) {
        mIntegrator = std::move(integrator);
    }
}