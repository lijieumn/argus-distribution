//
// Created by lijie on 9/6/18.
//

#ifndef ARGUS_DISTRIBUTION_SIMULATOR_H
#define ARGUS_DISTRIBUTION_SIMULATOR_H

#include <memory>
#include "integrators/integrator.h"
#include "display/timestep.h"

namespace simulators {

    class Simulator : public display::Timestep {
    protected:
        std::unique_ptr<Integrator> mIntegrator;

    public:
        Simulator(std::unique_ptr<Integrator> integrator);

        virtual void init() = 0;

        virtual void resetDrawable() = 0;

        virtual void step() = 0;

        virtual void saveScreenshot(int width, int height) = 0;

    };

}

#endif //ARGUS_DISTRIBUTION_SIMULATOR_H
