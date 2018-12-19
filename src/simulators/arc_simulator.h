//
// Created by jie on 9/7/18.
//

#ifndef ARGUS_DISTRIBUTION_ARC_SIMULATOR_H
#define ARGUS_DISTRIBUTION_ARC_SIMULATOR_H

#include "simulator.h"
#include "simulation.hpp"
#include "collision.hpp"

const static float clothColor[3] = {0.5, 0.5, 1};
const static float obsColor[3] = {0.5, 0.5, 0.5};

namespace simulators {

    class ARCSimulator : public Simulator {
    private:
        arcsim::Simulation &mSim;

        void update_obstacles(bool update_positions);

        vector<arcsim::Constraint *> get_constraints(bool include_proximity);

        void delete_constraints(const vector<arcsim::Constraint *> &cons);

        void step_mesh(arcsim::Mesh &mesh, double dt);

        void produce_argus_impacts(const std::vector<std::vector<arcsim::Impact> > &impacts,
                                   std::vector<std::vector<arcsim::ArgusImpact> > &argusImpacts);

        void setPreMergeNormals(std::vector<std::vector<arcsim::Impact> > &impacts);

        void setPostMergeNormals(std::vector<std::vector<arcsim::Impact> > &impacts);

        void physics_step(const vector<arcsim::Constraint *> &cons);

        void saveTimings();

        void save();

        std::string mTimingFile;
        string mOutputPrefix;

        bool mReplay = false;
        int mFrameskip;
        string mInputPrefix;

//        std::vector<arcsim::Vec3*> displayLines;
    public:
        ARCSimulator(std::unique_ptr<Integrator> integrator, arcsim::Simulation &sim, string timingFile = "", string outputPrefix = "", bool replay = false);

        void init();

        void resetDrawable();

        void step();

        void advanceStep();

        void advanceFrame();

        void saveScreenshot(int width, int height);

        void setTimingFile(string& timingFile);
        void setFrameskip(int frameskip);
        void setInputPrefix(string inputPrefix);
    };
}
#endif //ARGUS_DISTRIBUTION_ARC_SIMULATOR_H
