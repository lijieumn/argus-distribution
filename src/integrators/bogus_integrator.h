//
// Created by jie on 9/7/18.
//

#ifndef ARGUS_DISTRIBUTION_BOGUS_INTEGRATOR_H
#define ARGUS_DISTRIBUTION_BOGUS_INTEGRATOR_H
#include "integrator.h"
#include "sparse.hpp"
#include "collision.hpp"
#include "magic.hpp"

class BogusIntegrator : public Integrator {
public:

    vector<pair<arcsim::Vec3, arcsim::Vec3> >  solve(const arcsim::SpMat<arcsim::Mat3x3> &A, const vector<arcsim::Vec3> &b, vector<arcsim::Vec3> &linear_v,
                                     const vector<arcsim::ArgusImpact> &argusImpacts,  arcsim::Mesh &mesh, const double dt);

    vector<pair<arcsim::Vec3, arcsim::Vec3> > solve(const arcsim::SpMat<arcsim::Mat3x3> &A, const vector<arcsim::Vec3> &b, vector<arcsim::Vec3> &linear_v,
                                    const vector<arcsim::Impact> &impacts,  arcsim::Mesh &mesh, const double dt);

    vector<arcsim::Vec3> linear_solve(const arcsim::SpMat<arcsim::Mat3x3> &A, const vector<arcsim::Vec3> &b);
};

#endif //ARGUS_DISTRIBUTION_BOGUS_INTEGRATOR_H
