//
// Created by jie on 9/7/18.
//

#include "io.hpp"
#include "arc_simulator.h"
#include "proximity.hpp"
#include "magic.hpp"
#include "physics.hpp"
#include "log.hpp"
#include "mergehelper.hpp"
#include "mergeimpacts.hpp"
#include "dynamicremesh.hpp"
#include "sparse_solver.hpp"
#include "separate.hpp"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

namespace simulators {

    ARCSimulator::ARCSimulator(std::unique_ptr<Integrator> integrator, arcsim::Simulation &sim, string timingFile, string outputPrefix, bool replay ) : Simulator(
            std::move(integrator)), mSim(sim), mTimingFile(timingFile), mOutputPrefix(outputPrefix), mReplay(replay) {
        init();
    }

    void ARCSimulator::init() {
        resetDrawable();
    }

    void ARCSimulator::resetDrawable() {
        // This implementation is bad -- with different cloth pieces and obstacles using the same texture file. To be fixed.
        clothDrawable.texFile = mSim.cloths[0].texture;
        obstacleDrawable.texFile = mSim.obstacles[0].texture;

        clothDrawable.vSize = 0;
        clothDrawable.fSize = 0;
        for (int c = 0; c < mSim.cloths.size(); c++) {
            clothDrawable.vSize += mSim.cloths[c].mesh.verts.size();
            clothDrawable.fSize += mSim.cloths[c].mesh.faces.size();
        }
        obstacleDrawable.vSize = 0;
        obstacleDrawable.fSize = 0;
        for (int o = 0; o < mSim.obstacles.size(); o++) {
            obstacleDrawable.vSize += mSim.obstacles[o].get_mesh().verts.size();
            obstacleDrawable.fSize += mSim.obstacles[o].get_mesh().faces.size();
        }

        clothDrawable.vertices = std::make_unique<float[]>(clothDrawable.vSize * 3);
        clothDrawable.texCoords = std::make_unique<float[]>(clothDrawable.vSize*2);
        clothDrawable.colors = std::make_unique<float[]>(clothDrawable.vSize*3);
        clothDrawable.normals = std::make_unique<float[]>(clothDrawable.vSize * 3);
        clothDrawable.elements = std::make_unique<int[]>(clothDrawable.fSize*3);

        obstacleDrawable.vertices = std::make_unique<float[]>(obstacleDrawable.vSize * 3);
        obstacleDrawable.texCoords = std::make_unique<float[]>(obstacleDrawable.vSize*2);
        obstacleDrawable.colors = std::make_unique<float[]>(obstacleDrawable.vSize*3);
        obstacleDrawable.normals = std::make_unique<float[]>(obstacleDrawable.vSize * 3);
        obstacleDrawable.elements = std::make_unique<int[]>(obstacleDrawable.fSize*3);

        int vtxOffset = 0, faceOffset = 0;
        for (int c = 0; c < mSim.cloths.size(); c++) {
            arcsim::Mesh &mesh = mSim.cloths[c].mesh;
            compute_ws_data(mesh);
            for (int v = 0; v < mesh.verts.size(); v++) {
                for (int i = 0; i < 3; i++) {
                    clothDrawable.vertices[vtxOffset * 3 + v * 3 + i] = mesh.verts[v]->node->x[i];
                    clothDrawable.colors[vtxOffset * 3 + v * 3 + i] = clothColor[i];
                    clothDrawable.normals[vtxOffset * 3 + v * 3 + i] = mesh.verts[v]->node->n[i];
                }
                for (int i = 0; i < 2; i++) {
                    clothDrawable.texCoords[vtxOffset*2 + v*2 + i] = mesh.verts[v]->u[i];
                }
            }
            for (int f = 0; f < mesh.faces.size(); f++) {
                for (int i = 0; i < 3; i++) {
                    clothDrawable.elements[faceOffset * 3 + f * 3 + i] = mesh.faces[f]->v[i]->index + vtxOffset;
                }
            }
            vtxOffset += mesh.verts.size();
            faceOffset += mesh.faces.size();
        }
        vtxOffset = 0;
        faceOffset = 0;
        for (int o = 0; o < mSim.obstacles.size(); o++) {
            arcsim::Mesh &mesh = mSim.obstacles[o].get_mesh();
            compute_ws_data(mesh);
            for (int v = 0; v < mesh.verts.size(); v++) {
                for (int i = 0; i < 3; i++) {
                    obstacleDrawable.vertices[vtxOffset * 3 + v * 3 + i] = mesh.verts[v]->node->x[i];
                    obstacleDrawable.colors[vtxOffset * 3 + v * 3 + i] = obsColor[i];
                    obstacleDrawable.normals[vtxOffset * 3 + v * 3 + i] = mesh.verts[v]->node->n[i];
                }
                for (int i = 0; i < 2; i++) {
                    obstacleDrawable.texCoords[vtxOffset*2 + v*2 + i] = mesh.verts[v]->u[i];
                }
            }
            for (int f = 0; f < mesh.faces.size(); f++) {
                for (int i = 0; i < 3; i++) {
                    obstacleDrawable.elements[faceOffset * 3 + f * 3 + i] = mesh.faces[f]->v[i]->index + vtxOffset;
                }
            }
            vtxOffset += mesh.verts.size();
            faceOffset += mesh.faces.size();
        }
//        for (int l = 0; l < displayLines.size(); l++) {
//            for (int v = 0; v < 2; v++) {
//                for (int i = 0; i < 3; i++) {
//                    clothDrawable.vertices[vtxOffset*3 + l*2*3 + v*3 + i] = displayLines[l][v][i];
//                }
//                clothDrawable.elements[faceOffset*3 + l*2 + v] = vtxOffset + l*2 + v;
//            }
//        }
    }

    static arcsim::Timer totalTimer;

    void ARCSimulator::saveTimings () {
        static double old_totals[arcsim::Simulation::nModules] = {};
        if (mTimingFile.empty())
            return;
        ofstream out(mTimingFile, ios::app);
        for (int i = 0; i < arcsim::Simulation::nModules; i++) {
            out << mSim.timers[i].total - old_totals[i] << " ";
            old_totals[i] = mSim.timers[i].total;
        }
        out << endl;
    }

    void ARCSimulator::save () {
//        save(sim.cloth_meshes, frame);
        if (!mOutputPrefix.empty()/* && frame < 10000*/) {
            save_objs(mSim.cloth_meshes, arcsim::stringf("%s/%06d", mOutputPrefix.c_str(), mSim.frame));
        }
    }

    void ARCSimulator::step() {
        if (mReplay) {
            int fullframe = mSim.frame*mFrameskip;
            mSim.time = fullframe * mSim.frame_time;
            load_objs(mSim.cloth_meshes, arcsim::stringf("%s/%06d",mInputPrefix.c_str(), fullframe));
            if (mSim.cloth_meshes[0]->verts.empty()) {
                if (mSim.frame == 0) {
                    std::cout << "FIRST\n";
                    exit(EXIT_FAILURE);
                }
                if (!mOutputPrefix.empty()){
                    std::cout << "SECOND\n";
                    exit(EXIT_SUCCESS);
                }
                mSim.frame = 0;
                mSim.cloth_meshes[0] = mSim.liveMesh;
                step();
            }
            for (int o = 0; o < mSim.obstacles.size(); o++)
                mSim.obstacles[o].get_mesh(mSim.time);
            mSim.frame++;

            resetDrawable();
        } else {
            arcsim::Log* log = arcsim::Log::getInstance();
            advanceStep();
            log->setFrame(mSim.step);
            log->output();
            log->reset();
            if (mSim.step % mSim.frame_steps == 0) {
                save();
                saveTimings();
            }
            if (mSim.time >= mSim.end_time || mSim.frame >= mSim.end_frame) {
                totalTimer.tock();
                log->saveTotalTime(totalTimer.total);
                exit(EXIT_SUCCESS);
            }
        }
    }

    void ARCSimulator::advanceStep() {
        arcsim::VisualDebugger *vd = arcsim::VisualDebugger::getInstance();
        vd->clearData();
        activate_nodes(mSim.cloth_meshes[0]->nodes);

        mSim.cloth_meshes[0] = mSim.liveMesh;
        mSim.time += mSim.step_time;
        mSim.step++;

        // Updating the obstacles for the time step
        update_obstacles(false);

        // Getting constraints
        vector<arcsim::Constraint *> cons = get_constraints(true);
        // Perform a physics step
        physics_step(cons);

        if (mSim.step % mSim.frame_steps == 0) {
            //remeshing_step(sim);
            //sim.bufferedMeshes[3] = deep_copy(sim.cloths[0].mesh);
            mSim.frame++;
        }

        // Deleting constraints
        delete_constraints(cons);

        // Load buffer
        if (mSim.bufferId >= 0 && mSim.bufferId <= arcsim::Simulation::nStages) {
            mSim.cloth_meshes[0] = mSim.bufferedMeshes[mSim.bufferId];
        }
        resetDrawable();
    }

    void ARCSimulator::advanceFrame() {

    }

    void ARCSimulator::saveScreenshot(int width, int height) {
        if (!mReplay || mOutputPrefix.empty()) {
            return;
        }
        unsigned char *pixels = new unsigned char[width*height*3];
        glPixelStorei(GL_PACK_ALIGNMENT, 1);

        glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels);
        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height/2; j++) {
                for (int c = 0; c < 3; c++) {
                    swap(pixels[i*3 + j*width*3 + c], pixels[i*3 + (height - j - 1)*width*3 + c]);
                }
            }
        }

        char filename[256];
        snprintf(filename, 256, "%s/%06d.png", mOutputPrefix.c_str(), mSim.frame);
        stbi_write_png(filename, width, height, 3, pixels, 0);
        delete[] pixels;
    }

    void ARCSimulator::update_obstacles(bool update_positions) {
        double decay_time = 0.1,
                blend = mSim.step_time / decay_time;
        blend = blend / (1 + blend);
        for (int o = 0; o < mSim.obstacles.size(); o++) {
            mSim.obstacles[o].get_mesh(mSim.time);
            // sim.obstacles[o].blend_with_previous(sim.time, sim.step_time, blend);
            if (!update_positions) {
                // put positions back where they were
                arcsim::Mesh &mesh = mSim.obstacles[o].get_mesh();
                for (int n = 0; n < mesh.nodes.size(); n++) {
                    arcsim::Node *node = mesh.nodes[n];
                    node->v = (node->x - node->x0) / mSim.step_time;
                    node->x = node->x0;
                }
            }
        }
    }

    vector<arcsim::Constraint *> ARCSimulator::get_constraints(bool include_proximity) {
        vector<arcsim::Constraint *> cons;
        for (int h = 0; h < mSim.handles.size(); h++)
            append(cons, mSim.handles[h]->get_constraints(mSim.time));
        for (int m = 0; m < mSim.cloth_meshes.size(); m++) {
            mark_nodes_to_preserve(*mSim.cloth_meshes[m]);
        }
        if (include_proximity && mSim.enabled[arcsim::Simulation::Proximity]) {
            mSim.timers[arcsim::Simulation::Proximity].tick();
            append(cons, arcsim::proximity_constraints(mSim.cloth_meshes,
                                               mSim.obstacle_meshes,
                                               mSim.friction, mSim.obs_friction));
            mSim.timers[arcsim::Simulation::Proximity].tock();
        }
        return cons;
    }

    void ARCSimulator::delete_constraints(const vector<arcsim::Constraint *> &cons) {
        for (int c = 0; c < cons.size(); c++)
            delete cons[c];
    }


    void ARCSimulator::step_mesh(arcsim::Mesh &mesh, double dt) {
        for (int n = 0; n < mesh.nodes.size(); n++)
            mesh.nodes[n]->x += mesh.nodes[n]->v * dt;
    }

    void ARCSimulator::produce_argus_impacts(const std::vector<std::vector<arcsim::Impact> > &impacts,
                                             std::vector<std::vector<arcsim::ArgusImpact> > &argusImpacts) {

        // copy old meshes
        vector<arcsim::Mesh> old_meshes(mSim.cloths.size());
        vector<arcsim::Mesh *> old_meshes_p(mSim.cloths.size()); // for symmetry in separate()
        for (int c = 0; c < mSim.cloths.size(); c++) {
            old_meshes[c] = deep_copy(mSim.cloths[c].mesh);
            old_meshes_p[c] = &old_meshes[c];
        }

        // init the meshes
        init_meshes(mSim.cloth_meshes, old_meshes_p, mSim.obstacle_meshes);

        // remesh
        for (int c = 0; c < mSim.cloths.size(); c++) {
            vector<arcsim::Plane> planes = nearest_obstacle_planes(mSim.cloths[c].mesh,
                                                           mSim.obstacle_meshes);

            std::vector<arcsim::ArgusImpact> aImps = collision_refine(mSim.cloths[c], mSim.obstacle_meshes, planes, impacts[c],
                                                              true);
            argusImpacts.push_back(aImps);

        }

        // delete old meshes
        for (int c = 0; c < mSim.cloths.size(); c++)
            delete_mesh(old_meshes[c]);

    }


    void ARCSimulator::setPreMergeNormals(std::vector<std::vector<arcsim::Impact> > &impacts) {

        arcsim::VisualDebugger *vd = arcsim::VisualDebugger::getInstance();
        for (int i = 0; i < impacts[0].size(); i++) {
            arcsim::Impact impact = impacts[0][i];
            arcsim::Vec3 p;
            arcsim::Vec3 p2;
            if (impact.type == arcsim::Impact::EE) {
                p = impact.nodes[2]->x * (-impact.w[2]) + impact.nodes[3]->x * (-impact.w[3]);
                p2 = impact.nodes[0]->x * impact.w[0] + impact.nodes[1]->x * impact.w[1];
            } else if (impact.type == arcsim::Impact::VF) {
                p = impact.nodes[1]->x * (-impact.w[1]) + impact.nodes[2]->x * (-impact.w[2]) +
                    impact.nodes[3]->x * (-impact.w[3]);
                p2 = impact.nodes[0]->x;
            } else if (impact.type == arcsim::Impact::VE) {
                p = impact.nodes[1]->x * (impact.w[1]) + impact.nodes[2]->x * (impact.w[2]);
                p2 = impact.nodes[0]->x;
            } else {
                p = impact.nodes[1]->x;
                p2 = impact.nodes[0]->x;
            }
            vd->addVisualPoint3(p, arcsim::Vec3(1, 0, 0), 'b');
            vd->addVisualLine3(p, p + impact.n * .05, arcsim::Vec3(1, 0, 0), 'b');
            vd->addVisualPoint3(p2, arcsim::Vec3(1, 0, 0), 'b');

        }

    }


    void ARCSimulator::setPostMergeNormals(std::vector<std::vector<arcsim::Impact> > &impacts) {

        arcsim::VisualDebugger *vd = arcsim::VisualDebugger::getInstance();
        for (int i = 0; i < impacts[0].size(); i++) {
            arcsim::Impact impact = impacts[0][i];
            arcsim::Vec3 p;
            arcsim::Vec3 p2;
            if (impact.type == arcsim::Impact::EE) {
                p = impact.nodes[2]->x * (-impact.w[2]) + impact.nodes[3]->x * (-impact.w[3]);
                p2 = impact.nodes[0]->x * impact.w[0] + impact.nodes[1]->x * impact.w[1];
            } else if (impact.type == arcsim::Impact::VF) {
                p = impact.nodes[1]->x * (-impact.w[1]) + impact.nodes[2]->x * (-impact.w[2]) +
                    impact.nodes[3]->x * (-impact.w[3]);
                p2 = impact.nodes[0]->x;
            } else if (impact.type == arcsim::Impact::VE) {
                p = impact.nodes[1]->x * (impact.w[1]) + impact.nodes[2]->x * (impact.w[2]);
            } else {
                p = impact.nodes[1]->x;
            }
            vd->addVisualPoint3(p, arcsim::Vec3(0, 0, 1), 'a');
            vd->addVisualLine3(p, p + impact.n * .05, arcsim::Vec3(0, 0, 1), 'a');

            char switchChar;
            arcsim::Vec3 color;


            vd->addVisualPoint3(p2, color, switchChar);
            vd->addVisualPoint3(p, color, switchChar);
            //vd->addVisualLine3(p2,p,color,switchChar);
            vd->addVisualLine3(p2, p2 + 0.05 * impact.n, color, switchChar);
            //vd->addVisualLine3(p,p-0.05*impact.n, color, switchChar);



        }
    }

    struct ImpactCompare {
        bool operator()(const arcsim::Impact &lhs, const arcsim::Impact &rhs) const {
            if (lhs.type != rhs.type) {
                return lhs.type < rhs.type;
            }
            for (int n = 0; n < 4; n++) {
                if (lhs.nodes[n] != rhs.nodes[n]) {
                    return lhs.nodes[n] < rhs.nodes[n];
                }
            }
            return false;
        }
    };

    vector<arcsim::Impact> faster_pruning(vector<arcsim::Impact> &impacts) {
        map<arcsim::Impact, int, ImpactCompare> iMap;
        for (int i = 0; i < impacts.size(); i++) {
            arcsim::Impact &impact = impacts[i];
            auto search = iMap.find(impact);
            if (search != iMap.end()) {    // found
                // const Impact& impact2 = search->first;
                // if (impact.d )
            } else {
                iMap[impact] = i;
            }
        }
        vector<arcsim::Impact> pruned;
        for (auto &search : iMap) {
            pruned.push_back(impacts[search.second]);
        }
        return pruned;
    }


    void compute_internal_force_density(arcsim::Cloth &cloth) {
        vector<arcsim::Vec3> forceVec(cloth.mesh.nodes.size(), arcsim::Vec3(0, 0, 0));
        arcsim::add_internal_forces<arcsim::WS>(cloth, forceVec);
        for (int n = 0; n < cloth.mesh.nodes.size(); n++) {
            arcsim::Node *node = cloth.mesh.nodes[n];
            node->internal_force_density = forceVec[n] / node->a;
        }
    }

    void remeshing_step(arcsim::Simulation &sim) {
        if (!sim.enabled[arcsim::Simulation::Remeshing])
            return;
        // copy old meshes
        vector<arcsim::Mesh> old_meshes(sim.cloths.size());
        vector<arcsim::Mesh *> old_meshes_p(sim.cloths.size()); // for symmetry in separate()
        for (int c = 0; c < sim.cloths.size(); c++) {
            old_meshes[c] = deep_copy(sim.cloths[c].mesh);
            old_meshes_p[c] = &old_meshes[c];
        }
        // back up residuals
        // typedef vector<Residual> MeshResidual;
        // vector<MeshResidual> res;
        // if (sim.enabled[plasticity] && !initializing) {
        //     sim.timers[plasticity].tick();
        //     res.resize(sim.cloths.size());
        //     for (int c = 0; c < sim.cloths.size(); c++)
        //         res[c] = back_up_residuals(sim.cloths[c].mesh);
        //     sim.timers[plasticity].tock();
        // }
        // remesh
        sim.timers[arcsim::Simulation::Remeshing].tick();
        // init_meshes(sim.cloth_meshes, old_meshes_p, sim.obstacle_meshes);
        vector<arcsim::AccelStruct *> obs_accs = create_accel_structs(sim.obstacle_meshes, false);
        for (int c = 0; c < sim.cloths.size(); c++) {
            compute_internal_force_density(sim.cloths[c]);
            deactivate_nodes(sim.cloths[c].mesh.nodes);
            if (arcsim::magic.fixed_high_res_mesh) {
                static_remesh(sim.cloths[c], obs_accs, true);
            } else {

                // vector<Plane> obs_planes = nearest_obstacle_planes(sim.cloths[c].mesh,
                //                                                sim.obstacle_meshes);
                arcsim::NearestSearch ns;
                // vector<Plane> new_planes = ns.getNearestPlanes(sim.cloth_meshes, sim.obstacle_meshes);

                if (arcsim::magic.use_proximity_metric) {
                    arcsim::CuttingPlaneSet cutting_planes = ns.getCuttingPlanes(sim.cloth_meshes, sim.obstacle_meshes);
                    setPlaneSet(cutting_planes);
                } else {
                    vector<arcsim::Plane> obs_planes = nearest_obstacle_planes(sim.cloths[c].mesh, sim.obstacle_meshes);
                    arcsim::setPlanes(obs_planes);
                }
                dynamic_remesh(sim.cloths[c], /*cutting_planes, */sim.enabled[arcsim::Simulation::Plasticity], obs_accs, true);
            }
            activate_nodes(sim.cloths[c].mesh.nodes);
        }
        ICM_separate(sim.cloth_meshes, old_meshes_p, sim.obstacle_meshes, false);
        destroy_accel_structs(obs_accs);
        sim.timers[arcsim::Simulation::Remeshing].tock();
        for (int c = 0; c < sim.cloths.size(); c++)
            delete_mesh(old_meshes[c]);
    }

    void projectArgusImpacts(std::vector<std::vector<arcsim::ArgusImpact> > &argusImpacts) {

        arcsim::VisualDebugger *vd = arcsim::VisualDebugger::getInstance();

        // Project impacts before passing into the solver
        for (int i = 0; i < argusImpacts[0].size(); i++) {
            arcsim::ArgusImpact &argusImpact = argusImpacts[0][i];
            arcsim::Vec3 posA = argusImpact.nodeA->x;
            arcsim::Vec3 posB = argusImpact.nodeB ? argusImpact.nodeB->x : argusImpact.posB;
            arcsim::Vec3 normal = argusImpact.normal;
            // double d = dot(posA - posB, normal);
            // double thickness = ::magic.projection_thickness;
            // if (d < thickness) {
            // 	double difference = thickness - d;
            // 	argusImpact.nodeA->x += difference*normal;
            // }

            vd->addVisualPoint3(posA, arcsim::Vec3(0, 1, 1), 'n');
            vd->addVisualPoint3(posB, arcsim::Vec3(0, 1, 1), 'n');
            vd->addVisualLine3(posA, posA + normal * .05, arcsim::Vec3(0, 1, 1), 'n');

        }
    }

    void ARCSimulator::physics_step(const vector<arcsim::Constraint *> &cons) {

        // If physics is disabled, do nothing
        if (!mSim.enabled[arcsim::Simulation::Physics])
            return;
        mSim.timers[arcsim::Simulation::Physics].tick();

        arcsim::Timer t;
        double tPhysics = 0, tRemeshing = 0, tMerging = 0, tCollision = 0, tSolver = 0,
                tIntersectionRemoval = 0, tExtra = 0;

        // Increment free age of every node
        for (int c = 0; c < mSim.cloths.size(); c++) {
            for (int n = 0; n < mSim.cloths[c].mesh.nodes.size(); n++) {
                arcsim::Node *node = mSim.cloths[c].mesh.nodes[n];
                node->freeAge += 1;
            }
            // 	sim.cloths[c].mesh.reset_face_size_min(sim.cloths[c].remeshing.size_min);
        }

        // // Pre-refinement mesh buffer
        delete_mesh(*mSim.bufferedMeshes[arcsim::Simulation::PRE_REFINE]);
        *mSim.bufferedMeshes[arcsim::Simulation::PRE_REFINE] = deep_copy(mSim.cloths[0].mesh);
        t.tick();
        remeshing_step(mSim);
        t.tock();
        tRemeshing = t.last;
        delete_mesh(*mSim.bufferedMeshes[arcsim::Simulation::POST_REFINE]);
        *mSim.bufferedMeshes[arcsim::Simulation::POST_REFINE] = deep_copy(mSim.cloths[0].mesh);
        mSim.cloths[0].mesh.clearImpactPoints();

        arcsim::SpMat<arcsim::Mat3x3> A;
        vector<arcsim::Vec3> b;
        vector<arcsim::Vec3> predictedVels;
        t.tick();
        for (int c = 0; c < mSim.cloths.size(); c++) {
            vector<arcsim::Vec3> fext;
            vector<arcsim::Mat3x3> Jext;
            arcsim::add_ext_and_morphs(mSim, cons, fext, Jext, c);
            pair<arcsim::SpMat<arcsim::Mat3x3>, vector<arcsim::Vec3> > equation;
            equation = arcsim::obtain_implicit_equation(mSim.cloths[c], fext, Jext, cons, mSim.step_time);
            A = equation.first;
            b = equation.second;
            // predictedVels = linear_solve(A, b);
            predictedVels = arcsim::eigen_linear_solve(A, b);
            for (int n = 0; n < mSim.cloths[c].mesh.nodes.size(); n++) {
                arcsim::Node *node = mSim.cloths[c].mesh.nodes[n];
                node->v = predictedVels[n];
            }
        }
        t.tock();
        tSolver = t.last;

        // 1. Finding impacts between the cloth meshes and obstacle meshes
        vector<vector<arcsim::Impact> > impacts;
        t.tick();
        vector<arcsim::Impact> imps = prepare_impacts(mSim.cloth_meshes, mSim.obstacle_meshes);
        // pruneImpacts(imps);
        t.tock();
        tCollision = t.last;
        impacts.push_back(imps);

        arcsim::VisualDebugger *vd = arcsim::VisualDebugger::getInstance();
        arcsim::Log *log = arcsim::Log::getInstance();

        vector<vector<arcsim::ArgusImpact> > argusImpacts;
        int maxIter = 1;
        for (int iter = 0; iter < maxIter; iter++) {
            // 2. Pruning duplicate impacts ///////
            t.tick();
            impacts[0] = faster_pruning(impacts[0]);
            // prune_impacts(sim, impacts, impacts_equal);
            t.tock();
            tExtra = t.last;

            arcsim::MergeHelper merge(impacts[0], mSim.cloth_meshes[0]->kdTree, mSim.step_time);
            // merge.insertNodes(sim.cloths[0]);

            // 3. Merging impacts
#ifndef SILENCE_ARGUS
            if (impacts[0].size() > 0) {
                std::cout << "Before merge, #impacts: " << impacts[0].size() << std::endl;
            }
#endif
            setPreMergeNormals(impacts);

            if (!arcsim::magic.face_edge_constraints) {
                bool new_merge_strategy = true;
                if (new_merge_strategy) {
                    // merge.impacts = impacts[0];
                    t.tick();
                    log->setContactNumberBefore(impacts[0].size());
                    merge.mergeImpacts(mSim.cloths[0]);
                    t.tock();
                    tMerging = t.last;
                    // collapse_edges(sim);
                }
                if (arcsim::magic.merge_proximal_impacts && !new_merge_strategy) {
                    merge_proximal_impacts(impacts[0], mSim.cloth_meshes[0]);
                }

#ifndef SILENCE_ARGUS
                if (impacts[0].size() > 0) {
                    std::cout << "After merge, #impacts: " << impacts[0].size() << std::endl;
                }
#endif
                setPostMergeNormals(impacts);
                //-------------------------------------------//


                ////////// 4. Produce ArgusImpacts /////////////////////////
                if (new_merge_strategy) {
                    // insert_nodes(merge.clusters, sim.cloths[0], sim.obstacle_meshes);
                    vector<arcsim::ArgusImpact> aImps = convert_to_argus(merge.nodalImpacts);
                    argusImpacts.push_back(aImps);
                    merge.recycleMemory();
                } else {
                    produce_argus_impacts(impacts, argusImpacts);
                }

#ifndef SILENCE_ARGUS
                if (argusImpacts[0].size() > 0) {
                    std::cout << "After refine, #impacts: " << argusImpacts[0].size() << std::endl;
                }
#endif
                //-------------------------------------------//

#ifndef SILENCE_ARGUS
                std::cout << "finished producing argus impacts \n";
#endif
                ////////// 5. Projecting argus impacts before solving //////
                projectArgusImpacts(argusImpacts);
                //-------------------------------------------//

                // unsigned int numVerts = sim.cloth_meshes[0]->verts.size();
                // unsigned int numLeaves = sim.cloth_meshes[0]->kdTree->leavesNumber();
                // assert(numVerts == numLeaves);
                // sim.cloth_meshes[0]->kdTree

#ifndef SILENCE_ARGUS
                std::cout << "projected argus impacts \n";
#endif

                //prune_argus_impacts(sim,argusImpacts);

                reset_contact_forces(mSim.cloths[0].mesh);
            }
#ifndef SILENCE_ARGUS
            if (argusImpacts.size() > 0) {
                std::cout << "argus impacts size is " << argusImpacts[0].size() << std::endl;
            }
#endif

            // if (::magic.face_edge_constraints) {
            // projectImpacts(impacts[0]);
            // }

            // Pre physics step mesh buffer
            delete_mesh(*mSim.bufferedMeshes[arcsim::Simulation::POST_MERGE]);
            *mSim.bufferedMeshes[arcsim::Simulation::POST_MERGE] = deep_copy(mSim.cloths[0].mesh);

            // 6. SOLVE AND UPDATE
            for (int c = 0; c < mSim.cloths.size(); c++) {
                // Computing external forces and constructing the implicit equation consisting of A and b

                // Solving the implicit contact-friction problem to get the new velocity
                vector<pair<arcsim::Vec3, arcsim::Vec3> > v_and_r;

                if (arcsim::magic.face_edge_constraints) {
                    v_and_r = mIntegrator->solve(A, b, predictedVels, impacts[c], mSim.cloths[c].mesh,
                                                     mSim.step_time);
                } else {
                    v_and_r = mIntegrator->solve(A, b, predictedVels, argusImpacts[c], mSim.cloths[c].mesh,
                                                     mSim.step_time);
                }
                assert(v_and_r.size() == mSim.cloths[c].mesh.nodes.size());
                update(mSim.cloths[c], v_and_r, mSim.step_time, false);
            }

            for (int i = 0; i < mSim.cloths[0].mesh.nodes.size(); i++) {
                arcsim::Node *n = mSim.cloths[0].mesh.nodes[i];
                vd->addVisualPoint3(n->x, arcsim::Vec3(1, 0, 1), 'f');
                vd->addVisualLine3(n->x, n->x + n->r * 10e3, arcsim::Vec3(1, 0, 1), 'f');

                vd->addVisualPoint3(n->x, arcsim::Vec3(1, 0, 0), 'u');
                vd->addVisualLine3(n->x, n->x + n->v * mSim.step_time * 1e2, arcsim::Vec3(0, 0, 1), 'u');
            }

            for (int i = 0; i < mSim.obstacle_meshes[0]->nodes.size(); i++) {
                arcsim::Node *n = mSim.obstacle_meshes[0]->nodes[i];
                vd->addVisualPoint3(n->x, arcsim::Vec3(0, 0, 1), 'u');
                vd->addVisualLine3(n->x, n->x + n->v * 10e-2, arcsim::Vec3(0, 0, 1), 'u');
            }

            update_x0(*mSim.cloth_meshes[0]);
            update_n0(*mSim.cloth_meshes[0]);

            // Stepping cloth and obstacle meshes
            for (int c = 0; c < mSim.cloth_meshes.size(); c++)
                step_mesh(*mSim.cloth_meshes[c], mSim.step_time);
            for (int o = 0; o < mSim.obstacle_meshes.size(); o++)
                step_mesh(*mSim.obstacle_meshes[o], mSim.step_time);

            vector<arcsim::AccelStruct *> accs = create_accel_structs(mSim.cloth_meshes, true);
            vector<arcsim::AccelStruct *> obs_accs = create_accel_structs(mSim.obstacle_meshes, true);
            // Find impacts from collisions between cloths and obstacles
            vector<arcsim::Impact> unresolved = find_continuous_impacts(accs, obs_accs);
            destroy_accel_structs(accs);
            destroy_accel_structs(obs_accs);
            if (unresolved.size() == 0 || iter == maxIter - 1) {
                break;
            }
            for (int i = 0; i < unresolved.size(); i++) {
                impacts[0].push_back(unresolved[i]);
            }

            backto_x0(*mSim.cloth_meshes[0]);
            backto_n0(*mSim.cloth_meshes[0]);
        }

        delete_mesh(*mSim.bufferedMeshes[arcsim::Simulation::POST_PHYSICS]);
        *mSim.bufferedMeshes[arcsim::Simulation::POST_PHYSICS] = deep_copy(mSim.cloths[0].mesh);

        update_x0(*mSim.obstacle_meshes[0]);
        update_n0(*mSim.obstacle_meshes[0]);


        t.tick();
        vector<arcsim::Mesh> old_meshes(mSim.cloths.size());
        vector<arcsim::Mesh *> old_meshes_p(mSim.cloths.size());
        for (int c = 0; c < mSim.cloths.size(); c++) {
            old_meshes[c] = deep_copy(mSim.cloths[c].mesh);
            old_meshes_p[c] = &old_meshes[c];
        }
        ICM_separate(mSim.cloth_meshes, old_meshes_p, mSim.obstacle_meshes);
        t.tock();
        tIntersectionRemoval = t.last;
        for (int c = 0; c < mSim.cloths.size(); c++) {
            delete_mesh(old_meshes[c]);
        }

        // Caching the impact data before coarsening the mesh (for visual debugging purposes)
        // Note that right now this only caches data from the first cloth
        // cacheArgusImpacts(sim, argusImpacts);

        // Pre-coarsen mesh buffer
        delete_mesh(*mSim.bufferedMeshes[arcsim::Simulation::POST_ICM]);
        *mSim.bufferedMeshes[arcsim::Simulation::POST_ICM] = deep_copy(mSim.cloths[0].mesh);

        mSim.timers[arcsim::Simulation::Physics].tock();
        // Post-coarsened mesh buffer
        tPhysics = mSim.timers[arcsim::Simulation::Physics].last;


        log->setNodeNumber(mSim.cloths[0].mesh.nodes.size());
        if (arcsim::magic.face_edge_constraints) {
            log->setContactNumber(impacts[0].size());
        } else {
            log->setContactNumber(argusImpacts[0].size());
        }
        log->setPhysicsStepTime(tPhysics);
        log->setRemeshingTime(tRemeshing);
        log->setCollisionTime(tCollision);
        log->setExtraTime(tExtra);
        log->setMergingTime(tMerging);
        log->setSolverTime(tSolver);
        log->setICMTime(tIntersectionRemoval);
    }

    void ARCSimulator::setTimingFile(string &timingFile) {
        mTimingFile = timingFile;
    }

    void ARCSimulator::setFrameskip(int frameskip) {
        mFrameskip = frameskip;
    }

    void ARCSimulator::setInputPrefix(string inputPrefix) {
        mInputPrefix = inputPrefix;
    }
}