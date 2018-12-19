//
// Created by lijie on 9/28/18.
//

#include "arcsim_application.h"
#include "io.hpp"
#include "simulation.hpp"
#include "conf.hpp"
#include "log.hpp"
#include "simulators/arc_simulator.h"
#include "integrators/bogus_integrator.h"

namespace applications {


    bool isResume = false;
    string timingfile;
    string inprefix, outprefix;
    int frameskip;

    void init_physics(const string &json_file,
                      bool is_reloading, arcsim::Simulation &sim) {
        load_json(json_file, sim);
        if (!outprefix.empty()) {
            timingfile = arcsim::stringf("%s/timing", outprefix.c_str()).c_str();
            fstream tfs;
            tfs.open(timingfile,
                            is_reloading ? ios::out | ios::app : ios::out);
            // Make a copy of the config file for future use
            arcsim::copy_file(json_file.c_str(), arcsim::stringf("%s/conf.json", outprefix.c_str()));
            // And copy over all the obstacles
            vector<arcsim::Mesh *> base_meshes(sim.obstacles.size());
            for (int o = 0; o < sim.obstacles.size(); o++)
                base_meshes[o] = &sim.obstacles[o].base_mesh;
            arcsim::save_objs(base_meshes, arcsim::stringf("%s/obs", outprefix.c_str()));
        }
        string logFile;
        if (outprefix.empty()) {
            logFile = "output_log.txt";
        } else {
            string filename = isResume ? "log_resume.txt" : "log.txt";
            logFile = arcsim::stringf("%s/%s", outprefix.c_str(), filename.c_str()).c_str();
            if (!isResume) {
                fstream out(logFile, ios::out);
                out.close();
            }
        }
        arcsim::Log *log = arcsim::Log::getInstance();
        log->init(logFile);
        prepare(sim);
        if (!is_reloading) {
            // separate_obstacles(sim.obstacle_meshes, sim.cloth_meshes);
            relax_initial_state(sim);
        }
    }

    void display_physics(const std::vector<std::string> &args) {
        if (args.size() != 1 && args.size() != 2) {
            cout << "Runs the simulation with an OpenGL display." << endl;
            cout << "Arguments:" << endl;
            cout << "    <scene-file>: JSON file describing the simulation setup"
                 << endl;
            cout << "    <out-dir> (optional): Directory to save output in" << endl;
            exit(EXIT_FAILURE);
        }
        string json_file = args[0];
        outprefix = args.size() > 1 ? args[1] : "";
        if (!outprefix.empty())
            arcsim::ensure_existing_directory(outprefix);
        arcsim::Simulation sim;
        init_physics(json_file, false, sim);
        sim.frame = 0;
        sim.time = 0;
        sim.step = 0;
        if (!outprefix.empty())
            save_objs(sim.cloth_meshes, arcsim::stringf("%s/%06d", outprefix.c_str(), 0));
        std::shared_ptr<simulators::Simulator> simulator = make_shared<simulators::ARCSimulator>(std::make_unique<BogusIntegrator>(), sim, timingfile, outprefix);
        display::Interface i(simulator);
        i.run();
    }

    void run_physics(const std::vector<std::string> &args) {
        if (args.size() != 1 && args.size() != 2) {
            cout << "Runs the simulation in batch mode." << endl;
            cout << "Arguments:" << endl;
            cout << "    <scene-file>: JSON file describing the simulation setup"
                 << endl;
            cout << "    <out-dir> (optional): Directory to save output in" << endl;
            exit(EXIT_FAILURE);
        }
        string json_file = args[0];
        outprefix = args.size()>1 ? args[1] : "";
        if (!outprefix.empty())
            arcsim::ensure_existing_directory(outprefix);
        arcsim::Simulation sim;
        init_physics(json_file, false, sim);
        sim.frame = 0;
        sim.time = 0;
        sim.step = 0;
        if (!outprefix.empty())
            save_objs(sim.cloth_meshes, arcsim::stringf("%s/%06d", outprefix.c_str(), 0));
        std::shared_ptr<simulators::Simulator> simulator = make_shared<simulators::ARCSimulator>(std::make_unique<BogusIntegrator>(), sim, timingfile, outprefix);
        while (true) {
            simulator->step();
        }
    }

    void init_resume(const vector<string> &args, arcsim::Simulation &sim) {
        isResume = true;
        assert(args.size() == 2);
        outprefix = args[0];
        string start_frame_str = args[1];
        // Load like we would normally begin physics
        init_physics(arcsim::stringf("%s/conf.json", outprefix.c_str()), true, sim);
        // Get the initialization information
        sim.frame = atoi(start_frame_str.c_str());
        sim.time = sim.frame * sim.frame_time;
        sim.step = sim.frame * sim.frame_steps;
        for(int i=0; i<sim.obstacles.size(); ++i)
            sim.obstacles[i].get_mesh(sim.time);
        load_objs(sim.cloth_meshes, arcsim::stringf("%s/%06d",outprefix.c_str(),sim.frame));
        prepare(sim); // re-prepare the new cloth meshes
        // separate_obstacles(sim.obstacle_meshes, sim.cloth_meshes);
        for (int m = 0; m < sim.cloth_meshes.size(); m++) {
            arcsim::Mesh* mesh = sim.cloth_meshes[m];
            mesh->kdTree = new arcsim::KDTree(mesh->verts);
            for (int n = 0; n < mesh->nodes.size(); n++) {
                mesh->nodes[n]->inMesh = true;
            }
            // if (::magic.merge_radius < sim.cloths[m].remeshing.size_min) {
            //   sim.cloths[m].remeshing.size_min = ::magic.merge_radius;
            // } else {
            //   ::magic.merge_radius = sim.cloths[m].remeshing.size_min;
            // }
        }
        for (int m = 0; m < sim.obstacle_meshes.size(); m++) {
            arcsim::Mesh* mesh = sim.obstacle_meshes[m];
            for (int n = 0; n < mesh->nodes.size(); n++) {
                mesh->nodes[n]->inMesh = false;
            }
        }
    }

    void display_resume(const std::vector<std::string> &args) {
        if (args.size() != 2) {
            cout << "Resumes an incomplete simulation." << endl;
            cout << "Arguments:" << endl;
            cout << "    <out-dir>: Directory containing simulation output files"
                 << endl;
            cout << "    <resume-frame>: Frame number to resume from" << endl;
            exit(EXIT_FAILURE);
        }
        arcsim::Simulation sim;
        init_resume(args, sim);
        std::shared_ptr<simulators::Simulator> simulator = make_shared<simulators::ARCSimulator>(std::make_unique<BogusIntegrator>(), sim, timingfile, outprefix);
        display::Interface i(simulator);
        i.run();
    }

    void resume_physics (const vector<string> &args) {
        if (args.size() != 2) {
            cout << "Resumes an incomplete simulation in batch mode." << endl;
            cout << "Arguments:" << endl;
            cout << "    <out-dir>: Directory containing simulation output files"
                 << endl;
            cout << "    <resume-frame>: Frame number to resume from" << endl;
            exit(EXIT_FAILURE);
        }
        arcsim::Simulation sim;
        init_resume(args, sim);
        std::shared_ptr<simulators::Simulator> simulator = make_shared<simulators::ARCSimulator>(std::make_unique<BogusIntegrator>(), sim, timingfile, outprefix);
        while (true) {
            simulator->step();
        }
    }

    void display_replay (const vector<string> &args) {
        if (args.size() < 1 || args.size() > 3) {
            cout << "Replays the results of a simulation." << endl;
            cout << "Arguments:" << endl;
            cout << "    <out-dir>: Directory containing simulation output files"
                 << endl;
            cout << "    <sshot-dir> (optional): Directory to save images" << endl;
            cout << "    <frame-skip> (optional): Save image every n frames" << endl;
            exit(EXIT_FAILURE);
        }
        inprefix = args[0];
        outprefix = args.size()>1 ? args[1] : "";
        frameskip = args.size()>2 ? atoi(args[2].c_str()) : 1;
        if (!outprefix.empty())
            arcsim::ensure_existing_directory(outprefix);
        char config_backup_name[256];
        snprintf(config_backup_name, 256, "%s/%s", inprefix.c_str(), "conf.json");
        arcsim::Simulation sim;
        sim.frame = 0;
        load_json(config_backup_name, sim);
        prepare(sim);
        std::shared_ptr<simulators::ARCSimulator> simulator = make_shared<simulators::ARCSimulator>(std::make_unique<BogusIntegrator>(), sim, timingfile, outprefix, true);
        simulator->setInputPrefix(inprefix);
        simulator->setFrameskip(frameskip);
        display::Interface i(simulator);
        i.run();
    }

    ArcsimApplication::ArcsimApplication(int argc, const char **argv) {
        struct Action {
            string name;

            void (*run)(const vector<string> &args);
        } actions[] = {
                {"simulate", display_physics},
            {"simulateoffline", run_physics},
            {"resume", display_resume},
            {"resumeoffline", resume_physics},
            {"replay", display_replay}
        };
        int nactions = sizeof(actions) / sizeof(Action);
        string name = (argc <= 1) ? "" : argv[1];
        vector<string> args;
        for (int a = 2; a < argc; a++)
            args.push_back(argv[a]);
        for (int i = 0; i < nactions; i++) {
            if (name == actions[i].name) {
                actions[i].run(args);
                return;
            }
        }
        cout << "Usage: " << endl;
        cout << "    " << argv[0] << " <command> [<args>]" << endl;
        cout << "where <command> is one of" << endl;
        for (int i = 0; i < nactions; i++)
            cout << "    " << actions[i].name << endl;
        cout << endl;
        cout << "Run '" << argv[0] << " <command>' without extra arguments ";
        cout << "to get more information about a particular command." << endl;
        exit(EXIT_FAILURE);
    }

    void ArcsimApplication::init() {

    }

}