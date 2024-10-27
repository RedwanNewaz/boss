

#include <iostream>
#include <cassert>
#include <iterator>
#include <chrono>
#include <sstream>
#include "boss_interface.h"

#include <boost/program_options.hpp>
using namespace boost::program_options;

int main(int argc, char *argv[])
{
    options_description desc("Options");
    desc.add_options()
        ("help,h", "Help screen")
        ("config", value<std::string>()->default_value("cbo_param.yaml"), "bow config yaml file")
        ("verbose", value<bool>()->default_value(false), "verbose output");

    variables_map vm;
    store(parse_command_line(argc, argv, desc), vm);
    notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 1;
    }

    bool verbose = vm["verbose"].as<bool>();

   
    std::cout << "\nVerbose: " << (verbose ? "true" : "false") << std::endl;


    
     // Read start and goal locations 
    param_manager2 param(vm["config"].as<std::string>());
    std::vector<double> start, goal; 
    start = param.get_param<std::vector<double>>("start");
    goal = param.get_param<std::vector<double>>("goal");
    std::cout << "Start location: ";
    for (const auto& s : start) {
        std::cout << s << " ";
    }
    std::cout << "\nGoal location: ";
    for (const auto& g : goal) {
        std::cout << g << " ";
    }
    BossInterface boss(vm["config"].as<std::string>());
    boss.setGoal(goal[0], goal[1]);
    State state{start[0], start[1], M_PI_2, 0, 0};
    double dt = boss.getSampleTime(); 
    auto start_time = std::chrono::high_resolution_clock::now();
    std::stringstream ss; 

    do
    {
        auto traj = boss.getTraj(state[0], state[1], state[2]);
        // auto control = boss.getControl();
        if(traj.size() > 1)
            for(int i = 0; i < state.size(); ++i)
                state[i] = traj[1][i];

        if(verbose)
        {
            std::copy(state.begin(), state.end(), std::ostream_iterator<double>(ss, ", "));
            ss << "\n";
            // std::cout << ss.str() << std::endl;
        }

    }while(!boss.terminate(state[0], state[1]));

    // Record end time
    auto end_time = std::chrono::high_resolution_clock::now();

    // Calculate duration
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    // Output duration in microseconds
    std::cout << "[BOW]: Execution time: " << duration.count() << " milliseconds" << std::endl;
    if (verbose)
    {
        std::ofstream outFile("output.csv");
        if (outFile.is_open()) {
            // Write the stringstream contents to the file
            outFile << ss.str();
            // Close the file
            outFile.close();
            std::cout << "Stringstream contents have been written to output.csv" << std::endl;
        } else {
            std::cerr << "Unable to open file for writing" << std::endl;
        }
    }
        
    return 0;
}
