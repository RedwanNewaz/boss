#include "planner_interface.h"
#include <boost/program_options.hpp>



using namespace boost::program_options;


int main(int argc, char *argv[])
{
   // read config file thorugh boost program option
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
    auto param = std::make_shared<param_manager2>(vm["config"].as<std::string>());
    std::vector<double> start, goal, boundary; 
    start = param->get_param<std::vector<double>>("start");
    goal = param->get_param<std::vector<double>>("goal");
    
    std::cout << "Start location: ";
    for (const auto& s : start) {
        std::cout << s << " ";
         boundary.push_back(s);
    }
    std::cout << "\nGoal location: ";
    for (const auto& g : goal) {
        std::cout << g << " ";
        boundary.push_back(g);
    }

   //configure collision cheker 
   std::vector<std::vector<float>> obstacles;
   float robot_radius = param->get_param<float>("robot_radius");
   float obstacle_length = param->get_param<float>("obstacle_length");
   param->get_obstacles(obstacles);
   auto cc = std::make_shared<CollisionChecker>(obstacles, robot_radius, obstacle_length);
   
   auto lower = *std::min_element(boundary.begin(), boundary.end()) - 1.0;
   auto upper = *std::max_element(boundary.begin(), boundary.end()) + 1.0;
   PlannerInterface planner(cc, lower, upper, param);
   planner.plan(start, goal);
   
   return 0;
}
