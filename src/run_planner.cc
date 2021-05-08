#include <algorithm>
#include <chrono>
#include <ctime>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include <lemon/dim2.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/config.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/SimpleSetup.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include <yaml-cpp/yaml.h>

#include "mfplan/dijkstra.h"
#include "mfplan/mfplanner.h"

void setup_problem(std::string desc_file) {
  YAML::Node problem_desc = YAML::LoadFile(desc_file);
  std::string map_file = problem_desc["map_name"].as<std::string>();
  YAML::Node map_desc = YAML::LoadFile("maps/" + map_file);
  std::string map_folder = "maps/" + map_file.substr(0, map_file.find("/")+1);

  mfplan::CoordsAndFloor start = std::make_tuple(
      lemon::dim2::Point(
        problem_desc["start"]["coords"][0].as<int>(),
        problem_desc["start"]["coords"][1].as<int>()),
      problem_desc["start"]["floor"].as<int>());
  mfplan::CoordsAndFloor goal = std::make_tuple(
      lemon::dim2::Point(
        problem_desc["goal"]["coords"][0].as<int>(),
        problem_desc["goal"]["coords"][1].as<int>()),
      problem_desc["goal"]["floor"].as<int>());

  bool use_rrt_connect = (problem_desc["use_rrt_connect"] &&
      problem_desc["use_rrt_connect"].as<bool>());

  float t_0 = 0.5;
  float k_0 = 2;
  if (problem_desc["t_0"]) t_0 = problem_desc["t_0"].as<float>();
  if (problem_desc["k_0"]) k_0 = problem_desc["k_0"].as<float>();

  std::unordered_map<int, std::string> map_files;
  for (YAML::const_iterator it = map_desc["map_files"].begin();
      it != map_desc["map_files"].end(); ++it) {
    map_files.emplace(it->first.as<int>(),
        map_folder + it->second.as<std::string>());
  }

  std::chrono::system_clock::time_point start_time;
  std::chrono::system_clock::time_point end_time;
  std::chrono::duration<double> elapsed_seconds;
  if (map_desc["lgf_file"]) {
    std::string lgf_file = map_folder + map_desc["lgf_file"].as<std::string>();
    auto mfplanner = mfplan::MFPlanner(lgf_file, map_files);
    std::cout << "loaded map" << std::endl;

    start_time = std::chrono::system_clock::now();
    mfplan::EdgeList path = mfplanner.get_solution_path(
        start, goal, use_rrt_connect, t_0, k_0);
    end_time = std::chrono::system_clock::now();
    elapsed_seconds = end_time - start_time;
    mfplanner.print_edges(path);
  } else {
    start_time = std::chrono::system_clock::now();
    auto floor = mfplan::Floor(cv::imread(map_files[0]), 0);
    auto status_or_path = floor.find_path(
        std::get<0>(start), std::get<0>(goal), t_0, use_rrt_connect);
    end_time = std::chrono::system_clock::now();
    elapsed_seconds = end_time - start_time;
    auto best_path = std::get<1>(status_or_path);
    if (best_path) {
      floor.viz_path(best_path);
    }
  }
  std::cout << "solution found after " << elapsed_seconds.count()
            << " seconds" << std::endl;
}

int main(int argc, char** argv) {
  if (argc > 1) {
    std::string problem_desc_file = argv[1];
    setup_problem(problem_desc_file);
  }

  return 0;
}
