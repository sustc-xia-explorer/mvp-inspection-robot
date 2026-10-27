// Line protocol for deterministic closed-loop regression without ROS.
// STEP/APPLIED use public planner actions: diff [v,w], omni WORLD [vx,vy].
#include "neupan/neupan_planner.hpp"
#include "neupan/mlp.hpp"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "usage: neupan_sim_backend config.yaml model.bin\n";
    return 2;
  }
  try {
    auto planner = neupan::NeuPANPlanner::fromYaml(argv[1], argv[2]);
    const auto mlp = neupan::MLP::load(argv[2]);
    std::cout << std::setprecision(17);
    std::string command;
    while (std::cin >> command) {
      if (command == "MLP") {
        int n;
        if (!(std::cin >> n) || n < 1 || n > 100000)
          throw std::runtime_error("bad MLP point count");
        neupan::MatF points(2, n);
        for (int i = 0; i < n; ++i)
          if (!(std::cin >> points(0, i) >> points(1, i)))
            throw std::runtime_error("bad MLP point");
        const auto output = mlp.forward(points);
        std::cout << "MLP " << output.rows() << ' ' << output.cols();
        for (int col = 0; col < output.cols(); ++col)
          for (int row = 0; row < output.rows(); ++row)
            std::cout << ' ' << output(row, col);
        std::cout << '\n' << std::flush;
      } else if (command == "PATH" || command == "REPATH") {
        neupan::Vec3 replacement_state = neupan::Vec3::Zero();
        if (command == "REPATH" && !(std::cin >> replacement_state(0) >> replacement_state(1) >> replacement_state(2)))
          throw std::runtime_error("bad replacement state");
        int n;
        if (!(std::cin >> n) || n < 2 || n > 100000)
          throw std::runtime_error("invalid path count");
        std::vector<neupan::InitialPath::PathPoint> path;
        for (int i = 0; i < n; ++i) {
          double x, y, yaw;
          if (!(std::cin >> x >> y >> yaw)) throw std::runtime_error("bad path");
          path.emplace_back(x, y, yaw, 1.0);
        }
        if (command == "REPATH") planner.replaceInitialPath(std::move(path),replacement_state);
        else {
          planner.setInitialPath(std::move(path));
          planner.reset();
        }
        std::cout << "READY\n" << std::flush;
      } else if (command == "APPLIED") {
        neupan::Vec2 u;
        if (!(std::cin >> u(0) >> u(1))) throw std::runtime_error("bad applied command");
        planner.setAppliedCommand(u);
        std::cout << "ACK\n" << std::flush;
      } else if (command == "STEP") {
        neupan::Vec3 state;
        int n;
        if (!(std::cin >> state(0) >> state(1) >> state(2) >> n) || n < 0 || n > 100000)
          throw std::runtime_error("bad state/count");
        neupan::Mat2X points(2, n);
        for (int i = 0; i < n; ++i)
          if (!(std::cin >> points(0, i) >> points(1, i)))
            throw std::runtime_error("bad obstacle point");
        neupan::NeuPANPlanner::Info info;
        const auto begin = std::chrono::steady_clock::now();
        auto action = planner.forward(state, points, info);
        const double ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - begin).count();
        // Match the ROS1 wrapper's default solver_fail_grace=0.
        if (info.stop || info.arrive || (!info.solved && !info.braking)) action.setZero();
        std::cout << action(0) << ' ' << action(1) << ' ' << info.solved << ' '
                  << info.stop << ' ' << info.arrive << ' ' << info.min_distance
                  << ' ' << ms << ' ' << info.raw_action(0) << ' ' << info.raw_action(1)
                  << ' ' << info.braking << '\n' << std::flush;
      } else {
        throw std::runtime_error("unknown command");
      }
    }
  } catch (const std::exception& ex) {
    std::cerr << "sim backend: " << ex.what() << '\n';
    return 1;
  }
  return 0;
}
