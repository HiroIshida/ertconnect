#include "ompl/base/ScopedState.h"
#include <ompl/base/State.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/tools/lightning/LightningDB.h>
#include <experience/ERT.h>
#include <experience/ERTConnect.h>

#include <chrono>
#include <memory>
#include <thread>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace ot = ompl::tools;
using namespace std::chrono;


og::SimpleSetupPtr create_setup(){
  const auto space(std::make_shared<ob::RealVectorStateSpace>());
  space->addDimension(0.0, 1.0);
  space->addDimension(0.0, 1.0);
  const auto si = std::make_shared<ob::SpaceInformation>(space);

  const auto isValid = [](const ob::State* s){
    const auto& rs = s->as<ob::RealVectorStateSpace::StateType>();
    const auto x = (rs->values[0] - 0.5);
    const auto y = (rs->values[1] - 0.5);
    const double r = 0.4;
    // NOTE: to compare performance, the sleep is injected intentionaly
    std::this_thread::sleep_for(std::chrono::microseconds(100));
    return (x * x + y * y) > (r * r);
  };

  const auto setup = std::make_shared<og::SimpleSetup>(si);
  setup->setStateValidityChecker(isValid);
  return setup;
}


int main(){
  const auto setup = create_setup();
  const auto si = setup->getSpaceInformation();
  si->setStateValidityCheckingResolution(0.005);

  ob::ScopedState<> start(setup->getStateSpace());
  ob::ScopedState<> goal(setup->getStateSpace());

  auto rs_start = start->as<ob::RealVectorStateSpace::StateType>();
  auto rs_goal = goal->as<ob::RealVectorStateSpace::StateType>();
  rs_start->values[0] = 0.1;
  rs_start->values[1] = 0.1;
  rs_goal->values[0] = 0.9;
  rs_goal->values[1] = 0.9;

  const auto algo = std::make_shared<og::RRTConnect>(si);
  setup->setPlanner(algo);
  setup->setStartAndGoalStates(start, goal);
  const auto result = setup->solve(1.0);
  const bool solved = result && result != ob::PlannerStatus::APPROXIMATE_SOLUTION;
  std::cout << "rrt-connect solving time: " << setup->getLastPlanComputationTime() << std::endl;

  const auto p = setup->getSolutionPath().as<og::PathGeometric>();
  const auto experience = p->getStates();

  const auto ert_planner = std::make_shared<og::ERTConnect>(si);
  ert_planner->clear();

  // slightly different configuration from the one in the base-trajectory
  rs_start->values[0] = 0.15;
  rs_start->values[1] = 0.15;
  rs_goal->values[0] = 0.95;
  rs_goal->values[1] = 0.95;

  ert_planner->setExperience(experience);
  setup->setPlanner(ert_planner);
  setup->solve(1.0);
  std::cout << "ert-connect solving time: " << setup->getLastPlanComputationTime() << std::endl;
}
