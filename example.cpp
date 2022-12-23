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
    const double r = 0.43;
    // NOTE: to compare performance, the sleep is injected intentionaly
    std::this_thread::sleep_for(std::chrono::microseconds(100));
    return (x * x + y * y) > (r * r);
  };

  const auto setup = std::make_shared<og::SimpleSetup>(si);
  setup->setStateValidityChecker(isValid);
  return setup;
}

ot::LightningDBPtr create_database(og::SimpleSetupPtr setup, const int n_data, double& time_per_problem){
  const auto si = setup->getSpaceInformation();
  const auto algo = std::make_shared<og::RRTConnect>(si);
  setup->setPlanner(algo);

  // create many example data

  auto database = std::make_shared<ot::LightningDB>(si->getStateSpace());

  double time_sum_rrt = 0.0;
  for(size_t i = 0; i < n_data; ++i){
    setup->clear();
    const auto valid_sampler = setup->getSpaceInformation()->allocValidStateSampler();
    ob::ScopedState<> start(setup->getStateSpace());
    valid_sampler->sample(start.get());
    ob::ScopedState<> goal(setup->getStateSpace());
    valid_sampler->sample(goal.get());
    setup->setStartAndGoalStates(start, goal);
    const auto result = setup->solve(1.0);
    const bool solved = result && result != ob::PlannerStatus::APPROXIMATE_SOLUTION;
    if(solved){
      auto p = setup->getSolutionPath().as<og::PathGeometric>();
      double insertion_time;
      database->addPath(*p, insertion_time);
    }
    time_sum_rrt += setup->getLastPlanComputationTime();
  }
  time_per_problem = time_sum_rrt / (double)n_data;
  return database;
}


int main(){
  const auto setup = create_setup();
  const auto si = setup->getSpaceInformation();

  double rrt_time;
  const auto database = create_database(setup, 1000, rrt_time);
  const auto ert_planner = std::make_shared<og::ERTConnect>(si);

  const auto valid_sampler = setup->getSpaceInformation()->allocValidStateSampler();

  double time_sum_ert = 0.0;
  const double n_ert_trial = 300;
  for(size_t i=0; i < n_ert_trial; ++i){
    // define problem
    ert_planner->clear();
    ob::ScopedState<> start(setup->getStateSpace());
    valid_sampler->sample(start.get());
    ob::ScopedState<> goal(setup->getStateSpace());
    valid_sampler->sample(goal.get());

    // determine relevant path and set experience
    const auto relevant_path = database->findNearestStartGoal(1, start->as<ob::State>(), goal->as<ob::State>()).at(0);
    auto relevant_path_geometric(std::make_shared<og::PathGeometric>(si));
    for (std::size_t i = 0; i < relevant_path->numVertices(); ++i)
    {
        relevant_path_geometric->append(relevant_path->getVertex(i).getState());
    }
    const auto experience = relevant_path_geometric->getStates();

    // solve
    ert_planner->setExperience(experience);
    setup->setPlanner(ert_planner);
    setup->setStartAndGoalStates(start, goal);
    setup->solve(1.0);
    time_sum_ert += setup->getLastPlanComputationTime();
  }
  const double ert_time = time_sum_ert / (double)n_ert_trial;
  std::cout << "rrt-connect average solving time: " << rrt_time << std::endl;
  std::cout << "ert-connect average solvign time: " << ert_time << std::endl;
}
