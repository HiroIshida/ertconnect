#include <ompl/base/State.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <experience/ERT.h>

#include <memory>

namespace ob = ompl::base;
namespace og = ompl::geometric;

int main(){
  // create planner for 2d square space
  const auto space(std::make_shared<ob::RealVectorStateSpace>());
  space->addDimension(0.0, 1.0);
  space->addDimension(0.0, 1.0);
  const auto si = std::make_shared<ob::SpaceInformation>(space);

  const auto setup_rrt = std::make_unique<og::SimpleSetup>(si);
  setup_rrt->setStateValidityChecker([](const ob::State* s) { return true; });

  // create many example data
  std::vector<std::vector<ob::State*>> experiences;

  for(size_t i = 0; i < 200; ++i){
    const auto valid_sampler = setup_rrt->getSpaceInformation()->allocValidStateSampler();
    ob::ScopedState<> start(setup_rrt->getStateSpace());
    valid_sampler->sample(start.get());
    ob::ScopedState<> goal(setup_rrt->getStateSpace());
    valid_sampler->sample(goal.get());
    setup_rrt->setStartAndGoalStates(start, goal);
    const auto result = setup_rrt->solve(1.0);
    const bool solved = result && result != ob::PlannerStatus::APPROXIMATE_SOLUTION;
    if(solved){
      const auto p = setup_rrt->getSolutionPath().as<og::PathGeometric>();
      const auto experience = p->getStates();
      experiences.push_back(experience);
    }
  }

  // create ert planning by setting many experiences
  const auto ert_setup = std::make_unique<og::SimpleSetup>(si);
  ert_setup->setStateValidityChecker([](const ob::State* s) { return true; });
  const auto ert_planner = std::make_shared<og::ERT>(si);
  for(const auto& experience : experiences){
    ert_planner->setExperience(experience);
  }
  ert_setup->setPlanner(ert_planner);

  const auto valid_sampler = ert_setup->getSpaceInformation()->allocValidStateSampler();
  ob::ScopedState<> start(ert_setup->getStateSpace());
  valid_sampler->sample(start.get());
  ob::ScopedState<> goal(ert_setup->getStateSpace());
  valid_sampler->sample(goal.get());
  ert_setup->solve(1.0);
}
