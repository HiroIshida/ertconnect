#include <ompl/base/State.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/tools/lightning/LightningDB.h>
#include <experience/ERT.h>

#include <chrono>
#include <memory>
#include <thread>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace ot = ompl::tools;
using namespace std::chrono;


int main(){
  // create planner for 2d square space
  const auto space(std::make_shared<ob::RealVectorStateSpace>());
  space->addDimension(0.0, 1.0);
  space->addDimension(0.0, 1.0);
  const auto si = std::make_shared<ob::SpaceInformation>(space);

  const auto isValid = [](const ob::State* s){
    const auto& rs = s->as<ob::RealVectorStateSpace::StateType>();
    const auto x = (rs->values[0] - 0.5);
    const auto y = (rs->values[1] - 0.5);
    const double r = 0.43;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    return (x * x + y * y) > (r * r);
  };

  const auto setup_rrt = std::make_unique<og::SimpleSetup>(si);
  setup_rrt->setStateValidityChecker(isValid);
  const auto algo = std::make_shared<og::RRTConnect>(si);
  setup_rrt->setPlanner(algo);

  // create many example data
  std::vector<std::vector<ob::State*>> experiences;

  auto database = ot::LightningDB(space);

  double time_sum_rrt = 0.0;
  for(size_t i = 0; i < 100; ++i){
    setup_rrt->clear();
    const auto valid_sampler = setup_rrt->getSpaceInformation()->allocValidStateSampler();
    ob::ScopedState<> start(setup_rrt->getStateSpace());
    valid_sampler->sample(start.get());
    ob::ScopedState<> goal(setup_rrt->getStateSpace());
    valid_sampler->sample(goal.get());
    setup_rrt->setStartAndGoalStates(start, goal);
    const auto result = setup_rrt->solve(1.0);
    const bool solved = result && result != ob::PlannerStatus::APPROXIMATE_SOLUTION;
    if(solved){
      auto p = setup_rrt->getSolutionPath().as<og::PathGeometric>();
      double insertion_time;
      database.addPath(*p, insertion_time);
      const auto experience = p->getStates();
      experiences.push_back(experience);
    }
    time_sum_rrt += setup_rrt->getLastPlanComputationTime();
  }
  const double time_avrage_rrt = time_sum_rrt / 100.0;

  // create ert planning by setting many experiences
  const auto ert_setup = std::make_unique<og::SimpleSetup>(si);
  ert_setup->setStateValidityChecker(isValid);
  const auto ert_planner = std::make_shared<og::ERT>(si);

  const auto valid_sampler = ert_setup->getSpaceInformation()->allocValidStateSampler();

  double time_sum_ert = 0.0;
  for(size_t i=0; i<30; ++i){
    ert_planner->clear();
    ob::ScopedState<> start(ert_setup->getStateSpace());
    valid_sampler->sample(start.get());
    ob::ScopedState<> goal(ert_setup->getStateSpace());
    valid_sampler->sample(goal.get());
    auto start_ = start->as<ob::State>();
    auto goal_ = goal->as<ob::State>();
    const auto selected_path = database.findNearestStartGoal(1, start_, goal_).at(0);

    const auto chosenPath = database.findNearestStartGoal(1, start_, goal_).at(0);

    assert(chosenPath->numVertices() >= 2);

    auto chosen_path_geometric(std::make_shared<og::PathGeometric>(si));
    for (std::size_t i = 0; i < chosenPath->numVertices(); ++i)
    {
        chosen_path_geometric->append(chosenPath->getVertex(i).getState());
    }
    const auto experience = chosen_path_geometric->getStates();
    ert_planner->setExperience(experience);
    ert_setup->setPlanner(ert_planner);
    ert_setup->setStartAndGoalStates(start, goal);
    ert_setup->solve(1.0);
    time_sum_ert += ert_setup->getLastPlanComputationTime();
  }
  const double time_avrage_ert = time_sum_ert / 100.0;
  std::cout << "rrt: " << time_avrage_rrt << std::endl;
  std::cout << "ert: " << time_avrage_ert << std::endl;
}
