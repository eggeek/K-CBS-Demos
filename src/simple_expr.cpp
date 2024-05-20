
#include <cmath>
#include <ompl/multirobot/base/ProblemDefinition.h>
#include <ompl/multirobot/control/SpaceInformation.h>
#include <ompl/multirobot/control/planners/kcbs/KCBS.h>
#include <ompl/multirobot/control/planners/pp/PP.h>

#include "ompl/control/planners/rrt/RRT.h"
#include <ompl/base/goals/GoalRegion.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

#include "Obstacle.h"
#include <fstream>
#include <iostream>
#include <utility>

namespace omrb = ompl::multirobot::base;
namespace omrc = ompl::multirobot::control;
namespace ob = ompl::base;
namespace oc = ompl::control;

struct Env {
  // worskapce x/y, robot radius
  double maxx, maxy, r;
};

struct Obs {
  double xleft, yleft, xL, yL;
  void print(std::ostream &out) {
    out << xleft << " " << yleft << " " << xL << " " << yL << std::endl;
  }
};

using vmap = std::map<std::string, std::pair<double, double>>;
using oset = std::set<Obstacle *>;

/*
When performing Multi-Robot Motion Planning, it is sometimes the case that
robots are treated as "dynamic obstacles" by other robots (e.g. Prioritized
Planning and Kinodynamic Conflict-Based Search). Thus, we have extended OMPL to
account for dynamic obstacles. To do so, one must implement an additional method
called
``areStatesValid" from the StateValidityChecker class. The method should return
true if state1 and state2 are not in collision. state2 is a pair consisting of
the actual state and the SpaceInformation from which the state lives. The
SpaceInformation object is included so that heterogeneous robots can be properly
accounted for (see the example below). Keep in mind that time-dependence is
handled generically within OMPL. Please see
ob::StateValidityChecker::isValid(const State *state, const double time) for
details.
*/
class myDemoStateValidityChecker : public ob::StateValidityChecker {
public:
  myDemoStateValidityChecker(const ob::SpaceInformationPtr &si, const double r,
                             const oset &obstacles)
      : ob::StateValidityChecker(si), radius(r), obsts(obstacles) {}

  // Answers the question: is the robot described by `si_` at `state` valid?
  bool isValid(const ompl::base::State *state) const override {
    if (!si_->satisfiesBounds(state))
      return false;
    // cast the abstract state type to the type we expect
    const auto *se2state = state->as<ob::SE2StateSpace::StateType>();

    // extract the first component of the state and cast it to what we expect
    const auto *pos =
        se2state->as<ob::RealVectorStateSpace::StateType>(0)->values;
    for (auto it : obsts) {
      const auto *pos2 = it->getCenterPoint();
      const double rad2 = it->getBoundingRadius();
      double dist = sqrt(pow(pos[0] - pos2[0], 2) + pow(pos[1] - pos2[1], 2));
      if (dist <= (rad2 + radius))
        return false;
    }
    return true;
  }

  // Answers the question: does the robot described by `si_` at `state1` avoid
  // collision with some other robot described by a different `si` located at
  // `state2`?
  bool areStatesValid(const ompl::base::State *state1,
                      const std::pair<const ompl::base::SpaceInformationPtr,
                                      const ompl::base::State *>
                          state2) const override {
    /* We assume robots are all disks of varying size (see
     * myDemoStateValidityChecker::radii_) */

    // one can get the states of robots via these commands
    const auto *robot1_state = state1->as<ob::SE2StateSpace::StateType>();
    const auto *robot2_state =
        state2.second->as<ob::SE2StateSpace::StateType>();

    // one must code required logic to figure out if robot1 at state1 collides
    // with robot2 at state2 this example assumes all robots are disks in R^2 of
    // varying sizes.
    double rad1 = this->radius;
    double rad2 = this->radius;

    const double *robot1_pos =
        robot1_state->as<ob::RealVectorStateSpace::StateType>(0)->values;
    const double *robot2_pos =
        robot2_state->as<ob::RealVectorStateSpace::StateType>(0)->values;

    double dist = sqrt(pow(robot1_pos[0] - robot2_pos[0], 2) +
                       pow(robot1_pos[1] - robot2_pos[1], 2));

    return dist > (rad1 + rad2);
  }

private:
  double radius;
  oset obsts;
  // std::unordered_map<std::string, double> radii_{
  //     {"Robot 0", 0.1}, {"Robot 1", 0.2}, {"Robot 2", 0.3}};
};

/*
Initially, K-CBS is a fully decoupled algorithm. However, in
certain scenarios, coupling sub-sets of robots are neccesssary to find plans.
Thus, K-CBS couples certain robots, as needed, while planning. The user must
create this behavior based on problem specific elements. Thus, the user may
implement a SystemMerger, which returns a new omrc::SpaceInformation and
omrb::ProblemDefintion pair after coupling individuals index1 and index2
together.K-CBS calls the merge function, as needed, to find solutions. In the
first case, K-CBS is a fully coupled planner. However, experiments show that
this is rarely needed and K-CBS usually finds solutions much before this point.
Nevertheless, the capabilities are possible.
*/
class myDemoSystemMerger : public omrc::SystemMerger {
public:
  myDemoSystemMerger(const omrc::SpaceInformationPtr &si,
                     const omrb::ProblemDefinitionPtr pdef)
      : omrc::SystemMerger(si, pdef){};

  virtual std::pair<const omrc::SpaceInformationPtr,
                    const ompl::multirobot::base::ProblemDefinitionPtr>
  merge(const int index1, const int index2) const override {
    // actually perform the problem set-up here and return the SpaceInformation
    // and ProblemDefinition -- return pair with nullptrs if unable to merge
    if (index1 > 0 && index2 > 0)
      return std::make_pair(nullptr, nullptr);
    else
      return std::make_pair(nullptr, nullptr);
  }
};

class myDemoGoalCondition : public ob::GoalRegion {
public:
  myDemoGoalCondition(const ob::SpaceInformationPtr &si,
                      std::pair<int, int> goal)
      : ob::GoalRegion(si), gx_(goal.first), gy_(goal.second) {
    threshold_ = 0.2;
  }

  double distanceGoal(const ob::State *st) const override {
    const auto *robot_state = st->as<ob::SE2StateSpace::StateType>();
    const double *robot_pos =
        robot_state->as<ob::RealVectorStateSpace::StateType>(0)->values;
    return sqrt(pow(robot_pos[0] - gx_, 2) + pow(robot_pos[1] - gy_, 2));
  }

private:
  int gx_;
  int gy_;
};

void myDemoPropagateFunction(const ob::State *start, const oc::Control *control,
                             const double duration, ob::State *result) {
  const auto *se2state = start->as<ob::SE2StateSpace::StateType>();
  const double *pos =
      se2state->as<ob::RealVectorStateSpace::StateType>(0)->values;
  const double rot = se2state->as<ob::SO2StateSpace::StateType>(1)->value;
  const double *ctrl =
      control->as<oc::RealVectorControlSpace::ControlType>()->values;

  result->as<ob::SE2StateSpace::StateType>()->setXY(
      pos[0] + ctrl[0] * duration * cos(rot),
      pos[1] + ctrl[0] * duration * sin(rot));
  result->as<ob::SE2StateSpace::StateType>()->setYaw(rot + ctrl[1] * duration);
}

// K-CBS and PP both work for multiple type of low-level planners.
// Providing this function to the multi-agent space information will let you use
// any of them
ompl::base::PlannerPtr
myDemoPlannerAllocator(const ompl::base::SpaceInformationPtr &si) {
  const oc::SpaceInformationPtr siC =
      std::static_pointer_cast<ompl::control::SpaceInformation>(si);
  ompl::base::PlannerPtr planner = std::make_shared<oc::RRT>(siC);
  return planner;
}

inline ob::StateSpacePtr
createBounded2ndOrderCarStateSpace(const unsigned int x_max,
                                   const unsigned int y_max) {
  ob::StateSpacePtr space = std::make_shared<ob::CompoundStateSpace>();
  space->as<ob::CompoundStateSpace>()->addSubspace(
      ob::StateSpacePtr(new ob::RealVectorStateSpace(4)), 1.0);
  space->as<ob::CompoundStateSpace>()->addSubspace(
      ob::StateSpacePtr(new ob::SO2StateSpace()), 1.0);
  space->as<ob::CompoundStateSpace>()->lock();

  // set the bounds for the RealVectorStateSpace
  ob::RealVectorBounds bounds(4);
  bounds.setLow(0, 0);      //  x lower bound
  bounds.setHigh(0, x_max); // x upper bound
  bounds.setLow(1, 0);      // y lower bound
  bounds.setHigh(1, y_max); // y upper bound
  bounds.setLow(2, -1);     // v lower bound
  bounds.setHigh(2, 1);     // v upper bound
  bounds.setLow(3, -M_PI);  // phi lower bound
  bounds.setHigh(3, M_PI);  // phi upper bound
  space->as<ob::CompoundStateSpace>()
      ->as<ob::RealVectorStateSpace>(0)
      ->setBounds(bounds);

  return space;
}

inline oc::ControlSpacePtr createCspace(ob::StateSpacePtr &space) {
  auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));

  // set the bounds for the control space
  ob::RealVectorBounds cbounds(2);
  // cbounds.setLow(-1);
  // cbounds.setLow(1);
  cbounds.setLow(0, -1);
  cbounds.setHigh(0, 1);
  cbounds.setLow(1, -M_PI);
  cbounds.setHigh(1, M_PI);
  cspace->setBounds(cbounds);

  return cspace;
}

void plan(const std::string &plannerName, const std::string &caseName,
          const vmap &starts, const vmap &goals, oset &obsts, const Env &env) {
  // construct an instance of multi-robot space information
  auto ma_si(std::make_shared<omrc::SpaceInformation>());
  auto ma_pdef(std::make_shared<omrb::ProblemDefinition>(ma_si));

  for (auto itr = starts.begin(); itr != starts.end(); itr++) {
    // construct the state space we are planning in
    auto space = createBounded2ndOrderCarStateSpace(env.maxx, env.maxy);
    space->setName(itr->first);

    // set the bounds for the R^2 part of SE(2)
    // auto space = std::make_shared<ob::SE2StateSpace>();
    // ob::RealVectorBounds bounds(4);
    // bounds.setLow(0, 0);
    // bounds.setHigh(0, env.maxx);
    // bounds.setLow(1, 0);
    // bounds.setHigh(1, env.maxy);
    // bounds.setLow(2, -1);    // v lower bound
    // bounds.setHigh(2, 1);    // v upper bound
    // bounds.setLow(3, -M_PI); // phi lower bound
    // bounds.setHigh(3, M_PI); // phi upper bound
    // space->setBounds(bounds);

    // create a control space
    auto cspace = createCspace(space);

    // construct an instance of  space information from this control space
    auto si = std::make_shared<oc::SpaceInformation>(space, cspace);

    // set state validity checking for this space
    si->setStateValidityChecker(
        std::make_shared<myDemoStateValidityChecker>(si, env.r, obsts));

    // set the state propagation routine
    si->setStatePropagator(myDemoPropagateFunction);

    // it is highly recommended that all robots use the same propogation step
    // size
    si->setPropagationStepSize(0.1);

    // set this to remove the warning
    si->setMinMaxControlDuration(1, 10);

    // name the state space parameter (not required but helpful for
    // robot-to-robot collision checking)
    si->getStateSpace()->setName(itr->first);

    // create a start state
    ob::ScopedState<ob::SE2StateSpace> start(space);
    start[0] = itr->second.first;
    start[1] = itr->second.second;
    start[2] = 0.0;
    start[3] = 0.0;
    start[4] = 0.0;

    // create a problem instance
    auto pdef = std::make_shared<ob::ProblemDefinition>(si);

    // set the start and goal states
    pdef->addStartState(start);
    pdef->setGoal(
        std::make_shared<myDemoGoalCondition>(si, goals.at(itr->first)));

    // add the individual information to the multi-robot SpaceInformation and
    // ProblemDefinition
    ma_si->addIndividual(si);
    ma_pdef->addIndividual(pdef);
  }

  // lock the multi-robot SpaceInformation and ProblemDefinitions when done
  // adding individuals
  ma_si->lock();
  ma_pdef->lock();

  // set the planner allocator for the multi-agent planner
  ompl::base::PlannerAllocator allocator = myDemoPlannerAllocator;
  ma_si->setPlannerAllocator(allocator);

  if (plannerName == "PP") {
    // plan for all agents using a prioritized planner (PP)
    auto planner = std::make_shared<omrc::PP>(ma_si);
    planner->setProblemDefinition(
        ma_pdef); // be sure to set the problem definition
    bool solved = planner->as<omrb::Planner>()->solve(300.0);

    if (solved) {
      std::cout << "Found solution!" << std::endl;
      omrb::PlanPtr solution = ma_pdef->getSolutionPlan();
      std::ofstream myFile(plannerName + "-" + caseName + "-plan.txt");
      solution->as<omrc::PlanControl>()->printAsMatrix(myFile, "Robot");
    }
  } else if (plannerName == "K-CBS") {
    // plan using Kinodynamic Conflict Based Search
    auto planner = std::make_shared<omrc::KCBS>(ma_si);
    planner->setProblemDefinition(
        ma_pdef); // be sure to set the problem definition

    // set the system merger
    ma_si->setSystemMerger(
        std::make_shared<myDemoSystemMerger>(ma_si, ma_pdef));

    // set the merge bound of K-CBS
    planner->setMergeBound(10);

    // set the low-level solve time
    planner->setLowLevelSolveTime(0.5);

    bool solved = planner->as<omrb::Planner>()->solve(300.0);
    if (solved) {
      std::cout << "Found solution!" << std::endl;
      omrb::PlanPtr solution = ma_pdef->getSolutionPlan();
      std::ofstream myFile(plannerName + "-" + caseName + "-plan.txt");
      solution->as<omrc::PlanControl>()->printAsMatrix(myFile, "Robot");
    }
    // planner.reset();
  }
}

void saveEnvFile(const std::string &caseName, int nBots, const Env &env,
                 const std::vector<Obs> &obsts) {
  /*
      <minx maxx>
      <miny maxy>
      <numBots>
      <botXL botYL> x numBots
      <numObs>
      <xleft yleft xL yL> x numObs
   *
   */
  using namespace std;
  ofstream out(caseName + "-env.txt");
  out << 0 << " " << env.maxx << endl;
  out << 0 << " " << env.maxy << endl;
  out << nBots << endl;
  for (int i = 0; i < nBots; i++) {
    out << env.r << " " << env.r << endl;
  }
  out << obsts.size() << endl;
  for (auto obs : obsts) {
    obs.print(out);
  }
}

void run() {

  std::vector<Obs> rects = {
      // {0.5 + 5, 1.8 + 5, 9.0 + 5, 1.0 + 5},
      {4.0 + 5, 5.0 + 5, 2.0 + 5, 2.0 + 5}
  };
  oset obsts;
  for (const auto &r : rects) {
    obsts.insert(new RectangularObstacle(r.xleft, r.yleft, r.xL, r.yL));
  }

  const vmap starts{
      {"Robot 1", {1.0 + 5, 0.5 + 5}},
      {"Robot 2", {1.0 + 5, 3.5 + 5}},
      {"Robot 3", {9.0 + 5, 0.5 + 5}},
      {"Robot 4", {9.0 + 5, 3.5 + 5}},
  };
  const vmap goals{
      {"Robot 1", {9.0 + 5, 0.5 + 5}},
      {"Robot 2", {9.0 + 5, 9.0 + 5}},
      {"Robot 3", {1.0 + 5, 0.5 + 5}},
      {"Robot 4", {1.0 + 5, 9.0 + 5}},
  };
  Env env{20, 20, 0.1};
  saveEnvFile("n4-20x20", starts.size(), env, rects);
  plan("K-CBS", "n4-20x20", starts, goals, obsts, env);
}

int main() {
  run();
  return 0;
}

