#include "expr.h"
#include "ControlSpaceDatabase.h"
#include "GoalRegionDatabase.h"
#include "PlannerAllocatorDatabase.h"
#include "Robot.h"
#include "SimpleStateValidityChecker.h"
#include "StatePropagatorDatabase.h"
#include "StateSpaceDatabase.h"

#include <map>
#include <math.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/multirobot/control/planners/kcbs/KCBS.h>
#include <ompl/multirobot/control/planners/pp/PP.h>

namespace omrb = ompl::multirobot::base;
namespace omrc = ompl::multirobot::control;
namespace ob = ompl::base;
namespace oc = ompl::control;

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
  // result->as<ob::SE2StateSpace::StateType>()->setYaw(rot + ctrl[1] *
  // duration);
  result->as<ob::SE2StateSpace::StateType>()->setYaw(ctrl[1]);
}

void plan(const std::string &plannerName, const std::string &caseName,
          const vmap &starts, const vmap &goals, oset &obsts, const Env &env) {

  // construct all of the robots
  std::unordered_map<std::string, Robot *> robot_map;
  for (auto itr = starts.begin(); itr != starts.end(); itr++) {
    Robot *robot = new RectangularRobot(itr->first, env.xL, env.yL);
    robot_map[itr->first] = robot;
  }

  // construct an instance of multi-robot space information
  auto ma_si(std::make_shared<omrc::SpaceInformation>());
  auto ma_pdef(std::make_shared<omrb::ProblemDefinition>(ma_si));

  // construct four individuals that operate in SE3
  for (auto itr = starts.begin(); itr != starts.end(); itr++) {
    // construct the state space we are planning in
    // auto space = createBounded2ndOrderCarStateSpace(env.maxx, env.maxy);
    auto space = createFirstOrderStateSpace(env.maxx, env.maxy);

    // name the state space parameter
    space->setName(itr->first);

    // create a control space
    // auto cspace = createUniform2DRealVectorControlSpace(space);
    auto cspace = createFirstOrderControlSpace(space);

    // construct an instance of  space information from this control space
    auto si(std::make_shared<oc::SpaceInformation>(space, cspace));

    // // set state validity checking for this space
    // si->setStateValidityChecker(
    // std::make_shared<homogeneous2ndOrderCarSystemSVC>(si, robot_map, obsts));
    si->setStateValidityChecker(
        std::make_shared<SimpleValidityChecker>(si, robot_map, obsts));

    // set the state propagation routine

    // si->setStatePropagator(myDemoPropagateFunction);
    auto odeSolver(
        std::make_shared<oc::ODEBasicSolver<>>(si, &FirstOrderCarODE));
    si->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver));
    // auto odeSolver(
    //     std::make_shared<oc::ODEBasicSolver<>>(si, &SecondOrderCarODE));
    // si->setStatePropagator(oc::ODESolver::getStatePropagator(
    //     odeSolver, &SecondOrderCarODEPostIntegration));

    // set the propagation step size
    si->setPropagationStepSize(0.1);

    // set this to remove the warning
    si->setMinMaxControlDuration(1, 10);

    // create a start state
    ob::ScopedState<> start(space);
    start[0] = starts.at(itr->first).first;
    start[1] = starts.at(itr->first).second;
    start[2] = 0.0;
    start[3] = 0.0;
    start[4] = 0.0;

    // // create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // set the start and goal states
    pdef->addStartState(start);
    pdef->setGoal(std::make_shared<GoalRegion2ndOrderCar>(
        si, goals.at(itr->first).first, goals.at(itr->first).second));

    // add the individual information to the multi-robot SpaceInformation and
    // ProblemDefinition
    ma_si->addIndividual(si);
    ma_pdef->addIndividual(pdef);
  }

  // set the planner allocator for the multi-agent planner
  ompl::base::PlannerAllocator allocator = &allocateControlRRT;
  ma_si->setPlannerAllocator(allocator);

  ma_si->lock();
  ma_pdef->lock();

  omrb::PlannerPtr planner = nullptr;
  if (plannerName == "K-CBS") {
    // set the system merger
    // auto merger = std::make_shared<homogeneous2ndOrderCarSystemMerger>(
    //     ma_si, ma_pdef, robot_map, obsts, 10, starts, goals);
    auto merger = std::make_shared<myDemoSystemMerger>(ma_si, ma_pdef);
    ma_si->setSystemMerger(merger);
    // plan using Kinodynamic Conflict Based Search
    planner = std::make_shared<omrc::KCBS>(ma_si);
    planner->as<omrc::KCBS>()->setLowLevelSolveTime(5.);
    planner->as<omrc::KCBS>()->setMergeBound(1000);
  } else {
    // plan using Prioritized Planner
    planner = std::make_shared<omrc::PP>(ma_si);
  }

  planner->setProblemDefinition(
      ma_pdef); // be sure to set the problem definition

  auto start = std::chrono::high_resolution_clock::now();
  bool solved = planner->as<omrb::Planner>()->solve(300.0);
  auto end = std::chrono::high_resolution_clock::now();
  auto duration_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  double duration_s = (duration_ms.count() * 0.001);

  if (solved) {
    printf("Found Solution in %0.2f seconds!\n", duration_s);
    omrb::PlanPtr solution = ma_pdef->getSolutionPlan();
    std::ofstream myFile(plannerName + "-" + caseName + "-plan.txt");
    solution->as<omrc::PlanControl>()->printAsMatrix(myFile, "Robot");
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
    out << env.xL << " " << env.yL << endl;
  }
  out << obsts.size() << endl;
  for (auto obs : obsts) {
    obs.print(out);
  }
}

void run() {
  std::vector<Obs> rects = {// {0.5 + 5, 1.8 + 5, 9.0 + 5, 1.0 + 5},
                            // {4.0 + 5, 5.0 + 5, 2.0 + 5, 2.0 + 5}
                            {0.5 + 5.5, 1.8 + 5.0, 9.0, 1.0},
                            {4.0 + 5.5, 5.0 + 5.0, 2.0, 2.0}};
  oset obsts;
  for (const auto &r : rects) {
    obsts.insert(new RectangularObstacle(r.xleft, r.yleft, r.xL, r.yL));
  }
  // new RectangularObstacle(0.5, 1.8, 9.0, 1.0),
  // new RectangularObstacle(4.0, 5.0, 2.0, 2.0),
  const vmap starts{
      {"Robot 1", {1.0 + 5.0, 0.5 + 5.0}},
      {"Robot 2", {1.0 + 5.0, 3.5 + 5.0}},
      {"Robot 3", {9.0 + 5.0, 0.5 + 5.0}},
      {"Robot 4", {9.0 + 5.0, 3.5 + 5.0}},

  };
  const vmap goals{
      {"Robot 1", {9.0 + 5.0, 0.5 + 5.0}},
      {"Robot 2", {9.0 + 5.0, 9.0 + 5.0}},
      {"Robot 3", {1.0 + 5.0, 0.5 + 5.0}},
      {"Robot 4", {1.0 + 5.0, 9.0 + 5.0}},
  };
  Env env{20, 20, 1, 1};

  saveEnvFile("n4-10x10-rect2", starts.size(), env, rects);

  // plan("PP", "n4-10x10-rect2", starts, goals, obsts, env);
  plan("K-CBS", "n4-10x10-rect2", starts, goals, obsts, env);
}

int main() {
  run();
  return 0;
}
