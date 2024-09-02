#include "expr.h"
#include "ControlSpaceDatabase.h"
#include "GoalRegionDatabase.h"
#include "PlannerAllocatorDatabase.h"
#include "Robot.h"
#include "SimpleStateValidityChecker.h"
#include "SystemMergerDatabase.h"
#include "StatePropagatorDatabase.h"
#include "StateSpaceDatabase.h"
#include "customizedKCBS.h"

#include <map>
#include <math.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/multirobot/control/planners/pp/PP.h>
#include <ompl/util/Console.h>
#include <string>

namespace omrb = ompl::multirobot::base;
namespace omrc = ompl::multirobot::control;
namespace ob = ompl::base;
namespace oc = ompl::control;
namespace msg = ompl::msg;

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

void plan(const std::string &plannerName, const std::string &resfile, const Scen& scen, double timelimit, double stepsize) {

  // construct all of the robots
  std::unordered_map<std::string, Robot *> robot_map;
  double size = 0;
  for (auto bot: scen.bots) {
    Robot *robot = new RectangularRobot(bot.name, bot.xL, bot.yL);
    size = fmax(size, fmax(bot.xL, bot.yL));
    robot_map[bot.name] = robot;
  }
  vmap starts = scen.starts();
  vmap goals = scen.goals();
  oset obsts = scen.obsts();
  const Env& env = scen.env;

  // construct an instance of multi-robot space information
  auto ma_si(std::make_shared<omrc::SpaceInformation>());
  auto ma_pdef(std::make_shared<omrb::ProblemDefinition>(ma_si));

  // construct four individuals that operate in SE3
  for (auto itr = starts.begin(); itr != starts.end(); itr++) {
    // construct the state space we are planning in
    // auto space = createBounded2ndOrderCarStateSpace(env.maxx, env.maxy);
    auto space = createFirstOrderStateSpace(env.minx, env.maxx, env.miny, env.maxy, env.maxv, size);

    // name the state space parameter
    space->setName(itr->first);

    // create a control space
    // auto cspace = createUniform2DRealVectorControlSpace(space);
    auto cspace = createFirstOrderControlSpace(space, env.maxv);

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
    si->setPropagationStepSize(stepsize);

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
    auto merger = std::make_shared<homogeneous2ndOrderCarSystemMerger>(
        ma_si, ma_pdef, robot_map, obsts, 10, starts, goals);
    // auto merger = std::make_shared<myDemoSystemMerger>(ma_si, ma_pdef);
    ma_si->setSystemMerger(merger);
    // plan using Kinodynamic Conflict Based Search
    planner = std::make_shared<omrc::CustomizedKCBS>(ma_si);
    planner->as<omrc::CustomizedKCBS>()->setLowLevelSolveTime(5.);
    planner->as<omrc::CustomizedKCBS>()->setMergeBound(1000);
    planner->as<omrc::CustomizedKCBS>()->setSolfile(resfile);
  } else {
    // plan using Prioritized Planner
    planner = std::make_shared<omrc::PP>(ma_si);
  }

  planner->setProblemDefinition(
      ma_pdef); // be sure to set the problem definition

  auto start = std::chrono::high_resolution_clock::now();
  bool solved = planner->as<omrb::Planner>()->solve(timelimit);
  auto end = std::chrono::high_resolution_clock::now();
  auto duration_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  double duration_s = (duration_ms.count() * 0.001);

  if (solved) {
    printf("Found Solution in %0.2f seconds!\n", duration_s);
    // omrb::PlanPtr solution = ma_pdef->getSolutionPlan();
    // std::ofstream myFile(resfile);
    // solution->as<omrc::PlanControl>()->printAsMatrix(myFile, "Robot");
    // std::cout << "Length:" << solution->length() << std::endl;
  }
}

void run(std::string scenfile, std::string resfile, double timelimit, double stepsize) {
  Scen scen;
  scen.read(scenfile);
  msg::setLogLevel(msg::LOG_WARN);
  plan("K-CBS", resfile, scen, timelimit, stepsize);
}

int main(int argc, char** argv) {
  // ./bin/expr <scenfile> <resfile>
  if (argc < 3) {
    std::cout << "Run cmd like this: ./expr <scenfile> <resfile>" << std::endl;
    return 0;
  }
  std::string scenfile = std::string(argv[1]);
  std::string resfile = std::string(argv[2]);
  double timelimit = 300, stepsize=0.2;
  if (argc >= 4) {
    timelimit = std::stof(argv[3]);
  }
	if (argc >= 5) {
		stepsize = std::stof(argv[4]);
	}
  run(scenfile, resfile, timelimit, stepsize);
  return 0;
}
