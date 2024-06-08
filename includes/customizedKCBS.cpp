/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Justin Kottinger */

#include "customizedKCBS.h"
#include "ompl/util/Console.h"
#include <thread>

namespace omrc = ompl::multirobot::control;

ompl::multirobot::control::CustomizedKCBS::CustomizedKCBS(
    const ompl::multirobot::control::SpaceInformationPtr &si)
    : ompl::multirobot::base::Planner(si, "K-CBS"), llSolveTime_(1.),
      mergeBound_(std::numeric_limits<int>::max()), numNodesExpanded_(0),
      numApproxSolutions_(0), rootSolveTime_(-1) {
  siC_ = si.get();

  Planner::declareParam<double>(
      "low_level_solve_time", this, &CustomizedKCBS::setLowLevelSolveTime,
      &CustomizedKCBS::getLowLevelSolveTime, "0.:1.:10000000.");
  Planner::declareParam<double>("merge_bound", this,
                                &CustomizedKCBS::setMergeBound,
                                &CustomizedKCBS::getMergeBound, "0:1:10000000");
}

ompl::multirobot::control::CustomizedKCBS::~CustomizedKCBS() { freeMemory(); }

void ompl::multirobot::control::CustomizedKCBS::clear() {
  base::Planner::clear();
  freeMemory();
  numNodesExpanded_ = 0;
  numApproxSolutions_ = 0;
  rootSolveTime_ = -1;
}

void ompl::multirobot::control::CustomizedKCBS::freeMemory() {
  // clear the priority queue
  while (!pq_.empty()) {
    NodePtr n = pq_.top();
    pq_.pop();
    n.reset();
  }
  pq_ = std::priority_queue<NodePtr, std::vector<NodePtr>, NodeCompare>();
  // free memory of every node
  for (auto n : allNodesSet_)
    n.reset();
  allNodesSet_.clear();
  // free memory of every low-level solver
  for (auto &p : llSolvers_)
    p.reset();
  // free memory of all the dynamic obstaces
  for (unsigned int r = 0; r < siC_->getIndividualCount(); r++)
    siC_->getIndividual(r)->clearDynamicObstacles();
  // free memory of the merged planner (if it exists)
  if (mergedPlanner_)
    mergedPlanner_.reset();
  // reset conflict counter
  conflictCounter_.clear();
  // clear the boost graph
  tree_.clear();
  treeMap_.clear();
}

void ompl::multirobot::control::CustomizedKCBS::setup() {
  base::Planner::setup();

  // varify that all robots have the same propagation step-size
  double dt = siC_->getIndividual(0)->getPropagationStepSize();
  for (unsigned int r = 1; r < siC_->getIndividualCount(); r++) {
    double dt_other = siC_->getIndividual(r)->getPropagationStepSize();
    if (dt != dt_other) {
      OMPL_WARN("The propagation step size is different between planners. This "
                "may cause incorrect solutions.");
      return;
    }
  }

  if (!siC_->hasPlannerAllocator())
    throw Exception(getName().c_str(), "No PlannerAllocator provided!");

  // setup low-level planners
  llSolvers_.resize(siC_->getIndividualCount());
  for (unsigned int r = 0; r < siC_->getIndividualCount(); r++) {
    llSolvers_[r] = siC_->allocatePlannerForIndividual(r);
    llSolvers_[r]->setProblemDefinition(pdef_->getIndividual(r));
    // llSolvers_[r]->specs_.approximateSolutions = false; // TO-DO: this will
    // throw an error but it would be nice to set this to false
  }

  // setup conflictCounter_
  for (unsigned int r1 = 0; r1 < siC_->getIndividualCount(); r1++) {
    for (unsigned int r2 = r1 + 1; r2 < siC_->getIndividualCount(); r2++) {
      conflictCounter_.insert({std::make_pair(r1, r2), 0});
    }
  }

  // check if merger is set
  if (!siC_->getSystemMerger())
    OMPL_WARN("%s: SystemMerger not set! Planner will fail if mergeBound_ is "
              "triggered.",
              getName().c_str());
}

void ompl::multirobot::control::CustomizedKCBS::pushNode(const NodePtr &n) {
  // add a node to the tree_ and pq_
  allNodesSet_.insert(n);
  if (n->getID() == -1) {
    boost::graph_traits<BoostGraph>::vertex_descriptor v = add_vertex(n, tree_);
    treeMap_.insert({n->getName(), v});
    if (n->getParent())
      add_edge(treeMap_[n->getParent()->getName()], treeMap_[n->getName()],
               tree_);
  }
  pq_.push(n);
}

ompl::multirobot::control::CustomizedKCBS::NodePtr
ompl::multirobot::control::CustomizedKCBS::popNode() {
  // pop a node and assign it an ID
  NodePtr n = pq_.top();
  if (n->getID() == -1) {
    numNodesExpanded_ += 1;
    n->setID(numNodesExpanded_);
  }
  pq_.pop();
  return n;
}

std::vector<ompl::multirobot::control::CustomizedKCBS::Conflict>
ompl::multirobot::control::CustomizedKCBS::findConflicts(
    const PlanControlPtr &plan) const {
  // cast to desired type and interpolate the paths
  plan->interpolate();

  // get the maximum number of steps we need to simulate
  unsigned int maxSteps = 0;
  for (unsigned int r = 0; r != siC_->getIndividualCount(); r++) {
    if (plan->getPath(r)->getStateCount() > maxSteps)
      maxSteps = plan->getPath(r)->getStateCount();
  }

  // initialize an empty vector of conflicts
  std::vector<Conflict> confs;

  // perform a disjoint check for collision at every step in the plan, assume
  // robots stay in place once they reach goal
  for (unsigned int k = 0; k < maxSteps; k++) {
    for (unsigned int r1 = 0; r1 < siC_->getIndividualCount(); r1++) {
      for (unsigned int r2 = r1 + 1; r2 < siC_->getIndividualCount(); r2++) {
        // get the states for r1 and r2 at step k
        ompl::base::State *state1 = nullptr;
        ompl::base::State *state2 = nullptr;
        if (k < plan->getPath(r1)->getStateCount())
          state1 = plan->getPath(r1)->getState(k);
        else
          state1 = plan->getPath(r1)->getStates().back();

        if (k < plan->getPath(r2)->getStateCount())
          state2 = plan->getPath(r2)->getState(k);
        else
          state2 = plan->getPath(r2)->getStates().back();

        // use state validity checker to perform collision check
        auto otherStatePair = std::make_pair(siC_->getIndividual(r2), state2);
        if (!siC_->getIndividual(r1)->getStateValidityChecker()->areStatesValid(
                state1, otherStatePair)) {
          Conflict c(r1, r2, k, state1, otherStatePair.second);
          confs.push_back(c);
        }
      }
    }
  }
  return confs;
}

void ompl::multirobot::control::CustomizedKCBS::updateConflictCounter(
    const std::vector<Conflict> &conflicts) {
  // update the conflictCounter map with the newly found conflicts
  for (auto &c : conflicts)
    conflictCounter_[std::make_pair(c.robots_[0], c.robots_[1])] += 1;
}

std::pair<int, int> ompl::multirobot::control::CustomizedKCBS::mergeNeeded() {
  // iterate through all of the possible merge pairs and check if any pairs have
  // too many conflicts. If so, return the pair. Otherwise, return (-1, -1)
  // for (auto itr = conflictCounter_.begin(); itr != conflictCounter_.end();
  //      itr++) {
  //   if (itr->second > mergeBound_)
  //     return itr->first;
  // }
  return std::make_pair(-1, -1);
}

const ompl::multirobot::control::CustomizedKCBS::ConstraintPtr
ompl::multirobot::control::CustomizedKCBS::createConstraint(
    const unsigned int index, std::vector<Conflict> &confs) {
  // create new constraint for robot that avoids other_robot
  unsigned int other_index = (index == 0) ? 1 : 0;
  const ConstraintPtr constraint = std::make_shared<Constraint>(
      confs.front().robots_[index],
      siC_->getIndividual(confs.front().robots_[other_index]));
  for (auto &c : confs) {
    bool idx_exists =
        std::find(std::begin(c.robots_), std::end(c.robots_),
                  confs.front().robots_[index]) != std::end(c.robots_);
    bool other_exists =
        std::find(std::begin(c.robots_), std::end(c.robots_),
                  confs.front().robots_[other_index]) != std::end(c.robots_);
    if (idx_exists && other_exists) {
      constraint->timeSteps_.push_back(c.timeStep_);
      constraint->constrainingStates_.push_back(c.states_[other_index]);
    }
  }
  return constraint;
}

void ompl::multirobot::control::CustomizedKCBS::attemptReplan(
    const unsigned int robot, NodePtr node, const bool retry) {
  // collect all of the constraints on robot by traversing constraint tree back
  // to root node
  auto nCpy = node;
  std::vector<ConstraintPtr> constraints;
  while (nCpy->getConstraint()) {
    if (nCpy->getConstraint()->constrainedRobot_ == robot)
      constraints.push_back(nCpy->getConstraint());
    nCpy = nCpy->getParent();
  }

  // clear existing low-level planner data and existing dynamic obstacles
  siC_->getIndividual(robot)->clearDynamicObstacles();

  // add the new dynamic obstacles (the constraints)
  for (ConstraintPtr &c : constraints) {
    for (unsigned int k = 0; k < c->timeSteps_.size(); k++) {
      ompl::base::State *state =
          c->constrainingSiC_->cloneState(c->constrainingStates_[k]);
      const double time = c->timeSteps_[k] *
                          siC_->getIndividual(robot)->getPropagationStepSize();
      siC_->getIndividual(robot)->addDynamicObstacle(time, c->constrainingSiC_,
                                                     state);
    }
  }

  llSolvers_[robot]->clear();
  llSolvers_[robot]->getProblemDefinition()->clearSolutionPaths();

  // attempt to find another trajectory
  // if successful, add the new plan to node prior to exit
  ompl::base::PlannerStatus solved;
  if (retry)
    solved = node->getLowLevelSolver()->solve(llSolveTime_);
  else
    solved = llSolvers_[robot]->solve(llSolveTime_);

  if (solved == ompl::base::PlannerStatus::EXACT_SOLUTION) {
    PlanControlPtr new_plan = std::make_shared<PlanControl>(si_);
    ompl::control::PathControlPtr new_path = nullptr;
    if (retry)
      new_path = std::make_shared<ompl::control::PathControl>(
          *node->getLowLevelSolver()
               ->getProblemDefinition()
               ->getSolutionPath()
               ->as<ompl::control::PathControl>());
    else
      new_path = std::make_shared<ompl::control::PathControl>(
          *llSolvers_[robot]
               ->getProblemDefinition()
               ->getSolutionPath()
               ->as<ompl::control::PathControl>());

    for (unsigned int r = 0; r < siC_->getIndividualCount(); r++) {
      if (r == robot)
        new_plan->append(new_path);
      else
        new_plan->append(node->getParent()->getPlan()->getPath(r));
    }
    node->setPlan(new_plan);
    std::vector<Conflict> confs = findConflicts(node->getPlan());
    node->setConflicts(confs);
    // node->setCost(nodeCost(node));
    node->setCost(evaluateCost(confs)); // cost metric is undefined for this
    // portion bc there are no conflicts
  } else {
    numApproxSolutions_ += 1;
    // save the planner prior to exit only if planner not already saved
    if (!retry) {
      // need to save the existing low-level solver to the node and create a new
      // one for the rest of the system
      node->setLowLevelSolver(llSolvers_[robot]);
      llSolvers_[robot] = siC_->allocatePlannerForIndividual(robot);
      llSolvers_[robot]->setProblemDefinition(pdef_->getIndividual(robot));
    }
  }
  pushNode(node);
}

void ompl::multirobot::control::CustomizedKCBS::parallelRootSolutionHelper(
    PlanControlPtr plan, unsigned int startIdx, unsigned int endIdx,
    const ompl::base::PlannerTerminationCondition &ptc) {
  for (unsigned int i = startIdx; i < endIdx; i++) {
    while (!llSolvers_[i]->getProblemDefinition()->hasExactSolution() && !ptc)
      llSolvers_[i]->solve(llSolveTime_);
    auto path = std::make_shared<ompl::control::PathControl>(
        *llSolvers_[i]
             ->getProblemDefinition()
             ->getSolutionPath()
             ->as<ompl::control::PathControl>());
    plan->replace(i, path);
  }
}

std::vector<unsigned int>
ompl::multirobot::control::CustomizedKCBS::split(const unsigned int jobs,
                                                 const unsigned int workers) {
  std::vector<unsigned int> num_jobs_per_workers;
  // If x % n == 0 then the minimum
  // difference is 0 and all
  // numbers are x / n
  if (jobs % workers == 0) {
    for (unsigned int i = 0; i < workers; i++)
      num_jobs_per_workers.push_back(jobs / workers);
  } else {

    // up to n-(x % n) the values will be x / n
    // after that, the values will be x / n + 1
    unsigned int zp = workers - (jobs % workers);
    unsigned int pp = jobs / workers;
    for (unsigned int i = 0; i < workers; i++) {
      if (i >= zp)
        num_jobs_per_workers.push_back(pp + 1);
      else
        num_jobs_per_workers.push_back(pp);
    }
  }
  return num_jobs_per_workers;
}

void ompl::multirobot::control::CustomizedKCBS::parallelRootSolution(
    PlanControlPtr plan, const ompl::base::PlannerTerminationCondition &ptc) {
  // Create an array of threads.
  std::vector<std::thread> threads;

  // Divide the loop into equal segments.
  const unsigned int num_workers =
      std::min(siC_->getIndividualCount(),
               numThreads_); // check that there is a way for the system to tell
                             // us how many threads there are

  auto jobs_for_worker = split(siC_->getIndividualCount(), num_workers);

  OMPL_INFORM("%s: Assigned %d workers to plan paths for %d robots.",
              getName().c_str(), num_workers, siC_->getIndividualCount());

  // Create a thread for each segment.
  unsigned int start = 0;
  unsigned int end = jobs_for_worker[0];
  for (unsigned int i = 0; i < num_workers; i++) {
    threads.push_back(std::thread(
        &ompl::multirobot::control::CustomizedKCBS::parallelRootSolutionHelper,
        this, plan, start, end, ptc));

    // update start and end
    start = end;
    end += jobs_for_worker[i + 1];
  }

  // Join all of the threads.
  for (auto &thread : threads)
    thread.join();
}

void ompl::multirobot::control::CustomizedKCBS::parallelNodeExpansion(
    NodePtr &solution, std::vector<unsigned int> &resevered,
    std::pair<int, int> &merge_indices, std::mutex &lock) {
  // if (solution) // another thread beat this one to a solution
  //     return;

  // get the best unexplored node in the constraint tree
  NodePtr currentNode = popNode();
  OMPL_INFORM("%s: selected node with cost %d.", getName().c_str(),
              currentNode->getCost());

  // if current node has not plan, then attempt to find one again
  if (currentNode->getCost() == std::numeric_limits<double>::max()) {
    // check it constrained robot is already reserved by someone else
    auto reserved_itr =
        std::find(resevered.begin(), resevered.end(),
                  currentNode->getConstraint()->constrainedRobot_);
    if (reserved_itr == resevered.end()) // not reserved, proceed to plan for it
    {
      // use existing tree to attempt a replan
      // reserve robot
      resevered.push_back(currentNode->getConstraint()->constrainedRobot_);
      // attempt replan
      attemptReplan(currentNode->getConstraint()->constrainedRobot_,
                    currentNode, true);
    } else // already reserved, just push to queue for later use
      pushNode(currentNode);
  } else {
    // find conflicts in the current plan
    std::vector<Conflict> confs = currentNode->getConflicts();

    // if no conflicts were found, return as solution
    if (confs.empty()) {
      lock.lock();
      if (solution == nullptr || nodeCost(solution) > nodeCost(currentNode)) {
        solution = currentNode;
        auto tnow = std::chrono::steady_clock::now();
        auto dur = std::chrono::duration<double>(tnow - tstart).count();
        OMPL_WARN("Update solution to %.2f, took %.2f s",
                  nodeCost(currentNode), dur);
        OMPL_WARN("Save to file [%s].", this->solfile_.c_str());
        std::ofstream resfile(this->solfile_);
        solution->getPlan()->as<omrc::PlanControl>()->printAsMatrix(resfile,
                                                                    "Robot");
        resfile << "==============" << std::endl;
        resfile << "runtime " << dur << std::endl;
        resfile << "cost " << nodeCost(currentNode) << std::endl;
        resfile.close();

        auto pos = solfile_.find("kcbs.plan");
        std::string logfile = solfile_.substr(0, pos) + "kcbs.log";
        OMPL_WARN("Log to file [%s].", logfile.c_str());
        std::ofstream log(logfile, std::ios::app);
        log << dur << "," << nodeCost(currentNode) << std::endl;
      }
      lock.unlock();
      return;
    }

    // for debugging
    // for (auto &c: confs)
    // {
    //     std::cout << "conflict between " << c.robots_[0] << " and " <<
    //     c.robots_[1] << " at time " << c.timeStep_ << " with states " <<
    //     c.states_[0] << " and " << c.states_[1]  << std::endl;
    // }

    // FIXME: need to keep the merge logic somehow
    // if merge is needed, then merge and restart
    merge_indices = mergeNeeded();
    if (merge_indices != std::make_pair(-1, -1))
      return;

    // create a constraint for every agent in confs
    // for example, if conflicts occur between robots 0 and 2 for the interval
    // dt=[615, 665] then constraint1 is given to robot 0 which forces it to
    // avoid the states of robot 2 for all steps inside dt=[615, 665]
    // constraint2 is given to robot 2 which forces it to avoid the states of
    // robot 0 for all steps inside dt=[615, 665] then, replan for robots 0 and
    // 2 after adding the constraints as dynamic obstacles

    std::vector<ConstraintPtr> new_constraints;
    for (unsigned int r = 0; r < 2; r++) {
      // create a new constraint
      ConstraintPtr new_constraint = createConstraint(r, confs);
      new_constraints.push_back(new_constraint);
      // check it constrained robot is already reserved by someone else
      auto reserved_itr = std::find(resevered.begin(), resevered.end(),
                                    new_constraint->constrainedRobot_);
      if (reserved_itr !=
          resevered.end()) // reserved and cannot expand from this node
      {
        // push currentNode back into the queue and end function call
        pushNode(currentNode);
        return;
      }
    }

    // good news! -- we can expand from this node
    std::vector<std::thread> threads;
    for (unsigned int r = 0; r < 2; r++) {
      // create a new node to house the new constraint, also assign a parent
      NodePtr nxtNode = std::make_shared<Node>();
      nxtNode->setParent(currentNode);
      nxtNode->setConstraint(new_constraints[r]);
      // reserve robot
      resevered.push_back(new_constraints[r]->constrainedRobot_);
      // attempt to replan and push node to priority queue
      threads.push_back(std::thread(
          &ompl::multirobot::control::CustomizedKCBS::attemptReplan, this,
          new_constraints[r]->constrainedRobot_, nxtNode, false));
    }
    // Join all of the threads.
    for (auto &thread : threads) {
      thread.join();
    }
  }
}

double ompl::multirobot::control::CustomizedKCBS::nodeCost(NodePtr n) {
  auto plan = n->getPlan();
  // int res = 0;
  double dist = 0;
  for (unsigned int r = 0; r != siC_->getIndividualCount(); r++) {
    // res += plan->getPath(r)->getStateCount();
    auto path = plan->getPath(r);
    for (int i = 0; i + 1 < path->getStateCount(); i++) {
      auto s0 = path->getState(i);
      auto s1 = path->getState(i + 1);
      dist += siC_->getIndividual(r)->getStateSpace()->distance(s0, s1);
    }
  }
  return dist;
}

int ompl::multirobot::control::CustomizedKCBS::evaluateCost(
    const std::vector<Conflict> confs) {
  // // update the conflictCounter_;
  // updateConflictCounter(confs);

  // // find the conflicts with unique robot pairs
  // std::vector<Conflict> unique_pairs;
  // std::unique_copy(confs.begin(), confs.end(),
  // std::back_inserter(unique_pairs),
  //                  [](Conflict c1, Conflict c2) { return c1.robots_[0] ==
  //                  c2.robots_[0] && c1.robots_[1] == c2.robots_[1]; });

  // find the conflicts with unique robot pairs
  std::vector<Conflict> unique_pairs;
  for (auto &c : confs) {
    auto c_itr = std::find_if(unique_pairs.begin(), unique_pairs.end(),
                              [&c](const Conflict &other) {
                                return (c.robots_[0] == other.robots_[0] &&
                                        c.robots_[1] == other.robots_[1]);
                              });
    if (c_itr == unique_pairs.end())
      unique_pairs.push_back(c);
  }
  updateConflictCounter(unique_pairs);
  // return the number of unique pairs we have minus the first one (it will be
  // resolved)
  return unique_pairs.size();
}

ompl::base::PlannerStatus ompl::multirobot::control::CustomizedKCBS::solve(
    const ompl::base::PlannerTerminationCondition &ptc) {
  checkValidity();

  OMPL_INFORM("%s: Merge Bound set to %d", getName().c_str(), mergeBound_);
  OMPL_INFORM("%s: Starting planning. ", getName().c_str());

  // start the timer for root solution
  auto start = std::chrono::high_resolution_clock::now();
  // initialize the initial plan and fill it with the required size
  PlanControlPtr initalPlan = std::make_shared<PlanControl>(si_);
  for (unsigned int i = 0; i < siC_->getIndividualCount(); i++) {
    auto dummy_path =
        std::make_shared<ompl::control::PathControl>(siC_->getIndividual(i));
    initalPlan->append(dummy_path);
  }

  // get the initial solution
  parallelRootSolution(initalPlan, ptc);

  auto end = std::chrono::high_resolution_clock::now();
  auto duration_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  double duration_s = (duration_ms.count() * 0.001);
  rootSolveTime_ = duration_s;

  NodePtr solution = nullptr;
  bool solved = false;
  std::pair<int, int> merge_indices{-1, -1};

  // create root node
  NodePtr root = std::make_shared<Node>(initalPlan);
  std::vector<Conflict> confs = findConflicts(root->getPlan());
  if (confs.empty())
    solution = root; // found a solution in root node
  else {
    // for debugging
    root->setConflicts(confs);
    root->setCost(
        evaluateCost(confs) // cost for root node is technically undefined
        // nodeCost(root) // cost for root node is technically undefined
    );
    pushNode(root);
  }

  std::vector<unsigned int> resevered;
  std::mutex lock;
  while (!pq_.empty()) {
    // use multiple threads to expand multiple nodes at once
    const unsigned int numNodesInQueue = pq_.size();
    const unsigned int test = std::floor(numThreads_ / 2);
    const unsigned int numNodesSelect = std::min(numNodesInQueue, test);
    std::vector<std::thread> threads;
    for (unsigned int i = 0; i < numNodesSelect; i++) {
      threads.push_back(std::thread(
          &ompl::multirobot::control::CustomizedKCBS::parallelNodeExpansion,
          this, std::ref(solution), std::ref(resevered),
          std::ref(merge_indices), std::ref(lock)));
      std::this_thread::sleep_for(std::chrono::milliseconds(
          100)); // wait 0.1 seconds s.t. reserved is updated properly
    }
    // Join all of the threads.
    for (auto &thread : threads) {
      thread.join();
    }
    resevered.clear();
    if (merge_indices != std::make_pair(-1, -1)) {
      if (!siC_->getSystemMerger())
        OMPL_ERROR(
            "%s: Merge was triggered but no SystemMerger was provided. Unable "
            "to continue planning. Please fix your system merger.",
            getName().c_str());
      else {
        OMPL_INFORM("%s: Merge was triggered. Composing individuals %d and %d.",
                    getName().c_str(), merge_indices.first,
                    merge_indices.second);
        std::pair<const SpaceInformationPtr,
                  const ompl::multirobot::base::ProblemDefinitionPtr>
            new_defs = siC_->merge(merge_indices.first, merge_indices.second);
        if (new_defs.first && new_defs.second) {
          mergedPlanner_ = std::make_shared<CustomizedKCBS>(new_defs.first);
          mergedPlanner_->setLowLevelSolveTime(llSolveTime_);
          mergedPlanner_->setNumThreads(numThreads_);
          mergedPlanner_->setMergeBound(mergeBound_);
          mergedPlanner_->setProblemDefinition(new_defs.second);
          bool merge_solved = mergedPlanner_->solve(ptc);
          // create a new node to house the new constraint, also assign a parent
          if (merge_solved) {
            solution = std::make_shared<Node>();
            PlanControlPtr sol_plan = std::make_shared<PlanControl>(si_);
            for (unsigned int i = 0; i < new_defs.first->getIndividualCount();
                 i++)
              sol_plan->append(new_defs.second->getSolutionPlan()
                                   ->as<PlanControl>()
                                   ->getPath(i));
            solution->setPlan(sol_plan);
          }
        } else
          OMPL_ERROR("%s: SystemMerge was triggered but failed. Unable to "
                     "expand continue planning. Please fix your system merger.",
                     getName().c_str());
      }
      break;
    }
    if (ptc)
      break;
  }
  // save solution to file before segment fault
  if (solution != nullptr) {
    std::ofstream resfile(this->solfile_);
    solution->getPlan()->as<omrc::PlanControl>()->printAsMatrix(resfile,
                                                                "Robot");
    auto tnow = std::chrono::steady_clock::now();
    auto dur = std::chrono::duration<double>(tnow - tstart).count();
    resfile << "==============" << std::endl;
    resfile << "runtime " << dur << std::endl;
    resfile << "cost " << nodeCost(solution) << std::endl;
    resfile.close();
  }

  if (solution == nullptr) {
    OMPL_INFORM("%s: No solution found.", getName().c_str());
    return {solved, false};
  } else {
    if (solution->getID() == -1) {
      boost::graph_traits<BoostGraph>::vertex_descriptor v =
          add_vertex(solution, tree_);
      treeMap_.insert({solution->getName(), v});
      if (solution->getParent())
        add_edge(treeMap_[solution->getParent()->getName()],
                 treeMap_[solution->getName()], tree_);
      numNodesExpanded_ += 1;
      solution->setID(numNodesExpanded_);
    }
    solved = true;
    OMPL_INFORM("%s: Found Solution!", getName().c_str());
    pdef_->addSolutionPlan(solution->getPlan(), false, false, getName());
    OMPL_INFORM("%s: Planning Complete.", getName().c_str());
    return {solved, false};
  }
}

void ompl::multirobot::control::CustomizedKCBS::getPlannerData(
    ompl::base::PlannerData &data) const {
  std::cout << "getPlannerData() called" << std::endl;
}
