
#include "Robot.h"
#include "Obstacle.h"
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <set>

namespace ob = ompl::base;

class SimpleValidityChecker: public ob::StateValidityChecker
{
public:
    SimpleValidityChecker(const ob::SpaceInformationPtr &si, std::unordered_map<std::string, Robot*> robots, std::set<Obstacle*> obs_set = {}): 
        robot1_name_(si->getStateSpace()->getName()), robot1_(robots.at(robot1_name_)), rad1_(robot1_->getBoundingRadius()), 
        robots_(robots), ob::StateValidityChecker(si), obstacles_(obs_set)
    {
    }

    // Answers the question: is the robot described by `si_` at `state` valid?
    bool isValid(const ompl::base::State *state) const override
    {
        // check if state is within bounds
        if (!si_->satisfiesBounds(state))
            return false;

        // get the position of the robot
        const double* this_pos = state->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(0)->values;
        for (auto o_itr = obstacles_.begin(); o_itr != obstacles_.end(); ++o_itr)
        {
            const double* other_pos = (*o_itr)->getCenterPoint();
            const double other_rad = (*o_itr)->getBoundingRadius();
            if (!performQuickCollisionCheck(this_pos, other_pos, other_rad))
            {
                if (!performExactCollisionCheck(this_pos, other_pos, *o_itr))
                    return false;
            }
        }
        return true;
    }

    // Answers the question: does the robot described by `si_` at `state1` avoid collision with some other robot described by a different `si` located at `state2`?
    bool areStatesValid(const ompl::base::State* state1, const std::pair<const ompl::base::SpaceInformationPtr,const ompl::base::State*> state2) const override
    {
        // std::cout << "HERE!!!" << std::endl;
        // get the correct other robots information
        const std::string robot2_name = state2.first->getStateSpace()->getName();
        // get the pos and orientation of this robot
        const double* this_pos = state1->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(0)->values;
        
        if (robot2_name.find("and") != robot2_name.npos)
        {
            // we are comparing against a composed robot, need to do some extra work
            Robot* other_robot = robots_.at(robot2_name);
            for (unsigned int idx = 0; idx < 2; idx++)
            {
                // get the bounding radius of idx robot
                const double rad2 = other_robot->as<CompoundRobot>()->getRobot(idx)->getBoundingRadius();
                const double* other_pos = state2.second->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(2 * idx)->values;

                // perform quick collision check
                if (!performQuickCollisionCheck(this_pos, other_pos, rad2))
                {
                    // collision possible, must check exact
                    if (!performExactCollisionCheck(this_pos, other_pos, other_robot))
                        return false;
                }   
            }
            return true;
        }
        else
        {
            // comparing against a non-composed robot
            Robot* other_robot = robots_.at(robot2_name);
            const double rad2 = other_robot->getBoundingRadius();
            const double* other_pos = state2.second->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(0)->values;

            // perform quick collision check
            if (!performQuickCollisionCheck(this_pos, other_pos, rad2))
            {
                // collision possible, must check exact
                if (!performExactCollisionCheck(this_pos, other_pos, other_robot))
                    return false;
            }
            return true;
        }
    }

    void updateRobotMap(std::unordered_map<std::string, Robot*> new_robots)
    {
        robots_ = new_robots;
    }

private:
    // returns true if no collision is possible (i.e. robots are too far away to collide)
    bool performQuickCollisionCheck(const double* this_pos, const double* other_pos, const double other_rad) const
    {
        const double distance = sqrt(pow(this_pos[0] - other_pos[0], 2) + pow(this_pos[1] - other_pos[1], 2));
        if (distance > (rad1_ + other_rad))
            return true;
        else
            return false;
    }

    // returns true if no collision
    bool performExactCollisionCheck(const double* this_pos, const double* other_pos, const Robot* other_robot) const
    {
        BoostPolygon poly1 = robot1_->getShape();
        // use boost transform to create polygon at true robots location
        BoostPolygon tmp1;
        BoostPolygon result1;
        boost::geometry::correct(tmp1);
        boost::geometry::assign(tmp1, poly1);
        boost::geometry::strategy::transform::matrix_transformer<double, 2, 2> xfrm1(
                 1, 0, this_pos[0],
                 0, 1, this_pos[1],
                 0, 0, 1);
        boost::geometry::transform(tmp1, result1, xfrm1);
        boost::geometry::correct(result1);

        BoostPolygon poly2 = other_robot->getShape();
        // use boost transform to create polygon at true robots location
        BoostPolygon tmp2;
        BoostPolygon result2;
        boost::geometry::correct(tmp2);
        boost::geometry::assign(tmp2, poly2);
        boost::geometry::strategy::transform::matrix_transformer<double, 2, 2> xfrm2(
                 1, 0, other_pos[0],
                 0, 1, other_pos[1],
                 0, 0, 1);
        boost::geometry::transform(tmp2, result2, xfrm2);
        boost::geometry::correct(result2);

        // check if resulting polygons are in collision
        if (!boost::geometry::disjoint(result1, result2))
            return false;
        else
            return true;
    }

    // returns true if no collision
    bool performExactCollisionCheck(const double* this_pos, const double* other_pos, const Obstacle* obstacle) const
    {
        BoostPolygon poly1 = robot1_->getShape();
        // use boost transform to create polygon at true robots location
        BoostPolygon tmp1;
        BoostPolygon result1;
        boost::geometry::correct(tmp1);
        boost::geometry::assign(tmp1, poly1);
        boost::geometry::strategy::transform::matrix_transformer<double, 2, 2> xfrm1(
                 1, 0, this_pos[0],
                 0, 1, this_pos[1],
                          0,          0,  1);
        boost::geometry::transform(tmp1, result1, xfrm1);
        boost::geometry::correct(result1);

        auto exterior_points1 = boost::geometry::exterior_ring(result1);

        BoostPolygon poly2 = obstacle->getShape();
        boost::geometry::correct(poly2);

        // check if resulting polygons are in collision
        if (!boost::geometry::disjoint(result1, poly2))
            return false;
        else
            return true;
    }

    const std::string robot1_name_;
    const Robot* robot1_;
    const double rad1_;
    std::unordered_map<std::string, Robot*> robots_;
    std::set<Obstacle*> obstacles_;
};

class SimpleTwoValidityChecker: public ob::StateValidityChecker
{
public:
    SimpleTwoValidityChecker(const ob::SpaceInformationPtr &si, const Robot* robot1, const Robot* robot2,
        std::unordered_map<std::string, Robot*> robots, std::set<Obstacle*> obs_set = {}): 
        robot1_(robot1), rad1_(robot1_->getBoundingRadius()), robot2_(robot2), rad2_(robot2_->getBoundingRadius()), 
        robots_(robots), ob::StateValidityChecker(si), obstacles_(obs_set)
    {
    }

    // Answers the question: is the robot described by `si_` at `state` valid?
    bool isValid(const ompl::base::State *state) const override
    {
        // check if state is within bounds
        if (!si_->satisfiesBounds(state))
            return false;

        // check if the two robots collide with each other
        const double* this_pos = state->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(0)->values;
        const double* other_pos = state->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(2)->values;
        
        if (!performQuickCollisionCheck(this_pos, other_pos, rad2_))
        {
            if (!performExactCollisionCheck(this_pos, other_pos, robot2_))
                return false;
        }

        // iterate through both robots and check if both are valid against obstacles
        for (unsigned int idx = 0; idx < 2; idx++)
        {
            const double* this_pos = state->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(2 * idx)->values;
            for (auto o_itr = obstacles_.begin(); o_itr != obstacles_.end(); ++o_itr)
            {
                const double* other_pos = (*o_itr)->getCenterPoint();
                const double other_rad = (*o_itr)->getBoundingRadius();
                if (!performQuickCollisionCheck(this_pos, other_pos, other_rad))
                {
                    if (!performExactCollisionCheck(this_pos, other_pos, *o_itr))
                        return false;
                }
            }
        }
        return true;
    }

    // Answers the question: does the robot described by `si_` at `state1` avoid collision with some other robot described by a different `si` located at `state2`?
    bool areStatesValid(const ompl::base::State* state1, const std::pair<const ompl::base::SpaceInformationPtr,const ompl::base::State*> state2) const override
    {
        // get the correct other robots information
        const std::string other_robot_name = state2.first->getStateSpace()->getName();
        Robot* other_robot = robots_.at(other_robot_name);
        const double other_rad = other_robot->getBoundingRadius();
        const double* other_pos = state2.second->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(0)->values;

        // iterate through both robots and check if both are valid against other_robot
        for (unsigned int idx = 0; idx < 2; idx++)
        {
            const double* this_pos = state1->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(2 * idx)->values;
            if (!performQuickCollisionCheck(this_pos, other_pos, other_rad))
            {
                // collision possible, must check exact
                if (!performExactCollisionCheck(this_pos, other_pos, other_robot))
                    return false;
            }
        }
        return true;
    }
private:
    // returns true if no collision is possible (i.e. robots are too far away to collide)
    bool performQuickCollisionCheck(const double* this_pos, const double* other_pos, const double other_rad) const
    {
        const double distance = sqrt(pow(this_pos[0] - other_pos[0], 2) + pow(this_pos[1] - other_pos[1], 2));
        if (distance > (rad1_ + other_rad))
            return true;
        else
            return false;
    }

    // returns true if no collision
    bool performExactCollisionCheck(const double* this_pos, const double* other_pos, const Robot* other_robot) const
    {
        BoostPolygon poly1 = robot1_->getShape();
        // use boost transform to create polygon at true robots location
        BoostPolygon tmp1;
        BoostPolygon result1;
        boost::geometry::correct(tmp1);
        boost::geometry::assign(tmp1, poly1);
        boost::geometry::strategy::transform::matrix_transformer<double, 2, 2> xfrm1(
                1, 0, this_pos[0],
                0, 1, this_pos[1],
                0, 0, 1);
        boost::geometry::transform(tmp1, result1, xfrm1);
        boost::geometry::correct(result1);

        BoostPolygon poly2 = other_robot->getShape();
        // use boost transform to create polygon at true robots location
        BoostPolygon tmp2;
        BoostPolygon result2;
        boost::geometry::correct(tmp2);
        boost::geometry::assign(tmp2, poly2);
        boost::geometry::strategy::transform::matrix_transformer<double, 2, 2> xfrm2(
                1, 0, other_pos[0],
                0, 1, other_pos[1],
                0, 0, 1);
        boost::geometry::transform(tmp2, result2, xfrm2);
        boost::geometry::correct(result2);

        // check if resulting polygons are in collision
        if (!boost::geometry::disjoint(result1, result2))
            return false;
        else
            return true;
    }

    // returns true if no collision
    bool performExactCollisionCheck(const double* this_pos, const double* other_pos, const Obstacle* obstacle) const
    {
        BoostPolygon poly1 = robot1_->getShape();
        // use boost transform to create polygon at true robots location
        BoostPolygon tmp1;
        BoostPolygon result1;
        boost::geometry::correct(tmp1);
        boost::geometry::assign(tmp1, poly1);
        boost::geometry::strategy::transform::matrix_transformer<double, 2, 2> xfrm1(
                 1, 0, this_pos[0],
                 0, 1, this_pos[1],
                 0, 0, 1);
        boost::geometry::transform(tmp1, result1, xfrm1);
        boost::geometry::correct(result1);

        auto exterior_points1 = boost::geometry::exterior_ring(result1);

        BoostPolygon poly2 = obstacle->getShape();
        boost::geometry::correct(poly2);

        // check if resulting polygons are in collision
        if (!boost::geometry::disjoint(result1, poly2))
            return false;
        else
            return true;
    }
    const Robot* robot1_;
    const Robot* robot2_;
    const double rad1_;
    const double rad2_;
    std::unordered_map<std::string, Robot*> robots_;
    std::set<Obstacle*> obstacles_;
};
