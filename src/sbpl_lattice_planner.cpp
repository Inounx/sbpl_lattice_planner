/*********************************************************************
*
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
*
* Author: Mike Phillips
*********************************************************************/

#include <sbpl_lattice_planner/sbpl_lattice_planner.h>
#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Path.h>
#include <sbpl_lattice_planner/SBPLLatticePlannerStats.h>

#include <costmap_2d/inflation_layer.h>

using namespace std;
using namespace ros;

PLUGINLIB_EXPORT_CLASS(sbpl_lattice_planner::SBPLLatticePlanner, nav_core::BaseGlobalPlanner)

namespace sbpl_lattice_planner
{

class LatticeSCQ : public StateChangeQuery
{
public:
    LatticeSCQ(EnvironmentNAVXYTHETALAT* env, std::vector<nav2dcell_t> const & changedcellsV) :
        m_env(env),
        m_changedcellsV(changedcellsV)
    {
    }

    // lazy init, because we do not always end up calling this method
    virtual std::vector<int> const * getPredecessors() const
    {
        if(m_predsOfChangedCells.empty() && !m_changedcellsV.empty())
        {
            m_env->GetPredsofChangedEdges(&m_changedcellsV, &m_predsOfChangedCells);
        }
        return &m_predsOfChangedCells;
    }

    // lazy init, because we do not always end up calling this method
    virtual std::vector<int> const * getSuccessors() const
    {
        if(m_succsOfChangedCells.empty() && !m_changedcellsV.empty())
        {
            m_env->GetSuccsofChangedEdges(&m_changedcellsV, &m_succsOfChangedCells);
        }
        return &m_succsOfChangedCells;
    }

    EnvironmentNAVXYTHETALAT * m_env;
    std::vector<nav2dcell_t> const & m_changedcellsV;
    mutable std::vector<int> m_predsOfChangedCells;
    mutable std::vector<int> m_succsOfChangedCells;
};

//-----------------------------------------------------------------------------------
/**
 * @brief  Default constructor for the SBPLLatticePlanner object
 */
SBPLLatticePlanner::SBPLLatticePlanner() :
    m_initialized(false),
    m_costmap_ros(NULL)
{
}

//-----------------------------------------------------------------------------------
/**
 * @brief  Constructor for the SBPLLatticePlanner object
 * @param  name The name of this planner
 * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use
 */
SBPLLatticePlanner::SBPLLatticePlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) :
    m_initialized(false),
    m_costmap_ros(NULL)
{
    initialize(name, costmap_ros);
}

//-----------------------------------------------------------------------------------
/**
 * @brief  Initialization function for the SBPLLatticePlanner object
 * @param  name The name of this planner
 * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use
 */
void SBPLLatticePlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    if(!m_initialized)
    {
        ros::NodeHandle private_nh("~/"+name);
        ros::NodeHandle nh(name);

        ROS_INFO("Name is %s", name.c_str());

        private_nh.param("planner_type", m_planner_type, string("ARAPlanner"));
        private_nh.param("allocated_time", m_allocated_time, 10.0);
        private_nh.param("initial_epsilon", m_initial_epsilon, 3.0);
        private_nh.param("environment_type", m_environment_type, string("XYThetaLattice"));
        private_nh.param("forward_search", m_forward_search, bool(false));
        private_nh.param("primitive_filename", m_primitive_filename, string(""));
        private_nh.param("force_scratch_limit", m_force_scratch_limit, 500);

        double nominalvel_mpersecs, timetoturn45degsinplace_secs;
        private_nh.param("nominalvel_mpersecs", nominalvel_mpersecs, 0.4);
        private_nh.param("timetoturn45degsinplace_secs", timetoturn45degsinplace_secs, 0.6);

        int lethal_obstacle;
        private_nh.param("lethal_obstacle",lethal_obstacle,20);
        m_lethal_obstacle = (unsigned char) lethal_obstacle;
        m_inscribed_inflated_obstacle = m_lethal_obstacle-1;
        m_sbpl_cost_multiplier = (unsigned char) (costmap_2d::INSCRIBED_INFLATED_OBSTACLE / m_inscribed_inflated_obstacle + 1);
        ROS_DEBUG("SBPL: lethal: %uz, inscribed inflated: %uz, multiplier: %uz", lethal_obstacle, m_inscribed_inflated_obstacle, m_sbpl_cost_multiplier);

        m_costmap_ros = costmap_ros;

        std::vector<geometry_msgs::Point> footprint = m_costmap_ros->getRobotFootprint();

        if("XYThetaLattice" == m_environment_type)
        {
            ROS_DEBUG("Using a 3D costmap for theta lattice\n");
            m_env = new EnvironmentNAVXYTHETALAT();
        }
        else
        {
            ROS_ERROR("XYThetaLattice is currently the only supported environment!\n");
            exit(1);
        }

        // check if the costmap has an inflation layer
        // Warning: footprint updates after initialization are not supported here
        unsigned char cost_possibly_circumscribed_tresh = 0;
        for(std::vector<boost::shared_ptr<costmap_2d::Layer> >::const_iterator layer = m_costmap_ros->getLayeredCostmap()->getPlugins()->begin();
            layer != m_costmap_ros->getLayeredCostmap()->getPlugins()->end();
            ++layer)
        {
            boost::shared_ptr<costmap_2d::InflationLayer> inflation_layer = boost::dynamic_pointer_cast<costmap_2d::InflationLayer>(*layer);
            if (!inflation_layer) continue;

            cost_possibly_circumscribed_tresh = inflation_layer->computeCost(m_costmap_ros->getLayeredCostmap()->getCircumscribedRadius() / m_costmap_ros->getCostmap()->getResolution());
        }

        if(!m_env->SetEnvParameter("cost_inscribed_thresh", costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE)))
        {
            ROS_ERROR("Failed to set cost_inscribed_thresh parameter");
            exit(1);
        }

        if(!m_env->SetEnvParameter("cost_possibly_circumscribed_thresh", costMapCostToSBPLCost(cost_possibly_circumscribed_tresh)))
        {
            ROS_ERROR("Failed to set cost_possibly_circumscribed_thresh parameter");
            exit(1);
        }

        int obst_cost_thresh = costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE);
        vector<sbpl_2Dpt_t> perimeterptsV;
        perimeterptsV.reserve(footprint.size());
        for (size_t ii(0); ii < footprint.size(); ++ii)
        {
            sbpl_2Dpt_t pt;
            pt.x = footprint[ii].x;
            pt.y = footprint[ii].y;
            perimeterptsV.push_back(pt);
        }

        bool ret;
        try
        {
            ret = m_env->InitializeEnv(m_costmap_ros->getCostmap()->getSizeInCellsX(), // width
                                       m_costmap_ros->getCostmap()->getSizeInCellsY(), // height
                                       0, // mapdata
                                       0, 0, 0, // start (x, y, theta, t)
                                       0, 0, 0, // goal (x, y, theta)
                                       0, 0, 0, //goal tolerance
                                       perimeterptsV, m_costmap_ros->getCostmap()->getResolution(), nominalvel_mpersecs,
                                       timetoturn45degsinplace_secs, obst_cost_thresh,
                                       m_primitive_filename.c_str());
        }
        catch(SBPL_Exception e)
        {
            ROS_ERROR("SBPL encountered a fatal exception!");
            ret = false;
        }

        if(!ret)
        {
            ROS_ERROR("SBPL initialization failed!");
            exit(1);
        }

        for (ssize_t ix(0); ix < m_costmap_ros->getCostmap()->getSizeInCellsX(); ++ix)
        {
            for (ssize_t iy(0); iy < m_costmap_ros->getCostmap()->getSizeInCellsY(); ++iy)
            {
                m_env->UpdateCost(ix, iy, costMapCostToSBPLCost(m_costmap_ros->getCostmap()->getCost(ix,iy)));
            }
        }

        if ("ARAPlanner" == m_planner_type)
        {
            ROS_INFO("Planning with ARA*");
            m_planner = new ARAPlanner(m_env, m_forward_search);
        }
        else if ("ADPlanner" == m_planner_type)
        {
            ROS_INFO("Planning with AD*");
            m_planner = new ADPlanner(m_env, m_forward_search);
        }
        else
        {
            ROS_ERROR("ARAPlanner and ADPlanner are currently the only supported planners!\n");
            exit(1);
        }

        ROS_INFO("[sbpl_lattice_planner] Initialized successfully");
        m_plan_pub = private_nh.advertise<nav_msgs::Path>("plan", 1);
        m_stats_publisher = private_nh.advertise<sbpl_lattice_planner::SBPLLatticePlannerStats>("sbpl_lattice_planner_stats", 1);

        m_initialized = true;
    }
}

//-----------------------------------------------------------------------------------
/*! \brief This rescales the costmap according to a rosparam which sets the obstacle cost
 *  \note Taken from Sachin's sbpl_cart_planner
 */
unsigned char SBPLLatticePlanner::costMapCostToSBPLCost(unsigned char newcost)
{
    if(newcost == costmap_2d::LETHAL_OBSTACLE)
    {
        return m_lethal_obstacle;
    }
    else if(newcost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    {
        return m_inscribed_inflated_obstacle;
    }
    else if(newcost == 0 || newcost == costmap_2d::NO_INFORMATION)
    {
        return 0;
    }
    else
    {
        return (unsigned char) (newcost/m_sbpl_cost_multiplier + 0.5);
    }
}

//-----------------------------------------------------------------------------------
/*! \brief
 */
void SBPLLatticePlanner::publishStats(int solution_cost, int solution_size,
                                      const geometry_msgs::PoseStamped& start,
                                      const geometry_msgs::PoseStamped& goal)
{
    // Fill up statistics and publish
    sbpl_lattice_planner::SBPLLatticePlannerStats stats;
    stats.initial_epsilon = m_initial_epsilon;
    stats.plan_to_first_solution = false;
    stats.final_number_of_expands = m_planner->get_n_expands();
    stats.allocated_time = m_allocated_time;

    stats.time_to_first_solution = m_planner->get_initial_eps_planning_time();
    stats.actual_time = m_planner->get_final_eps_planning_time();
    stats.number_of_expands_initial_solution = m_planner->get_n_expands_init_solution();
    stats.final_epsilon = m_planner->get_final_epsilon();

    stats.solution_cost = solution_cost;
    stats.path_size = solution_size;
    stats.start = start;
    stats.goal = goal;
    m_stats_publisher.publish(stats);
}

//-----------------------------------------------------------------------------------
/**
 * @brief Given a goal pose in the world, compute a plan
 * @param start The start pose
 * @param goal The goal pose
 * @param plan The plan... filled by the planner
 * @return True if a valid plan was found, false otherwise
 */
bool SBPLLatticePlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                 const geometry_msgs::PoseStamped& goal,
                                 std::vector<geometry_msgs::PoseStamped>& plan)
{
    if(!m_initialized)
    {
        ROS_ERROR("Global planner is not initialized");
        return false;
    }

    plan.clear();

    ROS_INFO("[sbpl_lattice_planner] getting start point (%g,%g) goal point (%g,%g)",
             start.pose.position.x, start.pose.position.y,goal.pose.position.x, goal.pose.position.y);

    double theta_start = 2 * atan2(start.pose.orientation.z, start.pose.orientation.w);
    double theta_goal = 2 * atan2(goal.pose.orientation.z, goal.pose.orientation.w);

    try
    {
        int ret = m_env->SetStart(start.pose.position.x - m_costmap_ros->getCostmap()->getOriginX(), start.pose.position.y - m_costmap_ros->getCostmap()->getOriginY(), theta_start);
        if(ret < 0 || m_planner->set_start(ret) == 0)
        {
            ROS_ERROR("ERROR: failed to set start state\n");
            return false;
        }
    }
    catch(SBPL_Exception e)
    {
        ROS_ERROR("SBPL encountered a fatal exception while setting the start state");
        return false;
    }

    try
    {
        int ret = m_env->SetGoal(goal.pose.position.x - m_costmap_ros->getCostmap()->getOriginX(), goal.pose.position.y - m_costmap_ros->getCostmap()->getOriginY(), theta_goal);
        if(ret < 0 || m_planner->set_goal(ret) == 0)
        {
            ROS_ERROR("ERROR: failed to set goal state\n");
            return false;
        }
    }
    catch(SBPL_Exception e)
    {
        ROS_ERROR("SBPL encountered a fatal exception while setting the goal state");
        return false;
    }

    int offOnCount = 0;
    int onOffCount = 0;
    int allCount = 0;
    vector<nav2dcell_t> changedcellsV;

    for(unsigned int ix = 0; ix < m_costmap_ros->getCostmap()->getSizeInCellsX(); ix++)
    {
        for(unsigned int iy = 0; iy < m_costmap_ros->getCostmap()->getSizeInCellsY(); iy++)
        {
            unsigned char oldCost = m_env->GetMapCost(ix,iy);
            unsigned char newCost = costMapCostToSBPLCost(m_costmap_ros->getCostmap()->getCost(ix,iy));

            if(oldCost == newCost) continue;

            allCount++;

            //first case - off cell goes on

            if((oldCost != costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE) && oldCost != costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE)) &&
              (newCost == costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE) || newCost == costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE)))
            {
                offOnCount++;
            }

            if((oldCost == costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE) || oldCost == costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE)) &&
              (newCost != costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE) && newCost != costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE)))
            {
                onOffCount++;
            }
            m_env->UpdateCost(ix, iy, costMapCostToSBPLCost(m_costmap_ros->getCostmap()->getCost(ix,iy)));

            nav2dcell_t nav2dcell;
            nav2dcell.x = ix;
            nav2dcell.y = iy;
            changedcellsV.push_back(nav2dcell);
        }
    }

    try
    {
        if(!changedcellsV.empty())
        {
            StateChangeQuery* scq = new LatticeSCQ(m_env, changedcellsV);
            m_planner->costs_changed(*scq);
            delete scq;
        }

        if(allCount > m_force_scratch_limit)
        {
            m_planner->force_planning_from_scratch();
        }
    }
    catch(SBPL_Exception e)
    {
        ROS_ERROR("SBPL failed to update the costmap");
        return false;
    }

    //setting planner parameters
    ROS_DEBUG("allocated:%f, init eps:%f\n",m_allocated_time,m_initial_epsilon);
    m_planner->set_initialsolution_eps(m_initial_epsilon);
    m_planner->set_search_mode(false);

    ROS_DEBUG("[sbpl_lattice_planner] run planner");
    vector<int> solution_stateIDs;
    int solution_cost;

    try
    {
        int ret = m_planner->replan(m_allocated_time, &solution_stateIDs, &solution_cost);
        if(ret)
        {
            ROS_DEBUG("Solution is found\n");
        }
        else
        {
            ROS_INFO("Solution not found\n");
            publishStats(solution_cost, 0, start, goal);
            return false;
        }
    }
    catch(SBPL_Exception e)
    {
        ROS_ERROR("SBPL encountered a fatal exception while planning");
        return false;
    }

    ROS_DEBUG("size of solution=%d", (int)solution_stateIDs.size());

    vector<EnvNAVXYTHETALAT3Dpt_t> sbpl_path;
    try
    {
        m_env->ConvertStateIDPathintoXYThetaPath(&solution_stateIDs, &sbpl_path);
    }
    catch(SBPL_Exception e)
    {
        ROS_ERROR("SBPL encountered a fatal exception while reconstructing the path");
        return false;
    }
    ROS_DEBUG("Plan has %d points.\n", (int)sbpl_path.size());
    ros::Time plan_time = ros::Time::now();

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(sbpl_path.size());
    gui_path.header.frame_id = m_costmap_ros->getGlobalFrameID();
    gui_path.header.stamp = plan_time;
    for(unsigned int i=0; i<sbpl_path.size(); i++)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = plan_time;
        pose.header.frame_id = m_costmap_ros->getGlobalFrameID();

        pose.pose.position.x = sbpl_path[i].x + m_costmap_ros->getCostmap()->getOriginX();
        pose.pose.position.y = sbpl_path[i].y + m_costmap_ros->getCostmap()->getOriginY();
        pose.pose.position.z = start.pose.position.z;

        tf::Quaternion temp;
        temp.setRPY(0,0,sbpl_path[i].theta);
        pose.pose.orientation.x = temp.getX();
        pose.pose.orientation.y = temp.getY();
        pose.pose.orientation.z = temp.getZ();
        pose.pose.orientation.w = temp.getW();

        plan.push_back(pose);

        gui_path.poses[i] = plan[i];
    }
    m_plan_pub.publish(gui_path);
    publishStats(solution_cost, sbpl_path.size(), start, goal);

    return true;
}

}
