#ifndef SBPL_LATTICE_PLANNER_H
#define SBPL_LATTICE_PLANNER_H

#include <iostream>
#include <vector>

using namespace std;

/** ROS **/
#include <ros/ros.h>

// Costmap used for the map representation
#include <costmap_2d/costmap_2d_ros.h>

// sbpl headers
#include <sbpl/headers.h>

//global representation
#include <nav_core/base_global_planner.h>

namespace sbpl_lattice_planner
{

class SBPLLatticePlanner : public nav_core::BaseGlobalPlanner
{
public:
  
  /**
   * @brief  Default constructor for the NavFnROS object
   */
  SBPLLatticePlanner();

  
  /**
   * @brief  Constructor for the SBPLLatticePlanner object
   * @param  name The name of this planner
   * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use
   */
  SBPLLatticePlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);


  /**
   * @brief  Initialization function for the SBPLLatticePlanner object
   * @param  name The name of this planner
   * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use
   */
  virtual void initialize(std::string name, 
                          costmap_2d::Costmap2DROS* costmap_ros);
  
  /**
   * @brief Given a goal pose in the world, compute a plan
   * @param start The start pose 
   * @param goal The goal pose 
   * @param plan The plan... filled by the planner
   * @return True if a valid plan was found, false otherwise
   */
  virtual bool makePlan(const geometry_msgs::PoseStamped& start, 
                        const geometry_msgs::PoseStamped& goal, 
                        std::vector<geometry_msgs::PoseStamped>& plan);

  virtual ~SBPLLatticePlanner(){}

private:
  unsigned char costMapCostToSBPLCost(unsigned char newcost);
  void publishStats(int solution_cost, int solution_size, 
                    const geometry_msgs::PoseStamped& start, 
                    const geometry_msgs::PoseStamped& goal);

  bool m_initialized;

  SBPLPlanner* m_planner;
  EnvironmentNAVXYTHETALAT* m_env;
  
  std::string m_planner_type; /**< sbpl method to use for planning.  choices are ARAPlanner and ADPlanner */

  double m_allocated_time; /**< amount of time allowed for search */
  double m_initial_epsilon; /**< initial epsilon for beginning the anytime search */

  std::string m_environment_type; /** what type of environment in which to plan.  choices are 2D and XYThetaLattice. */
  std::string m_cost_map_topic; /** what topic is being used for the costmap topic */

  bool m_forward_search; /** whether to use forward or backward search */
  std::string m_primitive_filename; /** where to find the motion primitives for the current robot */
  int m_force_scratch_limit; /** the number of cells that have to be changed in the costmap to force the planner to plan from scratch even if its an incremental planner */

  unsigned char m_lethal_obstacle;
  unsigned char m_inscribed_inflated_obstacle;
  unsigned char m_sbpl_cost_multiplier;


  costmap_2d::Costmap2DROS* m_costmap_ros; /**< manages the cost map for us */

  ros::Publisher m_plan_pub;
  ros::Publisher m_stats_publisher;
  
  std::vector<geometry_msgs::Point> m_footprint;

};

} //namespace sbpl_lattice_planner

#endif

