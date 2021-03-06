#include <cmath>
#include <chrono>

#include <leatherman/print.h>
#include <leatherman/viz.h>

#include <ros/ros.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/PlanningScene.h>

#include <smpl/search/experience_graph_planner.h>
#include <smpl/collision_checker.h>
#include <smpl/graph/manip_lattice_egraph.h>
#include <smpl/graph/manip_lattice_action_space.h>
#include <smpl/heuristic/joint_dist_heuristic.h>
#include <smpl/heuristic/egraph_bfs_heuristic.h>
#include <smpl/heuristic/euclid_dist_heuristic.h>
#include <smpl/heuristic/generic_egraph_heuristic.h>
#include <smpl/occupancy_grid.h>
#include <smpl/robot_model.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <smpl/debug/visualizer_ros.h>

#include "utils.h"

namespace smpl = sbpl::motion;

/// \brief Defines a Robot Model for an (x, y) point robot
///
/// RobotModel base: basic requirements (variable types and limits)
///
/// ForwardKinematicsInterface: forward kinematics interface required by much
/// of smpl; trivial in this case to establish frame of reference
class KinematicVehicleModel :
    public virtual smpl::RobotModel,
    public virtual smpl::ForwardKinematicsInterface
{
public:

    KinematicVehicleModel() :
        smpl::RobotModel(),
        smpl::ForwardKinematicsInterface()
    {
        const std::vector<std::string> joint_names = { "x", "y" };
        setPlanningJoints(joint_names);
    }

    /// \name Required Public Functions from ForwardKinematicsInterface
    ///@{
    bool computeFK(
        const smpl::RobotState& state,
        const std::string& name,
        std::vector<double>& pose) override
    {
        return computePlanningLinkFK(state, pose);
    }

    bool computePlanningLinkFK(
        const smpl::RobotState& state,
        std::vector<double>& pose) override
    {
        pose = { state[0], state[1], 0.0, 0.0, 0.0, 0.0 };
        return true;
    }
    ///@}

    /// \name Required Public Functions from Robot Model
    ///@{
    double minPosLimit(int jidx) const override { return 0.0; }
    double maxPosLimit(int jidx) const override { return 0.0; }
    bool hasPosLimit(int jidx) const override { return false; }
    bool isContinuous(int jidx) const override { return false; }
    double velLimit(int jidx) const override { return 0.0; }
    double accLimit(int jidx) const override { return 0.0; }

    bool checkJointLimits(
        const smpl::RobotState& angles,
        bool verbose = false) override
    {
        return true;
    }
    ///@}

    /// \name Required Public Functions from Extension
    ///@{
    Extension* getExtension(size_t class_code) override
    {
        if (class_code == smpl::GetClassCode<RobotModel>() ||
            class_code == smpl::GetClassCode<ForwardKinematicsInterface>())
        {
            return this;
        } else {
            return nullptr;
        }
    }
    ///@}
};

/// \brief Defines a collision checker for an (x,y) point robot in a grid world.
class GridCollisionChecker : public smpl::CollisionChecker
{
public:

    GridCollisionChecker(sbpl::OccupancyGrid* grid) : m_grid(grid) { }

    bool isStateValid(
        const smpl::RobotState& state,
        bool verbose,
        bool visualize,
        double& dist) override;

    bool isStateToStateValid(
        const smpl::RobotState& start,
        const smpl::RobotState& finish,
        int& path_length,
        int& num_checks,
        double& dist) override;

    bool interpolatePath(
        const smpl::RobotState& start,
        const smpl::RobotState& finish,
        std::vector<smpl::RobotState>& path) override;

    visualization_msgs::MarkerArray getCollisionModelVisualization(
        const smpl::RobotState& state)
    { return visualization_msgs::MarkerArray(); }

    visualization_msgs::MarkerArray getVisualization(
        const smpl::RobotState& state)
    { return visualization_msgs::MarkerArray(); }

private:

    // a bit heavy-weight for this, since it overlays a distance transform
    sbpl::OccupancyGrid* m_grid;
};

bool GridCollisionChecker::isStateValid(
    const smpl::RobotState& state,
    bool verbose,
    bool visualize,
    double& dist)
{
    if (state.size() < 2) {
        ROS_ERROR("State contains insufficient data");
        return false;
    }
    double x = state[0];
    double y = state[1];
    double z = 0.0;
    if (!m_grid->isInBounds(x, y, z)) {
        ROS_DEBUG("state (%0.3f, %0.3f) is out of bounds", x, y);
        return false;
    }
    if (m_grid->getDistanceFromPoint(x, y, z) <= 0.0) {
        ROS_DEBUG("state (%0.3f, %0.3f) is occupied", x, y);
        return false;
    }
    return true;
}

bool GridCollisionChecker::isStateToStateValid(
    const smpl::RobotState& start,
    const smpl::RobotState& finish,
    int& path_length,
    int& num_checks,
    double& dist)
{
    std::vector<smpl::RobotState> path;
    if (!interpolatePath(start, finish, path)) {
        return false;
    }
    return std::all_of(
        path.begin(), path.end(),
        [&](const smpl::RobotState& state)
        {
            double dist;
            return isStateValid(state, false, false, dist);
        });
}

bool GridCollisionChecker::interpolatePath(
    const smpl::RobotState& start,
    const smpl::RobotState& finish,
    std::vector<smpl::RobotState>& path)
{
    m_grid->resolution();
    const Eigen::Vector2d vstart(start[0], start[1]);
    const Eigen::Vector2d vfinish(finish[0], finish[1]);
    int num_waypoints =
            std::ceil((vfinish - vstart).norm() / m_grid->resolution());
    num_waypoints = std::max(num_waypoints, 2);
    ROS_DEBUG("interpolate path with %d waypoints", num_waypoints);
    for (int i = 0; i < num_waypoints; ++i) {
        const double alpha = (double)i / (double)(num_waypoints - 1);
        Eigen::Vector2d vinterm = (1.0 - alpha) * vstart + alpha * vfinish;
        smpl::RobotState istate(2);
        istate[0] = vinterm.x();
        istate[1] = vinterm.y();
        path.push_back(std::move(istate));
    }
    return true;
}

/// Add a simple box obstacle in the center of the grid
void SetupOccupancyGrid(sbpl::OccupancyGrid& grid)
{
    const int x_count = grid.numCellsX();
    const int y_count = grid.numCellsY();
    const int z_count = grid.numCellsZ();

    std::vector<Eigen::Vector3d> points;

    // add horizontal strip down the middle, with holes on the ends
    for (int gx = 1; gx < x_count - 1; ++gx) {
        double cx, cy, cz;
        grid.gridToWorld(gx, y_count >> 1, 0, cx, cy, cz);
        points.emplace_back(cx, cy, cz);

        grid.gridToWorld(gx, (y_count >> 1) + 1, 0, cx, cy, cz);
        points.emplace_back(cx, cy, cz);

        grid.gridToWorld(gx, (y_count >> 1) - 1, 0, cx, cy, cz);
        points.emplace_back(cx, cy, cz);
    }

    ROS_INFO("Add %zu points to grid", points.size());
    grid.addPointsToField(points);
}

void PrintGrid(std::ostream& o, sbpl::OccupancyGrid& grid)
{
    for (int y = grid.numCellsY() - 1; y >= 0; --y) {
        for (int x = 0; x < grid.numCellsX(); ++x) {
            if (grid.getDistance(x, y, 0) <= 0) {
                o << "1 ";
            } else {
                o << "0 ";
            }
        }
        o << '\n';
    }
}

void PrintActionSpace(const smpl::ManipLatticeActionSpace& aspace)
{
    ROS_INFO("Action Set:");
    for (int i = 0; i < smpl::MotionPrimitive::Type::NUMBER_OF_MPRIM_TYPES; ++i) {
        smpl::MotionPrimitive::Type prim((smpl::MotionPrimitive::Type)i);
        ROS_INFO("  %s: %s @ %0.3f", to_string(prim).c_str(), aspace.useAmp(prim) ? "true" : "false", aspace.ampThresh(prim));
    }
    for (auto ait = aspace.begin(); ait != aspace.end(); ++ait) {
        ROS_INFO("  type: %s", to_string(ait->type).c_str());
        if (ait->type == sbpl::motion::MotionPrimitive::SNAP_TO_RPY) {
            ROS_INFO("    enabled: %s", aspace.useAmp(sbpl::motion::MotionPrimitive::SNAP_TO_RPY) ? "true" : "false");
            ROS_INFO("    thresh: %0.3f", aspace.ampThresh(sbpl::motion::MotionPrimitive::SNAP_TO_RPY));
        }
        else if (ait->type == sbpl::motion::MotionPrimitive::SNAP_TO_XYZ) {
            ROS_INFO("    enabled: %s", aspace.useAmp(sbpl::motion::MotionPrimitive::SNAP_TO_XYZ) ? "true" : "false");
            ROS_INFO("    thresh: %0.3f", aspace.ampThresh(sbpl::motion::MotionPrimitive::SNAP_TO_XYZ));
        }
        else if (ait->type == sbpl::motion::MotionPrimitive::SNAP_TO_XYZ_RPY) {
            ROS_INFO("    enabled: %s", aspace.useAmp(sbpl::motion::MotionPrimitive::SNAP_TO_XYZ_RPY) ? "true" : "false");
            ROS_INFO("    thresh: %0.3f", aspace.ampThresh(sbpl::motion::MotionPrimitive::SNAP_TO_XYZ_RPY));
        }
        else if (ait->type == sbpl::motion::MotionPrimitive::LONG_DISTANCE ||
                ait->type == sbpl::motion::MotionPrimitive::SHORT_DISTANCE)
        {
            ROS_INFO("    action: %s", to_string(ait->action).c_str());
        }
    }
}

visualization_msgs::Marker getPointVisualization(geometry_msgs::Point point_, std::string frame, std::string ns) {
    visualization_msgs::Marker point;
    point.header.frame_id = frame;
    point.header.stamp = ros::Time::now();
    point.ns = ns;
    point.action = visualization_msgs::Marker::ADD;
    point.pose.orientation.w = 1.0;
    point.id = 0;
    point.type = visualization_msgs::Marker::POINTS;
    point.scale.x = 0.2;
    point.scale.y = 0.2;
    point.scale.z = 0.2;
    point.color.g = 1.0;
    point.color.b = 1.0;
    point.color.a = 1;

    point.points.push_back(point_);

    //visualization_msgs::MarkerArray points;
    //points.markers = {point};

    return point;
}

visualization_msgs::Marker getLineVisualization(std::vector<geometry_msgs::Point> points, std::string frame, std::string ns) {
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = frame;
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = ns;
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = 1;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = 0.001;
    line_strip.color.g = 1.0;
    line_strip.color.a = 1;

    for(auto point : points)
        line_strip.points.push_back(point);

    return line_strip;
}

sbpl::OccupancyGrid setupOccupancyGrid(std::string planning_frame) {
    const double grid_res = 0.02;
    const double world_size_x = 1.0;
    const double world_size_y = 1.0;
    const double world_size_z = 1.5 * grid_res;
    const double world_origin_x = 0.0;
    const double world_origin_y = 0.0;
    const double world_origin_z = 0.0;
    const double max_distance_m = 0.2;
    const bool ref_count = false;
    sbpl::OccupancyGrid grid(
            world_size_x, world_size_y, world_size_z,
            grid_res,
            world_origin_x, world_origin_y, world_origin_z,
            max_distance_m,
            ref_count);
    grid.setReferenceFrame(planning_frame);
    SetupOccupancyGrid(grid);
    PrintGrid(std::cout, grid);

    return grid;
}

int main(int argc, char* argv[])
{
    std::string ns = "xytheta";
    ros::init(argc, argv, ns);
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    sbpl::VisualizerROS visualizer(nh, 100);
    sbpl::viz::set_visualizer(&visualizer);

    ros::Publisher ma_pub = nh.advertise<visualization_msgs::MarkerArray>(
            "visualization_markers", 100);
    ros::Publisher m_pub = nh.advertise<visualization_msgs::Marker>(
            "visualization_marker", 100);

    // let publishers set up
    ros::Duration(1.0).sleep();

    // everyone needs to know the name of the planning frame for reasons...
    std::string planning_frame;
    if (!ph.getParam("planning_frame", planning_frame)) {
        ROS_ERROR("Failed to retrieve param 'planning_frame' from the param server");
        return 1;
    }

    ROS_INFO("Planning Frame: %s", planning_frame.c_str());

    // 1. Instantiate Robot Model
    KinematicVehicleModel robot_model;

    // 2. Instantiate the Environment
    sbpl::OccupancyGrid grid = setupOccupancyGrid(planning_frame);

    // Visualize the obstacles and boundary.
    ma_pub.publish(grid.getOccupiedVoxelsVisualization());
    ma_pub.publish(grid.getBoundingBoxVisualization());

    GridCollisionChecker cc(&grid);

    // 3. Define Parameters
    smpl::PlanningParams params;

    // 4. Instantiate Planning Space
    auto pspace =
            std::make_shared<smpl::ManipLatticeEgraph>(&robot_model, &cc, &params);
    if (!pspace->init({ 0.02, 0.02 })) {
        ROS_ERROR("Failed to initialize Manip Lattice");
        return 1;
    }
    pspace->setVisualizationFrameId("map");

    std::string egraph_file;
    if (ph.getParam("egraph_file", egraph_file)) {
        ROS_INFO("Received egraph file path: %s. Loading...", egraph_file.c_str());
        if(!pspace->loadExperienceGraph(egraph_file))
            ROS_ERROR("Failed to load experience graph");
    }

    // 5. Instantiate and Initialize Motion Primitives
    std::string mprim_path;
    if (!ph.getParam("mprim_path", mprim_path)) {
        ROS_ERROR("Failed to retrieve 'mprim_path' from the param server");
        return 1;
    }

    auto aspace = std::make_shared<smpl::ManipLatticeActionSpace>(pspace.get());
    if (!aspace->load(mprim_path)) {
        return 1;
    }
    PrintActionSpace(*aspace);

    // 6. Associate Action Space with Planning Space
    pspace->setActionSpace(aspace);

    // 7. Instantiate Heuristic
    auto h = std::make_shared<smpl::JointDistHeuristic>(pspace, &grid);
    auto hEgraph = std::make_shared<smpl::GenericEgraphHeuristic>(pspace, &grid, h);

    // 8. Associate Heuristic with Planning Space (for adaptive motion
    // primitives)
    pspace->insertHeuristic(hEgraph.get());

    // 9. Instantiate and Initialize Search (associated with Planning Space)
    const bool forward = true;
    auto search = std::make_shared< smpl::ExperienceGraphPlanner >(pspace, hEgraph);

    const double epsilon = 5.0;
    search->set_initialsolution_eps(epsilon);
    search->set_search_mode(false);

    // 10. Set start state and goal condition in the Planning Space and
    // propagate state IDs to search
    double start_x = 0.6;
    double start_y = 0.33;
    //double start_z = 1.0;

    const smpl::RobotState start_state = { start_x, start_y};//, start_z };
    moveit_msgs::RobotState start_state_msg;
    start_state_msg.joint_state.name.push_back("x");
    start_state_msg.joint_state.name.push_back("y");
    start_state_msg.joint_state.position.push_back(start_x);
    start_state_msg.joint_state.position.push_back(start_y);

    double goal_x = 0.5;
    double goal_y = 0.66;
    //double goal_z = 1.0;
    const smpl::RobotState goal_state = { goal_x, goal_y};//, goal_z };

    smpl::GoalConstraint goal;
    goal.type = smpl::GoalType::JOINT_STATE_GOAL;
    goal.angles = goal_state;
    goal.angle_tolerances = { 0.02, 0.02 };

    if (!pspace->setGoal(goal)) {
        ROS_ERROR("Failed to set goal");
        return 1;
    }

    if (!pspace->setStart(start_state)) {
        ROS_ERROR("Failed to set start");
        return 1;
    }

    int start_id = pspace->getStartStateID();
    if (start_id < 0) {
        ROS_ERROR("Start state id is invalid");
        return 1;
    }

    int goal_id = pspace->getGoalStateID();
    if (goal_id < 0)  {
        ROS_ERROR("Goal state id is invalid");
        return 1;
    }

    if (search->set_start(start_id) == 0) {
        ROS_ERROR("Failed to set planner start state");
        return 1;
    }

    if (search->set_goal(goal_id) == 0) {
        ROS_ERROR("Failed to set planner goal state");
        return 1;
    }

    // 11. Plan a path
    ReplanParams search_params(10.0);
    search_params.initial_eps = epsilon;
    search_params.final_eps = 1.0;
    search_params.dec_eps = 0.2;
    search_params.return_first_solution = false;
    search_params.repair_time = 1.0;

    auto then = std::chrono::high_resolution_clock::now();
    std::vector<int> solution;
    int solcost;
    bool bret = search->replan(&solution, search_params, &solcost);
    if (!bret) {
        ROS_ERROR("Search failed to find a solution");
        return 1;
    }
    auto now = std::chrono::high_resolution_clock::now();
    const double elapsed = std::chrono::duration<double>(now - then).count();
    ROS_ERROR("Yipeee. got till here");

    // 12. Extract path from Planning Space
    std::vector<smpl::RobotState> path;
    if (!pspace->extractPath(solution, path)) {
        ROS_ERROR("Failed to extract path");
    }

    // Visualize the path.
    std::vector<geometry_msgs::Point> pathPoints;
    for(auto node : path) {
        geometry_msgs::Point point;
        point.x = node[0];
        point.y = node[1];
        pathPoints.push_back(point);
        //ROS_INFO("%f %f", point.x, point.y);
    }
    m_pub.publish(getLineVisualization(pathPoints, planning_frame, ns));
 
    // Save the path in a file.
    std::string plan_output_dir = "/home/aries/.ros/plans";

    moveit_msgs::RobotTrajectory rTraj;
    convertJointVariablePathToJointTrajectory(path, rTraj.joint_trajectory, &robot_model, planning_frame);
    ROS_ERROR("Size= %d", rTraj.joint_trajectory.points.size());
    moveit_msgs::RobotState trajectory_start;
    writePath(start_state_msg, rTraj, &robot_model, plan_output_dir);

    ROS_INFO("Path found!");
    ROS_INFO("  Planning Time: %0.3f", elapsed);
    ROS_INFO("  Expansion Count (total): %d", search->get_n_expands());
    ROS_INFO("  Expansion Count (initial): %d", search->get_n_expands_init_solution());
    ROS_INFO("  Solution (%zu)", solution.size());
    for (int id : solution) {
        ROS_INFO("    %d", id);
    }
    ROS_INFO("  Path (%zu)", path.size());
    for (const smpl::RobotState& point : path) {
        ROS_INFO("    (x: %0.3f, y: %0.3f)", point[0], point[1]);
    }

    return 0;
}
