#include <planning/astar.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <queue>
#include <unordered_map>

using namespace std::chrono;

mbot_lcm_msgs::path2D_t search_for_path(mbot_lcm_msgs::pose2D_t start,
                                        mbot_lcm_msgs::pose2D_t goal,
                                        const ObstacleDistanceGrid &distances,
                                        const SearchParams &params)
{
    cell_t startCell = global_position_to_grid_cell(Point<double>(start.x, start.y), distances);
    cell_t goalCell = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances);
    bool found_path = false;

    ////////////////// TODO(DONE): Implement your A* search here //////////////////////////

    Node *startNode = new Node(startCell.x, startCell.y);
    Node *goalNode = new Node(goalCell.x, goalCell.y);

    auto compare = [](Node *a, Node *b)
    { return (a->g_cost + a->h_cost) > (b->g_cost + b->h_cost); };

    std::priority_queue<Node *, std::vector<Node *>, decltype(compare)> openSet(compare);
    std::unordered_map<int64_t, Node *> allNodes;

    startNode->g_cost = 0;
    startNode->h_cost = h_cost(startNode, goalNode, distances);
    openSet.push(startNode);
    allNodes[grid_cell_to_hash(startNode->cell)] = startNode;

    Node *currentNode = nullptr;
    bool found_path = false;

    while (!openSet.empty())
    {
        currentNode = openSet.top();
        openSet.pop();

        if (*currentNode == *goalNode)
        {
            found_path = true;
            break;
        }

        auto neighbors = expand_node(currentNode, distances, params);
        for (auto neighbor : neighbors)
        {
            double tentative_gCost = currentNode->g_cost + g_cost(currentNode, neighbor, distances, params);

            int64_t hash = grid_cell_to_hash(neighbor->cell);
            if (allNodes.find(hash) == allNodes.end() || tentative_gCost < neighbor->g_cost)
            {
                neighbor->g_cost = tentative_gCost;
                neighbor->h_cost = h_cost(neighbor, goalNode, distances);
                neighbor->parent = currentNode;

                if (allNodes.find(hash) == allNodes.end())
                {
                    openSet.push(neighbor);
                    allNodes[hash] = neighbor;
                }
            }
        }
    }

    mbot_lcm_msgs::path2D_t path;
    path.utime = start.utime;
    if (found_path)
    {
        auto nodePath = extract_node_path(currentNode, startNode);
        path.path = extract_pose_path(nodePath, distances);
        path.path.pop_back();
        path.path.push_back(goal);
    }
    else
    {
        std::cout << "[A*] Didn't find a path\n";
    }

    path.path_length = path.path.size();

    for (auto pair : allNodes)
    {
        delete pair.second;
    }

    return path;
}

int64_t grid_cell_to_hash(const cell_t &cell)
{
    // generate hash value for node
    const int prime = 10000019;
    int64_t hashValue = 1;
    hashValue = prime * hashValue + cell.x;
    hashValue = prime * hashValue + cell.y;
    return hashValue;
}

double h_cost(Node *from, Node *goal, const ObstacleDistanceGrid &distances)
{
    double h_cost = 0.0;
    ////////////////// TODO(DONE): Implement your heuristic //////////////////////////
    int dx = abs(from->cell.x - goal->cell.x);
    int dy = abs(from->cell.y - goal->cell.y);
    h_cost = (dx + dy) + (sqrt(2.0) - 2) * std::min(dx, dy);
    return h_cost;
}

double g_cost(Node *from, Node *goal, const ObstacleDistanceGrid &distances, const SearchParams &params)
{
    double g_cost = 0.0;
    ////////////////// TODO(DONE): Implement your goal cost, use obstacle distances //////////////////////////

    double dx = abs(goal->cell.x - from->cell.x);
    double dy = abs(goal->cell.y - from->cell.y);
    double distance = sqrt(dx * dx + dy * dy);
    // double distance = (dx + dy) + (sqrt(2.0) - 2) * std::min(dx, dy);

    double obstacleDistanceCost = 0.0;

    double obstacleDistance_goal = distances(goal->cell.x, goal->cell.y);
    if (obstacleDistance_goal < params.maxDistanceWithCost && obstacleDistance_goal > params.minDistanceToObstacle)
    {
        obstacleDistanceCost = pow(params.maxDistanceWithCost - obstacleDistance_goal, params.distanceCostExponent);
    }

    double obstacleDistance_from = distances(from->cell.x, from->cell.y);
    if (obstacleDistance_from < params.maxDistanceWithCost && obstacleDistance_from > params.minDistanceToObstacle)
    {
        obstacleDistanceCost += pow(params.maxDistanceWithCost - obstacleDistance_from, params.distanceCostExponent);
    }

    g_cost = distance + obstacleDistanceCost;

    // std::cout << "g cost: distance cost: " << distance << std::endl;
    // std::cout << "g cost: obstacle cost: " << obstacleDistanceCost << std::endl;
    // std::cout << "g cost: " << g_cost << std::endl;

    return g_cost;
}

std::vector<Node *> expand_node(Node *node, const ObstacleDistanceGrid &distances, const SearchParams &params)
{
    std::vector<Node *> children;
    ////////////////// TODO(DONE): Implement your expand node algorithm //////////////////////////
    const int xDeltas[8] = {1, -1, 0, 0, 1, -1, 1, -1};
    const int yDeltas[8] = {0, 0, 1, -1, 1, -1, -1, 1};
    for (int n = 0; n < 8; ++n)
    {
        cell_t adjacentCell(node->cell.x + xDeltas[n], node->cell.y + yDeltas[n]);
        if (distances.isCellInGrid(adjacentCell.x, adjacentCell.y))
        {
            auto distanceToObstacle = distances(adjacentCell.x, adjacentCell.y);
            if (distanceToObstacle > params.minDistanceToObstacle)
            {
                Node *new_node = new Node(adjacentCell.x, adjacentCell.y);
                children.push_back(new_node);
            }
        }
    }

    return children;
}

std::vector<Node *> extract_node_path(Node *goal_node, Node *start_node)
{
    std::vector<Node *> path;
    ////////////////// TODO(DONE): Implement your extract node function //////////////////////////
    // Traverse nodes and add parent nodes to the vector
    Node *cur = goal_node;
    while (cur)
    {
        path.push_back(cur);
        cur = cur->parent;
        if (cur == start_node)
        {
            path.push_back(start_node);
            break;
        }
    }
    // Reverse path
    std::reverse(path.begin(), path.end());
    return path;
}

int64_t get_current_utime()
{
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();

    return static_cast<int64_t>(microseconds);
}

// To prune the path for the waypoint follower
std::vector<mbot_lcm_msgs::pose2D_t> extract_pose_path(std::vector<Node *> nodes, const ObstacleDistanceGrid &distances)
{
    std::vector<mbot_lcm_msgs::pose2D_t> path;
    ////////////////// TODO(DONE): Implement your extract_pose_path function //////////////////////////
    // This should turn the node path into a vector of poses (with heading) in the global frame
    // You should prune the path to get a waypoint path suitable for sending to motion controller
    auto nodePath = prune_node_path(nodes);
    if (nodePath.empty())
    {
        std::cout << "node path is empty, error occurs!" << std::endl;
        return;
    }

    auto globalPose = grid_position_to_global_position(nodePath[0]->cell, distances);

    for (int i = 0; i < nodePath.size(); i++)
    {
        mbot_lcm_msgs::pose2D_t pose;
        pose.x = globalPose.x;
        pose.y = globalPose.y;
        pose.utime = get_current_utime();
        if (i < nodePath.size() - 1)
        {
            auto nextGlobalPos = grid_position_to_global_position(nodePath[i + 1]->cell, distances);
            pose.theta = atan2(nextGlobalPos.y - pose.y, nextGlobalPos.x - pose.x);
            globalPose = nextGlobalPos;
        }
        else
        {
            pose.theta = (i > 0) ? path.back().theta : 0;
        }
    }

    return path;
}

bool is_in_list(Node *node, std::vector<Node *> list)
{
    for (auto &&item : list)
    {
        if (*node == *item)
            return true;
    }
    return false;
}

Node *get_from_list(Node *node, std::vector<Node *> list)
{
    for (auto &&n : list)
    {
        if (*node == *n)
            return n;
    }
    return NULL;
}

std::vector<Node *> prune_node_path(std::vector<Node *> nodePath)
{
    std::vector<Node *> new_node_path;
    ////////////////// TODO(DONE): Optionally implement a prune_node_path function //////////////////////////
    // This should remove points in the path along the same line
    if (nodePath.size() < 3)
    {
        return nodePath;
    }
    new_node_path.push_back(nodePath[0]);
    for (int i = 1; i < nodePath.size() - 1; i++)
    {
        Node *prev = nodePath[i - 1];
        Node *curr = nodePath[i];
        Node *next = nodePath[i + 1];
        auto crossProduct = (curr->cell.y - prev->cell.y) * (next->cell.x - curr->cell.x) -
                            (next->cell.y - curr->cell.y) * (curr->cell.x - prev->cell.x);
        if (crossProduct > 1e-2)
        {
            new_node_path.push_back(curr);
        }
    }
    new_node_path.push_back(nodePath.back());

    return new_node_path;
}
