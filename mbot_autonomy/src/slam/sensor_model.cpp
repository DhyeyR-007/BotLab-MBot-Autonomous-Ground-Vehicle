#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/grid_utils.hpp>
#include <utils/geometric/point.hpp>
#include <cmath>
#include <iomanip>
SensorModel::SensorModel(void)
    : sigma_hit_(0.075),
      occupancy_threshold_(0),
      ray_stride_(7),
      max_ray_range_(1000),
      search_range(10),
      offset_quality_weight(5)
{
    initialize_bfs_offsets();
}

struct Offset {
    int x, y;
    double distance;

    Offset(int x, int y) : x(x), y(y), distance(std::hypot(x, y)) {}
};

void SensorModel::initialize_bfs_offsets()
{
    /// Initialize the BFS offsets based on the search range
    std::vector<Offset> temp_offsets;

    for (int y = -search_range; y <= search_range; ++y)
    {
        for (int x = -search_range; x <= search_range; ++x)
        {
            if (std::hypot(x, y) <= search_range)
            {
                temp_offsets.emplace_back(x, y);
            }
        }
    }

    // Sort based on distance
    std::sort(temp_offsets.begin(), temp_offsets.end(), [](const Offset& a, const Offset& b) {
        return a.distance < b.distance;
    });

    // Store the sorted offsets
    bfs_offsets_.clear();
    for (const auto& offset : temp_offsets)
    {
        bfs_offsets_.emplace_back(offset.x, offset.y);
    }
}

double SensorModel::likelihood(const mbot_lcm_msgs::particle_t &sample,
                               const mbot_lcm_msgs::lidar_t &scan,
                               const OccupancyGrid &map)
{
    /// TODO(DONE): Compute the likelihood of the given particle using the provided laser scan and map.
    double likelihood = 1.0;
    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose, ray_stride_);

    for (auto &ray : movingScan)
    {
        auto score = scoreRay(sample, ray, map);
        likelihood += score;
    }
    return likelihood; // Placeholder
}

double SensorModel::scoreRay(const mbot_lcm_msgs::particle_t &sample, const adjusted_ray_t &ray, const OccupancyGrid &map)
{
    /// TODO(DONE): Compute a score for a given ray based on its end point and the map.
    // Consider the offset from the nearest occupied cell.
    if (ray.range > max_ray_range_)
    {
        return 0.05;
    }

    Point<float> endPoint = getRayEndPointOnMap(sample, ray, map);

    Point<int> current = global_position_to_grid_cell(endPoint, map);

    Point<int> nearestGridPoint;
    bool flag = gridBFS(current, map, nearestGridPoint);

    double distance = 0.01;
    if (flag == true)
    {
        auto nearestGlobalPoint = grid_position_to_global_position(nearestGridPoint, map);
        distance = std::hypot(endPoint.x - nearestGlobalPoint.x, endPoint.y - nearestGlobalPoint.y);
    }

    // std::cout << "distance: " << distance << " score: " << NormalPdf(distance / offset_quality_weight) << std::endl;
    return NormalPdf(distance / offset_quality_weight);
}

double SensorModel::NormalPdf(const double &x)
{
    return (1.0 / (sqrt(2.0 * M_PI) * sigma_hit_)) * exp((-0.5 * x * x) / (sigma_hit_ * sigma_hit_));
}

bool SensorModel::gridBFS(const Point<int> current, const OccupancyGrid &map, Point<int> &neighbor)
{
    /// TODO(DONE): Use Breadth First Search to find the nearest occupied cell to the given end point.

    for (auto offset : bfs_offsets_)
    {
        neighbor = current + offset;

        if (map.isCellInGrid(neighbor.x, neighbor.y) && map.logOdds(neighbor.x, neighbor.y) > occupancy_threshold_)
        {
            return true;
        }
    }
    return false; // Placeholder
}

Point<float> SensorModel::getRayEndPointOnMap(const mbot_lcm_msgs::particle_t &sample, const adjusted_ray_t &ray, const OccupancyGrid &map)
{
    /// TODO(DONE): Calculate the end point of a given ray on the map
    /// TODO: Measure offsets
    float sensor_offset_x = 0.0;
    float sensor_offset_y = 0.0;
    float sensor_offset_theta = 0.0;

    float globalSensorX = sample.pose.x + sensor_offset_x * std::cos(sample.pose.theta) - sensor_offset_y * std::sin(sample.pose.theta);
    float globalSensorY = sample.pose.y + sensor_offset_x * std::sin(sample.pose.theta) + sensor_offset_y * std::cos(sample.pose.theta);
    float globalSensorTheta = sample.pose.theta + sensor_offset_theta;

    Point<float> globalEndPoint(globalSensorX + ray.range * std::cos(globalSensorTheta + ray.theta),
                                globalSensorY + ray.range * std::sin(globalSensorTheta + ray.theta));
    return globalEndPoint;
}
