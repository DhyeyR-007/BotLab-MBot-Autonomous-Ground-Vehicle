#include <slam/mapping.hpp>
#include <utils/grid_utils.hpp>
#include <numeric>
#include <chrono>
#include <algorithm>
using namespace std::chrono;

Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
, initialized_(false)
{
}


void Mapping::updateMap(const mbot_lcm_msgs::lidar_t& scan,
                        const mbot_lcm_msgs::pose2D_t& pose,
                        OccupancyGrid& map)
{
    if (!initialized_){
        previousPose_ = pose;
        initialized_ = true;
    }

    MovingLaserScan movingScan(scan, previousPose_, pose);

    /// TODO(DONE): Update the map's log odds using the movingScan  
    //
    // Hint: Consider both the cells the laser hit and the cells it passed through.
    for(auto const& ray : movingScan){
        // std::cout << "ray range: " << ray.range << std::endl;
        scoreEndpoint(ray, map);
    }
    for(auto const& ray : movingScan){
        scoreRay(ray, map);
    }

    previousPose_ = pose;
}

void Mapping::scoreEndpoint(const adjusted_ray_t& ray, OccupancyGrid& map)
{
    /// TODO(DONE): Implement how to score the cell that the laser endpoint hits  
    if(ray.range > kMaxLaserDistance_){
        return;
    }
    Point<float> globalPosition(ray.origin.x + ray.range * std::cos(ray.theta), 
                                ray.origin.y + ray.range * std::sin(ray.theta));
    Point<int> point = global_position_to_grid_cell(globalPosition, map);
    
    CellOdds prevOdds = map.logOdds(point.x, point.y);
    map.setLogOdds(point.x, point.y, std::min(prevOdds + kHitOdds_, 127));

}

void Mapping::scoreRay(const adjusted_ray_t& ray, OccupancyGrid& map)
{
    /// TODO(DONE): Implement how to score the cells that the laser ray passes through  
    adjusted_ray_t rayInRange{ray.origin, 
                            std::min(ray.range, kMaxLaserDistance_), 
                            ray.theta};
    bresenham(rayInRange, map);
}

/*
Takes the ray and map, and returns a vector of map cells to check
*/
std::vector<Point<int>> Mapping::bresenham(const adjusted_ray_t& ray, OccupancyGrid& map)
{
    /// TODO(DONE): Implement the Bresenham's line algorithm to find cells touched by the ray.
    Point<float> laserEndpoint = global_position_to_grid_position(
        Point<float>(
            ray.origin.x + ray.range * std::cos(ray.theta),
            ray.origin.y + ray.range * std::sin(ray.theta)
            ), map);
    Point<int> startGridCell = global_position_to_grid_cell(ray.origin, map);
    Point<int> endGridCell;
    endGridCell.x = static_cast<int>(laserEndpoint.x);
    endGridCell.y = static_cast<int>(laserEndpoint.y);

    std::vector<Point<int>> rayCells;
    int dx = abs(endGridCell.x - startGridCell.x);
    int dy = abs(endGridCell.y - startGridCell.y);
    int sx = startGridCell.x < endGridCell.x ? 1 : -1;
    int sy = startGridCell.y < endGridCell.y ? 1 : -1;
    int err = dx - dy;
    int x = startGridCell.x;
    int y = startGridCell.y;
    int e2;
    while(x != endGridCell.x || y != endGridCell.y){
        rayCells.push_back(Point<int>(x, y));
        CellOdds prevOdds = map.logOdds(x, y);
        map.setLogOdds(x, y, std::max(-127, prevOdds - kMissOdds_));
        e2 = 2 * err;
        if (e2 >= -dy){
            err -= dy;
            x += sx;
        }
        if (e2 <= dx){
            err += dx;
            y += sy;
        }
    }

    return rayCells;
    
}

std::vector<Point<int>> Mapping::divideAndStepAlongRay(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    return {};
}
