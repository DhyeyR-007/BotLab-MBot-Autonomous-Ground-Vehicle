#include <utils/grid_utils.hpp>
#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/pose2D_t.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/geometric/angle_functions.hpp>
#include <cassert>

ParticleFilter::ParticleFilter(int numParticles)
    : kNumParticles_(numParticles),
      samplingAugmentation_(0.5, 0.9, numParticles),
      distribution_quality(1),
      quality_reinvigoration_percentage(0.1)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}

void ParticleFilter::initializeFilterAtPose(const mbot_lcm_msgs::pose2D_t &pose)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    float std_xy = 0.01;
    float std_theta = 0.01;
    std::normal_distribution<> dist_xy(0, std_xy);
    std::normal_distribution<> dist_theta(0, std_theta);
    for (int i = 0; i < kNumParticles_; ++i)
    {
        mbot_lcm_msgs::pose2D_t p;
        mbot_lcm_msgs::particle_t par;
        p.x = pose.x + dist_xy(numberGenerator_);
        p.y = pose.y + dist_xy(numberGenerator_);
        p.theta = wrap_to_pi(pose.theta + dist_theta(numberGenerator_));
        p.utime = 0;
        par.pose = p;
        par.weight = 1.0 / (double)kNumParticles_;
        par.parent_pose = pose;
        posterior_.push_back(par);
    }
    posterior_[posterior_.size() - 1].pose = pose;
}

void ParticleFilter::initializeFilterRandomly(const OccupancyGrid &map)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    RandomPoseSampler sampler(&map);
    for (int i = 0; i < kNumParticles_; ++i)
    {
        mbot_lcm_msgs::particle_t par = sampler.get_particle();
        par.weight = 1.0 / (double)kNumParticles_;
        posterior_.push_back(par);
    }
}

void ParticleFilter::resetOdometry(const mbot_lcm_msgs::pose2D_t &odometry)
{
    actionModel_.resetPrevious(odometry);
}

mbot_lcm_msgs::pose2D_t ParticleFilter::updateFilter(const mbot_lcm_msgs::pose2D_t &odometry,
                                                     const mbot_lcm_msgs::lidar_t &laser,
                                                     const OccupancyGrid &map)
{
    bool hasRobotMoved = actionModel_.updateAction(odometry);

    if (hasRobotMoved)
    {
        auto prior = resamplePosteriorDistribution(map);
        auto proposal = computeProposalDistribution(prior);

        posterior_ = computeNormalizedPosterior(proposal, laser, map);
        posteriorPose_ = estimatePosteriorPose(posterior_);
    }

    posteriorPose_.utime = odometry.utime;

    return posteriorPose_;
}

mbot_lcm_msgs::pose2D_t ParticleFilter::updateFilterActionOnly(const mbot_lcm_msgs::pose2D_t &odometry)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);

    if (hasRobotMoved)
    {
        auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(prior);
        posterior_ = proposal;
    }

    posteriorPose_ = odometry;

    return posteriorPose_;
}

mbot_lcm_msgs::pose2D_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}

mbot_lcm_msgs::particles_t ParticleFilter::particles(void) const
{
    mbot_lcm_msgs::particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}

ParticleList ParticleFilter::resamplePosteriorDistribution(const OccupancyGrid &map,
                                                           const bool keep_best,
                                                           const bool reinvigorate)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    ParticleList prior;
    std::uniform_real_distribution<double> rand_r(0, 1.0 / (double)kNumParticles_);
    int i = 0;
    double r = rand_r(numberGenerator_);
    double c = posterior_[0].weight;
    double u;
    for (int m = 0; m < kNumParticles_; ++m)
    {
        u = r + m * 1.0 / (double)kNumParticles_;
        while (u > c)
        {
            i++;
            c = c + posterior_[i].weight;
        }
        prior.push_back(posterior_[i]);
    }
    return prior;
}

ParticleList ParticleFilter::resamplePosteriorDistribution(const bool keep_best,
                                                           const bool reinvigorate)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    ParticleList prior;
    std::uniform_real_distribution<double> rand_r(0, 1.0 / (double)kNumParticles_);
    int i = 0;
    double r = rand_r(numberGenerator_);
    double c = posterior_[0].weight;
    double u;
    for (int m = 0; m < kNumParticles_; ++m)
    {
        u = r + m * 1.0 / (double)kNumParticles_;
        while (u > c)
        {
            i++;
            c = c + posterior_[i].weight;
        }
        prior.push_back(posterior_[i]);
    }
    return prior;

    // Placeholder
}

void ParticleFilter::reinvigoratePriorDistribution(ParticleList &prior)
{
    // Augmentation: if sensor model suspects an average particle quality of
    //      less than 15%, invigorate
    if (distribution_quality < 0.15) // TODO: make 0.15 a parameter
    {
        int count = 0;
        int max_count = floor(quality_reinvigoration_percentage * prior.size());

        std::random_device rd;
        std::default_random_engine generator(rd());
        auto ud01 = std::uniform_real_distribution<double>(0.0, 1.0);
        int step = std::max<int>(1, floor(ud01(generator) * prior.size() / max_count));

        for (int i = 0; i < max_count; i++)
        {
            prior[i * step] = randomPoseGen_.get_particle();
        }
    }

    // // Augmentation: randomize any unreasonable samples
    // if(map != nullptr)
    // {
    //     for (int i = 0; i < prior.size(); i++)
    //     {
    //         const auto& p = prior[i].pose;
    //         if(!map->isCellInGrid(p.x, p.y) ||
    //           (map->isCellInGrid(p.x, p.y) && (map->logOdds(p.x, p.y) >= 0)))
    //         {
    //             std::cout << "\tinvalid sample!!" << ", "
    //                 << !map->isCellInGrid(p.x, p.y) << ", "
    //                 << (map->isCellInGrid(p.x, p.y) && (map->logOdds(p.x, p.y) >= 0.0))
    //                 << std::endl;

    //             prior[i] = randomPoseGen_.get_particle();
    //         }
    //     }
    // }
}

ParticleList ParticleFilter::computeProposalDistribution(const ParticleList &prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    ParticleList proposal;
    for (auto par : prior)
    {
        proposal.push_back(actionModel_.applyAction(par));
    }
    return proposal;
}

ParticleList ParticleFilter::computeNormalizedPosterior(const ParticleList &proposal,
                                                        const mbot_lcm_msgs::lidar_t &laser,
                                                        const OccupancyGrid &map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the
    ///////////       particles in the proposal distribution
    ParticleList posterior;
    double w_sum = 0;
    double w_cur;
    for (auto pro : proposal)
    {
        mbot_lcm_msgs::particle_t new_par = pro;
        // std::cout <<"laser nums: " << laser.thetas.size() << std::endl;
        w_cur = sensorModel_.likelihood(new_par, laser, map);
        w_sum += w_cur;
        new_par.weight = w_cur;
        posterior.push_back(new_par);
    }
    for (auto &par : posterior)
    {
        par.weight /= w_sum;
    }
    // std::cout << "weight: " << w_sum << std::endl;
    // std::cout << "particle num: " << posterior.size() << std::endl;
    // std::cout << "posterior check "<< posterior[0].weight << "   "<<posterior[1].weight<<std::endl;
    return posterior;
}

mbot_lcm_msgs::pose2D_t ParticleFilter::estimatePosteriorPose(const ParticleList &posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    // Figure out which pose to take for the posterior pose
    // Weighted average is simple, but could be very bad
    // Maybe only take the best x% and then average.

    ParticleList sorted_particles = posterior;
    std::sort(sorted_particles.begin(), sorted_particles.end(),
              [](const mbot_lcm_msgs::particle_t &a, const mbot_lcm_msgs::particle_t &b)
              { return a.weight > b.weight; });

    size_t num_top_particles = sorted_particles.size() * 0.1;

    ParticleList top_particles(sorted_particles.begin(), sorted_particles.begin() + num_top_particles);

    mbot_lcm_msgs::pose2D_t pose = computeParticlesAverage(top_particles);
    return pose;
}

mbot_lcm_msgs::pose2D_t ParticleFilter::computeParticlesAverage(const ParticleList &particles_to_average)
{
    mbot_lcm_msgs::pose2D_t avg_pose;
    avg_pose.x = 0.0;
    avg_pose.y = 0.0;
    avg_pose.theta = 0.0;
    double sum_weight = 0.0;

    // Aux variables to compute theta average
    double theta_x = 0.0;
    double theta_y = 0.0;
    for (auto &&p : particles_to_average)
    {
        avg_pose.x += p.weight * p.pose.x;
        avg_pose.y += p.weight * p.pose.y;
        theta_x += p.weight * std::cos(p.pose.theta);
        theta_y += p.weight * std::sin(p.pose.theta);

        sum_weight += p.weight;
    }
    avg_pose.x /= sum_weight;
    avg_pose.y /= sum_weight;
    theta_x /= sum_weight;
    theta_y /= sum_weight;
    avg_pose.theta = std::atan2(theta_y, theta_x);

    return avg_pose;
}
