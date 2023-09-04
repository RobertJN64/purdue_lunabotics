#include <lunabot_control/mpc.h>

MPC::MPC(ros::NodeHandle *nh) {
    double frequency;
    std::string map_topic, path_topic, odom_topic, cmd_vel_topic;
    // Global params
    ros::param::get("/odom_topic", odom_topic);
    ros::param::get("/cmd_vel_topic", cmd_vel_topic);

    // Nav params
    ros::param::get("map_topic", map_topic);
    ros::param::get("global_path_topic", path_topic);

    // MPC params
    ros::param::get("~rollout_count", this->rollout_count_); //Number of possible velocities to create
    ros::param::get("~top_rollouts", this->top_rollouts_); //How many of the top velocities to choose
    ros::param::get("~iterations", this->iterations_); //How many times to repeat the process of generating/choosing velocities

    //Weights for cost function
    ros::param::get("~w_linear", this->w_linear_);
    ros::param::get("~w_angular", this->w_angular_);
    ros::param::get("~w_waypoint", this->w_waypoint_);
    ros::param::get("~w_occupied", this->w_occupied_);

    ros::param::get("~horizon_length", this->horizon_length_); //How many steps in the future to use to calculate costs
    ros::param::get("~frequency", frequency);
    ros::param::get("~min_distance_threshold", this->min_dist_thres_); // Minimum distance needed to be considered 'in' a point
    ros::param::get("~velocity_limits/linear", lin_lim_); //linear velocity limit
    ros::param::get("~velocity_limits/angular", ang_lim_); //angular velocity limit

    // MPC Variables
    this->delta_time_ = 1 / frequency;
    this->path_ind_ = 0;
    this->enabled_ = false;

    // Ros publishers and subscribers
    this->velocity_pub_ =
        nh->advertise<geometry_msgs::Twist>(cmd_vel_topic, 10);
    this->grid_sub_ = nh->subscribe(map_topic, 10, &MPC::update_grid, this);
    this->path_sub_ = nh->subscribe(path_topic, 10, &MPC::update_path,
                                    this); // Convert to Map
    this->robot_pos_sub_ = nh->subscribe(odom_topic, 10, &MPC::update_robot_pos,
                                         this); // Convert to Map
}

//Check if the map is currently occupied at the given position
bool MPC::check_collision_(Eigen::RowVectorXd pos) {
    std::vector<double> state = {pos(0), pos(1)};

    return this->map_.occupied_at_pos(state);
}

//Finds the closest distance between the given position and the path
double MPC::find_closest_distance_(Eigen::RowVectorXd pos) {
    std::vector<std::vector<double>> path = this->path_;

    if (path.empty()) {
        return -1;
    }

    double min_dist = -1;

    std::vector<double> point = path[0];
    for (int i = 1; i < path.size(); ++i) {
        std::vector<double> new_point = path[i];

        // Building Parametric Lines
        // (Find the distance (velocity) lines between two consecutive points)
        double x_vel = new_point[0] - point[0];
        double y_vel = new_point[1] - point[1];

        if (x_vel == 0 && y_vel == 0) {
            return 0;
        }

        double t =
            (x_vel * (pos(0) - point[0]) + y_vel * (pos(1) - point[1])) /
            (x_vel * x_vel +
             y_vel * y_vel); // Finding time where line is closest to point

        // Scaling to be on the line segment (Clamping)
        if (t < 0) {
            t = 0;
        } else if (t > 1) {
            t = 1;
        }

        // Find the closest x/y values based on the calculated time value (parametric)
        double x_closest = point[0] + t * x_vel;
        double y_closest = point[1] + t * y_vel;

        double distance =
            std::sqrt((x_closest - pos(0)) * (x_closest - pos(0)) +
                      (y_closest - pos[1]) * (y_closest - pos(1)));

        if (min_dist == -1 || distance < min_dist) {
            min_dist = distance;
        } //Find minimum

        point = new_point; //move to the next point
    }
    return min_dist;
}

//Tells whether the robot distance to next point on the path is less than the distance threshold
int MPC::is_close_() {
    return dist_to_setpoint_() <= min_dist_thres_ * min_dist_thres_;
}

//When the robot gets close to the next point on the path, increase the path index
//(keep going down the path) or if at the end, stop its movement.
void MPC::update_setpoint_() {
    if (is_close_()) {
        if (this->path_ind_ == this->path_.size() - 1) {
            publish_velocity_(0, 0);
            this->path_ind_ = 0;
            this->enabled_ = false;
        } else {
            this->path_ind_++;
        }
    }
}

//Called when a new grid (map) is received
void MPC::update_grid(const nav_msgs::OccupancyGrid &grid) {
    map_.update_map(grid);
}

//Called when a new path is received (resets path index to 0)
void MPC::update_path(const nav_msgs::Path &path) {
    ROS_INFO("PATH");
    this->path_.clear();
    this->path_ind_ = 0;
    for (int i = 0; i < path.poses.size(); ++i) {
        geometry_msgs::Point position = path.poses[i].pose.position;
        std::vector<double> pos;
        pos.push_back(position.x);
        pos.push_back(position.y);
        this->path_.push_back(pos);
    }

    this->enabled_ = true;
}

//Get the yaw out of a quaternion
static double get_yaw_(geometry_msgs::Quaternion q) {
    return std::atan2(2 * (q.z * q.w + q.x * q.y),
                      1 - 2 * (q.z * q.z + q.y * q.y));
}

//Called when a new robot position is received 
void MPC::update_robot_pos(const nav_msgs::Odometry &odometry) {
    geometry_msgs::Pose pose = odometry.pose.pose;
    this->robot_pos_.clear();
    this->robot_pos_.push_back(pose.position.x);
    this->robot_pos_.push_back(pose.position.y);
    this->robot_pos_.push_back(get_yaw_(pose.orientation));
}

double MPC::clamp_(double val, double low, double high) {
    if (val < low) {
        return low;
    } else if (val > high) {
        return high;
    } else {
        return val;
    }
}

//Publish the calculated velocity as a Twist (Ros Message)
void MPC::publish_velocity_(double linear, double angular) {
    geometry_msgs::Twist twist;
    // NOTE: Swap linear and angular velocity in simulation
    twist.linear.x = linear;
    twist.angular.z = angular;
    this->velocity_pub_.publish(twist);
}

//Returns a vector of matrices, each matrix is a generated list of velocities
//Generated based on the given means and standard deviations (matrices)
std::vector<Eigen::MatrixXd> MPC::normal_distribute_(Eigen::MatrixXd means,
                                                     Eigen::MatrixXd std_devs,
                                                     int count) {
    std::vector<Eigen::MatrixXd> random_vels(count);
    for (int i = 0; i < count; ++i) { //For each matrix, Generate normally distributed values
        Eigen::MatrixXd random_vel(means.rows(), means.cols());

        for (int j = 0; j < means.rows(); ++j) { 
            for (int k = 0; k < means.cols(); ++k) {
                std::random_device rd;
                std::default_random_engine generator(rd());
                std::normal_distribution<double> distribution(means(j, k),
                                                              std_devs(j, k));
                random_vel(j, k) = distribution(generator);
            }
        }
        random_vels[i] = random_vel;
    }
    return random_vels;
}

//Returns a matrix of means calculated based on the top samples (rollouts)
Eigen::MatrixXd
MPC::mean_(std::vector<std::pair<Eigen::MatrixXd, double>> rollouts) {
    Eigen::MatrixXd mean = Eigen::MatrixXd::Zero(this->horizon_length_, 2);
    for (int i = 0; i < this->top_rollouts_; ++i) {
        for (int j = 0; j < rollouts[i].first.rows(); ++j) {
            mean(j, 0) += rollouts[i].first(j, 3);
            mean(j, 1) += rollouts[i].first(j, 4);
        }
    }
    return mean / this->top_rollouts_;
}

//Returns a matrix of standard deviations based on the top samples
Eigen::MatrixXd
MPC::std_dev_(std::vector<std::pair<Eigen::MatrixXd, double>> rollouts) {
    Eigen::MatrixXd temp_mean = mean_(rollouts);
    Eigen::MatrixXd std_dev = Eigen::MatrixXd::Zero(this->horizon_length_, 2);
    for (int i = 0; i < this->top_rollouts_; ++i) {
        for (int j = 0; j < rollouts[i].first.rows(); ++j) {
            std_dev(j, 0) += (rollouts[i].first(j, 3) - temp_mean(j, 0)) *
                             (rollouts[i].first(j, 3) - temp_mean(j, 0));
            std_dev(j, 1) += (rollouts[i].first(j, 4) - temp_mean(j, 1)) *
                             (rollouts[i].first(j, 4) - temp_mean(j, 1));
        }
    }

    std_dev /= this->top_rollouts_ - 1;
    for (int i = 0; i < std_dev.rows(); ++i) {
        for (int j = 0; j < std_dev.cols(); ++j) {
            std_dev(i, j) = std::sqrt(std_dev(i, j));
        }
    }
    return std_dev;
}

//Returns the distance between the current robot position and a point on the path (by current path index)
double MPC::dist_to_setpoint_() {
    return ((robot_pos_[0] - path_[path_ind_][0]) *
                (robot_pos_[0] - path_[path_ind_][0]) +
            (robot_pos_[1] - path_[path_ind_][1]) *
                (robot_pos_[1] - path_[path_ind_][1]));
}

//Calculate the cost for a given sample/rollout of velocities
double MPC::calculate_cost_(Eigen::MatrixXd rollout) {
    SWRI_PROFILE("calculate-cost"); 
    double cost = 0;
    std::vector<double> robot_pos = this->robot_pos_;
    std::vector<double> goal = this->goal_;
    for (int i = 0; i < rollout.rows(); ++i) { //For each sample calculate based on:
        Eigen::RowVectorXd position = rollout.row(i);

        cost += this->w_linear_ * 
                ((position(0) - robot_pos[0]) * (position(0) - robot_pos[0]) +
                 (position(1) - robot_pos[1]) * (position(1) - robot_pos[1])); // Linear distance between the robot and modeled position

        cost += this->w_angular_ * (position(2) - robot_pos[2]) * // angular error between robot and modeled position
                (position(2) - robot_pos[2]);

        int path_cost_i = std::min(path_ind_ + i, (int)path_.size() - 1);

        cost += this->w_waypoint_ * ((position(0) - path_[path_cost_i][0]) *
                                         (position(0) - path_[path_cost_i][0]) +
                                     (position(1) - path_[path_cost_i][1]) *
                                         (position(1) - path_[path_cost_i][1])); //Distance from the next point on the path and modeled position

        cost += this->w_occupied_ * check_collision_(position); //Whether the position is occupied
    }

    return cost;
}

//Motion model used to calculate how the robot would move given the velocities of a sample
//Returns a matrix of the x, y, theta, v, omega values for each timestep 
Eigen::MatrixXd MPC::calculate_model_(Eigen::MatrixXd actions) {
    SWRI_PROFILE("calculate-model");
    Eigen::VectorXd current_pos(3); // 3 x 1 mat
    current_pos << this->robot_pos_[0], this->robot_pos_[1],
        this->robot_pos_[2];
    Eigen::MatrixXd model(this->horizon_length_, 5); // x, y, theta, v, omega

    for (int i = 0; i < this->horizon_length_; ++i) {
        model(i, 0) = current_pos(0);
        model(i, 1) = current_pos(1);
        model(i, 2) = current_pos(2);
        model(i, 3) = actions(i, 0);
        model(i, 4) = actions(i, 1);
        current_pos(0) +=
            std::cos(current_pos(2)) * this->delta_time_ * actions(i, 0);
        current_pos(1) +=
            std::sin(current_pos(2)) * this->delta_time_ * actions(i, 0);
        current_pos(2) += this->delta_time_ * actions(i, 1);
    }

    return model;
}

//Compares the scores (cost function) of two matrices
bool comparator(std::pair<Eigen::MatrixXd, double> a,
                std::pair<Eigen::MatrixXd, double> b) {
    return a.second < b.second;
}

//Main function to calculate velocities for MPC
void MPC::calculate_velocity() {
    SWRI_PROFILE("calculate-velocity");
    if (this->robot_pos_.empty() || this->path_.empty() || !this->enabled_) {
        return;
    }
    int horizon_length = this->horizon_length_;

    //Create initial matrices for means and StdDevs
    Eigen::MatrixXd means =
        Eigen::MatrixXd::Zero(horizon_length, 2); // v, omega
    Eigen::MatrixXd std_devs =
        Eigen::MatrixXd::Ones(horizon_length, 2); // v, omega

    int iterations = this->iterations_;
    int rollout_count = this->rollout_count_;
    int top_rollouts = this->top_rollouts_;

    //Repeat number of iterations
    for (int i = 0; i < iterations; ++i) {
        //Generate samples
        std::vector<Eigen::MatrixXd> random_actions =
            normal_distribute_(means, std_devs, rollout_count_);

        std::vector<std::pair<Eigen::MatrixXd, double>> rollouts(
            rollout_count_); // x, y, theta, v, omega for each horizon step paired with cost

        //For each sample/rollout, calculate the model and score it based on the cost function
        for (int j = 0; j < rollout_count_; ++j) {
            Eigen::MatrixXd state = calculate_model_(random_actions[j]);
            rollouts[j] = std::make_pair(state, calculate_cost_(state));
        }

        // Getting best options
        std::sort(rollouts.begin(), rollouts.end(), comparator);
        if (i == iterations - 1) {
            //MPC is done, publish velocity
            publish_velocity_(rollouts[0].first.coeff(0, 3),
                              rollouts[0].first.coeff(0, 4));
        } else {
            //Set the means and StdDevs to the avg of best samples and repeat
            means = mean_(rollouts);
            std_devs = std_dev_(rollouts);
        }
    }
    //Check if close to the path index
    this->update_setpoint_();
}
