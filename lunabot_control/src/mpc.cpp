#include lunabot_controlmpc.h

MPCMPC(rosNodeHandle nh) {
  double frequency;
  stdstring map_topic, path_topic, odom_topic, cmd_vel_topic;
   Global params
  rosparamget(odom_topic, odom_topic);
  rosparamget(cmd_vel_topic, cmd_vel_topic);

   Nav params
  rosparamget(map_topic, map_topic);
  rosparamget(global_path_topic, path_topic);

   MPC params
  rosparamget(~rollout_count, this-rollout_count_);
  rosparamget(~top_rollouts, this-top_rollouts_);
  rosparamget(~iterations, this-iterations_);
  rosparamget(~w_linear, this-w_linear_);
  rosparamget(~w_angular, this-w_angular_);
  rosparamget(~w_waypoint, this-w_waypoint_);
  rosparamget(~w_occupied, this-w_occupied_);
  rosparamget(~horizon_length, this-horizon_length_);
  rosparamget(~frequency, frequency);
  rosparamget(~min_distance_threshold, this-min_dist_thres_);
  rosparamget(~velocity_limitslinear, lin_lim_);
  rosparamget(~velocity_limitsangular, ang_lim_);

   MPC Variables
  this-delta_time_ = 1  frequency;
  this-path_ind_ = 0;
  this-enabled_ = false;

   Ros publishers and subscribers
  this-velocity_pub_ = nh-advertisegeometry_msgsTwist(cmd_vel_topic, 10);
  this-grid_sub_ = nh-subscribe(map_topic, 10, &MPCupdate_grid, this);
  this-path_sub_ = nh-subscribe(path_topic, 10, &MPCupdate_path,
                                  this);  Convert to Map
  this-robot_pos_sub_ = nh-subscribe(odom_topic, 10, &MPCupdate_robot_pos,
                                       this);  Convert to Map
}

bool MPCcheck_collision_(EigenRowVectorXd pos) {
  stdvectordouble state = {pos(0), pos(1)};

  return this-map_.occupied_at_pos(state);
}

double MPCfind_closest_distance_(EigenRowVectorXd pos) {
  stdvectorstdvectordouble path = this-path_;

  if (path.empty()) {
    return -1;
  }

  double min_dist = -1;

  stdvectordouble point = path[0];
  for (int i = 1; i  path.size(); ++i) {
    stdvectordouble new_point = path[i];

     Building Parametric Lines
    double x_vel = new_point[0] - point[0];
    double y_vel = new_point[1] - point[1];

    if (x_vel == 0 && y_vel == 0) {
      return 0;
    }

    double t = (x_vel  (pos(0) - point[0]) + y_vel  (pos(1) - point[1])) 
               (x_vel  x_vel + y_vel  y_vel);  Finding time where line is closest to point

     Scaling to be on the line segment
    if (t  0) {
      t = 0;
    } else if (t  1) {
      t = 1;
    }

    double x_closest = point[0] + t  x_vel;
    double y_closest = point[1] + t  y_vel;

    double distance = stdsqrt((x_closest - pos(0))  (x_closest - pos(0)) +
                                (y_closest - pos[1])  (y_closest - pos(1)));

    if (min_dist == -1  distance  min_dist) {
      min_dist = distance;
    }

    point = new_point;
  }
  return min_dist;
}

int MPCis_close_() { return dist_to_setpoint_() = min_dist_thres_  min_dist_thres_; }

void MPCupdate_setpoint_() {
  if (is_close_()) {
    if (this-path_ind_ == this-path_.size() - 1) {
      publish_velocity_(0, 0);
      this-path_ind_ = 0;
      this-enabled_ = false;
    } else {
      this-path_ind_++;
    }
  }
}

void MPCupdate_grid(const nav_msgsOccupancyGrid &grid) { map_.update_map(grid); }

void MPCupdate_path(const nav_msgsPath &path) {
  ROS_INFO(PATH);
  this-path_.clear();
  this-path_ind_ = 0;
  for (int i = 0; i  path.poses.size(); ++i) {
    geometry_msgsPoint position = path.poses[i].pose.position;
    stdvectordouble pos;
    pos.push_back(position.x);
    pos.push_back(position.y);
    this-path_.push_back(pos);
  }

  this-enabled_ = true;
}

static double get_yaw_(geometry_msgsQuaternion q) {
  return stdatan2(2  (q.z  q.w + q.x  q.y), 1 - 2  (q.z  q.z + q.y  q.y));
}

void MPCupdate_robot_pos(const nav_msgsOdometry &odometry) {
  geometry_msgsPose pose = odometry.pose.pose;
  this-robot_pos_.clear();
  this-robot_pos_.push_back(pose.position.x);
  this-robot_pos_.push_back(pose.position.y);
  this-robot_pos_.push_back(get_yaw_(pose.orientation));
}

double MPCclamp_(double val, double low, double high) {
  if (val  low) {
    return low;
  } else if (val  high) {
    return high;
  } else {
    return val;
  }
}

void MPCpublish_velocity_(double linear, double angular) {
  geometry_msgsTwist twist;
  linear = clamp_(linear, lin_lim_[0], lin_lim_[1]);
  angular = clamp_(angular, ang_lim_[0], ang_lim_[1]);
  twist.linear.x = linear;
  twist.angular.z = angular;
  this-velocity_pub_.publish(twist);
}

 See Boundary and Constraint Handling
 httpscma-es.github.iocmaes_sourcecode_page.html#practical
double MPCsmooth_clamp_(double x, double a, double b) {
  return a + (b - a)  (1 + sin(M_PI  x  2))  2;
}

stdvectorEigenMatrixXd MPCnormal_distribute_(EigenMatrixXd means,
                                                     EigenMatrixXd std_devs, int count) {
  stdvectorEigenMatrixXd random_vels(count);
  for (int i = 0; i  count; ++i) {
    EigenMatrixXd random_vel(means.rows(), means.cols());

    for (int j = 0; j  means.rows(); ++j) {
      for (int k = 0; k  means.cols(); ++k) {
        stdrandom_device rd;
        stddefault_random_engine generator(rd());
        stdnormal_distributiondouble distribution(means(j, k), std_devs(j, k));
        double rand_vel = distribution(generator);
        
        if (k == 0) {
          rand_vel = smooth_clamp_(rand_vel, lin_lim_[0], lin_lim_[1]);
        } else {
          rand_vel = smooth_clamp_(rand_vel, ang_lim_[0], ang_lim_[1]);
        }
        
          if(k == 0) {
           rand_vel = clamp_(rand_vel, lin_lim_[0], lin_lim_[1]);
          } else {
           rand_vel = clamp_(rand_vel, ang_lim_[0], ang_lim_[1]);
          }
        random_vel(j, k) = rand_vel;
      }
      double curvature = random_vel(j, 1);
      double min_curv = 2;
       if(curvature  0) {
      curvature = clamp_(curvature, -min_curv, min_curv);
       } else {
         curvature = clamp_(curvature, curvature, -min_curv);
       }
      random_vel(j, 0) = clamp_(random_vel(j, 0), lin_lim_[0], lin_lim_[1]);
      random_vel(j, 1) = curvature;
    }
    random_vels[i] = random_vel;
  }
  return random_vels;
}

EigenMatrixXd MPCmean_(stdvectorstdpairEigenMatrixXd, double rollouts) {
  EigenMatrixXd mean = EigenMatrixXdZero(this-horizon_length_, 2);
  for (int i = 0; i  this-top_rollouts_; ++i) {
    for (int j = 0; j  rollouts[i].first.rows(); ++j) {
      mean(j, 0) += rollouts[i].first(j, 3);
      mean(j, 1) += rollouts[i].first(j, 4);
    }
  }
  return mean  this-top_rollouts_;
}

EigenMatrixXd MPCstd_dev_(stdvectorstdpairEigenMatrixXd, double rollouts) {
  EigenMatrixXd temp_mean = mean_(rollouts);
  EigenMatrixXd std_dev = EigenMatrixXdZero(this-horizon_length_, 2);
  for (int i = 0; i  this-top_rollouts_; ++i) {
    for (int j = 0; j  rollouts[i].first.rows(); ++j) {
      std_dev(j, 0) +=
          (rollouts[i].first(j, 3) - temp_mean(j, 0))  (rollouts[i].first(j, 3) - temp_mean(j, 0));
      std_dev(j, 1) +=
          (rollouts[i].first(j, 4) - temp_mean(j, 1))  (rollouts[i].first(j, 4) - temp_mean(j, 1));
    }
  }
  ROS_ASSERT(this-top_rollouts_  1);

  std_dev = this-top_rollouts_ - 1;
  for (int i = 0; i  std_dev.rows(); ++i) {
    for (int j = 0; j  std_dev.cols(); ++j) {
      std_dev(i, j) = stdsqrt(std_dev(i, j));
    }
  }
  return std_dev;
}

double MPCdist_to_setpoint_() {
  return ((robot_pos_[0] - path_[path_ind_][0])  (robot_pos_[0] - path_[path_ind_][0]) +
          (robot_pos_[1] - path_[path_ind_][1])  (robot_pos_[1] - path_[path_ind_][1]));
}

double MPCcalculate_cost_(EigenMatrixXd rollout) {
  SWRI_PROFILE(calculate-cost);
  double cost = 0;
  stdvectordouble robot_pos = this-robot_pos_;
  stdvectordouble goal = this-goal_;
  for (int i = 0; i  rollout.rows(); ++i) {
    EigenRowVectorXd position = rollout.row(i);
    cost += this-w_linear_  ((position(0) - robot_pos[0])  (position(0) - robot_pos[0]) +
                               (position(1) - robot_pos[1])  (position(1) - robot_pos[1]));
    cost += this-w_angular_  (position(2) - robot_pos[2])  (position(2) - robot_pos[2]);
    int path_cost_i = stdmin(path_ind_ + i, (int)path_.size() - 1);
    cost += this-w_waypoint_ 
            ((position(0) - path_[path_cost_i][0])  (position(0) - path_[path_cost_i][0]) +
             (position(1) - path_[path_cost_i][1])  (position(1) - path_[path_cost_i][1]));
    cost += this-w_occupied_  check_collision_(position);
  }

  return cost;
}

EigenMatrixXd MPCcalculate_model_(EigenMatrixXd actions) {
  SWRI_PROFILE(calculate-model);
  EigenVectorXd current_pos(3);  3 x 1 mat
  current_pos  this-robot_pos_[0], this-robot_pos_[1], this-robot_pos_[2];
  EigenMatrixXd model(this-horizon_length_, 5);  x, y, theta, v, omega
  double wheel_separation = 0.4;

  for (int i = 0; i  this-horizon_length_; ++i) {
    model(i, 0) = current_pos(0);
    model(i, 1) = current_pos(1);
    model(i, 2) = current_pos(2);
    model(i, 3) = actions(i, 0);
    model(i, 4) = actions(i, 1);
    current_pos(0) += stdcos(current_pos(2))  this-delta_time_  actions(i, 0);
    current_pos(1) += stdsin(current_pos(2))  this-delta_time_  actions(i, 0);
    current_pos(2) += this-delta_time_  clamp_(actions(i, 0), 0, actions(i, 0))  actions(i, 1);
  }

  return model;
}

bool comparator(stdpairEigenMatrixXd, double a, stdpairEigenMatrixXd, double b) {
  return a.second  b.second;
}

void MPCcalculate_velocity() {
  SWRI_PROFILE(calculate-velocity);
  if (this-robot_pos_.empty()  this-path_.empty()  !this-enabled_) {
    return;
  }
  int horizon_length = this-horizon_length_;
  EigenMatrixXd means = EigenMatrixXdZero(horizon_length, 2);     v, omega
  EigenMatrixXd std_devs = EigenMatrixXdOnes(horizon_length, 2);  v, omega
  int iterations = this-iterations_;
  int rollout_count = this-rollout_count_;
  int top_rollouts = this-top_rollouts_;
  for (int i = 0; i  iterations; ++i) {
    stdvectorEigenMatrixXd random_actions =
        normal_distribute_(means, std_devs, rollout_count_);
    stdvectorstdpairEigenMatrixXd, double rollouts(
        rollout_count_);  x, y, theta, v, omega for each horizon step
                          paired with cost

    for (int j = 0; j  rollout_count_; ++j) {

      EigenMatrixXd state = calculate_model_(random_actions[j]);
      rollouts[j] = stdmake_pair(state, calculate_cost_(state));
    }

     Getting best options
    stdsort(rollouts.begin(), rollouts.end(), comparator);
    if (i == iterations - 1) {
      publish_velocity_(rollouts[0].first.coeff(0, 3), rollouts[0].first.coeff(0, 4));
    } else {
      means = mean_(rollouts);
      std_devs = std_dev_(rollouts);
    }
  }
  this-update_setpoint_();
}
