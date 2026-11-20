/* includes //{ */

#include <mrs_uav_state_estimators/estimators/state/f9p.h>

//}

/* typedefs //{ */

#if USE_ROS_TIMER == 1
typedef mrs_lib::ROSTimer TimerType;
#else
typedef mrs_lib::ThreadTimer TimerType;
#endif

//}

namespace mrs_uav_state_estimators
{

namespace f9p
{

/* initialize() //{*/
void F9P::initialize(const rclcpp::Node::SharedPtr &node, const std::shared_ptr<CommonHandlers_t> &ch, const std::shared_ptr<PrivateHandlers_t> &ph) {

  node_  = node;
  clock_ = node->get_clock();

  error_publisher_ = std::make_unique<mrs_lib::errorgraph::ErrorPublisher>(node_, clock_, "EstimationManager", getPrintName());

  cbkgrp_timers_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  ch_ = ch;
  ph_ = ph;

  ns_frame_id_ = ch_->uav_name + "/" + frame_id_;

  // | --------------- param loader initialization -------------- |

  if (is_core_plugin_) {

    if (!ph_->param_loader->addYamlFile(ament_index_cpp::get_package_share_directory(package_name_) + "/config/private/" + getName() + "/" + getName() +
                                        ".yaml")) {
      RCLCPP_ERROR(node_->get_logger(), "[%s]: failed to load the private config file", getPrintName().c_str());
      error_publisher_->addOneshotError("failed to load the private config file");
      error_publisher_->flushAndShutdown();
    }

    if (!ph_->param_loader->addYamlFile(ament_index_cpp::get_package_share_directory(package_name_) + "/config/public/" + getName() + "/" + getName() +
                                        ".yaml")) {
      RCLCPP_ERROR(node_->get_logger(), "[%s]: failed to load the public config file", getPrintName().c_str());
      error_publisher_->addOneshotError("failed to load the public config file");
      error_publisher_->flushAndShutdown();
    }
  }

  // | --------------------- load parameters -------------------- |

  ph_->param_loader->loadParam("max_flight_z", max_flight_z_);
  ph_->param_loader->loadParam("message/topic", msg_topic_);
  ph_->param_loader->loadParam("kickoff", kickoff_, false);
  msg_topic_ = "/" + ch_->uav_name + "/" + msg_topic_;

  ph_->param_loader->loadParam("gnss/topic", gnss_topic_);
  gnss_topic_ = "/" + ch_->uav_name + "/" + gnss_topic_;

  std::string alt_ref_source, rtk_baseline_topic, tripod_height_topic;
  ph_->param_loader->loadParam("altitude_reference/source", alt_ref_source);
  ph_->param_loader->loadParam("altitude_reference/rtk_baseline_topic", rtk_baseline_topic);
  ph_->param_loader->loadParam("altitude_reference/tripod_height_topic", tripod_height_topic);
  ph_->param_loader->loadParam("altitude_reference/max_vertical_accuracy", max_vertical_accuracy_);

  if (alt_ref_source != "start" && alt_ref_source != "rtk_base") {
    RCLCPP_ERROR(node_->get_logger(), "[%s]: altitude_reference/source must be \"start\" or \"rtk_base\", got \"%s\"", getPrintName().c_str(),
                 alt_ref_source.c_str());
    error_publisher_->addOneshotError("invalid altitude_reference/source");
    error_publisher_->flushAndShutdown();
  }
  use_rtk_base_ = alt_ref_source == "rtk_base";

  // | --------------- subscribers initialization --------------- |

  mrs_lib::SubscriberHandlerOptions shopts;

  shopts.node                                = node_;
  shopts.node_name                           = getPrintName();
  shopts.no_message_timeout                  = mrs_lib::no_timeout;
  shopts.threadsafe                          = true;
  shopts.autostart                           = true;
  shopts.subscription_options.callback_group = cbkgrp_subs_;

  sh_f9p_odom_ = mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>(shopts, msg_topic_, &F9P::callbackF9POdom, this);

  sh_gnss_ = mrs_lib::SubscriberHandler<sensor_msgs::msg::NavSatFix>(shopts, gnss_topic_, &F9P::callbackGnss, this);

  if (use_rtk_base_) {
    // best effort is compatible with both reliable and best-effort publishers (mavros)
    mrs_lib::SubscriberHandlerOptions shopts_rtk = shopts;
    shopts_rtk.qos                               = rclcpp::SensorDataQoS();
    sh_rtk_baseline_ = mrs_lib::SubscriberHandler<mavros_msgs::msg::RTKBaseline>(shopts_rtk, "/" + ch_->uav_name + "/" + rtk_baseline_topic);

    mrs_lib::SubscriberHandlerOptions shopts_tripod = shopts;
    shopts_tripod.qos                               = rclcpp::QoS(1).reliable().transient_local();
    sh_tripod_height_ = mrs_lib::SubscriberHandler<std_msgs::msg::Float64>(shopts_tripod, tripod_height_topic);
  }

  // | ------------------------- timers ------------------------- |

  mrs_lib::TimerHandlerOptions opts;

  opts.node           = node_;
  opts.autostart      = true;
  opts.callback_group = cbkgrp_timers_;

  t_check_hz_last_ = clock_->now();

  {
    std::function<void()> callback_fcn = std::bind(&F9P::timerCheckF9POdomHz, this);

    timer_check_f9p_odom_hz_ = std::make_shared<TimerType>(opts, rclcpp::Rate(1.0, clock_), callback_fcn);
  }

  // | ---------------- publishers initialization --------------- |

  ph_odom_ = mrs_lib::PublisherHandler<nav_msgs::msg::Odometry>(node_, "~/" + Support::toSnakeCase(getName()) + "/odom"); // needed for tf
                                                                                                                          //
  if (ch_->debug_topics.state) {
    ph_uav_state_ = mrs_lib::PublisherHandler<mrs_msgs::msg::UavState>(node_, "~/" + Support::toSnakeCase(getName()) + "/uav_state");
  }

  if (ch_->debug_topics.covariance) {
    ph_pose_covariance_  = mrs_lib::PublisherHandler<mrs_msgs::msg::Float64ArrayStamped>(node_, "~/" + Support::toSnakeCase(getName()) + "/pose_covariance");
    ph_twist_covariance_ = mrs_lib::PublisherHandler<mrs_msgs::msg::Float64ArrayStamped>(node_, "~/" + Support::toSnakeCase(getName()) + "/twist_covariance");
  }

  if (ch_->debug_topics.innovation) {
    ph_innovation_ = mrs_lib::PublisherHandler<nav_msgs::msg::Odometry>(node_, Support::toSnakeCase("~/" + getName()) + "/innovation");
  }

  if (ch_->debug_topics.diag) {
    ph_diagnostics_ = mrs_lib::PublisherHandler<mrs_msgs::msg::EstimatorDiagnostics>(node_, Support::toSnakeCase("~/" + getName()) + "/diagnostics");
  }

  // | ------------------ initialize published messages ------------------ |

  uav_state_init_.header.frame_id = ns_frame_id_;
  uav_state_init_.child_frame_id  = ch_->frames.ns_fcu;

  uav_state_init_.estimator_horizontal = est_lat_name_;
  uav_state_init_.estimator_vertical   = est_alt_name_;
  uav_state_init_.estimator_heading    = est_hdg_name_;

  innovation_init_.header.frame_id         = ns_frame_id_;
  innovation_init_.child_frame_id          = ch_->frames.ns_fcu;
  innovation_init_.pose.pose.orientation.w = 1.0;

  // | ------------------ finish initialization ----------------- |

  if (changeState(INITIALIZED_STATE)) {
    RCLCPP_INFO(node_->get_logger(), "[%s]: Estimator initialized", getPrintName().c_str());
  } else {
    RCLCPP_INFO(node_->get_logger(), "[%s]: Estimator could not be initialized", getPrintName().c_str());
  }
}
/*//}*/

/*//{ start() */
bool F9P::start(void) {

  if (isInState(READY_STATE)) {

    timer_check_f9p_odom_hz_->start();

    changeState(STARTED_STATE);
    return true;

  } else {
    RCLCPP_WARN(node_->get_logger(), "[%s]: Estimator must be in READY_STATE to start it", getPrintName().c_str());
    clock_->sleep_for(1s);
  }
  return false;

  RCLCPP_ERROR(node_->get_logger(), "[%s]: Failed to start", getPrintName().c_str());
  return false;
}
/*//}*/

/*//{ pause() */
bool F9P::pause(void) {

  if (isInState(RUNNING_STATE)) {

    timer_check_f9p_odom_hz_->stop();

    changeState(STOPPED_STATE);

    return true;
  }
  return false;
}
/*//}*/

/*//{ reset() */
bool F9P::reset(void) {

  if (!isInitialized()) {
    RCLCPP_ERROR(node_->get_logger(), "[%s]: Cannot reset uninitialized estimator", getPrintName().c_str());
    return false;
  }

  changeState(STOPPED_STATE);

  RCLCPP_INFO(node_->get_logger(), "[%s]: Estimator reset", getPrintName().c_str());

  return true;
}
/*//}*/

/* timerCheckF9POdomHz() //{*/
void F9P::timerCheckF9POdomHz() {

  if (!isInitialized()) {
    return;
  }

  if (isInState(INITIALIZED_STATE)) {

    // first wait for a message
    if (!sh_f9p_odom_.hasMsg()) {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: %s msg on topic %s", getPrintName().c_str(), Support::waiting_for_string.c_str(),
                           sh_f9p_odom_.topicName().c_str());
      error_publisher_->addWaitingForTopicError(sh_f9p_odom_.topicName());
      return;
    }

    // calculate average rate of messages
    if (counter_odom_msgs_ > 0) {

      rclcpp::Time t_now  = clock_->now();
      double       dt     = (t_now - t_check_hz_last_).seconds();
      double       avg_hz = counter_odom_msgs_ / dt;
      t_check_hz_last_    = t_now;
      counter_odom_msgs_  = 0;
      RCLCPP_INFO(node_->get_logger(), "[%s]: rate of f9p odom: %.2f Hz", getPrintName().c_str(), avg_hz);

      // check message rate stability
      if (abs(avg_hz - prev_avg_hz_) >= 5) {
        RCLCPP_INFO(node_->get_logger(), "[%s]: %s stable f9p odometry rate. now: %.2f Hz prev: %.2f Hz", getPrintName().c_str(),
                    Support::waiting_for_string.c_str(), avg_hz, prev_avg_hz_);
        prev_avg_hz_ = avg_hz;
        return;
      }

      // the message rate must be higher than required by the control manager
      if (!kickoff_ && avg_hz < ch_->desired_uav_state_rate * 0.9) {
        RCLCPP_ERROR(node_->get_logger(),
                     "[%s]: rate of f9p odom: %.2f Hz is lower than desired uav_state rate: %.2f Hz. Flight not allowed. Provide higher f9p odometry "
                     "rate "
                     "or use a higher-level controller.",
                     getPrintName().c_str(), avg_hz, ch_->desired_uav_state_rate);
        // note: might run the publishing asynchronously on the desired rate in this case to still be able to fly
        // the states would then be interpolated by ZOH (bleeh)
        /* timer_update_       = nh.createTimer(ros::Rate(ch_->desired_uav_state_rate), &F9P::timerUpdate, this); */
        return;
      }
    }

    if (use_rtk_base_ && !getHeightAboveBaseGround(true)) {
      return;
    }

    changeState(READY_STATE);
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: Estimator is ready to start", getPrintName().c_str());
    timer_check_f9p_odom_hz_->stop();
  }
}
/*//}*/

/* timerUpdate() //{*/

void F9P::timerUpdate() {

  if (!isInitialized()) {
    return;
  }

  if (isInState(STARTED_STATE)) {

    changeState(RUNNING_STATE);
  }

  // If the estimator is active the updateUavState is triggered directly by estimation manager
  if (!is_active_) {
    updateUavState();
  }

  publishUavState();
  publishOdom();
  publishCovariance();
  publishInnovation();
  publishDiagnostics();
}

/*//}*/

/* updateUavState() //{ */

void F9P::updateUavState() {

  std::scoped_lock lock(mutex_update_uav_state_);

  if (!isInitialized()) {
    return;
  }

  if (!got_gnss_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: Missing GNSS data on topic %s", getPrintName().c_str(), sh_f9p_odom_.topicName().c_str());
    return;
  }

  if (isInState(STARTED_STATE)) {

    changeState(RUNNING_STATE);
  }

  nav_msgs::msg::Odometry::ConstSharedPtr msg = sh_f9p_odom_.getMsg();

  if (first_iter_) {

    // updateUavState() can run before READY, so the capture waits for a good baseline
    std::optional<BaseGroundHeight> base_ground;
    if (use_rtk_base_) {
      base_ground = getHeightAboveBaseGround(false);
      if (!base_ground) {
        return;
      }
    }

    gnss_x_ = (gnss_x_ - msg->pose.pose.position.x) - ch_->world_origin.x;
    gnss_y_ = (gnss_y_ - msg->pose.pose.position.y) - ch_->world_origin.y;
    /* gnss_z_                    = gnss_z_ - msg->pose.pose.position.z; */ // absolute AMSL altitude

    if (use_rtk_base_) {
      gnss_z_ = base_ground->height - msg->pose.pose.position.z;
      RCLCPP_INFO(node_->get_logger(), "[%s]: altitude referenced to RTK base ground: height %.3f m, tripod %.3f m, offset %.3f m",
                  getPrintName().c_str(), base_ground->height, base_ground->tripod_height, gnss_z_);
    } else {
      gnss_z_ = -msg->pose.pose.position.z;  // relative altitude to start
    }

    prev_msg_   = msg;
    first_iter_ = false;
  }

  mrs_msgs::msg::UavState uav_state = uav_state_init_;

  /* const rclcpp::Time time_now = clock_->now(); */

  // here we are assuming the f9p odometry has correct timestamp
  const rclcpp::Time time_now = msg->header.stamp;
  uav_state.header.stamp      = time_now;

  uav_state.pose.position.x = msg->pose.pose.position.x + gnss_x_;
  uav_state.pose.position.y = msg->pose.pose.position.y + gnss_y_;
  uav_state.pose.position.z = msg->pose.pose.position.z + gnss_z_;

  uav_state.pose.orientation = msg->pose.pose.orientation;

  // uav_state has velocities in the parent frame
  uav_state.velocity.linear  = Support::rotateVector(msg->twist.twist.linear, msg->pose.pose.orientation);
  uav_state.velocity.angular = msg->twist.twist.angular;

  const nav_msgs::msg::Odometry odom = Support::uavStateToOdom(uav_state);

  nav_msgs::msg::Odometry innovation = innovation_init_;
  innovation.header.stamp            = time_now;

  innovation.pose.pose.position.x = prev_msg_->pose.pose.position.x - msg->pose.pose.position.x;
  innovation.pose.pose.position.y = prev_msg_->pose.pose.position.y - msg->pose.pose.position.y;
  innovation.pose.pose.position.z = prev_msg_->pose.pose.position.z - msg->pose.pose.position.z;

  mrs_msgs::msg::Float64ArrayStamped pose_covariance, twist_covariance;
  pose_covariance.header.stamp  = time_now;
  twist_covariance.header.stamp = time_now;

  const int n_states = 6; // TODO this should be defined somewhere else
  pose_covariance.values.resize(n_states * n_states);
  pose_covariance.values.at(n_states * AXIS_X + AXIS_X) = 1e-10;
  pose_covariance.values.at(n_states * AXIS_Y + AXIS_Y) = 1e-10;
  pose_covariance.values.at(n_states * AXIS_Z + AXIS_Z) = 1e-10;

  twist_covariance.values.resize(n_states * n_states);
  twist_covariance.values.at(n_states * AXIS_X + AXIS_X) = 1e-10;
  twist_covariance.values.at(n_states * AXIS_Y + AXIS_Y) = 1e-10;
  twist_covariance.values.at(n_states * AXIS_Z + AXIS_Z) = 1e-10;

  mrs_lib::set_mutexed(mtx_uav_state_, uav_state, uav_state_);
  mrs_lib::set_mutexed(mtx_odom_, odom, odom_);
  mrs_lib::set_mutexed(mtx_innovation_, innovation, innovation_);
  mrs_lib::set_mutexed(mtx_covariance_, pose_covariance, pose_covariance_);
  mrs_lib::set_mutexed(mtx_covariance_, twist_covariance, twist_covariance_);

  prev_msg_ = msg;
}

//}

/*//{ callbackF9POdom() */
void F9P::callbackF9POdom([[maybe_unused]] const nav_msgs::msg::Odometry::ConstSharedPtr msg) {

  counter_odom_msgs_++;

  if (!isInitialized()) {
    return;
  }

  // Prevents publishing uninitialized odometry message when the estimator is started as active
  if (!isRunning() && !isStarted()) {
    return;
  }

  updateUavState();

  publishUavState();
  publishOdom();
  publishCovariance();
  publishInnovation();
  publishDiagnostics();
}
/*//}*/

/*//{ callbackGnss() */
void F9P::callbackGnss(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {

  if (!isInitialized()) {
    return;
  }

  if (got_gnss_) {
    return;
  }

  if (!std::isfinite(msg->latitude)) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s] NaN detected in GNSS variable \"msg->latitude\"!!!", getPrintName().c_str());
    return;
  }

  if (!std::isfinite(msg->longitude)) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s] NaN detected in GNSS variable \"msg->longitude\"!!!", getPrintName().c_str());
    return;
  }

  mrs_lib::UTM(msg->latitude, msg->longitude, &gnss_x_, &gnss_y_);
  gnss_z_ = msg->altitude;
  RCLCPP_INFO(node_->get_logger(), "[%s]: First GNSS obtained: %.2f %.2f %.2f", getPrintName().c_str(), gnss_x_, gnss_y_, gnss_z_);
  got_gnss_ = true;
}
/*//}*/

/*//{ getHeightAboveBaseGround() */
std::optional<F9P::BaseGroundHeight> F9P::getHeightAboveBaseGround(const bool verbose) {

  auto wait = [&](const std::string &what) -> std::optional<BaseGroundHeight> {
    if (verbose) {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: %s %s", getPrintName().c_str(), Support::waiting_for_string.c_str(), what.c_str());
      error_publisher_->addWaitingForTopicError(what);
    }
    return std::nullopt;
  };

  if (!sh_tripod_height_.hasMsg()) {
    return wait(sh_tripod_height_.topicName());
  }

  const double tripod_height = sh_tripod_height_.getMsg()->data;
  if (!std::isfinite(tripod_height) || tripod_height <= 0.0 || tripod_height >= 5.0) {
    return wait("valid tripod height (got " + std::to_string(tripod_height) + " m, expected 0-5 m)");
  }

  if (!sh_rtk_baseline_.hasMsg()) {
    return wait(sh_rtk_baseline_.topicName());
  }

  const auto bl = sh_rtk_baseline_.getMsg();
  if (bl->rtk_health != 1 || bl->baseline_coords_type != mavros_msgs::msg::RTKBaseline::RTK_BASELINE_COORDINATE_SYSTEM_NED) {
    return wait("RTK fixed NED baseline");
  }

  if (bl->accuracy / 1000.0 > max_vertical_accuracy_) {
    return wait("RTK baseline accuracy <= " + std::to_string(max_vertical_accuracy_) + " m (got " + std::to_string(bl->accuracy / 1000.0) + " m)");
  }

  // NED: down is positive below the base antenna
  return BaseGroundHeight{tripod_height - bl->baseline_c_mm / 1000.0, tripod_height};
}
/*//}*/

/*//{ isConverged() */
bool F9P::isConverged() {

  // TODO: check convergence by rate of change of determinant
  // most likely not used in top-level estimator

  return true;
}
/*//}*/

/*//{ setUavState() */
bool F9P::setUavState([[maybe_unused]] const mrs_msgs::msg::UavState &uav_state) {

  if (!isInState(STOPPED_STATE)) {
    RCLCPP_WARN(node_->get_logger(), "[%s]: Estimator state can be set only in the STOPPED state", getPrintName().c_str());
    return false;
  }

  RCLCPP_WARN(node_->get_logger(), "[%s]: Setting the state of this estimator is not implemented.", getPrintName().c_str());
  return false;
}
/*//}*/

} // namespace f9p

} // namespace mrs_uav_state_estimators

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_uav_state_estimators::f9p::F9P, mrs_uav_managers::StateEstimator)
