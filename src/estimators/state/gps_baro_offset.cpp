#include <mrs_uav_state_estimators/estimators/state/state_generic.h>

#include <mrs_msgs/msg/hw_api_status.hpp>
#include <mrs_msgs/msg/float64_stamped.hpp>

namespace mrs_uav_state_estimators
{

namespace gps_baro_offset
{

const char estimator_name[] = "gps_baro_offset";
const bool is_core_plugin   = true;

class GpsBaroOffset : public StateGeneric {
public:
  GpsBaroOffset() : StateGeneric(estimator_name, is_core_plugin) {
  }

  void initialize(const rclcpp::Node::SharedPtr &node, const std::shared_ptr<CommonHandlers_t> &ch, const std::shared_ptr<PrivateHandlers_t> &ph) override;

  void updateUavState() override;

private:
  rclcpp::CallbackGroup::SharedPtr cbkgrp_status_;

  mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiStatus> sh_hw_api_status_;
  void                                                   callbackHwApiStatus(const mrs_msgs::msg::HwApiStatus::ConstSharedPtr msg);

  mrs_lib::PublisherHandler<mrs_msgs::msg::Float64Stamped> ph_z_offset_;

  // updateUavState() is called both from the estimator's timer and from the estimation manager;
  // serializes the base update + offset subtraction so the offset is never applied twice
  std::mutex mtx_offset_update_;

  std::atomic<bool>   is_first_status_msg_ = true;
  std::atomic<bool>   is_armed_prev_       = false;
  std::atomic<double> z_raw_last_          = 0.0; // latest unoffset altitude estimate, updated every updateUavState() tick
  std::atomic<double> z_offset_            = 0.0; // z_raw_last_ captured at the last disarmed -> armed edge
};

/* initialize() //{ */
void GpsBaroOffset::initialize(const rclcpp::Node::SharedPtr &node, const std::shared_ptr<CommonHandlers_t> &ch, const std::shared_ptr<PrivateHandlers_t> &ph) {

  StateGeneric::initialize(node, ch, ph);

  std::string topic_hw_api_status;
  ph->param_loader->loadParam("topics/hw_api_status", topic_hw_api_status);
  topic_hw_api_status = "/" + ch_->uav_name + "/" + topic_hw_api_status;

  if (!ph->param_loader->loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "[%s]: Could not load all non-optional parameters. Shutting down.", getPrintName().c_str());
    error_publisher_->addOneshotError("Could not load all non-optional parameters");
    error_publisher_->flushAndShutdown();
  }

  cbkgrp_status_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  mrs_lib::SubscriberHandlerOptions shopts;

  shopts.node                                = node_;
  shopts.node_name                           = getPrintName();
  shopts.no_message_timeout                  = mrs_lib::no_timeout;
  shopts.threadsafe                          = true;
  shopts.autostart                           = true;
  shopts.subscription_options.callback_group = cbkgrp_status_;

  sh_hw_api_status_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiStatus>(shopts, topic_hw_api_status, &GpsBaroOffset::callbackHwApiStatus, this);

  if (ch_->debug_topics.diag) {
    ph_z_offset_ = mrs_lib::PublisherHandler<mrs_msgs::msg::Float64Stamped>(node_, Support::toSnakeCase("~/" + getName()) + "/z_offset");
  }
}
/*//}*/

/* updateUavState() //{ */
void GpsBaroOffset::updateUavState() {

  std::scoped_lock lock(mtx_offset_update_);

  StateGeneric::updateUavState();

  const double offset = z_offset_.load();

  auto uav_state = mrs_lib::get_mutexed(mtx_uav_state_, uav_state_);
  auto odom      = mrs_lib::get_mutexed(mtx_odom_, odom_);

  // StateGeneric::updateUavState() always (re)computes z from scratch from the raw altitude estimate,
  // so this is never affected by the offset subtracted below
  z_raw_last_ = uav_state.pose.position.z;

  uav_state.pose.position.z -= offset;
  odom.pose.pose.position.z -= offset;

  mrs_lib::set_mutexed(mtx_uav_state_, uav_state, uav_state_);
  mrs_lib::set_mutexed(mtx_odom_, odom, odom_);

  if (ch_->debug_topics.diag) {
    mrs_msgs::msg::Float64Stamped offset_msg;
    offset_msg.header.stamp    = uav_state.header.stamp;
    offset_msg.header.frame_id = ns_frame_id_;
    offset_msg.value           = offset;
    ph_z_offset_.publish(offset_msg);
  }
}
/*//}*/

/* callbackHwApiStatus() //{ */
void GpsBaroOffset::callbackHwApiStatus(const mrs_msgs::msg::HwApiStatus::ConstSharedPtr msg) {

  if (!isInitialized()) {
    return;
  }

  const bool armed_now = msg->armed;

  // initialize only is_armed_prev_ otherwise if the estimator is restarted, the offset would apply also mid-air
  if (is_first_status_msg_) {
    is_first_status_msg_ = false;
    is_armed_prev_       = armed_now;
    return;
  }

  // offset only when disarmed -> armed and not in offboard. 
  if (armed_now && !is_armed_prev_ && !msg->offboard) {
    const double offset = z_raw_last_.load();
    z_offset_           = offset;
    RCLCPP_INFO(node_->get_logger(), "[%s]: captured altitude offset of %.3f m at arming", getPrintName().c_str(), offset);
  }

  is_armed_prev_ = armed_now;
}
/*//}*/

} // namespace gps_baro_offset

} // namespace mrs_uav_state_estimators

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_uav_state_estimators::gps_baro_offset::GpsBaroOffset, mrs_uav_managers::StateEstimator)
