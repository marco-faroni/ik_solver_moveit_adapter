#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>

using GetPositionIK = moveit_msgs::srv::GetPositionIK;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  //rclcpp::NodeOptions options;
  //options.use_intra_process_comms(true);

  auto node = rclcpp::Node::make_shared("moveit_ik_adapter_node");

  std::string service_name = "ik_adapter_service";

  // ---------- FIX: Reentrant callback group ----------
  auto cbg = node->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant
  );


  auto service = node->create_service<GetPositionIK>(
    service_name,
    [node](const std::shared_ptr<GetPositionIK::Request> request,
           std::shared_ptr<GetPositionIK::Response> response)
    {
      RCLCPP_INFO(node->get_logger(), "hello world");

      const auto& in_names =
          request->ik_request.robot_state.joint_state.name;

      std::size_t n = in_names.size();

      RCLCPP_INFO(node->get_logger(), "n=%zu", n);



      response->solution.joint_state.name = in_names;
      response->solution.joint_state.position.assign(n, 0.0);

      if (n>=6)
      {
        response->solution.joint_state.position[1]=-1.57;
        response->solution.joint_state.position[3]=-1.57;
        response->solution.joint_state.position[4]=-1.0;

      }

      response->error_code.val =
          moveit_msgs::msg::MoveItErrorCodes::SUCCESS;

      RCLCPP_INFO(node->get_logger(), "bye bye world");
    },
    rmw_qos_profile_services_default,
    cbg
  );

  RCLCPP_INFO(node->get_logger(),
              "MoveIt IK adapter service ready on '%s'",
              service_name.c_str());

  // ---------- FIX: Multi-threaded executor ----------
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  // --------------------------------------------------

  rclcpp::shutdown();
  return 0;
}
