#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <ik_solver_msgs/srv/get_ik.hpp>
#include <algorithm>
#include <random>

using GetPositionIK = moveit_msgs::srv::GetPositionIK;
using CustomIk = ik_solver_msgs::srv::GetIk;

class MoveItIkAdapter : public rclcpp::Node
{
public:
    MoveItIkAdapter()
        : Node("moveit_ik_adapter_node"), rng_(0)
    {

        std::string moveit_service_name;
        std::string custom_ik_service_name;
        std::vector<double> seed_weights;

        // Declare parameters
        this->declare_parameter<std::string>("moveit_service_name", "ik_adapter_service");
        this->declare_parameter<std::string>("custom_ik_service_name", "custom_ik_service");
        this->declare_parameter<std::vector<double>>("seed_weights", seed_weights);

        this->get_parameter("moveit_service_name", moveit_service_name);
        this->get_parameter("custom_ik_service_name", custom_ik_service_name);
        this->get_parameter("seed_weights", seed_weights);


        // Reentrant callback group
        cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // MoveIt IK service
        ik_service_ = this->create_service<GetPositionIK>(
            moveit_service_name,
            std::bind(&MoveItIkAdapter::handle_moveit_ik, this,
                      std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            cbg_);

        // Custom IK client
        ik_client_ = this->create_client<CustomIk>(custom_ik_service_name, rmw_qos_profile_services_default);

        RCLCPP_INFO(get_logger(), "MoveIt IK adapter ready: %s -> calling %s",
                    moveit_service_name.c_str(), custom_ik_service_name.c_str());

        seed_weights_ = seed_weights;

    }

private:
    void handle_moveit_ik(
        const std::shared_ptr<GetPositionIK::Request> request,
        std::shared_ptr<GetPositionIK::Response> response)
    {
        const auto& request_joint_names = request->ik_request.robot_state.joint_state.name;
        const auto& request_joint_positions = request->ik_request.robot_state.joint_state.position;

        if (!ik_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(get_logger(), "Custom IK service unavailable");
            response->error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
            return;
        }

        auto req = std::make_shared<CustomIk::Request>();
        req->target.pose = request->ik_request.pose_stamped;
        req->seed_joint_names = request_joint_names;
        req->max_number_of_solutions = 0;
        req->stall_iterations = 0;

        auto future = ik_client_->async_send_request(req);

        if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
            RCLCPP_ERROR(get_logger(), "Custom IK service did not respond in time");
            response->error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
            return;
        }

        auto res = future.get();

        if (!res->success || res->joint_names.empty() || res->solution.configurations.empty()) {
            RCLCPP_WARN(get_logger(), "IK solver failed: %s", res->message.c_str());
            response->error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
            return;
        }

        std::vector<double> seed_weights(res->joint_names.size());
        bool randomize_weights = false;
        if (seed_weights_.empty())
        {
          RCLCPP_INFO(get_logger(), "seed_weights is empty. default is all ones.");
          std::fill(seed_weights.begin(), seed_weights.end(), 1.0);
        }
        else if (seed_weights_.size() != seed_weights.size())
        {
          RCLCPP_WARN(get_logger(), "seed_weights has unexpected size %zu. Expected size %zu. Default values will be used (all ones).", seed_weights_.size(), seed_weights.size());
        }
        else
        {
          randomize_weights = std::all_of(seed_weights_.begin(), seed_weights_.end(), [](double w){ return std::abs(w) < 1e-12; });
          seed_weights = seed_weights_;
        }

        // Start with original robot state
        std::vector<double> full_solution = request_joint_positions;

        // Find the solution closest to the seed
        size_t best_idx = 0;
        if (randomize_weights)
        {
          std::random_device rd;
          std::uniform_int_distribution<size_t> random_idx_distr(0, res->solution.configurations.size() - 1);
          best_idx = random_idx_distr(rng_);
          RCLCPP_INFO(get_logger(), "All seed weights are zero. Random IK solution chosen: %zu", best_idx);
        }
        else
        {
          double min_distance = std::numeric_limits<double>::max();

          for (size_t sol_idx = 0; sol_idx < res->solution.configurations.size(); ++sol_idx)
          {
            const auto& candidate = res->solution.configurations[sol_idx].configuration;
            double dist = 0.0;

            for (size_t j = 0; j < res->joint_names.size(); ++j)
            {
                auto it = std::find(request_joint_names.begin(), request_joint_names.end(), res->joint_names[j]);
                if (it != request_joint_names.end())
                {
                    size_t idx = std::distance(request_joint_names.begin(), it);
                    if (idx < request_joint_positions.size()) {
                        double delta = candidate[j] - request_joint_positions[idx];
                        dist += seed_weights[j] * delta * delta;
                    }
                }
            }

            if (dist < min_distance)
            {
                min_distance = dist;
                best_idx = sol_idx;
            }
          }

        }

        // Merge the best solution into the full MoveIt state
        const auto& best_solution = res->solution.configurations[best_idx].configuration;
        for (size_t j = 0; j < res->joint_names.size(); ++j) {
            const std::string& joint_name = res->joint_names[j];
            auto it = std::find(request_joint_names.begin(), request_joint_names.end(), joint_name);
            if (it != request_joint_names.end()) {
                size_t idx = std::distance(request_joint_names.begin(), it);
                if (idx < full_solution.size()) {
                    full_solution[idx] = best_solution[j];
                }
            } else {
                RCLCPP_WARN(get_logger(), "IK solver returned joint '%s' not in MoveIt request", joint_name.c_str());
            }
        }

        response->solution.joint_state.name = request_joint_names;
        response->solution.joint_state.position = full_solution;
        response->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;

        RCLCPP_INFO(get_logger(), "MoveIt IK response ready (closest to seed selected)");
    }

    rclcpp::CallbackGroup::SharedPtr cbg_;
    rclcpp::Service<GetPositionIK>::SharedPtr ik_service_;
    rclcpp::Client<CustomIk>::SharedPtr ik_client_;
    std::vector<double> seed_weights_;
    std::mt19937 rng_;


};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MoveItIkAdapter>();

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
