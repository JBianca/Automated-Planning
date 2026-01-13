#include <memory>
#include <algorithm>
#include <string>
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#define ACTION_NAME "pick_hall_alpha"

using namespace std::chrono_literals;

class MoveAction : public plansys2::ActionExecutorClient
{
public:
  MoveAction()
  : plansys2::ActionExecutorClient(ACTION_NAME, 250ms)
  {
    progress_ = 0.0;
  }

private:
  void do_work()
  {
    if (progress_ < 1.0) {
      progress_ += 0.02;
      std::string feedback_msg = std::string(ACTION_NAME) + " action running";
      send_feedback(progress_, feedback_msg);
    } else {
      std::string result_msg = std::string(ACTION_NAME) + " completed";  
      finish(true, 1.0, result_msg);

      progress_ = 0.0;
      std::cout << std::endl;
    }

    std::cout << "\r\e[K" << std::flush;
    std::cout << ACTION_NAME << " in progress... [" << std::min(100.0, progress_ * 100.0) << "%]  " << std::flush;
  }

  float progress_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveAction>();

  node->set_parameter(rclcpp::Parameter("action_name", ACTION_NAME));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
