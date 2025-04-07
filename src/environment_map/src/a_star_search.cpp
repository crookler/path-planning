#include "rclcpp/rclcpp.hpp"
#include "path_interfaces/action/a_search_query.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace environment_mapping {

class AStarSearch : public rclcpp::Node {
public:
    using ASearchQuery = path_interfaces::action::ASearchQuery;
    using GoalHandlePath = rclcpp_action::ServerGoalHandle<ASearchQuery>;

    explicit AStarSearch(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("a_star_action_server", options) {
        this->action_server = rclcpp_action::create_server<ASearchQuery>(
            this,
            "a_star_search",
            std::bind(&AStarSearch::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&AStarSearch::handle_cancel, this, std::placeholders::_1),
            std::bind(&AStarSearch::handle_accepted, this, std::placeholders::_1));
    }

private:
    rclcpp_action::Server<ASearchQuery>::SharedPtr action_server;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuiud, 
        std::shared_ptr<const ASearchQuery::Goal> goal) {
            void(uuid); // Unused
            RCLCPP_INFO(this->get_logger(), "Navigation goal has ID: %d", goal->query.goal.header.seq);
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; // Just accept all goals for now (may add in some sugar to detect if goal is within map bounds)
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandlePath> goal_handle) {
            RCLCPP_INFO(this->get_logger(), "Cancel goal");
            return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(
        cost std::shared_ptr<GoalHandlePath> goal_handle) {
            std::thread{std::bind(&AStarSearch::execute, this, std::placeholders::_1), goal_handle}.detach() // Allow thread to run in parallel
    }

    void execute(
        const std::shared_ptr<GoalHandlePath> goal_handle) {
            //TODO: A* search
    }
};

}