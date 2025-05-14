// // #include <memory>
// // #include "rclcpp/rclcpp.hpp"
// // #include "geometry_msgs/msg/point_stamped.hpp"
// // #include "nav2_msgs/action/navigate_to_pose.hpp"
// // #include "rclcpp_action/rclcpp_action.hpp"

// // using NavigateToPose = nav2_msgs::action::NavigateToPose;
// // using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

// // class PatrolBot : public rclcpp::Node {
// // public:
// //   PatrolBot() : Node("patrol_bot_node") {
// //     // Создание клиента для действия NavigateToPose
// //     client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    
// //     // Подписка на точки для патрулирования
// //     point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
// //       "/clicked_point", 10,
// //       std::bind(&PatrolBot::clickPointCallback, this, std::placeholders::_1));
    
// //     // Ожидание сервера действий
// //     while (!client_->wait_for_action_server(std::chrono::seconds(1))) {
// //       RCLCPP_INFO(get_logger(), "Ожидание сервера navigate_to_pose...");
// //     }
// //     RCLCPP_INFO(get_logger(), "Сервер navigate_to_pose доступен");
// //   }
  
// //   void sendGoal(int trace_queue) {
// //     if (!client_->wait_for_action_server(std::chrono::seconds(1))) {
// //       RCLCPP_ERROR(get_logger(), "Сервер действий недоступен");
// //       return;
// //     }
    
// //     auto goal_msg = NavigateToPose::Goal();
// //     goal_msg.pose.header.stamp = now();
// //     goal_msg.pose.header.frame_id = "map";
// //     goal_msg.pose.pose.position = goals_[trace_queue].pose.position;
// //     goal_msg.pose.pose.orientation = goals_[trace_queue].pose.orientation;
    
// //     auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
// //     send_goal_options.goal_response_callback =
// //       std::bind(&PatrolBot::goalResponseCallback, this, std::placeholders::_1);
// //     send_goal_options.feedback_callback =
// //       std::bind(&PatrolBot::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
// //     send_goal_options.result_callback =
// //       std::bind(&PatrolBot::resultCallback, this, std::placeholders::_1);
    
// //     client_->async_send_goal(goal_msg, send_goal_options);
// //   }
  
// // private:
// //   void clickPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr point) {
// //     if (count_ >= 5) {
// //       RCLCPP_WARN(get_logger(), "Достигнуто максимальное количество точек (5)");
// //       return;
// //     }
    
// //     RCLCPP_INFO(get_logger(), "Получена точка: x=%.2f, y=%.2f", point->point.x, point->point.y);
    
// //     // Сохранение точки как цели
// //     goals_[count_].header.frame_id = "map";
// //     goals_[count_].header.stamp = now();
// //     goals_[count_].pose.position = point->point;
// //     goals_[count_].pose.orientation.z = sin(M_PI/4);  // 90 градусов
// //     goals_[count_].pose.orientation.w = cos(M_PI/4);
    
// //     count_++;
    
// //     // Если это первая точка, начинаем патрулирование
// //     if (count_ == 1) {
// //       trace_queue_ = 0;
// //       sendGoal(trace_queue_);
// //     }
// //   }
  
// //   void goalResponseCallback(std::shared_future<GoalHandle::SharedPtr> future) {
// //     auto goal_handle = future.get();
// //     if (!goal_handle) {
// //       RCLCPP_ERROR(get_logger(), "Цель отклонена сервером");
// //     } else {
// //       RCLCPP_INFO(get_logger(), "Цель принята сервером");
// //     }
// //   }
  
// //   void feedbackCallback(
// //     GoalHandle::SharedPtr,
// //     const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
// //     // Можно добавить обработку обратной связи
// //   }
  
// //   void resultCallback(const GoalHandle::WrappedResult & result) {
// //     switch (result.code) {
// //       case rclcpp_action::ResultCode::SUCCEEDED:
// //         RCLCPP_INFO(get_logger(), "Цель достигнута");
// //         break;
// //       case rclcpp_action::ResultCode::ABORTED:
// //         RCLCPP_ERROR(get_logger(), "Цель прервана");
// //         return;
// //       case rclcpp_action::ResultCode::CANCELED:
// //         RCLCPP_WARN(get_logger(), "Цель отменена");
// //         return;
// //       default:
// //         RCLCPP_ERROR(get_logger(), "Неизвестный результат");
// //         return;
// //     }
    
// //     // Переход к следующей точке
// //     trace_queue_ = (trace_queue_ + 1) % count_;
// //     sendGoal(trace_queue_);
// //   }
  
// //   rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
// //   rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
// //   geometry_msgs::msg::PoseStamped goals_[5];
// //   int count_ = 0;
// //   int trace_queue_ = -1;
// // };

// // int main(int argc, char ** argv) {
// //   rclcpp::init(argc, argv);
// //   auto node = std::make_shared<PatrolBot>();
// //   rclcpp::spin(node);
// //   rclcpp::shutdown();
// //   return 0;
// // }


// #include <memory>
// #include <iostream>
// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/point_stamped.hpp"
// #include "nav2_msgs/action/navigate_to_pose.hpp"
// #include "rclcpp_action/rclcpp_action.hpp"

// using NavigateToPose = nav2_msgs::action::NavigateToPose;
// using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

// class PatrolBot : public rclcpp::Node
// {
// public:
//   PatrolBot() : Node("patrol_bot_node"), count_(0), trace_queue_(-1), done_(true)
//   {
//     // Инициализация клиента для NavigateToPose
//     this->client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
//       this,
//       "navigate_to_pose");

//     // Подписка на точки из Rviz
//     point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
//       "/clicked_point", 10,
//       std::bind(&PatrolBot::clickPointCallback, this, std::placeholders::_1));

//     // Таймер для проверки состояния и отправки новых целей
//     timer_ = this->create_wall_timer(
//       std::chrono::milliseconds(500),
//       std::bind(&PatrolBot::timerCallback, this));
//   }

// private:
//   void timerCallback()
//   {
//     if (done_ && count_ > 0)
//     {
//       sendGoal();
//     }
//   }

//   void sendGoal()
//   {
//     trace_queue_++;
//     if (trace_queue_ == 5 || trace_queue_ == count_)
//       trace_queue_ = 0;

//     if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
//       RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
//       return;
//     }

//     auto goal_msg = NavigateToPose::Goal();
//     goal_msg.pose = goals_[trace_queue_];
//     goal_msg.pose.header.stamp = this->now();
//     goal_msg.pose.header.frame_id = "map";

//     RCLCPP_INFO(this->get_logger(), "Sending goal");

//     auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
//     send_goal_options.goal_response_callback =
//       std::bind(&PatrolBot::goalResponseCallback, this, std::placeholders::_1);
//     send_goal_options.feedback_callback =
//       std::bind(&PatrolBot::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
//     send_goal_options.result_callback =
//       std::bind(&PatrolBot::resultCallback, this, std::placeholders::_1);

//     this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
//     done_ = false;
//   }

//   void goalResponseCallback(const GoalHandleNavigateToPose::SharedPtr & goal_handle)
//   {
//     if (!goal_handle) {
//       RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
//     } else {
//       RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
//     }
//   }

//   void feedbackCallback(
//     GoalHandleNavigateToPose::SharedPtr,
//     const std::shared_ptr<const NavigateToPose::Feedback> feedback)
//   {
//     // Можно добавить обработку фидбэка, если нужно
//   }

//   void resultCallback(const GoalHandleNavigateToPose::WrappedResult & result)
//   {
//     switch (result.code) {
//       case rclcpp_action::ResultCode::SUCCEEDED:
//         RCLCPP_INFO(this->get_logger(), "Goal was reached");
//         break;
//       case rclcpp_action::ResultCode::ABORTED:
//         RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
//         break;
//       case rclcpp_action::ResultCode::CANCELED:
//         RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
//         break;
//       default:
//         RCLCPP_ERROR(this->get_logger(), "Unknown result code");
//         break;
//     }
//     done_ = true;
//   }

//   void clickPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr point)
//   {
//     if (count_ < 5) count_++;

//     RCLCPP_INFO(this->get_logger(), "Get point: x=%f, y=%f", point->point.x, point->point.y);

//     geometry_msgs::msg::PoseStamped goal_pose;
//     goal_pose.pose.position = point->point;
    
//     // Ориентация - поворот на 90 градусов
//     double target_angle = M_PI/2;
//     goal_pose.pose.orientation.z = sin(target_angle/2);
//     goal_pose.pose.orientation.w = cos(target_angle/2);
//     goal_pose.header.frame_id = "map";
//     goal_pose.header.stamp = this->now();

//     goals_[count_ - 1] = goal_pose;

//     if (count_ == 1) done_ = true;
//   }

//   rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
//   rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
//   rclcpp::TimerBase::SharedPtr timer_;

//   geometry_msgs::msg::PoseStamped goals_[5];
//   int count_;
//   int trace_queue_;
//   bool done_;
// };

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<PatrolBot>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }












// #include <memory>
// #include <iostream>
// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/point_stamped.hpp"
// #include "nav2_msgs/action/navigate_to_pose.hpp"
// #include "rclcpp_action/rclcpp_action.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"

// using NavigateToPose = nav2_msgs::action::NavigateToPose;
// using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

// class PatrolBot : public rclcpp::Node
// {
// public:
//   PatrolBot() : Node("patrol_bot_node"), count_(0), trace_queue_(-1), done_(true)
//   {
//     goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
//     "/move_base_simple/goal", 10);

//     this->client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
//       this,
//       "navigate_to_pose");

//     point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
//       "/clicked_point", 10,
//       std::bind(&PatrolBot::clickPointCallback, this, std::placeholders::_1));

//     timer_ = this->create_wall_timer(
//       std::chrono::milliseconds(500),
//       std::bind(&PatrolBot::timerCallback, this));
//   }

// private:
//   void timerCallback()
//   {
//     if (done_ && count_ > 0)
//     {
//       sendGoal();
//     }
//   }

//   // void sendGoal()
//   // {
//   //   trace_queue_++;
//   //   if (trace_queue_ == 5 || trace_queue_ == count_)
//   //     trace_queue_ = 0;

//   //   if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
//   //     RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
//   //     return;
//   //   }

//   //   auto goal_msg = NavigateToPose::Goal();
//   //   goal_msg.pose = goals_[trace_queue_];
//   //   goal_msg.pose.header.stamp = this->now();
//   //   goal_msg.pose.header.frame_id = "map";

//   //   RCLCPP_INFO(this->get_logger(), "Sending goal");

//   //   auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
//   //   send_goal_options.goal_response_callback =
//   //     std::bind(&PatrolBot::goalResponseCallback, this, std::placeholders::_1);
//   //   send_goal_options.feedback_callback =
//   //     std::bind(&PatrolBot::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
//   //   send_goal_options.result_callback =
//   //     std::bind(&PatrolBot::resultCallback, this, std::placeholders::_1);

//   //   this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
//   //   done_ = false;
//   // }
//   void sendGoal() {
//     if (count_ == 0) return;
    
//     trace_queue_ = (trace_queue_ + 1) % count_;
//     auto msg = goals_[trace_queue_];
//     msg.header.stamp = this->now();
//     goal_pub_->publish(msg);
//     RCLCPP_INFO(this->get_logger(), "Published goal to move_base_simple/goal");
//     done_ = true;
//   }

//   void goalResponseCallback(std::shared_future<GoalHandleNavigateToPose::SharedPtr> future)
//   {
//     auto goal_handle = future.get();
//     if (!goal_handle) {
//       RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
//     } else {
//       RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
//     }
//   }

//   void feedbackCallback(
//     GoalHandleNavigateToPose::SharedPtr,
//     const std::shared_ptr<const NavigateToPose::Feedback> feedback)
//   {
//     // Feedback processing can be added here if needed
//     (void)feedback; // Silence unused parameter warning
//   }

//   void resultCallback(const GoalHandleNavigateToPose::WrappedResult & result)
//   {
//     switch (result.code) {
//       case rclcpp_action::ResultCode::SUCCEEDED:
//         RCLCPP_INFO(this->get_logger(), "Goal was reached");
//         break;
//       case rclcpp_action::ResultCode::ABORTED:
//         RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
//         break;
//       case rclcpp_action::ResultCode::CANCELED:
//         RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
//         break;
//       default:
//         RCLCPP_ERROR(this->get_logger(), "Unknown result code");
//         break;
//     }
//     done_ = true;
//   }

//   void clickPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr point)
//   {
//     if (count_ < 5) count_++;

//     RCLCPP_INFO(this->get_logger(), "Get point: x=%f, y=%f", point->point.x, point->point.y);

//     geometry_msgs::msg::PoseStamped goal_pose;
//     goal_pose.pose.position = point->point;
    
//     double target_angle = M_PI/2;
//     goal_pose.pose.orientation.z = sin(target_angle/2);
//     goal_pose.pose.orientation.w = cos(target_angle/2);
//     goal_pose.header.frame_id = "map";
//     goal_pose.header.stamp = this->now();

//     goals_[count_ - 1] = goal_pose;

//     if (count_ == 1) done_ = true;
//   }

//   rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;

//   rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
//   rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
//   rclcpp::TimerBase::SharedPtr timer_;

//   geometry_msgs::msg::PoseStamped goals_[5];
//   int count_;
//   int trace_queue_;
//   bool done_;
// };

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<PatrolBot>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }




// #include <memory>
// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/point_stamped.hpp>
// #include <nav2_msgs/action/navigate_to_pose.hpp>
// #include <rclcpp_action/rclcpp_action.hpp>

// using NavigateToPose = nav2_msgs::action::NavigateToPose;
// using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

// class PatrolBot : public rclcpp::Node {
// public:
//     PatrolBot() : Node("patrol_bot_node") {
//         client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
//             this,
//             "navigate_to_pose");

//         point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
//             "/clicked_point", 10,
//             std::bind(&PatrolBot::clickPointCallback, this, std::placeholders::_1));

//         timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(500),
//             [this]() {
//                 if (done_ && count_ > 0) {
//                     sendGoal();
//                 }
//             });
//     }

// private:
//     void sendGoal() {
//         if (!client_ptr_->wait_for_action_server(std::chrono::seconds(1))) {
//             RCLCPP_ERROR(this->get_logger(), "Action server not available");
//             return;
//         }

//         trace_queue_ = (trace_queue_ + 1) % count_;
//         auto goal_msg = NavigateToPose::Goal();
//         goal_msg.pose = goals_[trace_queue_];
//         goal_msg.pose.header.stamp = this->now();
//         goal_msg.pose.header.frame_id = "map";

//         RCLCPP_INFO(this->get_logger(), "Sending goal to [%.2f, %.2f]", 
//                    goal_msg.pose.pose.position.x,
//                    goal_msg.pose.pose.position.y);

//         auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        
//         // Исправленные callback-функции
//         send_goal_options.goal_response_callback =
//             [this](std::shared_future<GoalHandleNavigateToPose::SharedPtr> future) {
//                 auto goal_handle = future.get();
//                 if (!goal_handle) {
//                     RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
//                 } else {
//                     RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
//                 }
//             };
        
//         send_goal_options.feedback_callback =
//             [this](
//                 GoalHandleNavigateToPose::SharedPtr,
//                 const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
//                 // Обработка feedback при необходимости
//             };
        
//         send_goal_options.result_callback =
//             [this](const GoalHandleNavigateToPose::WrappedResult & result) {
//                 switch (result.code) {
//                     case rclcpp_action::ResultCode::SUCCEEDED:
//                         RCLCPP_INFO(this->get_logger(), "Goal reached!");
//                         break;
//                     case rclcpp_action::ResultCode::ABORTED:
//                         RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
//                         break;
//                     case rclcpp_action::ResultCode::CANCELED:
//                         RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
//                         break;
//                     default:
//                         RCLCPP_ERROR(this->get_logger(), "Unknown result code");
//                         break;
//                 }
//                 done_ = true;
//             };

//         client_ptr_->async_send_goal(goal_msg, send_goal_options);
//         done_ = false;
//     }

//     void clickPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr point) {
//         if (count_ >= 5) {
//             RCLCPP_WARN(this->get_logger(), "Maximum number of points (5) reached");
//             return;
//         }

//         RCLCPP_INFO(this->get_logger(), "Received point: [%.2f, %.2f]", 
//                    point->point.x, point->point.y);

//         geometry_msgs::msg::PoseStamped goal_pose;
//         goal_pose.pose.position = point->point;
        
//         double target_angle = M_PI/2;
//         goal_pose.pose.orientation.z = sin(target_angle/2);
//         goal_pose.pose.orientation.w = cos(target_angle/2);
//         goal_pose.header.frame_id = "map";
//         goal_pose.header.stamp = this->now();

//         goals_[count_] = goal_pose;
//         count_++;

//         if (count_ == 1) {
//             done_ = true;
//         }
//     }

//     rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
//     rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
//     rclcpp::TimerBase::SharedPtr timer_;

//     geometry_msgs::msg::PoseStamped goals_[5];
//     int count_ = 0;
//     int trace_queue_ = -1;
//     bool done_ = true;
// };

// int main(int argc, char ** argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<PatrolBot>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }




// #include <memory>
// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/point_stamped.hpp>
// #include <nav2_msgs/action/navigate_to_pose.hpp>
// #include <rclcpp_action/rclcpp_action.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// using NavigateToPose = nav2_msgs::action::NavigateToPose;
// using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

// class PatrolBot : public rclcpp::Node {
// public:
//   PatrolBot() : Node("patrol_bot_node"), count_(0), current_goal_index_(-1), goal_active_(false) {
//     // Инициализация publisher для простых целей
//     goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
//       "/move_base_simple/goal", 10);

//     // Подписка на точки из Rviz
//     point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
//       "/clicked_point", 10,
//       std::bind(&PatrolBot::clickPointCallback, this, std::placeholders::_1));

//     // Подписка на одометрию для проверки достижения цели
//     odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
//       "/odom", 10,
//       std::bind(&PatrolBot::odomCallback, this, std::placeholders::_1));

//     // Таймер для проверки состояния
//     timer_ = this->create_wall_timer(
//       std::chrono::milliseconds(500),
//       std::bind(&PatrolBot::timerCallback, this));

//     // Параметр для расстояния срабатывания
//     this->declare_parameter("goal_tolerance", 0.3);
//     goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
//   }

// private:
//   void timerCallback() {
//     if (!goal_active_ && count_ > 0) {
//       sendNextGoal();
//     }
//   }

//   void sendNextGoal() {
//     if (count_ == 0) return;

//     current_goal_index_ = (current_goal_index_ + 1) % count_;
//     auto msg = goals_[current_goal_index_];
//     msg.header.stamp = this->now();
//     goal_pub_->publish(msg);
    
//     RCLCPP_INFO(this->get_logger(), "Sending goal %d: [%.2f, %.2f]", 
//                current_goal_index_,
//                msg.pose.position.x,
//                msg.pose.position.y);
    
//     goal_active_ = true;
//     goal_start_time_ = this->now();
//   }

//   void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
//     if (!goal_active_ || count_ == 0) return;

//     // Проверяем расстояние до текущей цели
//     auto current_goal = goals_[current_goal_index_];
//     double dx = msg->pose.pose.position.x - current_goal.pose.position.x;
//     double dy = msg->pose.pose.position.y - current_goal.pose.position.y;
//     double distance = sqrt(dx*dx + dy*dy);

//     // Проверяем таймаут (10 секунд)
//     auto elapsed = this->now() - goal_start_time_;
//     if (elapsed.seconds() > 10.0) {
//       RCLCPP_WARN(this->get_logger(), "Goal timeout, moving to next");
//       goal_active_ = false;
//       return;
//     }

//     // Если достигли цели
//     if (distance < goal_tolerance_) {
//       RCLCPP_INFO(this->get_logger(), "Goal %d reached!", current_goal_index_);
//       goal_active_ = false;
//     }
//   }

//   void clickPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr point) {
//     if (count_ >= 5) {
//       RCLCPP_WARN(this->get_logger(), "Maximum number of points (5) reached");
//       return;
//     }

//     geometry_msgs::msg::PoseStamped goal_pose;
//     goal_pose.pose.position = point->point;
    
//     // Фиксированная ориентация (90 градусов)
//     tf2::Quaternion q;
//     q.setRPY(0, 0, M_PI/2);
//     goal_pose.pose.orientation = tf2::toMsg(q);
    
//     goal_pose.header.frame_id = "map";
//     goal_pose.header.stamp = this->now();

//     goals_[count_++] = goal_pose;

//     RCLCPP_INFO(this->get_logger(), "Added point %d: [%.2f, %.2f]", 
//                count_-1, point->point.x, point->point.y);
//   }

//   // ROS2 интерфейсы
//   rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
//   rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
//   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
//   rclcpp::TimerBase::SharedPtr timer_;

//   // Данные
//   geometry_msgs::msg::PoseStamped goals_[5];
//   int count_;
//   int current_goal_index_;
//   bool goal_active_;
//   rclcpp::Time goal_start_time_;
//   double goal_tolerance_;
// };

// int main(int argc, char ** argv) {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<PatrolBot>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }


#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

class PatrolBot : public rclcpp::Node {
public:
  PatrolBot() : Node("patrol_bot_node"), count_(0), current_goal_index_(-1), goal_active_(false) {
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/move_base_simple/goal", 10);

    point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/clicked_point", 10,
      std::bind(&PatrolBot::clickPointCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&PatrolBot::odomCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&PatrolBot::timerCallback, this));

    this->declare_parameter("goal_tolerance", 0.3);
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
  }

private:
  void timerCallback() {
    if (!goal_active_ && count_ > 0) {
      sendNextGoal();
    }
  }

  void sendNextGoal() {
    if (count_ == 0) return;

    current_goal_index_ = (current_goal_index_ + 1) % count_;
    auto msg = goals_[current_goal_index_];
    msg.header.stamp = this->now();
    goal_pub_->publish(msg);
    
    RCLCPP_INFO(this->get_logger(), "Sending goal %d: [%.2f, %.2f]", 
               current_goal_index_,
               msg.pose.position.x,
               msg.pose.position.y);
    
    goal_active_ = true;
    goal_start_time_ = this->now();
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!goal_active_ || count_ == 0) return;

    auto current_goal = goals_[current_goal_index_];
    double dx = msg->pose.pose.position.x - current_goal.pose.position.x;
    double dy = msg->pose.pose.position.y - current_goal.pose.position.y;
    double distance = sqrt(dx*dx + dy*dy);

    auto elapsed = this->now() - goal_start_time_;
    if (elapsed.seconds() > 10.0) {
      RCLCPP_WARN(this->get_logger(), "Goal timeout, moving to next");
      goal_active_ = false;
      return;
    }

    if (distance < goal_tolerance_) {
      RCLCPP_INFO(this->get_logger(), "Goal %d reached!", current_goal_index_);
      goal_active_ = false;
    }
  }

  void clickPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr point) {
    if (count_ >= 5) {
      RCLCPP_WARN(this->get_logger(), "Maximum number of points (5) reached");
      return;
    }

    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.pose.position = point->point;
    
    // Ручное задание ориентации (90 градусов)
    double target_angle = M_PI/2;
    goal_pose.pose.orientation.z = sin(target_angle/2);
    goal_pose.pose.orientation.w = cos(target_angle/2);
    
    goal_pose.header.frame_id = "map";
    goal_pose.header.stamp = this->now();

    goals_[count_++] = goal_pose;

    RCLCPP_INFO(this->get_logger(), "Added point %d: [%.2f, %.2f]", 
               count_-1, point->point.x, point->point.y);
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::PoseStamped goals_[5];
  int count_;
  int current_goal_index_;
  bool goal_active_;
  rclcpp::Time goal_start_time_;
  double goal_tolerance_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PatrolBot>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}