
#include <memory>
#include <chrono>
#include <cinttypes>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "rt2_assignment1/srv/random_position.hpp"
#include "rt2_assignment1/srv/position.hpp"
#include "rt2_assignment1/srv/command.hpp"
#include "rclcpp_components/register_node_macro.hpp"


using std::placeholders::_1; // they will be replaced by the actual message during the run
using std::placeholders::_2;
using std::placeholders::_3;

namespace rt2_assignment1{

/*! StateMachine Class:
* The StateMachine Node is implemented as a class so that it can
* be structured as a component. It will be interfaced with the ROS node
* by means of the ros1_bridge package
*/
class StateMachine : public rclcpp::Node
{
public:
/** Initialisation of the  state_machine service and clients. 
	* Inside the Class constructor, there exists a /user_interface service server.
	* By means of the bind function, the callback is executed as soon as
	* the client makes a request. Then the FSM starts working. 
	* The two clients request respectively the /go_to_point service and the 
	* position server service.
	*/
  StateMachine(const rclcpp::NodeOptions & options)
  : Node("state_machine", options)
  {
    service_ = this->create_service<rt2_assignment1::srv::Command>(
      "/user_interface", std::bind(&StateMachine::handle_service, this, _1, _2, _3)); /**< server of type Command*/
    client_1 = this->create_client<rt2_assignment1::srv::Position>("/go_to_point"); /**< client_1 of type position  */
    while (!client_1->wait_for_service(std::chrono::seconds(1))){
     if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "client_1 interrupted while waiting for service to appear.");
      return;
    }
    
    }
    client_2 = this->create_client<rt2_assignment1::srv::RandomPosition>("/position_server"); /**< client_2 of type RandomPosition */
    while (!client_2->wait_for_service(std::chrono::seconds(1))){
     if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "client_2 interrupted while waiting for service to appear.");
      return;
    }
   
    }
  }

  

private:

  bool  start = false;
 
 /**
 * Documentation for the state_machine function.
 *
 * It requires a random position. Then it initialises the request 
 * of type position with these random values. By means of them,
 * it allows the robot to reach the expected Pose 
 * 
 */
  void state_machine(){

            if (this->start){
	   	 auto request_2 = std::make_shared<rt2_assignment1::srv::RandomPosition::Request>();
		
		 request_2->x_max = 5.0;
		 request_2->x_min = -5.0;
		 request_2->y_max = 5.0;
		 request_2->y_min = -5.0;
		 auto response_callback2 = [this](rclcpp::Client<rt2_assignment1::srv::RandomPosition>::SharedFuture future) {
		         auto request_1 = std::make_shared<rt2_assignment1::srv::Position::Request>();
		         this->response_2=future.get();
		         request_1->x = this->response_2->x;
		         request_1->y = this->response_2->y;
		         request_1->theta = this->response_2->theta;
		         std::cout << "My goal is: x= " << request_1->x << " y= " <<request_1->y << " theta = " <<request_1->theta << std::endl;
		         auto response_callback1 = [this](rclcpp::Client<rt2_assignment1::srv::Position>::SharedFuture future) {
		                                (void)future;
		                                std::cout << "Goal reached" << std::endl;
		                                this->state_machine();
            
                };
                
                auto result_1 = client_1->async_send_request(request_1, response_callback1);
           };
           auto future_result = client_2->async_send_request(request_2, response_callback2);

	}

  }
 
 /**Handle_service function to handle the actual service logic
 *
 * It takes as service request a string which is "start" if 
 * the user aims at activating the robot's behaviour. If so,
 * state_mach() function is called.
 * 
 *
 * @param request_header
 * @param request it retrieves a string
 * @param response it defines a boolean
 */
  void handle_service(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<rt2_assignment1::srv::Command::Request> request,
  const std::shared_ptr<rt2_assignment1::srv::Command::Response> response)
  {
  (void)request_header;
     if (request->command == "start"){
  
    	this->start = true;
    }
    else {
    	this->start = false;
    }
    response->ok=true;
    this->state_machine();
   }
  
  //variables declaration
  rclcpp::Service<rt2_assignment1::srv::Command>::SharedPtr service_;
  rclcpp::Client<rt2_assignment1::srv::Position>::SharedPtr client_1;
  rclcpp::Client<rt2_assignment1::srv::RandomPosition>::SharedPtr client_2; 
  std::shared_ptr<rt2_assignment1::srv::RandomPosition::Response> response_2;
  
};
}
RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::StateMachine)
