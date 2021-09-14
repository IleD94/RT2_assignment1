/**
* \file position_service.cpp
* \brief This files is a service server used to set a random value, between a min M ad a max N, for each of field the random position response
* \author Ilenia D'Angelo
* \version 1.0
* \date 10/09/2021
*
*
* \details
*
* Subscribes to: <BR>
* ° None
*
* Publishes to: <BR>
* ° None
*
* Services : <BR>
* ° /position_server
*
* Description :
*
* This node takes as request two double value M (as min) and N (as max) to produce a
* random value to fill the response fields x,y (and theta that has a costante range between (-3.14,3.14)).
* In this way it generates a random position into a specific range
*/

#include "ros/ros.h"
#include "rt2_assignment1/RandomPosition.h"

/**
 * @brief Random number generator 
 * @param M  defines the minimum value of the range for generating a random number
 * @param N is the maximum value of the range from which it generates a random number
 
 * @retval a random value for the callback myrandom. It is a double
 *
 * This function generates a random number into an interval (M,N)
 */

double randMToN(double M, double N)
{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }

/**
 * @brief myrandom is the callback function of the server service /position_server
 * @param req  the request received from the client. It gets a min and a max of values for x and y 
 * @param res  the response returned to the client. It is a random position expressed in its coordinates (x,y,theta)
 * 
 * @retval the boolean True
 * 
 * This function calls inside the randMToN function, which returns a pseudo-random double in the range 
 * between 0 and RAND_MAX. Everytime a request is received this callback is executed 
 * 
 */

bool myrandom (rt2_assignment1::RandomPosition::Request &req, rt2_assignment1::RandomPosition::Response &res){
    res.x = randMToN(req.x_min, req.x_max);
    res.y = randMToN(req.y_min, req.y_max);
    res.theta = randMToN(-3.14, 3.14);
    return true;
}
/**
 * @brief  main function
 * 
 * @retval 0
 * 
 * This function initializes the service server /position_server,
 * the ros node (random_position_server) and waits for a request from the service client
 * 
 */

int main(int argc, char **argv)
{
   ros::init(argc, argv, "random_position_server");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/position_server", myrandom);
   ros::spin();

   return 0;
}
