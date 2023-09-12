#include <cmath>
#include <iostream>
#include <fstream> // include this for saving .csv files
#include <string>
#include <unistd.h> // include this to use time delay
// #include "client.hpp" // include this for socket client in cpp. refer to https://github.com/OleguerCanal/cpp-python_socket


#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/gripper.h>


#include "examples_common.h"

// import ROS
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int16.h"
#include <sstream>
#include <math.h>

// To run the resulted executable in the build directory after "make" you need > ./vel_controller 192.16.0.2
#define PI 3.14159265
#define vec_size 3
//function to calculate dot product of two vectors
double dot_product(double vector_a[], double vector_b[]) {
   double product = 0;
   for (int i = 0; i < vec_size; i++)
   product = product + vector_a[i] * vector_b[i];
   return product;
}

float slip_value=0;
void slipCallback(const std_msgs::Float64& slip)
{
  slip_value = slip.data;
  // slip_value = 0; // added this to avoid activating control now!
}

float optimal_v_x = 0;
float optimal_v_y = 0;
float optimal_v_z = 0;
float optimal_w_x = 0;
float optimal_w_y = 0;
float optimal_w_z = 0;

void trajCallback(const std_msgs::Float64MultiArray& traj)
{
  optimal_v_x = traj.data[0];
  optimal_v_y = traj.data[1];
  optimal_v_z = traj.data[2];
  optimal_w_x = traj.data[3];
  optimal_w_y = traj.data[4];
  optimal_w_z = traj.data[5];
  // std::cout << "Optimal v_x: " << optimal_v_x  << std::endl;
  // std::cout << optimal_v_y  << std::endl;
  // std::cout << optimal_w_z  << std::endl;
  // std::cout << "______"  << std::endl;
}

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl; // The only argument given is robot ip
    return -1;
  }
  try {
    ros::init(argc, argv, "robotPub");
    ros::NodeHandle n;
    ros::Publisher robotPose_pub = n.advertise<std_msgs::Float64MultiArray>("robotPose", 1000);
    ros::Publisher slip_data_pub = n.advertise<std_msgs::Float64MultiArray>("slipData", 1000);
    ros::Subscriber sub = n.subscribe("slip_prediction", 1000, slipCallback);
    ros::Subscriber traj_sub = n.subscribe("optimal_traj", 1000, trajCallback);
    int count = 0;

    franka::Robot robot(argv[1]);
    franka::Gripper gripper(argv[1]);
    setDefaultBehavior(robot);
    // load the kinematics and dynamics model
    franka::Model model = robot.loadModel();


    // Create a vector of vectors which is a 2D matrix to push_back robot state data into.
    // To learn more about "The C++ Standard Template Library (STL)" refer to https://www.geeksforgeeks.org/the-c-standard-template-library-stl/
    std::vector< std::vector<double> > robot_pose;
    std::vector< std::vector<double> > joint_position;
    std::vector< std::vector<double> > joint_velocity;

    bool slip_flag_old = false;
    bool slip_flag_new = false;
    bool task_ending_phase = false;
    double slip_onset_time = 0;
    double slip_onset_v_x = 0;
    double slip_onset_v_y = 0;
    double slip_onset_delta_x = 0;
    double slip_onset_delta_y = 0;
    double remained_delta_x = 0;
    double T_slip = 0;
    double acc_slip = 0;
    double task_ending_v_x = 0;
    double task_ending_acc = 0;
    double task_ending_time = 0;
    
    // this is for cubic trajectory
    double T = 1.69; // T can change between 1.2  and 3.2
    double V_MAX_x = 8 / pow(T, 5); // max of V_MAX_x = 0.4 for translation_x = 0.35 m and translation_y = 0.5
    // double V_MAX_x = 14 / (5 * pow(T, 4) + 40 * pow(T, 2) + 16);
    // double V_MAX_x = 50 * 0.14 / (pow(T, 3) + 2*T);
    std::cout << V_MAX_x << std::endl;

    double translation_x = 0.35;
    double translation_y = 0.5;
    double V_MAX_y = (translation_y/translation_x) * V_MAX_x;
    double V_MAX_z = V_MAX_x / 0.21;
    
    double time = 0.0;
    double acc = 0;
    double v_x, v_y;
      // double v_z = 0;
    double w_x = 0;
    double w_y = 0;
    double w_z = 0;
    double v_z = 0;
    double v_x_0, v_y_0, w_z_0;

    // W_MAX_z = rotation_z / (2*T);
    std::string filename = "/home/kiyanoush/Cpp_ws/src/robotTest2/data/RT_test/ScienceRobotic/test/robot_pose_cpp.csv";
    std::vector<double> v_x_save;
    std::vector<double> v_y_save;
    std::vector<double> v_z_save;
    std::vector<double> w_x_save;
    std::vector<double> w_y_save;
    std::vector<double> w_z_save;


    // move the robot to a suitable home joint configuration
    std::array<double, 7> q_goal_1 = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator_1(0.4, q_goal_1);
    robot.control(motion_generator_1);

    franka::GripperState gripper_state = gripper.readOnce();
    gripper.move(gripper_state.max_width-0.005, 0.4); // first arg=width, second=speed

    // move the robot to the pregrasp joint configuration
    // std::array<double, 7> q_goal_2 = {{0.46594042874768027, 0.5736332212894555, -0.121005812794016, -1.427936394524156, 0.08442840939097934, 2.0023333249621924, 0.15335033177618285}};
    // theBest new
    // std::array<double, 7> q_goal_2 = {{0.4651429217714512, 0.5451831926466233, -0.1241411501164842, -1.3537963933302153, 0.0792001422961141, 1.89436454651777, 0.1592959655348413}};
    // new pose
    std::array<double, 7> q_goal_2 = {{-0.22654651400246525, 0.5248952914274307, -0.2769008542163717, -1.3916030072217795, 0.1707438929126655, 1.9306446890400737, 1.1513067523259584}};
    MotionGenerator motion_generator_2(0.4, q_goal_2);
    robot.control(motion_generator_2);

    // theBest old
    // std::array<double, 7> q_goal_3 = {{0.4622160324939316, 0.6747957952584068, -0.11265158838587289, -1.5016724266821633, 0.09726375189268904, 2.169628276489114, 0.1445644477809302}};
    // theBest new
    // std::array<double, 7> q_goal_3 = {{0.46573687304994976, 0.607229224859898, -0.1190305153536798, -1.462855484084864, 0.08778084733934619, 2.0633288509728755, 0.1533736983142435}};
    // snail killer bottle
    // std::array<double, 7> q_goal_3 = {{0.4302862460415172, 0.6226217721051964, -0.0733636891335603, -1.484176657599378, 0.062308337121343794, 2.099884310483682, 0.16476227917879832}};
    // Chilli can
    // std::array<double, 7> q_goal_3 = {{-0.19898747290664703, 0.8878438630912647, 0.754183526641446, -1.4858119658601288, -0.6488857671072086, 2.097742687524623, 0.5420324414059704}};
    // new pose
    std::array<double, 7> q_goal_3 = {{-0.23061297224329372, 0.6195902397278347, -0.2641631261615072, -1.4645645853451845, 0.17707194595405096, 2.0806598639301286, 1.1772814677869097}};
    MotionGenerator motion_generator_3(0.4, q_goal_3);
    robot.control(motion_generator_3);

    // Grasp the object.
    // gripper.move(0.0448, 0.5); // original for data collection
    // theBest new
    gripper.move(0.044, 0.5);
    // snail killer bottle
    // gripper.move(0.065, 0.5); // 0.066 original
    //  Thins
    // gripper.move(0.042, 0.5);
    // Rice
    // gripper.move(0.046, 0.5);
    // Carrs
    // gripper.move(0.057, 0.5);
    // Monster
    // gripper.move(0.062, 0.5); // actual value for testing is 0.06

    
   robot.setCollisionBehavior({{1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0}},
                               {{1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0}},
                               {{1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0}},
                               {{1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0}});
                               
    // move the robot to the pregrasp joint configuration
    // std::array<double, 7> q_goal_4 = {{0.46594042874768027, 0.5736332212894555, -0.121005812794016, -1.427936394524156, 0.08442840939097934, 2.0023333249621924, 0.15335033177618285}};
    // theBest new
    // std::array<double, 7> q_goal_4 = {{0.4651429217714512, 0.5451831926466233, -0.1241411501164842, -1.3537963933302153, 0.0792001422961141, 1.89436454651777, 0.1592959655348413}};
    // new pose
    std::array<double, 7> q_goal_4 = {{-0.22654651400246525, 0.5248952914274307, -0.2769008542163717, -1.3916030072217795, 0.1707438929126655, 1.9306446890400737, 1.1513067523259584}};
    MotionGenerator motion_generator_4(0.05, q_goal_4);
    robot.control(motion_generator_4);
    
    // socket_communication::Client client("127.0.0.1", 5003);
    // client.Send("Pre-grasp!");
    unsigned int microsecond = 1000000;
    usleep(0.3 * microsecond);//sleeps for 3 second

    // wait a bit before starting the linear motion in -X direction
    usleep(1000000);
    // client.Send("Object_Grasped!");

    // Set the joint impedance.
    robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  
   
    franka::RobotState grasp_state = robot.readOnce();
    double grasp_x = grasp_state.O_T_EE[12];
    double grasp_z = grasp_state.O_T_EE[14];

    // Here the Cartesian velocity control loop starts. It has a lambda definition which inholds the control callback function in itself.
    robot.control([=, &time, &model](const franka::RobotState& rob,
                             franka::Duration period) mutable -> franka::CartesianVelocities {
      
      std::array<double, 42> jacobian_array =
          model.zeroJacobian(franka::Frame::kEndEffector, rob);
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(rob.q.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(rob.dq.data());

      Eigen::VectorXd task_space_speed(6);
      task_space_speed = jacobian * dq;
      
      time += period.toSec();
  
      if (slip_value == 1){
        slip_flag_new = true;
      }
      // This if condition creates the cartesian trapezoidal velocity profile in which the duration of all three parts are T.
      if (time < 1){
        v_x = -0.02 * time;
        // v_y = (translation_y/translation_x) * v_x;
        v_y = -(translation_y/translation_x) * v_x;
        w_z = 0.01 * time;
        v_x_0 = v_x;
        v_y_0 = v_y;
        w_z_0 = w_z;
      }

      // this is for cubic trajectory
      if (slip_flag_new == false){
      if (time >= 1 && time < (1+T/2-T/6)){
        v_x = -V_MAX_x * (-pow((time - T/2 - 1), 4) + pow(T, 4)/16) - 0.02;
        v_y = -(translation_y/translation_x) * v_x;
        w_z = -(v_x + 0.02) * 3.5;
      } else if (time >=(1+T/2-T/6+T/10) && time < 1+T){
        v_x = -V_MAX_x * (-pow((time+T/4 - T/2 - 1), 4) + pow(T, 4)/16) - 0.02;
        v_y = -(translation_y/translation_x) * v_x;
        w_z = -(v_x + 0.02) * 3.5;
      }
      } else if ((slip_flag_new == true) && (time < T_slip)){
        if (optimal_v_x != 0){
          v_x = v_x + (optimal_v_x - v_x)/400;
          v_y = v_y + (optimal_v_y - v_y)/400;
          v_z = v_z + (optimal_v_z - v_z)/300;
          // w_x = w_x + (optimal_w_x - w_x) / 300;
          // w_y = w_y + (optimal_w_y - w_y) / 300;
          w_z = w_z + (optimal_w_z - w_z) / 300;
          // std::cout << "v_x: " << v_x << std::endl;
        } else if (optimal_v_x == 0){
          v_x = v_x - 0.001;
          v_y = v_y - 0.001;
          w_z = w_z - 0.001;
        }       
      }

      if (slip_flag_new == false){
      if ((time >= 1) && (time < (1+T/2))){
        v_z = V_MAX_z * (-pow((time - T/4 - 1), 4) + pow(T, 4)/256);
        // std::cout << "1: " << v_z << std::endl;
      } else if((time > (1+T/2)) && (time < (T+1))){
        v_z = -V_MAX_z * (-pow((time - T/4 - T/2 - 1), 4) + pow(T, 4)/256);
        // std::cout << "2: " << v_z << std::endl;
      }
      }

      std::vector<double> EE_pose_vec = {rob.O_T_EE[0], rob.O_T_EE[1], rob.O_T_EE[2], rob.O_T_EE[3], rob.O_T_EE[4], rob.O_T_EE[5], rob.O_T_EE[6],
                                  rob.O_T_EE[7], rob.O_T_EE[8], rob.O_T_EE[9], rob.O_T_EE[10], rob.O_T_EE[11], rob.O_T_EE[12], rob.O_T_EE[13],
                                rob.O_T_EE[14], rob.O_T_EE[15]};
      std_msgs::Float64MultiArray robot_pose_msg;
      robot_pose_msg.data.clear();
      robot_pose_msg.data.resize(23);
      EE_pose_vec.push_back(task_space_speed[0]);
      EE_pose_vec.push_back(task_space_speed[1]);
      EE_pose_vec.push_back(task_space_speed[2]);
      EE_pose_vec.push_back(task_space_speed[3]);
      EE_pose_vec.push_back(task_space_speed[4]);
      EE_pose_vec.push_back(task_space_speed[5]);
      EE_pose_vec.push_back(0.0); //this is the flag to continue/end subscribing fo python script subscriber
      robot_pose_msg.data = EE_pose_vec;
      robotPose_pub.publish(robot_pose_msg);

      if (slip_flag_new != slip_flag_old){
        slip_onset_time = time;
        slip_onset_v_x = v_x;
        slip_onset_v_y = v_y;

        slip_onset_delta_x = 0.5 * (slip_onset_time - 1) * (v_x - 0.02) - 0.5 * 0.02;
        slip_onset_delta_y = 0.5 * (slip_onset_time - 1) * (v_y - (translation_y/translation_x)*0.02) - 0.5 * (translation_y/translation_x)*0.02;

        remained_delta_x = translation_x + slip_onset_delta_x + 0.2 * slip_onset_v_x; // third term is because of 1000 -> 60 hz delay
        T_slip = slip_onset_time - 2*remained_delta_x/slip_onset_v_x;
        acc_slip = slip_onset_v_x / (T_slip - slip_onset_time);
        // std::cout << "T_slip: " << T_slip << std::endl;
        std::cout << "time: " << time << std::endl;
        std::cout << "slip_onset_v_x: " << slip_onset_v_x << std::endl;
      }

      std::vector<double> slip_onset_data = {slip_onset_v_x, (T_slip - slip_onset_time)};
      std_msgs::Float64MultiArray slip_msg;
      slip_msg.data.clear();
      slip_msg.data.resize(2);
      slip_msg.data = slip_onset_data;
      slip_data_pub.publish(slip_msg);

      
      if (slip_flag_new == true){
          sub.shutdown();
      }

      v_x_save.push_back(v_x);
      v_y_save.push_back(v_y);
      v_z_save.push_back(v_z);
      w_x_save.push_back(w_x);
      w_y_save.push_back(w_y);
      w_z_save.push_back(w_z);
    
      slip_flag_old = slip_flag_new;
      ros::spinOnce();
      ++count;

      // std::cout << w_z << std::endl;
      
      franka::CartesianVelocities output = {{v_x, v_y, v_z, w_x, w_y, 0}}; //Desired Cartesian velocity w.r.t. O-frame (base) {dx, dy, dz in [m/s], omegax, omegay, omegaz in [rad/s]
     
      if ((slip_onset_time==0 && time > T + 1-T/4) || ((rob.O_T_EE[12] - grasp_x) < -0.42)) {
        // std::cout << "task completion time: " << time << std::endl;
        int b = 0;
        unsigned int milisecond = 500;
        std_msgs::Float64MultiArray robot_pose_msg;
        robot_pose_msg.data.clear();
        robot_pose_msg.data.resize(23);
        //EE_pose_vec.push_back(1.0);
        int n = EE_pose_vec.size();
        EE_pose_vec[n-1] = 1.0;
        robot_pose_msg.data = EE_pose_vec ;
        
        while(b<5000){  // 5 seconds - ish   
          usleep(milisecond);
          b++;
          robotPose_pub.publish(robot_pose_msg);
          ros::spinOnce();
        }

        std::ofstream RobotPoseCSV;   
        RobotPoseCSV.open(filename);
        RobotPoseCSV << "v_x" << "," << "v_y" << "," << "v_z" << "," << "w_x" << "," << "w_y" << "," << "w_z" << std::endl;
        for (int i = 0; i < v_x_save.size(); i++)
        {
          // std::cout << "in the loop" << std::endl;
          RobotPoseCSV << v_x_save[i] << "," << v_y_save[i] << "," << v_z_save[i] << "," << w_x_save[i] << "," << w_y_save[i] << "," << w_z_save[i] << std::endl;
        }
        return franka::MotionFinished(output);
      }
      return output;
    });

  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
