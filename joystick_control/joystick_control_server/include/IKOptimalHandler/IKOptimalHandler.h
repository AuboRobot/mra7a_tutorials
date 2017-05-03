#include <ros/ros.h>
#include <vector>
#include <std_msgs/Float32MultiArray.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Quaternion.h>

#include <math.h>
#include <cmath>

#define Color_Red "\33[0:31m\\" // Color Start
#define Color_end "\33[0m" // To flush out prev settings

//\033[22;30m - black
//\033[22;31m - red
//\033[22;32m - green
//\033[22;33m - brown
//\033[22;34m - blue
//\033[22;35m - magenta
//\033[22;36m - cyan
//\033[22;37m - gray
//\033[01;30m - dark gray
//\033[01;31m - light red
//\033[01;32m - light green
//\033[01;33m - yellow
//\033[01;34m - light blue
//\033[01;35m - light magenta
//\033[01;36m - light cyan
//\033[01;37m - white

#include <iostream>

class IKOptimalHandler {
public:
    IKOptimalHandler();

    void echo_hello();
    double computeSingleArmEuclideanDistance(const std::vector<double> &array_1,
                                             const std::vector<double> &array_2);
    double computeSingleArmEuclideanDistance(const std::vector<double> &array_1,
                                             const std::vector<double> &array_2,
                                             int dof);
    bool getSingleOptimalIKSolution(const robot_state::RobotStatePtr kinematic_state,
                                    const robot_state::JointModelGroup* joint_model_group,
                                    const geometry_msgs::Pose pose,
                                    const std::vector<double> initial_joint_value,
                                    std::vector<double> &single_optimal_joint_value,
                                    int dof=7,
                                    int ik_try_num=5);

    bool getOptimalIKSolution_twoArmRespectively(const robot_state::RobotStatePtr kinematic_state,
                                                 const robot_state::JointModelGroup* joint_model_group_left,
                                                 const robot_state::JointModelGroup* joint_model_group_right,
                                                 const geometry_msgs::Pose pose_left,
                                                 const geometry_msgs::Pose pose_right,
                                                 const std::vector<double> initial_joint_value,
                                                 std::vector<double> &optimal_joint_value);

    double computeEuclideanDistance(const std::vector<double> &array_1,
                                    const std::vector<double> &array_2);

    bool getOptimalIKSolution(const robot_state::RobotStatePtr kinematic_state,
                              const robot_state::JointModelGroup* joint_model_group_left,
                              const robot_state::JointModelGroup* joint_model_group_right,
                              const geometry_msgs::Pose pose_left,
                              const geometry_msgs::Pose pose_right,
                              const std::vector<double> initial_joint_value,
                              std::vector<double> &optimal_joint_value);


    std::vector<std::vector<double> > interpolate(const std::vector<double> &array_1,
                                                  const std::vector<double> &array_2,
                                                  int dof,
                                                  double step=0.004);

};
