#include "IKOptimalHandler/IKOptimalHandler.h"


IKOptimalHandler::IKOptimalHandler(){}

void IKOptimalHandler::echo_hello(){
    std::cout<<"hello world"<<std::endl;
}

double IKOptimalHandler::computeSingleArmEuclideanDistance(const std::vector<double> &array_1,
                                                           const std::vector<double> &array_2)
{
    double weight[7] = {1.0, 0.86, 0.68, 0.55, 0.39, 0.3, 0.18};
    double distance = 0.0;

    for(int i=0; i< (int) array_1.size(); i++)
    {
        distance += weight[i]*weight[i]*(array_1[i] - array_2[i])*(array_1[i] - array_2[i]);
    }
    return sqrt(distance);
}

double IKOptimalHandler::computeSingleArmEuclideanDistance(const std::vector<double> &array_1,
                                                           const std::vector<double> &array_2,
                                                           int dof)
{
    if(dof == 6) {
        double weight[6] = {1.0, 0.86, 0.68, 0.3, 0.2, 0.1};
        double distance = 0.0;
        for(int i=0; i< (int) array_1.size(); i++)
        {
            distance += weight[i]*weight[i]*(array_1[i] - array_2[i])*(array_1[i] - array_2[i]);
        }
        return sqrt(distance);
    } else {
        ROS_ERROR("IKOptimalHandler::computeSingleArmEuclideanDistance:dof must=6");
    }

}

bool IKOptimalHandler::getSingleOptimalIKSolution(const robot_state::RobotStatePtr kinematic_state,
                                const robot_state::JointModelGroup* joint_model_group,
                                const geometry_msgs::Pose pose,
                                const std::vector<double> initial_joint_value,
                                std::vector<double> &single_optimal_joint_value,
                                int dof,
                                int ik_try_num){
    double min_distance = 1e6;
    int min_index = -1;
    std::vector<double> joint_values;
    std::vector<double> joint_values_nearby(dof,0.05);

    single_optimal_joint_value.resize(dof);
    Eigen::VectorXd  init_values(dof);

    for(size_t count = 0;count < dof;count++){
        init_values[count] = initial_joint_value[count];
    }

    //IK NUM,You can modify this num;
    for(size_t ik_num = 0;ik_num<ik_try_num;ik_num++){

        bool found_ik = false;

        kinematic_state->setJointGroupPositions(joint_model_group,init_values);

        if(ik_num > 0){

            kinematic_state->setToRandomPositionsNearBy(joint_model_group,*kinematic_state,joint_values_nearby);

        }

        found_ik  = kinematic_state->setFromIK(joint_model_group, pose);

        if (found_ik)
        {

            kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

            std::vector<double> joint_values_tmp;
            joint_values_tmp.resize(dof);


            for(size_t joint_num = 0 ;joint_num<joint_values.size();joint_num++){
                joint_values_tmp[joint_num]=joint_values[joint_num];

            }

            double tmp_distance = computeSingleArmEuclideanDistance(joint_values_tmp, initial_joint_value);

            if(tmp_distance < min_distance)
            {
                min_distance = tmp_distance;
                min_index = ik_num;
                single_optimal_joint_value = joint_values_tmp;
            }

        }
    }

    std::cout << "min_index-> "<< min_index << std::endl;
    std::cout << "min_distance-> "<< min_distance << std::endl;

    if(min_index == -1)
    {
        single_optimal_joint_value.resize(dof);
        ROS_ERROR(" Could not find any IK solution! Get Optimal Solution Failed!");
        return false;
    }
    return true;

}



bool IKOptimalHandler::getOptimalIKSolution_twoArmRespectively(const robot_state::RobotStatePtr kinematic_state,
                                             const robot_state::JointModelGroup* joint_model_group_left,
                                             const robot_state::JointModelGroup* joint_model_group_right,
                                             const geometry_msgs::Pose pose_left,
                                             const geometry_msgs::Pose pose_right,
                                             const std::vector<double> initial_joint_value,
                                             std::vector<double> &optimal_joint_value)
{
    std::vector<double> initial_joint_value_left,initial_joint_value_right;
    std::vector<double> optimal_joint_value_left,optimal_joint_value_right;

    //joint_model_group_left->getJointModelNames().size();
    optimal_joint_value.resize(14);
    initial_joint_value_left.resize(7);
    initial_joint_value_right.resize(7);

    for(size_t count = 0; count < 7 ; count++)
    {
        initial_joint_value_left[count] = initial_joint_value[count];
        initial_joint_value_right[count] = initial_joint_value[count+7];

    }
    bool found_ik_left = getSingleOptimalIKSolution(kinematic_state,joint_model_group_left,
                                                    pose_left,initial_joint_value_left,
                                                    optimal_joint_value_left);
    if(!found_ik_left)
    {
        ROS_ERROR("Could not find left arm IK, get optimal IK failed!");
        return false;

    }
    bool found_ik_right = getSingleOptimalIKSolution(kinematic_state,joint_model_group_right,
                                                     pose_right,initial_joint_value_right,
                                                     optimal_joint_value_right);

    if(!found_ik_right)
    {
        ROS_ERROR("Could not find right arm IK, get optimal IK failed!");
        return false;

    }
    for(size_t count = 0;count < 7 ; count++)
    {
        optimal_joint_value[count] = optimal_joint_value_left[count];
        optimal_joint_value[count+7] = optimal_joint_value_right[count];

    }
}

double IKOptimalHandler::computeEuclideanDistance(const std::vector<double> &array_1,
                                                  const std::vector<double> &array_2)
{
    double weight[14] = {1.0, 0.86, 0.68, 0.55, 0.39, 0.3, 0.18,1.0, 0.86, 0.68, 0.55, 0.39, 0.3, 0.18};
    double distance = 0.0;

    for(int i=0; i< (int) array_1.size(); i++)
    {
        distance += weight[i]*weight[i]*(array_1[i] - array_2[i])*(array_1[i] - array_2[i]);

    }

    return sqrt(distance);
}

bool IKOptimalHandler::getOptimalIKSolution(const robot_state::RobotStatePtr kinematic_state,
                          const robot_state::JointModelGroup* joint_model_group_left,
                          const robot_state::JointModelGroup* joint_model_group_right,
                          const geometry_msgs::Pose pose_left,
                          const geometry_msgs::Pose pose_right,
                          const std::vector<double> initial_joint_value,
                          std::vector<double> &optimal_joint_value)
{
    double min_distance = 1e6;
    int min_index = -1;
    std::vector<double> joint_values_right;
    std::vector<double> joint_values_left;
    std::vector<double> joint_values_nearby(7,0.05);

    optimal_joint_value.resize(14);
    Eigen::VectorXd  left_values(7),right_values(7);

    for(size_t count = 0;count < 7;count++){
        left_values[count] = initial_joint_value[count];
        right_values[count] = initial_joint_value[count+7];

    }

    for(size_t ik_num = 0;ik_num<200;ik_num++){

        bool found_ik_left = false;
        bool found_ik_right = false;

        kinematic_state->setJointGroupPositions(joint_model_group_left,left_values);
        kinematic_state->setJointGroupPositions(joint_model_group_right,right_values);

        if(ik_num > 0){

            kinematic_state->setToRandomPositionsNearBy(joint_model_group_left,*kinematic_state,joint_values_nearby);
            kinematic_state->setToRandomPositionsNearBy(joint_model_group_right,*kinematic_state,joint_values_nearby);
        }

        found_ik_left  = kinematic_state->setFromIK(joint_model_group_left, pose_left);
        found_ik_right = kinematic_state->setFromIK(joint_model_group_right, pose_right);

        if(!found_ik_left){

            ROS_ERROR("found_ik_left not found");
        }
        if(!found_ik_right){

            ROS_ERROR("found_ik_right not found");
        }

        if (found_ik_left && found_ik_right)
        {

            kinematic_state->copyJointGroupPositions(joint_model_group_right, joint_values_right);
            kinematic_state->copyJointGroupPositions(joint_model_group_left,  joint_values_left);

            std::vector<double> joint_values_tmp;
            joint_values_tmp.resize(14);


            for(size_t joint_num = 0 ;joint_num<joint_values_right.size();joint_num++){
                joint_values_tmp[joint_num]=joint_values_left[joint_num];
                joint_values_tmp[joint_num+7]=joint_values_right[joint_num];

            }            

            double tmp_distance = computeEuclideanDistance(joint_values_tmp, initial_joint_value);

            if(tmp_distance < min_distance)
            {
                min_distance = tmp_distance;
                min_index = ik_num;
                optimal_joint_value = joint_values_tmp;
            }

        }
    }

    std::cout << "min_index-> "<< min_index << std::endl;
    std::cout << "min_distance-> "<< min_distance << std::endl;

    if(min_index == -1)
    {
        optimal_joint_value.resize(14);
        ROS_ERROR(" Could not find any IK solution! Get Optimal Solution Failed!");
        return false;
    }
    return true;

}


/**
 * @brief interpolate
 * @param array_1
 * @param array_2
 * @return after a new trajectory(interpolate between array_1 and array_2)
 */
std::vector<std::vector<double> > IKOptimalHandler::interpolate(const std::vector<double> &array_1,
                                              const std::vector<double> &array_2,
                                              int dof,
                                              double step)
{

    /*1.compute the distance between the two points*///两个路点间的距离＝各关节在这两点间的距离平方和再开根
    double distance_sum = 0;
    for(int i=0; i<dof; i++) {
        //ROS_INFO("\033[01;34m i:%d %s",i,Color_end);
        distance_sum += pow(array_2[i]-array_1[i],2);
    }
    distance_sum = sqrt(distance_sum);
    ROS_INFO("\033[01;34m distance_sum:%lf %s",distance_sum,Color_end);
    /*2.compute the interpolate count*///(关节维数总长除以关节维数步长)-1-->插点个数
    unsigned int interpolate_count = distance_sum/sqrt(dof*pow(0.004,2));
    ROS_INFO("\033[01;34m interpolate_count:%d %s",interpolate_count,Color_end);
    /*interpolate*///先把起始点插入，然后循环插：起始点+步长×点数
    std::vector<std::vector<double> > trj_points;
    std::vector<double> joints;
    joints.resize(dof);

    trj_points.push_back(array_1);
    for (int i=0; i<interpolate_count; i++) {
        joints.clear();
        for (int j=0; j<dof; j++) {
            joints.push_back(array_1[j]+step*(i+1));
        }
        trj_points.push_back(joints);
    }
    trj_points.push_back(array_2);

    return trj_points;
}



