#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <tf2/LinearMath/Quaternion.h>


geometry_msgs::PoseWithCovarianceStamped gMsg;
ros::Publisher pub;

float init_cov_arr[36] = {
    0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942
    };
/** function declarations **/
bool moveToGoal(double xGoal, double yGoal,double wGoal);

/** declare the coordinates of interest **/
double table_x[10] = {1.65, 2.00, 10.69};
double table_y[10] = {7.90, 4.05, 0.67};
double table_w[10] = {3.10, 3.10, 2.5};

bool goalReached = false;

int orderTable = 0;
bool getTable = false;


bool goalReached1 = false;
bool goalReached2 = false;

char choose(){
        char choice='q';
        std::cout<<"|-------------------------------|"<<std::endl;
        std::cout<<"|PRESSE A KEY:"<<std::endl;
        std::cout<<"|'0': Room235 "<<std::endl;
        std::cout<<"|'1': Room236 "<<std::endl;
        std::cout<<"|'2': Room237 "<<std::endl;
        std::cout<<"|'3': Room238 "<<std::endl;
        std::cout<<"|'4': Room239 "<<std::endl;
        std::cout<<"|'5': Room233 "<<std::endl;
        std::cout<<"|'q': Quit "<<std::endl;
        std::cout<<"|-------------------------------|"<<std::endl;
        std::cout<<"|WHERE TO GO?";
        std::cin>>choice;

        return choice;


}

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    gMsg.header.seq = msg->header.seq + 1;
    gMsg.header.frame_id = "map";

    // Error: tf: Lookup would require extrapolation into the past
    // https://answers.ros.org/question/188023/tf-lookup-would-require-extrapolation-into-the-past/
    gMsg.header.stamp = ros::Time(0);
    //ROS_INFO_STREAM("Get AMCL Pose");

    // Consider covariance and difference distance between amcl_pose and goal_pose, which second condition can occurs repetable action.

}

void tableCallBack(const std_msgs::Int32::ConstPtr& msg){
    getTable = true;
    orderTable = msg->data;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "map_navigation_node");
    ros::NodeHandle n;
    pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 100);
    ros::Publisher goal_pub = n.advertise<std_msgs::Int32>("point_arrival",100);

    ros::Subscriber amcl_pose_sub = n.subscribe("/amcl_pose", 100, poseCallback);
    ros::Subscriber table_sub = n.subscribe("/table_num", 100, tableCallBack);

    //sc.playWave(path_to_sounds+"short_buzzer.wav");
    //tell the action client that we want to spin a thread by default

    char choice = 'q';
    do{
            choice =choose();
            if (choice == '0'){
                    goalReached1 = moveToGoal(table_x[0], table_y[0], table_w[0]);
            }else if (choice == '1'){
                    goalReached1 = moveToGoal(table_x[1], table_y[1], table_w[1]);
            }
            if (choice!='q'){
                    if (goalReached1){
                            ROS_INFO("Congratulations!");

                    }else{
                            ROS_INFO("Hard Luck!");
                    }
            }
    ros::spinOnce();
    }while(choice !='q');
    return 0;
}

bool moveToGoal(double xGoal, double yGoal, double wGoal){

        //define a client for to send goal requests to the move_base server through a SimpleActionClient
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

        //wait for the action server to come up
        while(!ac.waitForServer(ros::Duration(5.0))){
                ROS_INFO("Waiting for the move_base action server to come up");
        }

        tf2::Quaternion q;
        q.setRPY(0,0,wGoal);

        move_base_msgs::MoveBaseGoal goal;

        //set up the frame parameters
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        /* moving towards the goal*/

        goal.target_pose.pose.position.x =  xGoal;
        goal.target_pose.pose.position.y =  yGoal;
        goal.target_pose.pose.position.z =  0.0;
        goal.target_pose.pose.orientation.x = 0.0;
        goal.target_pose.pose.orientation.y = 0.0;
        goal.target_pose.pose.orientation.z = q[2];
        goal.target_pose.pose.orientation.w = q[3];

        ROS_INFO("Sending goal location ...");
        ac.sendGoal(goal);

        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                ROS_INFO("You have reached the destination");
                ROS_INFO("Spread the particle");
                gMsg.pose.pose.position.x = xGoal;
                gMsg.pose.pose.position.y = yGoal;
                gMsg.pose.pose.position.z = 0.0;
                gMsg.pose.pose.orientation.x = 0.0;
                gMsg.pose.pose.orientation.y = 0.0;
                gMsg.pose.pose.orientation.z = q[2];
                gMsg.pose.pose.orientation.w = q[3];
                for(int i = 0; i<36; i++)
                {
                    gMsg.pose.covariance[i] = init_cov_arr[i];
                }
                pub.publish(gMsg);
                return true;
        }
        else{
                ROS_INFO("The robot failed to reach the destination");
                return false;
        }

}

