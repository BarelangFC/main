#include <chrono>
#include <memory>
#include <string>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <cmath>
#include <random>
#include <zmqpp/zmqpp.hpp>
#include <nlohmann/json.hpp>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

#include "rclcpp/rclcpp.hpp"
#include "bfc_msgs/msg/button.hpp"
#include "bfc_msgs/msg/head_movement.hpp"
#include "bfc_msgs/msg/coordination.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"

#include "darknet_ros_msgs/msg/bounding_boxes.hpp"
#include "darknet_ros_msgs/msg/object_count.hpp"
#include <functional>

#include <fstream>
#include <sstream>
#include <vector>

#define PI 3.1415926535897932384626433832795
using namespace std::chrono_literals;
using namespace BT;
using namespace std;

class main_strategy : public rclcpp::Node
{
public:
    main_strategy()
        : Node("main_strategy", rclcpp::NodeOptions().use_intra_process_comms(true)), socket_(context_, zmqpp::socket_type::reply)
    {
        socket_.bind("tcp://0.0.0.0:5555");
        declareParameters();
        getParameters();

        robotNumber = this->get_parameter("robotNumber").as_int();

        button_ = this->create_subscription<bfc_msgs::msg::Button>(
            "button", 1,
            std::bind(&main_strategy::readButton, this, std::placeholders::_1));
        imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu", 1,
            std::bind(&main_strategy::readImu, this, std::placeholders::_1));
        gameControllerSubscription_ = this->create_subscription<std_msgs::msg::Int64MultiArray>(
            "game_controller", 1,
            std::bind(&main_strategy::readGameControllerData, this, std::placeholders::_1));

        trackbarSubscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "trackbar", 1,
            std::bind(&main_strategy::readTrackbar, this, std::placeholders::_1));

        voltage_n_odometry = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "voltodom", 1,
            std::bind(&main_strategy::readVoltageAndOdom, this, std::placeholders::_1));

        GridSub_ = this->create_subscription<std_msgs::msg::Int32>(
            "grid", 10,
            std::bind(&main_strategy::readGrid, this, std::placeholders::_1));

        subscriber_darknet = this->create_subscription<darknet_ros_msgs::msg::BoundingBoxes>(
            "darknet_ros/bounding_boxes", 1,
            std::bind(&main_strategy::callbackBoundingBox, this, std::placeholders::_1));
        
        subscriber_object_count = this->create_subscription<darknet_ros_msgs::msg::ObjectCount>(
            "darknet_ros/found_object", 1,
            std::bind(&main_strategy::callbackFoundObject, this, std::placeholders::_1));

        TargetPoseSub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            std::bind(&main_strategy::callbackTargetPose, this, std::placeholders::_1));

        object_distance = this->create_subscription<std_msgs::msg::Float32>(
            "object_distance", 10,
            std::bind(&main_strategy::callbackObjectDistance, this, std::placeholders::_1));

        ball_distance = this->create_subscription<std_msgs::msg::Float32>(
            "ball_distance", 1,
            std::bind(&main_strategy::callbackBallDistance, this, std::placeholders::_1));

        path_finding_subscription_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            "path_finding", 10,
            std::bind(&main_strategy::callbackPathFinding, this, std::placeholders::_1));

        keyboard_teleop = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&main_strategy::callbackTeleop, this, std::placeholders::_1));

        ball_pose_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "ball_pose", 1,
            std::bind(&main_strategy::callbackBallPose, this, std::placeholders::_1));

        camera_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "camera_odom", 1,
            std::bind(&main_strategy::callbackCameraOdom, this, std::placeholders::_1));
        if (robotNumber != 1)
        {
            robot1Subscription_ = this->create_subscription<bfc_msgs::msg::Coordination>(
                "/robot_1/coordination", 10, std::bind(&main_strategy::readRobotCoordinationData1, this, std::placeholders::_1));
        }

        if (robotNumber != 2)
        {
            robot2Subscription_ = this->create_subscription<bfc_msgs::msg::Coordination>(
                "/robot_2/coordination", 10, std::bind(&main_strategy::readRobotCoordinationData2, this, std::placeholders::_1));
        }

        if (robotNumber != 3)
        {
            robot3Subscription_ = this->create_subscription<bfc_msgs::msg::Coordination>(
                "/robot_3/coordination", 10, std::bind(&main_strategy::readRobotCoordinationData3, this, std::placeholders::_1));
        }

        if (robotNumber != 4)
        {
            robot4Subscription_ = this->create_subscription<bfc_msgs::msg::Coordination>(
                "/robot_4/coordination", 10, std::bind(&main_strategy::readRobotCoordinationData4, this, std::placeholders::_1));
        }

        if (robotNumber != 5)
        {
            robot5Subscription_ = this->create_subscription<bfc_msgs::msg::Coordination>(
                "/robot_5/coordination", 10, std::bind(&main_strategy::readRobotCoordinationData5, this, std::placeholders::_1));
        }

        robotCoordination_ = this->create_publisher<bfc_msgs::msg::Coordination>(
            "coordination", 1);
        cmd_head_ = this->create_publisher<bfc_msgs::msg::HeadMovement>(
            "head", 1);
        cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "walk", 1);
        cmd_mot_ = this->create_publisher<std_msgs::msg::String>(
            "motion", 1);
        Odometry_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "pose", 1);
        Update_coor_ = this->create_publisher<std_msgs::msg::Bool>(
            "update", 1);
        request_pub = this->create_publisher<std_msgs::msg::String>(
            "request_path", 1);
        ball_status_pub = this->create_publisher<std_msgs::msg::Bool>(
            "ball_status", 1);

        timer_ = this->create_wall_timer(40ms, std::bind(&main_strategy::timer_callback, this));

        factory.registerSimpleCondition("KillRun", std::bind(&main_strategy::KillRun, this));
        factory.registerSimpleCondition("WalkTowardsBall", std::bind(&main_strategy::WalkTowardsBall, this));
        factory.registerSimpleCondition("KickTowardsGoal", std::bind(&main_strategy::KickTowardsGoal, this));
        factory.registerSimpleCondition("GameControllerInit", std::bind(&main_strategy::GameControllerInit, this));
        factory.registerSimpleCondition("GameControllerReady", std::bind(&main_strategy::GameControllerReady, this));
        factory.registerSimpleCondition("GameControllerSet", std::bind(&main_strategy::GameControllerSet, this));
        factory.registerSimpleCondition("GameControllerPlay", std::bind(&main_strategy::GameControllerPlay, this));
        factory.registerSimpleCondition("GameControllerFinish", std::bind(&main_strategy::GameControllerFinish, this));
        factory.registerSimpleCondition("Executor", std::bind(&main_strategy::Executor, this));
        factory.registerSimpleCondition("StatePickup", std::bind(&main_strategy::StatePickup, this));
        factory.registerSimpleCondition("StateRelease", std::bind(&main_strategy::StateRelease, this));
        factory.registerSimpleCondition("BallFoundTeam", std::bind(&main_strategy::BallFoundTeam, this));
        factory.registerSimpleCondition("DoneKick", std::bind(&main_strategy::DoneKick, this));
        factory.registerSimpleCondition("StateKickOff", std::bind(&main_strategy::StateKickOff, this));
        factory.registerSimpleAction("GetData", std::bind(&main_strategy::GetData, this));
        factory.registerSimpleAction("SetHeadPos", std::bind(&main_strategy::SetHeadPos, this));
        factory.registerSimpleAction("BallFound", std::bind(&main_strategy::BallFound, this));
        factory.registerSimpleAction("BodyTrack", std::bind(&main_strategy::BodyTrack, this));
        factory.registerSimpleAction("BallApproach", std::bind(&main_strategy::BallApproach, this));
        factory.registerSimpleAction("SearchingBall", std::bind(&main_strategy::SearchingBall, this));
        factory.registerSimpleAction("RotateToGoal", std::bind(&main_strategy::RotateToGoal, this));
        factory.registerSimpleAction("Kick", std::bind(&main_strategy::Kick, this));
        factory.registerSimpleAction("InitialPosition", std::bind(&main_strategy::InitialPosition, this));
        factory.registerSimpleAction("RobotPositioning", std::bind(&main_strategy::RobotPositioning, this));
        factory.registerSimpleAction("ResetVar", std::bind(&main_strategy::ResetVar, this));
        factory.registerSimpleAction("BallTracking", std::bind(&main_strategy::BallTracking, this));
        factory.registerSimpleAction("Relax", std::bind(&main_strategy::Relax, this));
        factory.registerSimpleAction("Communication", std::bind(&main_strategy::Communication, this));
        factory.registerSimpleAction("FirstKick", std::bind(&main_strategy::FirstKick, this));
        factory.registerSimpleAction("SearchAfterKick", std::bind(&main_strategy::SearchAfterKick, this));
        factory.registerSimpleAction("WalkSearchBall", std::bind(&main_strategy::WalkSearchBall, this));
        factory.registerSimpleAction("OdomUpdate", std::bind(&main_strategy::OdomUpdate, this));

        tree = factory.createTreeFromFile(tree_path);
        pubZ = new BT::PublisherZMQ(tree);
    }

    void run()
    {
        executor_.add_node(this->get_node_base_interface());
        executor_.spin();
    }

private:
    bool doneScan = false;
    void scan_landmark()
    {
        if (cnt_sbr > 2)
        {
            doneScan = true;
        } else 
        {
            searchBallRectang(-1.6, -1.6, -0.8, 1.6);
            if (object_count > 1)
            {    
                auto msg_update = std_msgs::msg::Bool();
                msg_update.data = true;
                Update_coor_->publish(msg_update);
                // saveToCSV(Grid, robotPos_X, robotPos_Y, msg_yaw, headPan, headTilt, Left_X_Cross_X, Left_X_Cross_Y, Right_X_Cross_X, Right_X_Cross_Y, Left_T_Cross_X, Left_T_Cross_Y, Right_T_Cross_X, Right_T_Cross_Y, Left_Corner_X, Left_Corner_Y, Right_Corner_X, Right_Corner_Y, Left_L_Cross_X, Left_L_Cross_Y, Right_L_Cross_X, Right_L_Cross_Y, Pinalty_X, Pinalty_Y,Left_T_Corner_X,Left_T_Corner_Y, Right_T_Corner_X,Right_T_Corner_Y);
            }    
        }
    }

    NodeStatus KillRun()
    {
        // printf("...KillRun\n");
        if (lastState != msg_kill)
        {
            if (msg_kill == 0)
            {
                stateCondition = firstStateCondition;
                // State = 3;
                // Pickup = true;
                if (State == 1)
                {
                    dont_calibrate = true;
                }
                play = true;
            }
            else
            {
                motion("8");
                resetVariable();
                play = false;
            }
            lastState = msg_kill;
        }

        if (play)
        {
            // printf("...KillRun::SUCCESS\n");
            robotStatus = 1;
            return NodeStatus::SUCCESS;
        }
        else
        {
            stateCondition = 272;
            ballDistance = 999;
            BallGrid = 88;
            Grid = 88;
            foundBall = robotStatus = 0;
            return NodeStatus::FAILURE;
        }
    }

    void saveToCSV(int grid, double posx, double posy, double yaw, double pan, double tilt, int c0_x, int c0_y, int c1_x, int c1_y, int c2_x, int c2_y, int c3_x, int c3_y, int c4_x, int c4_y, int c5_x, int c5_y, int c6_x, int c6_y)
    {
        std::ofstream file;
        file.open("/home/barelang2/bfc_ros2/src/main/data_posisi.csv", std::ios::out | std::ios::app);
        file << grid << ", " << posx << ", " << posy << ", " << yaw << ", " << pan << ", " << tilt << ", " << c0_x << ", " << c0_y << ", " << c1_x << ", " << c1_y << ", " << c2_x << ", " << c2_y << ", " << c3_x << ", " << c3_y << ", " << c4_x << ", " << c4_y << ", " << c5_x << ", " << c5_y << ", " << c6_x << ", " << c6_y << std::endl;
        file.close();
        std::cout << grid << ", " << posx << ", " << posy << ", " << yaw << ", " << pan << ", " << tilt << ", " << c0_x << ", " << c0_y << ", " << c1_x << ", " << c1_y << ", " << c2_x << ", " << c2_y << ", " << c3_x << ", " << c3_y << ", " << c4_x << ", " << c4_y << ", " << c5_x << ", " << c5_y << ", " << c6_x << ", " << c6_y << std::endl;
    }

    bool doneGetData = false;
    bool doneSaveData = false;
    int cnt_get_data = 0;

    NodeStatus GetData()
    {
        // if (!doneSaveData && triggerSave)
        // {
        //     saveToCSV(Grid, robotPos_X, robotPos_Y, msg_yaw, headPan, headTilt, Left_X_Cross_X, Left_X_Cross_Y, Right_X_Cross_X, Right_X_Cross_Y, Left_T_Cross_X, Left_T_Cross_Y, Right_T_Cross_X, Right_T_Cross_Y, Left_Corner_X, Left_Corner_Y, Right_Corner_X, Right_Corner_Y, Pinalty_X, Pinalty_Y);
        //     doneSaveData = true;
        //     triggerSave = false;
        // }
        initialPos_X = -350;
        initialPos_Y = 0;
        if (doneMoved)
        {
            motion("0");
            return NodeStatus::FAILURE;
        } else 
        {
            new_out_grid(24, 0, 0, true); // grid tujuan - jojo
        }
        // new_out_grid(50, 50, -50, false);
        // printf("msg_yaw : %d\n", msg_yaw);
        // printf("ArahTendangan : %d\n", outTheta);
        // if (sumWalkX >= max_current)
        // {
        //     motion("0");
        // } else 
        // {
        //     Walk(0.0, jalan, 0.0);
        // }
        // calculate_target(450, 0, 0, 200);
        return NodeStatus::FAILURE;
    }

    NodeStatus SetHeadPos()
    {
        robotPos_X = convertGridX(6, 0);
        robotPos_Y = convertGridY(6, 0);
        headMove(dataPanKey, dataTiltKey);
        if (object_count > 1)
        {
            return NodeStatus::SUCCESS;
        }
        doneSaveData = false;
        return NodeStatus::FAILURE;
    }

    bool tracked = false;
    int BallFoundEntry = 0;
    NodeStatus BallFound()
    {
        if (ballLost(20))
        {
            delayWaitBall = 0;
            // searchBallRectang(-1.6, -1.6, -0.8, 1.6);
		    foundBall = 0;
		    ballDistance = 999;
		    stateCondition = 272;
		    BallGrid = 88;
            BallApproachEntry = 0;
            if (cam_x != 999 && cam_y != 999 && cam_x < 0 && headTilt >= -1.55 && !dont_calibrate)
		    {
		    	initialPos_X = initialPos_Y = 0;
		    	deltaPos_X = cam_x;
		    	deltaPos_Y = cam_y;
		    }	
            return NodeStatus::FAILURE;
        }

        foundBall = 1;
        refreshMoveGrid();
        return NodeStatus::SUCCESS;       
    }

    bool bodyTracked = false;
    bool useBodyTracking = false;
    int BodyTrackEntry = 0;
    NodeStatus BodyTrack()
    {
        
        return NodeStatus::FAILURE;
    }

    int ballOnLeftGoal = 0;
    int ballOnGoalSide()
    {
        if (robotPos_X > 325)
        {
            if (robotPos_Y < -150)
            {
            	ballOnLeftGoal = 1;
            	return 1;
            } else if (robotPos_Y > 150)
            {
            	ballOnLeftGoal = 0;
            	return 1;
            } else 
            {
            	return 0;
            }
        } else 
        {
            return 0;
        }
    }

    bool action_walk = false, isKicked = false;
    int BallApproachEntry = 0;
    NodeStatus BallApproach()
    {
        if (BallApproachEntry > 5)
        {
            if (action_walk)
            {
                motion("0");
                return NodeStatus::SUCCESS;
            }

            if (body_tracked)
            {
                if (ballDistance <= 150)
                {
                    printf("...myTurn!!!\n");
                    stateCondition = 232;
                    ballDistance = 1;
                } else 
                {
                
                    stateCondition = 272;
                }

                if (ballLost(20))
                {
                    Walk(0.0, 0.0, 0.0);
                    delayWaitBall = 0;
                    tracked = false;
                } else 
                {
                    trackBall();
                    if (delayWaitBall > 10)
                    {
                        tracked = true;
                    } else 
                    {
                        delayWaitBall++;
                        Walk(0.0, 0.0, 0.0);
                    }

                    if (tracked)
                    {
                        if (headTilt >= cAktif && headPan >= -0.4 && headPan <= 0.4)
                        {
                            if (robotDirection)
                            {
                                printf("...kick\n");
                                if (tendang)
                                {
                                    if (modeKick == 3)
                                    {
                                        headMove(-1.6, -1.6);
                                    } else if (modeKick == 4)
                                    {
                                        headMove(1.6, -1.6);
                                    } else 
                                    {
                                    	headMove(0.0, -1.6);
                                    }
                                    isKicked = action_walk = true;
                                } else 
                                {
                                    kick(modeKick);
                                }
                            } else 
                            {
                                printf("...IMu\n");
                                //new_out_grid(50, 50, -50, false);
                                if (sudutTendang != 0)
                                {
                                    Imu(sudutTendang, cSekarang);
                                } else 
                                {
                                    if (ballOnGoalSide())
                                    {
                                        if (ballOnLeftGoal == 1)
                                        {
                                            if (msg_yaw > 45)
                                            {
                                                modeKick = tendangDekat;
                                                Imu(90, cSekarang);
                                            } else 
                                            {
                                                modeKick = 3;
                                                Imu(0, cSekarang);
                                            }
                                        } else 
                                        {
                                            if (msg_yaw < -45)
                                            {
                                                modeKick = tendangDekat;
                                                Imu(-90, cSekarang);
                                            } else 
                                            {
                                                modeKick = 4;
                                                Imu(0, cSekarang);
                                            }
                                        }
                                    } else
                                    {                                        
                                        if (robotPos_Y > -100 && robotPos_Y < 100)
                                        {    
                                        	if (robotPos_X < 450)
                                        	{                                      	
				                            	if (robotPos_Y > 0)
				                                {
				                                  	//new_out_grid(50, 50, 0, false);
				                                  	theta = -10.0;
				                                } else 
				                                {
				                                  	//new_out_grid(51, 50, 0, false);
				                                  	theta = 10.0;
				                                }
				                            } else 
				                            {
				                            	theta = 0.0;
				                            } 
		                                                                                                         
                                            if (robotPos_X > 325)
                                            {
                                                if (msg_yaw > 45)
                                                {
                                                    modeKick = 4;
                                                    Imu(85, cSekarang);
                                                } else if (msg_yaw < -45)
                                                {
                                                    modeKick = 3;
                                                    Imu(-85, cSekarang);
                                                } else 
                                                {                                                	
                                                    modeKick = tendangJauh;
                                                    Imu(0, cSekarang);
                                                }
                                            } else 
                                            {
                                                modeKick = tendangJauh;
                                                Imu(0, cSekarang);
                                            }
                                        } else 
                                        {
                                        	if (robotPos_X < 450)
                                        	{
                                        		new_out_grid(50, 50, -50, false);
                                        	} else 
                                        	{
                                        		theta = 0.0;
                                        	}
                                            modeKick = tendangJauh;
                                            Imu(int(theta), cSekarang);
                                        }
                                    }
                                }
                            }
                        } else 
                        {
                            printf("...followBall\n");
                            if (robotNumber != 1)
                            {
                                if (robotPos_X > 0)
                                { 
                                    sudutTendang = 0;
                                } else 
                                {
                                    if (msg_yaw > 45)
                                    {
                                        sudutTendang = 90;
                                        modeKick = 4;
                                    } else if (msg_yaw < -45)
                                    {
                                        sudutTendang = -90;
                                        modeKick = 3;
                                    } else 
                                    {
                                        sudutTendang = 0;
                                        modeKick = tendangJauh;
                                    }
                                }
                            }
                            followBall(0);
                            robotDirection = ballPos = tendang = false;
                        }
                    }
                }
            } else 
            {
                if (ballLost(20))
                {
                    delayWaitBall = 0;
                    Walk(0.0, 0.0, 0.0);
                    searchBallRectang(-1.6, -1.6, -0.8, 1.6);
                } else 
                {
                    trackBall();
                    if (delayWaitBall > 20)
                    {
                        newBodyTracking();
                    } else 
                    {
                        delayWaitBall++;
                    }
                }
            }
        }
        else
        {
            action_walk = action_kick = false;
            body_tracked = tracked = tendang = ballPos = robotDirection = false;
            SearchAfterKickEntry = RobotPositioningEntry = WalkSearchBallEntry = relaxEntry = KickEntry = 0;
            Walk(0.0, 0.0, 0.0);
            BallApproachEntry++;
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus WalkTowardsBall()
    {
        return NodeStatus::FAILURE;
    }

    

    bool action_kick = false;
    bool robotFinalDirection = false;
    int RotateToGoalEntry = 0, robotFinalDirectionEntry = 0;
    NodeStatus RotateToGoal()
    {
        if (action_kick)
        {
            return NodeStatus::SUCCESS;
        } else 
        {
            if (robotDirection)
            {
                action_kick = true;
            } else
            {
                new_out_grid(50, 50 ,-50, false);
                Imu((int)theta, cSekarang);
            }
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus KickTowardsGoal()
    {
        
        return NodeStatus::FAILURE;
    }

    bool action_afterKick = false;
    int KickEntry = 0;
    NodeStatus Kick()
    {
        if (KickEntry > 5)
        {
            if (ballLost(20))
            {
            	headMove(0.0, cSekarang - 0.2);
            }
            else 
            {
            	trackBall();
            	if (delayWaitBall > 20)
            	{
		        	if (robotDirection && headPan >= -0.4 && headPan <= 0.4)
		        	{
		        		if (tendang)
		        		{
		        			sleep(2);
		        			motion("0");
		        		} else 
		        		{
		        			kick(tendangJauh);
		        		}
		        	}
		        	else 
		        	{
		        		if (msg_strategy == 1)
		        		{
		        			Imu(30, cSekarang);
		        		} else if (msg_strategy == 2)
		        		{
		        			Imu(-30, cSekarang);
		        		} else 
		        		{
		        			Imu(0, cSekarang);
		        		}
		        	}
		        } else 
		        {
		        	delayWaitBall++;
		        }
            }
        } else 
        {
            motion("0");
            delayWaitBall = 0;
            robotDirection = ballPos = tendang = false;
            KickEntry++;
        }
        return NodeStatus::FAILURE;
    }

    bool FirstKicked = false;
    int FirstKickEntry = 0;
    NodeStatus FirstKick()
    {
        
        return NodeStatus::FAILURE;
    }

    NodeStatus DoneKick()
    {
    	if (isKicked)
    	{
    		return NodeStatus::SUCCESS;
    	}
        return NodeStatus::FAILURE;
    }

    int SearchAfterKickEntry = 0;
    NodeStatus SearchAfterKick()
    {
    	if (SearchAfterKickEntry > 5)
    	{
    		printf("...myTurn!!!\n");
            stateCondition = 232;
            ballDistance = 1;
			if (ballLost(20))
			{
				if (modeKick == 3)
				{
				    headMove(-1.6, -1.6);
				} else if (modeKick == 4)
				{
				    headMove(1.6, -1.6);
				} else 
				{
				   	headMove(0.0, -1.6);
				}
			} else 
			{
				if (second > 6)
				{
					trackBall();
				}
			}
			
			cekWaktu(8);
			if (timer)
			{
				isKicked = false;
			}
		} else 
		{
			setWaktu();
			BallApproachEntry = 0;
			SearchAfterKickEntry++;
		}
        return NodeStatus::FAILURE;
    }

    int SearchingBallEntry = 0;
    NodeStatus SearchingBall()
    {
        
        return NodeStatus::FAILURE;
    }

    NodeStatus GameControllerInit()
    {
        // printf("...GameControllerInit\n");
        if (State == 0) {
            motion("0");
            Pickup = false;
            // printf("...GcInit::SUCCESS\n");
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus GameControllerReady()
    {
        // printf("...GameControllerReady\n");
        if (State == 1) {
            // printf("...GcReady::SUCCESS\n");
            foundBall = 0;
            stateCondition = 272;
            ballDistance = 999;
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus GameControllerSet()
    {
        // printf("...GameControllerSet\n");
        if (State == 2) {
            motion("0");
            // printf("...GcSet::SUCCESS\n");
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus GameControllerPlay()
    {
        // printf("...GameControllerPlay\n");
        if (State == 3) {
            // printf("...GcPlay::SUCCESS\n");
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus GameControllerFinish()
    {
        // printf("...GameControllerFinish\n");
        if (State == 4) {
            // printf("...GcFinish::SUCCESS\n");
            motion("0");
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    bool finishFirstKick = false;
    bool doneFirstKick = false;
    NodeStatus StateKickOff()
    {
        if (finishFirstKick)
        {
            robotKick = 1;
            return NodeStatus::SUCCESS;
        }

        if (KickOff == barelang_color)
        {
            FirstKicked = true;
        } else 
        {
            if (SecondaryTime <= 2)
            {
                if (useKickOffGoal)
                {
                    finishFirstKick = true;
                }
                FirstKicked = true;
                cekWaktu(5);
                if (timer)
                {
                    if (robotNumber == 2)
                    {
                        if (role == 1 && robot3FBall == 0)
                        {
                            //walkGrid(15, 0, 25);
                            finishFirstKick = true;
                        }
                    } else if (robotNumber == 3)
                    {
                        if (role == 1 && robot2FBall == 0)
                        {
                            //walkGrid(15, 0, 25);
                            finishFirstKick = true;
                        }
                    }
				}
            } else 
            {
            	setWaktu();
                if (ballLost(20))
                {
                    searchBallRectang(-1.6, -1.6, -0.8, 1.6);
                    motion("0");
                } else 
                {
                    trackBall();
                    if (tunggu > 100)
                    {
                    	hitungGerakBola();
                    	if (abs(deltaY) <= valBolaGerak)
                    	{
                    		motion("0");
                    	} else 
                    	{
                    		FirstKicked = finishFirstKick = true;
                    	}
                    } else 
                    {
                    	motion("0");
                    	tunggu++;
                    }
                }
            }
        }

        // if (!finishFirstKick)
        // {
        //     if (robotPos_X > 50)
        //     {
        //         finishFirstKick = true;
        //     }
        // }
        
        if (doneFirstKick)
        {
        	//if (abs(abs(koorRobotX) - abs(robotPos_X)) > 50)
        	if (robotPos_X > 70)
        	{
        		finishFirstKick = true;
        	} 
        }

       
        if (FirstKicked)
        {
            if (role == 0)
            {
                if (robot1BackIn == 1 || robot2BackIn == 1 || robot3BackIn == 1 || robot4BackIn == 1 || robot5BackIn == 1)
                {
                    bodyTracked = true;
                    finishFirstKick = true;
                }

                if (ballLost(20))
                {
                    cekWaktu(25);
                    if (second > 10)
                    {
                        searchBallRectang(-1.6, -1.6, -0.8, 1.6);
                        if (second > 15)
                        {
                        	if (timer)
                        	{
                 				finishFirstKick = true;
                 			} else 
                 			{
                 				jalanDirection(0.0, 0.0, -179);
                 			}
                        } else 
                        {
                 			jalanDirection(-0.03, 0.0, 0);       
                        }
                    } else
                    {
                        tiltSearchBall(0.0);
                        Walk(0.0, 0.0, 0.0);
                    }
                    
                    robotDirection = ballPos = tendang = false;
                    //stateCondition = 272;
                    //foundBall = 0;
                    //ballDistance = 999;
                }
                else
                {   
                    stateCondition = 232;
                    foundBall = 1;
                    ballDistance = 1; 
                    setWaktu();
                    trackBall();
                    if (headTilt >= cAktif && headPan >= -0.4 && headPan <= 0.4)
                    {
                        if (robotDirection)
                        {
                            if (tendang)
                            {
                                //finishFirstKick = true;
                                saveKoordinatRobot();
                                //motion("0");
                                doneFirstKick = true;
                            } else 
                            {
                                kick(tendangDekat);
                            }
                        } else 
                        {
                            if (msg_strategy == 1)
                            {
                                Imu(40, cSekarang);
                            } else if (msg_strategy == 2)
                            {
                                Imu(-40, cSekarang);
                            } else 
                            {
                                Imu(0, cSekarang);
                            }
                        }
                    } else 
                    {
                        robotDirection = ballPos = tendang = false;
                        followBall(0);
                    }
                }
            } else if (role == 1)
            {
                if (robot1BackIn == 1 || robot2BackIn == 1 || robot3BackIn == 1 || robot4BackIn == 1 || robot5BackIn == 1)
                {
                    finishFirstKick = true;
                } else 
                {
                    if (ballLost(20))
                    {
                        foundBall = 0;
                        ballDistance = 999;
                        delayWaitBall = 0;
                        searchBallRectang(-1.6, -1.6, -0.8, 1.6);
                    } else 
                    {
                        trackBall();
                        if (delayWaitBall > 20)
                        {
                            foundBall = 1;
                            // if (BallGrid <= 17)
                            if (ballDistance <= 85)
                            {
                                finishFirstKick = true;
                            }
                        } else 
                        {
                            delayWaitBall++;
                        }
                    }
                }
            }
        }
        return NodeStatus::FAILURE;
    }

    int GridTarget = 0, GridX = 0, GridY = 0;
    int role = 0; // 0 : attacker
    NodeStatus InitialPosition()
    {
        // printf("...InitialPosition\n");
        if (lockInitPos < 10)
        {
            if (State == 3) { // pickup
                if (msg_yaw >= 0) // dari sisi kiri
                {
                    initialPos_X = -125;
                    initialPos_Y = -300;
                    role = 0;
                } else if (msg_yaw < 0) //dari sisi kanan
                {
                    initialPos_X = -125;
                    initialPos_Y = 300;
                    role = 1;
                }
            } else if (State == 0 || State == 1) {
                if (modePlay == 0)
                {
                    if (msg_yaw > -90 && msg_yaw < 90)
                    {
                        initialPos_X = -360;
                        initialPos_Y = 0;
                        role = 0;
                    } else if (msg_yaw > 90)
                    {
                        initialPos_X = -360;
                        initialPos_Y = 130;
                        role = 1;
                    } else if (msg_yaw < -90)
                    {
                        initialPos_X = -360;
                        initialPos_Y = -130;
                        role = 1;
                    }
                } else if (modePlay == 1)
                {
                    if (msg_yaw >= 0) // dari sisi kiri
                    {
                        initialPos_X = -100;
                        initialPos_Y = -300;
                        role = 0;
                    } else if (msg_yaw < 0) //dari sisi kanan
                    {
                        initialPos_X = -200;
                        initialPos_Y = 300;
                        role = 1;
                    }
                }
            }
            lockInitPos++;
        }
        // printf("...InitialPosition::SUCCESS\n");
        return NodeStatus::SUCCESS;
    }

    int state_rotate = 0, last_state_rotate = 0;
    int angle_to_rotate = 0, reset_rotate = 0, last_angle_rotate = 0, reset_scan = 0;
    bool start_rotate = false;
    void rotateBodySearchGrid()
    {
        if (last_state_rotate != state_rotate)
        {
            posRotateNew = false;
            sabar = 0;
            searchKe = 0;
        }

        if (robotNumber == 1)
        {
            if (robot2DBall == 1) {new_out_grid(robot2GridBall, 0, 0, false); angle_to_rotate = (int)theta;}
            else if (robot3DBall == 1) {new_out_grid(robot3GridBall, 0, 0, false); angle_to_rotate = (int)theta;}
            else if (robot4DBall == 1) {new_out_grid(robot4GridBall, 0, 0, false); angle_to_rotate = (int)theta;}
            else if (robot5DBall == 1) {new_out_grid(robot5GridBall, 0, 0, false); angle_to_rotate = (int)theta;}
        } else if (robotNumber == 2)
        {
            if (robot1DBall == 1) {new_out_grid(robot1GridBall, 0, 0, false); angle_to_rotate = (int)theta;}
            else if (robot3DBall == 1) {new_out_grid(robot3GridBall, 0, 0, false); angle_to_rotate = (int)theta;}
            else if (robot4DBall == 1) {new_out_grid(robot4GridBall, 0, 0, false); angle_to_rotate = (int)theta;}
            else if (robot5DBall == 1) {new_out_grid(robot5GridBall, 0, 0, false); angle_to_rotate = (int)theta;}
        } else if (robotNumber == 3)
        {
            if (robot1DBall == 1) {new_out_grid(robot1GridBall, 0, 0, false); angle_to_rotate = (int)theta;}
            else if (robot2DBall == 1) {new_out_grid(robot2GridBall, 0, 0, false); angle_to_rotate = (int)theta;}
            else if (robot4DBall == 1) {new_out_grid(robot4GridBall, 0, 0, false); angle_to_rotate = (int)theta;}
            else if (robot5DBall == 1) {new_out_grid(robot5GridBall, 0, 0, false); angle_to_rotate = (int)theta;}
        } else if (robotNumber == 4)
        {
            if (robot1DBall == 1) {new_out_grid(robot1GridBall, 0, 0, false); angle_to_rotate = (int)theta;}
            else if (robot2DBall == 1) {new_out_grid(robot2GridBall, 0, 0, false); angle_to_rotate = (int)theta;}
            else if (robot3DBall == 1) {new_out_grid(robot3GridBall, 0, 0, false); angle_to_rotate = (int)theta;}
            else if (robot5DBall == 1) {new_out_grid(robot5GridBall, 0, 0, false); angle_to_rotate = (int)theta;}
        } else if (robotNumber == 5)
        {
            if (robot1DBall == 1) {new_out_grid(robot1GridBall, 0, 0, false); angle_to_rotate = (int)theta;}
            else if (robot2DBall == 1) {new_out_grid(robot2GridBall, 0, 0, false); angle_to_rotate = (int)theta;}
            else if (robot3DBall == 1) {new_out_grid(robot3GridBall, 0, 0, false); angle_to_rotate = (int)theta;}
            else if (robot4DBall == 1) {new_out_grid(robot4GridBall, 0, 0, false); angle_to_rotate = (int)theta;}
        }


        if (angle_to_rotate != 272)// && diffTheta > 90)
        {
            start_rotate = true;
        } else 
        {
            start_rotate = false;
        }

        // printf("...reset_rotate : %d\n", reset_rotate);

        // searchBallRectang(-1.6, -1.6, -0.8, 1.6);
        if (start_rotate)
        {
            if (reset_rotate > 20)
            {
                // printf("...startRotate : %d\n", start_rotate);
                // printf("...lastAngleRotate : %d\n", last_angle_rotate);
                // printf("...angleToRotate : %d\n", angle_to_rotate);
                // printf("...diffAngle : %d\n", abs(last_angle_rotate - angle_to_rotate));
                if (posRotateNew)
                {
                    motion("0");
                    if (abs(last_angle_rotate - angle_to_rotate) > 45)
                    {
                        reset_rotate = 0;
                    }
                } else
                {
                    rotateBodyImuNew(angle_to_rotate);
                }
            } else 
            {
                last_angle_rotate = angle_to_rotate;
                posRotateNew = false;
                reset_rotate++;
            }
        } else 
        {
            motion("0");
        }
    }

    int last_state_grid = 0;
    void walkGrid(int grid, int offx, int offy)
    {
        if (state_move_grid != last_state_grid)
        {
            searchKe = 0;
            sabar = 0;
            refreshMoveGrid();
            last_state_grid = state_move_grid;
        }

        printf("...searchKe: %d\n", searchKe);
        printf("...sabar: %d\n", sabar);
        printf("...state_move_grid: %d\n", state_move_grid);
        printf("...last_state_grid: %d\n", last_state_grid);

        switch (state_move_grid)
        {
            case 0:
                if (doneMoved)
                {
                    state_move_grid = 1;
                } else 
                {
                    if (searchKe >= 1)
                    {
                        searchBallRectang(-1.6, -1.6, -0.8, 1.6);
                        new_out_grid(grid, offx, offy, true);
                    } else
                    {
                        motion("0");
                        threeSearchBall();
                    }
                }
            break;

            case 1:
                if (posRotateNew)
                {
                    state_move_grid = 2;
                } else 
                {
                    if (searchKe >= 1)
                    {
                        searchBallRectang(-1.6, -1.6, -0.8, 1.6);
                        rotateBodyImuNew(90);
                    } else
                    {
                        motion("0");
                        threeSearchBall();
                    }
                }
            break;

            case 2:
                if (posRotateNew)
                {
                    state_move_grid = 1;
                } else 
                {
                    if (searchKe >= 1)
                    {
                        searchBallRectang(-1.6, -1.6, -0.8, 1.6);
                        rotateBodyImuNew(-90);
                    } else
                    {
                        motion("0");
                        threeSearchBall();
                    }
                }
            break;
        }
    }

    bool action_relax = false;
    bool stateWalkSearch = false;
    bool done_calibrate = false, dont_calibrate = false;
    int RobotPositioningEntry = 0, delay_calibrate = 0;
    NodeStatus RobotPositioning()
    {
        // printf("...RobotPositioning\n");
        if (RobotPositioningEntry > 5)
        {
            if (role == 0)
            {
            	if (KickOff == barelang_color)
            	{
                	GridTarget = 20; GridX = 50; GridY = -50;
                } else 
                {
                	GridTarget = 20; GridX = -35; GridY = -50;
                }
            } else if (role ==1)
            {
                if (robot1Status == 1)
                {
                    GridTarget = 15; GridX = 0; GridY = 25;
                } else 
                {
                    GridTarget = 9; GridX = 0; GridY = 25;
                }
            }

            if (Pickup || Remaining == 600 || Remaining == 300)
            {
            	dont_calibrate = true;
                if (done_calibrate)
                {
                    Pickup = false;
                    if (goalLost(20))
                    {
                    	panSearchBall(-1.55);
                    	delay_calibrate = 0;
                    } else 
                    {
                    	trackGoal();
		                if (delay_calibrate > 50)
		                {
		                	if (cam_x != 999 && cam_y != 999 && cam_x < 0 && headTilt >= -1.55 && !dont_calibrate)
		                	{
		                		initialPos_X = initialPos_Y = 0;
		                		deltaPos_X = cam_x;
		                		deltaPos_Y = cam_y;
		                	}	
		                	done_calibrate = true;
		                } else 
		                {
		                	delay_calibrate++;
		                }
		            }
                    return NodeStatus::SUCCESS;
                }

                if (doneMoved)
                {
                    motion("0");
                    if (goalLost(20))
                    {
                    	panSearchBall(-1.55);
                    	delay_calibrate = 0;
                    	cekWaktu(15);
                    	if (timer)
                    	{
                    		done_calibrate = true;
                    	}
                    } else 
                    {
                    	trackGoal();
		                if (delay_calibrate > 50)
		                {
		                	if (cam_x != 999 && cam_y != 999 && cam_x < 0 && headTilt >= -1.55 && !dont_calibrate)
		                	{
		                		initialPos_X = initialPos_Y = 0;
		                		deltaPos_X = cam_x;
		                		deltaPos_Y = cam_y;
		                	}
		                	done_calibrate = true;
		                } else 
		                {
		                	delay_calibrate++;
		                }
		             }
                } else 
                {
		            if (ballLost(20))
		            {
		                searchBallRectang(-1.6, -1.6, -0.8, 1.6);
                        delayWaitBall = 0;
		            } else 
		            {
		                trackBall();
                        if (delayWaitBall > 50)
                        {
                        	if (headTilt >= cAktif + 0.2)
                        	{
                        		Pickup = false;
                        	}
                        } else 
                        {
                            delayWaitBall++;
                        }
		            }
                    new_out_grid(GridTarget, GridX, GridY, true);
                }
            } else 
            {
                if (doneMoved)
                {
                    motion("0");
                    if (goalLost(20))
                    {
                    	panSearchBall(-1.55);
                    	delay_calibrate = 0;
                    } else 
                    {
                    	trackGoal();
		                if (delay_calibrate > 50)
		                {
		                	if (cam_x != 999 && cam_y != 999 && cam_x < 0 && headTilt >= -1.55 && !dont_calibrate)
		                	{
		                		initialPos_X = initialPos_Y = 0;
		                		deltaPos_X = cam_x;
		                		deltaPos_Y = cam_y;
		                	}	
		                	done_calibrate = true;
		                } else 
		                {
		                	delay_calibrate++;
		                }
		            }
                    return NodeStatus::SUCCESS;
                } else 
                {
                    if (SecondaryTime < 5)
                    {
                        if (posRotateNew)
                        {
                            motion("0");
                            doneMoved = true;
                        } else 
                        {
                            rotateBodyImuNew(0);
                        }
                    } else 
                    {
                        if (goalLost(20))
                        {
                            panSearchBall(-1.55);
                        } else 
                        {
                            trackGoal();
                            if (cam_x != 999 && cam_y != 999 && cam_x < 0 && headTilt >= -1.55 && !dont_calibrate)
						    {
						    	initialPos_X = initialPos_Y = 0;
						    	if (cekArah())
						    	{
						    		deltaPos_X = cam_x * -1;
									deltaPos_Y = cam_y * -1;
						    	}
						    	else 
						    	{
									deltaPos_X = cam_x;
									deltaPos_Y = cam_y;
								}
						    }
                        }
                        new_out_grid(GridTarget, GridX, GridY, true);
                    }
                }
            }
        } else
        {
            Walk(0.0, 0.0, 0.0);
            sabar = 0;
            setWaktu();
            refreshMoveGrid();
            delay_calibrate = 0;
            done_calibrate = false;
            RobotPositioningEntry++;
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus OdomUpdate()
    {
        if (done_calibrate)
        {
            Pickup = false;
            return NodeStatus::SUCCESS;
        }

        if (goalLost(20))
        {
            panSearchGoal(-2.0);
            delay_calibrate = 0;
        } else 
        {
            trackGoal();
            if (delay_calibrate > 50)
            {
                if (cam_x != 999 && cam_y != 999 && cam_x < 0)
                {
                    initialPos_X = initialPos_Y = 0;
                    deltaPos_X = cam_x;
                    deltaPos_Y = cam_y;
                }   
                done_calibrate = true;
            } else 
            {
                delay_calibrate++;
            }
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus ResetVar()
    {
        // printf("...ResetVar\n");
        refreshMoveGrid();
        done_calibrate = Release = Pickup = action_relax = false;
        action_kick = action_walk = action_afterKick = false;
        tracked = bodyTracked = robotDirection = ballPos = tendang = false;
        tracked = bodyTracked = action_kick = action_walk = action_afterKick = robotDirection = tendang = ballPos = false;
        delayWaitBall = delay_calibrate = 0;
        if (State == 2)
        {
            resetCommunication();
            tunggu = robotKick = 0;
            dont_calibrate = doneFirstKick = FirstKicked = finishFirstKick = false;
        }
        return NodeStatus::FAILURE;
    }

    bool lockRelax = false;
    int relaxEntry = 0;
    NodeStatus Relax()
    {
        if (relaxEntry > 5)
        {
            // printf("...Relax\n");
            // motion("0");
            printf("...waitingTurn!!!\n");
            ballDistance = 999;
            stateCondition = 272;
            if (ballLost(20))
            {
                rotateBodySearchGrid();
                searchBallRectang(-1.6, -1.6, -0.8, 1.6);
            } else 
            {
                // printf("...tracked!!!!\n");
                trackBall();
                newBodyTracking();
                if (cam_x != 999 && cam_y != 999 && cam_x < 0 && headTilt >= -1.55 && !dont_calibrate)
		        {
		        	initialPos_X = initialPos_Y = 0;
		        	deltaPos_X = cam_x;
		        	deltaPos_Y = cam_y;
		        }	
            }
        } else 
        {
            // motion("0"); // ganti 
            Walk(0.0, 0.0, 0.0);
            refreshMoveGrid();
            setWaktu();
            WalkSearchBallEntry = 0;
            reset_rotate = 0;
            body_tracked = false;
            RobotPositioningEntry = BallApproachEntry = 0;
            relaxEntry++;
        }

        return NodeStatus::FAILURE;
    }

    int WalkSearchBallEntry = 0;
    NodeStatus WalkSearchBall()
    {
        if (WalkSearchBallEntry > 5)
        {
            cekWaktu(2);
            if (timer)
            {
                if (role == 0)
                {
                    walkGrid(39, 0, 25);
                } else if (role == 1)
                {
                    if (robot1Status == 1)
                    {
                        walkGrid(27, 0, 25);
                    } else 
                    {
                        walkGrid(15, 0, 25);
                    }
                }
            } else 
            {
                tiltSearchBall(0.0);
                Walk(0.0, 0.0, 0.0);
                // motion("0"); // ganti
                // jalanDirection(kejarMid, 0.0, saveAngle);
            }
        } else 
        {
            setWaktu();
            saveSudutImu();
            RobotPositioningEntry = BallApproachEntry = sabar = relaxEntry = state_move_grid = 0;
            WalkSearchBallEntry++;
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus BallTracking()
    {
        // printf("...BallTracking\n");
        if (ballLost(20))
        {
            //searchBallRectang(-1.6, -1.6, -0.8, 1.6);
            //headMove(0.0, cSekarang);
            tiltSearchBall(0.0);
            delayWaitBall = 0;
        } else 
        {
            trackBall();
        }
        return NodeStatus::FAILURE;
    }

    int delayExecutor = 0;
    NodeStatus Executor()
    {
        // printf("...Executor\n");
        // printf("...seme : %d\n", ballDistance);
        // printf("...robot1dBall = %d\n...robot2dBall = %d\n...robot3dBall = %d\n...robot4dBall = %d\n...robot5dBall = %d\n", robot1DBall, robot2DBall, robot3DBall, robot4DBall, robot5DBall);
        
        if (robotNumber != 1) {
            if (robot1Status == 0 || robot1DBall == 232) { robot1DBall = 999; }
        }

        if (robotNumber != 2) {
            if (robot2Status == 0 || robot2DBall == 232) { robot2DBall = 999; }
        }

        if (robotNumber != 3) {
            if (robot3Status == 0 || robot3DBall == 232) { robot3DBall = 999; }
        }

        if (robotNumber != 4) {
            if (robot4Status == 0 || robot4DBall == 232) { robot4DBall = 999; }
        }

        if (robotNumber != 5) {
            if (robot5Status == 0 || robot5DBall == 232) { robot5DBall = 999; }
        }

        // if (lockRelax)
        // {
        //     if (delayExecutor > 50)
        //     {
        //         lockRelax = false;
        //     } else 
        //     {
        //         delayExecutor++;
        //     }
        // } else 
        // {
        //     delayExecutor = 0;
        // }

        if (useCoordination) {
            if (robotNumber == 1) {
                if (robot2State == 232 || robot3State == 232 || robot4State == 232 || robot5State == 232)
                {
                    return NodeStatus::FAILURE;
                }
                else if ( //jika jarak saya paling dekat dengan bola / saya dapat bola lebih dulu
                    ((ballDistance) < robot2DBall) &&
                    ((ballDistance) < robot3DBall) &&
                    ((ballDistance) < robot4DBall) &&
                    ((ballDistance) < robot5DBall)
                    ) {
                    // printf("...Executor::SUCCESS\n");
                    return NodeStatus::SUCCESS;
                }
            } else if (robotNumber == 2) {
                if (robot1State == 232 || robot3State == 232 || robot4State == 232 || robot5State == 232)
                {
                    return NodeStatus::FAILURE;
                }
                else if ( //jika jarak saya paling dekat dengan bola / saya dapat bola lebih dulu
                    ((ballDistance) < robot1DBall) &&
                    ((ballDistance) < robot3DBall) &&
                    ((ballDistance) < robot4DBall) &&
                    ((ballDistance) < robot5DBall)
                    ) {
                    // printf("...Executor::SUCCESS\n");
                    return NodeStatus::SUCCESS;
                }
            } else if (robotNumber == 3) {
                if (robot1State == 232 || robot2State == 232 || robot4State == 232 || robot5State == 232)
                {
                    return NodeStatus::FAILURE;
                }
                else if ( //jika jarak saya paling dekat dengan bola / saya dapat bola lebih dulu
                    ((ballDistance) < robot1DBall) &&
                    ((ballDistance) < robot2DBall) &&
                    ((ballDistance) < robot4DBall) &&
                    ((ballDistance) < robot5DBall)
                    ) {
                    // printf("...Executor::SUCCESS\n");
                    return NodeStatus::SUCCESS;
                }
            } else if (robotNumber == 4) {
                if (robot1State == 232 || robot2State == 232 || robot3State == 232 || robot5State == 232)
                {
                    return NodeStatus::FAILURE;
                }
                else if ( //jika jarak saya paling dekat dengan bola / saya dapat bola lebih dulu
                    ((ballDistance) < robot1DBall) &&
                    ((ballDistance) < robot2DBall) &&
                    ((ballDistance) < robot3DBall) &&
                    ((ballDistance) < robot5DBall)
                    ) {
                    // printf("...Executor::SUCCESS\n");
                    return NodeStatus::SUCCESS;
                }
            } else if (robotNumber == 5) {
                if (robot1State == 232 || robot2State == 232 || robot3State == 232 || robot4State == 232)
                {
                    return NodeStatus::FAILURE;
                }
                else if ( //jika jarak saya paling dekat dengan bola / saya dapat bola lebih dulu
                    ((ballDistance) < robot1DBall) &&
                    ((ballDistance) < robot2DBall) &&
                    ((ballDistance) < robot3DBall) &&
                    ((ballDistance) < robot4DBall)
                    ) {
                    // printf("...Executor::SUCCESS\n");    
                    return NodeStatus::SUCCESS;
                }
            }
        } else {
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    int lockInitPos = 0;
    bool Pickup = false, Release = false;
    NodeStatus StatePickup()
    {
        // printf("...StatePickup\n");
        if (Penalty1 == 5 && timColour1 == barelang_color || Penalty2 == 5 && timColour2 == barelang_color) {
            Pickup = true;
        }  
        if (Pickup )
        {
            FirstKicked = true;
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus StateRelease()
    {
        if (Penalty1 != 5 && timColour1 == barelang_color || Penalty2 != 5 && timColour2 == barelang_color)
        { 
            return NodeStatus::SUCCESS;
        } else 
        {
            motion("0");
        }
        return NodeStatus::FAILURE;
    }

    int delay_koordinasi = 0;
    NodeStatus Communication()
    {
        // printf("...Communication\n");
        if (delay_koordinasi > 20)
        {
            sendRobotCoordinationData(robotNumber, robotStatus, stateCondition, Grid, foundBall, ballDistance, BallGrid, robotKick);
            delay_koordinasi = 0;
        } else 
        {
            delay_koordinasi++;
        }
        return NodeStatus::SUCCESS;
    }

    int headGrid = 0;
    NodeStatus BallFoundTeam()
    {
        if (robotNumber == 1)
        {
            if (robot2FBall == 1) {headGrid = robot2GridBall; RobotPositioningEntry = 0; return NodeStatus::SUCCESS;}
            else if (robot3FBall == 1) {headGrid = robot3GridBall; RobotPositioningEntry = 0; return NodeStatus::SUCCESS;}
            else if (robot4FBall == 1) {headGrid = robot4GridBall; RobotPositioningEntry = 0; return NodeStatus::SUCCESS;}
            else if (robot5FBall == 1) {headGrid = robot5GridBall; RobotPositioningEntry = 0; return NodeStatus::SUCCESS;}
        } else if (robotNumber == 2)
        {
            if (robot1FBall == 1) {headGrid = robot1GridBall; RobotPositioningEntry = 0; return NodeStatus::SUCCESS;}
            else if (robot3FBall == 1) {headGrid = robot3GridBall; RobotPositioningEntry = 0; return NodeStatus::SUCCESS;}
            else if (robot4FBall == 1) {headGrid = robot4GridBall; RobotPositioningEntry = 0; return NodeStatus::SUCCESS;}
            else if (robot5FBall == 1) {headGrid = robot5GridBall;  RobotPositioningEntry = 0; return NodeStatus::SUCCESS;}
        } else if (robotNumber == 3)
        {
            if (robot1FBall == 1) {headGrid = robot1GridBall; RobotPositioningEntry = 0; return NodeStatus::SUCCESS;}
            else if (robot2FBall == 1) {headGrid = robot2GridBall; RobotPositioningEntry = 0; return NodeStatus::SUCCESS;}
            else if (robot4FBall == 1) {headGrid = robot4GridBall; RobotPositioningEntry = 0; return NodeStatus::SUCCESS;}
            else if (robot5FBall == 1) {headGrid = robot5GridBall; RobotPositioningEntry = 0; return NodeStatus::SUCCESS;}
        } else if (robotNumber == 4)
        {
            if (robot1FBall == 1) {headGrid = robot1GridBall;  RobotPositioningEntry = 0; return NodeStatus::SUCCESS;}
            else if (robot2FBall == 1) {headGrid = robot2GridBall;  RobotPositioningEntry = 0; return NodeStatus::SUCCESS;}
            else if (robot3FBall == 1) {headGrid = robot3GridBall;  RobotPositioningEntry = 0; return NodeStatus::SUCCESS;}
            else if (robot5FBall == 1) {headGrid = robot5GridBall; RobotPositioningEntry = 0; return NodeStatus::SUCCESS;}
        } else if (robotNumber == 5)
        {
            if (robot1FBall == 1) {headGrid = robot1GridBall;  RobotPositioningEntry = 0; return NodeStatus::SUCCESS;}
            else if (robot2FBall == 1) {headGrid = robot2GridBall;  RobotPositioningEntry = 0; return NodeStatus::SUCCESS;}
            else if (robot3FBall == 1) {headGrid = robot3GridBall;  RobotPositioningEntry = 0; return NodeStatus::SUCCESS;}
            else if (robot4FBall == 1) {headGrid = robot4GridBall;  RobotPositioningEntry = 0; return NodeStatus::SUCCESS;}
        }
        headGrid = 88;
        return NodeStatus::FAILURE;
    }

    int state_move_grid = 0, last_move = 0, cnt_move_to_grid = 0;
    double theta, sagital, lateral, walkSagital, walkLateral;
    int outTheta = 0,  diffTheta = 0;
    void new_out_grid(int targetRobotGrid, int targetRobotGridOffsetX, int targetRobotGridOffsetY, bool isGoToGrid)
    {
        double targetRobotPos_X = convertGridX(targetRobotGrid, targetRobotGridOffsetX);
        double targetRobotPos_Y = convertGridY(targetRobotGrid, targetRobotGridOffsetY);
        // printf("...msg_yaw = %d\n", msg_yaw);
        printf("...robotPos = %f, %f\n", robotPos_X, robotPos_Y);
        printf("...target = %f, %f\n", targetRobotPos_X, targetRobotPos_Y);
        // Calculate the distance between the two points
        double distance = sqrt(pow(targetRobotPos_X - robotPos_X, 2) + pow(targetRobotPos_Y - robotPos_Y, 2));
        // Calculate the angle between the two points in degrees
        theta = atan2(targetRobotPos_Y - robotPos_Y, targetRobotPos_X - robotPos_X) * 180 / PI;
        printf("...distance = %d Cm\n", (int)distance);
        printf("...angle = %d Deg\n", (int)theta);
        outTheta = int(theta) + int(msg_yaw);
        diffTheta = abs(abs((int)theta) - abs(msg_yaw));
        printf("diffTheta = %d\n", diffTheta);
        int tolerance_x = abs((int)targetRobotPos_X - (int)robotPos_X);
        int tolerance_y = abs((int)targetRobotPos_Y - (int)robotPos_Y);
        // printf("...tolerance = %d, %d\n", tolerance_x, tolerance_y);
        if (isGoToGrid)
        {
            if (tolerance_x <= 15 && tolerance_y <= 15)
            // if (distance < 30)
            {
                if (posRotateNew)
                {
                    // Walk(0.0, 0.0, 0.0);
                    motion("0");
                    printf("...SELESAI!!!!\n");
                    doneMoved = true;
                } else 
                {
                    rotateBodyImuNew(0);
                }                
            }
            else
            {   
                posRotateNew = false;
                if (cnt_move_to_grid > 10)
                {
                    // printf("...jalan theta !!! \n");
                    if (diffTheta < 30)
                    {
                        jalanDirection(kejar, 0.0, theta);
                    } else 
                    {
                        jalanDirection(0.0, 0.0, theta);
                    }
                }
                else
                {
                    reset_velocity();
                    cnt_move_to_grid++;
                }
            }
        }
    }

    double get_slope(int xG, int yG, int xB, int yB)
    {
        double slope = (xG - xB) / (yG - yB);
        return slope;
    }

    int xTar = 0, yTar = 0;
    void calculate_target(int xG, int yG, int xB, int yB)
    {   
        xTar = xB - 25;
        yTar = get_slope(xG, yG, xB, yB) * (xTar - xB) + yB;
        printf("...Target : %d, %d\n", xTar, yTar);
    }

    void kickOrient(int Grid)
    {
        if (Grid == 53) 
        {
            sudutTendang = 60;
        } else if (Grid == 52)
        {
            sudutTendang = 61;
        } else if (Grid == 51)
        {
            sudutTendang = 17;
        } else if (Grid == 50)
        {
            sudutTendang = -18;
        } else if (Grid == 49)
        {
            sudutTendang = -66;
        } else if (Grid == 48)
        {
            sudutTendang = -72;
        } else if (Grid == 47)
        {
            sudutTendang = 50;
        } else if (Grid == 46)
        {
            sudutTendang = 29;
        } else if (Grid == 41)
        {
            sudutTendang = 35;
        } else if (Grid == 40)
        {
            sudutTendang = 18;
        } else if (Grid == 35)
        {
            sudutTendang = 28;
        } else if (Grid == 34)
        {
            sudutTendang = 15;
        }else if (Grid == 29)
        {
            sudutTendang = 22;
        } else if (Grid == 28)
        {
            sudutTendang = 9;
        }else if (Grid == 43)
        {
            sudutTendang = -49;
        } else if (Grid == 42)
        {
            sudutTendang = -52;
        }else if (Grid == 37)
        {
            sudutTendang = -34;
        } else if (Grid == 36)
        {
            sudutTendang = -42;
        }else if (Grid == 31)
        {
            sudutTendang = -23;
        }else if (Grid == 30)
        {
            sudutTendang = -38;
        }else if (Grid == 25)
        {
            sudutTendang = -19;
        }else if (Grid == 24)
        {
            sudutTendang = -31;
        } else if (Grid == 50 || Grid == 44 || Grid == 38 || Grid == 32 || Grid == 26)
        {
            sudutTendang = 0;
        }else if (Grid == 51 || Grid == 45 || Grid == 39 || Grid == 33 || Grid == 27)
        {
            sudutTendang = 0;
        } else 
        {
            sudutTendang = 0;
        }
    }

    void resetCommunication()
    {
        robot1Id = 0; robot1Status = 0; robot1State = 0; robot1GridPosition = 88; robot1FBall = 0; robot1DBall = 999; robot1GridBall = 88; robot1BackIn = 0; robot1Voltage = 0;
        robot2Id = 0; robot2Status = 0; robot2State = 0; robot2GridPosition = 88; robot2FBall = 0; robot2DBall = 999; robot2GridBall = 88; robot2BackIn = 0; robot2Voltage = 0;
        robot3Id = 0; robot3Status = 0; robot3State = 0; robot3GridPosition = 88; robot3FBall = 0; robot3DBall = 999; robot3GridBall = 88; robot3BackIn = 0; robot3Voltage = 0;
        robot4Id = 0; robot4Status = 0; robot4State = 0; robot4GridPosition = 88; robot4FBall = 0; robot4DBall = 999; robot4GridBall = 88; robot4BackIn = 0; robot4Voltage = 0;
        robot5Id = 0; robot5Status = 0; robot5State = 0; robot5GridPosition = 88; robot5FBall = 0; robot5DBall = 999; robot5GridBall = 88; robot5BackIn = 0; robot5Voltage = 0;
    }

    void resetVariable()
    {
        resetCommunication();
        resetCntMoveGrid();
        refreshMoveGrid();
        reset_velocity();
        setWaktu();
        BallFoundEntry = BodyTrackEntry = BallApproachEntry = RotateToGoalEntry = KickEntry = FirstKickEntry = 0;
        delay = cnt_path = cnt_path_ = state_move_grid = stateCondition = cnt_move_to_grid = cnt_sbr = 0;
        Ball_X = Ball_Y = Goal_X = Goal_Y = -1;
        B_pole_X = T_pole_X = B_pole_Y = T_pole_Y = -1;
        done_calibrate = tracked = bodyTracked = action_afterKick = action_kick = action_relax = action_walk = doneSaveData = done_path_tracked = done_generated = doneGetData =  false;
        delay_calibrate = lockInitPos = sumWalkX = 0;
        robotPos_X = robotPos_Y = deltaPos_X = deltaPos_Y = field_x = field_y = x_pos = y_pos = 0.0;
        filtered_imu_yaw = filtered_x_vel = filtered_y_vel = 0;
        object_count = 0;
    }

    void display()
    {
    }

    void displayBT()
    {
        printf("strategy = %d, play = %d\n", msg_strategy, msg_kill);
        // printf("Grid = %d\n", Grid);
        // printf("robotPos = %f, %f\n", robotPos_X, robotPos_Y);
        printf("sudut = %d\n", msg_yaw);
        printf("head = %.2f, %.2f\n", headPan, headTilt);
        printf("head(offset) = %.2f, %.2f\n", PAN, TILT);
        printf("stateGameController = %d\n", State);
        // printf("stateCondition = %d\n", stateCondition);
        // printf("ball = %d, %d\n", Ball_X, Ball_Y);
        // printf("actual walk = %d mm, %d mm, %f m, %f m\n", vx, vy, robotWalkX, robotWalkY);
        // printf("walk active, support leg = %d, %d\n", walkActive, supportLeg);
        // printf("class id = %s\n", class_id.c_str());
        // printf("knee Current = %d\n", kneeCurr);
        // printf("stabilize state = %d\n", stabilize_state);
        // printf("second = %d\n", second);
        // printf("ObjectCount = %d\n", object_count);

        printf("Grid : %d\n", Grid);
        printf("CamPos : %.2f, %.2f\n", cam_x, cam_y);
        printf("RobotPos : %.2f, %.2f\n", robotPos_X, robotPos_Y);
        printf("BallDis : %d\n", ballDistance);
        printf("BallPos : %.2f, %.2f\n", ball_pose_x, ball_pose_y);
        printf("TarPos : %d, %d\n", xTar, yTar);
        printf("GridTarget : %d, %d, %d\n", GridTarget, GridX, GridY);
        printf("KickOff, barelang_color : %d, %d \n", KickOff, barelang_color);
        printf("Penalty1, timColour1 : %d, %d\n", Penalty1, timColour1);
        printf("Penalty2, timColour2 : %d, %d\n", Penalty2, timColour2);
        printf("Pickup : %d\n", Pickup);
        printf("Release : %d\n", Release);
        printf("FirstKicked : %d\n", FirstKicked);
        printf("sudutTendang : %d\n", sudutTendang);
        printf("role : %d\n", role);
        printf("robotKick : %d\n", robotKick);
        printf("deltaY : %f\n", deltaY);
        printf("deltaX : %f\n", deltaX);

        printf("delay_koordinasi : %d\n", delay_koordinasi);

        printf("  BarelangFC%d : Found = %d \t Dis = %d \t State = %d\n", robot1Id, robot1FBall, robot1DBall, robot1State);
        printf("  BarelangFC%d : Found = %d \t Dis = %d \t State = %d\n", robot2Id, robot2FBall, robot2DBall, robot2State);
        printf("  BarelangFC%d : Found = %d \t Dis = %d \t State = %d\n", robot3Id, robot3FBall, robot3DBall, robot3State);
        printf("  BarelangFC%d : Found = %d \t Dis = %d \t State = %d\n", robot4Id, robot4FBall, robot4DBall, robot4State);
        printf("  BarelangFC%d : Found = %d \t Dis = %d \t State = %d\n", robot5Id, robot5FBall, robot5DBall, robot5State);

        printf("\n\n");
    }

    void timer_callback()
    {
        // gridLocalization();
        Grid = coordinates_to_grid(robotPos_X, robotPos_Y);

        if (useDisplay)
        {
            displayBT();
        }

        if (msg_strategy == 4)
        {
            play = false;
            motion("8");
            stateCondition = 150;
            /*if (ballLost(20))
            {
                tiltSearchBall(0.0);
            }
            else
            {
            	hitungGerakBola();
                trackBall();
            }*/
            if (goalLost(20))
            {
                panSearchGoal(msg_yaw);
            }
            else
            {
            	trackGoal();
            }
        }
        else
        {
            if (!play)
            {
                predictGoal(msg_yaw, -1.6);
            }
        }

        if (stabilize_state == 1)
        {
            // printf("...STABILIZE!!!\n");
            if (cnt_stab > 5)
            {
                cekWaktu(1);
                if (timer)
                {
                    stabilize_state = 0;
                }
            }
            else
            {
                motion("0");
                setWaktu();
                cnt_stab++;
            }
        }
        
        /*if (cam_x != 999 && cam_y != 999 && !dont_calibrate)
		{
		   	initialPos_X = initialPos_Y = 0;
		   	if (cekArah())
		   	{
		   		deltaPos_X = cam_x * -1;
				deltaPos_Y = cam_y * -1;
		   	}
		   	else 
		   	{
				deltaPos_X = cam_x;
				deltaPos_Y = cam_y;
			}
		 }*/

        auto msg_pose = nav_msgs::msg::Odometry();
        // robotPos_X = field_x + initialPos_X;
        // robotPos_Y = field_y + initialPos_Y;
        robotPos_X = deltaPos_X + initialPos_X;
        robotPos_Y = deltaPos_Y + initialPos_Y;
        msg_pose.pose.pose.position.x = robotPos_X;
        msg_pose.pose.pose.position.y = robotPos_Y;
        msg_pose.pose.pose.position.z = 1.0;
        Odometry_->publish(msg_pose);

        zmqpp::message incoming;
        if (socket_.receive(incoming, true))
        {
            nlohmann::json j;
            j["a"] = robotPos_X;
            j["b"] = robotPos_Y;
            j["c"] = ballDistance;
            j["d"] = ball_pose_x;
            j["e"] = Pickup;
            j["f"] = play;
            j["g"] = msg_yaw;
            j["h"] = msg_strategy;
            j["i"] = role;
            j["z"] = ball_pose_y;
            string message = j.dump();

            zmqpp::message reply;
            reply << message;
            socket_.send(reply);
        }

        auto msg_ball_stat = std_msgs::msg::Bool();
        if (ballLost(20))
        {
            msg_ball_stat.data = false;
        } else 
        {
            msg_ball_stat.data = true;
        }
        ball_status_pub->publish(msg_ball_stat);

        tree.tickRoot();
        Ball_X = Pinalty_X = Goal_X = Xcross_X = -1;
        Ball_Y = Pinalty_Y = Goal_Y = Xcross_X = -1;
        b_pole_1_x = b_pole_1_y = b_pole_2_x = b_pole_2_y = t_pole_1_x = t_pole_1_y = t_pole_2_x = t_pole_2_y = cB_pole_X = cB_pole_Y = cT_pole_X = cT_pole_Y = -1;
        Left_X_Cross_X = Left_X_Cross_Y = Right_X_Cross_X = Right_X_Cross_Y = Left_T_Cross_X = Left_T_Cross_Y = Right_T_Cross_X = Right_T_Cross_Y = -1;
        Left_Corner_X = Left_Corner_Y = Right_Corner_X = Right_Corner_Y = Left_L_Cross_X = Left_L_Cross_Y = Right_L_Cross_X = Right_L_Cross_Y = -1;
        Left_T_Corner_X = Left_T_Corner_Y = Right_T_Corner_X = Right_T_Corner_Y = Robot_X = Robot_Y = goal_L_pole_X = goal_L_pole_Y = goal_R_pole_X = goal_R_pole_Y = -1;

        x_min = -1;
        x_max = -1;
        y_min = -1;
        y_max = -1;
    }

    void resetNormallSearchBall()
    {
        delayWaitBall = 0;
        saveAngle	=
		searchKe	=	//counting berapa kali search
		rotasi		=
		sabar		=
		matte		=
		tiltPos		= 0;

		posRotasi	=
		firstRotate	=
		secondRotate	=
		thirdRotate	=
		fourthRotate	=
		firstWalk	=
		secondWalk	= false;
    }

    void resetCase0()
    {
        delayWaitBall = 0;
        saveAngle	=
		searchKe	=	//counting berapa kali search
		rotasi		=
		sabar		=
		matte		=
		tiltPos		= 0;

		posRotasi	=
		firstRotate	=
		secondRotate	=
		thirdRotate	=
		fourthRotate	=
		firstWalk	=
		secondWalk	= false;
    }

    void resetCase2()
    {
        robotDirection = false;
    }

    void resetCase3()
    {
        delay = 0;
        cnt_sbr = 0;
        tendang = ballPos = false;
    }

    void resetCase4()
    {
        delay = 0;
        cnt_sbr = 0;
    }

    double coorXx, coorXy, coorYy, coorYx, sX, sY, deltaPos_X = 0, deltaPos_Y = 0;
    void mapping(double arukuX, double arukuY) // jojo
    {
        // value use minimal / 10
        if (arukuX > 0)
        {
            if (arukuX > 0 && arukuX <= 0.01)
            {
                sX = 1.54 * arukuX / 0.01; //1.6
            }
            else if (arukuX > 0.01 && arukuX <= 0.02)
            {
                sX = 2.36 * arukuX / 0.02; //2.18
            }
            else if (arukuX > 0.02 && arukuX <= 0.03)
            {
                sX = 3.58 * arukuX / 0.03; //3.46
            }
            else if (arukuX > 0.03 && arukuX <= 0.04)
            {
                sX = 4.82 * arukuX / 0.04; //4
            }
            else if (arukuX > 0.04 && arukuX <= 0.05)
            {
                sX = 5.85 * arukuX / 0.05; //5
            }
            else if (arukuX > 0.05 && arukuX <= 0.06)
            {
                sX = 6.5 * arukuX / 0.06; //5.75
            }
        }
        else if (arukuX < 0)
        {
            if (arukuX == -0.01)
            {
                sX = 0;
            }
            else if (arukuX == -0.02)
            {
                sX = -1.28;
            }
            else if (arukuX == -0.03)
            {
                sX = -1.91;
            }
            else
            {
                sX = -1.91 * arukuX / -0.03;
            }
        }
        else if (arukuX == 0)
        {
            sX = 0;
        }

        if (arukuY > 0)
        {
            //printf("POSITIF\n"); //LEFT
            if (arukuY > 0 && arukuY <= 0.01)
            {
                sY = (0 * arukuY / 0.01) / 2; //2.5
            }
            else if (arukuY > 0.01 && arukuY <= 0.02)
            {
                sY = (3.01 * arukuY / 0.02) / 2; //4
            }
            else if (arukuY > 0.02 && arukuY <= 0.03)
            {
                sY = (4.56 * arukuY / 0.03) / 2; //5	//6.54  //-2.1
            }
            else
            {
                sY = (4.56 * arukuY / 0.03) / 2; //4.85 //-2.1
            }
        }
        else if (arukuY < 0)
        {

            if (arukuY < 0 && arukuY >= -0.01)
            {
                sY = (0 * arukuY / -0.01) / 2; //2.5
            }
            else if (arukuY < -0.01 && arukuY >= -0.02)
            {
                sY = (-3.41 * arukuY / -0.02) / 2; //-4
            }
            else if (arukuY < -0.02 && arukuY >= -0.03)
            {
                sY = (-4.83 * arukuY / -0.03) / 2; //-5.11 //-2.1
            }
            else
            {
                sY = (-4.83 * arukuY / -0.03) / 2; //-5.11 //-2.1
            }
        }
        else if (arukuY == 0)
        {
            sY = 0;
        }

        // printf("robotWalk = %f, %f\n", sX, sY);
        //bagian X
        coorXx = (sX * (cos((abs(msg_yaw)) * PI / 180)));
        coorXy = (sX * (sin(msg_yaw * PI / 180)));
        //bagian Y
        if (msg_yaw <= 180 && msg_yaw >= -90)
        {
            coorYx = (sY * (cos((abs(msg_yaw - 90)) * PI / 180)));
            //coorYx = sY * cos(angleYm*PI/180);
            coorYy = sY * sin((msg_yaw - 90) * PI / 180);
        }
        else
        {
            coorYx = (sY * (cos((abs(msg_yaw + 270)) * PI / 180)));
            //coorYx = sY * cos(angleYm*PI/180);
            coorYy = (sY * (sin((msg_yaw + 270) * PI / 180)));
        }
        deltaPos_X = deltaPos_X + (coorXx * 1) + (1.1 * coorYx); //komen Yx
        deltaPos_Y = deltaPos_Y + (1.1 * coorXy) + coorYy;       //Xy = x1.1
        // printf("deltaPos = %f, %f\n", deltaPos_X, deltaPos_Y);
    }

    // Define constants for the filter
    const double alpha = 0.5;
    const double dt = 0.04;

    // Define variables for the filter
    double filtered_x_vel = 0.0;
    double filtered_y_vel = 0.0;
    double filtered_imu_yaw = 0.0;

    // Define variables for the robot's position and heading
    double field_x = 0.0;
    double field_y = 0.0;
    double x_pos = 0.0;
    double y_pos = 0.0;

    // Define the function to update the robot's position using filtered IMU readings
    void new_mapping(double x_vel, double y_vel, int imu_yaw) {
        // Apply the low-pass filter to the IMU readings
        filtered_x_vel = alpha * filtered_x_vel + (1 - alpha) * x_vel;
        filtered_y_vel = alpha * filtered_y_vel + (1 - alpha) * y_vel;
        filtered_imu_yaw = alpha * filtered_imu_yaw + (1 - alpha) * imu_yaw;

        // Convert the filtered IMU yaw from degrees to radians
        double imu_yaw_rad = filtered_imu_yaw * PI / 180.0;

        // Calculate the robot's new position based on its filtered velocity and heading
        x_pos += filtered_x_vel * cos(imu_yaw_rad) + filtered_y_vel * sin(imu_yaw_rad) * dt;
        y_pos += filtered_x_vel * sin(imu_yaw_rad) + filtered_y_vel * cos(imu_yaw_rad) * dt;

        // Map the robot's position to the RoboCup field coordinates in cm
        field_x = x_pos * 100;
        field_y = y_pos * 100;
    }

    void readTrackbar(const std_msgs::msg::Float64MultiArray::SharedPtr msg_trackbar_)
    {
        ball_panKP = goal_panKP = msg_trackbar_->data[0];
        ball_panKD = goal_panKD = msg_trackbar_->data[1];
        ball_tiltKP = goal_tiltKP = msg_trackbar_->data[2];
        ball_tiltKD = goal_tiltKD = msg_trackbar_->data[3];
    }

    void readButton(const bfc_msgs::msg::Button::SharedPtr msg_btn_)
    {
        msg_strategy = msg_btn_->strategy;
        msg_kill = msg_btn_->kill;
    }

    // Define necessary constants
    const double CUTOFF_FREQUENCY = 10.0; // Filter cutoff frequency in Hz

    // Define necessary variables
    double prev_filtered_value = 0.0;

    // Define low-pass filter function
    double low_pass_filter(double prev_filtered_value, double raw_value, double dt, double cutoff_freq) {
        double alpha = 2 * PI * cutoff_freq * dt / (2 * PI * cutoff_freq * dt + 1);
        double filtered_value = alpha * raw_value + (1 - alpha) * prev_filtered_value;
        return filtered_value;
    }

    // Define function to convert yaw angle from degrees to radians within the range of -pi to +pi
    double degrees_to_radians(double degrees) {
        double radians = degrees * PI / 180.0;
        while (radians > PI) {
            radians -= 2 * PI;
        }
        while (radians < -PI) {
            radians += 2 * PI;
        }
        return radians;
    }

    double radians_to_degrees(double radians) {
        double degrees = radians * 180.0 / M_PI;
        while (degrees > 180.0) {
            degrees -= 360.0;
        }
        while (degrees < -180.0) {
            degrees += 360.0;
        }
        return degrees;
    }

    bool robotFall = false;
    void readImu(const sensor_msgs::msg::Imu::SharedPtr msg_imu_)
    {
        msg_roll = msg_imu_->angular_velocity.x;
        msg_pitch = msg_imu_->angular_velocity.y;
        msg_yaw = msg_imu_->angular_velocity.z;

        // double filtered_value = low_pass_filter(prev_filtered_value, degrees_to_radians(msg_yaw), 0.04, CUTOFF_FREQUENCY);
        // prev_filtered_value = filtered_value;
        // // printf("... msg_yaw = %d, filtered_yaw = %.2f\n", msg_yaw, radians_to_degrees(filtered_value));
        // msg_yaw = (int)radians_to_degrees(filtered_value);

        if (msg_roll >= 45 || msg_roll <= 45 || msg_pitch >= 45 || msg_pitch <= 45)
        {
            robotFall = true;
        }
        else
        {
            robotFall = false;
        }
    }

    void objCoor(const geometry_msgs::msg::PointStamped::SharedPtr msg_bbox_)
    {
        Ball_X = msg_bbox_->point.x;
        Ball_Y = msg_bbox_->point.y;
    }

    void readGameControllerData(const std_msgs::msg::Int64MultiArray::SharedPtr msg)
    {
        State = msg->data[0]; // 0 = initial, 1 = ready, 2 = set, 3 = Play, 4 = Finish
        FirstHalf = msg->data[1];
        Version = msg->data[2];
        PacketNumber = msg->data[3];
        PlayerTeam = msg->data[4];
        // GameTipe = msg->data[7];
        KickOff = msg->data[5]; // 1 = kickoff team kita, 0 = kickoff team lawan
        SecondaryState = msg->data[6];
        DropTeam = msg->data[7];
        DropTime = msg->data[8];
        Remaining = msg->data[9];      // waktu permainan 600 = 10 menit
        SecondaryTime = msg->data[10]; // jika waktu tunggu kickoff lebih dari 10 detik dan positionning 30 detik
        // ket :    1 = msg->data[]; untuk data GameController yang kiri
        //	        2 = msg->data[]; untuk data GameController yang kanan
        timNumber1 = msg->data[11]; //
        timNumber2 = msg->data[12]; //
        timColour1 = msg->data[13]; //
        timColour2 = msg->data[14];
        Score1 = msg->data[15];
        Score2 = msg->data[16];
        Penaltyshoot1 = msg->data[17];
        Penaltyshoot2 = msg->data[18];
        Singleshoot1 = msg->data[19];
        Singleshoot2 = msg->data[20];
        // Coachsequence1 = msg->data[24];
        // Coachsequence2 = msg->data[25];

        Penalty1 = msg->data[21];
        Penalty2 = msg->data[22];
        TimeUnpenalis1 = msg->data[23];
        TimeUnpenalis2 = msg->data[24];
        // YellowCard1 = msg->data[30];
        // YellowCard2 = msg->data[31];
        // RedCard1 = msg->data[32];
        // RedCard2 = msg->data[33];
    }

    double PAN = 0.0, TILT = 0.0;
    // Head Movement =============================================================================
    void headMove(double pan, double tilt)
    {
        if (useRos)
        { // Socket
            auto msg_head_ = bfc_msgs::msg::HeadMovement();
            msg_head_.pan = ceil(pan * 100.0) / 100.0;
            msg_head_.tilt = ceil(tilt * 100.0) / 100.0;
            cmd_head_->publish(msg_head_);
        }
        else
        { // NotePad
            FILE *fp, *outputfp;
            outputfp = fopen("../Player/HeadNote", "wb");
            fprintf(outputfp, "%.2lf,%.2lf", pan, tilt);
            fclose(outputfp);
        }

        headPan = posPan = ceil(pan * 100.0) / 100.0;
        headTilt = posTilt = ceil(tilt * 100.0) / 100.0;
        PAN = headPan - 0.15;
        TILT = headTilt;
        // printf("headPan = %.2f \t headTilt = %.2f\n", pan, tilt);
    }

    // Robot Movement ============================================================================
    void motion(char line[2])
    {
        // printf("Motion = %s\n",line[2]);
        // Tendang Jauh Kiri		= 1
        // Tendang Jauh Kanan		= 2

        // Tendang Pelan Kiri		= 3
        // Tendang Pelan Kanan		= 4

        // Tendang Ke Samping Kiri	= 5
        // Tendang Ke Samping Kanan	= 6

        // Tendang WalkKick Kiri		= t
        // Tendang WalkKick Kanan	= y

        // Duduk				= 7
        // Berdiri			= 8
        // Play				= 9
        // Stop				= 0

        if (useRos)
        { // Socket
            auto msg_mot_ = std_msgs::msg::String();
            msg_mot_.data = &line[0];
            cmd_mot_->publish(msg_mot_);
        }
        else
        { // NotePad
            FILE *outputfp;
            outputfp = fopen("../Player/WalkNote", "wb");
            fprintf(outputfp, "%s", &line[0]);
            fclose(outputfp);
        }
    }

    // Walk Controller ===========================================================================
    double walkX = 0.0,
           walkY = 0.0,
           walkA = 0.0;
    double Walk(double x, double y, double a)
    {
        char line[50];

        motion("9");

        walkX = x;
        walkY = y;
        walkA = a;

        if (useRos)
        {
            auto msg_walk_ = geometry_msgs::msg::Twist();
            msg_walk_.linear.x = x + erorrXwalk;
            msg_walk_.linear.y = y + erorrYwalk;
            msg_walk_.linear.z = a + erorrAwalk;
            cmd_vel_->publish(msg_walk_);
        }
        else
        { // NotePad
            FILE *outputfp;
            strcpy(line, "walk");
            outputfp = fopen("../Player/WalkNote", "wb");
            fprintf(outputfp, "%s,%.2lf,%.2lf,%.2lf", &line[0], x + erorrXwalk, y, a); // setting jalan ditempat default
            fclose(outputfp);
            // printf("walk : %g,%g,%g\n",x,y,a);
        }

        return (walkX, walkY, walkA);
    }

    // Acceleration & Decceleration
    const double velX_Kp = 0.005, velY_Kp = 0.01, velA_Kp = 0.1;
    double velocityX = 0.0, velocityY = 0.0, velocityA = 0.0;
    void set_velocity(double vWalkX, double vWalkY, double vWalkA)
    {
        double error_X = velX_Kp * vWalkX;
        double error_Y = velY_Kp * vWalkY;
        double error_A = velA_Kp * vWalkA;
        velocityX = velocityX + error_X;
        velocityY = velocityY + error_Y;
        velocityA = velocityA + error_A;
        // if (abs(vy) > 0.02 || abs(va) > 0.3)
        // {
        //     if (vWalkX > kejarMid)
        //     {
        //         vWalkX = kejarMid; // reduce speed for stability
        //     }
        // }
        if (vWalkX > 0 && velocityX > vWalkX)
        {
            velocityX = vWalkX;
        }
        else if (vWalkX < 0 && velocityX < vWalkX)
        {
            velocityX = vWalkX;
        }
        if (vWalkY > 0 && velocityY > vWalkY)
        {
            velocityY = vWalkY;
        }
        else if (vWalkY < 0 && velocityY < vWalkY)
        {
            velocityY = vWalkY;
        }
        if (vWalkA > 0 && velocityA > vWalkA)
        {
            velocityA = vWalkA;
        }
        else if (vWalkA < 0 && velocityA < vWalkA)
        {
            velocityA = vWalkA;
        }   

        if (vWalkX == 0.0)
        {
            velocityX = 0;
        }
        // printf("ERROR = %f,%f,%f\n", error_X, error_Y, error_A);
        // printf("NEW VELOCITY = %.2f, %.2f, %.2f \n", velocityX, velocityY, velocityA);
        
        if (stabilize_state == 0)
        {
            if (cnt_stab2 <= 0)
            {
                // Walk(velocityX, velocityY, velocityA);
                Walk(velocityX, vWalkY, vWalkA);
            }
            else
            {
                Walk(0.0, 0.0, 0.0);
                reset_velocity();
                cnt_stab2--;
            }
        }
        else
        {
            reset_velocity();
        }
    }

    void reset_velocity()
    {
        velocityX = velocityY = velocityA = 0;
    }

    void declareParameters()
    {
        this->declare_parameter("robotNumber", 0);
        this->declare_parameter("frame_X", 640);
        this->declare_parameter("frame_Y", 480);
        this->declare_parameter("pPanTendangKanan", -0.20);
        this->declare_parameter("pTiltTendangKanan", -0.57);
        this->declare_parameter("pPanTendangKiri", -0.20);
        this->declare_parameter("pTiltTendangKiri", -0.57);
        this->declare_parameter("ballPositioningSpeed", 0.12);
        this->declare_parameter("cSekarang", -0.60);
        this->declare_parameter("cAktif", -1.40);
        this->declare_parameter("posTiltLocal", -1.90);
        this->declare_parameter("posTiltGoal", -1.80);
        this->declare_parameter("ball_panKP", 0.075);
        this->declare_parameter("ball_panKD", 0.0000505);
        this->declare_parameter("ball_tiltKP", 0.05);
        this->declare_parameter("ball_tiltKD", 0.0000755);
        this->declare_parameter("goal_panKP", 0.10);
        this->declare_parameter("goal_panKD", 0.000050);
        this->declare_parameter("goal_tiltKP", 0.05);
        this->declare_parameter("goal_tiltKD", 0.000050);
        this->declare_parameter("errorXwalk", 0.0);
        this->declare_parameter("errorYwalk", 0.0);
        this->declare_parameter("errorAwalk", 0.0);
        this->declare_parameter("jalan", 0.040);
        this->declare_parameter("lari", 0.050);
        this->declare_parameter("kejar", 0.060);
        this->declare_parameter("kejarMid", 0.070);
        this->declare_parameter("kejarMax", 0.089);
        this->declare_parameter("tendangJauh", 1);
        this->declare_parameter("tendangSamping", 3);
        this->declare_parameter("tendangDekat", 5);
        this->declare_parameter("sudutTengah", 0);
        this->declare_parameter("sudutKanan", 30);
        this->declare_parameter("sudutKiri", -30);
        this->declare_parameter("rotateGoal_x", -0.0005);
        this->declare_parameter("rotateGoal_y", 0.017);
        this->declare_parameter("rotateGoal_a", 0.14);
        this->declare_parameter("myAccrX", 0.3);
        this->declare_parameter("myAccrY", 0.0);
        this->declare_parameter("tinggiRobot", 48);
        this->declare_parameter("outputSudutY1", 9.0);
        this->declare_parameter("inputSudutY1", -0.40);
        this->declare_parameter("outputSudutY2", 60.0);
        this->declare_parameter("inputSudutY2", -1.43);
        this->declare_parameter("outputSudutX1", 0.0);
        this->declare_parameter("inputSudutX1", 0.0);
        this->declare_parameter("outputSudutX2", 45.0);
        this->declare_parameter("inputSudutX2", -0.8);
        this->declare_parameter("usePenaltyStrategy", false);
        this->declare_parameter("useVision", false);
        this->declare_parameter("useImu", false);
        this->declare_parameter("useRos", false);
        this->declare_parameter("useGameController", false);
        this->declare_parameter("useCoordination", false);
        this->declare_parameter("useLocalization", false);
        this->declare_parameter("useFollowSearchGoal", false);
        this->declare_parameter("useSearchGoal", false);
        this->declare_parameter("useDribble", false);
        this->declare_parameter("dribbleOnly", false);
        this->declare_parameter("useSideKick", false);
        this->declare_parameter("useLastDirection", false);
        this->declare_parameter("useNearFollowSearchGoal", false);
        this->declare_parameter("firstStateCondition", 0);
        this->declare_parameter("firstStateLocalization", 0);
        this->declare_parameter("barelang_color", 0);
        this->declare_parameter("dropball", 0);
        this->declare_parameter("team", 0);
        this->declare_parameter("useDisplay", false);
        this->declare_parameter("useWalkKick", false);
        this->declare_parameter("tree_path", "");
        this->declare_parameter("max_current", 15);
        this->declare_parameter("modePlay", 0);
        this->declare_parameter("useFollowExecutor", false);
        this->declare_parameter("forceKanan", false);
        this->declare_parameter("forceKiri", false);
        this->declare_parameter("useBodyTracking", false);
        this->declare_parameter("useKickOffGoal", false);
        this->declare_parameter("valBolaGerak", 10);
    }

    std::string tree_path = "";
    int valBolaGerak = 0;
    void getParameters()
    {
        ball_panKP = this->get_parameter("ball_panKP").as_double();
        ball_panKD = this->get_parameter("ball_panKD").as_double();
        ball_tiltKP = this->get_parameter("ball_tiltKP").as_double();
        ball_tiltKD = this->get_parameter("ball_tiltKD").as_double();
        goal_panKP = this->get_parameter("goal_panKP").as_double();
        goal_panKD = this->get_parameter("goal_panKD").as_double();
        goal_tiltKP = this->get_parameter("goal_tiltKP").as_double();
        goal_tiltKD = this->get_parameter("goal_tiltKD").as_double();
        frame_X = this->get_parameter("frame_X").as_int();
        frame_Y = this->get_parameter("frame_Y").as_int();
        robotNumber = this->get_parameter("robotNumber").as_int();
        pPanTendangKanan = this->get_parameter("pPanTendangKanan").as_double();
        pTiltTendangKanan = this->get_parameter("pTiltTendangKanan").as_double();
        pPanTendangKiri = this->get_parameter("pPanTendangKiri").as_double();
        pTiltTendangKiri = this->get_parameter("pTiltTendangKiri").as_double();
        ballPositioningSpeed = this->get_parameter("ballPositioningSpeed").as_double();
        cSekarang = this->get_parameter("cSekarang").as_double();
        cAktif = this->get_parameter("cAktif").as_double();
        posTiltLocal = this->get_parameter("posTiltLocal").as_double();
        posTiltGoal = this->get_parameter("posTiltGoal").as_double();
        erorrXwalk = this->get_parameter("errorXwalk").as_double();
        erorrYwalk = this->get_parameter("errorYwalk").as_double();
        erorrAwalk = this->get_parameter("errorAwalk").as_double();
        jalan = this->get_parameter("jalan").as_double();
        lari = this->get_parameter("lari").as_double();
        kejar = this->get_parameter("kejar").as_double();
        kejarMid = this->get_parameter("kejarMid").as_double();
        kejarMax = this->get_parameter("kejarMax").as_double();
        tendangJauh = this->get_parameter("tendangJauh").as_int();
        tendangSamping = this->get_parameter("tendangSamping").as_int();
        tendangDekat = this->get_parameter("tendangDekat").as_int();
        sudutTengah = this->get_parameter("sudutTengah").as_int();
        sudutKanan = this->get_parameter("sudutKanan").as_int();
        sudutKiri = this->get_parameter("sudutKiri").as_int();
        rotateGoal_x = this->get_parameter("rotateGoal_x").as_double();
        rotateGoal_y = this->get_parameter("rotateGoal_y").as_double();
        rotateGoal_a = this->get_parameter("rotateGoal_a").as_double();
        myAccrX = this->get_parameter("myAccrX").as_double();
        myAccrY = this->get_parameter("myAccrY").as_double();
        tinggiRobot = this->get_parameter("tinggiRobot").as_int();
        outputSudutY1 = this->get_parameter("outputSudutY1").as_double();
        inputSudutY1 = this->get_parameter("inputSudutY1").as_double();
        outputSudutY2 = this->get_parameter("outputSudutY2").as_double();
        inputSudutY2 = this->get_parameter("inputSudutY2").as_double();
        outputSudutX1 = this->get_parameter("outputSudutX1").as_double();
        inputSudutX1 = this->get_parameter("inputSudutX1").as_double();
        outputSudutX2 = this->get_parameter("outputSudutX2").as_double();
        inputSudutX2 = this->get_parameter("inputSudutX2").as_double();
        useRos = this->get_parameter("useRos").as_bool();
        usePenaltyStrategy = this->get_parameter("usePenaltyStrategy").as_bool();
        useVision = this->get_parameter("useVision").as_bool();
        useImu = this->get_parameter("useImu").as_bool();
        useGameController = this->get_parameter("useGameController").as_bool();
        useCoordination = this->get_parameter("useCoordination").as_bool();
        useLocalization = this->get_parameter("useLocalization").as_bool();
        useFollowSearchGoal = this->get_parameter("useFollowSearchGoal").as_bool();
        useSearchGoal = this->get_parameter("useSearchGoal").as_bool();
        useDribble = this->get_parameter("useDribble").as_bool();
        dribbleOnly = this->get_parameter("dribbleOnly").as_bool();
        useSideKick = this->get_parameter("useSideKick").as_bool();
        useLastDirection = this->get_parameter("useLastDirection").as_bool();
        useNearFollowSearchGoal = this->get_parameter("useNearFollowSearchGoal").as_bool();
        firstStateCondition = this->get_parameter("firstStateCondition").as_int();
        firstStateLocalization = this->get_parameter("firstStateLocalization").as_int();
        barelang_color = this->get_parameter("barelang_color").as_int();
        team = this->get_parameter("team").as_int();
        dropball = this->get_parameter("dropball").as_int();
        useDisplay = this->get_parameter("useDisplay").as_bool();
        useWalkKick = this->get_parameter("useWalkKick").as_bool();
        tree_path = this->get_parameter("tree_path").as_string();
        max_current = this->get_parameter("max_current").as_int();
        modePlay = this->get_parameter("modePlay").as_int();
        useFollowExecutor = this->get_parameter("useFollowExecutor").as_bool();
        forceKanan = this->get_parameter("forceKanan").as_bool();
        forceKiri = this->get_parameter("forceKiri").as_bool();
        useBodyTracking = this->get_parameter("useBodyTracking").as_bool();
        useKickOffGoal = this->get_parameter("useKickOffGoal").as_bool();
        valBolaGerak = this->get_parameter("valBolaGerak").as_int();
    }

    void sendRobotCoordinationData(signed short rNumber, signed short rStatus, signed short sNumber, signed short gridPos, signed short fBall, signed short dBall, signed short gridBall, signed short backIn)
    {
        auto msg = bfc_msgs::msg::Coordination();
        msg.robot_number = rNumber;
        msg.status = rStatus;
        msg.state = sNumber;
        msg.grid_position = gridPos;
        msg.found_ball = fBall;
        msg.distance_ball = dBall;
        msg.grid_ball = gridBall;
        msg.back_in = backIn;
        robotCoordination_->publish(msg);
    }

    void readRobotCoordinationData1(const bfc_msgs::msg::Coordination::SharedPtr message)
    {
        // printf("..koordinasi r 1 masuk\n");
        // resetCommunication();
        robot1Id = message->robot_number;
        robot1Status = message->status;
        robot1State = message->state;
        robot1GridPosition = message->grid_position;
        robot1FBall = message->found_ball;
        robot1DBall = message->distance_ball;
        robot1GridBall = message->grid_ball;
        robot1BackIn = message->back_in;
        robot1KickOff = message->kick_off;

        if (robot1DBall == 0)
        {
            robot1DBall = 999;
        }
    }

    void readRobotCoordinationData2(const bfc_msgs::msg::Coordination::SharedPtr message)
    {
        // printf("..koordinasi r 2 masuk\n");
        // resetCommunication();
        robot2Id = message->robot_number;
        robot2Status = message->status;
        robot2State = message->state;
        robot2GridPosition = message->grid_position;
        robot2FBall = message->found_ball;
        robot2DBall = message->distance_ball;
        robot2GridBall = message->grid_ball;
        robot2BackIn = message->back_in;
        robot2KickOff = message->kick_off;

        if (robot2DBall == 0)
        {
            robot2DBall = 999;
        }
    }

    void readRobotCoordinationData3(const bfc_msgs::msg::Coordination::SharedPtr message)
    {
        // printf("..koordinasi r 3 masuk\n");
        // resetCommunication();
        robot3Id = message->robot_number;
        robot3Status = message->status;
        robot3State = message->state;
        robot3GridPosition = message->grid_position;
        robot3FBall = message->found_ball;
        robot3DBall = message->distance_ball;
        robot3GridBall = message->grid_ball;
        robot3BackIn = message->back_in;
        robot3KickOff = message->kick_off;
    
        if (robot3DBall == 0)
        {
            robot3DBall = 999;
        }
    }

    void readRobotCoordinationData4(const bfc_msgs::msg::Coordination::SharedPtr message)
    {
        // printf("..koordinasi r 4 masuk\n");
        // resetCommunication();
        robot4Id = message->robot_number;
        robot4Status = message->status;
        robot4State = message->state;
        robot4GridPosition = message->grid_position;
        robot4FBall = message->found_ball;
        robot4DBall = message->distance_ball;
        robot4GridBall = message->grid_ball;
        robot4BackIn = message->back_in;
        robot4KickOff = message->kick_off;
    
        if (robot4DBall == 0)
        {
            robot4DBall = 999;
        }
    }

    void readRobotCoordinationData5(const bfc_msgs::msg::Coordination::SharedPtr message)
    {
        // printf("..koordinasi r 5 masuk\n");
        // resetCommunication();
        robot5Id = message->robot_number;
        robot5Status = message->status;
        robot5State = message->state;
        robot5GridPosition = message->grid_position;
        robot5FBall = message->found_ball;
        robot5DBall = message->distance_ball;
        robot5GridBall = message->grid_ball;
        robot5BackIn = message->back_in;
        robot5KickOff = message->kick_off;
    
        if (robot5DBall == 0)
        {
            robot5DBall = 999;
        }    
    }

    int voltage = 0, walkActive = 0, supportLeg = 0, lastSupportLeg = 0, kneeCurr = 0, stabilize_state = 0, cnt_stab = 0, cnt_stab2 = 0;
    int vx = 0, vy = 0, va = 0, sumWalkX = 0;
    double robotWalkX = 0.0, robotWalkY = 0.0, robotWalkA = 0.0;
    void readVoltageAndOdom(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        voltage = msg->data[0];
        walkActive = msg->data[1];
        supportLeg = msg->data[2];
        kneeCurr = msg->data[3];
        vx = msg->data[4];
        vy = msg->data[5];
        va = msg->data[6];

        robotWalkX = (double)vx / 1000;
        robotWalkY = (double)vy / 1000;
        robotWalkA = (double)va / 1000;

        if (walkActive == 1)
        {
            if (supportLeg != lastSupportLeg)
            {
                // new_mapping(robotWalkX, robotWalkY, msg_yaw);
                // mapping(robotWalkX, robotWalkY);
                mapping(robotWalkX, 0.0);
                sumWalkX += 1;
                lastSupportLeg = supportLeg;
            }
        }
        // printf("knee current = %d\n", kneeCurr);
        if (kneeCurr >= max_current)
        {
            cnt_stab = 0;
            cnt_stab2 = 50;
            stabilize_state = 1;
        }

        kneeCurr = 0;
    }

    int gridUpdate = 0;
    void readGrid(const std_msgs::msg::Int32::SharedPtr msg)
    {
        gridUpdate = msg->data;
        printf("...GridUpdate : %d\n", gridUpdate);
        robotPos_X = convertGridX(gridUpdate, 0);
        robotPos_Y = convertGridY(gridUpdate, 0);
    }

    int object_count = 0;

    void callbackFoundObject(const darknet_ros_msgs::msg::ObjectCount::SharedPtr msg)
    {
        object_count = msg->count;
        // printf("cnt = %d\n", object_count);
    }
    int x_min, x_max, y_min, y_max;
    std::string class_id;
    float b_pole_1_x, b_pole_1_y, b_pole_2_x, b_pole_2_y, cB_pole_X, cB_pole_Y;
    float t_pole_1_x, t_pole_1_y, t_pole_2_x, t_pole_2_y, cT_pole_X, cT_pole_Y;
    void callbackBoundingBox(const darknet_ros_msgs::msg::BoundingBoxes::SharedPtr msg)
    {
        std::vector<float> x_values_b, y_values_b, x_values_t, y_values_t;
        std::vector<int> id_classes_b, id_classes_t;

        for (const auto& bbox : msg->bounding_boxes)
        {
            const auto& id_class = bbox.id;
            const auto x_center = (bbox.xmin + bbox.xmax) / 2;
            const auto y_center = (bbox.ymin + bbox.ymax) / 2;

            switch (id_class)
            {
                case 0:
                    Ball_X = x_center;
                    Ball_Y = y_center;
                    break;
                case 1:
                    x_values_b.push_back(x_center);
                    y_values_b.push_back(y_center);
                    id_classes_b.push_back(id_class);
                    break;
                case 2:
                    Robot_X = x_center;
                    Robot_Y = y_center;
                    break;
                case 3:
                    Pinalty_X = x_center;
                    Pinalty_Y = y_center;
                    break;
                case 4:
                    Xcross_X = x_center;
                    Xcross_Y = y_center;
                    break;
                case 5:
                    x_values_t.push_back(x_center);
                    y_values_t.push_back(y_center);
                    id_classes_t.push_back(id_class);
                    break;
                case 6:
                    Goal_X = x_center;
                    Goal_Y = y_center;
                    break;
                case 7:
                    // Ball_X = x_center;
                    // Ball_Y = y_center;
                    break;
                case 8:
                    // Robot_X = x_center;
                    // Robot_Y = y_center;
                    break;
                case 9:
                    // goal_L_pole_X = x_center;
                    // goal_L_pole_Y = y_center;
                    break;
                case 10:
                    // goal_R_pole_X = x_center;
                    // goal_R_pole_Y = y_center;
                    break;
                default:
                    break; // ignore any other classes
            }
        }

        if (id_classes_b.size() == 2)
        {
            //RCLCPP_INFO(rclcpp::get_logger("B_pole detection"), "All 2 B_poles detected in one frame:");
            for (size_t i = 0; i < 2; ++i)
            {
                B_pole_X = x_values_b[i];
                B_pole_Y = y_values_b[i];
                if (id_classes_b[i] == 1)  // Check if the ID class is for a B_pole
                {
                    if (i == 0)
                    {
                        b_pole_1_x = x_values_b[i];
                        b_pole_1_y = y_values_b[i];
                    }
                    else
                    {
                        b_pole_2_x = x_values_b[i];
                        b_pole_2_y = y_values_b[i];
                    }

                    // RCLCPP_INFO(rclcpp::get_logger("bounding_box_detection"), "Detected B_poles: (%f, %f) and (%f, %f)", b_pole_1_x, b_pole_1_y, b_pole_2_x, b_pole_2_y);
                }

                // RCLCPP_INFO(rclcpp::get_logger("B_pole detection"), "B_pole %zu: (x, y) = (%f, %f)", i + 1, x_values_b[i], y_values_b[i]);
            }
        }

        if (id_classes_t.size() == 2)
        {
            //RCLCPP_INFO(rclcpp::get_logger("T_pole detection"), "All 2 T_poles detected in one frame:");
            for (size_t i = 0; i < 2; ++i)
            {
                T_pole_X = x_values_t[i];
                T_pole_Y = y_values_t[i];

                if (id_classes_t[i] == 5)  // Check if the ID class is for a T_pole
                {
                    if (i == 0)
                    {
                        t_pole_1_x = x_values_t[i];
                        t_pole_1_y = y_values_t[i];
                    }
                    else
                    {
                        t_pole_2_x = x_values_t[i];
                        t_pole_2_y = y_values_t[i];
                    }

                    // RCLCPP_INFO(rclcpp::get_logger("bounding_box_detection"), "Detected T_poles: (%f, %f) and (%f, %f)", t_pole_1_x, t_pole_1_y, t_pole_2_x, t_pole_2_y);
                }

                // RCLCPP_INFO(rclcpp::get_logger("T_pole detection"), "T_pole %zu: (x, y) = (%f, %f)", i + 1, x_values_t[i], y_values_t[i]);
            }
        }

        //RCLCPP_INFO(rclcpp::get_logger("bounding_box_detection"), "Detected B_poles: (%f, %f) and (%f, %f)", b_pole_1_x, b_pole_1_y, b_pole_2_x, b_pole_2_y);
        //RCLCPP_INFO(rclcpp::get_logger("bounding_box_detection"), "Detected T_poles: (%f, %f) and (%f, %f)", t_pole_1_x, t_pole_1_y, t_pole_2_x, t_pole_2_y);


        cB_pole_X = (b_pole_1_x + b_pole_2_x) / 2;
        cB_pole_Y = (b_pole_1_y + b_pole_2_y) / 2;       
        cT_pole_X = (t_pole_1_x + t_pole_2_x) / 2;
        cT_pole_Y = (t_pole_1_y + t_pole_2_y) / 2;

        //Goal_X = (cB_pole_X + cT_pole_X) / 2;
        //Goal_Y = (cB_pole_Y + cT_pole_Y) / 2;
    }

    double targetPoseX = 0.0, targetPoseY = 0.0;
    void callbackTargetPose(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        targetPoseX = msg->pose.pose.position.x;
        targetPoseY = msg->pose.pose.position.y;
    }

    double landmarkDistance = 0;
    void callbackObjectDistance(const std_msgs::msg::Float32::SharedPtr msg)
    {
        landmarkDistance = msg->data;
    }

    int ballDistance = 0;
    void callbackBallDistance(const std_msgs::msg::Float32::SharedPtr msg)
    {
        ballDistance = (int)msg->data;
        if (ballDistance == -1)
        {
            ballDistance = 999;
        }
    }

    vector<int> grid_list;
    void callbackPathFinding(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
    {
        size_t numElements = msg->data.size();
        for (int i = 0; i < numElements; i++)
        {
            grid_list.push_back(msg->data[i]);
        } 
        // print the contents of the vector
        cout << "grid_list : "  ;
        for (int i = 0; i < grid_list.size(); i++) {
            cout << grid_list[i] << " ";
        }
        cout << endl;
        cout << "size : " << grid_list.size() << endl;
    }

    double dataPanKey = 0, dataTiltKey = 0;
    bool triggerSave = false;
    void callbackTeleop(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (msg->linear.x > 0)
        {
            dataTiltKey -= 0.2;
        } else if (msg->linear.x < 0)
        {
            dataTiltKey += 0.2;
        }else if (msg->angular.z > 0)
        {
            dataPanKey += 0.2;
        } else if (msg->angular.z < 0)
        {
            dataPanKey -= 0.2;
        } else if (msg->angular.z == 0)
        {
            triggerSave = true;
        } else 
        {
            triggerSave = false;
        }
    }

    double ball_pose_x = 0, ball_pose_y = 0; 
    int BallGrid = 0, robotStatus = 0, foundBall = 0;
    void callbackBallPose(const nav_msgs::msg::Odometry::SharedPtr msg_pose)
    {
        if (msg_pose->pose.pose.position.x == -1 && msg_pose->pose.pose.position.y == -1)
        {
            BallGrid = 88;
            ball_pose_x = -1;
        	ball_pose_y = -1;
        } else 
        {
            BallGrid = coordinates_to_grid(ball_pose_x, ball_pose_y);         
        	ball_pose_x = msg_pose->pose.pose.position.x - 450;
        	ball_pose_y = msg_pose->pose.pose.position.y - 300;
        }
    }

    double cam_x = 0.0, cam_y = 0.0;
    void callbackCameraOdom(const nav_msgs::msg::Odometry::SharedPtr msg_pose)
    {
        if (msg_pose->pose.pose.position.x != -1 && msg_pose->pose.pose.position.y != -1)
        {
            cam_x = msg_pose->pose.pose.position.x;
            cam_y = msg_pose->pose.pose.position.y;
        } else 
        {
            cam_x = cam_y = 999;
        }
    }

    // Function To Set Timer =====================================================================
    struct timeval t1,
        t2;
    int attends;
    double elapsedTime,
        second;
    bool timer = false;
    void setWaktu()
    {
        elapsedTime =
            second =
                attends = 0;
        timer = false;

        gettimeofday(&t1, NULL);
    }
    // Function For Check Timer
    void cekWaktu(double detik)
    {
        if (attends > 10)
        {
            gettimeofday(&t2, NULL);

            // compute and print the elapsed time in millisec
            elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;
            elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;
            second = elapsedTime / 1000.0;
            // printf ("  waktu berlangsung = %.f detik \n\n\n\n", second);

            if (second >= detik)
            {
                timer = true;
            }
            else
            {
                timer = false;
            }
        }
        else
        {
            attends++;
        }
    }

    // Checking Lost Ball ========================================================================
    int countBallLost = 0,
        countBallFound = 0,
        returnBallVal;
    int ballLost(int threshold)
    {
        if (useVision)
        {
            if (Ball_X == -1 && Ball_Y == -1)
            {
                countBallFound = 0;
                countBallLost++;
                if (countBallLost >= threshold)
                {
                    returnBallVal = 1;
                }
            }
            else
            {
                countBallLost = 0;
                countBallFound++;
                if (countBallFound > 1)
                {
                    returnBallVal = 0;
                }
            }
        }
        else
        {
            countBallFound = 0;
            countBallLost++;
            if (countBallLost >= threshold)
            {
                returnBallVal = 1;
            }
        }
        return returnBallVal;
    }

    // Checking Lost Pen ========================================================================
    int countPenLost = 0,
        countPenFound = 0,
        returnPenVal;
    int penLost(int threshold)
    {
        if (useVision)
        {
            if (Pinalty_X == -1 && Pinalty_Y == -1)
            {
                countPenFound = 0;
                countPenLost++;
                if (countPenLost >= threshold)
                {
                    returnPenVal = 1;
                }
            }
            else
            {
                countPenLost = 0;
                countPenFound++;
                if (countPenFound > 1)
                {
                    returnPenVal = 0;
                }
            }
        }
        else
        {
            countPenFound = 0;
            countPenLost++;
            if (countPenLost >= threshold)
            {
                returnPenVal = 1;
            }
        }
        return returnPenVal;
    }

    // Search ball ===============================================================================
    double tiltRate = -0.03,
           panRate = -0.03,
           searchKe = 0,

           batasKanan = -1.6,
           batasKiri = 1.6,
           batasAtas = -2.0,
           batasBawah = -0.6;

    void tiltSearchBall(double tempPosPan)
    { // printf("  tiltSearchBall\n\n");
        posPan = tempPosPan;
        posTilt += tiltRate;

        if (posTilt <= batasAtas || posTilt >= batasBawah)
        {
            tiltRate *= -1;
        }

        if (posTilt <= batasAtas)
        {
            posTilt = batasAtas;
        }
        else if (posTilt >= batasBawah)
        {
            posTilt = batasBawah;
        }

        headMove(posPan, posTilt); // printf("posPan = %.2f \t posTilt = %.2f\n", posPan, posTilt);
    }

    void tiltPredictSearchBall(double tempPosPan)
    { // printf("  tiltSearchBall\n\n");
        posTilt += tiltRate;

        if (posTilt <= batasAtas || posTilt >= -0.8)
        {
            tiltRate *= -1;
        }

        if (posTilt <= batasAtas)
        {
            posTilt = batasAtas;
        }
        else if (posTilt >= -0.8)
        {
            posTilt = -0.8;
        }

        posPan = tempPosPan / 57.29; // sudut * nilai per satu sudut(nilai servo)

        if (posPan >= batasKiri)
        {
            posPan = batasKiri;
        }
        else if (posPan <= batasKanan)
        {
            posPan = batasKanan;
        }

        headMove(posPan, posTilt); // printf("posPan = %.2f \t posTilt = %.2f\n", posPan, posTilt);
    }

    double cnt_pan_search_ball = 0;
    void panSearchBall(double tempPosTilt)
    { // printf("  panSearchBall\n\n");
        posTilt = tempPosTilt;
        posPan += panRate;

        if (posPan <= batasKanan || posPan >= batasKiri)
        {
            panRate *= -1;
        }

        if (headPan <= (0.0 + panRate) && headPan >= (0.0 - panRate))
        {
            cnt_pan_search_ball += 0.5;
        }

        if (posPan >= batasKiri)
        {
            posPan = batasKiri;
        }
        else if (posPan <= batasKanan)
        {
            posPan = batasKanan;
        }

        headMove(posPan, posTilt); // printf("posPan = %.2f \t posTilt = %.2f\n", posPan, posTilt);
    }

    int i = 1,
        panKe = 2,
        tiltKe = 1;
    double panSearch[5] = {1.6, 0.8, 0.0, -0.8, -1.6},
           // tiltSearch1[3] = {-0.6, -1.2, -1.8},
        tiltSearch1[3] = {-0.8, -1.4, -1.8},
           tiltSearch2[2] = {-0.8, -1.4};
    void SearchBall(int mode)
    { // printf("  normalSearchBall\n\n");
        if (mode == 1)
        { // (atas-bawah)
            posTilt += tiltRate;
            if (posTilt <= batasAtas || posTilt >= batasBawah)
            {
                if (panKe == 2)
                {
                    searchKe += 1;
                }

                tiltRate *= -1;
                panKe += i;
                if (panKe >= 4 || panKe <= 0)
                {
                    i = -i;
                }
            }
            posPan = panSearch[panKe];
            printf("count pan = %d\n", panKe);
        }
        else if (mode == 2)
        { // (kiri-kanan)
            posPan += panRate;
            if (posPan <= batasKanan || posPan >= batasKiri)
            {
                if (tiltKe == 1)
                {
                    searchKe += 1;
                }
                panRate *= -1;
                tiltKe += i;

                if (tiltKe >= 2 || tiltKe <= 0)
                {
                    i = -i;
                }
            }
            posTilt = tiltSearch1[tiltKe]; // printf("count tilt = %d\n", tiltKe);
        }
        else if (mode == 3)
        { // muter-muter
            posPan += panRate;
            if (posPan <= batasKanan || posPan >= batasKiri)
            {
                panRate *= -1;
                countTilt++;
                if (countTilt > 1)
                    countTilt = 0;
            }
            posTilt = tiltSearch2[countTilt]; // printf("count tilt = %d\n", countTilt);
        }

        if (posPan >= batasKiri)
        {
            posPan = batasKiri;
        }
        else if (posPan <= batasKanan)
        {
            posPan = batasKanan;
        }
        if (posTilt <= batasAtas)
        {
            posTilt = batasAtas;
        }
        else if (posTilt >= batasBawah)
        {
            posTilt = batasBawah;
        }

        headMove(posPan, posTilt); // printf("posPan = %.2f \t posTilt = %.2f\n", posPan, posTilt);
    }

    int sabar = 0,
        tiltPos = 0;
    void threeSearchBall()
    {
        if (sabar > 7)
        {
            posPan += panRate;
            if (posPan <= batasKanan || posPan >= batasKiri)
            {
                if (tiltPos == 2 && posPan <= -1.5)
                {
                    searchKe += 1;
                }
                panRate *= -1;
                tiltPos += i;
                if (tiltPos >= 2 || tiltPos <= 0)
                {
                    i = -i;
                }
            }
            posTilt = tiltSearch1[tiltPos]; // printf("count tilt = %d\n", tiltPos);
        }
        else
        {
            posPan = 1.45;
            posTilt = -0.8;
            tiltPos = 0;
            searchKe = 0;
            i = 1;
            tiltRate = -0.05;
            panRate = -0.05;
            sabar++;
        }

        if (posPan >= batasKiri)
        {
            posPan = batasKiri;
        }
        else if (posPan <= batasKanan)
        {
            posPan = batasKanan;
        }
        if (posTilt <= batasAtas)
        {
            posTilt = batasAtas;
        }
        else if (posTilt >= batasBawah)
        {
            posTilt = batasBawah;
        }

        headMove(posPan, posTilt); // printf("posPan = %.2f \t posTilt = %.2f\n", posPan, posTilt);
    }

    void rightSearchBall()
    { // printf("  rightSearchBall\n\n");
        posTilt += tiltRate;
        if (posTilt <= batasAtas || posTilt >= batasBawah)
        {
            tiltRate *= -1;
            posPan += panRate;
            if (posPan >= 0.0 || posPan <= batasKanan)
            {
                panRate *= -1;
            }
        }

        if (posPan >= 0.0)
        {
            posPan = 0.0;
        }
        else if (posPan <= batasKanan)
        {
            posPan = batasKanan;
        }
        if (posTilt <= batasAtas)
        {
            posTilt = batasAtas;
        }
        else if (posTilt >= batasBawah)
        {
            posTilt = batasBawah;
        }

        headMove(posPan, posTilt);
    }

    void leftSearchBall()
    { // printf("  leftSearchBall\n\n");
        posTilt += tiltRate;
        if (posTilt <= batasAtas || posTilt >= batasBawah)
        {
            tiltRate *= -1;
            posPan += panRate;
            if (posPan >= batasKiri || posPan <= 0.0)
            {
                panRate *= -1;
            }
        }

        if (posPan <= 0.0)
        {
            posPan = 0.0;
        }
        else if (posPan >= batasKiri)
        {
            posPan = batasKiri;
        }
        if (posTilt <= batasAtas)
        {
            posTilt = batasAtas;
        }
        else if (posTilt >= batasBawah)
        {
            posTilt = batasBawah;
        }

        headMove(posPan, posTilt);
    }

    double invPan;
    void searchBallPan(double uPan, double uTilt)
    {
        posTilt = uTilt;
        invPan = uPan * -1;

        posPan += panRate;

        if (posPan <= invPan)
        {
            posPan = invPan;
            panRate *= -1;
        }
        else if (posPan >= uPan)
        {
            posPan = uPan;
            panRate *= -1;
        }

        headMove(posPan, posTilt);
    }

    bool neckX;
    double cnt_sbr = 0;
    void searchBallRectang(double atas, double kanan, double bawah, double kiri)
    {
        if (neckX)
        {
            posPan += panRate;
            if (posPan >= kiri || posPan <= kanan)
            {
                panRate *= -1;
                cnt_sbr += 0.5;
                neckX = false;
            }
        }
        else
        {
            posTilt += tiltRate;
            if (posTilt <= atas || posTilt >= bawah)
            {
                tiltRate *= -1;
                cnt_sbr += 0.5;
                neckX = true;
            }
        }

        if (posPan >= kiri)
        {
            posPan = kiri;
        }
        else if (posPan <= kanan)
        {
            posPan = kanan;
        }
        if (posTilt <= atas)
        {
            posTilt = atas;
        }
        else if (posTilt >= bawah)
        {
            posTilt = bawah;
        }

        headMove(posPan, posTilt); // printf("pan = %f, tilt = %f\n",posPan,posTilt);
    }

    double ballPan = 0,
           ballTilt = 0;
    void saveBallLocation()
    {
        //	trackBall();
        ballPan = posPan;
        ballTilt = posTilt;
    }

    void loadBallLocation(double tilt)
    {
        // posPan = 0.0;
        // posTilt = -0.8;
        posPan = ballPan;
        posTilt = ballTilt + tilt;
        headMove(posPan, posTilt);
    }

    int panDegree, tiltRad = 0;
    double panRad = 0;
    void savePan()
    {
        panRad = headPan;
        tiltRad = posTilt;
        panDegree = (posPan * (180 / PI)) + msg_yaw;
        panRad = panRad * -1;
        panDegree = panDegree * -1;
    }

    double koorRobotX,
        koorRobotY = 0;
    void saveKoordinatRobot()
    {
        koorRobotX = deltaPos_X;
        koorRobotY = deltaPos_Y;
    }

    void loadKoordinatRobot()
    {
        // resetOdometry();
        // deltaPos_X = koorRobotX;
        // deltaPos_Y = koorRobotY;
        initialPos_X = koorRobotX;
        initialPos_Y = koorRobotY;
    }

    int semeh = 0;
    int koordinasiJarak()
    {
        semeh = (int)(headTilt*-100);
        return semeh;
    }

    // Ball Tracking =============================================================================
    double intPanB = 0, dervPanB = 0, errorPanB = 0, preErrPanB = 0,
           PPanB = 0, IPanB = 0, DPanB = 0,
           intTiltB = 0, dervTiltB = 0, errorTiltB = 0, preErrTiltB = 0,
           PTiltB = 0, ITiltB = 0, DTiltB = 0,
           dtB = 0.04;
    double B_Pan_err_diff, B_Pan_err, B_Tilt_err_diff, B_Tilt_err, B_PanAngle, B_TiltAngle,
        pOffsetB, iOffsetB, dOffsetB,
        errorPanBRad, errorTiltBRad,
        offsetSetPointBall;
    void trackBall()
    {
        if (useVision)
        {
            if (Ball_X != -1 && Ball_Y != -1)
            { // printf("Tracking");
                // mode 1 ######################################################################
                /*// PID pan ==========================================================
                errorPanB  = (double)Ball_x - (frame_X / 2);//160
                PPanB  = errorPanB  * 0.00010; // Tune in Kp Pan 0.00035 //kalau kepala msh goyang2, kurangin nilainya

                intPanB += errorPanB * dtB;
                IPanB = intPanB * 0.0;

                dervPanB = (errorPanB - preErrPanB) / dtB;
                DPanB = dervPanB * 0.00001;

                preErrPanB = errorPanB;

                //posPan += PPanB*-1; //dikali -1 kalau receler terbalik dalam pemasangan
                posPan += (PPanB + IPanB + DPanB) * -1;


                // PID tilt ==========================================================
                errorTiltB = (double)Ball_y - (frame_Y / 2);//120
                PTiltB = errorTiltB * 0.00010; // Tune in Kp Tilt 0.00030

                intTiltB += errorTiltB * dtB;
                ITiltB = intTiltB * 0.0;

                dervTiltB = (errorTiltB - preErrTiltB) / dtB;
                DTiltB = dervTiltB * 0.00001;

                preErrTiltB = errorTiltB;

                //posTilt += PTiltB;
                posTilt += (PTiltB + ITiltB + DTiltB);*/

                // mode 2 ######################################################################
                //  offsetSetPointBall = ((int)(posTilt * 30)+7); //+54
                //  if (offsetSetPointBall > 36) offsetSetPointBall = 36;
                //  else if (offsetSetPointBall < 0) offsetSetPointBall = 0;

                // errorPanB  = (double)Ball_X - ((frame_X / 2) + offsetSetPointBall);//160
                errorPanB = (double)Ball_X - (frame_X / 2);
                errorTiltB = (double)Ball_Y - (frame_Y / 2); // 120
                errorPanB *= -1;
                errorTiltB *= -1;
                errorPanB *= (90 / (double)frame_X);  // pixel per angle
                errorTiltB *= (60 / (double)frame_Y); // pixel per angle
                // errorPanB *= (77.32 / (double)frame_X); // pixel per angle
                // errorTiltB *= (61.93 / (double)frame_Y); // pixel per angle

                errorPanBRad = (errorPanB * PI) / 180;
                errorTiltBRad = (errorTiltB * PI) / 180;
                // printf("errorPan = %.2f \t errorTilt = %.2f\n", errorPanB, errorTiltB);
                // printf("RadrrorPan = %.2f \t RaderrorTilt = %.2f\n", errorPanBRad, errorTiltBRad);
                // printf("KPPan = %f \t KDPan = %f\t", kamera.panKP, kamera.panKD); printf("KPTilt = %f \t KDTilt = %f\n", kamera.tiltKP, kamera.tiltKD);

                B_Pan_err_diff = errorPanBRad - B_Pan_err;
                B_Tilt_err_diff = errorTiltBRad - B_Tilt_err;

                // PID pan ==========================================================
                // PPanB  = B_Pan_err  * kamera.panKP; // Tune in Kp Pan 0.00035 //kalau kepala msh goyang2, kurangin nilainya
                PPanB = B_Pan_err * ball_panKP; // Tune in Kp Pan 0.00035 //kalau kepala msh goyang2, kurangin nilainya
                intPanB += B_Pan_err * dtB;
                IPanB = intPanB * 0.0;
                dervPanB = B_Pan_err_diff / dtB;
                // DPanB = dervPanB * kamera.panKD;
                DPanB = dervPanB * ball_panKD;
                B_Pan_err = errorPanBRad;
                posPan += (PPanB + IPanB + DPanB);

                // PID tilt ==========================================================
                // PTiltB = B_Tilt_err * kamera.tiltKP; // Tune in Kp Tilt 0.00030
                PTiltB = B_Tilt_err * ball_tiltKP; // Tune in Kp Tilt 0.00030

                intTiltB += B_Tilt_err * dtB;
                ITiltB = intTiltB * 0.0;

                dervTiltB = B_Tilt_err_diff / dtB;
                // DTiltB = dervTiltB * kamera.tiltKD;
                DTiltB = dervTiltB * ball_tiltKD;

                preErrTiltB = errorTiltB;
                B_Tilt_err = errorTiltBRad;
                posTilt += (PTiltB + ITiltB + DTiltB) * -1;

                if (posPan >= 1.6)
                {
                    posPan = 1.6;
                }
                else if (posPan <= -1.6)
                {
                    posPan = -1.6;
                }
                if (posTilt <= -2.0)
                {
                    posTilt = -2.0;
                }
                else if (posTilt >= -0.4)
                {
                    posTilt = -0.4;
                }

                headMove(posPan, posTilt); // printf("posPan = %.2f \t posTilt = %.2f\n", posPan, posTilt);

                koordinasiJarak();
                saveBallLocation();
            }
        }
    }

    // Pen Tracking =============================================================================
    double intPanP = 0, dervPanP = 0, errorPanP = 0, preErrPanP = 0,
        PPanP = 0, IPanP = 0, DPanP = 0,
        intTiltP = 0, dervTiltP = 0, errorTiltP = 0, preErrTiltP = 0,
        PTiltP = 0, ITiltP = 0, DTiltP = 0,
        dtP = 0.04;

    double P_Pan_err_diff, P_Pan_err, P_Tilt_err_diff, P_Tilt_err, P_PanAngle, P_TiltAngle,
        pOffsetP, iOffsetP, dOffsetP,
        errorPanPRad, errorTiltPRad,
        offsetSetPointPall;

    void trackPen()
    {
        if (useVision)
        {
            if (Pinalty_X != -1 && Pinalty_Y != -1)
            { // printf("Tracking");
                // mode 1 ######################################################################
                /*// PID pan ==========================================================
                errorPanP  = (double)Pinalty_X - (frame_X / 2);//160
                PPanP  = errorPanP  * 0.00010; // Tune in Kp Pan 0.00035 //kalau kepala msh goyang2, kurangin nilainya

                intPanP += errorPanP * dtB;
                IPanP = intPanP * 0.0;

                dervPanP = (errorPanP - preErrPanP) / dtB;
                DPanP = dervPanP * 0.00001;

                preErrPanP = errorPanP;

                //posPan += PPanP*-1; //dikali -1 kalau receler terbalik dalam pemasangan
                posPan += (PPanP + IPanP + DPanP) * -1;


                // PID tilt ==========================================================
                errorTiltP = (double)Pinalty_Y - (frame_Y / 2);//120
                PTiltB = errorTiltP * 0.00010; // Tune in Kp Tilt 0.00030

                intTiltP += errorTiltP * dtB;
                ITiltB = intTiltP * 0.0;

                dervTiltP = (errorTiltP - preErrTiltP) / dtB;
                DTiltB = dervTiltP * 0.00001;

                preErrTiltP = errorTiltP;

                //posTilt += PTiltB;
                posTilt += (PTiltB + ITiltB + DTiltB);*/

                // mode 2 ######################################################################
                //  offsetSetPointBall = ((int)(posTilt * 30)+7); //+54
                //  if (offsetSetPointBall > 36) offsetSetPointBall = 36;
                //  else if (offsetSetPointBall < 0) offsetSetPointBall = 0;

                // errorPanP  = (double)Pinalty_X - ((frame_X / 2) + offsetSetPointBall);//160
                errorPanP = (double)Pinalty_X - (frame_X / 2);
                errorTiltP = (double)Pinalty_Y - (frame_Y / 2); // 120
                errorPanP *= -1;
                errorTiltP *= -1;
                errorPanP *= (90 / (double)frame_X);  // pixel per msg_yaw
                errorTiltP *= (60 / (double)frame_Y); // pixel per msg_yaw
                // errorPanP *= (77.32 / (double)frame_X); // pixel per msg_yaw
                // errorTiltP *= (61.93 / (double)frame_Y); // pixel per msg_yaw

                errorPanPRad = (errorPanP * PI) / 180;
                errorTiltPRad = (errorTiltP * PI) / 180;
                // printf("errorPan = %.2f \t errorTilt = %.2f\n", errorPanP, errorTiltP);
                // printf("RadrrorPan = %.2f \t RaderrorTilt = %.2f\n", errorPanPRad, errorTiltPRad);
                // printf("KPPan = %f \t KDPan = %f\t", kamera.panKP, kamera.panKD); printf("KPTilt = %f \t KDTilt = %f\n", kamera.tiltKP, kamera.tiltKD);

                P_Pan_err_diff = errorPanPRad - P_Pan_err;
                P_Tilt_err_diff = errorTiltPRad - P_Tilt_err;

                // PID pan ==========================================================
                // PPanP  = P_Pan_err  * kamera.panKP; // Tune in Kp Pan 0.00035 //kalau kepala msh goyang2, kurangin nilainya
                PPanP = P_Pan_err * ball_panKP; // Tune in Kp Pan 0.00035 //kalau kepala msh goyang2, kurangin nilainya
                intPanP += P_Pan_err * dtP;
                IPanP = intPanP * 0.0;
                dervPanP = P_Pan_err_diff / dtP;
                // DPanP = dervPanP * kamera.panKD;
                DPanP = dervPanP * ball_panKD;
                P_Pan_err = errorPanPRad;
                posPan += (PPanP + IPanP + DPanP);

                // PID tilt ==========================================================
                // PTiltP = P_Tilt_err * kamera.tiltKP; // Tune in Kp Tilt 0.00030
                PTiltP = P_Tilt_err * ball_tiltKP; // Tune in Kp Tilt 0.00030

                intTiltP += P_Tilt_err * dtP;
                ITiltP = intTiltP * 0.0;

                dervTiltP = P_Tilt_err_diff / dtP;
                // DTiltP = dervTiltP * kamera.tiltKD;
                DTiltP = dervTiltP * ball_tiltKD;

                preErrTiltP = errorTiltP;
                P_Tilt_err = errorTiltPRad;
                posTilt += (PTiltP + ITiltP + DTiltP) * -1;

                if (posPan >= 1.6)
                {
                    posPan = 1.6;
                }
                else if (posPan <= -1.6)
                {
                    posPan = -1.6;
                }
                if (posTilt <= -2.0)
                {
                    posTilt = -2.0;
                }
                else if (posTilt >= -0.4)
                {
                    posTilt = -0.4;
                }

                headMove(posPan, posTilt); // printf("posPan = %.2f \t posTilt = %.2f\n", posPan, posTilt);

            }
        }
    }


    // Body Tracking Ball ========================================================================
    double errorBodyPosition,
        bodyP_Controller;
    int bodyTrue = 0,
        delayTrue = 0;
    int bodyTrackingBall(int threshold)
    {
        errorBodyPosition = 0 - headPan;
        bodyP_Controller = errorBodyPosition * -0.5; //-0.5

        if (ballLost(20))
        {
            Walk(0.0, 0.0, 0.0);
            bodyP_Controller = bodyTrue = delayTrue = 0;
        }
        else
        {
            if (errorBodyPosition >= -0.3 && errorBodyPosition <= 0.3)
            { // untuk hasil hadap 0.8
                // motion("0");
                Walk(0.0, 0.0, 0.0);
                delayTrue++;
            }
            else
            {
                trackBall();

                if (bodyP_Controller < 0)
                {
                    bodyP_Controller = -0.2;
                } // kanan 0.15
                else
                {
                    bodyP_Controller = 0.2;
                } // kiri 0.15

                bodyTrue = delayTrue = 0;
                Walk(0.0, 0.0, bodyP_Controller);
            }

            if (delayTrue >= threshold)
            {
                bodyTrue = 1;
            }
            else
            {
                bodyTrue = 0;
            }
        } // printf("Body Error = %.2f\t Body P Controller = %.2f\n",errorBodyPosition,bodyP_Controller);
        return bodyTrue;
    }

    // Hitung Jarak Bola berdasarkan headTilt ==============================================================
    double alphaY, // hasil derajat ketika headTilt
        betaY,
        inputY, // nilai realtime headTilt

        alphaX, // hasil derajat ketika headPan
        betaX,
        inputX, // nilai realtime headPan

        jarakBola_Y, // hasil jarak(cm) dari kalkulasi headTilt
        jarakBola_X, // hasil jarak(cm) dari kalkulasi headPan
        jarakBola;

    void kalkulasiJarakBola()
    {
        inputY = headTilt;
        inputX = headPan;

        // alphaY = -57.29 * headTilt; //(metode 1)
        alphaY = outputSudutY1 + ((outputSudutY2 - outputSudutY1) / (inputSudutY2 - inputSudutY1)) * (inputY - inputSudutY1); //(metode 2) //printf("  alphaY = %.2f,", alphaY);
        betaY = 180 - (90 + alphaY);                                                                                          // printf("  betaY = %.2f,", betaY);

        // alphaX = -57.29 * headPan; //(metode 1)
        alphaX = outputSudutX1 + ((outputSudutX2 - outputSudutX1) / (inputSudutX2 - inputSudutX1)) * (inputX - inputSudutX1); //(metode 2) //printf("  alphaX = %.2f,", alphaX);
        betaX = 180 - (90 + alphaX);                                                                                          // printf("  betaX = %.2f,", betaX);

        // sin & cos dalam c++ adalah radian, oleh karena itu harus : sin( ... * PI / 180)
        jarakBola_Y = (tinggiRobot * sin(alphaY * PI / 180)) / sin(betaY * PI / 180); // printf("  jarakBola_Y = %.f,", jarakBola_Y);
        jarakBola_X = (jarakBola_Y * sin(alphaX * PI / 180)) / sin(betaX * PI / 180); // printf("  jarakBola_X = %.f\n\n\n\n", jarakBola_X);
        jarakBola = sqrt((jarakBola_Y * jarakBola_Y) + (jarakBola_X * jarakBola_X));

        // regresi (metode 3)
        // jarakBola_Y = (-933.9*(pow(posTilt,5))) + (-5340.8*(pow(posTilt,4))) + (-12018*(pow(posTilt,3))) + (-13183*(pow(posTilt,2))) + (-7050.2*posTilt) - 1454.3;
    }

    // Untuk Kalkulasi Posisi P1
    double P1_X, P1_Y;
    void hitungKoordinatBolaP1()
    {
        // mode1--------------
        // kalkulasiJarakBola();
        // P1_X = jarakBola_X;
        // P1_Y = jarakBola_Y;
        // mode2--------------
        P1_X = Ball_Y;
        P1_Y = Ball_X;
        // printf("  P1_X = %.2f,  P1_Y = %.2f,", P1_X, P1_Y);
    }

    // Untuk Kalkulasi Posisi P2
    double P2_X, P2_Y;
    void hitungKoordinatBolaP2()
    {
        // mode1--------------
        // kalkulasiJarakBola();
        // P2_X = jarakBola_X;
        // P2_Y = jarakBola_Y;
        // mode2--------------
        P2_X = Ball_Y;
        P2_Y = Ball_X;
        // printf("  P2_X = %.2f,  P2_Y = %.2f,", P2_X, P2_Y);
    }

    // Untuk penyebut dan pembilang gradient
    double deltaY = 0, deltaX = 0;
    void hitungDeltaY()
    {
        deltaY = P2_Y - P1_Y; // mendekat positif
        // deltaY = P1_Y - P2_Y; //mendekat positif
        // printf("  deltaY = %.2f,", deltaY);
    }

    void hitungDeltaX()
    {
        deltaX = P2_X - P1_X; // mendekat negatif
        // deltaX = P1_X - P2_X; //mendekat positif
        // printf("  deltaX = %.2f,\n\n", deltaX);
    }

    // Untuk Gradient
    double gradient;
    void hitungGradient()
    {
        gradient = deltaY / deltaX;
        // printf("  gradient = %.2f,", gradient);
    }

    int cntOke1,
        cntOke2,
        cntUlang,
        kondisiBola = 0;
    bool oke = true,
         ulang = false;
    void hitungGerakBola()
    { // Bagian pengecekan pergerakan bola
        if (Ball_X == -1 && Ball_Y == -1)
        { // bola hilang
            deltaY =
                deltaX =
                    jarakBola_X =
                        jarakBola_Y = 0;
        }

        if (ulang)
        { // printf("ulang \n");
            cntOke1 =
                cntOke2 = 0;
            if (cntUlang > 5)
            {
                ulang = false;
                oke = true;
            }
            else
            {
                deltaY =
                    deltaX =
                        jarakBola_X =
                            jarakBola_Y = 0;

                cntUlang++;
            }
        }

        if (oke)
        { // printf("oke \n");
            cntUlang = 0;
            if (cntOke1 > 5)
            {
                hitungKoordinatBolaP2();

                hitungDeltaY();
                hitungDeltaX();
                // hitungGradient();

                if (cntOke2 > 10)
                {
                    oke = false;
                    ulang = true;
                }
                else
                {
                    cntOke2++;
                }
            }
            else
            {
                hitungKoordinatBolaP1();
                cntOke1++;
            }
        }

        // if ((deltaY >= 20 && deltaX <= -20) || (deltaY >= 20 && deltaX >= 20)) { //0.5 //7.0
        if (deltaY >= 10)
        { // 0.5 //7.0 // 30
            // printf("  deltaY = %.f,", deltaY);
            // printf("  Bola Menjauh\n");
            kondisiBola = 1;
            //} else if((deltaY <= -20 && deltaX <= -20) || (deltaY <= -20 && deltaX >= 20)) { //-2 //-1.4
        }
        else if (deltaY <= -10)
        { //-2 //-1.4 // -30
            // printf("  deltaY = %.f,", deltaY);
            // printf("  Bola Mendekat\n");
            kondisiBola = -1;
        }
        else if (deltaY >= -0.3 && deltaY <= 0.3 && deltaX >= -0.3 && deltaX <= 0.3)
        { //-2 //-1.4 // -5
            // printf("  Bola Diam");
            kondisiBola = 0;
        }
    }

    // Follow Ball ===============================================================================
    int	countReadyKick;
    double	SetPointPan = 0,
        SetPointTilt = -0.8,//-0.08
        errorfPan,
        errorfTilt,
        PyMove = 0,
        PxMove = 0,
        PaMove = 0;
    void followBall(int mode){ //0 normal, 1 sambil belok
        // trackBall();

        if	(posTilt >= SetPointTilt) { posTilt = SetPointTilt; }
        else if	(posTilt < -2.0) { posTilt = -2.0; }

        errorfPan  = posPan - SetPointPan;
        errorfTilt = posTilt - SetPointTilt;

        if (posTilt >= SetPointTilt && posPan < 0.4 && posPan > -0.4 && Ball_X != -1 && Ball_Y != -1) { //Stop(bola sudah dekat)
            countReadyKick++;
        } else { //Kejar Bola(bola masih jauh)
            countReadyKick = 0;
        }

        if (countReadyKick >= 1) { //5
            PxMove = 0.0; //jalan ditempat
            PyMove = errorfPan * 0.040; //0.045
            PaMove = errorfPan * 0.20; //0.30; //0.045
        } else {
            if (headTilt < -1.5) {
                PxMove = kejarMax; //0.08
            } else if (headTilt >= -1.5 && headTilt < -1.4) {
                PxMove = kejarMid; //0.07
            } else if (headTilt > -1.0) {
                PxMove = lari; //0.05
            } else {
                PxMove = kejar; //0.06
            }
            //PxMove = errorfTilt * 0.1 * -13; //Robot besar 0.13, robot kecil 0.1
            // PxMove = 0.06 / -1.6 * posTilt; //0.04-0.06
            PyMove = errorfPan * 0.50;//0.125; //0.045
            PaMove = errorfPan * 0.45;//0.25; //0.35; //0.045
        }

        if (mode == 0) { // Mode differential walking
            if (errorfPan > -0.4 && errorfPan < 0.4) { 	//printf("AAAAAAAA\n");
                Walk(PxMove, 0.0, PaMove);
            } else { 					//printf("BBBBBBBB\n");
                Walk(0.0, 0.0, PaMove);
            }
        }
        else if (mode == 1) { // Mode omnidirectional walking
            if (errorfPan > -0.4 && errorfPan < 0.4) { 	//printf("CCCCCCCC\n");
                Walk(PxMove, PyMove, PaMove);
            } else { 					//printf("DDDDDDDD\n");
                Walk(0.0, 0.0, PaMove);
            }
        }
    }

    int reset_body = 0;
    double last_pan = 0;
    bool body_tracked = false;
    void newBodyTracking()
    {
        //trackBall();
        if (!body_tracked)
        {
            if (posPan < 0.04 && posPan > -0.04)
            { // Stop(bola sudah dekat)
                motion("0");
                last_pan = posPan;
                body_tracked = true;
            }
            else
            { // Kejar Bola(bola masih jauh)
                errorfPan = posPan - SetPointPan;
                PyMove = errorfPan * 0.20; // 0.125; //0.045
                PaMove = errorfPan * 0.30; // 0.25; //0.35; //0.045

                // Define smoothing factor
                double alpha = 0.2;

                // Initialize smoothed velocities
                double smoothedPxMove = PxMove;
                double smoothedPyMove = PyMove;
                double smoothedPaMove = PaMove;

                // Calculate smoothed velocities using EMA algorithm
                smoothedPxMove = alpha * PxMove + (1 - alpha) * smoothedPxMove;
                smoothedPyMove = alpha * PyMove + (1 - alpha) * smoothedPyMove;
                smoothedPaMove = alpha * PaMove + (1 - alpha) * smoothedPaMove;

                Walk(0.0, 0.0, smoothedPaMove);
                printf("...rotate\n");
            }
        }
        else
        {
            if (abs(last_pan - posPan) > 45 * PI/180.0)
            {
                body_tracked = false;
            }
        }
    }

    bool doneGkBack = false;
    bool y_done = false, x_done = false;
    int cntBackPos = 0;
    void gkBackPos()
    {
        double y_move = (abs(odom_pose_y) * 0.15) / 0.5;

        if (posRotateNew)
        {
            if (odom_pose_y >= -0.1 && odom_pose_y <= 0.1)
            {
                doneGkBack = true;
            }
            else
            {
                if (cntBackPos > 30)
                {
                    if (odom_pose_y > 0)
                    {
                        jalanDirection(0.0, -y_move, 0.0);
                    }
                    else
                    {
                        jalanDirection(0.0, y_move, 0.0);
                    }
                }
                else
                {
                    jalanDirection(0.0, 0.0, 0.0);
                    cntBackPos++;
                }
            }
        }
        else
        {
            rotateBodyImuNew(0);
        }
    }

    int maxY = 0, maxX = 0, minY = 0, minX = 0, lastKoorX = 0;
    bool doneBanting = false, keeperNoTrack = false;
    ;
    void banting()
    {
        if (ballLost(20))
        {
            //Walk(0.0, 0.0, 0.0);
            delayWaitBall = 0;
            threeSearchBall();
            ballPos = robotDirection = false;
            tunggu = lastKoorX = minX = minY = maxX = maxY = 0;
        }
        else
        {
            if (delayWaitBall > 30)
            {
                loadBallLocation(0.2);
                // hitungGerakBola();

                if (deltaX >= maxX)
                {
                    maxX = deltaX;
                }

                if (deltaY <= minY)
                {
                    minY = deltaY;
                }

                if (deltaX <= minX)
                {
                    minX = deltaX;
                }

                if (deltaY >= maxY)
                {
                    maxY = deltaY;
                }

                if (tunggu > 20)
                {
                    if (Ball_Y >= frame_Y / 2)
                    {
                        if (Ball_X > lastKoorX + 35)
                        {
                            motion("4");
                            //headMove(-1.6, -1.6);
                            doneBanting = true;
                        }
                        else if (Ball_X < lastKoorX - 35)
                        {
                            motion("3");
                            //headMove(1.6, -1.6);
                            doneBanting = true;
                        }
                        else
                        {
                            // motion("7");
                            headMove(0.0, -0.6);
                            doneBanting = true;
                        }
                    }
                }
                else
                {
                    lastKoorX = Ball_X;
                    tunggu++;
                }
            }
            else
            {
                // trackBall();
                keeperNoTrack = true;
                saveBallLocation();
                delayWaitBall++;
            }
        }
    }

    // IMU ===========================================================================
    int setPoint1,
        setPoint2;

    double errorCPosPan,
        errorCPosTilt,
        alfaImu,
        bodyYImu,
        bodyXImu;

    //	X+ = maju
    //	X- = mundur
    //	Y+ = samping kiri
    //	Y- = samping kanan
    //	A+ = putar kiri
    //	A- = putar kanan

    void rotateDirec(int arah, double jarakTilt)
    {
        errorCPosPan = posPan;                 // adalah nilai tengah pan, dan menjadi titik berhenti jika telah tepenuhi
        errorCPosTilt = posTilt - (jarakTilt); //-0.45 adalah nilai tengah tilt, robot akan jalan ditempat(tidak maju/mundur) jika nilai terpenuhi

        bodyXImu = errorCPosTilt * (-0.04);     // nilai pengali ini harus tetap bernilai negatif //besarnya kalkulasi maju/mundur yg dibutuhkan tehadap posTilt
        bodyYImu = abs(errorCPosPan / 100) + 0.017; // 0.017;
        alfaImu = errorCPosPan * 0.7;               // 0.7; nilai pengali ini harus tetap bernilai positif //besarnya kalkulasi rotate yg dibutuhkan tehadap posPan

        if (arah <= 0)
        { // rotate ke kanan
            // alfaImu = -0.15; bodyYImu = 0.017;
            if (bodyYImu < 0)
            {
                bodyYImu = -bodyYImu;
            }
        }
        else
        { // rotate ke kiri
            // alfaImu = 0.15; bodyYImu = -0.017;
            if (bodyYImu > 0)
            {
                bodyYImu = -bodyYImu;
            }
        }
    }

    void rotateParabolic(int arah, double jarakTilt)
    {
        rotateDirec(arah, jarakTilt);
        Walk(bodyXImu, bodyYImu, alfaImu);
    }

    bool robotDirection = false;
    int eleh = 0,
        btsRotate = 0,
        btsSetPoint = 0;
    void Imu(int gawang, double jarakTilt)
    { // opsi 2, menggunakan "mode" untuk mengecek robot direction
        // trackBall();

        if (gawang > 180)
        {
            gawang = 180;
        }
        else if (gawang < -180)
        {
            gawang = -180;
        }

        setPoint1 = 10 + gawang;  // 10 //15 //20 //60
        setPoint2 = -10 + gawang; // 10 //15 //20 //60

        if (setPoint1 > 180 || setPoint2 < -180)
        { // jika arah imu dibelakang
            if (setPoint1 > 180)
            { // nilai setpoint1 diubah jd negatif
                btsSetPoint = setPoint1 - 360;

                if (msg_yaw >= setPoint2 || msg_yaw <= btsSetPoint)
                {
                    bodyXImu = bodyYImu = alfaImu = 0.0;
                    if (eleh > 10)
                    {
                        robotDirection = true;
                    }
                    else
                    {
                        eleh++;
                    }
                }
                else
                {
                    if (gawang >= 0)
                    {
                        btsRotate = gawang - 180;
                        if ((msg_yaw <= gawang) && (msg_yaw >= btsRotate))
                        {
                            rotateDirec(-1, jarakTilt); // printf("  rotate ke kanan\n\n");
                        }
                        else
                        {
                            rotateDirec(1, jarakTilt); // printf("  rotate ke kiri\n\n");
                        }
                    }
                    else
                    {
                        btsRotate = gawang + 180;
                        if ((msg_yaw >= gawang) && (msg_yaw <= btsRotate))
                        {
                            rotateDirec(1, jarakTilt); // printf("  rotate ke kiri\n\n");
                        }
                        else
                        {
                            rotateDirec(-1, jarakTilt); // printf("  rotate ke kanan\n\n");
                        }
                    }

                    eleh = 0;
                    robotDirection = false;
                }
                Walk(bodyXImu, bodyYImu, alfaImu);
            }
            else
            { // nilai setPoint2 diubah jadi positif
                btsSetPoint = setPoint2 + 360;

                if (msg_yaw >= btsSetPoint || msg_yaw <= setPoint1)
                {
                    bodyXImu = bodyYImu = alfaImu = 0.0;
                    if (eleh > 10)
                    {
                        robotDirection = true;
                    }
                    else
                    {
                        eleh++;
                    }
                }
                else
                {
                    if (gawang >= 0)
                    {
                        btsRotate = gawang - 180;
                        if ((msg_yaw <= gawang) && (msg_yaw >= btsRotate))
                        {
                            rotateDirec(-1, jarakTilt); // printf("  rotate ke kanan\n\n");
                        }
                        else
                        {
                            rotateDirec(1, jarakTilt); // printf("  rotate ke kiri\n\n");
                        }
                    }
                    else
                    {
                        btsRotate = gawang + 180;
                        if ((msg_yaw >= gawang) && (msg_yaw <= btsRotate))
                        {
                            rotateDirec(1, jarakTilt); // printf("  rotate ke kiri\n\n");
                        }
                        else
                        {
                            rotateDirec(-1, jarakTilt); // printf("  rotate ke kanan\n\n");
                        }
                    }

                    eleh = 0;
                    robotDirection = false;
                }
                Walk(bodyXImu, bodyYImu, alfaImu);
            }
        }
        else
        { // arah imu kedepan
            if (msg_yaw >= setPoint2 && msg_yaw <= setPoint1)
            {
                bodyXImu = bodyYImu = alfaImu = 0.0;
                if (eleh > 10)
                {
                    robotDirection = true;
                }
                else
                {
                    eleh++;
                }
            }
            else
            {
                if (gawang >= 0)
                {
                    btsRotate = gawang - 180;
                    if ((msg_yaw <= gawang) && (msg_yaw >= btsRotate))
                    {
                        rotateDirec(-1, jarakTilt); // printf("  rotate ke kanan\n\n");
                    }
                    else
                    {
                        rotateDirec(1, jarakTilt); // printf("  rotate ke kiri\n\n");
                    }
                }
                else
                {
                    btsRotate = gawang + 180;
                    if ((msg_yaw >= gawang) && (msg_yaw <= btsRotate))
                    {
                        rotateDirec(1, jarakTilt); // printf("  rotate ke kiri\n\n");
                    }
                    else
                    {
                        rotateDirec(-1, jarakTilt); // printf("  rotate ke kanan\n\n");
                    }
                }

                eleh = 0;
                robotDirection = false;
            }
            Walk(bodyXImu, bodyYImu, alfaImu);
        }
    }

    // Rotate Body with IMU ===========================================================
    bool posRotate = false;

    void rotateBodyImu(int rotate)
    {
        trackBall();

        setPoint1 = 5 + rotate;  // 20 bisa kemungkinan 180 keatas
        setPoint2 = -5 + rotate; // 20 bisa kemungkinan -180 bawah

        if (msg_yaw > setPoint2 && msg_yaw < setPoint1)
        {
            posRotate = true;
        }
        else
        {
            if (msg_yaw > rotate)
            { // putar Kiri
                Walk(0.0, 0.0, 0.2);
            }
            else
            { // putar Kanan
                Walk(0.0, 0.0, -0.2);
            }
            // posRotate = false;
        }
    }

    // Rotate Body with IMU ===========================================================
    bool posRotateNew = false;
    double IaMove, errorImuNew,
        putar;
    void rotateBodyImuNew(int rotate)
    {
        // trackBall();

        setPoint1 = 5 + rotate;  // 10 //15 //20 //60
        setPoint2 = -5 + rotate; // 10 //15 //20 //60

        if (setPoint1 > 180 || setPoint2 < -180)
        { // jika arah imu dibelakang
            if (setPoint1 > 180)
            { // nilai setpoint1 diubah jd negatif
                btsSetPoint = setPoint1 - 360;

                if (msg_yaw >= setPoint2 || msg_yaw <= btsSetPoint)
                { // misal 170 ke -170
                    PaMove = 0.00;
                    posRotateNew = true;
                }
                else
                {
                    btsRotate = rotate - 180;
                    if ((msg_yaw <= rotate) && (msg_yaw >= btsRotate))
                    {                                        // misal di range 0 - 180, maka putar kanan
                        putar = (rotate - msg_yaw) * 0.0065; // 0.0033
                        PaMove = -putar;
                    }
                    else
                    { // putar kiri
                        if (msg_yaw > rotate)
                        {
                            putar = (msg_yaw - rotate) * 0.0065;
                        } // 0.0033
                        else
                        {
                            putar = ((180 - rotate) + (180 + msg_yaw)) * 0.0065;
                        } // 0.0033
                        PaMove = putar;
                    }
                }
            }
            else
            { // nilai setPoint2 diubah jadi positif
                btsSetPoint = setPoint2 + 360;

                if (msg_yaw >= btsSetPoint || msg_yaw <= setPoint1)
                {
                    PaMove = 0.00;
                    posRotateNew = true;
                }
                else
                {
                    btsRotate = rotate + 180;
                    if ((msg_yaw >= rotate) && (msg_yaw <= btsRotate))
                    {                                           // misal di range -180 - 0, maka putar kiri
                        putar = abs(rotate - msg_yaw) * 0.0065; // 0.0033
                        PaMove = putar;
                    }
                    else
                    { // putar kanan
                        if (msg_yaw < rotate)
                        {
                            putar = (rotate - msg_yaw) * 0.0065;
                        } // 0.0033
                        else
                        {
                            putar = ((180 + rotate) + (180 - msg_yaw)) * 0.0065;
                        } // 0.0033
                        PaMove = -putar;
                    }
                }
            }
        }
        else
        { // arah imu kedepan
            if (msg_yaw >= setPoint2 && msg_yaw <= setPoint1)
            {
                PaMove = 0.00;
                posRotateNew = true;
            }
            else
            {
                if (rotate >= 0)
                {
                    btsRotate = rotate - 180;
                    if ((msg_yaw <= rotate) && (msg_yaw >= btsRotate))
                    {                                        // putar kanan
                        putar = (rotate - msg_yaw) * 0.0065; // 0.0033
                        PaMove = -putar;
                    }
                    else
                    { // putar kiri
                        if (msg_yaw > rotate)
                        {
                            putar = (msg_yaw - rotate) * 0.0065;
                        } // 0.0033
                        else
                        {
                            putar = ((180 - rotate) + (180 + msg_yaw)) * 0.0065;
                        } // 0.0033
                        PaMove = putar;
                    }
                }
                else
                {
                    btsRotate = rotate + 180;
                    if ((msg_yaw >= rotate) && (msg_yaw <= btsRotate))
                    {                                           // maka putar kiri
                        putar = abs(rotate - msg_yaw) * 0.0065; // 0.0033
                        PaMove = putar;
                    }
                    else
                    { // putar kanan
                        if (msg_yaw < rotate)
                        {
                            putar = (rotate - msg_yaw) * 0.0065;
                        } // 0.0033
                        else
                        {
                            putar = ((180 + rotate) + (180 - msg_yaw)) * 0.0065;
                        } // 0.0033
                        PaMove = -putar;
                    }
                }
            }
        }
        if (!posRotateNew)
        {
            Walk(0.0, 0.0, PaMove);
        }
    }

    //	A+ = putar kiri
    //	A- = putar kanan
    void jalanDirection(double Xwalk, double Ywalk, double rotate)
    {
        if (rotate > 180)
        {
            rotate = 180;
        }
        else if (rotate < -180)
        {
            rotate = -180;
        }
        //-175 sampai -185
        setPoint1 = 5 + rotate;  // 10 //15 //20 //60
        setPoint2 = -5 + rotate; // 10 //15 //20 //60

        if (setPoint1 > 180 || setPoint2 < -180)
        { // jika arah imu dibelakang
            if (setPoint1 > 180)
            { // nilai setpoint1 diubah jd negatif
                btsSetPoint = setPoint1 - 360;

                if (msg_yaw >= setPoint2 || msg_yaw <= btsSetPoint)
                { // misal 170 ke -170
                    PaMove = 0.00;
                }
                else
                {
                    btsRotate = rotate - 180;
                    if ((msg_yaw <= rotate) && (msg_yaw >= btsRotate))
                    {                                        // misal di range 0 - 180, maka putar kanan
                        putar = (rotate - msg_yaw) * 0.0065; // 0.0033
                        PaMove = -putar;
                    }
                    else
                    { // putar kiri
                        if (msg_yaw > rotate)
                        {
                            putar = (msg_yaw - rotate) * 0.004;
                        } // 0.0033
                        else
                        {
                            putar = ((180 - rotate) + (180 + msg_yaw)) * 0.004;
                        } // 0.0033
                        PaMove = putar;
                    }
                }
            }
            else
            { // nilai setPoint2 diubah jadi positif
                btsSetPoint = setPoint2 + 360;

                if (msg_yaw >= btsSetPoint || msg_yaw <= setPoint1)
                {
                    PaMove = 0.00;
                }
                else
                {
                    btsRotate = rotate + 180;
                    if ((msg_yaw >= rotate) && (msg_yaw <= btsRotate))
                    {                                          // misal di range -180 - 0, maka putar kiri
                        putar = abs(rotate - msg_yaw) * 0.004; // 0.0033
                        PaMove = putar;
                    }
                    else
                    { // putar kanan
                        if (msg_yaw < rotate)
                        {
                            putar = (rotate - msg_yaw) * 0.004;
                        } // 0.0033
                        else
                        {
                            putar = ((180 + rotate) + (180 - msg_yaw)) * 0.004;
                        } // 0.0033
                        PaMove = -putar;
                    }
                }
            }
        }
        else
        { // arah imu kedepan
            if (msg_yaw >= setPoint2 && msg_yaw <= setPoint1)
            {
                PaMove = 0.00;
            }
            else
            {
                if (rotate >= 0)
                {
                    btsRotate = rotate - 180;
                    if ((msg_yaw <= rotate) && (msg_yaw >= btsRotate))
                    {                                       // putar kanan
                        putar = (rotate - msg_yaw) * 0.004; // 0.0033
                        PaMove = -putar;
                    }
                    else
                    { // putar kiri
                        if (msg_yaw > rotate)
                        {
                            putar = (msg_yaw - rotate) * 0.004;
                        } // 0.0033
                        else
                        {
                            putar = ((180 - rotate) + (180 + msg_yaw)) * 0.004;
                        } // 0.0033
                        PaMove = putar;
                    }
                }
                else
                {
                    btsRotate = rotate + 180;
                    if ((msg_yaw >= rotate) && (msg_yaw <= btsRotate))
                    {                                          // maka putar kiri
                        putar = abs(rotate - msg_yaw) * 0.004; // 0.0033
                        PaMove = putar;
                    }
                    else
                    { // putar kanan
                        if (msg_yaw < rotate)
                        {
                            putar = (rotate - msg_yaw) * 0.004;
                        } // 0.0033
                        else
                        {
                            putar = ((180 + rotate) + (180 - msg_yaw)) * 0.004;
                        } // 0.0033
                        PaMove = -putar;
                    }
                }
            }
        }

        if (PaMove > 0.3)
        {
            PaMove = 0.3;
        }
        else if (PaMove < -0.3)
        {
            PaMove = -0.3;
        }

        Walk(Xwalk, Ywalk, PaMove);
    }

    // Ball Positioning Using P Controller =======================================================
    double	errorPosX,
        errorPosY,
        PxMoveBallPos,
        PyMoveBallPos,
        PaMoveBallPos;
    bool	ballPos = false;
    void ballPositioning(double setPointX, double setPointY, double speed) {
        errorPosX = headPan - setPointX;
        errorPosY = headTilt - setPointY;

        if ((errorPosX > -0.15 && errorPosX < 0.15) && (errorPosY > -0.10)) { //&& errorPosY < 0.10)) { //sudah sesuai
            PyMoveBallPos = 0.00;
            PxMoveBallPos = 0.00;
            ballPos = true;
        } else { //belum sesuai
            ballPos = false;
            if ((headPan >= 1.0 && headTilt >= -1.2) || (headPan <= -1.0 && headTilt >= -1.2)) { //bola disamping //pan tilt kircok (polar)
                PxMoveBallPos = -0.03;
                PyMoveBallPos = errorPosX * 0.08;//0.12;
            } else {
                if ((headPan >= 0.04 && headTilt > setPointY) || (headPan <= -0.04 && headTilt > setPointY)) { //bola disamping //pan tilt kircok (polar)
                    PxMoveBallPos = -0.03;
                } else {
                    //Xmove
                    if (headTilt > setPointY) { //> (setPointY + 0.1)) { //kelebihan
                        PxMoveBallPos = -0.03;
                    } else if (headTilt >= (setPointY - 0.1) && headTilt <= setPointY) { //<= (setPointY + 0.1)) { //sudah dalam range
                        PxMoveBallPos = 0.00;
                    } else if (headTilt >= (setPointY - 0.3) && headTilt < (setPointY - 0.1)) { //bola sudah dekat
                        PxMoveBallPos = errorPosY * -speed;
                        //PxMoveBallPos = errorPosY * -0.08;
    //					PxMoveBallPos = 0.01;
                        if (PxMoveBallPos >= 0.02) { PxMoveBallPos = 0.02; }
                        else if (PxMoveBallPos <= 0.00) { PxMoveBallPos = 0.00; }
                    } else { //bola masih jauh
                        PxMoveBallPos = headTilt * (0.08 / -1.6); //0.05
                        //PxMoveBallPos = kejar;
    //					PxMoveBallPos = 0.05;
                    }

                    //Ymove
                    if (headTilt >= (setPointY - 0.03)) { //> (setPointY + 0.1)) { //kelebihan
                        PyMoveBallPos = 0.00;
                    } else {
                        if (headPan >= (setPointX - 0.1) && headPan <= (setPointX + 0.1)) { //sudah dalam range
                            PyMoveBallPos = 0.00;
                        } else { //belum dalam range
                            PyMoveBallPos = errorPosX * 0.08;//0.08;//0.12;
                        }
                    }
                }
            }
            
        } Walk(PxMoveBallPos, PyMoveBallPos, 0.0);
    }
    
    // Dribble Ball ======================================================================
    int bawaBola;
    double setPointFootY, setPointFootY1, setPointFootY2;
    void dribble(int gawang, double speed)
    {
        // trackBall();

        // setPoint1 =  20 + gawang;//20
        // setPoint2 = -20 + gawang;//20

        // if (msg_yaw >= setPoint2 && msg_yaw <= setPoint1) {
        //	robotDirection = true;
        // } else  { robotDirection = false; }

        // if (robotDirection) { //printf("arah imu sudah benar\n");
        // if (headTilt <= -0.7 || bawaBola >= 300) {
        //	bawaBola = 0;
        //	robotDirection = false;
        //	stateCondition = 5;
        // } bawaBola++;

        // Ball Positioning ======================================
        // if (posPan >= 0) { setPointFootY = 0.25;  }//kiri
        // else             { setPointFootY = -0.25; }//kanan
        // errorPosX = headPan - setPointFootY;

        setPointFootY1 = 0.2;       // 0.22
        setPointFootY2 = -0.2;      //-0.22
        errorPosY = headTilt + 0.5; // 0.04;//0.05;//0.08;

        // if(errorPosX >= -0.1 && errorPosX <= 0.1) {
        if (headPan <= setPointFootY1 && headPan >= setPointFootY2)
        { // -0.2 > x < 0.2
            // if (headTilt >= -0.8) {
            PxMoveBallPos = 0.3 * speed*-1;
            //} else {
            //	PxMoveBallPos = errorPosY*speed*-1;
            //}
            // Walk(PxMoveBallPos, 0.0, 0.0);
            jalanDirection(PxMoveBallPos, 0.0, gawang);
        }
        else
        { // x < -0.2 || x > 0.2
            if (headTilt >= pTiltTendangKanan)
            {
                PxMoveBallPos = errorPosY * -speed;
                // PxMoveBallPos = -0.02;//-0.03;
                // Walk(-0.03, 0.0, 0.0);
            }
            else
            {
                PxMoveBallPos = 0.0;
            }

            if (headPan > setPointFootY1)
            { // printf("kiri bos\n");
                PyMoveBallPos = (headPan - 0.1) * 0.06;
            }
            else if (headPan < setPointFootY2)
            { // printf("kanan bos\n");
                PyMoveBallPos = (headPan + 0.1) * 0.06;
            }

            // Walk(PxMoveBallPos, PyMoveBallPos, 0.0);
            jalanDirection(PxMoveBallPos, PyMoveBallPos, gawang);

            // if (headTilt >= (cSekarang - 0.2)) {
            //	Walk(-0.03, 0.0, 0.0);
            // } else {
            //	if (headPan > setPointFootY1) { //printf("kiri bos\n");
            //		PyMoveBallPos = (headPan - 0.1) * 0.06;
            //		Walk(0.0, PyMoveBallPos, 0.0);
            //	} else if (headPan < setPointFootY2) { //printf("kanan bos\n");
            //		PyMoveBallPos = (headPan + 0.1) * 0.06;
            //		Walk(0.0, PyMoveBallPos, 0.0);
            //	}
            // }

            // if(errorPosY >= -0.1) { // XmoveBackWard
            //	Walk(-0.03, 0.0, 0.0);
            // } else { // Ymove
            //	PyMoveBallPos = errorPosX*0.06;
            //	Walk(0.0, PyMoveBallPos, 0.0);
            // }
        }
        //} else { //printf("cari arah imu\n");
        //	bawaBola = 0;
        //	if (posTilt > -0.8 && posPan > -0.5 && posPan < 0.5) {//bola dekat
        //		Imu(gawang, cSekarang);
        //	} else {//bola masih jauh
        //		followBall(0);
        //	}
        //}
    }

    // Checking Lost Goal ========================================================================
    int countGoalLost = 0,
        countGoalFound = 0,
        returnGoalVal;
    int goalLost(int threshold)
    {
        if (useVision)
        {
            if (Goal_X == -1 && Goal_Y == -1)
            {
                countGoalFound = 0;
                countGoalLost++;
                if (countGoalLost >= threshold)
                {
                    returnGoalVal = 1;
                }
            }
            else
            {
                if (headTilt < -1.0)
                {
                    countGoalLost = 0;
                    countGoalFound++;
                    if (countGoalFound > 1)
                    {
                        returnGoalVal = 0;
                    }
                }
            }
        }
        else
        {
            countGoalFound = 0;
            countGoalLost++;
            if (countGoalLost >= threshold)
            {
                returnGoalVal = 1;
            }
        }
        return returnGoalVal;
    }

    // Goal Tracking =============================================================================
    double intPanG = 0, dervPanG = 0, errorPanG = 0, preErrPanG = 0,
           PPanG = 0, IPanG = 0, DPanG = 0,
           intTiltG = 0, dervTiltG = 0, errorTiltG = 0, preErrTiltG = 0,
           PTiltG = 0, ITiltG = 0, DTiltG = 0,
           dtG = 0.04;
    double G_Pan_err_diff, G_Pan_err, G_Tilt_err_diff, G_Tilt_err, G_PanAngle, G_TiltAngle,
        pOffsetG, iOffsetG, dOffsetG,
        errorPanGRad, errorTiltGRad;
    int offsetSetPointGoal;
    void trackGoal()
    {
        if (useVision)
        {
            if (Goal_X != -1 && Goal_Y != -1)
            { // printf("Tracking");
                // mode 1 ######################################################################
                // PID pan ==========================================================
                /*errorPanG  = (double)Goal_X - (frame_X / 2);//160
                PPanG  = errorPanG  * 0.00010; //Tune in Kp Pan  0.00035 //kalau kepala msh goyang2, kurangin nilainya

                intPanG += errorPanG * dtG;
                IPanG = intPanG * 0.0;

                dervPanG = (errorPanG - preErrPanG) / dtG;
                DPanG = dervPanG * 0.00001;

                preErrPanG = errorPanG;

                //posPan += PPanG*-1; //dikali -1 kalau receler terbalik dalam pemasangan
                posPan += (PPanG + IPanG + DPanG) * -1;


                //PID tilt ==========================================================
                errorTiltG = (double)Goal_Y - (frame_Y / 2);//120
                PTiltG = errorTiltG * 0.00010; //Tune in Kp Tilt 0.00030

                intTiltG += errorTiltG * dtG;
                ITiltG = intTiltG * 0.0;

                dervTiltG = (errorTiltG - preErrTiltG) / dtG;
                DTiltG = dervTiltG * 0; //0.00001;

                preErrTiltG = errorTiltG;

                //posTilt += PTiltG;
                posTilt += (PTiltG + ITiltG + DTiltG);*/

                // mode 2 ######################################################################
                //  offsetSetPointGoal = (int)((posTilt * 30) + 54);
                //  if (offsetSetPointGoal > 36) offsetSetPointGoal = 36;
                //  else if (offsetSetPointGoal < 0) offsetSetPointGoal = 0;

                // errorPanG  = (double)Goal_X - ((frame_X / 2) + offsetSetPointGoal);//160
                errorPanG = (double)Goal_X - (frame_X / 2);
                errorTiltG = (double)Goal_Y - (frame_Y / 2); // 120

                errorPanG *= -1;
                errorTiltG *= -1;
                errorPanG *= (90 / (double)frame_X);     // pixel per msg_yaw
                errorTiltG *= (60 / (double)frame_Y);    // pixel per msg_yaw
                errorPanG *= (77.32 / (double)frame_X);  // pixel per msg_yaw
                errorTiltG *= (61.93 / (double)frame_Y); // pixel per msg_yaw

                errorPanGRad = (errorPanG * PI) / 180;
                errorTiltGRad = (errorTiltG * PI) / 180;
                // printf("errorPan = %.2f \t errorTilt = %.2f\n", errorPanG, errorTiltG);
                // printf("RadrrorPan = %.2f \t RaderrorTilt = %.2f\n", errorPanGRad, errorTiltGRad);
                // printf("KPPan = %f \t KDPan = %f\t", kamera.panKP, kamera.panKD); printf("KPTilt = %f \t KDTilt = %f\n", kamera.tiltKP, kamera.tiltKD);

                G_Pan_err_diff = errorPanGRad - G_Pan_err;
                G_Tilt_err_diff = errorTiltGRad - G_Tilt_err;

                // PID pan ==========================================================
                // PPanG  = G_Pan_err  * kamera.panKP; // Tune in Kp Pan 0.00035 //kalau kepala msh goyang2, kurangin nilainya
                PPanG = G_Pan_err * goal_panKP; // Tune in Kp Pan 0.00035 //kalau kepala msh goyang2, kurangin nilainya
                intPanG += G_Pan_err * dtG;
                IPanG = intPanG * 0.0;
                dervPanG = G_Pan_err_diff / dtG;
                // DPanG = dervPanG * kamera.panKD;
                DPanG = dervPanG * goal_panKD;
                G_Pan_err = errorPanGRad;
                posPan += (PPanG + IPanG + DPanG);

                // PID tilt ==========================================================
                // PTiltG = G_Tilt_err * kamera.tiltKP; // Tune in Kp Tilt 0.00030
                PTiltG = G_Tilt_err * goal_tiltKP; // Tune in Kp Tilt 0.00030

                intTiltG += G_Tilt_err * dtG;
                ITiltG = intTiltG * 0.0;

                dervTiltG = G_Tilt_err_diff / dtG;
                // DTiltG = dervTiltG * kamera.tiltKD;
                DTiltG = dervTiltG * goal_tiltKD;

                preErrTiltG = errorTiltG;
                G_Tilt_err = errorTiltGRad;
                posTilt += (PTiltG + ITiltG + DTiltG) * -1;

                if (posPan >= 1.6)
                {
                    posPan = 1.6;
                }
                else if (posPan <= -1.6)
                {
                    posPan = -1.6;
                }
                if (posTilt <= -2.0)
                {
                    posTilt = -2.0;
                }
                else if (posTilt >= -0.4)
                {
                    posTilt = -0.4;
                }

                headMove(posPan, posTilt); // printf("posPan = %.2f \t posTilt = %.2f\n", posPan, posTilt);
            }
        }
    }

    // Follow Goal ===============================================================================
    int countReadyStop = 0;
    void followGoal(double Xwalk, double SetPan, int mode)
    { // 0 normal, 1 sambil belok
        errorfPan = posPan - SetPan;

        if (posTilt < -2.0 && posPan < 0.4 && posPan > -0.4 && Goal_X != -1 && Goal_Y != -1)
        { // Stop
            countReadyStop++;
        }
        else
        { // Follow
            countReadyStop = 0;
        }

        if (countReadyStop >= 5)
        {
            PxMove = 0.00;              // jalan ditempat
            PyMove = errorfPan * 0.040; // 0.045
            PaMove = errorfPan * 0.20;  // 0.30; //0.045
        }
        else
        {
            PxMove = Xwalk;             // 0.08
            PyMove = errorfPan * 0.040; // 0.045
            PaMove = errorfPan * 0.20;  // 0.35; //0.045
        }

        if (mode == 0)
        { // pake alfa
            if (errorfPan > -0.4 && errorfPan < 0.4)
            { // printf("AAAAAAAA\n");
                Walk(PxMove, 0.0, PaMove);
            }
            else
            { // printf("BBBBBBBB\n");
                Walk(0.0, 0.0, PaMove);
            }
        }
        else if (mode == 1)
        { // tanpa alfa
            if (errorfPan > -0.4 && errorfPan < 0.4)
            { // printf("CCCCCCCC\n");
                Walk(PxMove, PyMove, 0.0);
            }
            else
            { // printf("DDDDDDDD\n");
                Walk(0.0, PyMove, 0.0);
            }
        }
    }

    // Body Tracking Goal ========================================================================
    double errorBodyPositionG,
        bodyP_ControllerG;
    int bodyTrueG = 0,
        delayTrueG = 0;
    int bodyTrackingGoal(int threshold)
    {
        //	trackGoal();

        errorBodyPositionG = 0 - headPan;
        bodyP_ControllerG = errorBodyPositionG * -0.5; //-0.5

        if (errorBodyPositionG >= -0.1 && errorBodyPositionG <= 0.1)
        { // untuk hasil hadap 0.8
            // motion("0");
            Walk(0.0, 0.0, 0.0);
            delayTrueG++;
        }
        else
        {
            trackGoal();

            bodyTrueG =
                delayTrueG = 0;

            if (bodyP_ControllerG < 0)
            { // kanan
                // Walk(rotateGoal_x, abs(rotateGoal_y), -abs(rotateGoal_a));
                Walk(rotateGoal_x, rotateGoal_y, -rotateGoal_a);
            }
            else
            { // kiri
                // Walk(rotateGoal_x, -abs(rotateGoal_y), abs(rotateGoal_a));
                Walk(rotateGoal_x, -rotateGoal_y, rotateGoal_a);
            }
        }

        if (delayTrueG >= threshold)
        {
            bodyTrueG = 1;
        }
        else
        {
            bodyTrueG = 0;
        }
        return bodyTrueG;
    }

    // Search Goal =====================================================================================================
    double goalPan = 0;
    void saveGoalLocation()
    {
        //	trackGoal();
        goalPan = headPan;
    }

    // Landmark Tracking =============================================================================
    double intPanL = 0, dervPanL = 0, errorPanL = 0, preErrPanL = 0,
           PPanL = 0, IPanL = 0, DPanL = 0,
           intTiltL = 0, dervTiltL = 0, errorTiltL = 0, preErrTiltL = 0,
           PTiltL = 0, ITiltL = 0, DTiltL = 0,
           dtL = 0.04;
    double L_Pan_err_diff, L_Pan_err, L_Tilt_err_diff, L_Tilt_err,
        errorPanLRad, errorTiltLRad;
    int offsetSetPointLand;
    void trackLand()
    {
        if (useVision)
        {
            if (Xcross_LX != -1 && Xcross_LY != -1)
            { // printf("Tracking");
                // mode 1 ######################################################################
                // PID pan ==========================================================
                /*errorPanL  = (double)Goal_X - (frame_X / 2);//160
                PPanL  = errorPanL  * 0.00010; //Tune in Kp Pan  0.00035 //kalau kepala msh goyang2, kurangin nilainya

                intPanL += errorPanL * dtL;
                IPanL = intPanL * 0.0;

                dervPanL = (errorPanL - preErrPanL) / dtL;
                DPanL = dervPanL * 0.00001;

                preErrPanL = errorPanL;

                //posPan += PPanL*-1; //dikali -1 kalau receler terbalik dalam pemasangan
                posPan += (PPanL + IPanL + DPanL) * -1;


                //PID tilt ==========================================================
                errorTiltL = (double)Goal_Y - (frame_Y / 2);//120
                PTiltL = errorTiltL * 0.00010; //Tune in Kp Tilt 0.00030

                intTiltL += errorTiltL * dtL;
                ITiltL = intTiltL * 0.0;

                dervTiltL = (errorTiltL - preErrTiltL) / dtL;
                DTiltL = dervTiltL * 0; //0.00001;

                preErrTiltL = errorTiltL;

                //posTilt += PTiltL;
                posTilt += (PTiltL + ITiltL + DTiltL);*/

                // mode 2 ######################################################################
                offsetSetPointLand = (int)((posTilt * 30) + 54);
                if (offsetSetPointLand > 36)
                    offsetSetPointLand = 36;
                else if (offsetSetPointLand < 0)
                    offsetSetPointLand = 0;

                if ((Xcross_LX != -1 && Xcross_LY != -1) && (Xcross_RX != -1 && Xcross_RY != -1))
                {
                    errorPanL = (double)Xcross_RX - ((frame_X / 2) + offsetSetPointLand); // 160
                    errorTiltL = (double)Xcross_RY - (frame_Y / 2);                       // 120
                }
                else
                {
                    errorPanL = (double)Xcross_LX - ((frame_X / 2) + offsetSetPointLand); // 160
                    errorTiltL = (double)Xcross_LY - (frame_Y / 2);                       // 120
                }

                errorPanL *= -1;
                errorTiltL *= -1;
                errorPanL *= (90 / (double)frame_X);     // pixel per msg_yaw
                errorTiltL *= (60 / (double)frame_Y);    // pixel per msg_yaw
                errorPanL *= (77.32 / (double)frame_X);  // pixel per msg_yaw
                errorTiltL *= (61.93 / (double)frame_Y); // pixel per msg_yaw

                errorPanLRad = (errorPanL * PI) / 180;
                errorTiltLRad = (errorTiltL * PI) / 180;
                // printf("errorPan = %.2f \t errorTilt = %.2f\n", errorPanL, errorTiltL);
                // printf("RadrrorPan = %.2f \t RaderrorTilt = %.2f\n", errorPanLRad, errorTiltLRad);
                // printf("KPPan = %f \t KDPan = %f\t", kamera.panKP, kamera.panKD); printf("KPTilt = %f \t KDTilt = %f\n", kamera.tiltKP, kamera.tiltKD);

                L_Pan_err_diff = errorPanLRad - L_Pan_err;
                L_Tilt_err_diff = errorTiltLRad - L_Tilt_err;

                // PID pan ==========================================================
                // PPanL  = L_Pan_err  * kamera.panKP; // Tune in Kp Pan 0.00035 //kalau kepala msh goyang2, kurangin nilainya
                PPanL = L_Pan_err * goal_panKP; // Tune in Kp Pan 0.00035 //kalau kepala msh goyang2, kurangin nilainya
                intPanL += L_Pan_err * dtL;
                IPanL = intPanL * 0.0;
                dervPanL = L_Pan_err_diff / dtL;
                // DPanL = dervPanL * kamera.panKD;
                DPanL = dervPanL * goal_panKD;
                L_Pan_err = errorPanLRad;
                posPan += (PPanL + IPanL + DPanL);

                // PID tilt ==========================================================
                // PTiltL = L_Tilt_err * kamera.tiltKP; // Tune in Kp Tilt 0.00030
                PTiltL = L_Tilt_err * goal_tiltKP; // Tune in Kp Tilt 0.00030

                intTiltL += L_Tilt_err * dtL;
                ITiltL = intTiltL * 0.0;

                dervTiltL = L_Tilt_err_diff / dtL;
                // DTiltL = dervTiltL * kamera.tiltKD;
                DTiltL = dervTiltL * goal_tiltKD;

                preErrTiltL = errorTiltL;
                L_Tilt_err = errorTiltLRad;
                posTilt += (PTiltL + ITiltL + DTiltL) * -1;

                if (posPan >= 1.6)
                {
                    posPan = 1.6;
                }
                else if (posPan <= -1.6)
                {
                    posPan = -1.6;
                }
                if (posTilt <= -2.0)
                {
                    posTilt = -2.0;
                }
                else if (posTilt >= -0.4)
                {
                    posTilt = -0.4;
                }

                headMove(posPan, posTilt); // printf("posPan = %.2f \t posTilt = %.2f\n", posPan, posTilt);
            }
        }
    }

    // Follow Landmark ===============================================================================
    int countReadyStopL = 0;
    void followLand(double Xwalk, double SetPan, int mode)
    { // 0 normal, 1 sambil belok
        errorfPan = posPan - SetPan;

        if (posTilt < -2.0 && posPan < 0.4 && posPan > -0.4 && Xcross_LX != -1 && Xcross_LY != -1)
        { // Stop
            countReadyStopL++;
        }
        else
        { // Follow
            countReadyStopL = 0;
        }

        if (countReadyStopL >= 5)
        {
            PxMove = 0.00;              // jalan ditempat
            PyMove = errorfPan * 0.040; // 0.045
            PaMove = errorfPan * 0.20;  // 0.30; //0.045
        }
        else
        {
            PxMove = Xwalk;             // 0.08
            PyMove = errorfPan * 0.040; // 0.045
            PaMove = errorfPan * 0.20;  // 0.35; //0.045
        }

        if (mode == 0)
        { // pake alfa
            if (errorfPan > -0.4 && errorfPan < 0.4)
            { // printf("AAAAAAAA\n");
                Walk(PxMove, 0.0, PaMove);
            }
            else
            { // printf("BBBBBBBB\n");
                Walk(0.0, 0.0, PaMove);
            }
        }
        else if (mode == 1)
        { // tanpa alfa
            if (errorfPan > -0.4 && errorfPan < 0.4)
            { // printf("CCCCCCCC\n");
                Walk(PxMove, PyMove, 0.0);
            }
            else
            { // printf("DDDDDDDD\n");
                Walk(0.0, PyMove, 0.0);
            }
        }
    }

    void saveSudutImu()
    {
        //	sudut();
        saveAngle = msg_yaw;
    }

    int prediksiGoalPan = 0;
    double Rotate = 0;
    void prediksiArahGoal()
    {
        prediksiGoalPan = (int)(57.29 * headPan);
        Rotate = headPan * (8 / 1.57); // 90 derajat = 8 detik //waktu rotate
    }

    int count = 0;
    bool goalSearch = false,
         searchGoalFinish = false;
    void panSearchGoal(double arah)
    { // printf("  panSearchBall\n\n");
        if (panRate < 0)
        {
            panRate = -panRate;
        }

        if (arah < 0)
        {
            headPan += panRate;
            if (headPan >= batasKiri)
            {
                prediksiGoalPan = 0;
                saveAngle = 0;
                Rotate = 0;
                goalSearch = true;
            }
        }
        else
        {
            headPan -= panRate;
            if (headPan <= batasKanan)
            {
                prediksiGoalPan = 0;
                saveAngle = 0;
                Rotate = 0;
                goalSearch = true;
            }
        }

        if (headPan >= batasKiri)
        {
            headPan = batasKiri;
        }
        else if (headPan <= batasKanan)
        {
            headPan = batasKanan;
        }

        headMove(headPan, -2.0); // printf("posPan = %.2f \t posTilt = %.2f\n", posPan, posTilt);
    }

    // +predictPanGoal = kiri
    // -predictPanGoal = kanan

    //	A+ = putar kiri
    //	A- = putar kanan
    double predictGoalPan;
    void predictGoal(double alpha, double tilt)
    {
        predictGoalPan = alpha / 57.29; // sudut * nilai per satu sudut(nilai servo)

        if (predictGoalPan <= -1.6)
        {
            predictGoalPan = -1.6;
        }
        else if (predictGoalPan >= 1.6)
        {
            predictGoalPan = 1.6;
        }
        // printf("  predict = %f\n\n",predictGoalPan);

        headMove(predictGoalPan, tilt);
    }

    void predictGoalTeam(double alpha, double tilt)
    {
        int setPointPan1, setPointPan2,
            btsSetPointPan, btsRotatePan;
        double putarPan;

        if (alpha > 180)
        {
            alpha = 180;
        }
        else if (alpha < -180)
        {
            alpha = -180;
        }

        setPointPan1 = 5 + alpha;
        setPointPan2 = -5 + alpha;

        if (setPointPan1 > 180 || setPointPan2 < -180)
        { // jika arah imu dibelakang
            if (setPointPan1 > 180)
            { // nilai setPointPan1 diubah jd negatif
                btsSetPointPan = setPointPan1 - 360;

                if (msg_yaw >= setPointPan2 || msg_yaw <= btsSetPointPan)
                { // misal 170 ke -170
                    predictGoalPan = 0.00;
                }
                else
                {
                    btsRotatePan = alpha - 180;
                    if ((msg_yaw <= alpha) && (msg_yaw >= btsRotatePan))
                    { // misal di range 0 - 180, maka putarPan kanan
                        putarPan = (alpha - msg_yaw) / 57.29;
                        predictGoalPan = -putarPan;
                    }
                    else
                    { // putarPan kiri
                        if (msg_yaw > alpha)
                        {
                            putarPan = (msg_yaw - alpha) / 57.29;
                        } // 0.0033
                        else
                        {
                            putarPan = ((180 - alpha) + (180 + msg_yaw)) / 57.29;
                        } // 0.0033
                        predictGoalPan = putarPan;
                    }
                }
            }
            else
            { // nilai setPointPan2 diubah jadi positif
                btsSetPointPan = setPointPan2 + 360;

                if (msg_yaw >= btsSetPointPan || msg_yaw <= setPointPan1)
                {
                    predictGoalPan = 0.00;
                }
                else
                {
                    btsRotatePan = alpha + 180;
                    if ((msg_yaw >= alpha) && (msg_yaw <= btsRotatePan))
                    {                                            // misal di range -180 - 0, maka putarPan kiri
                        putarPan = abs(alpha - msg_yaw) / 57.29; // 0.0033
                        predictGoalPan = putarPan;
                    }
                    else
                    { // putarPan kanan
                        if (msg_yaw < alpha)
                        {
                            putarPan = (alpha - msg_yaw) / 57.29;
                        } // 0.0033
                        else
                        {
                            putarPan = ((180 + alpha) + (180 - msg_yaw)) / 57.29;
                        } // 0.0033
                        predictGoalPan = -putarPan;
                    }
                }
            }
        }
        else
        { // arah imu kedepan
            if (msg_yaw >= setPointPan2 && msg_yaw <= setPointPan1)
            {
                predictGoalPan = 0.00;
            }
            else
            {
                if (alpha >= 0)
                {
                    btsRotatePan = alpha - 180;
                    if ((msg_yaw <= alpha) && (msg_yaw >= btsRotatePan))
                    {                                         // putarPan kanan
                        putarPan = (alpha - msg_yaw) / 57.29; // 0.0033
                        predictGoalPan = -putarPan;
                    }
                    else
                    { // putarPan kiri
                        if (msg_yaw > alpha)
                        {
                            putarPan = (msg_yaw - alpha) / 57.29;
                        } // 0.0033
                        else
                        {
                            putarPan = ((180 - alpha) + (180 + msg_yaw)) / 57.29;
                        } // 0.0033
                        predictGoalPan = putarPan;
                    }
                }
                else
                {
                    btsRotatePan = alpha + 180;
                    if ((msg_yaw >= alpha) && (msg_yaw <= btsRotatePan))
                    { // maka putarPan kiri
                        putarPan = abs(alpha - msg_yaw) / 57.29;
                        ; // 0.0033
                        predictGoalPan = putarPan;
                    }
                    else
                    { // putarPan kanan
                        if (msg_yaw < alpha)
                        {
                            putarPan = (alpha - msg_yaw) / 57.29;
                        } // 0.0033
                        else
                        {
                            putarPan = ((180 + alpha) + (180 - msg_yaw)) / 57.29;
                        } // 0.0033
                        predictGoalPan = -putarPan;
                    }
                }
            }
        }

        if (predictGoalPan <= -1.6)
        {
            predictGoalPan = -1.6;
        }
        else if (predictGoalPan >= 1.6)
        {
            predictGoalPan = 1.6;
        }

        headMove(predictGoalPan, tilt);
    }

    int goalSide = 0;
    double arahPandang = 0;
    int cekArah()
    {
        arahPandang = msg_yaw - (57.29 * headPan);
        if (arahPandang >= -90 && arahPandang <= 90)
        { // gawang lawan
            goalSide = 0;
        }
        else
        { // gawang sendiri
            goalSide = 1;
        }
        return goalSide;
    }

    // Tendang ===================================================================================
    bool tendang = false;
    void kick(int mode)
    {
        //	trackBall();
        if (mode == 3 || mode == 4 || mode == 33 || mode == 44)
        {
            if (mode == 3 || mode == 33)
            {
                kanan = true;
                kiri = false;
            } // arah kanan
            else if (mode == 4 || mode == 44)
            {
                kiri = true;
                kanan = false;
            } // arah kiri
        }
        else
        {
            if (forceKanan && forceKiri)
            {
                if (posPan >= 0 && kanan == false && kiri == false)
                { // kiri
                    kiri = true;
                    kanan = false;
                }
                else if (posPan <= 0 && kanan == false && kiri == false)
                // else
                { // kanan
                    kanan = true;
                    kiri = false;
                }
            } else
            {
                if (forceKanan)
                {
                    kanan = true;
                    kiri = false;
                } else if (forceKiri)
                {
                    kiri = true;
                    kanan = false;
                }
            }
        }

        if (kiri)
        { // kiri
            // if (posPan >= 0) { //kiri
            if (ballPos)
            { // printf("ball pos left true\n");
                // motion("0");
                if (mode == 1 || mode == 2)
                {
                    motion("0");
                    sleep(1);
                    motion("1");
                    tendang = true;
                }
                else if (mode == 3 || mode == 4)
                {
                    motion("0");
                    sleep(1); // 10
                    motion("4");
                    tendang = true;
                }
                else if (mode == 33 || mode == 44)
                {
                    motion("0");
                    sleep(1); // 10
                    motion("@");
                    tendang = true;
                } 
                else if (mode == 5 || mode == 6)
                {
                    motion("0");
                    sleep(1);
                    motion("5");
                    tendang = true;
                } 
            }
            else
            {
                ballPositioning(pPanTendangKiri, pTiltTendangKiri, ballPositioningSpeed); // 0.15
            }
        }
        if (kanan)
        { // kanan
            //} else { //kanan
            if (ballPos)
            { // printf("ball pos right true\n");
                // motion("0");
                if (mode == 1 || mode == 2)
                {
                    motion("0");
                    sleep(1); // 7
                    motion("2");
                    tendang = true;
                }
                else if (mode == 3 || mode == 4)
                {
                    motion("0");
                    sleep(1); // 10
                    motion("3");
                    tendang = true;
                }
                else if (mode == 33 || mode == 44)
                {
                    motion("0");
                    sleep(1); // 10
                    motion("!");
                    tendang = true;
                }
                else if (mode == 5 || mode == 6)
                {
                    motion("0");
                    sleep(1); // 7
                    motion("6");
                    tendang = true;
                }
            }
            else
            {
                ballPositioning(pPanTendangKanan, pTiltTendangKanan, ballPositioningSpeed); // 0.15
            }
        }
    }

    void kickNoSudut(int mode) {
    //trackBall();
        if (headTilt >= -1.5) {
		if (robotDirection && headPan >= -0.4 && headPan <= 0.4) {
				if (ballPos) { //printf("ball pos left true\n");
					motion("0");
					if (mode == 1 || mode == 2) {
						usleep(900000); //6
						motion("2");
					} else if (mode == 3 || mode == 4) {
						usleep(300000); //10
						sleep(1); //10
						motion("4");
						//motion("4");
					} else if (mode == 5 || mode == 6) {
						usleep(850000);
						motion("6");
					}
					tendang = true;
				} else {
					ballPositioning(pPanTendangKanan, pTiltTendangKanan, ballPositioningSpeed); //0.15
				}			
		} else {
			if (headTilt >= (cAktif  + 0.2) && headPan >= -0.4 && headPan <= 0.4) {
				//Imu(sudut, cSekarang - 0.20);
				robotDirection = true;
			} else {
				robotDirection = false;
				followBall(0);
			}
			
		}
	} else {
		if	(posTilt >= SetPointTilt) { posTilt = SetPointTilt; }
		else if	(posTilt < -2.0) { posTilt = -2.0; }

		errorfPan  = posPan - SetPointPan;
		errorfTilt = posTilt - SetPointTilt;

		if (posTilt >= SetPointTilt && posPan < 0.4 && posPan > -0.4 && Ball_X != -1 && Ball_Y != -1) { //Stop(bola sudah dekat)
			PxMove = 0.0; //jalan ditempat
			PyMove = errorfPan * 0.040; //0.045
			PaMove = errorfPan * 0.20; //0.30; //0.045
		} else { //Kejar Bola(bola masih jauh)
			PxMove = kejarMax; //0.06
			PyMove = errorfPan * 0.045; //0.045
			PaMove = errorfPan * 0.25; //0.35; //0.045
		}

		if (errorfPan > -0.4 && errorfPan < 0.4) { 	//printf("AAAAAAAA\n");
			Walk(PxMove, 0.0, PaMove);
		} else { 					//printf("BBBBBBBB\n");
			Walk(0.0, 0.0, PaMove);
		}
		//followBall(0);
	}
}

    void rotateKickOffImu(int sudut, int mode)
    {
        if (headTilt >= cSekarang)
        {
            if (robotDirection && headPan >= -0.4 && headPan <= 0.4)
            {
                if (sudut >= 0)
                { // kiri
                    if (ballPos)
                    { // printf("ball pos left true\n");
                        motion("0");
                        if (mode == 1 || mode == 2)
                        {
                            sleep(1); // 6
                            motion("1");
                        }
                        else if (mode == 3 || mode == 4)
                        {
                            sleep(1); // 10
                            motion("0");
                        }
                        else if (mode == 5 || mode == 6)
                        {
                            sleep(1);
                            motion("5");
                        }
                        tendang = true;
                    }
                    else
                    {
                        ballPositioning(pPanTendangKiri, pTiltTendangKiri, ballPositioningSpeed); // 0.15
                    }
                }
                else
                { // kanan
                    if (ballPos)
                    { // printf("ball pos right true\n");
                        motion("0");
                        if (mode == 1 || mode == 2)
                        {
                            sleep(1); // 7
                            motion("2");
                        }
                        else if (mode == 3 || mode == 4)
                        {
                            sleep(1); // 10
                            motion("0");
                        }
                        else if (mode == 5 || mode == 6)
                        {
                            sleep(1); // 7
                            motion("6");
                        }
                        tendang = true;
                    }
                    else
                    {
                        ballPositioning(pPanTendangKanan, pTiltTendangKanan, ballPositioningSpeed); // 0.15
                    }
                }
            }
            else
            {
                if (headTilt >= cAktif && headPan >= -0.4 && headPan <= 0.4)
                {                          // +
                    Imu(sudut, cSekarang); //- 0.20
                }
                else
                {
                    robotDirection = false;
                    followBall(0);
                }
            }
        }
        else
        {
            if (posTilt >= SetPointTilt)
            {
                posTilt = SetPointTilt;
            }
            else if (posTilt < -2.0)
            {
                posTilt = -2.0;
            }

            errorfPan = posPan - SetPointPan;
            errorfTilt = posTilt - SetPointTilt;

            if (posTilt >= SetPointTilt && posPan < 0.4 && posPan > -0.4 && Ball_X != -1 && Ball_Y != -1)
            {                               // Stop(bola sudah dekat)
                PxMove = 0.0;               // jalan ditempat
                PyMove = errorfPan * 0.040; // 0.045
                PaMove = errorfPan * 0.20;  // 0.30; //0.045
            }
            else
            {                              // Kejar Bola(bola masih jauh)
                PxMove = kejarMax;         // 0.06
                PyMove = errorfPan * 0.45; // 0.045
                PaMove = errorfPan * 0.25; // 0.35; //0.045
            }

            if (errorfPan > -0.4 && errorfPan < 0.4)
            { // printf("AAAAAAAA\n");
                Walk(PxMove, 0.0, PaMove);
            }
            else
            { // printf("BBBBBBBB\n");
                Walk(0.0, 0.0, PaMove);
            }
            // followBall(0);
        }
    }

    void rotateKickOff(double timeRotate, int mode)
    {
        trackBall();

        // if (headTilt >= -1.0) {
        if (headTilt >= -0.8 && headPan >= -0.4 && headPan <= 0.4)
        {
            if (reset > 5)
            { // printf("set......................!!!\n");
                cekWaktu(abs(timeRotate));
                if (timer)
                { // printf("true......................!!!\n");
                    if (posPan >= 0)
                    { // kiri
                        if (ballPos)
                        { // printf("ball pos left true\n");
                            motion("0");
                            if (mode == 1 || mode == 2)
                            {
                                sleep(2); // 6
                                motion("1");
                            }
                            else if (mode == 3 || mode == 4)
                            {
                                // usleep(1000000); //10
                                sleep(2); // 10
                                motion("3");
                                // motion("4");
                            }
                            else if (mode == 5 || mode == 6)
                            {
                                sleep(2);
                                motion("5");
                            }
                            tendang = true;
                        }
                        else
                        {
                            ballPositioning(pPanTendangKiri, pTiltTendangKiri, ballPositioningSpeed); // 0.15
                        }
                    }
                    else
                    { // kanan
                        if (ballPos)
                        { // printf("ball pos right true\n");
                            motion("0");
                            if (mode == 1 || mode == 2)
                            {
                                sleep(2); // 7
                                motion("2");
                            }
                            else if (mode == 3 || mode == 4)
                            {
                                // usleep(1000000); //10
                                sleep(2); // 10
                                // motion("4");
                                motion("4");
                            }
                            else if (mode == 5 || mode == 6)
                            {
                                sleep(2); // 7
                                motion("6");
                            }
                            tendang = true;
                        }
                        else
                        {
                            ballPositioning(pPanTendangKanan, pTiltTendangKanan, ballPositioningSpeed); // 0.15
                        }
                    }
                }
                else
                {
                    // rotateParabolic(timeRotate, cSekarang);
                    if (timeRotate < 0)
                    { // kanan
                        Walk(rotateGoal_x, rotateGoal_y, -rotateGoal_a);
                    }
                    else
                    { // kiri
                        Walk(rotateGoal_x, -rotateGoal_y, rotateGoal_a);
                    }
                }
            }
            else
            { // printf("cek......................!!!\n");
                Walk(0.0, 0.0, 0.0);
                setWaktu();
                reset++;
            }
        }
        else
        {
            reset = 0;
            followBall(0);
        }
    }

    /* int convertGridX(int valueGrid, int valueOffSetX)
    { // Konvert grid posisi robot jadi nilai koordinat x
        int tempCoorX, tempGridtoX;

        if (valueGrid % 6 == 0)
        {
            // printf("POPO\n");
            tempGridtoX = valueGrid / 6;
        }
        else
        {
            // printf("PIPI\n");
            tempGridtoX = (valueGrid / 6) + 1;
        }

        if (tempGridtoX <= 1)
        {
            tempGridtoX = 1;
        }
        else if (tempGridtoX >= 9)
        {
            tempGridtoX = 9;
        }

        tempCoorX = (((tempGridtoX * 100) - 500) + valueOffSetX);

        return tempCoorX;
    }

    int convertGridY(int valueGrid, int valueOffSetY)
    { // Konvert grid posisi robot jadi nilai koordinat y
        int tempCoorY, tempGridtoY;
        if (valueGrid % 6 == 0)
        {
            // printf("POPO\n");
            tempGridtoY = 6;
        }
        else
        {
            // printf("PIPI\n");
            tempGridtoY = (valueGrid - ((valueGrid / 6) * 6));
        }

        if (tempGridtoY <= 1)
        {
            tempGridtoY = 1;
        }
        else if (tempGridtoY >= 6)
        {
            tempGridtoY = 6;
        }

        tempCoorY = (((tempGridtoY * 100) - 350) + valueOffSetY);

        return tempCoorY;
    } */

    int tempCoorX, tempGridtoX;
    int tempCoorY, tempGridtoY;

    int convertGridX(int valueGrid, int valueOffSetX) {
        //convert Grid To Koordinat X
        tempGridtoX = (valueGrid / 6) + 1;
        if(tempGridtoX <= 1){
            tempGridtoX = 1;
        }
        else if(tempGridtoX >= 9){
            tempGridtoX = 9;
        }
        tempCoorX = ((((int)(tempGridtoX)*100)-500))+valueOffSetX;
        return tempCoorX;
    }

    int convertGridY(int valueGrid, int valueOffsetY)
    {
        //convert Grid To Koordinat Y
        if(valueGrid == 2 || valueGrid == 8 || valueGrid == 14 || valueGrid == 20 || valueGrid == 26 || valueGrid == 32 || valueGrid == 38 || valueGrid == 44 || valueGrid == 50){
            tempGridtoY = 3;
            if (valueOffsetY <= -300)
            {
                valueOffsetY = -300;
            } else if (valueOffsetY >= 200)
            {
                valueOffsetY = 200;
            } 
        }
        else if(valueGrid == 3 || valueGrid == 9 || valueGrid == 15 || valueGrid == 21 || valueGrid == 27 || valueGrid == 33 || valueGrid == 39 || valueGrid == 45 || valueGrid == 51){
            tempGridtoY = 4;
            if (valueOffsetY <= -200)
            {
                valueOffsetY = -200;
            } else if (valueOffsetY >= 300)
            {
                valueOffsetY = 300;
            }
        }
        else if(valueGrid == 1 || valueGrid == 7 || valueGrid == 13 || valueGrid == 19 || valueGrid == 25 || valueGrid == 31 || valueGrid == 37 || valueGrid == 43 || valueGrid == 49){
            tempGridtoY = 2;
            if (valueOffsetY <= -400)
            {
                valueOffsetY = -400;
            } else if (valueOffsetY >= 100)
            {
                valueOffsetY = 100;
            } 
        }
        else if(valueGrid == 4 || valueGrid == 10 || valueGrid == 16 || valueGrid == 22 || valueGrid == 28 || valueGrid == 34 || valueGrid == 40 || valueGrid == 46 || valueGrid == 52){
            tempGridtoY = 5;
            if (valueOffsetY <= -100)
            {
                valueOffsetY = -100;
            } else if (valueOffsetY >= 400)
            {
                valueOffsetY = 400;
            }
        }
        else if(valueGrid == 0 || valueGrid == 6 || valueGrid == 12 || valueGrid == 18 || valueGrid == 24 || valueGrid == 30 || valueGrid == 36 || valueGrid == 42 || valueGrid == 48){
            tempGridtoY = 1;
            if (valueOffsetY >= 0)
            {
                valueOffsetY = 0;
            }
        }
        else{
            tempGridtoY = 6;
            if (valueOffsetY <= 0)
            {
                valueOffsetY = 0;
            }
        }
        tempCoorY = (-(((int)(tempGridtoY)*100)-350))+valueOffsetY;
        return tempCoorY;
    }

    bool doneMoved = false,
         setGrid1 = false,
         setGrid2 = false;

    int countMoveGrid1 = 0, //count walk ditempat
        countMoveGrid2 = 0, //count rotate
        countMoveGrid3 = 0; //count walk x,y

    double rotateMoveGrid = 0;
    bool udah = false;

    void moveGrid(int valueGrid, int valueOffSetX, int valueOffSetY)
    { //Bergerak menuju grid yang ditentukan
        double c, s, sn, x, y, r, rotate, speedX, speedY, speedrX, speedrY, nilaiSudut;

        c = cos(msg_yaw);
        s = sin(msg_yaw);
        sn = sin(msg_yaw) * -1;

        if (msg_yaw < 0)
        {
            nilaiSudut = msg_yaw + 360;
        }
        else
        {
            nilaiSudut = msg_yaw;
        }
        // printf("robotPos = %f, %f\n", robotPos_X, robotPos_Y);
        // printf("Nilai Sudut = %.2lf\n", nilaiSudut);
        x = robotPos_X - convertGridX(valueGrid, valueOffSetX); //-240--240	= 0
        y = robotPos_Y - convertGridY(valueGrid, valueOffSetY); //-320--0	= -320
        r = sqrt((x * x) + (y * y));
        // printf("NILAI R = %.2lf\n",r);
        //printf("ADA APA\n");
        if (robotPos_X >= convertGridX(valueGrid, valueOffSetX))
        { //Target ada dibelakang
            // printf("MASUK HAHA\n");
            if (robotPos_Y >= convertGridY(valueGrid, valueOffSetY))
            { //saat ini sebelah kanan
                rotate = -180 + asin(y / r) * (180 / PI);
            }
            else if (robotPos_Y < convertGridY(valueGrid, valueOffSetY))
            { //saat ini sebelah kiri
                rotate = 180 + asin(y / r) * (180 / PI);
            }
        }
        else if (robotPos_X < convertGridX(valueGrid, valueOffSetX))
        { //Target ada didepan
            // printf("MASUK HIHI\n");
            if (robotPos_Y < convertGridY(valueGrid, valueOffSetY))
            { //saat ini sebelah kiri
                //printf("MASUK HUHU\n");
                rotate = 0 - asin(y / r) * (180 / PI);
                //printf("%.2lf\n",valueRotateBody);
            }
            else if (robotPos_Y >= convertGridY(valueGrid, valueOffSetY))
            { //saat ini sebalah kanan
                //printf("MASUK HOHO\n");
                rotate = 0 - asin(y / r) * (180 / PI);
            }
        }
        rotateMoveGrid = rotate;
        // printf("rotate move grid = %.2lf\n", rotateMoveGrid);
        if (robotPos_X >= (convertGridX(valueGrid, valueOffSetX) - 15) && robotPos_X < (convertGridX(valueGrid, valueOffSetX) + 15) &&
            robotPos_Y >= (convertGridY(valueGrid, valueOffSetY) - 15) && robotPos_Y < (convertGridY(valueGrid, valueOffSetY) + 15))
        {
            countMoveGrid1 =
                countMoveGrid2 =
                    countMoveGrid3 = 0;

            posRotateNew =
                setGrid1 =
                    setGrid2 = false;
            doneMoved = true;
        }
        else
        {
            if (countMoveGrid1 >= 5)
            {
                //RY < TY => RY = -30 TY = -25 DY = -30
                if (r < 30)
                {
                    //printf("TIME TO SHOWWWWWW>>>>>> \n");
                    if (countMoveGrid2 >= 5)
                    {
                        if (nilaiSudut > 270 || nilaiSudut <= 90)
                        {
                            //printf("MASUK 0 0 0 0\n");
                            if (posRotateNew)
                            {
                                //printf("SELESAI ROTATE 0\n");
                                if (countMoveGrid3 >= 5)
                                {
                                    if (!setGrid1 && !setGrid2)
                                    {
                                        if (robotPos_X >= convertGridX(valueGrid, valueOffSetX))
                                        {
                                            setGrid1 = true; //RX = -300 TX = -400 DX = -300 -(-400) = 100
                                                             //RX = 30 TX = -400 DX = 30 -(-400) = 430
                                        }
                                        else if (robotPos_X < convertGridX(valueGrid, valueOffSetX))
                                        {
                                            setGrid2 = true; //RX = 300 TX = 400 DX = 300 - 400 = -100
                                                             //RX = -30 TX = -25 DX = -30 -(-25) = -5
                                        }
                                    }
                                    else
                                    {
                                        if (setGrid1)
                                        {
                                            speedX = x * 0.02;
                                            //printf("SET GRID 1\n");
                                        }
                                        else if (setGrid2)
                                        {
                                            speedX = x * -0.02;
                                            //printf("SET GRID 2\n");
                                        }
                                    }

                                    speedY = y * 0.02;

                                    if (speedX >= kejarMax)
                                    {
                                        speedX = kejarMax;
                                    }
                                    else if (speedX <= -0.03)
                                    {
                                        speedX = -0.03;
                                    }
                                    if (speedY >= 0.04)
                                    {
                                        speedY = 0.04;
                                    }
                                    else if (speedY <= -0.04)
                                    {
                                        speedY = -0.04;
                                    }

                                    Walk(speedX, speedY, 0.0);
                                }
                                else
                                {
                                    Walk(0.0, 0.0, 0.0);
                                    countMoveGrid3++;
                                }
                                udah = true;
                            }
                            else
                            {
                                rotateBodyImuNew(0);
                            }
                        }
                        else if (nilaiSudut > 90 || nilaiSudut <= 270)
                        {
                            //printf("MASUK 180 180 180 180\n");
                            if (posRotateNew)
                            {
                                //printf("SELESAI ROTATE 0\n");
                                if (countMoveGrid3 >= 5)
                                {
                                    if (!setGrid1 && !setGrid2)
                                    {
                                        if (robotPos_X >= convertGridX(valueGrid, valueOffSetX))
                                        {
                                            setGrid1 = true; //RX = -300 TX = -400 DX = -300 -(-400) = 100
                                                             //RX = 30 TX = -400 DX = 30 -(-400) = 430
                                        }
                                        else if (robotPos_X < convertGridX(valueGrid, valueOffSetX))
                                        {
                                            setGrid2 = true; //RX = 300 TX = 400 DX = 300 - 400 = -100
                                                             //RX = -30 TX = -25 DX = -30 -(-25) = -5
                                        }
                                    }
                                    else
                                    {
                                        if (setGrid1)
                                        {
                                            speedX = x * 0.02;
                                            //printf("SET GRID 1\n");
                                        }
                                        else if (setGrid2)
                                        {
                                            speedX = x * -0.02;
                                            //printf("SET GRID 2\n");
                                        }
                                    }
                                    speedY = y * 0.02;

                                    if (speedX >= 0.06)
                                    {
                                        speedX = 0.06;
                                    }
                                    else if (speedX <= -0.03)
                                    {
                                        speedX = -0.03;
                                    }
                                    if (speedY >= 0.04)
                                    {
                                        speedY = 0.04;
                                    }
                                    else if (speedY <= -0.04)
                                    {
                                        speedY = -0.04;
                                    }

                                    Walk(speedX, -speedY, 0.0);
                                }
                                else
                                {
                                    Walk(0.0, 0.0, 0.0);
                                    countMoveGrid3++;
                                }
                                udah = true;
                            }
                            else
                            {
                                rotateBodyImuNew(0);
                            }
                        }
                    }
                    else
                    {
                        posRotateNew = false;
                        Walk(0.0, 0.0, 0.0);
                        countMoveGrid2++;
                    }
                }
                else
                {
                    setGrid1 = false;
                    setGrid2 = false;
                    posRotateNew = false;
                    countMoveGrid2 = 0;
                    countMoveGrid3 = 0;
                    jalanDirection(kejarMax, 0.0, rotate);
                }
            }
            else
            {
                Walk(0.0, 0.0, 0.0);
                countMoveGrid1++;
            }
        }
    }

    void refreshMoveGrid()
    {
        posRotateNew = false;
        doneMoved = false;
        udah = false;
        countMoveGrid1 =
            countMoveGrid2 =
                countMoveGrid3 = 
                    cnt_move_to_grid = 0;
        
    }

    void resetCntMoveGrid()
    {
    }

    int Grid = 0;
    void gridLocalization()
    { //Konvert koordinat posisi robot (x,y) jadi Grid posisi robot
        int tempGridX, tempGridY, GridX, GridY;
        GridX = robotPos_X;
        GridY = robotPos_Y;

        if (GridX >= 450)
        {
            GridX = 450;
        }
        else if (GridX <= -450)
        {
            GridX = -450;
        }
        if (GridY >= 300)
        {
            GridY = 300;
        }
        else if (GridY <= -300)
        {
            GridY = -300;
        }

        tempGridX = ((GridX + 450) / 100);
        tempGridY = ((GridY + 300) / 100);

        if (tempGridX >= 8)
        {
            tempGridX = 8;
        }
        else if (tempGridX <= 0)
        {
            tempGridX = 0;
        }
        if (tempGridY >= 5)
        {
            tempGridY = 5;
        }
        else if (tempGridY <= 0)
        {
            tempGridY = 0;
        }

        Grid = tempGridY + (6 * (tempGridX));
        if (Grid <= 0)
        {
            Grid = 0;
        }
        else if (Grid >= 53)
        {
            Grid = 53;
        }
    }

    const double CAMERA_HEIGHT = 0.6; // meters
    const double HEAD_TILT_ZERO_POS = -PI/2.0; // radians
    const double CAMERA_FOV_VERTICAL = 51.0; // degrees
    const double PIXELS_PER_DEGREE = 7.11; // adjust this value based on your camera

    // Calculate the distance to the landmark given the distance to the object and the head tilt angle
    double calculateDistance(double objectDistance, double headTiltAngle) {
        // Convert the tilt angle from degrees to radians and adjust for the zero position
        double tiltAngle = headTiltAngle * PI/180.0 - HEAD_TILT_ZERO_POS;

        // Calculate the angle between the camera and the landmark using trigonometry
        double angleToLandmark = std::atan2(CAMERA_HEIGHT, objectDistance) - tiltAngle;

        // Calculate the distance to the landmark using trigonometry
        double landmarkDistance = CAMERA_HEIGHT / std::tan(angleToLandmark);

        return landmarkDistance;
    }

    // Calculate the number of pixels per meter based on the camera's field of view and the image size
    double calculatePixelsPerMeter(int imageHeight) {
        double fovRadians = CAMERA_FOV_VERTICAL * PI/180.0;
        double pixelsPerMeter = imageHeight / (2.0 * CAMERA_HEIGHT * std::tan(fovRadians/2.0));
        return pixelsPerMeter;
    }

    // Calculate the head tilt angle given the position of the servo
    double calculateHeadTiltAngle(int servoPosition) {
        // Adjust for the zero position of the tilt servo
        double headTiltAngle = servoPosition - HEAD_TILT_ZERO_POS;

        return headTiltAngle * 180.0/PI;
    }

    // Calculate the robot's x and y coordinates on the field
    void calculateRobotPosition(double xL, double yL, double d, double theta) {
        // Map the angle to the range of -180 to 180 degrees
        theta = fmod(theta + 540.0, 360.0) - 180.0;

        // Convert the orientation angle from degrees to radians
        double theta_rad = theta * PI / 180.0;

        // Calculate the robot's x and y coordinates on the field
        double xR = xL + d * cos(theta_rad);
        double yR = yL + d * sin(theta_rad);

        // Print the results
        std::cout << "Robot position: (" << xR << ", " << yR << ")" << std::endl;
        robotPos_X = xR;
        robotPos_Y = yR;
    }

    void updateCoordinate()
    {
        double objectDistance = landmarkDistance*100; // meters
        double servoPosition = headTilt; // radians
        int imageHeight = frame_Y; // pixels
        double landmarkCenter = Pinalty_Y; // pixels

        double headTiltAngle = calculateHeadTiltAngle(servoPosition);
        double pixelsPerMeter = calculatePixelsPerMeter(imageHeight);
        double pixelsFromTop = landmarkCenter - imageHeight/2.0;
        double landmarkDistance = calculateDistance(objectDistance, headTiltAngle)*100;
        double landmarkHeight = pixelsFromTop / pixelsPerMeter;

        std::cout << "Object distance: " << objectDistance << " cm" << std::endl;
        std::cout << "Head tilt angle: " << headTiltAngle << " degrees" << std::endl;
        std::cout << "Pixels per meter: " << pixelsPerMeter << " pixels/meter" << std::endl;
        std::cout << "Landmark distance: " << landmarkDistance << " cm" << std::endl;
        std::cout << "Landmark height: " << landmarkHeight << " meters" << std::endl;

        double orientationAngle = (msg_yaw*-1) + (headPan *180/PI); 
        calculateRobotPosition(-350, 0, objectDistance, orientationAngle);
    }

    // normal search Ball ========================================================================
    bool	posRotasi = false;
    int	rotasi = 0;
    void rotateSearchBall(int rotate) {
        if (rotate >= 0) {
            rotasi = rotate - 180;
            // sudut();

            setPoint1 =  20 + rotasi; //20
            setPoint2 = -20 + rotasi; //20

            if (msg_yaw > setPoint2 && msg_yaw < setPoint1 ) {
                posRotasi = true;
            } else {
                //jalanDirection(0.0, 0.0, rotasi);
                Walk(0.0, 0.0, 0.27);
            }
        } else {
            rotasi = rotate + 180;
            // sudut();

            setPoint1 =  20 + rotasi; //20
            setPoint2 = -20 + rotasi; //20

            if (msg_yaw > setPoint2 && msg_yaw < setPoint1 ) {
                posRotasi = true;
            } else {
                //jalanDirection(0.0, 0.0, rotasi);
                Walk(0.0, 0.0, 0.27);
            }
        }
    }

    int rst_nsb = 0, rst_nsb1 = 0;
    bool nsb_1 = false;
    bool	firstRotate  = false,
        secondRotate = false,
        thirdRotate = false,
        fourthRotate = false,

        firstWalk = false,
        secondWalk = false;
    void normalSearchBall() {
        //mode1
        /* if (!firstRotate) { //ini step yang pertama kali dilakukan saat masuk case 0
            if (searchKe >= 5) {
                Walk(0.0, 0.0, 0.0); //X, Y, W
                firstRotate  = true;
                searchKe = 0;
            } else if (searchKe >= 2 && searchKe < 5) { printf("  2.........\n\n");
                SearchBall(2);
                //tiltSearchBall(0.0);
                Walk(0.0, 0.0, 0.15);
            } else { printf("  1.........\n\n");
                SearchBall(2);
                Walk(0.0, 0.0, 0.0); //X, Y, W
            }
        } else { //setelah rotate pertama tidak dapat, maka cari sambil jalan(dengan imu)
            if (!secondRotate) { //yang pertama kali dilakukan
                if (searchKe >= 3) { printf("  4.........\n\n");
                    SearchBall(2);
                    //tiltSearchBall(0.0);
                    Walk(0.0, 0.0, 0.12);

                    if (searchKe >= 5) {
                        searchKe = 0;
                        secondRotate  = true;
                    }
                } else { printf("  3.........\n\n");
                    SearchBall(2);
                    Walk(kejar, 0.0, 0.0);
                }
            } else { //jalan dengan arah sebaliknya
                if (searchKe >= 6) { printf("  6.........\n\n");
                    SearchBall(2);
                    if (searchKe >= 8) {
                        if (searchKe > 12) {
                            jalanDirection(kejar, 0.0, 0);
                        } else {
                            //searchKe = 0;
                            //firstRotate = secondRotate = false;
                            Walk(kejar, 0.0, 0.0);
                        }
                    } else {
                        //tiltSearchBall(0.0);
                        Walk(0.0, 0.0, 0.12);
                    }
                } else { printf("  5.........\n\n");
                    SearchBall(2);
                    Walk(kejar, 0.0, 0.0);
                }
            }
        } */

        //mode 2
        if (!firstRotate) {
            if (matte > 5) {
                if (searchKe == 1) {
                    if (posRotasi) {
                            motion("0");
                        //Walk(0.0, 0.0, 0.0);
                        //jalanDirection(0.0, 0.0, rotasi);     //before
                        matte = 0;
                        firstRotate = true;
                    } else {
                            motion("9");
                        //headMove(0.0, -1.6);
                        threeSearchBall();
                        rotateSearchBall(saveAngle);
                    }
                } else {
                        motion("0");
                    saveSudutImu();
                    threeSearchBall();
                    //Walk(0.0, 0.0, 0.0);  //before
                }
            } else {
                posRotasi = false;
                sabar = 0;
                searchKe = 0;
                matte++;
            }
        } else {
            if (!secondRotate) {
                if (matte > 5) {
                    if (searchKe == 1) {
                        if (posRotasi) {
                                motion("0");
                            //Walk(0.0, 0.0, 0.0);
                            //jalanDirection(0.0, 0.0, rotasi);     //before
                            matte = 0;
                            secondRotate = true;
                        } else {
                                motion("9");
                            //headMove(0.0, -1.6);
                            threeSearchBall();
                            rotateSearchBall(saveAngle);
                        }
                    } else {
                            motion("0");
                        saveSudutImu();
                        threeSearchBall();
                        //Walk(0.0, 0.0, 0.0);
                        //jalanDirection(0.0, 0.0, saveAngle);  //before
                    }
                } else {
                    posRotasi = false;
                    sabar = 0;
                    searchKe = 0;
                    matte++;
                }
            } else {
                if (!firstWalk) {
                    if (matte > 5) {
                        if (searchKe == 2) {
                                motion("0");
                            //posRotasi = false;
                            saveSudutImu();
                            //Walk(0.0, 0.0, 0.0);
                            //jalanDirection(0.0, 0.0, saveAngle);  //before
                            matte = 0;
                            firstWalk = true;
                        } else {
                                motion("9");
                            threeSearchBall();
                            //Walk(kejar, 0.0, 0.0);
                            jalanDirection(kejar, 0.0, rotasi);
                        }
                    } else {
                        posRotasi = false;
                        sabar = 0;
                        searchKe = 0;
                        matte++;
                    }
                } else {
                    if (!thirdRotate) {
                        if (matte > 5) {
                            if (posRotasi) {
                                    motion("0");
                                //Walk(0.0, 0.0, 0.0);
                                //jalanDirection(0.0, 0.0, rotasi);
                                //sabar = 0;
                                //searchKe = 0;
                                matte = 0;
                                thirdRotate = true;
                            } else {
                                    motion("9");
                                threeSearchBall();
                                rotateSearchBall(saveAngle);
                            }
                        } else {
                            posRotasi = false;
                            sabar = 0;
                            searchKe = 0;
                            matte++;
                        }
                    } else {
                        if (!secondWalk) {
                            if (matte > 5) {
                                if (searchKe == 4) {
                                        motion("0");
                                    //posRotasi = false;
                                    saveSudutImu();
                                    //Walk(0.0, 0.0, 0.0);
                                    //jalanDirection(0.0, 0.0, rotasi);
                                    matte = 0;
                                    secondWalk = true;
                                } else {
                                        motion("9");
                                    threeSearchBall();
                                    //Walk(kejar, 0.0, 0.0);
                                    jalanDirection(kejar, 0.0, rotasi);
                                }
                            } else {
                                posRotasi = false;
                                sabar = 0;
                                searchKe = 0;
                                matte++;
                            }
                        } else {
                            if (!fourthRotate) {
                                if (matte > 5) {
                                    if (posRotasi) {
                                            motion("0");
                                        //Walk(0.0, 0.0, 0.0);
                                        //jalanDirection(0.0, 0.0, rotasi);
                                        //sabar = 0;
                                        //searchKe = 0;
                                        matte = 0;
                                        fourthRotate = true;
                                    } else {
                                            motion("9");
                                        threeSearchBall();
                                        rotateSearchBall(saveAngle);
                                    }
                                } else {
                                    posRotasi = false;
                                    sabar = 0;
                                    searchKe = 0;
                                    matte++;
                                }
                            } else {
                                if (matte > 5) {
                                        motion("9");
                                    threeSearchBall();
                                    //Walk(kejar, 0.0, 0.0);
                                    jalanDirection(kejar, 0.0, rotasi);
                                } else {
                                    posRotasi = false;
                                    sabar = 0;
                                    searchKe = 0;
                                    matte++;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    int convertRow(int grid)
    {
        return grid / 6;
    }

    int convertCol(int grid)
    {
        return grid % 6;
    }

    int coordinates_to_grid(int GridX, int GridY) 
    {
    // Convert the input coordinates to be relative to the center of the grid
    GridX += 450;  // adjust for the center at 4.5m
    GridY = 300 - GridY;  // adjust for the center at 3m

    // Clamp the input coordinates
    GridX = std::max(std::min(GridX, 900), 0);
    GridY = std::max(std::min(GridY, 600), 0);

    // Convert coordinates to grid positions
    int tempGridX = GridX / 100;
    int tempGridY = GridY / 100;

    // Calculate grid position
    int Grid_Pose = tempGridY % 6 + 6 * tempGridX;

    return Grid_Pose;
    }

    // Function to convert row, col to coordinate string
    string toCoord(int row, int col) {
        stringstream ss;
        ss << row << " " << col;
        return ss.str();
    }

    bool done_generated = false;
    // Read in the start and end points for the agent
    int startRow = 0, startCol = 3, endRow = 3, endCol = 2, startRow_ = 0, startCol_ = 3, endRow_ = 7, endCol_ = 7;
    void generate_path()
    {
        if (!done_generated)
        {
            // Define the dimensions of the grid
            int rows = 9;
            int cols = 6;

            // Define the grid as a 2D array of characters
            char grid[rows][cols];

            // Initialize the grid with empty spaces
            for (int i = 0; i < rows; i++) {
                for (int j = 0; j < cols; j++) {
                    grid[i][j] = '.';
                }
            }

            // Read in the values to place in the grid
            int numAgents = 2;

            vector<vector<string>> agentPaths(numAgents);

            for (int i = 0; i < numAgents; i++) {

                // Convert start and end points to coordinates
                string startCoord = toCoord(startRow, startCol);
                string endCoord = toCoord(endRow, endCol);
                string startCoord_ = toCoord(startRow_, startCol_);
                string endCoord_ = toCoord(endRow_, endCol_);

                // Store the path in the agentPaths vector
                vector<string> path {startCoord, endCoord, startCoord_, endCoord_};
                agentPaths[i] = path;
            }

            // Define an array to store the row and column values for each '@' character
            int numObstacle = 2;
            int values[numObstacle][2] = {
                {1, 3},
                {2, 2}
            };

            // Place the '@' characters in the grid
            for (int i = 0; i < numObstacle; i++) {
                int row = values[i][0];
                int col = values[i][1];
                grid[row][col] = '@';
            }

            // Open a file to write the grid and agent paths to
            ofstream outfile("grid.txt");

            // Write the grid to the file
            outfile << rows << " " << cols << endl;
            for (int i = 0; i < rows; i++) {
                for (int j = 0; j < cols; j++) {
                    outfile << grid[i][j] << " ";
                }
                outfile << endl;
            }

            // Write the agent paths to the file
            outfile << numAgents << endl;
            for (int i = 0; i < numAgents; i++) {
                if (i < 1)
                {
                    outfile << agentPaths[i][0] << " " << agentPaths[i][1] << endl;
                    outfile << agentPaths[i][2] << " " << agentPaths[i][3] << endl;
                }
            }

            // Close the file
            outfile.close();

            auto msg = std_msgs::msg::String();
            msg.data = "request";
            request_pub->publish(msg);

            done_generated = true;
        }
    }

    int cnt_path = 0, last_next_grid = 0, next_grid = 0, cnt_path_ = 0;
    bool done_path_tracked = false;
    void path_tracking(int offsetX, int offsetY)
    {
        printf("... robotPos : %.2f, %.2f\n", robotPos_X, robotPos_Y);
        printf("... grid = %d\n", Grid);
        printf("... next_grid  = %d\n", next_grid);
        printf("... cnt_path : %d\n", cnt_path);
        printf("... size : %d\n", grid_list.size());
        if (cnt_path == grid_list.size()-1)
        {
            next_grid = grid_list.at(grid_list.size() - 1);
            if (doneMoved)
            {
                motion("0");
                done_path_tracked = true;
            } else 
            {
                new_out_grid(next_grid, offsetX, offsetY, true);
            }
        }
        else
        {
            next_grid = grid_list.at(cnt_path);
            if (Grid != next_grid && !doneMoved)
            {
                printf("...hihihi\n");
                printf("...cnt_path : %d\n", cnt_path);
                new_out_grid(next_grid, 0, 0, true);
            } else 
            {
                printf("...hehehe\n");
                refreshMoveGrid();
                cnt_path++;
                last_next_grid = next_grid;
            }
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr trackbarSubscription_;
    rclcpp::Subscription<bfc_msgs::msg::Button>::SharedPtr button_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr bounding_boxes_;
    rclcpp::Subscription<bfc_msgs::msg::Coordination>::SharedPtr robot1Subscription_;
    rclcpp::Subscription<bfc_msgs::msg::Coordination>::SharedPtr robot2Subscription_;
    rclcpp::Subscription<bfc_msgs::msg::Coordination>::SharedPtr robot3Subscription_;
    rclcpp::Subscription<bfc_msgs::msg::Coordination>::SharedPtr robot4Subscription_;
    rclcpp::Subscription<bfc_msgs::msg::Coordination>::SharedPtr robot5Subscription_;
    rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr gameControllerSubscription_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr voltage_n_odometry;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr GridSub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr TargetPoseSub_;
    rclcpp::Subscription<darknet_ros_msgs::msg::BoundingBoxes>::SharedPtr subscriber_darknet;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr object_distance;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ball_distance;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr path_finding_subscription_;
    rclcpp::Subscription<darknet_ros_msgs::msg::ObjectCount>::SharedPtr subscriber_object_count;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr keyboard_teleop;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ball_pose_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr camera_odom_sub;
    rclcpp::Publisher<bfc_msgs::msg::Coordination>::SharedPtr robotCoordination_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_mot_;
    rclcpp::Publisher<bfc_msgs::msg::HeadMovement>::SharedPtr cmd_head_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr Odometry_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Update_coor_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr request_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ball_status_pub;
    BehaviorTreeFactory factory;
    Tree tree;
    PublisherZMQ *pubZ;
    zmqpp::context context_;
    zmqpp::socket socket_;

    int odom_pose_x, odom_pose_y, odom_pose_z, firstStateLocalization, stateLocalization;
    int msg_strategy, msg_kill, msg_roll, msg_pitch, msg_yaw, modePlay;
    int robotKick = 0;

    int robot1Id, robot1Status, robot1State, robot1GridPosition, robot1FBall, robot1DBall, robot1GridBall, robot1BackIn, robot1Voltage, robot1KickOff;
    int robot2Id, robot2Status, robot2State, robot2GridPosition, robot2FBall, robot2DBall, robot2GridBall, robot2BackIn, robot2Voltage, robot2KickOff;
    int robot3Id, robot3Status, robot3State, robot3GridPosition, robot3FBall, robot3DBall, robot3GridBall, robot3BackIn, robot3Voltage, robot3KickOff;
    int robot4Id, robot4Status, robot4State, robot4GridPosition, robot4FBall, robot4DBall, robot4GridBall, robot4BackIn, robot4Voltage, robot4KickOff;
    int robot5Id, robot5Status, robot5State, robot5GridPosition, robot5FBall, robot5DBall, robot5GridBall, robot5BackIn, robot5Voltage, robot5KickOff;

    int Ball_X, Ball_Y, Ball_W, Ball_H, Ball_D, Pinalty_X, Pinalty_Y, Xcross_X, Xcross_Y;
    int B_pole_X, B_pole_Y, T_pole_X, T_pole_Y;
    int Goal_X, Goal_Y, Goal_W, Goal_H, Goal_LH, Goal_RH, Goal_C, Goal_LD, Goal_RD;
    int Goal_LX, Goal_LY, Goal_RX, Goal_RY, Xcross_RX, Xcross_RY, Xcross_LX, Xcross_LY;
    int Pinalty_D, Lcross_LD, Lcross_RD, Xcross_LD, Xcross_RD, Tcross_LD, Tcross_RD;
    int Left_X_Cross_X, Left_X_Cross_Y, Right_X_Cross_X, Right_X_Cross_Y, Left_T_Cross_X, Left_T_Cross_Y, Right_T_Cross_X, Right_T_Cross_Y;
    int Left_Corner_X, Left_Corner_Y, Right_Corner_X, Right_Corner_Y, Left_L_Cross_X, Left_L_Cross_Y, Right_L_Cross_X, Right_L_Cross_Y;
    int Left_T_Corner_X, Left_T_Corner_Y, Right_T_Corner_X, Right_T_Corner_Y, Robot_X, Robot_Y, goal_L_pole_X, goal_L_pole_Y, goal_R_pole_X, goal_R_pole_Y;
    int Left_T_Pole, Right_T_Pole, Left_B_Pole, Right_B_Pole;

    int State, Player, Team,
        FirstHalf,
        Version,
        PacketNumber,
        PlayerTeam,
        GameTipe,
        KickOff,
        SecondaryState,
        DropTeam,
        DropTime,
        Remaining,
        SecondaryTime,
        // ket : 1 = untuk data GameController yang kiri
        //	 2 = untuk data GameController yang kanan
        timNumber1,
        timNumber2,
        timColour1,
        timColour2,
        Score1,
        Score2,
        Penaltyshoot1,
        Penaltyshoot2,
        Singleshoot1,
        Singleshoot2,
        Coachsequence1,
        Coachsequence2,

        Penalty1,
        Penalty2,
        TimeUnpenalis1,
        TimeUnpenalis2,
        YellowCard1,
        YellowCard2,
        RedCard1,
        RedCard2;

    int team, barelang_color, dropball, max_current;
    /////////////////////////////////////////////////////////
    ///////////////////Variable Global///////////////////////
    /////////////////////////////////////////////////////////
    int robotNumber,
        stateCondition = 0,
        firstStateCondition = 0, // switch strategy

        stateGameController = 0,

        state,     // kill n run
        lastState, // kill n run
        wait = 0,
        delay = 0,
        delayWaitBall = 0,
        countBearing = 0,
        countDribble = 0,
        tunda = 0,
        tunggu = 0,
        waiting = 0,
        waitTracking = 0,
        reset = 0,
        matte = 0,
        chotto = 0,
        sudutTendang = 0,
        modeTendang = 0,

        modeKick = 1,

        saveAngle = 0,
        lastDirection = 0,

        countHilang = 0,

        confirmsBall = 0,
        countTilt = 0,
        sumTilt = 0;

    double headPan, // f.Kepala
        headTilt,   // f.Kepala
        posPan,
        posTilt,
        errorPan,  // f.Kepala
        errorTilt, // f.Kepala
        PPan,      // f.Kepala
        PTilt,     // f.Kepala

        ball_panKP,  // PID trackBall
        ball_panKD,  // PID trackBall
        ball_tiltKP, // PID trackBall
        ball_tiltKD, // PID trackBall

        goal_panKP,  // PID trackGoal
        goal_panKD,  // PID trackGoal
        goal_tiltKP, // PID trackGoal
        goal_tiltKD,

        land_panKP,  // PID trackLand
        land_panKD,  // PID trackLand
        land_tiltKP, // PID trackLand
        land_tiltKD, // PID trackLand

        panMax, panMin,

        RollCM,
        PitchCM,
        YawCM; // PID trackGoal

    double ballPositioningSpeed,
        pTiltTendangKanan,
        pPanTendangKanan,
        pTiltTendangKiri,
        pPanTendangKiri,
        pTiltOper,
        pPanOper,
        cSekarang,
        cAktif,
        posTiltLocal,
        posTiltGoal,
        erorrXwalk,
        erorrYwalk,
        erorrAwalk,
        jalan,
        lari,
        kejar,
        kejarMid,
        kejarMax,
        tinggiRobot,
        outputSudutY1,
        inputSudutY1,
        outputSudutY2,
        inputSudutY2,
        outputSudutX1,
        inputSudutX1,
        outputSudutX2,
        inputSudutX2,
        frame_X,
        frame_Y,
        rotateGoal_x,
        rotateGoal_y,
        rotateGoal_a,

        sendAngle,

        robotPos_X,   // om
        initialPos_X, // ov
        robotPos_Y,   // om
        initialPos_Y, // om
        initialAlpha,

        kurama = 0,
        offsetPan = 0,
        myAccrX = 0,
        myAccrY = 0;

    int sudutTengah,
        sudutKanan,
        sudutKiri,
        tendangJauh,
        tendangDekat,
        tendangSamping;

    bool play,
        kanan,
        kiri,
        forceKanan,
        forceKiri,
        exeCutor = false,
        useVision, // f.setting
        useRos,
        useGameController,       // f.setting
        useCoordination,         // f.setting
        useLocalization,         // f.setting
        useFollowSearchGoal,     // f.setting
        useImu,                  // f.setting
        useDribble,              // f.setting
        dribbleOnly,             // f.setting
        useSearchGoal,           // f.setting
        useSideKick,             // f.setting
        useLastDirection,        // f.setting
        useNearFollowSearchGoal, // f.setting
        useUpdateCoordinate,     // f.setting
        usePenaltyStrategy,
        useDisplay,
        useWalkKick,
        useFollowExecutor,
        useKickOffGoal;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<main_strategy>();
    node->run();
    // while (rclcpp::ok())
    // {
    //     node->run();
    // }
    rclcpp::shutdown();
    return 0;
}
