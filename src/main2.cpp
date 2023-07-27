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
        : Node("main_strategy")
    {
        declareParameters();
        getParameters();

        robotNumber = this->get_parameter("robotNumber").as_int();

        button_ = this->create_subscription<bfc_msgs::msg::Button>(
            "button", 10,
            std::bind(&main_strategy::readButton, this, std::placeholders::_1));
        imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu", 10,
            std::bind(&main_strategy::readImu, this, std::placeholders::_1));
        gameControllerSubscription_ = this->create_subscription<std_msgs::msg::Int64MultiArray>(
            "game_controller", 10,
            std::bind(&main_strategy::readGameControllerData, this, std::placeholders::_1));

        trackbarSubscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "trackbar", 10,
            std::bind(&main_strategy::readTrackbar, this, std::placeholders::_1));

        voltage_n_odometry = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "voltodom", 10,
            std::bind(&main_strategy::readVoltageAndOdom, this, std::placeholders::_1));

        GridSub_ = this->create_subscription<std_msgs::msg::Int32>(
            "grid", 10,
            std::bind(&main_strategy::readGrid, this, std::placeholders::_1));

        subscriber_darknet = this->create_subscription<darknet_ros_msgs::msg::BoundingBoxes>(
            "darknet_ros/bounding_boxes", 10,
            std::bind(&main_strategy::callbackBoundingBox, this, std::placeholders::_1));
        
        subscriber_object_count = this->create_subscription<darknet_ros_msgs::msg::ObjectCount>(
            "darknet_ros/found_object", 10,
            std::bind(&main_strategy::callbackFoundObject, this, std::placeholders::_1));

        TargetPoseSub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            std::bind(&main_strategy::callbackTargetPose, this, std::placeholders::_1));

        object_distance = this->create_subscription<std_msgs::msg::Float32>(
            "object_distance", 10,
            std::bind(&main_strategy::callbackObjectDistance, this, std::placeholders::_1));

        path_finding_subscription_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            "path_finding", 10,
            std::bind(&main_strategy::callbackPathFinding, this, std::placeholders::_1));

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

        timer_ = this->create_wall_timer(40ms, std::bind(&main_strategy::timer_callback, this));

        factory.registerSimpleCondition("isKillrun", std::bind(&main_strategy::isKillrun, this));
        factory.registerSimpleCondition("isReady", std::bind(&main_strategy::isReady, this));
        factory.registerSimpleCondition("isInitial", std::bind(&main_strategy::isInitial, this));
        factory.registerSimpleCondition("ispickUp", std::bind(&main_strategy::ispickUp, this));
        factory.registerSimpleCondition("isKickoff", std::bind(&main_strategy::isKickoff, this));
        factory.registerSimpleCondition("isIniattack", std::bind(&main_strategy::isIniattack, this));
        factory.registerSimpleCondition("isInidefensleft", std::bind(&main_strategy::isInidefensleft, this));
        factory.registerSimpleCondition("isInidefensright", std::bind(&main_strategy::isInidefensright, this));
        factory.registerSimpleCondition("isAttacker", std::bind(&main_strategy::isAttacker, this));
        factory.registerSimpleCondition("isDefenderleft", std::bind(&main_strategy::isDefenderleft, this));
        factory.registerSimpleCondition("isDefenderright", std::bind(&main_strategy::isDefenderright, this));
        factory.registerSimpleCondition("iscekPos", std::bind(&main_strategy::iscekPos, this));
        factory.registerSimpleCondition("isSet", std::bind(&main_strategy::isSet, this));
        factory.registerSimpleCondition("isPlay", std::bind(&main_strategy::isPlay, this));
        factory.registerSimpleCondition("unPenalty", std::bind(&main_strategy::unPenalty, this));
        factory.registerSimpleCondition("isPassed", std::bind(&main_strategy::isPassed, this));
        factory.registerSimpleCondition("isStepBreak", std::bind(&main_strategy::isStepBreak, this));
        factory.registerSimpleCondition("isGoalie", std::bind(&main_strategy::IsGoalie, this));
        factory.registerSimpleCondition("isKeeperZone", std::bind(&main_strategy::isKeeperZone, this));
        factory.registerSimpleCondition("isExecutor", std::bind(&main_strategy::isExecutor, this));
        factory.registerSimpleCondition("isGoalkeeper", std::bind(&main_strategy::isGoalkeeper, this));
        //factory.registerSimpleCondition("isGoal", std::bind(&main_strategy::isGoal, this));
        factory.registerSimpleAction("robotDiam", std::bind(&main_strategy::robotDiam, this));
        factory.registerSimpleAction("iniPosattack", std::bind(&main_strategy::iniPosattack, this));
        factory.registerSimpleAction("iniPosdefensleft", std::bind(&main_strategy::iniPosdefensleft, this));
        factory.registerSimpleAction("iniPosdefensright", std::bind(&main_strategy::iniPosdefensright, this));
        factory.registerSimpleAction("Release", std::bind(&main_strategy::Release, this));
        factory.registerSimpleAction("Attack", std::bind(&main_strategy::Attack, this));
        factory.registerSimpleAction("Defensleft", std::bind(&main_strategy::Defensleft, this));
        factory.registerSimpleAction("Defensright", std::bind(&main_strategy::Defensright, this));
        factory.registerSimpleAction("localizationKickoff", std::bind(&main_strategy::localizationKickoff, this));
        factory.registerSimpleAction("Searchball", std::bind(&main_strategy::Searchball, this));
        factory.registerSimpleAction("Pattack", std::bind(&main_strategy::Pattack, this));
        factory.registerSimpleAction("Delayrobot", std::bind(&main_strategy::Delayrobot, this));
        //factory.registerSimpleAction("Gotoposition", std::bind(&main_strategy::Gotoposition, this));
        factory.registerSimpleAction("Ceklocalization", std::bind(&main_strategy::Ceklocalization, this));
        factory.registerSimpleAction("MoveToGrid", std::bind(&main_strategy::MoveToGrid, this));
        factory.registerSimpleAction("UpdateCoor", std::bind(&main_strategy::UpdateCoor, this));
        factory.registerSimpleAction("RobotPassing", std::bind(&main_strategy::RobotPassing, this));
        factory.registerSimpleAction("sendPassingCoordination", std::bind(&main_strategy::sendPassingCoordination, this));
        factory.registerSimpleAction("JustWalk", std::bind(&main_strategy::JustWalk, this));
        factory.registerSimpleAction("InGoalie", std::bind(&main_strategy::InGoalie, this));
        factory.registerSimpleAction("IniPosGoalie", std::bind(&main_strategy::IniPosGoalie, this));
        factory.registerSimpleAction("HeadUp", std::bind(&main_strategy::HeadUp, this));
        factory.registerSimpleAction("GoalKeeper", std::bind(&main_strategy::GoalKeeper, this));
        factory.registerSimpleAction("Support", std::bind(&main_strategy::Support, this));
        factory.registerSimpleAction("Communication", std::bind(&main_strategy::Communication, this));
        factory.registerSimpleAction("Homing", std::bind(&main_strategy::Homing, this));
        factory.registerSimpleAction("SelfUpdate", std::bind(&main_strategy::SelfUpdate, this));
        factory.registerSimpleAction("GetData", std::bind(&main_strategy::GetData, this));
        factory.registerSimpleAction("Shoot", std::bind(&main_strategy::Shoot, this));
        factory.registerSimpleAction("PathFinding", std::bind(&main_strategy::PathFinding, this));
        factory.registerSimpleAction("PathTracking", std::bind(&main_strategy::PathTracking, this));
        factory.registerSimpleAction("LookToGrid", std::bind(&main_strategy::LookToGrid, this));
        tree = factory.createTreeFromFile(tree_path);
        pubZ = new BT::PublisherZMQ(tree);
    }

    void run()
    {
        executor_.add_node(this->get_node_base_interface());
        executor_.spin();
    }

private:
    NodeStatus isKillrun()
    {
        if (lastState != msg_kill)
        {
            if (msg_kill == 0)
            {
                stateCondition=firstStateCondition;
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
            return NodeStatus::SUCCESS;
        }
        else
        {
            return NodeStatus::FAILURE;
        }
    }

    NodeStatus isInitial()
    {
        if (State == 0)
        {
            motion("0");
            resetVariable();
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    int lastStateLocalization = 0, lastScore = 0;
    NodeStatus isReady()
    {
        if (stateLocalization != lastStateLocalization)
        {
            if (stateLocalization != 150)
            {
                setWaktu();
                refreshMoveGrid();
            }
            lastStateLocalization = stateLocalization;
        }

        if ((State == 1 && Remaining == 600) || (State == 1 && Remaining == 300)) // ketika awal masuk
        {
            stateLocalization = 0;
            return NodeStatus::SUCCESS;
        }
        else if ((State == 1 && Remaining != 600) || (State == 1 && Remaining != 300)) // ketika terjadi goal
        {
            stateLocalization = 1;
            return NodeStatus::SUCCESS;
        }
        else
        {
            stateLocalization = 150;
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus isSet()
    {
        if (State == 2)
        {
            motion("0");
            resetVariable();
            resetCntMoveGrid();
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus isPlay()
    {
        if (State == 3)
        {
            return NodeStatus::SUCCESS;
        }
        stateCondition = 0;
        return NodeStatus::FAILURE;
    }

    bool Pickup = false;
    NodeStatus ispickUp()
    {
        if (Penalty1 == 5 && timColour1 == barelang_color || Penalty2 == 5 && timColour2 == barelang_color)
        {
            resetpickup();
            refreshMoveGrid();
            motion("0");
            Pickup = true;

            //Attack
            if (msg_strategy == 1 && msg_yaw > 0) //dari sisi kiri lapangan
            {
                saka1 = true;
                saki1 = true;
                saka2 = false;
                saki2 = false;
                saka3 = false;
                saki3 = false;
                initialPos_X = -240;
                initialPos_Y = -310;
            }
            if (msg_strategy == 1 && msg_yaw < 0) //dari sisi kanan lapangan
            {
                saka1 = true;
                saki1 = true;
                saka2 = false;
                saki2 = false;
                saka3 = false;
                saki3 = false;
                initialPos_X = -240;
                initialPos_Y = 310;
            }

            //Defens Right
            if (msg_strategy == 2 && msg_yaw > 0) //dari sisi kiri lapangan
            {
                saka1 = false;
                saki1 = false;
                saka2 = true;
                saki2 = true;
                saka3 = false;
                saki3 = false;
                initialPos_X = -240;
                initialPos_Y = -310;
            }
            if (msg_strategy == 2 && msg_yaw < 0) //dari sisi kanan lapangan
            {
                saka1 = false;
                saki1 = false;
                saka2 = true;
                saki2 = true;
                saka3 = false;
                saki3 = false;
                initialPos_X = -240;
                initialPos_Y = 310;
            }

            //Defens Left
            if (msg_strategy == 3 && msg_yaw > 0) //dari samping kiri lapangan
            {
                saka1 = false;
                saki1 = false;
                saka2 = false;
                saki2 = false;
                saka3 = true;
                saki3 = true;
                initialPos_X = -240;
                initialPos_Y = -310;
            }
            if (msg_strategy == 3 && msg_yaw < 0) //dari samping kanan lapangan
            {
                saka1 = false;
                saki1 = false;
                saka2 = false;
                saki2 = false;
                saka3 = true;
                saki3 = true;
                initialPos_X = -240;
                initialPos_Y = 310;
            }
        }
        if (Pickup == true)
        {
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus isKickoff()
    {
        if (KickOff == barelang_color)
        {
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus isIniattack()
    {
        if (msg_strategy == 1 || Remaining != 600 && msg_strategy == 1)
        {
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus isInidefensleft()
    {
        if (msg_strategy == 3 || Remaining != 600 && msg_strategy == 2)
        {
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus isInidefensright()
    {
        if (msg_strategy == 2 || Remaining != 600 && msg_strategy == 3)
        {
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus isAttacker()
    {
        if (msg_strategy == 1)
        {
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus isDefenderleft()
    {
        if (msg_strategy == 3)
        {
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus isDefenderright()
    {
        if (msg_strategy == 2)
        {
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus unPenalty()
    {
        if (Penalty1 != 5 && timColour1 == barelang_color || Penalty2 != 5 && timColour2 == barelang_color)
        {
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    // NodeStatus isGoal()
    // {
    //     if(Remaining != 600 && State == 1)
    //     {
    //         return NodeStatus::SUCCESS;
    //     }

    //     return NodeStatus::FAILURE;
    // }

    bool rbtdiam = false;
    NodeStatus robotDiam()
    {
        motion("0");
        return NodeStatus::SUCCESS;
    }

    NodeStatus iniPosattack()
    {
        if (msg_strategy == 1) // Attack
        {
            // initialPos_X = -360;
            // initialPos_Y = 0;
            initialPos_X = -50;
            initialPos_Y = 310;
        }
        return NodeStatus::SUCCESS;
    }

    NodeStatus iniPosdefensleft()
    {
        if (msg_strategy == 3) // Defens Left
        {
            // initialPos_X = -360;
            // initialPos_Y = -130;
            initialPos_X = -240;
            initialPos_Y = -310;
        }
        return NodeStatus::SUCCESS;
    }

    NodeStatus iniPosdefensright()
    {
        if (msg_strategy == 2) // Defens Right
        {
            // initialPos_X = -360;
            // initialPos_Y = 130;
            initialPos_X = -240;
            initialPos_Y = 310;
        }
        return NodeStatus::SUCCESS;
    }

    int rst_mov_attack = 0;
    NodeStatus Attack()
    {
        // if (doneMoved)
        // {
        //     return NodeStatus::SUCCESS;
        // }
        // else
        // {
        //     // IC();
        //     moveGrid(27, -50, 50);
        // }
        if (rst_mov_attack > 5)
        {
            // if (distance < 30)
            if (doneMoved)
            {
                // if (posRotateNew)
                // {
                //     motion("0");
                // } else 
                // {
                //     rotateBodyImuNew(0);jdjjdd
                // }
                motion("0");
            } else 
            {
                new_out_grid(21, 50, 50, true);
            }
        } else 
        {
            refreshMoveGrid();
            rst_mov_attack++;
        }
        return NodeStatus::FAILURE;
    }

    int rst_dev_left = 0;
    NodeStatus Defensleft()
    {
        // if (doneMoved)
        // {
        //     return NodeStatus::SUCCESS;
        // }
        // else
        // {
        //     // IC();
        //     moveGrid(21, -50, -50);
        // }
        if (rst_dev_left > 5)
        {
            new_out_grid(16, 0, 0, true);
        } else 
        {
            refreshMoveGrid();
            rst_dev_left++;
        }
        return NodeStatus::FAILURE;
    }

    int rst_dev_right = 0;
    NodeStatus Defensright()
    {
        // if (doneMoved)
        // {
        //     return NodeStatus::SUCCESS;
        // }
        // else
        // {
        //     // IC();
        //     moveGrid(22, -50, 50);
        // }
        if (rst_dev_right > 5)
        {
            new_out_grid(13, 0, 0, true);
        } else 
        {
            refreshMoveGrid();
            rst_dev_right++;
        }
        return NodeStatus::FAILURE;
    }

    int rst_loc_kick_off = 0;
    NodeStatus localizationKickoff()
    {
        if (msg_strategy == 1)
        {
            // if (doneMoved)
            // {
            //     return NodeStatus::SUCCESS;
            // }
            // else
            // {
            //     // IC();
            //     // moveGrid(21, -50, 50);
            //     new_out_grid(21, -50, 50, true)
            // }
            if (rst_loc_kick_off > 5)
            {
                new_out_grid(21, -50, 50, true);
            } else 
            {
                refreshMoveGrid();
                rst_loc_kick_off++;
            }
        }

        if (msg_strategy == 2)
        {
            // if (doneMoved)
            // {
            //     return NodeStatus::SUCCESS;
            // }
            // else
            // {
            //     // IC();
            //     moveGrid(22, -50, 50);
            // }
            if (rst_loc_kick_off > 5)
            {
                new_out_grid(16, 0, 0, true);
            } else 
            {
                refreshMoveGrid();
                rst_loc_kick_off++;
            }
        }

        if (msg_strategy == 3)
        {
            // if (doneMoved)
            // {
            //     return NodeStatus::SUCCESS;
            // }
            // else
            // {
            //     // IC();
            //     moveGrid(21, -50, -50);
            // }
            if (rst_loc_kick_off > 5)
            {
                new_out_grid(13, 0, 0, true);
            } else 
            {
                refreshMoveGrid();
                rst_loc_kick_off++;
            }
        }
        return NodeStatus::FAILURE;
    }

    bool saka1 = false;
    bool saki1 = false;
    bool saka2 = false;
    bool saki2 = false;
    bool saka3 = false;
    bool saki3 = false;
    int rst_release = 0;
    NodeStatus Release()
    {
        if (ballLost(20))
        {
            threeSearchBall();
            if (saka1 == true || saki1 == true)
            {
                // if (doneMoved)
                // {
                //     Pickup = false;
                //     stateCondition = 0;
                //     return NodeStatus::SUCCESS;
                // }
                // else
                // {
                //     // IC();
                //     moveGrid(27, -50, 50);
                // }
                if (rst_release > 5)
                {
                    new_out_grid(27, -50, 50, true);
                } else 
                {
                    refreshMoveGrid();
                    rst_release++;
                }
            }

            if (saka2 == true || saki2 == true)
            {
                // if (doneMoved)
                // {
                //     Pickup = false;
                //     return NodeStatus::SUCCESS;
                // }
                // else
                // {
                //     // IC();
                //     moveGrid(22, -50, 50);
                // }
                if (rst_release > 5)
                {
                    new_out_grid(22, -50, 50, true);
                } else 
                {
                    refreshMoveGrid();
                    rst_release++;
                }
            }

            if (saka3 == true || saki3 == true)
            {
                // if (doneMoved)
                // {
                //     Pickup = false;
                //     return NodeStatus::SUCCESS;
                // }
                // else
                // {
                //     // IC();
                //     moveGrid(21, -50, -50);
                // }
                if (rst_release > 5)
                {
                    new_out_grid(21, -50, -50, true);
                } else 
                {
                    refreshMoveGrid();
                    rst_release++;
                }
            }
            delayWaitBall = 0;
        } 
        else 
        {
            trackBall();
            if (delayWaitBall > 20)
            {
                Pickup = false;
            } else 
            {
                delayWaitBall++;
            }
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus Searchball()
    {
        if (ballLost(20))
        {
            searchBallRectang(-1.6, -1.6, -0.8, 1.6);
        }
        else
        {
            trackBall();
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus Delayrobot()
    {
        //threeSearchBall();   
        if(SecondaryTime == 0)
        {
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    // NodeStatus Gotoposition()
    // {
    //     if(msg_strategy == 1)
    //     {
    //         if (doneMoved)
    //         {
    //             return NodeStatus::SUCCESS;
    //         }else
    //         {
    //             // IC();
    //             moveGrid(21, -50, -50);
    //         }
    //     }

    //     if(msg_strategy == 2)
    //     {
    //         if(doneMoved)
    //         {
    //             return NodeStatus::SUCCESS;
    //         }else
    //         {
    //             // IC();
    //             moveGrid(22, -50 ,50);
    //         }
    //     }

    //     if(msg_strategy == 3)
    //     {
    //         if(doneMoved)
    //         {
    //             return NodeStatus::SUCCESS;
    //         }else
    //         {
    //             // IC();
    //             moveGrid(21, -50 ,-50);
    //         }
    //     }
    //     return NodeStatus::FAILURE;
    // }

    NodeStatus Pattack()
    {
        printf("...StateCondition : %d\n", stateCondition);
        switch (stateCondition)
        {
        case 0: // search ball

            if (ballLost(20))
            {
                motion("0");
                threeSearchBall();
                // normalSearchBall();
            }
            else
            {
                trackBall();
                if (delayWaitBall > 20)
                {

                    stateCondition = 1;
                }
                else
                {
                    reset_velocity();
                    set_velocity(0.0, 0.0, 0.0);
                    delayWaitBall++;
                }
            }

            break;

        case 1: // follow Ball
            if (ballLost(20))
            {
                resetCase0();
                stateCondition = 0;
            }
            else
            {
                trackBall();
                printf("...headTilt : %.2f\n", headTilt);
                if (headTilt >= cAktif && headPan >= -0.4 && headPan <= 0.4)
                {
                    new_out_grid(50, 50, -50, false);
                    if (!useSideKick)
                    {
                        if (theta > 0)
                        {
                            sudutTendang = theta + 10;
                        } else if (theta < 0)
                        {
                            sudutTendang = theta - 10;
                        } else
                        {
                            sudutTendang = theta;
                        }
                        modeTendang = 1;
                    }
                    else 
                    {
                        if (msg_yaw > 45)
                        {
                            sudutTendang = 75 + theta + 30;
                            modeTendang = 4;
                        }
                        else if (msg_yaw < -45)
                        {
                            sudutTendang = -75 + theta - 30;
                            modeTendang = 3;
                        }
                        else
                        {
                            if (theta > 0)
                            {
                                sudutTendang = theta + 10;
                            } else if (theta < 0)
                            {
                                sudutTendang = theta - 10;
                            } else
                            {
                                sudutTendang = theta;
                            }
                            modeTendang = 1;
                        }
                    }
                    resetCase2(); //2
                    stateCondition = 2; //2
                    // motion("0");
                }
                else
                {
                    followBall(0);
                }
            }

            break;

        case 2: // Imu
            if (ballLost(20))
            {
                resetCase0();
                stateCondition = 0;
            }
            else
            {
                trackBall();
                // if (headTilt >= cSekarang && headPan >= -0.4 && headPan <= 0.4)
                // {
                    if (robotDirection && headTilt >= cAktif && headPan >= -0.4 && headPan <= 0.4)
                    {
                        reset_velocity();
                        resetCase3();
                        stateCondition = 3;
                    }
                    else
                    {
                        Imu(sudutTendang, cSekarang);
                    }
                // }
                // else
                // {
                //     resetCase0();
                //     stateCondition = 0;
                // }
            }

            break;

        case 3: // BallPos -> Tendang

            if (ballLost(20))
            {
                resetCase0();
                stateCondition = 0;
            }
            else
            {
                trackBall();
                // if (headTilt >= cSekarang && headPan >= -0.5 && headPan <= 0.5)
                // {
                    if (tendang)
                    {
                        resetCase4();
                        stateCondition = 4;
                    }
                    else
                    {
                        kick(modeTendang);
                    }
                // }
                // else
                // {
                //     followBall(0);
                // }
            }

            break;

        case 4: // search after kick

            if (delay > 5)
            {
                if (useWalkKick && (modeTendang == 5 || modeTendang == 6))
                {
                    cekWaktu(1);
                }
                else
                {
                    cekWaktu(5);
                }
                if (timer)
                {
                    if (ballLost(20))
                    {
                        resetCase0();
                        stateCondition = 0;
                    }
                    else
                    {
                        trackBall();
                        if (headTilt >= cSekarang)
                        {
                            resetCase3();
                            stateCondition = 3;
                        }
                        else
                        {
                            resetCase0();
                            stateCondition = 0;
                        }
                    }
                }
                else
                {
                    if (modeTendang == 4)
                    {
                        headMove(1.6, -1.6);
                    }
                    else if (modeTendang == 3)
                    {
                        headMove(-1.6, -1.6);
                    }
                    else
                    {
                        headMove(0.0, -1.6);
                    }
                }
            }
            else
            {
                reset_velocity();
                setWaktu();
                delay++;
            }

            break;

        case 5:
            if (doneBanting)
            {
                if (!robotFall)
                {
                    if (posRotateNew)
                    {
                        motion("0");
                    }
                    else
                    {
                        rotateBodyImuNew(0);
                    }
                }
            }
            else
            {
                banting();
            }
            break;

        default:
            break;
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus iscekPos()
    {
        if (msg_strategy == 1) // Attack
        {
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus Ceklocalization()
    {
        if (msg_strategy == 1) // Attack
        {
            initialPos_X = -360;
            initialPos_Y = 0;

            if (doneMoved)
            {
                return NodeStatus::SUCCESS;
            }
            else
            {
                // IC();
                moveGrid(21, 50, 0);
            }
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus MoveToGrid()
    {
        initialPos_X = -350;
        initialPos_Y = 0;
        if (doneMoved)
        {
            motion("0");
        } else 
        {
            new_out_grid(29, 0, 0, true);
        }
        return NodeStatus::FAILURE;
    }

    void saveToCSV(int grid, double posx, double posy, double yaw, double pan, double tilt, int c0_x, int c0_y, int c1_x, int c1_y, int c2_x, int c2_y, int c3_x, int c3_y, int c4_x, int c4_y, int c5_x, int c5_y, int c6_x, int c6_y, int c7_x, int c7_y, int c8_x, int c8_y, int c9_x, int c9_y, int c10_x, int c10_y)
    {
        std::ofstream file;
        file.open("/home/barelangfc3/bfc_ros2/src/main/data_posisi.csv", std::ios::out | std::ios::app);
        file << grid << ", " << posx << ", " << posy << ", " << yaw << ", " << pan << ", " << tilt << ", " << c0_x << ", " << c0_y << ", " << c1_x << ", " << c1_y << ", " << c2_x << ", " << c2_y << ", " << c3_x << ", " << c3_y << ", " << c4_x << ", " << c4_y << ", " << c5_x << ", " << c5_y << ", " << c6_x << ", " << c6_y << ", " << c7_x << ", " << c7_y << ", " << c8_x << ", " << c8_y << ", " << c9_x << ", " << c9_y << ", " << c10_x << ", " << c10_y << std::endl;
        file.close();
        std::cout << grid << ", " << posx << ", " << posy << ", " << yaw << ", " << pan << ", " << tilt << ", " << c0_x << ", " << c0_y << ", " << c1_x << ", " << c1_y << ", " << c2_x << ", " << c2_y << ", " << c3_x << ", " << c3_y << ", " << c4_x << ", " << c4_y << ", " << c5_x << ", " << c5_y << ", " << c6_x << ", " << c6_y << ", " << c7_x << ", " << c7_y << ", " << c8_x << ", " << c8_y << ", " << c9_x << ", " << c9_y << ", " << c10_x << ", " << c10_y << std::endl;
    }

    bool doneGetData = false;
    int cnt_get_data = 0;
    NodeStatus GetData()
    {
        /* if (!doneGetData)
        {   if(object_count == 1){
                doneGetData = true;
                print("objek kedetek 1 ga nyimpan\n");
            }else{
                if (cnt_get_data <= 20)
                {
                    doneGetData = false;
                    printf("... data ke %d\n", cnt_get_data);
                    print("objek kedetek lebih nyimpan\n");
                    robotPos_X = convertGridX(1, 0);
                    robotPos_Y = convertGridY(1, 0);
                    saveToCSV(Grid, robotPos_X, robotPos_Y, msg_yaw, headPan, headTilt, Left_X_Cross_X, Left_X_Cross_Y, Right_X_Cross_X, Right_X_Cross_Y, Left_T_Cross_X, Left_T_Cross_Y, Right_T_Cross_X, Right_T_Cross_Y, Left_Corner_X, Left_Corner_Y, Right_Corner_X, Right_Corner_Y, Left_L_Cross_X, Left_L_Cross_Y, Right_L_Cross_X, Right_L_Cross_Y, Pinalty_X, Pinalty_Y,Left_T_Corner_X,Left_T_Corner_Y, Right_T_Corner_X,Right_T_Corner_Y);
                    cnt_get_data++;
                } else 
                {
                    print("True\n");
                    doneGetData = true;
                }
            }
        } */

        /* if (!doneGetData)
        {
            if(msg_strategy == 0){
                robotPos_X = convertGridX(53, 0);
                robotPos_Y = convertGridY(53, 0);
            }else if(msg_strategy == 1){
                robotPos_X = convertGridX(52, 0);
                robotPos_Y = convertGridY(52, 0);
            }else if(msg_strategy == 2){
                robotPos_X = convertGridX(51, 0);
                robotPos_Y = convertGridY(51, 0);
            }else if(msg_strategy == 3){
                robotPos_X = convertGridX(50, 0);
                robotPos_Y = convertGridY(50, 0);
            }
            if (cnt_sbr >= 4)
            {
                doneGetData = true;
            } else 
            {
                searchBallRectang(-1.6, -1.6, -0.8, 1.6);
                if (object_count > 1)
                {    
                    saveToCSV(Grid, robotPos_X, robotPos_Y, msg_yaw, headPan, headTilt, Left_X_Cross_X, Left_X_Cross_Y, Right_X_Cross_X, Right_X_Cross_Y, Left_T_Cross_X, Left_T_Cross_Y, Right_T_Cross_X, Right_T_Cross_Y, Left_Corner_X, Left_Corner_Y, Right_Corner_X, Right_Corner_Y, Left_L_Cross_X, Left_L_Cross_Y, Right_L_Cross_X, Right_L_Cross_Y, Pinalty_X, Pinalty_Y,Left_T_Corner_X,Left_T_Corner_Y, Right_T_Corner_X,Right_T_Corner_Y);
                }
            }
            searchBallRectang(-1.6, -1.6, -0.8, 1.6);
            printf("...cnt_sbr : %f\n", cnt_sbr); 
        } */
        support_grid = coordinates_to_grid(convertGridX(21, 0), convertGridY(21, -400));
        printf("...support grid kanan 21: %d\n", support_grid); // 19
        support_grid = coordinates_to_grid(convertGridX(23, 0), convertGridY(23, 400));
        printf("...support grid kiri 23: %d\n", support_grid); // 23
        return NodeStatus::FAILURE;
    }

    NodeStatus Shoot()
    {
        switch (stateCondition)
        {
            case 0: // search ball
                tendang = false;
                ballPos = false;
                robotDirection = false;
                if (ballLost(20))
                {
                    motion("0");
                    //tiltSearchBall(0.0);
                    headMove(0.04, -1.28);
                } else 
                {
                    trackBall();
                    if (delayWaitBall > 20)
                    {  
                        stateCondition = 1;
                    } else
                    {
                        delayWaitBall++;
                    }
                }
                
            break;

            case 1: // follow Ball
            printf("case 1 cuy");
                motion("9");
                tendang = false;
                ballPos = false;
                robotDirection = false;
                delayWaitBall = 0;
                if (ballLost(20))
                {
                    resetCase0();
                    stateCondition = 0;
                } else
                {
                    trackBall();
                    if (headTilt >= cAktif && headPan >= -0.4 && headPan <= 0.4)
                    {   
                        stateCondition = 2;
                    } else {
                        followBall(0);
                    }
                }
                
            break;

            case 2: // BallPos -> Tendang
                
                if (ballLost(20))
                {
                    resetCase0();
                    stateCondition = 0;
                } else 
                {
                    trackBall();
                    if (tendang)
                    {
                        resetCase0();
                        stateCondition = 0;
                    } else 
                    {

                        kickNoSudut(2);
                    }
                }

            break;

        default:
            break;
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus PathFinding()
    {
        if (done_generated)
        {
            return NodeStatus::SUCCESS;
        } else 
        {
            initialPos_Y = convertGridY(18, 50);
            initialPos_X = convertGridX(18, 0);
            startRow = convertRow(18);
            startCol = convertCol(18);
            endRow = convertRow(27);
            endCol = convertCol(27);
            printf("...Grid : %d\n", Grid);
            printf("...start : %d, %d\n", startRow, startCol);
            printf("...end : %d, %d\n", endRow, endCol);
            generate_path();
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus PathTracking()
    {
        initialPos_X = convertGridX(3, -50);
        initialPos_Y = convertGridY(3, 0);
        if (done_path_tracked)
        {
            motion("0");
            return NodeStatus::SUCCESS;
        } else 
        {
            path_tracking();
        }
        return NodeStatus::FAILURE;
    }

    int state_move_grid = 0, last_move = 0, cnt_move_to_grid = 0;
    double distance, theta, sagital, lateral, walkSagital, walkLateral;
    void new_out_grid(int targetRobotGrid, int targetRobotGridOffsetX, int targetRobotGridOffsetY, bool isGoToGrid)
    {
        double targetRobotPos_X = convertGridX(targetRobotGrid, targetRobotGridOffsetX);
        double targetRobotPos_Y = convertGridY(targetRobotGrid, targetRobotGridOffsetY);
        // printf("...msg_yaw = %d\n", msg_yaw);
        printf("...robotPos = %f, %f\n", robotPos_X, robotPos_Y);
        printf("...target = %f, %f\n", targetRobotPos_X, targetRobotPos_Y);
        // Calculate the distance between the two points
        distance = sqrt(pow(targetRobotPos_X - robotPos_X, 2) + pow(targetRobotPos_Y - robotPos_Y, 2));
        // Calculate the angle between the two points in degrees
        theta = atan2(targetRobotPos_Y - robotPos_Y, targetRobotPos_X - robotPos_X) * 180 / PI;
        printf("...distance = %d Cm\n", (int)distance);
        printf("...angle = %d Deg\n", (int)theta);
        int diffTheta = abs(int(theta) - msg_yaw);
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
                    // set_velocity(0.0, 0.0, 0.0);
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
                if (cnt_move_to_grid > 10)
                {
                    // printf("...jalan theta !!! \n");
                    jalanDirection(kejarMax, 0.0, theta);
                }
                else
                {
                    reset_velocity();
                    cnt_move_to_grid++;
                }
            }
        }
    }

    NodeStatus UpdateCoor()
    {

        return NodeStatus::FAILURE;
    }

    bool passed = false;
    NodeStatus isPassed()
    {
        if (passed)
        {
            cekWaktu(3);
            if (timer)
            {
                return NodeStatus::SUCCESS;
            }
        }
        else
        {
            setWaktu();
        }
        sendRobotCoordinationData(robotNumber, 1, 0, Grid, 0, 999, 88, 1, voltage);
        return NodeStatus::FAILURE;
    }

  

    NodeStatus RobotPassing()
    {
        if (passed)
        {
            sleep(0.5);
            motion("0");
            return NodeStatus::SUCCESS;
        }
        else
        {
            motion("1");
            passed = true;
        }
        return NodeStatus::FAILURE;
    }

    int cnt_rst_1000 = 0;
    NodeStatus sendPassingCoordination()
    {
        if (cnt_rst_1000 > 5)
        {
            sendRobotCoordinationData(robotNumber, 1, 0, Grid, 0, 999, 88, 1, voltage);
            return NodeStatus::SUCCESS;
        }
        else
        {
            cnt_rst_1000++;
        }
        sendRobotCoordinationData(robotNumber, 1, 1000, Grid, 0, 999, 88, 1, voltage);
        return NodeStatus::FAILURE;
    }

    NodeStatus isStepBreak()
    {
        if (sumWalkX >= 40)
        {
            motion("0");
            reset_velocity();
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus JustWalk()
    {
        set_velocity(kejarMax, 0.0, 0.0);
        return NodeStatus::FAILURE;
    }

    NodeStatus IsGoalie()
    {
        if (msg_strategy == 0)
        {
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus IniPosGoalie()
    {
        initialPos_X = -350;
        initialPos_Y = -310;
        // initialPos_X = -450;
        // initialPos_Y = -0;
        return NodeStatus::SUCCESS;
    }

    int rst_mov = 0;
    bool done_first_in_gk = false;
    NodeStatus InGoalie()
    {
        if (done_first_in_gk)
        {
            return NodeStatus::SUCCESS;
        }
        else
        {
            if (rst_mov > 5)
            {
                cekWaktu(30);
                if (timer)
                {
                    motion("0");
                    done_first_in_gk = true;
                }
                else
                {
                    new_out_grid(4, -50, -25, true);
                }
            }
            else
            {
                refreshMoveGrid();
                setWaktu();
                rst_mov++;
            }
            return NodeStatus::FAILURE;
        }
    }

    bool homing = false;
    NodeStatus isKeeperZone()
    {
        printf("...homing = %d\n", homing);
        if (robotPos_X < -350 && !homing)
        {
            refreshMoveGrid();
            state_gk = 0;
            return NodeStatus::SUCCESS;
        }
        stateCondition = 0;
        homing = true;
        return NodeStatus::FAILURE;
    }

    int reset_head_up = 0;
    NodeStatus HeadUp()
    {
        if (ballLost(20))
        {
            tiltSearchBall(0.0);
            if (reset_head_up > 5)
            {
                if (posRotateNew)
                {
                    motion("0");
                }
                else
                {
                    rotateBodyImuNew(0);
                }
            }
            else
            {
                posRotateNew = false;
                reset_head_up++;
            }
        }
        else
        {
            reset_head_up = 0;
        }

        if (reset_head_up == 0 || homing)
        {
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    int cnt_sit = 0, state_gk = 0;
    NodeStatus GoalKeeper()
    {
        printf("...state_gk : %d\n", state_gk);
        switch (state_gk)
        {
        case 0:
            if (cnt_sit > 5)
            {
                motion("7");
                state_gk = 1;
            }
            else
            {
                motion("8");
                cnt_sit++;
            }
            break;

        case 1:
            cnt_sit = 0;
            if (ballLost(20))
            {
                // tiltSearchBall(0.0);
            }
            else
            {
                trackBall();
                if (headTilt >= cSekarang)
                {
                    motion("8");
                    robotDirection = false;
                    reset_velocity();
                    state_gk = 2;
                }
            }
            break;

        case 2:
            if (ballLost(20))
            {
                // tiltSearchBall(0.0);
                reset_velocity();
                state_gk = 1;
            }
            else
            {
                trackBall();
                if (headTilt >= cAktif && headPan > -0.4 && headPan < 0.4)
                {
                    if (robotDirection && headTilt >= cSekarang && headPan > -0.4 && headPan < 0.4)
                    {
                        reset_velocity();
                        tendang = ballPos = false;
                        state_gk = 3;
                    }
                    else
                    {
                        Imu(0, cSekarang);
                    }
                }
                else
                {
                    followBall(0);
                }
            }
            break;

        case 3:
            if (ballLost(20))
            {
                // tiltSearchBall(0.0);
                state_gk = 1;
            }
            else
            {
                trackBall();
                if (tendang)
                {
                    sleep(1);
                    state_gk = 4;
                }
                else
                {
                    kick(5);
                }
            }
            break;

        case 4:
            return NodeStatus::SUCCESS;
            break;

        default:
            break;
        }
        return NodeStatus::FAILURE;
    }

    int cnt_homing = 0, init_homing = 0;
    NodeStatus Homing()
    {
        double homingX = -450;
        double homingY = 0;
        if (init_homing == 0)
        {
            homingX = robotPos_X - 100;
        }
        // printf("...error = %d, %d\n", abs(homingX - robotPos_X), abs(homingY - robotPos_Y));
        if (abs(homingX - robotPos_X) <= 15 && abs(homingY - robotPos_Y) <= 30)
        {
            motion("0");
            homing = false;
        }
        else
        {
            if (cnt_homing > 5)
            {
                init_homing = 1;
                if (posRotateNew)
                {
                    if (robotPos_X > homingX)
                    {
                        jalanDirection(-0.03, 0.0, 0);
                    }
                    else if (robotPos_X <= homingX)
                    {
                        if (robotPos_Y < homingY)
                        {
                            jalanDirection(0.0, 0.03, 0);
                        }
                        else if (robotPos_Y > homingY)
                        {
                            jalanDirection(0.0, -0.03, 0);
                        }
                    }
                }
                else
                {
                    rotateBodyImuNew(0);
                }
            }
            else
            {
                init_homing = 0;
                refreshMoveGrid();
                cnt_homing++;
            }
        }
        return NodeStatus::FAILURE;
    }

    int rst_head_grid = 0, HeadGrid = 0;
    NodeStatus Communication()
    {
        if (ballLost(20))
        {
            sendRobotCoordinationData(robotNumber, 1, stateCondition, Grid, 0, 999, 88, 1, voltage);
            stateCondition = 0;
            
        }
        else
        {
            sendRobotCoordinationData(robotNumber, 1, stateCondition, Grid, 0, semeh, 88, 1, voltage);
            rst_head_grid = 0;
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus LookToGrid()
    {
        if (robotNumber != 1 && robot1FBall == 1 && robot1State != 0)
        {
            // jika ada robot yang melihat bola
            HeadGrid = robot1GridPosition;
        } else if (robotNumber != 2 && robot2FBall == 1 && robot2State != 0)
        {
            // jika ada robot yang melihat bola
            HeadGrid = robot2GridPosition;
        } else if (robotNumber != 3 && robot3FBall == 1 && robot3State != 0)
        {
            // jika ada robot yang melihat bola
            HeadGrid = robot3GridPosition;
        } else if (robotNumber != 4 && robot4FBall == 1 && robot4State != 0)
        {
            // jika ada robot yang melihat bola
            HeadGrid = robot4GridPosition;
        } else if (robotNumber != 5 && robot5FBall == 1 && robot5State != 0)
        {
            // jika ada robot yang melihat bola
            HeadGrid = robot5GridPosition;
        } else 
        {
            HeadGrid = 88;
        }
        if (HeadGrid != 88)
        {
            new_out_grid(HeadGrid, 0, 0, false);
            tiltPredictSearchBall(theta);
            if (rst_head_grid > 5)
            {
                if (posRotateNew)
                {
                    motion("0");
                    // return NodeStatus::SUCCESS;
                } else 
                {
                    rotateBodyImuNew(theta);
                }
            } else 
            {
                posRotateNew = false;
                resetCntMoveGrid();
                rst_head_grid++;
            }
        } else 
        {
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus isExecutor()
    {
        printf("...semeh = %d\n", semeh);
        printf("...robot1dBall = %d\n...robot2dBall = %d\n...robot3dBall = %d\n...robot4dBall = %d\n...robot5dBall = %d\n", robot1DBall, robot2DBall, robot3DBall, robot4DBall, robot5DBall);
        if (useCoordination)
        {
            if (robotNumber != 1)
            {
                if  ( //jika ada robot lain yang sudah masuk case eksekusi
                    (robot1State == 1 || robot1State == 2 || robot1State == 3 || robot1State == 4 ) 
                    ) 
                {
                    return NodeStatus::FAILURE;
                }
                else if ( //jika jarak saya paling dekat dengan bola / saya dapat bola lebih dulu
                    (semeh < robot1DBall)
                )
                    {
                    resetCntMoveGrid();
                    return NodeStatus::SUCCESS;
                }
            }

            if (robotNumber != 2)
            {
                if  ( //jika ada robot lain yang sudah masuk case eksekusi
                    (robot2State == 1 || robot2State == 2 || robot2State == 3 || robot2State == 4 )
                    )
                {
                    return NodeStatus::FAILURE;
                }
                else if ( //jika jarak saya paling dekat dengan bola / saya dapat bola lebih dulu
                    (semeh < robot2DBall)
                    )
                {
                    resetCntMoveGrid();
                    return NodeStatus::SUCCESS;
                }
            }
            
            if (robotNumber != 3)
            {
                if  ( //jika ada robot lain yang sudah masuk case eksekusi
                    (robot3State == 1 || robot3State == 2 || robot3State == 3 || robot3State == 4)
                    )
                {
                    return NodeStatus::FAILURE;
                }
                else if ( //jika jarak saya paling dekat dengan bola / saya dapat bola lebih dulu
                    (semeh < robot3DBall)
                    )
                {
                    resetCntMoveGrid();
                    return NodeStatus::SUCCESS;
                }
            }
            
            if (robotNumber != 4)
            {
                if  ( //jika ada robot lain yang sudah masuk case eksekusi
                    (robot4State == 1 || robot4State == 2 || robot4State == 3 || robot4State == 4)
                    )
                {
                    return NodeStatus::FAILURE;
                }
                else if ( //jika jarak saya paling dekat dengan bola / saya dapat bola lebih dulu
                    (semeh < robot4DBall) 
                    )
                {
                    resetCntMoveGrid();
                    return NodeStatus::SUCCESS;
                }
            }
            
            if (robotNumber != 5)
            {
                if  ( //jika ada robot lain yang sudah masuk case eksekusi
                    (robot5State == 1 || robot5State == 2 || robot5State == 3 || robot5State == 4) 
                    )
                {
                    return NodeStatus::FAILURE;
                }
                else if ( //jika jarak saya paling dekat dengan bola / saya dapat bola lebih dulu
                    (semeh < robot5DBall) 
                    )
                {
                    resetCntMoveGrid();
                    return NodeStatus::SUCCESS;
                }
            }
        }
        else
        {
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus isGoalkeeper()
    {
        if(msg_strategy == 0)
        {
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    int rst_support = 0, support_grid = 0;
    NodeStatus Support()
    {
        if (ballLost(20))
        {
            threeSearchBall();
        }
        else
        {
            trackBall();
        }
        stateCondition = 0;
        // motion("0");
        // reset_velocity();
        if (rst_support > 100)
        {
            if (robotNumber == 1)
            {
                if (robot2FBall == 1)
                {
                    support_grid = coordinates_to_grid(convertGridX(robot2GridPosition, 300), convertGridY(robot2GridPosition, -200));
                } else if (robot3FBall == 1)
                {
                    support_grid = coordinates_to_grid(convertGridX(robot3GridPosition, 300), convertGridY(robot3GridPosition, 200));
                }
            } else if (robotNumber == 2)
            {
                if (robot1FBall == 1)
                {
                    support_grid = coordinates_to_grid(convertGridX(robot1GridPosition, -300), convertGridY(robot1GridPosition, 200));
                } else if (robot3FBall == 1)
                {
                    support_grid = coordinates_to_grid(convertGridX(robot3GridPosition, 0), convertGridY(robot3GridPosition, 250));
                }
            } else if (robotNumber == 3)
            {
                if (robot1FBall == 1)
                {
                    support_grid = coordinates_to_grid(convertGridX(robot1GridPosition, -300), convertGridY(robot1GridPosition, -200));
                } else if (robot2FBall == 1)
                {
                    support_grid = coordinates_to_grid(convertGridX(robot2GridPosition, 0), convertGridY(robot2GridPosition, -250));
                }
            }

            if (support_grid >= 0 && support_grid <= 53)
            {
                new_out_grid(support_grid, 0, 0, true);
            } else 
            {
                motion("0");
            }
        } else
        {
            motion("0");
            reset_velocity();
            refreshMoveGrid();
            rst_support++;
        }
        return NodeStatus::FAILURE;
    }

    NodeStatus SelfUpdate()
    {
        if (penLost(20))
        {
            motion("0");
            tiltSearchBall(0.0);
        } else 
        {
            trackPen();
            // if (headPan > -0.1 && headPan < 0.1)
            // {
                updateCoordinate();
            //     motion("0");
            // } else 
            // {
            //     newBodyTracking();
            // }
        }
        return NodeStatus::FAILURE;
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
        cnt_path = cnt_path_ = state_move_grid = stateCondition = cnt_move_to_grid = rst_mov = state_gk = cnt_sit = rst_support = cnt_sbr = 0;
        Ball_X = Ball_Y = Goal_X = Goal_Y = -1;
        done_path_tracked = done_generated = Pickup = passed = homing = done_first_in_gk = doneGetData = false;
        cnt_rst_1000 = sumWalkX = cnt_homing = 0;
        robotPos_X = robotPos_Y = deltaPos_X = deltaPos_Y = field_x = field_y = x_pos = y_pos = 0.0;
        filtered_imu_yaw = filtered_x_vel = filtered_y_vel = 0;
        object_count = rst_loc_kick_off = rst_dev_left = rst_dev_right = rst_mov_attack = 0;
    }

    void resetpickup()
    {
        bool saka1 = false;
        bool saki1 = false;
        bool saka2 = false;
        bool saki2 = false;
        bool saka3 = false;
        bool saki3 = false;
    }

    void display()
    {
    }

    void displayBT()
    {
        printf("strategy = %d, play = %d\n", msg_strategy, msg_kill);
        printf("Grid = %d\n", Grid);
        printf("robotPos = %f, %f\n", robotPos_X, robotPos_Y);
        printf("sudut = %d\n", msg_yaw);
        printf("head = %.2f, %.2f\n", headPan, headTilt);
        printf("head(offset) = %.2f, %.2f\n", headPan - 0.15, headTilt);
        printf("stateGameController = %d\n", State);
        printf("stateCondition = %d\n", stateCondition);
        printf("ball = %d, %d\n", Ball_X, Ball_Y);
        printf("actual walk = %d mm, %d mm, %f m, %f m\n", vx, vy, robotWalkX, robotWalkY);
        printf("walk active, support leg = %d, %d\n", walkActive, supportLeg);
        printf("class id = %s\n", class_id.c_str());
        printf("knee Current = %d\n", kneeCurr);
        printf("stabilize state = %d\n", stabilize_state);
        printf("second = %d\n", second);
        printf("ObjectCount = %d\n", object_count);
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
            if (ballLost(20))
            {
                tiltSearchBall(0.0);
            }
            else
            {
                trackBall();
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
            printf("...STABILIZE!!!\n");
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

        auto msg_pose = nav_msgs::msg::Odometry();
        // robotPos_X = field_x + initialPos_X;
        // robotPos_Y = field_y + initialPos_Y;
        robotPos_X = deltaPos_X + initialPos_X;
        robotPos_Y = deltaPos_Y + initialPos_Y;
        msg_pose.pose.pose.position.x = robotPos_X;
        msg_pose.pose.pose.position.y = robotPos_Y;
        msg_pose.pose.pose.position.z = 1.0;
        Odometry_->publish(msg_pose);

        tree.tickRoot();
        Ball_X = Pinalty_X = -1;
        Ball_Y = Pinalty_Y = -1;
        Left_X_Cross_X = Left_X_Cross_Y = Right_X_Cross_X = Right_X_Cross_Y = Left_T_Cross_X = Left_T_Cross_Y = Right_T_Cross_X = Right_T_Cross_Y = -1;
        Left_Corner_X = Left_Corner_Y = Right_Corner_X = Right_Corner_Y = Left_L_Cross_X = Left_L_Cross_Y = Right_L_Cross_X = Right_L_Cross_Y = -1;
        Left_T_Corner_X = Left_T_Corner_Y = Right_T_Corner_X = Right_T_Corner_Y = -1;

        x_min = -1;
        x_max = -1;
        y_min = -1;
        y_max = -1;
    }

    void resetCase0()
    {
        delayWaitBall = 0;
    }

    void resetCase2()
    {
        robotDirection = false;
    }

    void resetCase3()
    {
        tendang = ballPos = false;
    }

    void resetCase4()
    {
        delay = 0;
    }

    double coorXx, coorXy, coorYy, coorYx, sX, sY, deltaPos_X = 0, deltaPos_Y = 0;
    void mapping(double arukuX, double arukuY)
    {
        // value use minimal / 10
        if (arukuX > 0)
        {
            if (arukuX > 0 && arukuX <= 0.03)
            {
                sX = 4.35 * arukuX / 0.03; //2.1
            }
            else if (arukuX > 0.03 && arukuX <= 0.04)
            {
                sX = 5.533 * arukuX / 0.04; //4.2
            }
            else if (arukuX > 0.04 && arukuX <= 0.05)
            {
                sX = 6.23833 * arukuX / 0.05; //5.2
            }
            else if (arukuX > 0.05 && arukuX <= 0.06)
            {
                sX = 8.05666 * arukuX / 0.06; //5.7
            }
            else if (arukuX > 0.06 && arukuX <= 0.07)
            {
                sX = 8.0516666 * arukuX / 0.07; //6.9
            }
            else if (arukuX > 0.07 && arukuX <= 0.08)
            {
                sX = 8.058666 * arukuX / 0.08; //7.5
            }
        }
        else if (arukuX < 0)
        {
            if (arukuX == -0.01)
            {
                sX = -0.2;
            }
            else if (arukuX == -0.02)
            {
                sX = -0.2;
            }
            else if (arukuX == -0.03)
            {
                sX = -0.2;
            }
            else
            {
                sX = -0.2 * arukuX / -0.03;
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
                sY = 1.4 * arukuY / 0.01; //2.5
            }
            else if (arukuY > 0.01 && arukuY <= 0.02)
            {
                sY = 1.9 * arukuY / 0.02; //4
            }
            else if (arukuY > 0.02 && arukuY <= 0.03)
            {
                sY = 4.65 * arukuY / 0.03; //5	//6.54  //-2.1
            }
            else
            {
                sY = 4.65 * arukuY / 0.03; //4.85 //-2.1
            }
        }
        else if (arukuY < 0)
        {

            if (arukuY < 0 && arukuY >= -0.01)
            {
                sY = -1.4 * arukuY / -0.01; //2.5
            }
            else if (arukuY < -0.01 && arukuY >= -0.02)
            {
                sY = -1.94 * arukuY / -0.02; //-4
            }
            else if (arukuY < -0.02 && arukuY >= -0.03)
            {
                sY = -5.61 * arukuY / -0.03; //-5.11 //-2.1
            }
            else
            {
                sY = -5.61 * arukuY / -0.03; //-5.11 //-2.1
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

        double filtered_value = low_pass_filter(prev_filtered_value, degrees_to_radians(msg_yaw), 0.04, CUTOFF_FREQUENCY);
        prev_filtered_value = filtered_value;
        // printf("... msg_yaw = %d, filtered_yaw = %.2f\n", msg_yaw, radians_to_degrees(filtered_value));
        msg_yaw = (int)radians_to_degrees(filtered_value);

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
        motion("9");
        char line[50];

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
        if (abs(vy) > 0.02 || abs(va) > 0.3)
        {
            if (vWalkX > kejarMid)
            {
                vWalkX = kejarMid; // reduce speed for stability
            }
        }
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
        // if (vWalkX == 0.0)
        // {
        //     velocityX = 0;
        // }
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
        this->declare_parameter("pPanTendang", -0.20);
        this->declare_parameter("pTiltTendang", -0.57);
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
    }

    std::string tree_path = "";
    void getParameters()
    {
        ball_panKP = this->get_parameter("ball_panKP").as_double();
        ball_panKD = this->get_parameter("ball_panKD").as_double();
        ball_tiltKP = this->get_parameter("ball_tiltKP").as_double();
        ball_tiltKD = this->get_parameter("ball_tiltKD").as_double();
        frame_X = this->get_parameter("frame_X").as_int();
        frame_Y = this->get_parameter("frame_Y").as_int();
        robotNumber = this->get_parameter("robotNumber").as_int();
        pPanTendang = this->get_parameter("pPanTendang").as_double();
        pTiltTendang = this->get_parameter("pTiltTendang").as_double();
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
    }

    void sendRobotCoordinationData(signed short rNumber, signed short rStatus, signed short sNumber, signed short gridPos, signed short fBall, signed short dBall, signed short gridBall, signed short backIn, signed short voltage)
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
        msg.voltage = voltage;
        robotCoordination_->publish(msg);
    }

    void readRobotCoordinationData1(const bfc_msgs::msg::Coordination::SharedPtr message)
    {
        resetCommunication();
        robot1Id = message->robot_number;
        robot1Status = message->status;
        robot1State = message->state;
        robot1GridPosition = message->grid_position;
        robot1FBall = message->found_ball;
        robot1DBall = message->distance_ball;
        robot1GridBall = message->grid_ball;
        robot1BackIn = message->back_in;
    }

    void readRobotCoordinationData2(const bfc_msgs::msg::Coordination::SharedPtr message)
    {
        resetCommunication();
        robot2Id = message->robot_number;
        robot2Status = message->status;
        robot2State = message->state;
        robot2GridPosition = message->grid_position;
        robot2FBall = message->found_ball;
        robot2DBall = message->distance_ball;
        robot2GridBall = message->grid_ball;
        robot2BackIn = message->back_in;
    }

    void readRobotCoordinationData3(const bfc_msgs::msg::Coordination::SharedPtr message)
    {
        resetCommunication();
        robot3Id = message->robot_number;
        robot3Status = message->status;
        robot3State = message->state;
        robot3GridPosition = message->grid_position;
        robot3FBall = message->found_ball;
        robot3DBall = message->distance_ball;
        robot3GridBall = message->grid_ball;
        robot3BackIn = message->back_in;
    }

    void readRobotCoordinationData4(const bfc_msgs::msg::Coordination::SharedPtr message)
    {
        resetCommunication();
        robot4Id = message->robot_number;
        robot4Status = message->status;
        robot4State = message->state;
        robot4GridPosition = message->grid_position;
        robot4FBall = message->found_ball;
        robot4DBall = message->distance_ball;
        robot4GridBall = message->grid_ball;
        robot4BackIn = message->back_in;
    }

    void readRobotCoordinationData5(const bfc_msgs::msg::Coordination::SharedPtr message)
    {
        resetCommunication();
        robot5Id = message->robot_number;
        robot5Status = message->status;
        robot5State = message->state;
        robot5GridPosition = message->grid_position;
        robot5FBall = message->found_ball;
        robot5DBall = message->distance_ball;
        robot5GridBall = message->grid_ball;
        robot5BackIn = message->back_in;
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
                mapping(robotWalkX, robotWalkY);
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
    }

    int object_count = 0;

    void callbackFoundObject(const darknet_ros_msgs::msg::ObjectCount::SharedPtr msg)
    {
        object_count = msg->count;
        // printf("cnt = %d\n", object_count);
    }
    int x_min, x_max, y_min, y_max;
    std::string class_id;
    void callbackBoundingBox(const darknet_ros_msgs::msg::BoundingBoxes::SharedPtr msg)
    {
        /* // RCLCPP_INFO(this->get_logger(), "Bouding Boxes (Class): '%s'", msg->bounding_boxes[0].class_id.c_str());
        class_id = msg->bounding_boxes[0].class_id.c_str();
        // RCLCPP_INFO(this->get_logger(), "Bouding Boxes (xmin): '%d'", msg->bounding_boxes[0].xmin);
        x_min = msg->bounding_boxes[0].xmin;
        // RCLCPP_INFO(this->get_logger(), "Bouding Boxes (xmax): '%d'", msg->bounding_boxes[0].xmax);
        x_max = msg->bounding_boxes[0].xmax;
        // RCLCPP_INFO(this->get_logger(), "Bouding Boxes (ymin): '%d'", msg->bounding_boxes[0].ymin);
        y_min = msg->bounding_boxes[0].ymin;
        // RCLCPP_INFO(this->get_logger(), "Bouding Boxes (ymax): '%d'", msg->bounding_boxes[0].ymax);
        y_max = msg->bounding_boxes[0].ymax; */

        for (const auto& bbox : msg->bounding_boxes)
        {
            // RCLCPP_INFO(this->get_logger(), "Class: %s", bbox.class_id.c_str());
            const auto& id_class = bbox.id;
            const auto x_center = (bbox.xmin + bbox.xmax) / 2;
            const auto y_center = (bbox.ymin + bbox.ymax) / 2;

            switch (id_class)
            {
                case 0:
                    // RCLCPP_INFO(this->get_logger(), "Left_X_Cross: (%d, %d)", x_center, y_center);
                    Left_X_Cross_X = x_center;
                    Left_X_Cross_Y = y_center;
                    continue;
                case 1:
                    // RCLCPP_INFO(this->get_logger(), "Right_X_Cross: (%d, %d)", x_center, y_center);
                    Right_X_Cross_X = x_center;
                    Right_X_Cross_Y = y_center;
                    continue;
                case 2:
                    // RCLCPP_INFO(this->get_logger(), "Left_T_Cross: (%d, %d)", x_center, y_center);
                    Left_T_Cross_X = x_center;
                    Left_T_Cross_Y = y_center;
                    continue;
                case 3:
                    // RCLCPP_INFO(this->get_logger(), "Right_T_Cross: (%d, %d)", x_center, y_center);
                    Right_T_Cross_X = x_center;
                    Right_T_Cross_Y = y_center;
                    continue;
                case 4:
                    // RCLCPP_INFO(this->get_logger(), "Left_Corner: (%d, %d)", x_center, y_center);
                    Left_Corner_X = x_center;
                    Left_Corner_Y = y_center;
                    continue;
                case 5:
                    // RCLCPP_INFO(this->get_logger(), "Right_Corner: (%d, %d)", x_center, y_center);
                    Right_Corner_X = x_center;
                    Right_Corner_Y = y_center;
                    continue;
                case 6:
                    // RCLCPP_INFO(this->get_logger(), "Left_L_Cross: (%d, %d)", x_center, y_center);
                    Left_L_Cross_X = x_center;
                    Left_L_Cross_Y = y_center;
                    continue;
                case 7:
                    // RCLCPP_INFO(this->get_logger(), "Right_L_Cross: (%d, %d)", x_center, y_center);
                    Right_L_Cross_X = x_center;
                    Right_L_Cross_Y = y_center;
                    continue;
                case 8:
                    // RCLCPP_INFO(this->get_logger(), "Pinalty: (%d, %d)", x_center, y_center);
                    Pinalty_X = x_center;
                    Pinalty_Y = y_center;
                    continue;
                case 9:
                    // RCLCPP_INFO(this->get_logger(), "ball: (%d, %d)", x_center, y_center);
                    Ball_X = x_center;
                    Ball_Y = y_center;
                    continue;
                case 10:
                    // RCLCPP_INFO(this->get_logger(), "Left_T_Corner: (%d, %d)", x_center, y_center);
                    Left_T_Corner_X = x_center;
                    Left_T_Corner_Y = y_center;
                    continue;
                case 11:
                    // RCLCPP_INFO(this->get_logger(), "Right_T_Corner: (%d, %d)", x_center, y_center);
                    Right_T_Corner_X = x_center;
                    Right_T_Corner_Y = y_center;
                    continue;

                // add additional cases for each class
                default:
                    break; // ignore any other classes
            }
        }
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

    vector<int> grid_list;
    void callbackPathFinding(std_msgs::msg::Int16MultiArray::SharedPtr msg)
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
    double deltaY, deltaX;
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
        if (deltaY >= 0.5)
        { // 0.5 //7.0 // 30
            // printf("  deltaY = %.f,", deltaY);
            // printf("  Bola Menjauh\n");
            kondisiBola = 1;
            //} else if((deltaY <= -20 && deltaX <= -20) || (deltaY <= -20 && deltaX >= 20)) { //-2 //-1.4
        }
        else if (deltaY <= -0.5)
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

    int countReadyKick;
    double SetPointPan = 0, SetPointTilt = -0.5, errorfPan, errorfTilt, PyMove = 0, PxMove = 0, PaMove = 0;

    void followBall(int mode) 
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
        { // Stop(bola sudah dekat)
            countReadyKick++;
        } 
        else 
        { // Kejar Bola(bola masih jauh)
            countReadyKick = 0;
        }

        if (countReadyKick >= 1) 
        { 
            PxMove = 0.0;
            PyMove = errorfPan * 0.040;
            PaMove = errorfPan * 0.20;
        } 
        else 
        {
            PxMove = kejarMax;
            PyMove = errorfPan * 0.40;
            PaMove = errorfPan * 0.30;
        }

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

        // Use the smoothed velocities for robot movement
        if (mode == 0) 
        {
            if (errorfPan > -0.2 && errorfPan < 0.2) 
            {
                set_velocity(smoothedPxMove, 0.0, smoothedPaMove);
            } 
            else 
            {
                set_velocity(0.0, 0.0, smoothedPaMove);
            }
        } 
        else if (mode == 1) 
        {
            if (errorfPan > -0.4 && errorfPan < 0.4) 
            {
                set_velocity(smoothedPxMove, smoothedPyMove, smoothedPaMove);
            } 
            else 
            {
                set_velocity(0.0, 0.0, smoothedPaMove);
            }
        }
    }

    void newBodyTracking()
    {
        //trackBall();
        if (posPan < 0.05 && posPan > -0.05)
        { // Stop(bola sudah dekat)
            countReadyKick++;
        }
        else
        { // Kejar Bola(bola masih jauh)
            countReadyKick = 0;
        }

        if (countReadyKick >= 1)
        {                              // 5
            // PxMove = 0.0;              // jalan ditempat
            // PyMove = errorfPan * 0.05; // 0.045
            // PaMove = errorfPan * 0.20; // 0.30; //0.045
            motion("0");
        }
        else
        {
            errorfPan = posPan - SetPointPan;
            PyMove = errorfPan * 0.20; // 0.125; //0.045
            PaMove = errorfPan * 0.30; // 0.25; //0.35; //0.045
        }

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

        /* if (errorfPan > -0.2 && errorfPan < 0.2)
        { // printf("AAAAAAAA\n");
            set_velocity(0.0, 0.0, smoothedPaMove);
        }
        else
        { // printf("BBBBBBBB\n");
            set_velocity(0.0, smoothedPyMove, 0.0);
        } */
        set_velocity(0.0, 0.0, smoothedPaMove);
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
            Walk(0.0, 0.0, 0.0);
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
                            // motion("6");
                            headMove(-1.6, -1.6);
                            doneBanting = true;
                        }
                        else if (Ball_X < lastKoorX - 35)
                        {
                            // motion("5");
                            headMove(1.6, -1.6);
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

        bodyXImu = errorCPosTilt * (-0.03);
        // IC(bodyXImu);                               // nilai pengali ini harus tetap bernilai negatif //besarnya kalkulasi maju/mundur yg dibutuhkan tehadap posTilt
        bodyYImu = abs(errorCPosPan / 100) + 0.015; // 0.017;
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
                // Walk(bodyXImu, bodyYImu, alfaImu);
                set_velocity(bodyXImu, bodyYImu, alfaImu);
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
                // Walk(bodyXImu, bodyYImu, alfaImu);
                set_velocity(bodyXImu, bodyYImu, alfaImu);
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

        // Walk(Xwalk, Ywalk, PaMove);
        set_velocity(Xwalk, Ywalk, PaMove);
    }

    // Ball Positioning Using P Controller =======================================================
    /* double errorPosX,
        errorPosY,
        PxMoveBallPos,
        PyMoveBallPos,
        PaMoveBallPos;
    bool ballPos = false;
    void ballPositioning(double setPointX, double setPointY, double speed)
    {

        errorPosX = headPan - setPointX;
        errorPosY = headTilt - setPointY;

        // IC(errorPosX, errorPosY);

        if ((errorPosX > -0.08 && errorPosX < 0.08) && (errorPosY > -0.2))
        { //&& errorPosY < 0.10)) { //sudah sesuai
            PyMoveBallPos = 0.00;
            PxMoveBallPos = 0.00;
            ballPos = true;
        }
        else
        { // belum sesuai
            ballPos = false;
            if ((headPan >= 1.0 && headTilt >= -1.2) || (headPan <= -1.0 && headTilt >= -1.2))
            { // bola disamping //pan tilt kircok (polar)
                PxMoveBallPos = -0.03;
                PyMoveBallPos = errorPosX * 0.1; // 0.12;
            }
            else
            {
                // Xmove
                if (headTilt > setPointY)
                { //> (setPointY + 0.1)) { //kelebihan
                    PxMoveBallPos = -0.03;
                }
                else if (headTilt >= (setPointY - 0.1) && headTilt <= setPointY)
                { //<= (setPointY + 0.1)) { //sudah dalam range
                    PxMoveBallPos = 0.00;
                }
                else if (headTilt >= (setPointY - 0.3) && headTilt < (setPointY))
                { // bola sudah dekat
                    PxMoveBallPos = errorPosY * -speed;
                    if (PxMoveBallPos >= 0.02)
                    {
                        PxMoveBallPos = 0.02;
                    }
                    else if (PxMoveBallPos <= 0.00)
                    {
                        PxMoveBallPos = 0.00;
                    }
                }
                else
                {                                                 // bola masih jauh
                    PxMoveBallPos = headTilt * (kejarMax / -1.6); // 0.05
                }

                // Ymove
                if (headTilt >= (setPointY))
                { //> (setPointY + 0.1)) { //kelebihan
                    PyMoveBallPos = 0.00;
                }
                else
                {
                    if (headPan >= (setPointX - 0.1) && headPan <= (setPointX + 0.1))
                    { // sudah dalam range
                        PyMoveBallPos = 0.00;
                    }
                    else
                    {                                     // belum dalam range
                        PyMoveBallPos = errorPosX * 0.12; // 0.08;//0.12;
                    }
                }
            }
        }
        // Walk(PxMoveBallPos, PyMoveBallPos, 0.0);
        set_velocity(PxMoveBallPos, PyMoveBallPos, 0.0);
    } */

    // Ball Positioning Using P Controller =======================================================
    double errorPosX,
        errorPosY,
        PxMoveBallPos,
        PyMoveBallPos,
        PaMoveBallPos;
    bool ballPos = false;

    void ballPositioning(double setPointX, double setPointY, double speed, double desiredYaw) {

        errorPosX = headPan - setPointX;
        errorPosY = headTilt - setPointY;
        double errorYaw = msg_yaw - desiredYaw;

        if ((errorPosX > -0.1 && errorPosX < 0.1) && (errorPosY > -0.1)) { //&& errorPosY < 0.10)) { //sudah sesuai
            PyMoveBallPos = 0.00;
            PxMoveBallPos = 0.00;
            ballPos = true;
        }
        else { // belum sesuai
            ballPos = false;
            if ((headPan >= 1.0 && headTilt >= -1.2) || (headPan <= -1.0 && headTilt >= -1.2)) { // bola disamping //pan tilt kircok (polar)
                PxMoveBallPos = -0.03;
                PyMoveBallPos = errorPosX * 0.1; // 0.12;
            }
            else {
                // Xmove
                if (headTilt > setPointY) { //> (setPointY + 0.1)) { //kelebihan
                    PxMoveBallPos = -0.03;
                }
                else if (headTilt >= (setPointY - 0.1) && headTilt <= setPointY) { //<= (setPointY + 0.1)) { //sudah dalam range
                    PxMoveBallPos = 0.00;
                }
                else if (headTilt >= (setPointY - 0.3) && headTilt < (setPointY)) { // bola sudah dekat
                    PxMoveBallPos = errorPosY * -speed;
                    if (PxMoveBallPos >= 0.03) {
                        PxMoveBallPos = 0.03;
                    }
                    else if (PxMoveBallPos <= 0.00) {
                        PxMoveBallPos = 0.00;
                    }
                }
                else { // bola masih jauh
                    PxMoveBallPos = headTilt * (kejarMax / -1.6); // 0.05
                }

                // Ymove
                if (headTilt >= (setPointY)) { //> (setPointY + 0.1)) { //kelebihan
                    PyMoveBallPos = 0.00;
                }
                else {
                    if (headPan >= (setPointX - 0.1) && headPan <= (setPointX + 0.1)) { // sudah dalam range
                        PyMoveBallPos = 0.00;
                    }
                    else { // belum dalam range
                        PyMoveBallPos = errorPosX * 0.12; // 0.08;//0.12;
                    }
                }
            }

            // Rotate to desired yaw angle
            if (abs(errorYaw) > 0.05) {
                PaMoveBallPos = errorYaw * 0.01;
            }
            else {
                PaMoveBallPos = 0;
            }
        }
        // Define smoothing factor
        double alpha = 0.2;

        // Initialize smoothed velocities
        double smoothedPxMove = PxMoveBallPos;
        double smoothedPyMove = PyMoveBallPos;
        double smoothedPaMove = PaMoveBallPos;

        // Calculate smoothed velocities using EMA algorithm
        smoothedPxMove = alpha * PxMoveBallPos + (1 - alpha) * smoothedPxMove;
        smoothedPyMove = alpha * PyMoveBallPos + (1 - alpha) * smoothedPyMove;
        smoothedPaMove = alpha * PaMoveBallPos + (1 - alpha) * smoothedPaMove;
        // set_velocity(PxMoveBallPos, PyMoveBallPos, 0.0);
        // set_velocity(smoothedPxMove, smoothedPyMove, smoothedPaMove);
        set_velocity(smoothedPxMove, smoothedPyMove, 0.0);
    }

    // Dribble Ball ======================================================================
    int bawaBola;
    double setPointFootY, setPointFootY1, setPointFootY2;
    void dribble(int gawang, double speed)
    {
        trackBall();

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
            PxMoveBallPos = 0.3 * speed;
            //} else {
            //	PxMoveBallPos = errorPosY*speed*-1;
            //}
            Walk(PxMoveBallPos, 0.0, 0.0);
        }
        else
        { // x < -0.2 || x > 0.2
            if (headTilt >= pTiltTendang)
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

            Walk(PxMoveBallPos, PyMoveBallPos, 0.0);

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

        if (posTilt < -2.0 && posPan < 0.4 && posPan > -0.4 && Xcross_LX != -1 && Xcross_LY != -1)
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
        if (mode == 3 || mode == 4)
        {
            if (mode == 3)
            {
                kanan = true;
                kiri = false;
            } // arah kanan
            else if (mode == 4)
            {
                kiri = true;
                kanan = false;
            } // arah kiri
        }
        else
        {
            // if (posPan >= 0 && kanan == false && kiri == false)
            if (robotPos_Y < 0)
            { // kiri
                kiri = true;
                kanan = false;
            }
            // else if (posPan <= 0 && kanan == false && kiri == false)
            else
            { // kanan
                kanan = true;
                kiri = false;
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
                }
                else if (mode == 3 || mode == 4)
                {
                    motion("0");
                    sleep(1); // 10
                    motion("4");
                }
                else if (mode == 5 || mode == 6)
                {
                    // sleep(1);
                    motion("5");
                }
                tendang = true;
            }
            else
            {
                ballPositioning(-(pPanTendang - 0.05), pTiltTendang, ballPositioningSpeed, sudutTendang); // 0.15
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
                }
                else if (mode == 3 || mode == 4)
                {
                    motion("0");
                    sleep(1); // 10
                    motion("3");
                }
                else if (mode == 5 || mode == 6)
                {
                    // sleep(1); // 7
                    motion("6");
                }
                tendang = true;
            }
            else
            {
                ballPositioning(pPanTendang + 0.02, pTiltTendang, ballPositioningSpeed, sudutTendang); // 0.15
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
					ballPositioning(pPanTendang -0.05, pTiltTendang, ballPositioningSpeed, sudutTendang); //0.15
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
                        ballPositioning(-(pPanOper - 20), pTiltOper, ballPositioningSpeed, sudut); // 0.15
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
                        ballPositioning(pPanOper, pTiltOper, ballPositioningSpeed, sudut); // 0.15
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
                            ballPositioning(pPanOper, pTiltOper, ballPositioningSpeed, sudutTendang); // 0.15
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
                            ballPositioning(-pPanOper, pTiltOper, ballPositioningSpeed, sudutTendang); // 0.15
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
        if ((int)robotPos_X >= (convertGridX(valueGrid, valueOffSetX) - 15) && (int)robotPos_X < (convertGridX(valueGrid, valueOffSetX) + 15) &&
            (int)robotPos_Y >= (convertGridY(valueGrid, valueOffSetY) - 15) && (int)robotPos_Y < (convertGridY(valueGrid, valueOffSetY) + 15))
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
        ballPos = false;
        countMoveGrid1 =
            countMoveGrid2 =
                countMoveGrid3 = 
                    cnt_move_to_grid = 0;
        
    }

    void resetCntMoveGrid()
    {
        rst_release = rst_dev_left = rst_dev_right = rst_loc_kick_off = rst_mov = rst_mov_attack = rst_support = 0;
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

    int rst_nsb = 0, rst_nsb1 = 0;
    bool nsb_1 = false;
    void normalSearchBall() // belum siap
    {
        if (rst_nsb > 5)
        {
            if (nsb_1)
            {
                if (rst_nsb1 > 5)
                {
                    if (cnt_sbr >= 2)
                    {
                    } 
                    searchBallRectang(-1.6, -1.6, -0.8, 1.6);
                } else 
                {
                    motion("0");
                    reset_velocity();
                    cnt_sbr = 0;
                    sumWalkX = 0;
                    posRotateNew = false;
                }
            } else 
            {
                if (cnt_pan_search_ball >= 2)
                {
                    reset_velocity();
                    motion("0");
                    nsb_1 = true;
                } else 
                {
                    panSearchBall(-0.8);
                    set_velocity(-0.03, 0.0, 0.0);
                }
            }
        } else 
        {
            reset_velocity();
            saveSudutImu();
            refreshMoveGrid();
            posRotateNew = false;
            cnt_pan_search_ball = 0;
            rst_nsb++;
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
    void path_tracking()
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
                new_out_grid(next_grid, 0, 0, true);
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
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr path_finding_subscription_;
    rclcpp::Subscription<darknet_ros_msgs::msg::ObjectCount>::SharedPtr subscriber_object_count;
    rclcpp::Publisher<bfc_msgs::msg::Coordination>::SharedPtr robotCoordination_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_mot_;
    rclcpp::Publisher<bfc_msgs::msg::HeadMovement>::SharedPtr cmd_head_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr Odometry_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Update_coor_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr request_pub;
    BehaviorTreeFactory factory;
    Tree tree;
    PublisherZMQ *pubZ;

    int odom_pose_x, odom_pose_y, odom_pose_z, firstStateLocalization, stateLocalization;
    int msg_strategy, msg_kill, msg_roll, msg_pitch, msg_yaw;

    int robot1Id, robot1Status, robot1State, robot1GridPosition, robot1FBall, robot1DBall, robot1GridBall, robot1BackIn, robot1Voltage;
    int robot2Id, robot2Status, robot2State, robot2GridPosition, robot2FBall, robot2DBall, robot2GridBall, robot2BackIn, robot2Voltage;
    int robot3Id, robot3Status, robot3State, robot3GridPosition, robot3FBall, robot3DBall, robot3GridBall, robot3BackIn, robot3Voltage;
    int robot4Id, robot4Status, robot4State, robot4GridPosition, robot4FBall, robot4DBall, robot4GridBall, robot4BackIn, robot4Voltage;
    int robot5Id, robot5Status, robot5State, robot5GridPosition, robot5FBall, robot5DBall, robot5GridBall, robot5BackIn, robot5Voltage;

    int Ball_X, Ball_Y, Ball_W, Ball_H, Ball_D, Pinalty_X, Pinalty_Y;
    int Goal_X, Goal_Y, Goal_W, Goal_H, Goal_LH, Goal_RH, Goal_C, Goal_LD, Goal_RD;
    int Goal_LX, Goal_LY, Goal_RX, Goal_RY, Xcross_RX, Xcross_RY, Xcross_LX, Xcross_LY;
    int Pinalty_D, Lcross_LD, Lcross_RD, Xcross_LD, Xcross_RD, Tcross_LD, Tcross_RD;
    int Left_X_Cross_X, Left_X_Cross_Y, Right_X_Cross_X, Right_X_Cross_Y, Left_T_Cross_X, Left_T_Cross_Y, Right_T_Cross_X, Right_T_Cross_Y;
    int Left_Corner_X, Left_Corner_Y, Right_Corner_X, Right_Corner_Y, Left_L_Cross_X, Left_L_Cross_Y, Right_L_Cross_X, Right_L_Cross_Y;
    int Left_T_Corner_X, Left_T_Corner_Y, Right_T_Corner_X, Right_T_Corner_Y;

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
        pTiltTendang,
        pPanTendang,
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
        useWalkKick;
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
