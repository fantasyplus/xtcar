void BehaviourStateMachine::threadSendLastTraj()
{

    while (true)
    {
        //收到1代表小车和大车位姿细调完成，进入发送跟随轨迹阶段
        while (true)
        {

            // std::cout << std::this_thread::get_id() << std::endl;
            ROS_INFO("waiting car pose adjust completed......need task_end==1 , now task_end==%d", (int)task_status.task_end);

            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            if ((int)task_status.task_end != 1 && !task_status_flag)
            {
                continue;
            }
            else
            {
                task_status_flag = true;
                break;
            }
        }

        mpc_msgs::Lane send_lane;
        readTrajFile(send_lane);
        if (send_lane.waypoints.empty())
        {
            ROS_ERROR("can't open traj_file in threadSendLastTraj()");
            continue;
        }

        ROS_INFO("---------------send last follow lane------------------ , size:%d", (int)send_lane.waypoints.size());

        //发送最后一段固定轨迹
        _mpc_lane = send_lane;

        while (true)
        {

            // std::cout << std::this_thread::get_id() << std::endl;
            ROS_INFO("waiting two car follow lane completed......need task_end==2 , now task_end==%d", (int)task_status.task_end);

            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            //收到2代表两车跟随轨迹完成，进入最终的结束阶段并结束该线程
            if ((int)task_status.task_end == 2)
            {
                ROS_INFO("---------------enter last traj end point---------------");
                //清空_mpc_lane
                _mpc_lane.waypoints.clear();

                //清空可视化的东西
                int cnt = 10;
                while (cnt--)
                {
                    nav_msgs::Path empty_path;
                    empty_path.header.frame_id = "map";
                    _pub_vis_car_path.publish(empty_path);
                    _pub_vis_mpc_lane.publish(empty_path);
                }

                task_status_flag = false;

                break;
            }
        }
    }
}