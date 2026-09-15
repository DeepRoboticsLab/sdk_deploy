/**
 * @file state_machine_base.h
 * @brief state machine base class for different robot state machine control
 * @author DeepRobotics
 * @version 1.0
 * @date 2025-11-07
 * 
 * @copyright Copyright (c) 2025  DeepRobotics
 * 
 */

#pragma once

#include "state_base.h"
#include "safe_controller.hpp"
#include "ros2_command_interface.hpp"
#include "drdds/msg/std_msg_int32.hpp"
#include "topic_trace.hpp"

class TimeTool {
private:
    int tfd;    /**< Timer descriptor.*/
    int efd;    /**< Epoll descriptor.*/
    int fds, ret; /**< Variables used to initialize the timer.*/
    uint64_t value; /**< Variables used to initialize the timer.*/
    struct epoll_event ev, *evptr; /**< Variables used to initialize the timer.*/
    struct itimerspec time_intv;  /**< Variables used to initialize the timer.*/
public:
    timespec system_time; /**< A class for accurately obtaining time.*/
    /**
    * @brief Initialize timer, input cycle(ms).
    * @param Cycle time unit: ms
    */
    void time_init(int ms) {
        tfd = timerfd_create(CLOCK_MONOTONIC, 0);   //创建定时器
        if (tfd == -1) {
            printf("create timer fd fail \r\n");
        }
        time_intv.it_value.tv_sec = 0; //设定2s超时
        time_intv.it_value.tv_nsec = 1000 * 1000 * ms;
        time_intv.it_interval.tv_sec = time_intv.it_value.tv_sec;   //每隔2s超时
        time_intv.it_interval.tv_nsec = time_intv.it_value.tv_nsec;

        printf("timer start ...\n");
        timerfd_settime(tfd, 0, &time_intv, NULL);  //启动定时器

        efd = epoll_create1(0); //创建epoll实例
        if (efd == -1) {
            printf("create epoll fail \r\n");
            close(tfd);
        }

        evptr = (struct epoll_event *) calloc(1, sizeof(struct epoll_event));
        if (evptr == NULL) {
            printf("epoll event calloc fail \r\n");
            close(tfd);
            close(efd);
        }

        ev.data.fd = tfd;
        ev.events = EPOLLIN;    //监听定时器读事件，当定时器超时时，定时器描述符可读。
        epoll_ctl(efd, EPOLL_CTL_ADD, tfd, &ev); //添加到epoll监听队列中
    }

    /**
    * @brief Acquire interrupt signal
    * @return 1:Enter interrupt 0:no
    */
    int time_interrupt() {
        fds = epoll_wait(efd, evptr, 1, 10);
        if (evptr[0].events & EPOLLIN) {
            ret = read(evptr->data.fd, &value, sizeof(uint64_t));
            if (ret == -1) {
                printf("read return 1 -1, errno :%d \r\n", errno);
            }
            return 1;
        }
        return 0;
    } /**< Acquire interrupt signal.*/

    /**
    * @brief How long has it been
    * @param Initial time
    */
    double getCurrentTime(double start_time) {
        clock_gettime(1, &system_time);
        return system_time.tv_sec + system_time.tv_nsec / 1e9 - start_time;
    }

    /**
    * @brief Get current time
    */
    double get_start_time() {
        clock_gettime(1, &system_time);
        return system_time.tv_sec + system_time.tv_nsec / 1e9;
    } /**< Get start time.*/
};


class StateMachineBase {
private:
    /* data */
public:
    StateMachineBase(const RobotType &robot_type) : robot_type_(robot_type) {

    }

    virtual ~StateMachineBase() {}

    virtual void Start() = 0;

    virtual void Run() {
        run_thread_ = std::thread(&StateMachineBase::RunThread, this);
        rclcpp::spin(ri_ptr_->get_node());

        if (run_thread_.joinable()) 
            run_thread_.join();
    }

    virtual void RunThread() {
        set_timer.time_init(5);
        startTime = set_timer.get_start_time();

        while (rclcpp::ok()) {
            if (set_timer.time_interrupt()) {
                ri_ptr_->RefreshRobotData();

                ros_control_->UpdateCommand(current_state_name_);
                current_controller_->Run();

                if (emergency_stop_requested_.exchange(false)) {
                    {
                        char buf[128];
                        snprintf(buf, sizeof(buf),
                            "Executing: current_state='%s' -> kJointDamping (run_cnt=%d)",
                            current_controller_->state_name_.c_str(), run_cnt_);
                        topic_trace::LogEstopEvent(ri_ptr_->get_node()->get_logger(), "[ESTOP]", buf);
                    }
                    uc_ptr_->SetTargetMode(uint8_t(RobotMotionState::JointDamping));
                    next_state_name_ = StateName::kJointDamping;
                } else if (current_controller_->LoseControlJudge()) {
                    next_state_name_ = StateName::kJointDamping;
                } else {
                    next_state_name_ = current_controller_->GetNextStateName();
                }

                if (next_state_name_ != current_state_name_) {
                    current_controller_->OnExit();
                    std::cout << current_controller_->state_name_ << " ------------> ";
                    current_controller_ = GetStateControllerPtr(next_state_name_);
                    std::cout << current_controller_->state_name_ << std::endl;
                    current_controller_->OnEnter();
                    current_state_name_ = next_state_name_;
                }
                ros_control_->Publish(current_state_name_);
                ++run_cnt_;
            }
        }
    }

    virtual void Stop() = 0;

    virtual std::shared_ptr<StateBase> GetStateControllerPtr(StateName state_name) = 0;
    
    const RobotType robot_type_;

    int run_cnt_ = 0;
    double _dt = 0.001, startTime;
    TimeTool set_timer{};

    std::shared_ptr<UserCommandInterface> uc_ptr_;
    std::shared_ptr<RobotInterface> ri_ptr_;
    std::shared_ptr<ControlParameters> cp_ptr_;
    std::shared_ptr<SafeController> sc_ptr_;

    std::shared_ptr<StateBase> current_controller_;
    StateName current_state_name_, next_state_name_;

    std::thread run_thread_;
    std::shared_ptr<sdk_control::Ros2ControlStatus> ros_control_;

    // 急停信号：由 lite3_transfer 进程发布 /EMERGENCY_STOP_SIGNAL 触发
    std::atomic<bool> emergency_stop_requested_{false};
    rclcpp::Subscription<drdds::msg::StdMsgInt32>::SharedPtr estop_signal_sub_;

    // 在 ri_ptr_ 初始化后调用，订阅跨进程急停信号
    void InitEmergencyStopListener() {
        {
            char buf[128];
            snprintf(buf, sizeof(buf),
                "Subscribing to /EMERGENCY_STOP_SIGNAL on node '%s'",
                ri_ptr_->get_node()->get_name());
            topic_trace::LogEstopEvent(ri_ptr_->get_node()->get_logger(), "[ESTOP]", buf);
        }
        estop_signal_sub_ = ri_ptr_->get_node()->create_subscription<drdds::msg::StdMsgInt32>(
            "/EMERGENCY_STOP_SIGNAL", 1,
            [this](const drdds::msg::StdMsgInt32::SharedPtr msg) {
                char buf[256];
                snprintf(buf, sizeof(buf),
                    "Received /EMERGENCY_STOP_SIGNAL value=%d, "
                    "current_state='%s', run_cnt=%d",
                    msg->value,
                    current_controller_ ? current_controller_->state_name_.c_str() : "nullptr",
                    run_cnt_);
                topic_trace::LogEstopEvent(ri_ptr_->get_node()->get_logger(), "[ESTOP][4/4]", buf);
                emergency_stop_requested_.store(true);
                topic_trace::LogEstopEvent(ri_ptr_->get_node()->get_logger(), "[ESTOP][4/4]",
                    "emergency_stop_requested_ set to true");
            }
        );
        topic_trace::LogEstopEvent(ri_ptr_->get_node()->get_logger(), "[ESTOP]",
            "/EMERGENCY_STOP_SIGNAL subscription created");
    }
};

