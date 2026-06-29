#include "areumii_hardware_interface/areumii_hardware_interface.hpp"

#include <limits>
#include <vector>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace areumii_hardware_interface
{

    hardware_interface::CallbackReturn AreumiiHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        // Warning: 이게 그냥 예외처리 코드가 아니다!!
        // 
        // 부모 클래스(hardware_interface::SystemInterface)에서 on_init() 함수를 호출을 하는 것!!
        // 이를 자식 클래스의 on_init() 에서 하는 거임!!
        //
        // 하는 일은 입력받은 인수인 info 를 info_ 에 대입하는 것.
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        pos_commands_.assign(info.joints.size(), std::numeric_limits<double>::quiet_NaN());
        vel_commands_.assign(info.joints.size(), std::numeric_limits<double>::quiet_NaN());


        pos_states_.assign(info.joints.size(), std::numeric_limits<double>::quiet_NaN());
        vel_states_.assign(info.joints.size(), std::numeric_limits<double>::quiet_NaN());
        

        return hardware_interface::CallbackReturn::SUCCESS;
    }



    CallbackReturn AreumiiHardwareInterface::on_shutdown(const rclcpp_lifecycle::State &)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    CallbackReturn AreumiiHardwareInterface::on_error(const rclcpp_lifecycle::State &)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }




    CallbackReturn AreumiiHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
    {
    // TODO
        
        // 하드웨어 연결 및 메모리 할당 등을 수행합니다.
        // USB/시리얼 포트를 열거나, 통신 속도를 맞추는 등 실제 통신을 위한 준비 작업을 여기서 합니다. (아직 모터를 돌리지는 않습니다.)

        // 하드웨어 자원 할당 및 통신 시작 코드
        // 예를 들어 시리얼 통신의 경우
        // my_driver_.open("/dev/ttyUSB0", 115200);

        // 이 함수가 호출된 후에는 state interface (센서 값) 은 받아올 수 있지만,
        // command 는 모터로 줄 수 없는 상태가 됨 (on_activate 를 호출해야만 구동 가능)

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn AreumiiHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
    {
    // TODO

        // on_configure 와 반대되는 흐름으로 코드

        return CallbackReturn::SUCCESS;
    }

    
    
    CallbackReturn AreumiiHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
    {
        // command 를 줄 수 있는 상태가 됨.
        return CallbackReturn::SUCCESS;
    }


    CallbackReturn AreumiiHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
    {
        return CallbackReturn::SUCCESS;
    }




    std::vector<hardware_interface::StateInterface> AreumiiHardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            // 하드웨어 인터페이스 내부의 변수와 ROS 2 시스템(컨트롤러) 사이의 통로를 연결하는 핵심적인 부분
            // "내 클래스의 이 변수가 로봇의 어느 데이터를 담당하는지"를 ROS 2에 등록하는 과정
            //
            // 정확히는 pos_states_ 변수가 private 멤버 변수이므로, 외부에서 해당 변수를 호출할 수 없음.
            // 
            // 이때 ROS2 Controller Manager 가 export_state_interfaces() 함수를 호출한다는 점을 이용하여, 
            // state_interfaces.emplace_back() 함수에 의해 다음을 알게 됨. 
            //      - info_.joints[i].name                  : 조인트 이름
            //      - hardware_interface::HW_IF_POSITION    : 제어 타입
            //      - &pos_states_[i]                     : pos_states_ 주소
            //
            // emplace_back 된 state_interfaces return 함으로써 
            // 하드웨어 인터페이스 내부의 변수(pos_states_)와 ROS 2 시스템(컨트롤러) 사이의 통로를 연결하게 됨.

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_states_[i]));
            // [ &pos_states_[i] >> 각 joint 의 state interface 의 값이 담길 변수의 주소 ]
            //
            //   0. StateInterface() : xacro 의 joint 의 state interface 와 &pos_states_[i] 주소를 연결. 
            //                         (이를 통해 Controller manager 가 이를 알 수 있게 한다.)
            //   1. read() 함수로 모터의 각도 값을 읽어옴. 이를 pos_states_[i] 변수에 값으로 "덮어씌움".
            //   2. Controller manager 가 &pos_states_[i] 주소를 JointStateBroadcaster 에게 전달함.
            //   3. JointStateBroadcaster 가 &pos_states_[i] 주소의 pos_states_[i] 값을 읽음.
            

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_states_[i]));
                
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> AreumiiHardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            // 하드웨어 인터페이스 내부의 변수와 ROS 2 시스템(컨트롤러) 사이의 통로를 연결하는 핵심적인 부분
            // "내 클래스의 이 변수가 로봇의 어느 데이터를 담당하는지"를 ROS 2에 등록하는 과정

            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_commands_[i]));
            // [ &pos_commands_[i] >> 각 joint 의 command interface 의 값이 담길 변수의 주소 ]
            //
            //   0. CommandInterface() : xacro 의 joint 의 command interface 와 &pos_commands_[i] 주소를 연결. 
            //                           (이를 통해 Controller manager 가 이를 알 수 있게 한다.)
            //   1. JointTrajectoryController(또는 임의의 ROS2 컨트롤러)가 출력한 command(또는 desired, reference 등) 값을 &pos_commands_[i] 주소에 직접 써넣는다.
            //   2. 이후 write() 함수에서 pos_commands_[i] 변수에 적힌 값을 보고, 실제 모터로 통신을 보냄. 
            

            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_commands_[i]));

        }
        return command_interfaces;
    }



    hardware_interface::return_type AreumiiHardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        while (!shm_ptr->try_read_fb(fb_buf));  // torn read면 재시도        
        
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            pos_states_[i] = static_cast<double>(fb_buf[i].pos);
            vel_states_[i] = static_cast<double>(fb_buf[i].vel);
        }
        
        
        return hardware_interface::return_type::OK;
    }

    
    
    hardware_interface::return_type AreumiiHardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        // ctrl_buf[0].pos = 1;
        // ctrl_buf[1].pos = 1;
        // ctrl_buf[2].pos = 1;
        // ctrl_buf[3].pos = 1;
        // ctrl_buf[4].pos = 1;
        // ctrl_buf[5].pos = 1;
        // ctrl_buf[6].pos = 1;
        // ctrl_buf[7].pos = 1;
        // ctrl_buf[8].pos = 1;
        // ctrl_buf[9].pos = 1;
        // ctrl_buf[10].pos = 1;
        // ctrl_buf[11].pos = 1;
        // ctrl_buf[12].pos = 1;
        // ctrl_buf[13].pos = 1;  // 단위:rad 

        // ctrl_buf[0].vel = 1;
        // ctrl_buf[1].vel = 1;
        // ctrl_buf[2].vel = 1;
        // ctrl_buf[3].vel = 1;
        // ctrl_buf[4].vel = 1;
        // ctrl_buf[5].vel = 1;
        // ctrl_buf[6].vel = 1;
        // ctrl_buf[7].vel = 1;
        // ctrl_buf[8].vel = 1;
        // ctrl_buf[9].vel = 1;
        // ctrl_buf[10].vel = 1;
        // ctrl_buf[11].vel = 1;
        // ctrl_buf[12].vel = 1;
        // ctrl_buf[13].vel = 1;  // 단위:rad/s 
        // shm_ptr->write_ctrl(ctrl_buf);
        //나는 보통 이런식으로 루프 언롤링을 선호함. 또한 숫자를 매크로나 상수 표현으로 하는 편. 


        
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            ctrl_buf[i].pos = pos_commands_[i];
            ctrl_buf[i].vel = vel_commands_[i];

            // {pos, vel} 제어의 경우 >> 미완성
            // 승윤이_write(pos_commands_[i]);
            // 승윤이_write(vel_commands_[i]);

        }
        shm_ptr->write_ctrl(ctrl_buf);

        return hardware_interface::return_type::OK;
    }




}


PLUGINLIB_EXPORT_CLASS(
areumii_hardware_interface::AreumiiHardwareInterface, hardware_interface::SystemInterface
)