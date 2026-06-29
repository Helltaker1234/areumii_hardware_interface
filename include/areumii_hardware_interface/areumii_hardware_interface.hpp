#include "hardware_interface/system_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "Sharemem.hpp"
#include <vector>


#define AREUMII_HARDWARE_INTERFACE_PUBLIC __attribute__((visibility("default")))



namespace areumii_hardware_interface
{
    constexpr int MOTOR_NUM =14;

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    
class AreumiiHardwareInterface : 
    public hardware_interface::SystemInterface,
    public rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
{
private:
    Control_Shm<MOTOR_NUM> ctrl_shm;
    Control_param ctrl_buf[MOTOR_NUM];
    Feedback_Param fb_buf[MOTOR_NUM];
    Control_Shm<MOTOR_NUM>::Control_Struct* shm_ptr;
public:

    AreumiiHardwareInterface() : ctrl_shm(13563267) { shm_ptr = ctrl_shm.get(); };


// 초기화 및 메모리 연결 (System Interface Setup)

    // override: 부모 클래스에서 정의된 가상 함수들을 자식 클래스에서 제대로 재정의함을 컴파일러에게 알림(C++11)
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    // 하드웨어 설정 파일을 로드하고 초기화
    //
    // 로봇의 URDF 파일 안에 작성된 <hardware> 태그의 정보(파라미터, 조인트 이름, 제한값 등)가 info 변수에 담겨 들어옴.
    // 이 info 정보들을 파싱하여 클래스 내부의 변수에 저장
    
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    // 센서 값(위치, 속도 등)을 ROS 2 시스템에 전달할 경로를 정의
    // "현재 위치나 속도 데이터는 내 클래스의 어떤 변수를 읽으면 돼!"라고 ROS 2 시스템에 메모리 주소(포인터)를 등록하는 과정

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    // 제어 명령(토크, 위치 등)을 받을 경로를 정의
    // ROS 2의 제어기(Controller)가 계산한 목표값을 써넣을 수 있도록, 클래스 내부의 특정 변수 공간을 개방해 주는 과정



// 라이프사이클 상태 관리 (Lifecycle State Machine)

    CallbackReturn on_configure(const rclcpp_lifecycle::State &);
    // 하드웨어 연결 및 메모리 할당 등을 수행합니다.
    // Unconfigured에서 Inactive 상태로 넘어갈 때 호출됩니다. 
    // USB/시리얼 포트를 열거나, 통신 속도를 맞추는 등 실제 통신을 위한 준비 작업을 여기서 합니다. (아직 모터를 돌리지는 않습니다.)

    CallbackReturn on_activate(const rclcpp_lifecycle::State &);
    // 하드웨어 제어를 시작(모터 전원 켜기 등)합니다.
    //
    // Inactive에서 Active 상태로 넘어갈 때 호출됩니다. 
    // 하드웨어 드라이버에 'Enable(활성화)' 신호를 보내 모터에 토크를 인가하고 움직일 준비를 완료합니다.

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
    // 하드웨어 제어를 중지(모터 정지 등)합니다.
    // Active에서 Inactive 상태로 내려올 때 호출됩니다. 들어오는 명령을 차단하고 로봇을 부드럽게 정지(Disable)시킵니다. 통신 포트는 아직 열려있습니다.

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &);
    // 자원을 해제하고 안전하게 종료합니다.
    // 동적으로 할당한 메모리를 지우고, 시리얼 포트나 통신 소켓을 완전히 닫아 시스템을 깔끔하게 종료합니다.


    CallbackReturn on_error(const rclcpp_lifecycle::State &);
    // 위 과정 중 어디선가 에러가 났을 때 호출되는 비상 탈출구입니다. 
    // 통신이 끊기는 등 치명적인 문제가 발생했을 때 모터를 강제로 끄는 등 안전(Failsafe) 로직을 구현합니다.



// 실시간 제어 루프 (Real-time Control Loop)
    
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    // 실제 하드웨어에서 센서 값을 읽어와 ROS 2 시스템에 업데이트합니다.
    // 
    // 실제 모터 드라이버나 엔코더에 통신(USB, CAN 등)을 보내 현재 상태값을 받아온 뒤, export_state_interfaces에서 연결해둔 변수에 그 값을 덮어씁니다.
    
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    // ROS 2 시스템에서 받은 명령을 실제 모터/드라이버로 전송합니다.
    // 
    // ROS 2 제어기가 계산해서 export_command_interfaces로 열어둔 변수에 써넣은 목표값(목표 각도, 속도 등)을 읽어서, 
    // 실제 하드웨어가 알아들을 수 있는 패킷으로 변환해 통신 케이블로 전송합니다.


// (optional)
// prepare_command_mode_switch()
// perform_command_mode_switch()


private:

    // 조인트 상태 및 명령을 저장할 벡터 (A1_URDF.ros2_control.xacro 에 맞춤)
    std::vector<double> pos_commands_;
    std::vector<double> vel_commands_;

    std::vector<double> pos_states_;
    std::vector<double> vel_states_;



};

} 
