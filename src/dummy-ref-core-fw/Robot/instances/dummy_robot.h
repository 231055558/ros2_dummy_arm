#ifndef REF_STM32F4_FW_DUMMY_ROBOT_H
#define REF_STM32F4_FW_DUMMY_ROBOT_H

#include "algorithms/kinematic/6dof_kinematic.h"
#include "actuators/ctrl_step/ctrl_step.hpp"
#include "string"
#define ALL 0

/*
  |   PARAMS   | `current_limit` | `acceleration` | `dce_kp` | `dce_kv` | `dce_ki` | `dce_kd` |
  | ---------- | --------------- | -------------- | -------- | -------- | -------- | -------- |
  | **Joint1** | 2               | 30             | 1000     | 80       | 200      | 250      |
  | **Joint2** | 3               | 30             | 1000     | 80       | 200      | 200      |
  | **Joint3** | 3               | 30             | 1500     | 80       | 200      | 250      |
  | **Joint4** | 2               | 30             | 1000     | 80       | 200      | 250      |
  | **Joint5** | 2               | 30             | 1000     | 80       | 200      | 250      |
  | **Joint6** | 2               | 30             | 1000     | 80       | 200      | 250      |
 */


class DummyHand
{
public:
    uint8_t nodeID = 7;
//    float maxCurrent = 0.7;
    float maxCurrent = 0.7;


    DummyHand(CAN_HandleTypeDef* _hcan, uint8_t _id);


    void SetAngle(float _angle);
    void SetMaxCurrent(float _val);
    void SetEnable(bool _enable);


    // Communication protocol definitions
    auto MakeProtocolDefinitions()
    {
        return make_protocol_member_list(
            make_protocol_function("set_angle", *this, &DummyHand::SetAngle, "angle"),
            make_protocol_function("set_enable", *this, &DummyHand::SetEnable, "enable"),
            make_protocol_function("set_current_limit", *this, &DummyHand::SetMaxCurrent, "current")
        );
    }


private:
    CAN_HandleTypeDef* hcan;
    uint8_t canBuf[8];
    CAN_TxHeaderTypeDef txHeader;
    float minAngle = 0;
    float maxAngle = 45;
};


class StepHand : public CtrlStepMotor {
public:
    // 构造函数
    StepHand(CAN_HandleTypeDef* hcan, uint8_t id)
            : CtrlStepMotor(hcan, id, false, 8, -100, 100) // 固定参数：inverse=false, reduction=1, 角度范围0-100
    {
    }

    float current=0.7;

    float OpenedAngle=100;
    float ClosedAngle=-100;

    //限速控制位置
    void SetAngleWithSpeedLimit (float _angle)
    {
        float ag= ClosedAngle+_angle;
        SetAngleWithVelocityLimit(ag,70);
    }
    //电流控制位置
    void SetAngleWithCurrentLimit(float inverse)
    {
        SetCurrentSetPoint(inverse*current);
    }

    //
    //校准位置模式 
    void HandCalibration()
    {

    }

    // ✅ 便捷控制方法 - 直接对应ASCII命令
    void OpenHand()
    {
        SetAngleWithCurrentLimit(1);  // 对应 !HAND_O
    }
    
    void CloseHand()
    {
        SetAngleWithCurrentLimit(-1); // 对应 !HAND_C
    }

    // ✅ Fibre协议包装方法 - 解决类型安全问题
    void SetHandEnable(bool enable)
    {
        CtrlStepMotor::SetEnable(enable);  // 包装基类方法
    }
    
    void SetHandPosition(float pos)
    {
        CtrlStepMotor::SetPositionSetPoint(pos);  // 包装基类方法
    }
    
    void SetHandCurrent(float current)
    {
        CtrlStepMotor::SetCurrentSetPoint(current);  // 包装基类方法
    }
    
    void SetHandVelocity(float vel)
    {
        CtrlStepMotor::SetVelocitySetPoint(vel);  // 包装基类方法
    }

    // 状态查询
    bool isEnabled() const {
        return state != STOP;
    }

    // ✅ Fibre协议支持 - 完整暴露夹爪控制功能
    auto MakeProtocolDefinitions()
    {
        return make_protocol_member_list(
            // 基础控制功能 - 使用包装方法解决类型安全问题
            make_protocol_function("set_enable", *this, &StepHand::SetHandEnable, "enable"),        // !HAND_EN/!HAND_DIS
            make_protocol_function("set_position", *this, &StepHand::SetHandPosition, "pos"),       // 基础位置控制
            make_protocol_function("set_current", *this, &StepHand::SetHandCurrent, "current"),     // 基础电流控制
            make_protocol_function("set_velocity", *this, &StepHand::SetHandVelocity, "vel"),       // 基础速度控制
            
            // StepHand特有的高级功能
            make_protocol_function("set_angle_with_speed_limit", *this, &StepHand::SetAngleWithSpeedLimit, "angle"),      // !HAND_POS
            make_protocol_function("set_angle_with_current_limit", *this, &StepHand::SetAngleWithCurrentLimit, "inverse"), // !HAND_O/!HAND_C
            make_protocol_function("hand_calibration", *this, &StepHand::HandCalibration),          // !HAND_ZERO
            
            // 便捷控制方法 (直接对应ASCII命令)
            make_protocol_function("open", *this, &StepHand::OpenHand),      // 直接打开 (!HAND_O)
            make_protocol_function("close", *this, &StepHand::CloseHand),    // 直接关闭 (!HAND_C)
            
            // 暴露关键属性
            make_protocol_property("current", &this->current),
            make_protocol_property("opened_angle", &this->OpenedAngle),
            make_protocol_property("closed_angle", &this->ClosedAngle),
            make_protocol_ro_property("angle", &this->angle),
            make_protocol_ro_property("temperature", &this->temperature)
        );
    }

};




class DummyRobot
{
public:
    explicit DummyRobot(CAN_HandleTypeDef* _hcan);
    ~DummyRobot();


    enum CommandMode
    {
        COMMAND_TARGET_POINT_SEQUENTIAL = 1,
        COMMAND_TARGET_POINT_INTERRUPTABLE,
        COMMAND_CONTINUES_TRAJECTORY,
        COMMAND_MOTOR_TUNING
    };


    class TuningHelper
    {
    public:
        explicit TuningHelper(DummyRobot* _context) : context(_context)
        {
        }

        void SetTuningFlag(uint8_t _flag);
        void Tick(uint32_t _timeMillis);
        void SetFreqAndAmp(float _freq, float _amp);


        // Communication protocol definitions
        auto MakeProtocolDefinitions()
        {
            return make_protocol_member_list(
                make_protocol_function("set_tuning_freq_amp", *this,
                                       &TuningHelper::SetFreqAndAmp, "freq", "amp"),
                make_protocol_function("set_tuning_flag", *this,
                                       &TuningHelper::SetTuningFlag, "flag")
            );
        }


    private:
        DummyRobot* context;
        float time = 0;
        uint8_t tuningFlag = 0;
        float frequency = 1;
        float amplitude = 1;
    };
    TuningHelper tuningHelper = TuningHelper(this);


    // This is the pose when power on.
    const DOF6Kinematic::Joint6D_t REST_POSE = {0, -75, 180, 0, 0, 0};
//    const float DEFAULT_JOINT_SPEED = 30;  // degree/s
    const float DEFAULT_JOINT_SPEED = 80;  // degree/s
    const DOF6Kinematic::Joint6D_t DEFAULT_JOINT_ACCELERATION_BASES = {150, 100, 200, 200, 200, 200};
//    const float DEFAULT_JOINT_ACCELERATION_LOW = 30;    // 0~100
    const float DEFAULT_JOINT_ACCELERATION_LOW = 5;    // 0~100
    const float DEFAULT_JOINT_ACCELERATION_HIGH = 100;  // 0~100
    const CommandMode DEFAULT_COMMAND_MODE = COMMAND_TARGET_POINT_INTERRUPTABLE;


    DOF6Kinematic::Joint6D_t currentJoints = REST_POSE;
    DOF6Kinematic::Joint6D_t targetJoints = REST_POSE;
    DOF6Kinematic::Joint6D_t initPose = REST_POSE;
    DOF6Kinematic::Pose6D_t currentPose6D = {};
    volatile uint8_t jointsStateFlag = 0b00000000;
    CommandMode commandMode = DEFAULT_COMMAND_MODE;
    CtrlStepMotor* motorJ[7] = {nullptr};
    DummyHand* hand = {nullptr};
//    CtrlStepMotor* hand2 = {nullptr};
    StepHand* hand2 = {nullptr};


    void Init();
    bool MoveJ(float _j1, float _j2, float _j3, float _j4, float _j5, float _j6);
    bool MoveL(float _x, float _y, float _z, float _a, float _b, float _c);
    void MoveJoints(DOF6Kinematic::Joint6D_t _joints);
    void SetJointSpeed(float _speed);
    void SetJointAcceleration(float _acc);
    void UpdateJointAngles();
    void UpdateJointAnglesCallback();
    void UpdateJointPose6D();
    void Reboot();
    void SetEnable(bool _enable);
    void SetRGBEnable(bool _enable);
    bool GetRGBEnabled();
    void SetRGBMode(uint32_t mode);
    uint32_t GetRGBMode();
    void CalibrateHomeOffset();
    void Homing();
    void Resting();
    bool IsMoving();
    bool IsEnabled();
    void SetCommandMode(uint32_t _mode);


    // Communication protocol definitions
    auto MakeProtocolDefinitions()
    {
        return make_protocol_member_list(
            make_protocol_function("calibrate_home_offset", *this, &DummyRobot::CalibrateHomeOffset),
            make_protocol_function("homing", *this, &DummyRobot::Homing),
            make_protocol_function("resting", *this, &DummyRobot::Resting),
            make_protocol_object("joint_1", motorJ[1]->MakeProtocolDefinitions()),
            make_protocol_object("joint_2", motorJ[2]->MakeProtocolDefinitions()),
            make_protocol_object("joint_3", motorJ[3]->MakeProtocolDefinitions()),
            make_protocol_object("joint_4", motorJ[4]->MakeProtocolDefinitions()),
            make_protocol_object("joint_5", motorJ[5]->MakeProtocolDefinitions()),
            make_protocol_object("joint_6", motorJ[6]->MakeProtocolDefinitions()),
            make_protocol_object("joint_all", motorJ[ALL]->MakeProtocolDefinitions()),
            make_protocol_object("hand", hand2->MakeProtocolDefinitions()),  // ✅ 启用：StepHand已实现MakeProtocolDefinitions
            make_protocol_function("reboot", *this, &DummyRobot::Reboot),
            make_protocol_function("set_enable", *this, &DummyRobot::SetEnable, "enable"),
            make_protocol_function("set_rgb_enable", *this, &DummyRobot::SetRGBEnable, "enable"),
            make_protocol_function("set_rgb_mode", *this, &DummyRobot::SetRGBMode, "mode"),
            make_protocol_function("move_j", *this, &DummyRobot::MoveJ, "j1", "j2", "j3", "j4", "j5", "j6"),
            make_protocol_function("move_l", *this, &DummyRobot::MoveL, "x", "y", "z", "a", "b", "c"),
            make_protocol_function("set_joint_speed", *this, &DummyRobot::SetJointSpeed, "speed"),
            make_protocol_function("set_joint_acc", *this, &DummyRobot::SetJointAcceleration, "acc"),
            make_protocol_function("set_command_mode", *this, &DummyRobot::SetCommandMode, "mode"),
            make_protocol_object("tuning", tuningHelper.MakeProtocolDefinitions())
        );
    }


    class CommandHandler
    {
    public:
        explicit CommandHandler(DummyRobot* _context) : context(_context)
        {
            commandFifo = osMessageQueueNew(16, 64, nullptr);
        }

        uint32_t Push(const std::string &_cmd);
        std::string Pop(uint32_t timeout);
        uint32_t ParseCommand(const std::string &_cmd);
        uint32_t GetSpace();
        void ClearFifo();
        void EmergencyStop();


    private:
        DummyRobot* context;
        osMessageQueueId_t commandFifo;
        char strBuffer[64]{};
    };
    CommandHandler commandHandler = CommandHandler(this);


private:
    CAN_HandleTypeDef* hcan;
    float jointSpeed = DEFAULT_JOINT_SPEED;
    float jointSpeedRatio = 1;
//    DOF6Kinematic::Joint6D_t dynamicJointSpeeds = {1, 1, 1, 1, 1, 1};
//    DOF6Kinematic::Joint6D_t dynamicJointSpeeds = {0.5f, 0.5f, 0.5f, 1.5f, 1.5f, 1.5f};
    DOF6Kinematic::Joint6D_t dynamicJointSpeeds = {0.8f, 1.2f, 1.2f, 1.5f, 1.5f, 1.5f};  // ✅ 提高J2,J3速度比例
    DOF6Kinematic* dof6Solver;
    bool isEnabled = false;
    bool isRGBEnabled = true;
    uint32_t rgbMode = 0;
};


#endif //REF_STM32F4_FW_DUMMY_ROBOT_H
