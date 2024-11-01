//
// Created by LEGION on 2021/10/4.
//
#include "ChassisTask.h"

PID_Regulator_t pidRegulator = {//此为储存pid参数的结构体，四个底盘电机共用
        .kp = 60,
        .ki = 0,
        .kd = 0,
        .componentKpMax = 16384,
        .componentKiMax = 0,
        .componentKdMax = 0,
        .outputMax = 16384 //3508电机输出电流上限，可以调小，勿调大
};

MOTOR_INIT_t chassisMotorInit = {//四个底盘电机共用的初始化结构体
        .speedPIDp = &pidRegulator,
        .anglePIDp = nullptr,
        ._motorID = MOTOR_ID_1,
        .reductionRatio = 19.0f,
        .ctrlType = SPEED_Single,
};
Motor CMFL(MOTOR_ID_1,&chassisMotorInit);//定义左前轮电机
Motor CMFR(MOTOR_ID_2,&chassisMotorInit);//定义右前轮电机
Motor CMBL(MOTOR_ID_3,&chassisMotorInit);//定义左后轮电机
Motor CMBR(MOTOR_ID_4,&chassisMotorInit);//定义右后轮电机

uint8_t ChassisStopFlag = 1;
float FBVelocity,LRVelocity,RTVelocity;

void ChassisStart(){
    ChassisStopFlag = 0;
    FBVelocity = 0;
    LRVelocity = 0;
    RTVelocity = 0;
}

/**
 * @brief 底盘任务的处理函数，定时执行
 * @callergraph void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) in Device.cpp
 */
void ChassisHandle() {
    if(ChassisStopFlag == 0) {
        WheelsSpeedCalc(FBVelocity, LRVelocity, RTVelocity);
    }
    CMFL.Handle();
    CMFR.Handle();
    CMBL.Handle();
    CMBR.Handle();
}

/**
 * @brief 用于控制任务控制底盘速度
 * @param _fbV 底盘前后方向速度
 * @param _lrV 底盘左右方向速度
 * @param _rtV 底盘旋转速度
 */
void ChassisSetVelocity(float _fbV,float _lrV,float _rtV){
    ChassisStopFlag = 0;
    FBVelocity = _fbV;
    LRVelocity = _lrV;
    RTVelocity = _rtV;
}

/**
 * @brief 执行急停模式的底盘任务处理
 */
void ChassisStop(){
    ChassisStopFlag = 1;
    CMFL.Stop();
    CMFR.Stop();
    CMBL.Stop();
    CMBR.Stop();
}

/**
 * @brief 计算麦克纳姆轮速度
 * @param fbVelocity 前后方向速度
 * @param lrVelocity 左右方向速度
 * @param rtVelocity 旋转速度
 */
void WheelsSpeedCalc(float fbVelocity, float lrVelocity, float rtVelocity) {
    float CMFLSpeed, CMFRSpeed, CMBLSpeed, CMBRSpeed;
    
    // 将旋转速度从RPM转换为弧度/秒
    rtVelocity = RPM2RADpS(rtVelocity);

    /*
     * 麦克纳姆轮运动学逆解:
     * 左前轮 = +前后速度 -左右速度 -旋转速度
     * 右前轮 = +前后速度 +左右速度 +旋转速度
     * 左后轮 = +前后速度 +左右速度 -旋转速度
     * 右后轮 = +前后速度 -左右速度 +旋转速度
     */
    
    // 计算四个轮子的速度
    CMFLSpeed = fbVelocity - lrVelocity - rtVelocity;
    CMFRSpeed = fbVelocity + lrVelocity + rtVelocity;
    CMBLSpeed = fbVelocity + lrVelocity - rtVelocity;
    CMBRSpeed = fbVelocity - lrVelocity + rtVelocity;

    // 将线速度转换为角速度（rad/s）
    CMFLSpeed = CMFLSpeed / (WHEEL_DIAMETER/2.0f);
    CMFRSpeed = CMFRSpeed / (WHEEL_DIAMETER/2.0f);
    CMBLSpeed = CMBLSpeed / (WHEEL_DIAMETER/2.0f);
    CMBRSpeed = CMBRSpeed / (WHEEL_DIAMETER/2.0f);

    // 设置电机目标速度（转换为RPM）
    CMFL.SetTargetSpeed(-RADpS2RPM(CMFLSpeed));  // 左侧电机反向
    CMFR.SetTargetSpeed(RADpS2RPM(CMFRSpeed));
    CMBL.SetTargetSpeed(-RADpS2RPM(CMBLSpeed));  // 左侧电机反向
    CMBR.SetTargetSpeed(RADpS2RPM(CMBRSpeed));
}