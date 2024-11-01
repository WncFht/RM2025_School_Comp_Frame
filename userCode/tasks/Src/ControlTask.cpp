//
// Created by LEGION on 2021/10/4.
//
#include "ControlTask.h"

void CtrlHandle(){
    if (RemoteControl::rcInfo.sRight == DOWN_POS){//右侧三档，急停模式
        ChassisStop();
				UserStop();
    }else {//其他正常模式
        switch (RemoteControl::rcInfo.sLeft) {
            case UP_POS://左侧一档，底盘控制模式
                // 遥控器数值映射到底盘速度
                // right_col: 前后速度控制
                // right_rol: 左右速度控制
                // left_rol: 旋转速度控制
                ChassisSetVelocity(RemoteControl::rcInfo.right_col*4.2,
                                 RemoteControl::rcInfo.right_rol*4.2,
                                 RemoteControl::rcInfo.left_rol*60);
                break;
            case MID_POS://左侧二档，预留
            case DOWN_POS://左侧三档，预留
            default:
                ChassisStop();
                break;
        }
    }
}