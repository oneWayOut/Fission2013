/*
 *       AP_MotorsQuad.cpp - ArduCopter motors library
 *       Code by RandyMackay. DIYDrones.com
 *
 *       This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 */

#include "AP_MotorsQuad.h"

#include "MasterOrSlave.h"  //cdc

#if MY_BOARD_MODE == SLAVE_MODE
extern uint8_t myflightMode;
#endif


// setup_motors - configures the motors for a quad
void AP_MotorsQuad::setup_motors()
{
    // call parent
    AP_MotorsMatrix::setup_motors();

    // hard coded config for supported frames
    if( _frame_orientation == AP_MOTORS_PLUS_FRAME ) {
        // plus frame set-up
        add_motor(AP_MOTORS_MOT_1,  90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
        add_motor(AP_MOTORS_MOT_2, -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
        add_motor(AP_MOTORS_MOT_3,   0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
        add_motor(AP_MOTORS_MOT_4, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);
    }else if( _frame_orientation == AP_MOTORS_V_FRAME ) {
        // V frame set-up
        add_motor(AP_MOTORS_MOT_1,   45,  0.7981,  1);
        add_motor(AP_MOTORS_MOT_2, -135,  1.0000,  3);
        add_motor(AP_MOTORS_MOT_3,  -45, -0.7981,  4);
        add_motor(AP_MOTORS_MOT_4,  135, -1.0000,  2);
    }else if( _frame_orientation == AP_MOTORS_H_FRAME ) {
        // H frame set-up - same as X but motors spin in opposite directions
        add_motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 1);
        add_motor(AP_MOTORS_MOT_2, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 3);
        add_motor(AP_MOTORS_MOT_3,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  4);
        add_motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  2);
    }else{
#if MY_BOARD_MODE == SLAVE_MODE
		if(myflightMode==1){//模式切换后重新配置为四轴X模式
			add_motor(AP_MOTORS_MOT_1,	 45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1);
			add_motor(AP_MOTORS_MOT_2, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3);
			add_motor(AP_MOTORS_MOT_3,	-45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4);
			add_motor(AP_MOTORS_MOT_4,	135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2);
		}
		else if(myflightMode==2){
			/*配置为单机半固定翼模式，电机倾转30度*/
			/*注意yawfactor, 0.2为电机旋向贡献的量，1.0为电机在左右侧推力贡献的量*/
			add_motor_raw(AP_MOTORS_MOT_1,	-1.0,  1.0, -1.0+0.2, 8);
			add_motor_raw(AP_MOTORS_MOT_2,	 1.0, -1.0,  1.0+0.2, 6);
			add_motor_raw(AP_MOTORS_MOT_3,	 1.0,  1.0,  1.0-0.2,  7);
			add_motor_raw(AP_MOTORS_MOT_4,	-1.0, -1.0, -1.0-0.2,  5);
		}else ;
#else
		// X frame set-up
        add_motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1);
        add_motor(AP_MOTORS_MOT_2, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3);
        add_motor(AP_MOTORS_MOT_3,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4);
        add_motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2);
#endif
    }
}
