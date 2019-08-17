/*
 *       AP_MotorsOcta.cpp - ArduCopter motors library
 *       Code by RandyMackay. DIYDrones.com
 *
 *       This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 */

#include "AP_MotorsOcta.h"
#include "MasterOrSlave.h"  //cdc

#if MY_BOARD_MODE == MASTER_MODE
extern uint8_t myflightMode;
#endif

// setup_motors - configures the motors for a octa
void AP_MotorsOcta::setup_motors()
{
    // call parent
    AP_MotorsMatrix::setup_motors();

    // hard coded config for supported frames
    if( _frame_orientation == AP_MOTORS_PLUS_FRAME ) {
        // plus frame set-up
        add_motor(AP_MOTORS_MOT_1,    0,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
        add_motor(AP_MOTORS_MOT_2,  180,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5);
        add_motor(AP_MOTORS_MOT_3,   45,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
        add_motor(AP_MOTORS_MOT_4,  135,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
        add_motor(AP_MOTORS_MOT_5,  -45,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8);
        add_motor(AP_MOTORS_MOT_6, -135,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6);
        add_motor(AP_MOTORS_MOT_7,  -90,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7);
        add_motor(AP_MOTORS_MOT_8,   90,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);

    }else if( _frame_orientation == AP_MOTORS_V_FRAME ) {
        // V frame set-up
        add_motor_raw(AP_MOTORS_MOT_1,  1.0,  0.34, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7);
        add_motor_raw(AP_MOTORS_MOT_2, -1.0, -0.32, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);
        add_motor_raw(AP_MOTORS_MOT_3,  1.0, -0.32, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6);
        add_motor_raw(AP_MOTORS_MOT_4, -0.5,  -1.0, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
        add_motor_raw(AP_MOTORS_MOT_5,  1.0,   1.0, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8);
        add_motor_raw(AP_MOTORS_MOT_6, -1.0,  0.34, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
        add_motor_raw(AP_MOTORS_MOT_7, -1.0,   1.0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
        add_motor_raw(AP_MOTORS_MOT_8,  0.5,  -1.0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5);

    }
	/*cdc defined OCTA Frame 2013*/
#if MY_BOARD_MODE == MASTER_MODE
	else if(_frame_orientation == AP_MOTORS_MY_OCTA){
		if(myflightMode==0){
			/*original octa frame			
			7------5------1------3
				   |	  |
				   |	  |
			6------2------4------8         */				
			/*original octa frame motor setup
	        add_motor_raw(AP_MOTORS_MOT_1, -0.33,  1.0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
	        add_motor_raw(AP_MOTORS_MOT_2,  0.33, -1.0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5);
	        add_motor_raw(AP_MOTORS_MOT_3,  -1.0,  1.0, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
	        add_motor_raw(AP_MOTORS_MOT_4, -0.33, -1.0, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
	        add_motor_raw(AP_MOTORS_MOT_5,  0.33,  1.0, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8);
	        add_motor_raw(AP_MOTORS_MOT_6,   1.0, -1.0, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6);
	        add_motor_raw(AP_MOTORS_MOT_7,   1.0,  1.0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7);
	        add_motor_raw(AP_MOTORS_MOT_8,  -1.0, -1.0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);*/



			/*2013.9.3重新分配了通道,顺序排列电机*/
			/*
			3------1------7------5
				   |	  |          5和8间距90
				   |	  |
			2------4------6------8
	间距      100     50    100
			*/
			add_motor_raw(AP_MOTORS_MOT_1,  0.25,  1.0, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1);
			add_motor_raw(AP_MOTORS_MOT_2,   1.0, -1.0, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
			add_motor_raw(AP_MOTORS_MOT_3,   1.0,  1.0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);
			add_motor_raw(AP_MOTORS_MOT_4,  0.25, -1.0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4);

			add_motor_raw(AP_MOTORS_MOT_5,  -1.0,  1.0, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5);
	        add_motor_raw(AP_MOTORS_MOT_6, -0.25, -1.0, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6);
			add_motor_raw(AP_MOTORS_MOT_7, -0.25,  1.0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7);
			add_motor_raw(AP_MOTORS_MOT_8,  -1.0, -1.0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  8);
		}
		else if(myflightMode==1){//模式切换后重新配置为四轴X模式
	        add_motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1);
	        add_motor(AP_MOTORS_MOT_2, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
	        add_motor(AP_MOTORS_MOT_3,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);
	        add_motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4);
		}
		else if(myflightMode==2){
			/*配置为单机半固定翼模式，电机倾转30度*/
			/*注意yawfactor, 0.2为电机旋向贡献的量，1.0为电机在左右侧推力贡献的量*/
			add_motor_raw(AP_MOTORS_MOT_1,  -1.0,  1.0, -1.0+0.2, 1);
			add_motor_raw(AP_MOTORS_MOT_2,   1.0, -1.0,  1.0+0.2, 2);
			add_motor_raw(AP_MOTORS_MOT_3,   1.0,  1.0,  1.0-0.2, 3);
			add_motor_raw(AP_MOTORS_MOT_4,  -1.0, -1.0, -1.0-0.2, 4);
		}
		else ;
    }
#endif
	else {
        // X frame set-up
        add_motor(AP_MOTORS_MOT_1,   22.5, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
        add_motor(AP_MOTORS_MOT_2, -157.5, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5);
        add_motor(AP_MOTORS_MOT_3,   67.5, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
        add_motor(AP_MOTORS_MOT_4,  157.5, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
        add_motor(AP_MOTORS_MOT_5,  -22.5, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8);
        add_motor(AP_MOTORS_MOT_6, -112.5, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6);
        add_motor(AP_MOTORS_MOT_7,  -67.5, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7);
        add_motor(AP_MOTORS_MOT_8,  112.5, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);
    }
}
