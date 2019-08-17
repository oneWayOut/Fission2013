/****************************************
      cdc  add for Master APM code
****************************************/
#include <MasterOrSlave.h>

#if MY_BOARD_MODE == MASTER_MODE

#define MYMSG_RQ   0xA5     /*request for master msg*/
#define MYMSG_NACK 0xA7     /*nack master*/

static int16_t mymotor_out[4] = {0,0,0,0};

static uint8_t changeModeFlag = 0;


static void masterUARTInit(void)
{
		/*重新设置6,7,8通道的pwm频率为50Hz*/
	hal.rcout->set_freq( _BV(6), 50);  /*6=CH_7, 设置6,7,8*/	


	hal.uartC->begin(115200, 8, 64);  /*baudRate, rxSpace, txSpace*/
	hal.scheduler->delay(5);
//	hal.uartA->begin(115200, 8, 64);  /*baudRate, rxSpace, txSpace*/

	hal.uartA->printf("master\n");
}

static void sendSlaveMsg(void)
{
	uint8_t  recvByte;	
	uint8_t  num, ix;	
	uint8_t  sendByte;

	num = hal.uartC->available();

	for(ix=0; ix<num; ix++){
		recvByte = hal.uartC->read();
		switch(recvByte){
		case MYMSG_RQ:
			if(changeModeFlag == 0){
				hal.uartC->write(0xFF);
			}else{  /*通知从机切换模式，同时设置主机模式*/
				hal.uartC->write(0xFA);
				masterMode = 0;
			}

			setMyMotorOut();
			
			for(ix=0; ix<4; ix++){
				sendByte = (mymotor_out[ix]&0xFF00)>>8;
				hal.uartC->write(sendByte);
				sendByte = (mymotor_out[ix]&0xFF);
				hal.uartC->write(sendByte);		
			}
			ix = num; //jump out
			break;
		default: /**/
			break;
		}
		
	}
		

	/*清空未读完的废数据*/
	if(hal.uartC->available()>=1){
		hal.uartC->flush();
	}	


		/*roll通道的输出至舵机测试, cdc test*/
	//hal.rcout->write(6, g.rc_1.radio_out);	

}

 
/***********************************************************************************
original octa frame

   7------5------1------3
          |      |
          |      |
   6------2------4------8


new my octa frame: 3,4,7,8 motors are clockwise
   3------1------7------5
          |      |
          |      |
   2------4------6------8
*************************************************************************************/
/*set output to mymotor_out[4]
 */
static void setMyMotorOut(void)
{
	mymotor_out[0] = motors.motor_out[4];
	mymotor_out[1] = motors.motor_out[5];
	mymotor_out[2] = motors.motor_out[6];
	mymotor_out[3] = motors.motor_out[7];
}

/**************************************************
模式切换，分离时需要进行的工作:
1 重新设置机架配置，转换为四轴模式；
2 发送消息给从机，通知其模式转换
**************************************************/
static void changeMasterMode(void)
{	
	/*改写了AP_MotorsOcta.cpp中的setup_motors函数*/
	masterMode = 0;
	motors.setup_motors();
	masterMode = 1;

	changeModeFlag = 1;	
}

#endif
