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
	hal.uartC->begin(115200, 8, 64);  /*baudRate, rxSpace, txSpace*/
	hal.scheduler->delay(5);
//	hal.uartA->begin(115200, 8, 64);  /*baudRate, rxSpace, txSpace*/
	hal.scheduler->delay(5);

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
			}else{  /*֪ͨ�ӻ��л�ģʽ��ͬʱ��������ģʽ*/
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
		

	/*���δ����ķ�����*/
	if(hal.uartC->available()>=1){
		hal.uartC->flush();
	}	

}

 
/***********************************************************************************
my octa frame: 7,2,1,8 motors are clockwise
   7------5------1------3
          |      |
          |      |
   6------2------4------8
master left side change to quad X frame, right side is controlled by slave board
   7------5          3------1
          | changeto |      |
          |          |      |
   6------2          2------4
*************************************************************************************/
/*set output to mymotor_out[4]
 */
static void setMyMotorOut(void)
{
	mymotor_out[0] = motors.motor_out[2];
	mymotor_out[1] = motors.motor_out[3];
	mymotor_out[2] = motors.motor_out[0];
	mymotor_out[3] = motors.motor_out[7];
}

/**************************************************
ģʽ�л�������ʱ��Ҫ���еĹ���:
1 �������û������ã�ת��Ϊ����ģʽ��
2 ������Ϣ���ӻ���֪ͨ��ģʽת��
**************************************************/
static void changeMasterMode(void)
{	
	/*��д��AP_MotorsOcta.cpp�е�setup_motors����*/
	masterMode = 0;
	motors.setup_motors();
	masterMode = 1;

	changeModeFlag = 1;	
}

#endif
