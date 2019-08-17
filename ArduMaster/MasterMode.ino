/****************************************
      cdc  add for Master APM code
****************************************/
#include <MasterOrSlave.h>
#if MY_BOARD_MODE == MASTER_MODE

#define MYMSG_RQ   0xA5     /*request for master msg*/
#define MYMSG_NACK 0xA7     /*nack master*/

static int16_t mymotor_out[4] = {0,0,0,0}; 

static uint8_t changeModeFlag = 0;  /*�������Ƿ�ı���ģʽ*/

static int16_t myLED_count = 0;


static void masterUARTInit(void)
{
	/*��������6,7,8ͨ����pwmƵ��Ϊ50Hz*/
	hal.rcout->set_freq( _BV(6), 50);  /*6=CH_7, ����6,7,8*/	


	hal.uartC->begin(115200, 8, 64);  /*baudRate, rxSpace, txSpace*/
	hal.scheduler->delay(2);
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
			}else{  /*֪ͨ�ӻ��л�ģʽ��ͬʱ��������ģʽ*/
				hal.uartC->write(0xFA);
				sendMsgFlag = 0;  /*���ٷ�����Ϣ���ӻ�*/
				changeModeFlag = 0;
			}

			setMyMotorOut();
			
			for(ix=0; ix<4; ix++){
				sendByte = (mymotor_out[ix]&0xFF00)>>8;/*ȡ��λ*/
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
		

	/*���δ����ķ����*/
	if(hal.uartC->available()>=1){
		hal.uartC->flush();
	}	

		/*rollͨ���������������, cdc test*/
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
ģʽ�л���������רΪ�̶���, ����ʱ��Ҫ���еĹ���:
1 �������û�����ã�ת��Ϊ������ģʽ��
2 ����ת���������
**************************************************/
static void change2PlaneMode(void)
{
	myflightMode = 2;
	motors.setup_motors();
	
	myLED_count = mainLoop_count;
}


/**************************************************
ģʽ�л�������ת��Ϊ4�ᣬ����ʱ��Ҫ���еĹ���:
1 �������û�����ã�ת��Ϊ����ģʽ��
2 ��������
3 ������Ϣ��ӻ�֪ͨ��ģʽת��
**************************************************/
static void change2QuadMode(void)
{	
	/*��д��AP_MotorsOcta.cpp�е�setup_motors����*/
	myflightMode = 1;
	motors.setup_motors();
	changeModeFlag = 1;	
	
	myLED_count = mainLoop_count;
}


#define CH_AILERON 5  /*����*/
#define CH_VERT    6  /*��ת���*/
#define CH_MAG     7  /*�����*/

/*�����Ͽ����ͨ�������������*/
#define AWAY_SERVO_OUT  1015
#define CNNT_SERVO_OUT  1941

/*��ת�������ֱλ�ú�ˮƽλ��ʱ�������������
��Ҫ����ת�Ǿݴ˼������*/
#define VERTICAL_SERVO_OUT 2078
#define HORIZON_SERVO_OUT  1237

/*����ͨ����δ����*/
#define AILERON_MID_OUT 1520

/*�����ʱ�����*/
static void sendServoLockOut()
{
	switch(myflightMode)
	{
	case 0:
		hal.rcout->write(CH_AILERON, AILERON_MID_OUT);
		hal.rcout->write(CH_VERT,    VERTICAL_SERVO_OUT);
		hal.rcout->write(CH_MAG,     CNNT_SERVO_OUT);	
		break;		
	case 1:
		hal.rcout->write(CH_AILERON, AILERON_MID_OUT);
		hal.rcout->write(CH_VERT,    VERTICAL_SERVO_OUT);
		hal.rcout->write(CH_MAG,     AWAY_SERVO_OUT);
		break;
	case 2: /*�ٶ���ʱ���������ȫ���룬����Ҫ�������, ����ͨ��ͨ��ң�����ı����ϵֱ�����*/
		hal.rcout->write(CH_VERT,    (VERTICAL_SERVO_OUT*2+HORIZON_SERVO_OUT)/3);//angle=30

		/*ֱ�ӽ�����ͨ������ת��Ϊ���*/
		hal.rcout->write(CH_AILERON, AILERON_MID_OUT + (g.rc_1.radio_in - g.rc_1.radio_trim)); 
		//hal.rcout->write(CH_MAG,     AWAY_SERVO_OUT);
		break;
	}
}

static void myLED_hint(void)
{
	static uint8_t flag = 0;
	if(mainLoop_count-myLED_count<300)  /*��˸����*/
	{
		flag = !flag;
		if (flag) {
		    digitalWriteFast(C_LED_PIN, LED_OFF);
		}else{
		    digitalWriteFast(C_LED_PIN, LED_ON);
		}
	}
}


#endif
