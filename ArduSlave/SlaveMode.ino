/****************************************
      cdc  add for slave APM code
****************************************/
#include <MasterOrSlave.h>

#if MY_BOARD_MODE == SLAVE_MODE

#define MYMSG_RQ   0xA5     /*request for master msg*/
#define MYMSG_NACK 0xA7     /*nack master*/

static int16_t mymotor_out[4] = {0,0,0,0};

static int16_t myLED_count = 0;

static void slaveUARTInit(void)
{
	/*��������6,7,8ͨ����pwmƵ��Ϊ50Hz*/
	hal.rcout->set_freq( _BV(6), 50);  /*6=CH_7, ����6,7,8*/
	
	hal.uartC->begin(115200, 64, 8);  /*baudRate, rxSpace, txSpace*/
	hal.scheduler->delay(2);
	//hal.uartC->write(MYMSG_RQ);
	hal.uartA->printf("my Slave APM Init\n");
}

static void recvMasterMsg(void)
{
	static uint32_t preRecvTime = 0;
	
	uint8_t preRecvByte = 0;
	uint8_t thisRecvByte = 0;
		
	int16_t  recvNum=0;
	int16_t ix;	
	
	recvNum = hal.uartC->available();
	if(recvNum==9){		
		thisRecvByte = hal.uartC->read();

	//	hal.uartA->printf("msgHead=0x%x;", thisRecvByte);
		switch(thisRecvByte){
		case 0xFF:  /*cdc the motor out msg package*/
			for(ix=0; ix<4; ix++){
				preRecvByte = hal.uartC->read();
				thisRecvByte = hal.uartC->read();
				mymotor_out[ix] = int16_t((preRecvByte<<8)+thisRecvByte);

				/*give cmd to drive the four motors*/
				hal.rcout->write(ix, mymotor_out[ix]);

				/*cdc for Serial port print debug*/
				hal.uartA->printf("m%d=0x%x; ", ix+1, mymotor_out[ix]);
			}			
		//	hal.uartA->printf("\n");

			hal.uartC->write(MYMSG_RQ);
			
			/*cdc remember the last time received a whole package*/
			preRecvTime = hal.scheduler->millis();


			break;			
			
		case 0xFA: /*change the slave to a quad alone*/
			hal.uartC->flush();
			
			hal.uartA->printf("recv change mode msg ");
			
			change2QuadMode();

			break;			
		default:			
			/*wait for half a cycle and flush the UART to reconnect to master board;
			 when the master received the NACK, it do not answer the slave board*/
			hal.scheduler->delay(10);  /*100Hz~10ms*/
			hal.uartC->flush();	

			hal.uartC->write(MYMSG_NACK);
			break;
		}		
	}else if(recvNum>9){
		hal.scheduler->delay(10);  /*100Hz~10ms*/
		hal.uartC->flush();	
	}else{//recvNum<9
	/*cdc if the  slave board cann't recv a whole msg package in limited time;
       it send MYMSG_RQ to master board to request a msg */
		if((hal.scheduler->millis()-preRecvTime)>80){
			hal.uartC->flush();
			hal.uartC->write(MYMSG_RQ);
			preRecvTime = hal.scheduler->millis();
			hal.uartA->printf("rqst msg ");
		}		
	}
}


/**************************************************

**************************************************/
static void change2PlaneMode(void)
{
	myflightMode = 2;
	motors.setup_motors();
	
	myLED_count = mainLoop_count;
}


/**************************************************

**************************************************/
static void change2QuadMode(void)
{	
	/*��д��AP_MotorsOcta.cpp�е�setup_motors����*/
	myflightMode = 1;
	motors.setup_motors();

	
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
		hal.rcout->write(CH_AILERON, AILERON_MID_OUT - (g.rc_1.radio_in - g.rc_1.radio_trim)); 
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
