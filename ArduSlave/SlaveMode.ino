/****************************************
      cdc  add for slave APM code
****************************************/
#include <MasterOrSlave.h>

#if MY_BOARD_MODE == SLAVE_MODE

#define MYMSG_RQ   0xA5     /*request for master msg*/
#define MYMSG_NACK 0xA7     /*nack master*/

static int16_t mymotor_out[4] = {0,0,0,0};

static void slaveUARTInit(void)
{
	hal.uartC->begin(115200, 64, 8);  /*baudRate, rxSpace, txSpace*/
	hal.scheduler->delay(5);
	//hal.uartC->write(MYMSG_RQ);
	hal.uartA->printf("my Slave APM board Init\n");
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

		//hal.uartA->printf("msgHead=0x%x;", thisRecvByte);
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
			hal.uartA->printf("\n");

			hal.uartC->write(MYMSG_RQ);
			
			/*cdc remember the last time received a whole package*/
			preRecvTime = hal.scheduler->millis();


			break;			
		case 0xFE: /*cdc change the flight mode*/
			hal.uartC->flush();
			hal.uartC->write(MYMSG_RQ);
			
			preRecvTime = hal.scheduler->millis();
			break;
		case 0xFA: /*change the slave to a quad alone*/
			hal.uartC->flush();

			slaveMode = 0;
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
		}		
	}
}

#endif
