/*
    ModbusSerial.h - Header for ModbusSerial Library
    Copyright (C) 2014 André Sarmento Barbosa
*/
#include <Arduino.h>

#ifndef QPACKSERIAL_H
#define QPACKSERIAL_H

#define	QP_FRAME_LEN			128

#define QP_ERROR_INVALID_PACKET	1
#define QP_ERROR_CRC			1 << 1
#define QP_ERROR_DVNAME			1 << 2
#define QP_ERROR_INVALID_REG	1 << 3
#define QP_ERROR_INVALID_VALUE	1 << 4

#define QP_ERROR_CRC_MASK		1 << 1
#define QP_ERROR_REG_MASK		1 << 3
#define QP_ERROR_VALUE_MASK		1 << 4

#define QP_MIN_PACKET_LEN		16
#define QP_FIELD_LENGTH_MAX		10
#define HOST_NAME_LEN			4
#define DEVICE_NAME_LEN			4
#define REG_NAME_LEN			4
#define CRC_LEN					4

#define printk(x)	Serial.print(x)
#define printk_fmt(x,y) Serial.print(x,y)

typedef unsigned int u_int;
typedef int (*cb_cmd_handler)(char* raddr,char* cmd,bool is_set);

typedef struct TRegister {
    char address[10];
    word value;
	cb_cmd_handler func;
    struct TRegister* next;
} TRegister;


class QpackSerial {
    private:
		TRegister *_regs_head;
        TRegister *_regs_last;
		
        Stream* _port;
        long  _baud;
        u_int _format;
        int   _txPin;
		bool  is_setcmd;
		bool  is_crc;
		char  resp_crc[CRC_LEN+1];
		bool  packet_received;
		char  config_dvname[DEVICE_NAME_LEN+1];
		char  host_nm[HOST_NAME_LEN+1];
		char  device_nm[DEVICE_NAME_LEN+1];
		char  reg[REG_NAME_LEN+1];
		char  value[QP_FIELD_LENGTH_MAX+1];
        unsigned int _t15; // inter character time out
        unsigned int _t35; // frame delay
        
		TRegister* searchRegister(char* addr);
	protected:
        byte _frame[QP_FRAME_LEN];
        volatile byte  _len;

    public:
        QpackSerial();
		
        void addReg(char* address, word value, cb_cmd_handler cmd_func);
		
        bool config(HardwareSerial* port, long baud, u_int format, char* name);
        
        void task();
        int parse_qpack(byte* frame);
		int receive_qpack_packet(byte* frame);
		int exec_cmd(void);
		int format_resp(int error, byte* frame);
		
        
};

#endif //QPACKSERIAL_H
