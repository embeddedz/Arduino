/*
    ModbusSerial.cpp - Source for Modbus Serial Library
    Copyright (C) 2014 André Sarmento Barbosa
*/
#include "QpackSerial.h"
#include "String.h"

extern "C" 
{
#include "crc16.h"
}


QpackSerial::QpackSerial() 
{
	_regs_head = 0;
    _regs_last = 0;
	_len = 0;
	
	packet_received = false;
	is_crc = false;
}

TRegister* QpackSerial::searchRegister(char* address) {
    TRegister *reg = _regs_head;
    //if there is no register configured, bail
    if(reg == 0) return(0);
    //scan through the linked list until the end of the list or the register is found.
    //return the pointer.
    do {
        if (!strcmp(reg->address,address)) return(reg);
        reg = reg->next;
	} while(reg);
	return(0);
}

void QpackSerial::addReg(char* address, word value, cb_cmd_handler cmd_func) {
    TRegister *newreg;

	newreg = (TRegister *) malloc(sizeof(TRegister));
	strcpy(newreg->address,address);
	newreg->value		= value;
	newreg->func		= cmd_func;
	newreg->next		= 0;

	if(_regs_head == 0) {
        _regs_head = newreg;
        _regs_last = _regs_head;
    } else {
        //Assign the last register's next pointer to newreg.
        _regs_last->next = newreg;
        //then make temp the last register in the list.
        _regs_last = newreg;
    }
}


byte ascii_u4_convt(byte ascii)
{
	char u4=0;
	if((ascii>='0') && (ascii<='9'))
		u4 = ascii - 48;
	else if((ascii>='A') && (ascii<='F'))
		u4 = ascii - 55;
	else if((ascii>='a') && (ascii<='f'))
		u4 = ascii - 87;
		
	return u4;
}

word asciiStr_hex_convt(byte* str,byte len)
{
	word hex=0;
	for(byte i=0; i<len; i++)
		hex |= ascii_u4_convt(str[i])<< (len-i-1)*4;
	
	return hex;	
}


byte u4_ascii_convt(byte u4)
{
	char ascii=0;
	if((u4>=0) && (u4<0xa))
		ascii = u4 + 48;
	else if((u4>=0xa) && (u4<=0xf))
		ascii = u4 + 55;
	else if((u4>=0xa) && (u4<=0xf))
		ascii = u4 + 87;	
	return ascii;
}

void hex_asciiStr_convt(byte* str,word val,byte len)
{
	byte i=0;
	for(; i<len; i++)
		str[i] = u4_ascii_convt((val>>(len - i -1)*4) & 0xf);
		
	str[i] = '\0';
}


bool QpackSerial::config(HardwareSerial* port, long baud, u_int format, char* name) {
    this->_port = port;
    (*port).begin(baud, format);
	while (!(*port));
    delay(2000);

	/*
    if (txPin >= 0) {
        pinMode(txPin, OUTPUT);
        digitalWrite(txPin, LOW);
    }*/

	strncpy(this->config_dvname,name,DEVICE_NAME_LEN);
    if (baud > 19200) {
        _t15 = 750;
        _t35 = 1750;
    } else {
        _t15 = 15000000/baud; // 1T * 1.5 = T1.5
        _t35 = 35000000/baud; // 1T * 3.5 = T3.5
    }
	
	printk("port configured");
  	printk("\n");
    return true;
}

int QpackSerial::receive_qpack_packet(byte* frame) 
{
    while ((*_port).available() > _len)	{
        _len = (*_port).available();
		printk("_len:");
    	printk(_len);
  		printk("\n");
        delayMicroseconds(_t15);
    }

    if (_len == 0) return 1;
	
    static byte i;
    for (i=0 ; i < _len ; i++) 
	{
		frame[i] = (*_port).read();
		printk(frame[i]);
		printk(" ");
	}
	printk("\n");

	if(frame[i-1] == '\r')
	{
		return 0;
	}

	_len = 0;
	printk("packet does not end with CR");
	printk("\n");	
    return 1;
}	

int QpackSerial::parse_qpack(byte* frame) 
{
	char *token=NULL;
	byte len=0;
	u_int crc=0,crc_local=0;
	int ret=0;
	
	//check if packet is valid
	if((_len<QP_MIN_PACKET_LEN) || (frame[0] != '$'))
	{
		ret |= QP_ERROR_INVALID_PACKET;
		goto parse_qpack_err; 
	}	
		
	token = strtok((char *)&frame[0],"*");	
	len = strlen(token);
	
	//check if CRC's included
	this->is_crc = false;
	if(len < _len)	//find '*' in packet, CRC is present
	{   
		if((_len - len) != (CRC_LEN +2)) 
		{								
			ret |= QP_ERROR_INVALID_PACKET;		//CRC must be 4 charaters long
			goto parse_qpack_err; 
		}	
    	this->is_crc = true;
  		crc = asciiStr_hex_convt(&frame[_len - 5],4);
		crc_local = crc_16(&frame[1],_len-7);
		printk("crc:");
		printk_fmt(crc,HEX);
		printk(",crc_local:");
		printk_fmt(crc_local,HEX);
		printk("\n");

		if (crc != crc_local) 
    	{
			ret |= QP_ERROR_CRC;
			goto parse_qpack_err; 
		}	
	}  

    //CRC Check
   

/*	
	token = strtok_r(&frame[1],",",last);		//copy the first part of packet, which is Host name
	if(!token)
		return QP_INVALID_PACKET;
	else if((len = strlen(token) != HOST_NAME_LEN)	//host name should be 4 letters long
		return QP_INVALID_PACKET;
	strcpy(host_nm,token);
	
	token = strtok(NULL,",",last);			//copy the second part of packet, which is device name
	if(!token)
		return QP_INVALID_PACKET;
	else if((len = strlen(token) != DEVICE_NAME_LEN)	//device name should be 4 letters long
		return QP_INVALID_PACKET;
	strcpy(device_nm,token);
	
	token = strtok(NULL,",",last);			//copy the third part of packet, which is reg addr
	if(!token)
		return QP_INVALID_PACKET;
	else if((len = strlen(token) != REG_NAME_LEN)	//register name should be 4 letters long
	{
		if(is_crc)								//if crc's present, delim should be '*'
		{
			sub_token = strtol(token,"*",sub_last);
			len = strlen(sub_token);
		}
		if(len == REG_NAME_LEN)					//this means right after reg field, comes CRC field, so it's a get cmd
		{
			is_setcmd = false;
			break;
		}
		return QP_INVALID_PACKET;
	}
	strcpy(reg,token);	
	
	token = strtok(NULL,"\r");			//copy the fouth part of packet, which is value, if it's a set cmd
	if(!token)
		return QP_INVALID_PACKET;
	else if((len = strlen(token) != REG_NAME_LEN)	//register name should be 4 letters long
		return QP_INVALID_PACKET;
	strcpy(reg,token);	
*/
	
	strncpy(this->host_nm,(const char*)&frame[1],4);		//copy host name field, 4 characters
	strncpy(this->device_nm,(const char*)&frame[6],4);		//copy device name field, 4 characters
	if(strcmp(this->device_nm,this->config_dvname))
	{
		ret |= QP_ERROR_DVNAME;
		goto parse_qpack_err; 
	}	
	strncpy(this->reg,(const char*)&frame[11],4);		//copy reg name field, 4 characters
	if(frame[15] == ',')		//value field comes after reg field
	{
		if(this->is_crc == true)
		{
			token = strtok((char *)&frame[16],"*");
		}
		else
		{
			token = strtok((char *)&frame[16],"\r");
		}	
		if(!token)
		{
			ret |= QP_ERROR_INVALID_PACKET;
			goto parse_qpack_err; 
		}
			
		strcpy(this->value,token);
		this->is_setcmd = true;
	}
	else if(frame[15] == '*')	//CRC field comes after reg field
	{
		//CRC field is already parsed
		this->is_setcmd = false;
	}
	else	//packet ends here
		this->is_setcmd = false;

parse_qpack_err:
	_len = 0;
	return ret;
}

int QpackSerial::exec_cmd(void) 
{	
	int ret=0;
	TRegister *reg=NULL;
	if((reg = this->searchRegister(this->reg)))
	{
		ret = reg->func(this->reg,this->value,this->is_setcmd);	
		return ret;		
	}	
	return QP_ERROR_INVALID_REG;
}



int QpackSerial::format_resp(int error,byte* frame)
{
	char length=0;
	word crc=0;

	memset(frame,0,QP_FRAME_LEN);
	strcpy((char *)frame,"$");
	strcat((char *)frame,this->config_dvname);
	strcat((char *)frame,",");
	strcat((char *)frame,this->host_nm);
	strcat((char *)frame,",");
	
	if(!error)
	{
		return 0;		//currently there's no response if no error
	}	
	else if((error&QP_ERROR_REG_MASK) == QP_ERROR_INVALID_REG)
	{
		strcat((char *)frame,"xreg");
	}
	else if((error&QP_ERROR_VALUE_MASK) == QP_ERROR_INVALID_VALUE)
	{
		strcat((char *)frame,"xval");
	}
	else if((error&QP_ERROR_CRC_MASK) == QP_ERROR_CRC)
	{
		strcat((char *)frame,"xcrc");
	}
		
	if(this->is_crc == true)
	{
		crc = crc_16(frame,strlen((const char*)frame));
		hex_asciiStr_convt((byte*)this->resp_crc,crc,CRC_LEN);
		strcat((char *)frame,"*");
		strcat((char *)frame,this->resp_crc);
		strcat((char *)frame,"\r");		
	}	

	length = strlen((const char*)frame);
	for(byte i=0;i<length;i++)
		_port->write(frame[i]);
	return 0;
}


void QpackSerial::task() 
{
	int ret=0;
	
	if(!this->receive_qpack_packet(this->_frame))
	{
		this->packet_received = false;
		if(!(ret |= this->parse_qpack(this->_frame)))
		{
			ret |= this->exec_cmd();				
		}	
		else
		{
	
		}
		printk("Qpack error:");
		printk(ret);
		printk("\n");
			
		this->format_resp(ret,this->_frame);	
	}
	//delay(100);
}





