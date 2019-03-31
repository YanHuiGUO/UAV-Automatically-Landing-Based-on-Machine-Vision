#pragma once  

#include <windows.h>  
#include <tchar.h>
#include <Deal_Frame.h>
#define COMPORT "COM5:"
#define Buad 115200
#define Frame_Len 25

#define TestComport 0 //0: 关闭串口测试 1:开启串口测试
typedef unsigned(__stdcall *PTHREEA_START) (void *);
class CSerial
{
public:
	CSerial(void);
	~CSerial(void);

	//打开串口  
	BOOL OpenSerialPort(TCHAR* port, UINT baud_rate, BYTE date_bits, BYTE stop_bit, BYTE parity = NOPARITY);

	//发送数据  
	BOOL SendData(byte* data, int len);

	//数据组帧
	void Generate_Frame(Deal_Frame p);
public:
	char data[Frame_Len];
	void floatToBytes(float p, byte* res);
	HANDLE m_hComm;
};
extern 	CSerial serial;