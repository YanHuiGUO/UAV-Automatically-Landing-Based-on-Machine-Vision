#pragma once  

#include <windows.h>  
#include <tchar.h>
#include <Deal_Frame.h>
#define COMPORT "COM5:"
#define Buad 115200
#define Frame_Len 25

#define TestComport 0 //0: �رմ��ڲ��� 1:�������ڲ���
typedef unsigned(__stdcall *PTHREEA_START) (void *);
class CSerial
{
public:
	CSerial(void);
	~CSerial(void);

	//�򿪴���  
	BOOL OpenSerialPort(TCHAR* port, UINT baud_rate, BYTE date_bits, BYTE stop_bit, BYTE parity = NOPARITY);

	//��������  
	BOOL SendData(byte* data, int len);

	//������֡
	void Generate_Frame(Deal_Frame p);
public:
	char data[Frame_Len];
	void floatToBytes(float p, byte* res);
	HANDLE m_hComm;
};
extern 	CSerial serial;