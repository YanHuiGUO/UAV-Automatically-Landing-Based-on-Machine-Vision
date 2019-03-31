#include"StdAfx.h"  
#include"Serial.h"  
#include<process.h>  
CSerial serial;

CSerial::CSerial(void)
{
	m_hComm = INVALID_HANDLE_VALUE;
}

CSerial::~CSerial(void)
{
	if (m_hComm != INVALID_HANDLE_VALUE) {
		CloseHandle(m_hComm);
	}
}
//������תbytes
void  CSerial::floatToBytes(float p,byte *res){
	byte *t = (byte*)&p;

	for(int i = 0; i<4; i++)
		{
			res[i] = *t++;//����Ӧ��ַ�е����ݱ��浽char������       
		}

}
/*********************************************************************************************
* ����    ��    �������̻߳ص�����
* ����       �� �յ����ݺ󣬼򵥵���ʾ����
********************************************************************************************/	

DWORD WINAPI CommProc(LPVOID lpParam) {

	CSerial* pSerial = (CSerial*)lpParam;
	byte buf[512];
	//��մ���  
	PurgeComm(pSerial->m_hComm, PURGE_RXCLEAR | PURGE_TXCLEAR);


	DWORD dwRead;
	while (pSerial->m_hComm != INVALID_HANDLE_VALUE) {
		BOOL bReadOK = ReadFile(pSerial->m_hComm, buf, 512, &dwRead, NULL);
		if (bReadOK && (dwRead > 0)) {
			buf[dwRead] = '\0';

#if TestComport 
			//���Դ���������ȷ���
			if (buf[0] == 0xea){
				for (int i = 0; i < Frame_Len; i++)
					cout << hex << (unsigned int)buf[i] << ' ';
				cout << endl;
			}
#endif
		}
	}
	return 0;
}

/*******************************************************************************************
* ����     ��  �򿪴���
* port     :   ���ں�, ��_T("COM1:")
* baud_rate:   ������
* date_bits:  ����λ����Ч��Χ4~8��
* stop_bit :    ֹͣλ
* parity   : ��żУ�顣Ĭ��Ϊ��У�顣NOPARITY 0�� ODDPARITY 1��EVENPARITY 2��MARKPARITY 3��SPACEPARITY 4
********************************************************************************************/
BOOL CSerial::OpenSerialPort(TCHAR* port, UINT baud_rate, BYTE date_bits, BYTE stop_bit, BYTE parity)
{
	//�򿪴���  
	m_hComm = CreateFile(port, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);//��ռ��ʽ�򿪴���  

	TCHAR err[512];

	if (m_hComm == INVALID_HANDLE_VALUE) {
		_stprintf(err, _T("�򿪴���%s ʧ�ܣ���鿴�ô����Ƿ��ѱ�ռ��"), port);
		MessageBox(NULL, err, _T("��ʾ"), MB_OK);
		return FALSE;
	}

	//MessageBox(NULL,_T("�򿪳ɹ�"),_T("��ʾ"),MB_OK);  

	//��ȡ����Ĭ������  
	DCB dcb;
	if (!GetCommState(m_hComm, &dcb)) {
		MessageBox(NULL, _T("��ȡ���ڵ�ǰ���Բ���ʧ��"), _T("��ʾ"), MB_OK);
	}

	//���ô��ڲ���  
	dcb.BaudRate = baud_rate;  //������  
	dcb.fBinary = TRUE;            //������ģʽ������ΪTRUE  
	dcb.ByteSize = date_bits;  //����λ����Χ4-8  
	dcb.StopBits = ONESTOPBIT; //ֹͣλ  

	if (parity == NOPARITY) {
		dcb.fParity = FALSE;   //��żУ�顣����żУ��  
		dcb.Parity = parity;   //У��ģʽ������żУ��  
	}
	else {
		dcb.fParity = TRUE;        //��żУ�顣  
		dcb.Parity = parity;   //У��ģʽ������żУ��  
	}

	dcb.fOutxCtsFlow = FALSE;  //CTS���ϵ�Ӳ������  
	dcb.fOutxDsrFlow = FALSE;  //DST���ϵ�Ӳ������  
	dcb.fDtrControl = DTR_CONTROL_ENABLE;//DTR����  
	dcb.fDsrSensitivity = FALSE;
	dcb.fTXContinueOnXoff = FALSE;//  
	dcb.fOutX = FALSE;         //�Ƿ�ʹ��XON/XOFFЭ��  
	dcb.fInX = FALSE;          //�Ƿ�ʹ��XON/XOFFЭ��  
	dcb.fErrorChar = FALSE;        //�Ƿ�ʹ�÷��ʹ���Э��  
	dcb.fNull = FALSE;         //ͣ��null stripping  
	dcb.fRtsControl = RTS_CONTROL_ENABLE;//  
	dcb.fAbortOnError = FALSE; //���ڷ��ʹ��󣬲�����ֹ���ڶ�д  

	//���ô��ڲ���  
	if (!SetCommState(m_hComm, &dcb)) {
		MessageBox(NULL, _T("���ô��ڲ���ʧ��"), _T("��ʾ"), MB_OK);
		return FALSE;
	}

	//���ô����¼�  
	SetCommMask(m_hComm, EV_RXCHAR);//�ڻ��������ַ�ʱ�����¼�  
	SetupComm(m_hComm, 16384, 16384);

	//���ô��ڶ�дʱ��  
	COMMTIMEOUTS CommTimeOuts;
	GetCommTimeouts(m_hComm, &CommTimeOuts);
	CommTimeOuts.ReadIntervalTimeout = MAXDWORD;
	CommTimeOuts.ReadTotalTimeoutMultiplier = 0;
	CommTimeOuts.ReadTotalTimeoutConstant = 0;
	CommTimeOuts.WriteTotalTimeoutMultiplier = 10;
	CommTimeOuts.WriteTotalTimeoutConstant = 1000;

	if (!SetCommTimeouts(m_hComm, &CommTimeOuts)) {
		MessageBox(NULL, _T("���ô���ʱ��ʧ��"), _T("��ʾ"), MB_OK);
		return FALSE;
	}
#if TestComport 
	//�����̣߳���ȡ����  
	HANDLE hReadCommThread = (HANDLE)_beginthreadex(NULL, 0, (PTHREEA_START)CommProc, (LPVOID) this, 0, NULL);
#endif
	return TRUE;
}

/********************************************************************************************
* ����    ��    ͨ�����ڷ���һ������
********************************************************************************************/
BOOL CSerial::SendData(byte* data, int len) {
	if (m_hComm == INVALID_HANDLE_VALUE) {
		MessageBox(NULL, _T("����δ��"), _T("��ʾ"), MB_OK);
		return FALSE;
	}

	//��մ���  
	PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_TXCLEAR);

	//д����  
	DWORD dwWrite = 0;
	DWORD dwRet = WriteFile(m_hComm, data, len, &dwWrite, NULL);

	//��մ���  
	PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_TXCLEAR);

	if (!dwRet) {
		MessageBox(NULL, _T("��������ʧ��"), _T("��ʾ"), MB_OK);
		return FALSE;
	}
	return TRUE;
}

//֡ͷ+x�������� + y�������� + ������e + �����dir + ���Ƶľ���dis + �����Բ��־λ + ������Բ��־λ + ֡β
//25���ֽ�
//������֡������

void CSerial::Generate_Frame(Deal_Frame p){

		byte *generatepack = new byte[Frame_Len];
		generatepack[0] = (byte)0xea;
		generatepack[1] = (byte)Frame_Len;
		int index = 2;
		byte * temparr = new byte[4];

		floatToBytes(p.Targe_Para.center_x,temparr);

		generatepack[index] = temparr[0]; index++;
		generatepack[index] = temparr[1]; index++;
		generatepack[index] = temparr[2]; index++;
		generatepack[index] = temparr[3]; index++;


		floatToBytes(p.Targe_Para.center_y, temparr);

		generatepack[index] = temparr[0]; index++;
		generatepack[index] = temparr[1]; index++;
		generatepack[index] = temparr[2]; index++;
		generatepack[index] = temparr[3]; index++;

		floatToBytes(p.Targe_Para.e, temparr);

		generatepack[index] = temparr[0]; index++;
		generatepack[index] = temparr[1]; index++;
		generatepack[index] = temparr[2]; index++;
		generatepack[index] = temparr[3]; index++;


		floatToBytes(p.Targe_Para.dir, temparr);

		generatepack[index] = temparr[0]; index++;
		generatepack[index] = temparr[1]; index++;
		generatepack[index] = temparr[2]; index++;
		generatepack[index] = temparr[3]; index++;


		floatToBytes(p.Targe_Para.dis, temparr);

		generatepack[index] = temparr[0]; index++;
		generatepack[index] = temparr[1]; index++;
		generatepack[index] = temparr[2]; index++;
		generatepack[index] = temparr[3]; index++;


		generatepack[index] = p.Targe_Para.isGet_Red; index++;
		generatepack[index] = p.Targe_Para.isGet_Blue; index++;
		
		generatepack[index] = (generatepack[0] & generatepack[1] & generatepack[2] & generatepack[3]) + 0xaa;

		//����
		SendData(generatepack, Frame_Len);

	//cout << "x����Բ������";
	//cout << setw(-5) << p.Targe_Para.center_x << endl;
	//cout << "y����Բ������";
	//cout << setw(-5) << p.Targe_Para.center_y << endl;
	//cout << "������";
	//cout << setw(-5) << p.Targe_Para.e << endl;
	//cout << "�����";
	//cout << setw(-5) << p.Targe_Para.dir << endl;
	//cout << "�������";
	//cout << setw(-5) << p.Targe_Para.dis << endl;
	//cout << "��׽����ɫ�бꣿ";
	//cout << setw(-5) << p.Targe_Para.isGet_Red << endl;
	//cout << "��׽����ɫ�бꣿ";
	//cout << setw(-5) << p.Targe_Para.isGet_Blue << endl;


	
}