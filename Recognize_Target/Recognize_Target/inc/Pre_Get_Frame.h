#ifndef _PRE_GET_FRAME_H
#define _PRE_GET_FRAME_H
#include "common.h"
#include <process.h>  
#include <Serial.h>
//Ԥ����֡
extern  DWORD WINAPI start_save_vedio(LPVOID lpParam);
class Pre_Frame{
public:
	double time_start;
	double fps;
	string fpsString;
	string videoPath;
	VideoWriter writer;
	Mat save_img;
	bool Deal_Complete;
	double time_deal_one_frame;
	
	Pre_Frame() :
		time_deal_one_frame(0),
		time_one_frame(0),
		time_now(0),
		time_last(0),
		time_start(0),
		videoPath("D:\\quater\\Recognize_Target\\vedio\\Video.avi"),
		writer(videoPath, CV_FOURCC('X', 'V', 'I', 'D'), 25, Size(IMG_Col, IMG_Row), 1)
		{
			Deal_Complete = false;
		
		//	CloseHandle(hSaveVedioThread);
		};
	bool get_Camera(void);//��ȡ����ͷ
	Mat getFrame(void);//��ȡÿ֡ͼ��
	bool start_save_pic(string path, Mat img, int cnt);//����Ϊ·�����洢�˶���֡
	bool Cut_need_area(RotatedRect rect, Mat Src_Img,Mat& Dir_Img);//��ȡ���������Ǹ�����,����Ϊ�������������ؽ�ȡ����

private:
	Mat frame;

	VideoCapture camera;
	double time_one_frame;
	double time_now;
	double time_last;
	
};
extern Pre_Frame Camera_frame;

#endif
