// Train_Data_get.cpp : �������̨Ӧ�ó������ڵ㡣
//
#include <Pre_Get_Frame.h>
#include <Deal_Frame.h>
#include <Serial.h>
#include <process.h>  
Pre_Frame Camera_frame;
Deal_Frame Camera_Deal_frame;

int main()
{

	while (!Camera_frame.get_Camera());
	
	while (Camera_frame.getFrame().size().area()<=0);

	cout << "��ȡ����ͷ�ɹ�����ʼԤ����" << endl;

	

	while (!serial.OpenSerialPort(_T(COMPORT), Buad, 8, 1));  //�򿪴��ں��Զ���������  

	cout << "���ڴ򿪳ɹ�" << endl;

	////�򴮿ڷ�������  
	//byte data[100] = "1111111111111111112312312312111111111111111111112312312312312313123123312323\n";
	//if (serial.m_hComm == INVALID_HANDLE_VALUE) {
	//	MessageBox(NULL, _T("����δ��"), _T("��ʾ"), MB_OK);
	//	
	//}
	//else serial.SendData(data, sizeof(data)/sizeof(data[0]));

	//char aa[4];
	//serial.floatToBytes(0.1231f, aa);
	//float* pf = (float*)aa;

	//std::cout << (*pf) << std::endl;



	//������Ƶ�߳�
	HANDLE hSaveVedioThread = (HANDLE)_beginthreadex(NULL, 0, (PTHREEA_START)start_save_vedio, (LPVOID)&Camera_frame, 0, NULL);
	//CloseHandle(hSaveVedioThread);

	double time_deal_one_frame = 0;
	while (1){
		double time_now = (double)getTickCount();
		Mat frame;
		frame = Camera_frame.getFrame();

		Mat img_red, img_blue;
		img_red.create(IMG_Row, IMG_Col, CV_8UC1);
		img_blue.create(IMG_Row, IMG_Col, CV_8UC1);
		if (Camera_Deal_frame.Pre_DealFrame(frame, img_red, img_blue)){
			//imshow("Ԥ����ĺ�ɫ", img_red);
			RotatedRect rect = Camera_Deal_frame.Get_Target_Contour(img_red, frame);
			// ��֡����Ϣд�����֡��
			putText(frame,                  // ͼ�����
				Camera_frame.fpsString,                  // string����������
				cv::Point(5, 20),           // �������꣬�����½�Ϊԭ��
				cv::FONT_HERSHEY_SIMPLEX,   // ��������
				0.5,                    // �����С
				cv::Scalar(0, 0, 0));           // ������ɫ

			if (true == Camera_Deal_frame.Targe_Para.isGet_Red){
				double dir_angle = Camera_Deal_frame.Detect_Angle(rect, img_blue);
				Camera_Deal_frame.ShowPos(frame, rect, dir_angle);
				ellipse(frame, rect, CV_RGB(0, 255, 128), 2);
				putText(frame,                  // ͼ�����
					"LOCK RED TARGET",
					cv::Point(150, 30),           // �������꣬�����½�Ϊԭ��
					cv::FONT_HERSHEY_COMPLEX,   // ��������
					1,                    // �����С
					cv::Scalar(0, 255, 0));           // ������ɫ
				if (true == Camera_Deal_frame.Targe_Para.isGet_Blue)
					putText(frame,                  // ͼ�����
					"LOCK BLUE TARGET",
					cv::Point(150, 60),           // �������꣬�����½�Ϊԭ��
					cv::FONT_HERSHEY_COMPLEX,   // ��������
					1,                    // �����С
					cv::Scalar(0,255, 0));           // ������ɫ
				else putText(frame,                  // ͼ�����
					"UNLOCK BLUE TARGET",
					cv::Point(150, 60),           // �������꣬�����½�Ϊԭ��
					cv::FONT_HERSHEY_COMPLEX,   // ��������
					1,                    // �����С
					cv::Scalar(0, 0, 255));           // ������ɫ
			}
			else putText(frame,                  // ͼ�����
				"UNLOCK RED TARGET",
				cv::Point(150, 30),           // �������꣬�����½�Ϊԭ��
				cv::FONT_HERSHEY_COMPLEX,   // ��������
				1,                    // �����С
				cv::Scalar(0, 0, 255));           // ������ɫ
			
		}


	

		Camera_frame.save_img = frame;

		double time_last = (double)getTickCount();
		Camera_frame.time_deal_one_frame = (time_last - time_now) / getTickFrequency();

		//��д��Ƶ���߳���ȥ���������ˣ������﷢�������ݻῨ֡
		serial.Generate_Frame(Camera_Deal_frame);

		Camera_frame.Deal_Complete = true; // ��־λ��true,�߳̿�ʼд�뵱ǰ֡ͼ�ͷ��ʹ�������


		cout <<"�㷨����ʱ��:"<< Camera_frame.time_deal_one_frame << endl;

		imshow("ʶ����б�", frame);
		cvWaitKey(1);		
	}

	if (CloseHandle(serial.m_hComm))
		cout << "���ڹر�" << endl;
	
	return 0;
}

