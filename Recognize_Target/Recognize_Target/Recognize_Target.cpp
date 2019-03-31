// Train_Data_get.cpp : 定义控制台应用程序的入口点。
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

	cout << "获取摄像头成功，开始预处理" << endl;

	

	while (!serial.OpenSerialPort(_T(COMPORT), Buad, 8, 1));  //打开串口后，自动接收数据  

	cout << "串口打开成功" << endl;

	////向串口发送数据  
	//byte data[100] = "1111111111111111112312312312111111111111111111112312312312312313123123312323\n";
	//if (serial.m_hComm == INVALID_HANDLE_VALUE) {
	//	MessageBox(NULL, _T("串口未打开"), _T("提示"), MB_OK);
	//	
	//}
	//else serial.SendData(data, sizeof(data)/sizeof(data[0]));

	//char aa[4];
	//serial.floatToBytes(0.1231f, aa);
	//float* pf = (float*)aa;

	//std::cout << (*pf) << std::endl;



	//创建视频线程
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
			//imshow("预处理的红色", img_red);
			RotatedRect rect = Camera_Deal_frame.Get_Target_Contour(img_red, frame);
			// 将帧率信息写在输出帧上
			putText(frame,                  // 图像矩阵
				Camera_frame.fpsString,                  // string型文字内容
				cv::Point(5, 20),           // 文字坐标，以左下角为原点
				cv::FONT_HERSHEY_SIMPLEX,   // 字体类型
				0.5,                    // 字体大小
				cv::Scalar(0, 0, 0));           // 字体颜色

			if (true == Camera_Deal_frame.Targe_Para.isGet_Red){
				double dir_angle = Camera_Deal_frame.Detect_Angle(rect, img_blue);
				Camera_Deal_frame.ShowPos(frame, rect, dir_angle);
				ellipse(frame, rect, CV_RGB(0, 255, 128), 2);
				putText(frame,                  // 图像矩阵
					"LOCK RED TARGET",
					cv::Point(150, 30),           // 文字坐标，以左下角为原点
					cv::FONT_HERSHEY_COMPLEX,   // 字体类型
					1,                    // 字体大小
					cv::Scalar(0, 255, 0));           // 字体颜色
				if (true == Camera_Deal_frame.Targe_Para.isGet_Blue)
					putText(frame,                  // 图像矩阵
					"LOCK BLUE TARGET",
					cv::Point(150, 60),           // 文字坐标，以左下角为原点
					cv::FONT_HERSHEY_COMPLEX,   // 字体类型
					1,                    // 字体大小
					cv::Scalar(0,255, 0));           // 字体颜色
				else putText(frame,                  // 图像矩阵
					"UNLOCK BLUE TARGET",
					cv::Point(150, 60),           // 文字坐标，以左下角为原点
					cv::FONT_HERSHEY_COMPLEX,   // 字体类型
					1,                    // 字体大小
					cv::Scalar(0, 0, 255));           // 字体颜色
			}
			else putText(frame,                  // 图像矩阵
				"UNLOCK RED TARGET",
				cv::Point(150, 30),           // 文字坐标，以左下角为原点
				cv::FONT_HERSHEY_COMPLEX,   // 字体类型
				1,                    // 字体大小
				cv::Scalar(0, 0, 255));           // 字体颜色
			
		}


	

		Camera_frame.save_img = frame;

		double time_last = (double)getTickCount();
		Camera_frame.time_deal_one_frame = (time_last - time_now) / getTickFrequency();

		//在写视频的线程中去发送数据了，在这里发串口数据会卡帧
		serial.Generate_Frame(Camera_Deal_frame);

		Camera_frame.Deal_Complete = true; // 标志位置true,线程开始写入当前帧图和发送串口数据


		cout <<"算法处理时间:"<< Camera_frame.time_deal_one_frame << endl;

		imshow("识别出靶标", frame);
		cvWaitKey(1);		
	}

	if (CloseHandle(serial.m_hComm))
		cout << "串口关闭" << endl;
	
	return 0;
}

