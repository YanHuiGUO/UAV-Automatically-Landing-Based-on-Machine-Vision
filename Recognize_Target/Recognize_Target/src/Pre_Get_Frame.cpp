#include <Pre_Get_Frame.h>
#include <Deal_Frame.h>
#include <math.h> 
#include <Serial.h>
//����С��λ�ĺ���
string do_fraction(double value, int decplaces = 16)
{
	ostringstream out;
	out.precision(decplaces);//����Ĭ�Ͼ���
	out << setiosflags(ios::fixed) << value;
	return out.str();
}
//��ȡ����ͷ
bool Pre_Frame::get_Camera(void){
	camera.open(0);
	if (!camera.isOpened())
	{
		cout << "����ͷ��ʧ��" << endl;
		return false;
	}

	return true;
}
//��ȡԭʼ֡�ź�
Mat Pre_Frame::getFrame(void){

	time_now = (double)getTickCount();
	camera >> frame;
	frame = Camera_Deal_frame.calibration_camera(frame);
	time_one_frame = (time_now - time_last) / getTickFrequency();
	fps = 1.0 / time_one_frame;
	string str = do_fraction(fps,2);
	fpsString = "FPS:";
	fpsString += str;                    // ��"FPS:"�����֡����ֵ�ַ���
	//// ��֡����Ϣд�����֡��
	//putText(frame,                  // ͼ�����
	//	fpsString,                  // string����������
	//	cv::Point(5, 20),           // �������꣬�����½�Ϊԭ��
	//	cv::FONT_HERSHEY_SIMPLEX,   // ��������
	//	0.5,                    // �����С
	//	cv::Scalar(0, 0, 0));           // ������ɫ

	//imshow("Camera FPS", frame);
	
	time_last = time_now;
	return frame;
}
//������Ҫ������
bool Pre_Frame::Cut_need_area(RotatedRect rect, Mat Src_Img, Mat &Dir_Img)
{

	Rect orig = Rect(0, 0, Src_Img.size().width, Src_Img.size().height);
	Rect roRect = rect.boundingRect(); //���ذ�����ת���ε���С����  
	Point seed;
	Rect region = roRect & orig;
	

	double Angle = 0;
	if (rect.size.area() > 0)
	{
		Dir_Img = Src_Img(region);
	}
	else 
		return false;

	return  true;
}
//������Ƶ���߳�
DWORD WINAPI  start_save_vedio(LPVOID lpParam){
	static unsigned long  time_cnt = 0;
	static int vedio_cnt = 1;
	Pre_Frame* p = (Pre_Frame*)lpParam;
	while (1){
		if (true == p->Deal_Complete)
		{
			//���ʹ�������
			
			//20ms���ҽ���һ��,����������¼��83������Ƶ
			if ((int)(time_cnt += 1) <= (int)(5000.0f/0.02f)){
				if (time_cnt % (int)(100.0f / 0.02f) == 0)
				{
					vedio_cnt++;
					p->writer.release(); p->writer;
					p->videoPath = "D:\\quater\\Recognize_Target\\vedio\\Video" + to_string(vedio_cnt) + ".avi";
					p->writer.open(p->videoPath, CV_FOURCC('X', 'V', 'I', 'D'), 25, Size(IMG_Col, IMG_Row), 1);
				}
				p->writer << p->save_img;	
			}
			
			p->Deal_Complete = false;
		}
	}
	return 0;
}

bool Pre_Frame::start_save_pic(string path, Mat img, int cnt){
	string Image_name;
#ifdef Write_Posive_Frame
	Image_name = path + "\\target_pos" + to_string(cnt) + ".jpg";
#endif
#ifdef Write_Negotive_Frame
	Image_name = path + "\\target_neg" + to_string(cnt) + ".jpg";
#endif

	if (imwrite(Image_name, img)) //����һ֡ͼƬ 
		return true;
	else return false;

}
