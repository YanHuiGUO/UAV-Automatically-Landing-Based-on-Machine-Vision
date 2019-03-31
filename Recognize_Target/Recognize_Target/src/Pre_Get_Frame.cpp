#include <Pre_Get_Frame.h>
#include <Deal_Frame.h>
#include <math.h> 
#include <Serial.h>
//保留小数位的函数
string do_fraction(double value, int decplaces = 16)
{
	ostringstream out;
	out.precision(decplaces);//覆盖默认精度
	out << setiosflags(ios::fixed) << value;
	return out.str();
}
//读取摄像头
bool Pre_Frame::get_Camera(void){
	camera.open(0);
	if (!camera.isOpened())
	{
		cout << "摄像头打开失败" << endl;
		return false;
	}

	return true;
}
//获取原始帧信号
Mat Pre_Frame::getFrame(void){

	time_now = (double)getTickCount();
	camera >> frame;
	frame = Camera_Deal_frame.calibration_camera(frame);
	time_one_frame = (time_now - time_last) / getTickFrequency();
	fps = 1.0 / time_one_frame;
	string str = do_fraction(fps,2);
	fpsString = "FPS:";
	fpsString += str;                    // 在"FPS:"后加入帧率数值字符串
	//// 将帧率信息写在输出帧上
	//putText(frame,                  // 图像矩阵
	//	fpsString,                  // string型文字内容
	//	cv::Point(5, 20),           // 文字坐标，以左下角为原点
	//	cv::FONT_HERSHEY_SIMPLEX,   // 字体类型
	//	0.5,                    // 字体大小
	//	cv::Scalar(0, 0, 0));           // 字体颜色

	//imshow("Camera FPS", frame);
	
	time_last = time_now;
	return frame;
}
//剪切需要的区域
bool Pre_Frame::Cut_need_area(RotatedRect rect, Mat Src_Img, Mat &Dir_Img)
{

	Rect orig = Rect(0, 0, Src_Img.size().width, Src_Img.size().height);
	Rect roRect = rect.boundingRect(); //返回包含旋转矩形的最小矩形  
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
//保存视频的线程
DWORD WINAPI  start_save_vedio(LPVOID lpParam){
	static unsigned long  time_cnt = 0;
	static int vedio_cnt = 1;
	Pre_Frame* p = (Pre_Frame*)lpParam;
	while (1){
		if (true == p->Deal_Complete)
		{
			//发送串口数据
			
			//20ms左右进来一次,所以这里是录制83分钟视频
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

	if (imwrite(Image_name, img)) //保存一帧图片 
		return true;
	else return false;

}
