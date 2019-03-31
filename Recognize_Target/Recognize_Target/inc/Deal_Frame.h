#ifndef _DEAL_FRAME_H
#define _DEAL_FRAME_H
#include "common.h"
#define M_PI   3.14159265358979323846

class Deal_Frame{
	public:

		struct Ellipse_Coeff//椭圆方程的参数
		{
			double A;
			double B;
			double C;
			double F;
			double center_x;
			double center_y;
			double a_length;
			double b_length;
			double angle;
		};

		struct Targe_Para_Group//靶标参数
		{
			float x;
			float y;
			float e;
			float dir;
			float undulate_angle;//靶标起伏角度
			int center_x;
			int center_y;
			bool isGet_Red;//红色圆捕获标识
			bool isGet_Blue;//蓝色圆捕获标识
			float dis;
		}Targe_Para;
		Deal_Frame(){

			Targe_Para.x = Targe_Para.y = 0;
			Targe_Para.e = Targe_Para.dir = Targe_Para.undulate_angle = 0;
			Targe_Para.center_x = Targe_Para.center_y = 0;
			Targe_Para.isGet_Blue = Targe_Para.isGet_Red = false;
		};
		
		Mat calibration_camera(Mat img);

		//预处理图像,缩放成Imgsize大小,获得红色部分的灰度图像,获得蓝色部分的灰度图像
		bool Pre_DealFrame(Mat &Img, Mat &GrayForRedCircle, Mat &GrayForBlueCircle, int RowsSize = IMG_Row, int ColsSize = IMG_Col);

		//获取目标轮廓信息，参数为红色灰度图像
		RotatedRect Get_Target_Contour(Mat RedGrayImg, Mat Src);


		//计算离心率
		double CalcEccentricity(double a, double b);
		
		//j计算偏航角度
		double CalcAngle(Point2f center1, Point2f center2);

		//颜色过滤
		int Classify_Red(uchar const *SrcHweight, uchar const *SrcSweight, uchar const *SrcVweight,
			uchar *DirHweight, uchar *DirSweight, uchar *DirVweight);
		int Classify_Blue(uchar const *SrcHweight, uchar const *SrcSweight, uchar const *SrcVweight,
			uchar *Dirweight);

		//找到蓝色圆
		Point find_Blue_Circle(Mat Img);
		bool Filtet_Ellipse_Blue(Ellipse_Coeff Ellipse_CoeffGroup_Calc_Ellipse, vector<Point> Contours_temp_Calc_Ellipse, int s_of_img);
		
		//颜色分割（红，蓝）para1:带分割图像 para2:分割后的红色灰度图像 para3:分割后的蓝色灰度图像 para4:图像大小
		int SplitColorImg(vector<Mat> &InputImg, vector<Mat> &OutPutImg, Mat &Blue_Img, Size ImgSize);
		
		//获取椭圆方程系数
		Ellipse_Coeff GetCoeff(RotatedRect inputCoeff);

		//椭圆第一次过滤
		bool Filtet_Ellipse_1st(Ellipse_Coeff Ellipse_CoeffGroup_Calc_Ellipse, vector<Point> Contours_temp_Calc_Ellipse);
		//椭圆第二次过滤
		vector<cv::RotatedRect> Filtet_Ellipse_2st(vector<cv::RotatedRect> Contours_temp_Calc_2st);

		//canny算子的自适应阈值
		void AdaptiveFindThreshold(const Mat image, double *low, double *high, int aperture_size = 3);
		void _AdaptiveFindThreshold(CvMat *dx, CvMat *dy, double *low, double *high);
		
		//获取角度
		double Detect_Angle(RotatedRect Src, Mat Src_Img);
		
		//显示数据
		void ShowPos(Mat &Img, RotatedRect contourstemp, double dirangle);
	
};
extern Deal_Frame Camera_Deal_frame;
#endif