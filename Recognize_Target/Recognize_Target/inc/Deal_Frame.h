#ifndef _DEAL_FRAME_H
#define _DEAL_FRAME_H
#include "common.h"
#define M_PI   3.14159265358979323846

class Deal_Frame{
	public:

		struct Ellipse_Coeff//��Բ���̵Ĳ���
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

		struct Targe_Para_Group//�б����
		{
			float x;
			float y;
			float e;
			float dir;
			float undulate_angle;//�б�����Ƕ�
			int center_x;
			int center_y;
			bool isGet_Red;//��ɫԲ�����ʶ
			bool isGet_Blue;//��ɫԲ�����ʶ
			float dis;
		}Targe_Para;
		Deal_Frame(){

			Targe_Para.x = Targe_Para.y = 0;
			Targe_Para.e = Targe_Para.dir = Targe_Para.undulate_angle = 0;
			Targe_Para.center_x = Targe_Para.center_y = 0;
			Targe_Para.isGet_Blue = Targe_Para.isGet_Red = false;
		};
		
		Mat calibration_camera(Mat img);

		//Ԥ����ͼ��,���ų�Imgsize��С,��ú�ɫ���ֵĻҶ�ͼ��,�����ɫ���ֵĻҶ�ͼ��
		bool Pre_DealFrame(Mat &Img, Mat &GrayForRedCircle, Mat &GrayForBlueCircle, int RowsSize = IMG_Row, int ColsSize = IMG_Col);

		//��ȡĿ��������Ϣ������Ϊ��ɫ�Ҷ�ͼ��
		RotatedRect Get_Target_Contour(Mat RedGrayImg, Mat Src);


		//����������
		double CalcEccentricity(double a, double b);
		
		//j����ƫ���Ƕ�
		double CalcAngle(Point2f center1, Point2f center2);

		//��ɫ����
		int Classify_Red(uchar const *SrcHweight, uchar const *SrcSweight, uchar const *SrcVweight,
			uchar *DirHweight, uchar *DirSweight, uchar *DirVweight);
		int Classify_Blue(uchar const *SrcHweight, uchar const *SrcSweight, uchar const *SrcVweight,
			uchar *Dirweight);

		//�ҵ���ɫԲ
		Point find_Blue_Circle(Mat Img);
		bool Filtet_Ellipse_Blue(Ellipse_Coeff Ellipse_CoeffGroup_Calc_Ellipse, vector<Point> Contours_temp_Calc_Ellipse, int s_of_img);
		
		//��ɫ�ָ�죬����para1:���ָ�ͼ�� para2:�ָ��ĺ�ɫ�Ҷ�ͼ�� para3:�ָ�����ɫ�Ҷ�ͼ�� para4:ͼ���С
		int SplitColorImg(vector<Mat> &InputImg, vector<Mat> &OutPutImg, Mat &Blue_Img, Size ImgSize);
		
		//��ȡ��Բ����ϵ��
		Ellipse_Coeff GetCoeff(RotatedRect inputCoeff);

		//��Բ��һ�ι���
		bool Filtet_Ellipse_1st(Ellipse_Coeff Ellipse_CoeffGroup_Calc_Ellipse, vector<Point> Contours_temp_Calc_Ellipse);
		//��Բ�ڶ��ι���
		vector<cv::RotatedRect> Filtet_Ellipse_2st(vector<cv::RotatedRect> Contours_temp_Calc_2st);

		//canny���ӵ�����Ӧ��ֵ
		void AdaptiveFindThreshold(const Mat image, double *low, double *high, int aperture_size = 3);
		void _AdaptiveFindThreshold(CvMat *dx, CvMat *dy, double *low, double *high);
		
		//��ȡ�Ƕ�
		double Detect_Angle(RotatedRect Src, Mat Src_Img);
		
		//��ʾ����
		void ShowPos(Mat &Img, RotatedRect contourstemp, double dirangle);
	
};
extern Deal_Frame Camera_Deal_frame;
#endif