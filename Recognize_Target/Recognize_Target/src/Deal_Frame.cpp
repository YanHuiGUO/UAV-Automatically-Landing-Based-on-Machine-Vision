#include <Deal_Frame.h>
//靶标识别类
//bug记录：
//1:蓝圆的识别没有筛选，导致在没有蓝色圆的情况下，也会出现计算dir信息的情况
//2:截取蓝色圆区域的代码要重构，有些问题

Mat Deal_Frame::calibration_camera(Mat img){
	/*内参数*/
	Mat cameraMatrix = (Mat_<double>(3, 3) << 575.1911057627621, 0, 320.9206237613332,
		0, 571.7041902411318, 226.7566459326267,
		0, 0, 1);

	/*畸变参数*/
	Mat distCoeffs = (Mat_<double>(1, 5) << -0.4382279001714561, 0.2634996843856358, -0.000386825855848607, 0.0004513359699597199, -0.09142447090044117);

	Mat mapx = Mat(img.size(), CV_32FC1);
	Mat mapy = Mat(img.size(), CV_32FC1);
	Mat R = Mat::eye(3, 3, CV_32F);
	Mat newimage;
	initUndistortRectifyMap(cameraMatrix, distCoeffs, R, cameraMatrix, img.size(), CV_32FC1, mapx, mapy);
	remap(img, newimage, mapx, mapy, INTER_LINEAR);

	//undistort(img, newimage, cameraMatrix, distCoeffs);
	return newimage;
}

//计算Canny自适应阈值
void Deal_Frame::_AdaptiveFindThreshold(CvMat *dx, CvMat *dy, double *low, double *high)
{
	CvSize size;
	IplImage *imge = 0;
	int i, j;
	CvHistogram *hist;
	int hist_size = 255;
	float range_0[] = { 0, 256 };
	float* ranges[] = { range_0 };
	double PercentOfPixelsNotEdges = 0.7;
	size = cvGetSize(dx);
	imge = cvCreateImage(size, IPL_DEPTH_32F, 1);
	// 计算边缘的强度, 并存于图像中                                          
	float maxv = 0;
	for (i = 0; i < size.height; i++)
	{
		const short* _dx = (short*)(dx->data.ptr + dx->step*i);
		const short* _dy = (short*)(dy->data.ptr + dy->step*i);
		float* _image = (float *)(imge->imageData + imge->widthStep*i);
		for (j = 0; j < size.width; j++)
		{
			_image[j] = (float)(abs(_dx[j]) + abs(_dy[j]));
			maxv = maxv < _image[j] ? _image[j] : maxv;

		}
	}
	if (maxv == 0){
		*high = 0;
		*low = 0;
		cvReleaseImage(&imge);
		return;
	}

	// 计算直方图                                                            
	range_0[1] = maxv;
	hist_size = (int)(hist_size > maxv ? maxv : hist_size);
	hist = cvCreateHist(1, &hist_size, CV_HIST_ARRAY, ranges, 1);
	cvCalcHist(&imge, hist, 0, NULL);
	int total = (int)(size.height * size.width * PercentOfPixelsNotEdges);
	float sum = 0;
	int icount = hist->mat.dim[0].size;

	float *h = (float*)cvPtr1D(hist->bins, 0);
	for (i = 0; i < icount; i++)
	{
		sum += h[i];
		if (sum > total)
			break;
	}
	// 计算高低门限                                                          
	*high = (i + 1) * maxv / hist_size;
	*low = *high * 0.4;
	cvReleaseImage(&imge);
	cvReleaseHist(&hist);
}

void Deal_Frame::AdaptiveFindThreshold(const Mat image, double *low, double *high, int aperture_size )
{

	const int cn = image.channels();
	cv::Mat dx(image.rows, image.cols, CV_16SC(cn));
	cv::Mat dy(image.rows, image.cols, CV_16SC(cn));

	cv::Sobel(image, dx, CV_16S, 1, 0, aperture_size, 1, 0);
	cv::Sobel(image, dy, CV_16S, 0, 1, aperture_size, 1, 0);

	CvMat _dx = dx, _dy = dy;
	_AdaptiveFindThreshold(&_dx, &_dy, low, high);

}

//计算离心率
double Deal_Frame::CalcEccentricity(double a, double b)
{
	double Eccentricity = 0;

	if (a> b)
		Eccentricity = (double)sqrtf(1 - pow(b / a, 2));
	else if (a <= b)
		Eccentricity = (double)sqrtf(1 - pow(a / b, 2));

	return Eccentricity;
}

//回去椭圆方程参数
Deal_Frame::Ellipse_Coeff Deal_Frame::GetCoeff(RotatedRect inputCoeff)
{
	Ellipse_Coeff Coeff_temp;
	Coeff_temp.center_x = inputCoeff.center.x;
	Coeff_temp.center_y = inputCoeff.center.y;
	Coeff_temp.a_length = inputCoeff.size.height / 2;
	Coeff_temp.b_length = inputCoeff.size.width / 2;
	Coeff_temp.angle = 90 - inputCoeff.angle;
	return Coeff_temp;
}

//************************蓝色圆处理区*****************************//
//蓝色圆处理时的椭圆过滤
bool Deal_Frame::Filtet_Ellipse_Blue(Ellipse_Coeff Ellipse_CoeffGroup_Calc_Ellipse, vector<Point> Contours_temp_Calc_Ellipse, int s_of_img)
{
	//x′ = x−x0  y′ = y−y0 
	//Ax′2 + Bx′y′ + Cy′2 + f = 0

	double result = 0, x_ = 0, y_ = 0;
	double dis = 0;
	unsigned int countPix = 0;
	double rate = 0;
	double s = 0;
	double e = 0;
	for (size_t k = 0; k < Contours_temp_Calc_Ellipse.size(); k++)
	{
		x_ = Contours_temp_Calc_Ellipse[k].x - Ellipse_CoeffGroup_Calc_Ellipse.center_x;
		y_ = Contours_temp_Calc_Ellipse[k].y - Ellipse_CoeffGroup_Calc_Ellipse.center_y;
		dis = sqrtf(x_*x_ + y_*y_);

		if (dis <= Ellipse_CoeffGroup_Calc_Ellipse.a_length*1.05 && dis >= Ellipse_CoeffGroup_Calc_Ellipse.b_length*0.95)
			countPix++;
	}

	rate = (double)countPix / Contours_temp_Calc_Ellipse.size();
	s = M_PI*Ellipse_CoeffGroup_Calc_Ellipse.a_length * Ellipse_CoeffGroup_Calc_Ellipse.b_length;
	e = CalcEccentricity(Ellipse_CoeffGroup_Calc_Ellipse.a_length, Ellipse_CoeffGroup_Calc_Ellipse.b_length);
	//angle值已做90-处理，修复OPENCV BUG
	//在本方案中，椭圆的这个angle值反映了椭圆所在平台相对于地面的平坦程度

	//这个rate值对遮挡很敏感，当调高这个阈值时，遮挡一部分的时候效果不好，调低的时候遮挡效果也很差
	//e这个值比较重要,当船颠簸过大的e就比较大，我们应该不使用这样的蓝色圆，同时也可以起到过滤杂圆的效果,
	//因为蓝色圆的轮廓线较小，所以rate的过滤作用没有红色圆的rate过滤作用大
	if ((rate - 0.7)>0 && s >= 25 && s <= s_of_img * 0.3&& e<0.5)
		return true;

	return false;
}

//捕获蓝色圆
Point Deal_Frame::find_Blue_Circle(Mat Img)
{
	Mat Tmep = Img.clone();;
	Point result;
	//高斯滤波
	GaussianBlur(Tmep, Tmep, Size(3, 3), 0, 0);
	medianBlur(Tmep, Tmep, 3);
	//imshow("原始蓝色", Img);

	threshold(Tmep, Tmep, 50, 0, THRESH_TOZERO);
	threshold(Tmep, Tmep, 0, 255, THRESH_OTSU);


	double low = 0.0, high = 0.0;
	AdaptiveFindThreshold(Tmep, &low, &high);

	Mat element = getStructuringElement(MORPH_ELLIPSE, Size(9, 9));

	morphologyEx(Tmep, Tmep, MORPH_CLOSE, element);


	Canny(Tmep, Tmep, low, high, 3);
	//imshow("捕获的蓝色", Tmep);
	//	imshow("捕获的轮廓", Tmep);
	vector<vector<Point> > contours;

	vector<vector<Point>> contours_temp;//存过滤后的原始轮廓
	vector<Vec4i> hierarchy;


	findContours(Tmep, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	vector<cv::RotatedRect> contours01; // 存椭圆过滤后的的椭圆方程参数
	vector<cv::RotatedRect> contours02;

	for (size_t k = 0; k < contours.size(); k++)
	{
		if (contours[k].size() > 5){ //使用前一定要判断是否大于5个点,小于5个点要报错
			contours01.push_back(fitEllipse(contours[k]));
			contours_temp.push_back(contours[k]);
		}
	}

	Ellipse_Coeff Ellipse_CoeffGroup;
	for (size_t k = 0; k < contours01.size(); k++)
	{
		bool GetEllpises_1st;
		Ellipse_CoeffGroup = (GetCoeff(contours01.at(k)));
		GetEllpises_1st = Filtet_Ellipse_Blue(Ellipse_CoeffGroup, contours_temp[k], Img.size().height*Img.size().width);
		if (GetEllpises_1st)
		{
			contours02.push_back(contours01.at(k));

		}
	}

	RotatedRect fitellipse_Blue;
	if (contours02.size() != 0)
	{
		fitellipse_Blue = contours02[0];

		for (size_t k = 1; k < contours02.size(); k++)
		{
			double s1 = (double)fitellipse_Blue.size.height * fitellipse_Blue.size.width / 4;
			double s2 = (double)contours02[k].size.height * contours02[k].size.width / 4;
			float a = fitellipse_Blue.size.width / 2;
			float b = fitellipse_Blue.size.height / 2;
			double e1 = CalcEccentricity(a, b);
			a = contours02[k].size.width / 2;
			b = contours02[k].size.height / 2;
			double e2 = CalcEccentricity(a, b);

			if (s2>s1)
				fitellipse_Blue = contours02[k];
		}
	}

	ellipse(Img, fitellipse_Blue, 255, 2);

	if (fitellipse_Blue.size.area() != 0)
	{
		Targe_Para.isGet_Blue = true;
		result.x = fitellipse_Blue.center.x;
		result.y = fitellipse_Blue.center.y;
	}
	else
	{
		Targe_Para.isGet_Blue = false;
	}
	//绘制在图形上
	//ellipse(Tmep, fitellipse_Blue, CV_RGB(255, 255,255), 2);
	imshow("找到蓝色", Img);
	return result;
}

//计算偏航角度 para1:红圆圆心 para2:蓝圆圆心 返回偏航角度信息
//偏航角范围,左:0到-180, 右:0到+180 
double  Deal_Frame::CalcAngle(Point2f center1, Point2f center2)
{
	double result;
	if (center1.y < center2.y) result = atanf((center2.x - center1.x) / (center2.y - center1.y)) / M_PI * 180;
	else if (center1.y > center2.y)
	{
		if (center2.x == center1.x)
			return 180;
		center2.x - center1.x > 0 ? result = 180 - (atanf((center2.x - center1.x) / -(center2.y - center1.y))) / M_PI * 180
			: result = -180 + (atanf((center2.x - center1.x) / (center2.y - center1.y))) / M_PI * 180;
	}
	else
	{
		result = center2.x - center1.x > 0 ? 90 : -90;
	}
	return result;
}


//截取蓝色圆区域并识别角度
double Deal_Frame::Detect_Angle(RotatedRect Src, Mat Src_Img)
{

	Rect orig = Rect(0, 0, Src_Img.size().width, Src_Img.size().height);
	Rect roRect = Src.boundingRect(); //返回包含旋转矩形的最小矩形  
	Point seed;
	Rect region = roRect & orig;
	Point targer_orig_point = region.tl(); // 计算交叉点再原图像坐标系中的坐标
	Mat GetAngleIMG;
	double Angle = 0;
	if (region.area() > 0)
	{
		GetAngleIMG = Src_Img(region);
	}
	else
		return 0;

	if (GetAngleIMG.size().height>0 && GetAngleIMG.size().width>0)
	{
		Point2f CirCle_Red;
		Point2f CirCle_Blue;
		CirCle_Red.y = (double)Src.center.y;
		CirCle_Red.x = (double)Src.center.x;

		CirCle_Blue = find_Blue_Circle(GetAngleIMG);

		//当检测到了蓝色圆
		if ( true ==Targe_Para.isGet_Blue){
			//这里对截取的图像区域做个坐标系变化到原图像坐标系中
			CirCle_Blue.x += targer_orig_point.x;
			CirCle_Blue.y += targer_orig_point.y;
			Angle = CalcAngle(CirCle_Red, CirCle_Blue);
		}
	}
	else Angle = 0;

	return  Angle;
}

//****************************************************************//


//************************红色圆处理区****************************//


//	   黑   灰   白    红     橙    黄   绿   青    蓝   紫
//hmin 0    0     0   0/156   11    26   35    78  100   125 
//hmax 180 180   180 10/180   25    34   77    99  124   155 
//smin 0	 0	  0    43     43    43   43    43   43   43 
//smax 255  43   30    255    255   255  255   255  255  255
//vmin 0    46   221   46     46    46   46    46   46   46 
//vmax 46   220  255   255    255   255  255   255  255  255
//放开一点红色阈值范围
#define JudgeRed ((*SrcHweight >= 0 && *SrcHweight <= 20) || (*SrcHweight >= 155 && *SrcHweight <= 180))
#define JudgeBlue ((*SrcHweight >= 100 && *SrcHweight <= 130))
int Deal_Frame::Classify_Red(uchar const *SrcHweight, uchar const *SrcSweight, uchar const *SrcVweight,
	uchar *DirHweight, uchar *DirSweight, uchar *DirVweight)
{
	if (JudgeRed)
	{
		*DirHweight = *SrcHweight;
		*DirSweight = *SrcSweight;
		*DirVweight = *SrcVweight;
	}
	else
	{
		*DirHweight = 0;
		*DirSweight = 0;
		*DirVweight = 255;
	}
	return 0;
}
int Deal_Frame::Classify_Blue(uchar const *SrcHweight, uchar const *SrcSweight, uchar const *SrcVweight,
	uchar *Dirweight)
{
	if (JudgeBlue)
	{
		*Dirweight = *SrcSweight;
	}
	else
	{
		*Dirweight = 0;

	}
	return 0;
}
//分离各个颜色成独立矩阵,这里同时划分了红色和蓝色颜色区域，方便后面处理蓝色圆，直接使用Blue_Img
int Deal_Frame::SplitColorImg(vector<Mat> &InputImg, vector<Mat> &OutPutImg, Mat &Blue_Img, Size ImgSize)
{


	//遍历像素
	for (int row = 0; row < ImgSize.height; row++)
	{
		uchar *PointInputImg_H = InputImg.at(0).ptr<uchar>(row);
		uchar *PointInputImg_S = InputImg.at(1).ptr<uchar>(row);
		uchar *PointInputImg_V = InputImg.at(2).ptr<uchar>(row);
		uchar *PointOutPutImg_H = OutPutImg.at(0).ptr<uchar>(row);
		uchar *PointOutPutImg_S = OutPutImg.at(1).ptr<uchar>(row);
		uchar *PointOutPutImg_V = OutPutImg.at(2).ptr<uchar>(row);
		uchar* PointgetBlue = Blue_Img.ptr<uchar>(row);
		//根据H分量做颜色划分
		for (int col = 0; col < ImgSize.width; col++)
		{
			Classify_Red(PointInputImg_H + col, PointInputImg_S + col, PointInputImg_V + col, PointOutPutImg_H + col, PointOutPutImg_S + col, PointOutPutImg_V + col);
			Classify_Blue(PointInputImg_H + col, PointInputImg_S + col, PointInputImg_V + col, PointgetBlue + col);
		}

	}

	return 0;
}

//第一次椭圆过滤,计算拟合误差，误差大的轮廓过滤掉
bool Deal_Frame::Filtet_Ellipse_1st(Ellipse_Coeff Ellipse_CoeffGroup_Calc_Ellipse, vector<Point> Contours_temp_Calc_Ellipse)
{
	//x′ = x−x0  y′ = y−y0 
	//Ax′2 + Bx′y′ + Cy′2 + f = 0

	double result = 0, x_ = 0, y_ = 0;
	double dis = 0;
	unsigned int countPix = 0;
	double rate = 0;
	double s = 0;
	double e = 0;
	for (size_t k = 0; k < Contours_temp_Calc_Ellipse.size(); k++)
	{
		x_ = Contours_temp_Calc_Ellipse[k].x - Ellipse_CoeffGroup_Calc_Ellipse.center_x;
		y_ = Contours_temp_Calc_Ellipse[k].y - Ellipse_CoeffGroup_Calc_Ellipse.center_y;
		dis = sqrtf(x_*x_ + y_*y_);

		if (dis <= Ellipse_CoeffGroup_Calc_Ellipse.a_length*1.05 && dis >= Ellipse_CoeffGroup_Calc_Ellipse.b_length*0.95)
			countPix++;
	}

	rate = (double)countPix / Contours_temp_Calc_Ellipse.size();
	s = M_PI*Ellipse_CoeffGroup_Calc_Ellipse.a_length * Ellipse_CoeffGroup_Calc_Ellipse.b_length;
	e = CalcEccentricity(Ellipse_CoeffGroup_Calc_Ellipse.a_length, Ellipse_CoeffGroup_Calc_Ellipse.b_length);
	//angle值已做90-处理，修复OPENCV BUG
	//在本方案中，椭圆的这个angle值反映了椭圆所在平台相对于地面的平坦程度
	//如果考虑到船的颠簸起伏，这个值要放宽一些。
	//if (rate > 0.85 &&s<76800 * 0.95&& abs(Ellipse_CoeffGroup_Calc_Ellipse.angle)<5 && e<0.93
	if (rate >= 0.965 &&s<IMG_Col*IMG_Row * 0.95 && s >= IMG_Col*IMG_Row*0.0001)
		return true;

	return false;
}

//第二次椭圆过滤
vector<cv::RotatedRect> Deal_Frame::Filtet_Ellipse_2st(vector<cv::RotatedRect> Contours_temp_Calc_2st)
{
	RotatedRect Temp;
	vector<RotatedRect> result;
	size_t n = Contours_temp_Calc_2st.size();
	if (n >= 2){
		for (size_t k = 0; k < n; k++)
		{
			Temp = Contours_temp_Calc_2st[k];
			int cnt = 1;
			for (size_t m = 0; m < n; m++){
				if (k != m){
					double x = Temp.center.x - Contours_temp_Calc_2st[m].center.x;
					double y = Temp.center.y - Contours_temp_Calc_2st[m].center.y;
					double angletmep = Temp.angle - Contours_temp_Calc_2st[m].angle;
					double dis = sqrtf(x*x + y*y);
					double e1 = CalcEccentricity(Temp.size.height / 2, Temp.size.width / 2);
					double e2 = CalcEccentricity(Contours_temp_Calc_2st[m].size.height / 2, Contours_temp_Calc_2st[m].size.width / 2);
					if (dis < 50){
						//修改这个地方的参数决定了筛选杂圆的晒选率，如果严格了则会出现遮挡时效果不好的情况
						cnt++;
					}
				}
			}
			if (cnt >= 3)
				result.push_back(Temp);
		}
	}

	return result;
}

//****************************************************************//
//显示数据到图像上
//赋值到Target_Para包中
void Deal_Frame::ShowPos(Mat &Img, RotatedRect contourstemp, double dirangle)
{
	stringstream position_x_temp, position_y_temp, angle_temp, Eccen_Temp, dirangle_temp;
	string position_x, position_y, angle, Eccen, dirangle_temp_s;
	position_x_temp << (int)contourstemp.center.x;
	position_x = "x:" + position_x_temp.str();

	position_y_temp << (int)contourstemp.center.y;
	position_y = "y:" + position_y_temp.str();

	angle_temp << fixed << setprecision(2) << 90.0f - contourstemp.angle;
	angle = "angle:" + angle_temp.str();


	dirangle_temp << fixed << setprecision(2) << (double)dirangle;
	dirangle_temp_s = "Dir:" + dirangle_temp.str();

	float Eccentricity;
	float a = contourstemp.size.width / 2;
	float b = contourstemp.size.height / 2;
	Eccentricity = CalcEccentricity(a, b);

	Eccen_Temp  <<fixed << setprecision(2) << Eccentricity;
	Eccen = "e:" + Eccen_Temp.str();

	putText(Img, position_x, Point(0, 40), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1.5, 8);
	putText(Img, position_y, Point(0, 60), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1.5, 8);
	putText(Img, Eccen, Point(0, 80), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1.5, 8);
	putText(Img, dirangle_temp_s, Point(0, 100), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1.5, 8);
	putText(Img, angle, Point(0, 120), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1.5, 8);


	position_x_temp>>Targe_Para.center_x;
	position_y_temp>>Targe_Para.center_y;
	dirangle_temp >> Targe_Para.dir;
	Eccen_Temp>>Targe_Para.e;


}

//预处理图像，得到红圆的灰度轮廓和蓝色部分的灰度图像
bool Deal_Frame::Pre_DealFrame(Mat &Img, Mat &GrayForRedCircle, Mat &GrayForBlueCircle, int RowsSize, int ColsSize){
	Mat MatImgSrc;
	//原图的大小为640*480，注意在OPENCV里面反过来了,读入的图像到Mat是480*640,既rows=480,cols=640，而在Size中是640*480
	Mat hsv = Mat::zeros(RowsSize, ColsSize, CV_8UC3);

	resize(Img, Img, Size(ColsSize, RowsSize), 0, 0, CV_INTER_CUBIC);
	MatImgSrc = Img.clone();
	//imshow("原图", MatImgSrc);
	double time0 = static_cast<double>(getTickCount());    //起始时间
	cvtColor(MatImgSrc, hsv, CV_BGR2HSV);
	vector<Mat> MatHsvSplit;
	vector<Mat> MatHsvOut(hsv.channels());
	//creat函数是行列参数是反的
	MatHsvOut.at(0).create(RowsSize, ColsSize, CV_8UC1);
	MatHsvOut.at(1).create(RowsSize, ColsSize, CV_8UC1);
	MatHsvOut.at(2).create(RowsSize, ColsSize, CV_8UC1);

	split(hsv, MatHsvSplit);
	Mat GrayForTrigle = Mat::zeros(RowsSize, ColsSize, CV_8UC1);
	SplitColorImg(MatHsvSplit, MatHsvOut, GrayForBlueCircle, MatHsvOut.at(0).size());

	GrayForRedCircle = MatHsvOut.at(1);
	
	//高斯滤波
	GaussianBlur(GrayForRedCircle, GrayForRedCircle, Size(3, 3), 0, 0);
	//中值滤波
	medianBlur(GrayForRedCircle, GrayForRedCircle, 3);

	//对颜色分割后的灰度图像进行阈值，这里对处理之后对人的皮肤和靶标的区分的不太好，参数设得太大的话靶标倾斜一定角度时会漏检测
	//如果实际环境中不存在这种无法区分的情况那么本算法即可使用，否则的话要基于学习的方法来试试
	threshold(GrayForRedCircle, GrayForRedCircle, 80, 0, THRESH_TOZERO);
	threshold(GrayForBlueCircle, GrayForBlueCircle, 50, 0, THRESH_TOZERO);
	//imshow("分割的蓝色部分", GrayForTrigle);

	//最后大律法阈值一次减少后面提取轮廓时的噪点
	threshold(GrayForRedCircle, GrayForRedCircle, 0, 255, THRESH_OTSU);
	//imshow("阈值过后的红色区域的图像", GrayForRedCircle);

	//备份一个灰度图给三角形检测用


	//canny自适应阈值
	double low = 0.0, high = 0.0;
	AdaptiveFindThreshold(GrayForRedCircle, &low, &high);
	
	//形态学处理
	Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
	morphologyEx(GrayForRedCircle, GrayForRedCircle, MORPH_CLOSE, element);

	//imshow("形态学处理后的图像", Gray);
	Canny(GrayForRedCircle, GrayForRedCircle, low, high, 3);

	return true;
}

RotatedRect Deal_Frame::Get_Target_Contour(Mat RedGrayImg,Mat Src){
	vector<vector<Point>> contours;
	vector<vector<Point>> contours_temp;
	vector<Vec4i> hierarchy;
	findContours(RedGrayImg, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	vector<cv::RotatedRect> contours01;
	vector<cv::RotatedRect> contours02;
	vector<cv::RotatedRect> contours03;

	for (size_t k = 0; k < contours.size(); k++)
	{
		if (contours[k].size() > 10)
			//使用前一定要判断是否大于5个点,小于5个点要报错,这里用35来初步过滤小轮廓
		{
			contours01.push_back(fitEllipse(Mat(contours[k])));
			//原始轮廓过滤
			contours_temp.push_back(contours[k]);
		}
	}

	//-------------------------------------------------
	//未经椭圆筛选的效果
	//for (size_t k = 0; k < contours01.size(); k++)
	//{
	//	ellipse(MatImgSrc, contours01.at(k), CV_RGB(0, 255, 0), 1);

	//}
	//-----------------------------------------------

	////第一次筛选
	////根据长短轴和面积筛选椭圆
	Ellipse_Coeff Ellipse_CoeffGroup;
	for (size_t k = 0; k < contours01.size(); k++)
	{
		bool GetEllpises_1st;
		Ellipse_CoeffGroup = (GetCoeff(contours01.at(k)));
		GetEllpises_1st = Filtet_Ellipse_1st(Ellipse_CoeffGroup, contours_temp[k]);
		if (GetEllpises_1st)
		{
			contours02.push_back(contours01.at(k));
			//黄线
			//ellipse(MatImgSrc, contours01.at(k), CV_RGB(255, 255, 0), 1);
		}
	}

	//第二次筛选
	//根据离心率和两圆圆心间距
	contours03 = Filtet_Ellipse_2st(contours02);
	for (size_t k = 0; k < contours03.size(); k++)
	{
		//蓝色线
		//ellipse(MatImgSrc, contours03.at(k), CV_RGB(0, 128, 255), 1);
	}
	//第三次筛选，从椭圆簇中选取用于姿态估计的椭圆
	RotatedRect fitellipse;
	if (contours03.size() != 0)
	{
		fitellipse = contours03[0];

		for (size_t k = 1; k < contours03.size(); k++)
		{
			double s1 = (double)fitellipse.size.height * fitellipse.size.width / 4;
			double s2 = (double)contours03[k].size.height * contours03[k].size.width / 4;
			float a = fitellipse.size.width / 2;
			float b = fitellipse.size.height / 2;
			float Eccentricity1 = CalcEccentricity(a, b);
			a = contours03[k].size.width / 2;
			b = contours03[k].size.height / 2;
			float Eccentricity2 = CalcEccentricity(a, b);
			if (s2>s1)
				fitellipse = contours03[k];
		}
	}
	if (fitellipse.size.area()!=0)
		Targe_Para.isGet_Red = true;
	else Targe_Para.isGet_Red = false;


//ellipse(Src, fitellipse, CV_RGB(0, 255, 128), 2);
//	imshow("捕获的圆", Src);

	return fitellipse;
}