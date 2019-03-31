#ifndef _COMMON_H
#define _COMMON_H
#define ARMAPI extern "C"
	
//����û��Ӱ��ľ���
#pragma diag_suppress 550 //������δʹ�õı�������
#pragma diag_suppress 1 //ͷ�ļ�ĩβ�޿հ��еľ���
#pragma diag_suppress 9 //ע�Ͳ��ֵĽṹ���淶�ľ���

#pragma pack (1)  //!!!!!ע����sizeof��ṹ����ڴ��Сʱ�����ֽڶ�������⣡�����ǵ�
									//��õ�ֵ����Ϊ�ṹ���л����������ڴ��������������������ֽڶ���Ϊ1�ֽڣ�ֻҪ��1������������

//��ѧ��
#define ABS(x) ((x)>0?(x):-(x))


//���ñ������ͺ꣬���ִ���bit����
#define INT_8  char
#define INT_16 short int
#define INT_32 int 
#define INT_64 long int
	
#define UINT_8 unsigned char
#define UINT_16 unsigned short 
#define UINT_32 unsigned int 
#define UINT_64 unsigned  long int
	
#define FLOAT_32 float
#define FLOAT_64 double
	
//����C#
typedef UINT_16 UInt16 ;
typedef FLOAT_32 Single;
typedef INT_32 Int32;
typedef INT_8  byte;
typedef INT_16   Int16;
typedef UINT_16  ushort;
typedef UINT_32 UInt32;
#endif 
