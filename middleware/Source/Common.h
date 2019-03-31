#ifndef _COMMON_H
#define _COMMON_H
#define ARMAPI extern "C"
	
//屏蔽没有影响的警告
#pragma diag_suppress 550 //声明后未使用的变量警告
#pragma diag_suppress 1 //头文件末尾无空白行的警告
#pragma diag_suppress 9 //注释部分的结构不规范的警告

#pragma pack (1)  //!!!!!注意用sizeof求结构体的内存大小时，有字节对齐的问题！，就是当
									//求得的值必须为结构体中基本变量的内存的整数倍，这个是设置字节对其为1字节，只要是1的整数倍就行

//数学宏
#define ABS(x) ((x)>0?(x):-(x))


//常用变量类型宏，数字代表bit数量
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
	
//适配C#
typedef UINT_16 UInt16 ;
typedef FLOAT_32 Single;
typedef INT_32 Int32;
typedef INT_8  byte;
typedef INT_16   Int16;
typedef UINT_16  ushort;
typedef UINT_32 UInt32;
#endif 
