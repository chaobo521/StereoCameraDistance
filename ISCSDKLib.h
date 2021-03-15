// 以下の ifdef ブロックは DLL からのエクスポートを容易にするマクロを作成するための 
// 一般的な方法です。この DLL 内のすべてのファイルは、コマンド ラインで定義された ISCDOTNET_EXPORTS
// シンボルを使用してコンパイルされます。このシンボルは、この DLL を使用するプロジェクトでは定義できません。
// ソースファイルがこのファイルを含んでいる他のプロジェクトは、 
// ISCDOTNET_API 関数を DLL からインポートされたと見なすのに対し、この DLL は、このマクロで定義された
// シンボルをエクスポートされたと見なします。

#ifndef __ISCSDKLIB_H__
#define	__ISCSDKLIB_H__


typedef unsigned int	DWORD;
typedef unsigned char	BYTE;

//#include "USBComController.h"

//#ifdef ISCDOTNET_EXPORTS
//#define ISCDOTNET_API __declspec(dllexport)
//#else
//#define ISCDOTNET_API __declspec(dllimport)
//#endif

//extern "C" {
class ISCSDKLib
{
public:

	struct CameraParamInfo
	{
		// D INF値
		float	fD_INF;
		// D INT値
		unsigned int nD_INF;
		// BF値
		float fBF;
		// 基線長値
		float fBaseLength;
		// 視野角
		float fViewAngle;
		// 画像サイズ(幅)
		unsigned int nImageWidth;
		// 画像サイズ(縦)
		unsigned int nImageHeight;
		// プロダクトナンバー
		unsigned int nProductNumber;
		// シリアルナンバー
		unsigned int nSerialNumber;
		// FPGA version
		unsigned int nFPGA_version;
		// 物体認識用の基準係数
		unsigned int nDistanceHistValue;
		// 除外対象とする視差の閾値
		unsigned int  nParallaxThreshold;
	};


	int OpenISC();
	int StartGrab(int nMode);
	int StopGrab();
	int GetImage(BYTE* pBuffer1, BYTE* pBuffer2,int nSkip);
	int GetDepthInfo(float* pBuffer);

	// FPS取得
	int GetFPS(float* pFPS);

	int GetFullFrameInfo(BYTE* pBuffer);

	int GetCameraParamInfo(CameraParamInfo* pParam);

	int SetCameraRegData(BYTE* pwBuf, DWORD wSize);
	int GetCameraRegData(BYTE* pwBuf, BYTE* prBuf, DWORD wSize, DWORD rSize);

	int SetAutoCalibration(int nMode);
	int GetAutoCalibration(int* nMode);

	int SetShutterControlMode(bool nMode);
	int GetShutterControlMode(bool* nMode);


	int GetImageSize(unsigned int* pnWidth,unsigned int* pnHeight);

	int GetExposureValue(unsigned int* pnValue );
	int SetExposureValue(unsigned int nValue);

	int GetGainValue(unsigned int* pnValue);
	int SetGainValue(unsigned int nValue);

	int CloseISC();

	int SetNoiseFilter(unsigned int nValue);
	int GetNoiseFilter(unsigned int* pnValue);

	int SetMeasArea(int nTop, int nLeft, int nRight, int nBottom);
	int GetMeasArea(int* nTop, int* nLeft, int* nRight, int* nBottom);

	int SetDoubleShutterControlMode(int nMode);
	int GetDoubleShutterControlMode(int* nMode);


	int SaveMemoryData();
	int GetRegInfo(BYTE* pBuff);
	void InitBitmapInfo();


	int labeling2(float* pDepth);
	void detect_object2(float* pDepth, int nLabelCount);
	int search_label(int j, int i);
	void change_label();
		
};

#endif	/*__ISCSDKLIB_H__*/
