// sanyi.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/imgproc/imgproc_c.h"
#include <opencv2/dnn.hpp>
#include "ISCSDKLib.h"
#include <fstream>
#include <unistd.h>
using namespace cv;
using namespace dnn;
using namespace std;

typedef unsigned char       BYTE;
BYTE * pSI_RGBImage;
float confThreshold = 0.5; // Confidence threshold
float nmsThreshold = 0.4;  // Non-maximum suppression threshold
float maskThreshold = 0.5;
int inpWidth = 416;  // Width of network's input image
int inpHeight = 416; // Height of network's input image
vector<string> classes;
std::vector<String> outNames;
Net net;
void SI_2_RGB(BYTE *sibuf, DWORD dwSize)
{

	// sony岦偗color map
	short bnd0 = 2;
	short bnd1 = 3;
	short bnd2 = 14;
	short bnd3 = 22;
	short bnd4 = 37;
	short bnd5 = 124;
	short bnd6 = 256;

	//	for (cnt = 0; cnt<COLOR_PALETTE_SIZE / 2; cnt++)
	for (int cnt = 0; cnt < dwSize; cnt++)
	{
		// 惵
		if (sibuf[cnt] >= 0 && sibuf[cnt] < bnd0)
		{
			pSI_RGBImage[cnt * 3] = 0;
			pSI_RGBImage[(cnt * 3) + 1] = 0;
			pSI_RGBImage[(cnt * 3) + 2] = 0;
		}
		if (sibuf[cnt] >= bnd0 && sibuf[cnt] < bnd1)
		{
			pSI_RGBImage[cnt * 3] = 255;
			pSI_RGBImage[(cnt * 3) + 1] = 0;
			pSI_RGBImage[(cnt * 3) + 2] = 0;
		}

		// 惵->悈怓
		if (sibuf[cnt] >= bnd1 && sibuf[cnt] < bnd2)
		{
			pSI_RGBImage[cnt * 3] = 255;
			pSI_RGBImage[(cnt * 3) + 1] = (unsigned char)(255 * ((float)(sibuf[cnt] - bnd1) / (float)(bnd2 - bnd1)));
			pSI_RGBImage[(cnt * 3) + 2] = 0;
		}

		// 悈怓->椢
		if (sibuf[cnt] >= bnd2 && sibuf[cnt] < bnd3)
		{
			pSI_RGBImage[cnt * 3] = (unsigned char)(255 * (1.0 - ((float)(sibuf[cnt] - bnd2) / (float)(bnd3 - bnd2))));
			pSI_RGBImage[(cnt * 3) + 1] = 255;
			pSI_RGBImage[(cnt * 3) + 2] = 0;
		}

		// 椢->墿怓
		if (sibuf[cnt] >= bnd3 && sibuf[cnt] < bnd4)
		{
			pSI_RGBImage[cnt * 3] = 0;
			pSI_RGBImage[(cnt * 3) + 1] = 255;
			pSI_RGBImage[(cnt * 3) + 2] = (unsigned char)(255 * ((float)(sibuf[cnt] - bnd3) / (float)(bnd4 - bnd3)));
		}

		// 墿怓->愒
		if (sibuf[cnt] >= bnd4 && sibuf[cnt] < bnd5)
		{
			pSI_RGBImage[cnt * 3] = 0;
			pSI_RGBImage[(cnt * 3) + 1] = (unsigned char)(255 * (1.0 - ((float)(sibuf[cnt] - bnd4) / (float)(bnd5 - bnd4))));
			pSI_RGBImage[(cnt * 3) + 2] = 255;
		}

		// 愒->巼
		if (sibuf[cnt] >= bnd5 && sibuf[cnt] < bnd6)
		{
			pSI_RGBImage[cnt * 3] = (unsigned char)(255 * ((float)(sibuf[cnt] - bnd5) / (float)(bnd6 - bnd5)));
			pSI_RGBImage[(cnt * 3) + 1] = 0;
			pSI_RGBImage[(cnt * 3) + 2] = 255;
		}
	}
}
vector<String> getOutputsNames(const Net& net)
{
	static vector<String> names;
	if (names.empty())
	{
		//Get the indices of the output layers, i.e. the layers with unconnected outputs
		vector<int> outLayers = net.getUnconnectedOutLayers();

		//get the names of all the layers in the network
		vector<String> layersNames = net.getLayerNames();

		// Get the names of the output layers in names
		names.resize(outLayers.size());
		for (size_t i = 0; i < outLayers.size(); ++i)
			names[i] = layersNames[outLayers[i] - 1];
	}
	return names;
}
void drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat& frame)
{
	//Draw a rectangle displaying the bounding box
	if (right - left > 500) return;
	rectangle(frame, Point(left, top), Point(right, bottom), Scalar(255, 178, 50), 3);

	//Get the label for the class name and its confidence
	string label = format("%.2f", conf);
	string pointlabel = format("%d,%d", right - left, bottom - top);
	if (!classes.empty())
	{
		CV_Assert(classId < (int)classes.size());
		label = classes[classId];// +":" + pointlabel;
	}

	//Display the label at the top of the bounding box
	int baseLine;
	Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
	top = max(top, labelSize.height);
	//	rectangle(frame, Point(left, top - round(1.5*labelSize.height)), Point(left + round(1.5*labelSize.width), top + baseLine), Scalar(255, 255, 255), FILLED);
	putText(frame, label, Point(left, bottom - 5), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 1);
	//	putText(frame, pointlabel, Point(left, bottom-3 ), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 1);
}
vector<Rect> postprocess(Mat& frame, const vector<Mat>& outs)
{
	vector<int> classIds;
	vector<float> confidences;
	vector<Rect> boxes;

	for (size_t i = 0; i < outs.size(); ++i)
	{
		// Scan through all the bounding boxes output from the network and keep only the
		// ones with high confidence scores. Assign the box's class label as the class
		// with the highest score for the box.
		float* data = (float*)outs[i].data;
		for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
		{
			Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
			Point classIdPoint;
			double confidence;
			// Get the value and location of the maximum score
			minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
			if (confidence > confThreshold)
			{
				int centerX = (int)(data[0] * frame.cols);
				int centerY = (int)(data[1] * frame.rows);
				int width = (int)(data[2] * frame.cols);
				int height = (int)(data[3] * frame.rows);
				int left = centerX - width / 2;
				int top = centerY - height / 2;

				classIds.push_back(classIdPoint.x);
				confidences.push_back((float)confidence);
				boxes.push_back(Rect(left, top, width, height));
			}
		}
	}

	// Perform non maximum suppression to eliminate redundant overlapping boxes with
	// lower confidences
	vector<int> indices;
	vector<Rect> boxesROI;
	NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
	for (size_t i = 0; i < indices.size(); ++i)
	{
		int idx = indices[i];
		Rect box = boxes[idx];
		drawPred(classIds[idx], confidences[idx], box.x, box.y,
			box.x + box.width, box.y + box.height, frame);
		boxesROI.push_back(box);

	}
	return boxesROI;
}
//根据ROI及视差整数计算距离
double  getDistance(cv::Mat & src, cv::Mat decDisp, cv::Rect roiRect, double bf, double d_inf, double minValue)
{
	roiRect.x = roiRect.x < decDisp.cols ? roiRect.x : decDisp.cols;
	roiRect.x = roiRect.x > 0 ? roiRect.x : 0;
	roiRect.y = roiRect.y < decDisp.rows ? roiRect.y : decDisp.rows;
	roiRect.y = roiRect.y > 0 ? roiRect.y : 0;
	roiRect.width = (roiRect.x + roiRect.width) < decDisp.cols ? roiRect.width : (decDisp.cols - roiRect.x);
	roiRect.height = (roiRect.y + roiRect.height) < decDisp.rows ? roiRect.height : (decDisp.rows - roiRect.y);
	cv::Mat ROI = decDisp(roiRect);
	cv::Point minLoc, maxLoc;
	double minVal, maxVal = 0;
	int allNum = 0;
	float allParallar = 0, meanParallar;
	double distance = 0, modeDistance;
	cv::Mat paraNum(1, decDisp.cols, CV_32FC1, cv::Scalar::all(0));
	float * paraNumRow = paraNum.ptr<float>(0);
	for (int m = 0; m < ROI.rows; m++)
	{
		float * ROIRow = ROI.ptr <float>(m);
		for (int n = 0; n < ROI.cols; n++)
		{
			if (ROIRow[n] > 2)
			{
				paraNumRow[(int)ROIRow[n]]++;
			}
		}
	}
	minMaxLoc(paraNum, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());
	if (maxVal > 0)
	{
		modeDistance = bf / maxLoc.x;
		int maxDiffpara = int(bf / (modeDistance - 0.08) - maxLoc.x);
		if (!maxDiffpara) maxDiffpara++;

		for (int m = 0; m < ROI.rows; m++)
		{
			float * ROIRow = ROI.ptr<float>(m);
			for (int n = 0; n < ROI.cols; n++)
			{
				if (ROIRow[n] >= maxLoc.x - maxDiffpara && ROIRow[n] <= maxLoc.x + maxDiffpara)
				{
					allParallar += ROIRow[n];
					allNum++;
				}
			}
		}
		meanParallar = allParallar / allNum;
		distance = bf / (meanParallar - d_inf);
		rectangle(src, roiRect, cv::Scalar::all(0), 1);
		if (meanParallar >= minValue)
		{
			putText(src, cv::format("d:%.3fm", distance), cv::Point(roiRect.tl().x, roiRect.tl().y + 25), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 255, 0), 2, 8);
		}
	}
	return distance;
}
int main()
{
        ISCSDKLib iscsdklib;

	std::string  names_file = "./YoloCppSDK/coco.names";
	std::string  cfg_file = "./YoloCppSDK/yolo.cfg";
	std::string  weights_file = "./YoloCppSDK/yolo.weights";///
	
	
	unsigned int nImageW, nImageH;
	BYTE * pLeftImage;
	BYTE * pRightImage;
	
	BYTE ReadMode = 2;
	cout<<"Original ReadMode:"<<ReadMode<<endl;
	float M_BF = 109.64, D_INF = 0.10;
	cout << "请输入要读取相机的模式" << endl;
	cout << " >> 2 ：-> 视差 + 校正后左图" << endl;
	cout << " >> 3 ：校正后图-> 左图 + 右图" << endl;
	cout << " >> 4 ：原图->  左图 + 右图" << endl;
	cout << "完成后按 回车（Enter）结束。" << endl;
	cin >> ReadMode;
	cout<<"Input ReadMode:"<<ReadMode<<endl;

	ReadMode = ReadMode - 0x30;
	cout<<"ReadMode:"<<ReadMode<<endl;

	cout << "初始化相机，请稍等" << endl;
	int ret = iscsdklib.OpenISC();
	sleep(1);
	if (!(ret == 0 || ret == -1) )
	{
		iscsdklib.CloseISC();
		cout << "相机初始化失败，错误码："<<  ret << endl;
		return 0;
	}
	iscsdklib.GetImageSize(&nImageW, &nImageH);
	sleep(1);
	pLeftImage = (BYTE*)malloc(nImageW *nImageH);
	pRightImage = (BYTE*)malloc(nImageW*nImageH);
	//	pRGBImage = (BYTE*)malloc(nImageW *nImageH * 3);//初始化rgb缓冲区
	pSI_RGBImage = (BYTE*)malloc(nImageW *nImageH * 3);//初始化视差伪彩图缓冲区
	iscsdklib.StartGrab((int)ReadMode);
	sleep(1);
	iscsdklib.SetShutterControlMode(1);
	cout << "AI模型加载中..." << endl;

	std::ifstream ifs(names_file.c_str());
	std::string line;
	while (getline(ifs, line)) classes.push_back(line);

	net = readNetFromDarknet(cfg_file, weights_file);
	net.setPreferableBackend(DNN_BACKEND_CUDA);
	net.setPreferableTarget(DNN_TARGET_CUDA);
	outNames = getOutputsNames(net);
	while (1)
	{
		Mat frameL,frameR, frameD, frameDRGB;//当读取图像为左图,右图，视差图整数，伪彩图	
		Mat frameDispfloat(nImageH, nImageW, CV_32FC1);//视差小数图像
		int nRet;
		Mat blob;
		nRet = iscsdklib.GetImage(pLeftImage, pRightImage, 0);
		iscsdklib.GetDepthInfo((float *)frameDispfloat.data);
		
		switch (ReadMode)
		{
		case 2://视差
		{
			
				frameL = cv::Mat(nImageH, nImageW, CV_8UC1, pRightImage).clone();
				SI_2_RGB(pLeftImage, nImageH * nImageW);
				frameD = cv::Mat(nImageH, nImageW, CV_8UC1, pLeftImage).clone();
				frameDRGB = cv::Mat(nImageH, nImageW, CV_8UC3, pSI_RGBImage).clone();//m_ShowImage.rightImage
				
			break;
		}
		case 3://校正后
		case 4://原图	
		{
			frameL = cv::Mat(nImageH, nImageW, CV_8UC1, pLeftImage).clone();
			frameR = cv::Mat(nImageH, nImageW, CV_8UC1, pRightImage).clone();
			break;
		}
		}

		if (frameL.empty()) continue;
		cv::flip(frameL, frameL, 0);
		cv::flip(frameR, frameR, 0);
		cv::flip(frameDispfloat, frameDispfloat, 0);
		cv::flip(frameD, frameD, 0);
		cv::flip(frameDRGB, frameDRGB, 0);
		cvtColor(frameL, frameL, CV_GRAY2RGB);	
		blobFromImage(frameL, blob, 1 / 255.0, cv::Size(inpWidth, inpHeight), Scalar(0, 0, 0), true, false);

		net.setInput(blob);
		// Runs the forward pass to get output of the output layers

		vector<Mat> outs;
		net.forward(outs, outNames);

		// Remove the bounding boxes with low confidence
		vector<Rect> result_vec = postprocess(frameL, outs);
		switch (ReadMode)
		{
		case 2://视差
		{

			for (int i = 0; i < result_vec.size(); i++)
			{
				getDistance(frameL, frameDispfloat, result_vec[i], M_BF, D_INF, 2.0);//根据detection框坐标进行距离计算
			}
			imshow("frameL", frameL);
			imshow("frameD", frameDRGB);
			break;
		}
		case 3://校正后
		case 4://原图	
		{
			imshow("frameL", frameL);
			imshow("frameR", frameR);
			break;
		}
		}
		int key = waitKey(5);
		if (key =='q')
		{
			break;
		}

	}
	iscsdklib.StopGrab();
	iscsdklib.CloseISC();
}

