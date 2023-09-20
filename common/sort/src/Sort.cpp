/**
 * @file Sort.cpp
 * @author 梁俊玮 (1609205169@qq.com)
 * @brief 
 * @version 1.0
 * @date 2021-11-27
 * 
 * @copyright Copyright SCUT RobotLab(c) 2022
 * 
 */
#include "Sort.h"

/**
 * @brief 计算两个bbox之间的iou
 * 
 * @param  box_a            其中一个盒子
 * @param  box_b            另外一个盒子
 * @return double 			iou值
 */
double GetIOU(Rect_<float> box_a, Rect_<float> box_b)
{
	float in = (box_a & box_b).area();
	float un = box_a.area() + box_b.area() - in;

	if (un < DBL_EPSILON)
		return 0;

	return (double)(in / un);
}

/**
 * @brief 目标跟踪算法（融合了检测与分类）
 * 
 * @param  frame            输入图像
 * @param  dectBoxes       检测盒子
 * @param  trackers         追踪器向量
 * @return int            输出结果（调试用）
 */
int TargetSort(vector<DetectBox> &dectBoxes, vector<TrackBox> &trackers)
{
	// 特殊情况flag,-1代表无结果产出
	int result = 0;
	// 帧数计时器
	static int frameCount = 0;
	frameCount++;
	//cout<<"frame_num:"<<frameCount<<endl;	//帧数计时器输出

	// iou矩阵
	vector<vector<double>> iouMatrix;
	// 进行了卡尔曼滤波预测的盒子
	vector<Rect_<float>> predictedBoxes;
	// 成功进行匹配的容器
	vector<int> assignment;
	// 没匹配的检测
	set<int> unmatchedDetections;
	// 没匹配的跟踪
	set<int> unmatchedTrajectories;
	// 所有的目标
	set<int> allItems;
	// 匹配到的目标
	set<int> matchedItems;
	// 配对好的一对
	vector<Point> matchedPairs;

	// 第一次进入这个函数
	if (trackers.size() == 0)
	{
		// 以检测盒子中的内容初始化追踪器
		for (auto dBox : dectBoxes)
		{
			Rect box = dBox.box;
			TrackBox kBox;
			KalmanTracker tracker = KalmanTracker((Rect_<float>)box);
			kBox.tracker = tracker;
			kBox.unblockProb = dBox.unblockProb;
			trackers.push_back(kBox);
		}
	}

	// 追踪器预测本帧
	for (auto it = trackers.begin(); it != trackers.end();)
	{
		// 运行卡尔曼滤波中的预测
		(*it).tracker.predict();
		// 获取预测结果
		Rect_<float> pBox = (*it).tracker.getHistory();
		// 更新追踪器状态
		(*it).situation = 2;
		// 更新被遮挡概率
		(*it).unblockProb = 0;
		// 若预测结果越界则删除，否则将结果加入已预测向量中
		if (pBox.x >= 0 && pBox.y >= 0)
		{
			predictedBoxes.push_back(pBox);
			it++;
		}
		else
		{
			it = trackers.erase(it);
		}
	}

	// 追踪器数量
	unsigned int trackNum = predictedBoxes.size();
	// 检测框数量
	unsigned int dectNum = dectBoxes.size();

	// 如果追踪器与检测框的数量都为零，则返回失败
	if (trackNum == 0 && dectNum == 0)
	{
		result = -1;
		return result;
	}

	// 针对预测时被删光的追踪器，避免下一步操作出错，用检测结果创建追踪器
	if (trackNum == 0)
	{
		for (auto dBox : dectBoxes)
		{
			// 创建追踪器
			Rect box = dBox.box;
			KalmanTracker tracker = KalmanTracker((Rect_<float>)box);
			// 创建卡尔曼滤波盒子
			TrackBox kBox;
			kBox.tracker = tracker;
			kBox.unblockProb = dBox.unblockProb;
			// 将新的追踪器，推进追踪器向量
			trackers.push_back(kBox);
			// 改变输出结果状态位
			result = -1;
			return result;
		}
	}

	// 为匈牙利算法做好准备
	// iou矩阵改变形状
	iouMatrix.resize(trackNum, vector<double>(dectNum, 0));
	for (unsigned int i = 0; i < trackNum; i++)
	{
		for (unsigned int j = 0; j < dectNum; j++)
		{
			// 将iou矩阵变为iou代价矩阵
			iouMatrix[i][j] = 1 - GetIOU(predictedBoxes[i], (Rect_<float>)(dectBoxes[j].box));
		}
	}

	// 匈牙利算法计算与分配
	HungarianAlgorithm HungAlgo;
	HungAlgo.Solve(iouMatrix, assignment);

	// 检测框多于追踪器，有新的目标出现
	if (dectNum > trackNum)
	{
		// 检测框中的比较多，以检测框中的目标为全部目标
		for (unsigned int n = 0; n < dectNum; n++)
			allItems.insert(n);

		// 以追踪框中的为已配对的
		for (unsigned int i = 0; i < trackNum; ++i)
			matchedItems.insert(assignment[i]);

		// 找出上述两者的差集，以其为没配对的
		set_difference(allItems.begin(), allItems.end(),
					   matchedItems.begin(), matchedItems.end(),
					   insert_iterator<set<int>>(unmatchedDetections, unmatchedDetections.begin()));
	}
	else if (dectNum < trackNum) //检测框少于追踪器
	{
		// 将没有受到分配的追踪器找出来，没有受到分配的检测框为新目标
		for (unsigned int i = 0; i < trackNum; ++i)
			if (assignment[i] == -1) // 没有分配好这个会变成-1
				unmatchedTrajectories.insert(i);
	}

	// 对分配矩阵进行处理，这一步主要是剔除那些匹配不符合iou阈值标准的
	for (unsigned int i = 0; i < trackNum; ++i)
	{
		// 没有匹配到检测框的跟踪器不管
		if (assignment[i] == -1)
		{
			continue;
		}
		// 匹配好的不满足iou阈值加入未匹配中，啥事情都没有的才加入已匹配队列
		if (1 - iouMatrix[i][assignment[i]] < sort_param.IOU_THRESHOLD)
		{
			unmatchedTrajectories.insert(i);
			unmatchedDetections.insert(assignment[i]);
		}
		else
		{
			matchedPairs.push_back(cv::Point(i, assignment[i]));
		}
	}

	// 对于没有检测结果来更新的追踪框，我们采取以预测来更新
	for (unsigned int i = 0; i < unmatchedTrajectories.size(); i++)
	{
		Rect_<float> pBox = trackers[i].tracker.getHistory();
		trackers[i].tracker.pre_update(pBox);
	}

	// 对已分配队列进行处理，用检测结果对跟踪器进行更新
	for (unsigned int i = 0; i < matchedPairs.size(); i++)
	{
		int dectId, trackId;
		trackId = matchedPairs[i].x;
		dectId = matchedPairs[i].y;

		trackers[trackId].tracker.update((Rect_<float>)(dectBoxes[dectId].box));
		//更改追踪器状态
		trackers[trackId].situation = 1;
		trackers[trackId].unblockProb = dectBoxes[dectId].unblockProb;
	}

	// 对没有匹配到的检测框创建追踪器
	for (auto umd : unmatchedDetections)
	{
		auto box = dectBoxes[umd];
		KalmanTracker tracker = KalmanTracker((Rect_<float>)(dectBoxes[umd].box));
		TrackBox kBox;
		kBox.tracker = tracker;
		kBox.unblockProb = dectBoxes[umd].unblockProb;
		trackers.push_back(kBox);
	}

	//处理输出结果
	for (auto it = trackers.begin(); it != trackers.end();)
	{
		//如果追踪器距离上次更新的帧数还没到最大限制，同时追踪器超过连续击中次数或者在前几帧
		if (((*it).tracker.getSinceTime() <= sort_param.MAX_AGE) &&
			((*it).tracker.getHits() >= sort_param.MIN_HITS || frameCount <= sort_param.MIN_HITS))
		{
			// 進入則說明可以輸出
			(*it).outputFlag = true;
			(*it).resetCount++;
			if ((*it).resetRank == 0)
			{
				// 重置等級最低時每幀都重置
				(*it).resetFlag = true;
				(*it).resetCount = 0;
			}
			else if ((*it).resetRank == 1 && (*it).resetCount >= sort_param.LOW_RESET_TIME)
			{
				// 重置等級中間時快速重置
				(*it).resetFlag = true;
				(*it).resetCount = 0;
			}
			else if ((*it).resetRank == 2 && (*it).resetCount >= sort_param.HIGH_RESET_TIME)
			{
				// 重置等級最大時自然重置
				(*it).resetFlag = true;
				(*it).resetCount = 0;
			}
		}
		else
		{
			if ((*it).situation == 2)
			{
				(*it).outputFlag = false;
			}
		}
		it++;
	}

	//删除寿命长的
	for (auto it = trackers.begin(); it != trackers.end();)
	{
		if ((it != trackers.end()) && ((*it).tracker.getSinceTime() > sort_param.MAX_AGE))
		{
			it = trackers.erase(it);
		}
		else
		{
			it++;
		}
	}

	return result;
}
