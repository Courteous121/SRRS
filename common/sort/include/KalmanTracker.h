///////////////////////////////////////////////////////////////////////////////
// KalmanTracker.h: KalmanTracker Class Declaration

#ifndef KALMAN_H
#define KALMAN_H 2

#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "Parameter.h"

using namespace std;
using namespace cv;

#define StateType Rect_<float>

// This class represents the internel state of individual tracked objects observed as bounding box.
class KalmanTracker
{
public:
	KalmanTracker()
	{
		init_kf(StateType());
		m_time_since_update = 0;
		m_hits = 0;
		m_hit_streak = 0;
		m_age = 0;
	}
	KalmanTracker(StateType initRect)
	{
		init_kf(initRect);
		m_time_since_update = 0;
		m_hits = 0;
		m_hit_streak = 0;
		m_age = 0;
	}

	~KalmanTracker()
	{
		m_history.clear();
	}

	void predict();
	void update(StateType stateMat);
	void pre_update(StateType stateMat);

	StateType get_state();
	StateType get_rect_xysr(float cx, float cy, float s, float r);

	inline int getSinceTime()
	{
		return this->m_time_since_update;
	}
	inline int getHits()
	{
		return this->m_hit_streak;
	}
	inline StateType getHistory()
	{
		return this->m_history.back();
	}

private:
	int m_time_since_update;
	int m_hits;
	int m_hit_streak;
	int m_age;
	void init_kf(StateType stateMat);

	cv::KalmanFilter kf;
	cv::Mat measurement;

	std::vector<StateType> m_history;
};

#endif
