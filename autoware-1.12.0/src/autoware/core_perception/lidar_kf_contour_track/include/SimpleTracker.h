/*
 * Copyright 2016-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef SimpleTracker_H_
#define SimpleTracker_H_

#include "op_planner/RoadNetwork.h"
#include "op_planner/PlanningHelpers.h"
#include "op_utility/UtilityH.h"
#include "opencv2/video/tracking.hpp"
#include <vector>
#include <math.h>
#include <iostream>

namespace ContourTrackerNS
{

#define DEBUG_TRACKER 0
#define NEVER_GORGET_TIME -1000
#define ACCELERATION_CALC_TIME 0.25     // time diff between acc calculation
#define ACCELERATION_DECISION_VALUE 0.5 // judge direction of acc
#define PREV_TRACK_SIZE 25              // max num of history track result
#define PREV_TRACK_SMOOTH_DATA 0.475    //
#define PREV_TRACK_SMOOTH_SMOOTH 0.3    // use last and next position smooth, this is weight of last and next position

enum TRACKING_TYPE
{
  ASSOCIATE_ONLY  = 0,
  SIMPLE_TRACKER  = 1,
  CONTOUR_TRACKER = 2
};

struct Kalman1dState
{
  double MovCov;     // double q; //moving noise covariance
  double MeasureCov; // double r; //measurement noise covariance
  double x;          // value
  double p;          // estimation error covariance
  double k;          // kalman gain
};

class kalmanFilter1D
{
 public:
  Kalman1dState result;

  kalmanFilter1D()
  {
  }
  kalmanFilter1D(double MovCov, double MeasureCov, double p, double intial_value)
  {
    result.MovCov     = MovCov;
    result.MeasureCov = MeasureCov;
    result.p          = p;
    result.x          = intial_value;
  }

  Kalman1dState Update(double measurement)
  {
    // prediction update
    result.p = result.p + result.MovCov;

    // measurement update
    result.k = result.p / (result.p + result.MeasureCov);
    result.x = result.x + result.k * (measurement - result.x);
    result.p = (1 - result.k) * result.p;

    return result;
  }
};

class KFTrackV
{
 private:
  cv::KalmanFilter m_filter;
  double prev_x, prev_y, prev_v, prev_a, prev_accel; // position, vel, angle of movement, accelerate
  double prev_big_v;                                 //???
  double prev_big_a;
  long m_id;
  int nStates;            // dimension of state
  int nMeasure;           // dimension of measurement
  int MinAppearanceCount; // if times we detect this obj more than this num, we can calculate its velocity
  double time_diff;

 public:
  int m_bUpdated;     /* when match this obj with new obj sucessfully, we will update this obj and set it as "true";
                         If we don't match this obj, it may be set as "false", which depends on TRACKING_TYPE*/
  int region_id;      // Interest region is divided into several circles(regions). At which region this obj locates.
  double forget_time; // This obj will be removed from track list if it isn't be detected in this time span.
  int m_iLife;        // times that this obj was detected.

  PlannerHNS::DetectedObject obj; // Used for associate only , don't remove

  long GetTrackID()
  {
    return m_id;
  }

  KFTrackV(double x, double y, double a, long id, double _dt, int _MinAppearanceCount = 1)
  {
    //		errorSmoother.result.MovCov = 0.125;
    //		errorSmoother.result.MeasureCov = 0.1;
    //		errorSmoother.result.p = 1;
    //		errorSmoother.result.x = 0;
    time_diff   = 0;
    region_id   = -1;
    forget_time = NEVER_GORGET_TIME; // this is very bad , dangerous
    m_iLife     = 0;                 // detected times
    prev_x      = x;
    prev_y      = y;
    prev_v      = 0;
    prev_accel  = 0;
    prev_big_v  = 0;
    prev_big_a  = 0;
    prev_a      = a; // angle of movement
    nStates     = 4; // position, velcity
    nMeasure    = 2; // position

    MinAppearanceCount = _MinAppearanceCount;

    m_id       = id;
    m_bUpdated = true;

    m_filter = cv::KalmanFilter(nStates, nMeasure);
#if (CV_MAJOR_VERSION == 2)
    m_filter.transitionMatrix = *(cv::Mat_<float>(nStates, nStates) << 1, 0, _dt, 0,
                                  0, 1, 0, _dt,
                                  0, 0, 1, 0,
                                  0, 0, 0, 1);
#elif (CV_MAJOR_VERSION == 3)
    m_filter.transitionMatrix = (cv::Mat_<float>(nStates, nStates) << 1, 0, _dt, 0,
                                 0, 1, 0, _dt,
                                 0, 0, 1, 0,
                                 0, 0, 0, 1);
#endif

    m_filter.statePre.at<float>(0) = x;
    m_filter.statePre.at<float>(1) = y;
    m_filter.statePre.at<float>(2) = 0;
    m_filter.statePre.at<float>(3) = 0;

    m_filter.statePost = m_filter.statePre;

    setIdentity(m_filter.measurementMatrix);

    cv::setIdentity(m_filter.measurementNoiseCov, cv::Scalar::all(0.0001));
    cv::setIdentity(m_filter.processNoiseCov, cv::Scalar::all(0.0001));
    cv::setIdentity(m_filter.errorCovPost, cv::Scalar::all(0.075));

    m_filter.predict();

    // errorSmoother.Update(a);
  }

  // old object --> predict object
  void UpdateTracking(double _dt, const PlannerHNS::DetectedObject &oldObj, PlannerHNS::DetectedObject &predObj)
  {
#if (CV_MAJOR_VERSION == 2)
    m_filter.transitionMatrix = *(cv::Mat_<float>(nStates, nStates) << 1, 0, _dt, 0,
                                  0, 1, 0, _dt,
                                  0, 0, 1, 0,
                                  0, 0, 0, 1);
#elif (CV_MAJOR_VERSION == 3)
    m_filter.transitionMatrix = (cv::Mat_<float>(nStates, nStates) << 1, 0, _dt, 0,
                                 0, 1, 0, _dt,
                                 0, 0, 1, 0,
                                 0, 0, 0, 1);
#endif

    cv::Mat_<float> measurement(nMeasure, 1);
    cv::Mat_<float> prediction(nStates, 1);

    measurement(0) = oldObj.center.pos.x;
    measurement(1) = oldObj.center.pos.y;

    prediction = m_filter.correct(measurement); //???

    predObj.center.pos.x = prediction.at<float>(0);
    predObj.center.pos.y = prediction.at<float>(1);
    double vx            = prediction.at<float>(2);
    double vy            = prediction.at<float>(3);

    double currA     = 0; // angle of movement
    double currV     = 0; // predicted velovity
    double currAccel = 0;

    if (m_iLife > 1)
    {
      currV = sqrt(vx * vx + vy * vy);

      double diff_y = predObj.center.pos.y - prev_y;
      double diff_x = predObj.center.pos.x - prev_x;
      if (hypot(diff_y, diff_x) > 0.2)
      {
        currA = atan2(diff_y, diff_x);
      }
      else
      {
        currA = prev_a;
      }
    }

    if (m_iLife > MinAppearanceCount)
    {
      predObj.center.pos.a = currA; // use angle of movement as direction of pose
      predObj.center.v     = currV;
      predObj.bVelocity    = true;
      //			predObj.acceleration_raw = (currV - prev_v)/_dt;
      // get acc: if time diff is long enough, update acc
      if (time_diff > ACCELERATION_CALC_TIME)
      {
        currAccel  = (currV - prev_big_v) / time_diff;
        prev_big_v = currV;
        time_diff  = 0;
      }
      else
      {
        time_diff += _dt;
        currAccel = prev_accel;
      }

      predObj.acceleration_raw = currAccel;
      if (fabs(predObj.acceleration_raw) < ACCELERATION_DECISION_VALUE)
        predObj.acceleration_desc = 0;
      else if (predObj.acceleration_raw > ACCELERATION_DECISION_VALUE)
        predObj.acceleration_desc = 1;
      else if (predObj.acceleration_raw < -ACCELERATION_DECISION_VALUE)
        predObj.acceleration_desc = -1;

      // predObj.acceleration_desc = UtilityHNS::UtilityH::GetSign(predObj.acceleration_raw - prev_big_a);
      // std::cout << "Acceleraaaaaaaaaaaaaaate : " << predObj.acceleration << ", BigV:" << prev_big_v << std::endl;
    }
    else
    {
      predObj.bDirection = false;
      predObj.bVelocity  = false;
    }
    // limit num of history positions: erase oldest history positions
    if (predObj.centers_list.size() > PREV_TRACK_SIZE)
      predObj.centers_list.erase(predObj.centers_list.begin() + 0);

    if (predObj.centers_list.size() > 1)
    {
      double diff_y = predObj.center.pos.y - predObj.centers_list.at(predObj.centers_list.size() - 1).pos.y;
      double diff_x = predObj.center.pos.x - predObj.centers_list.at(predObj.centers_list.size() - 1).pos.x;
      // if move far enough, update/add path
      if (hypot(diff_y, diff_x) > 0.1)
      {
        predObj.centers_list.push_back(predObj.center);
        //(in/out path, in weight of current position, in weight of last and next position, in change in an iteration)
        PlannerHNS::PlanningHelpers::SmoothPath(predObj.centers_list, PREV_TRACK_SMOOTH_DATA, PREV_TRACK_SMOOTH_SMOOTH, 0.1);
        // cost is length of path
        PlannerHNS::PlanningHelpers::CalcAngleAndCost(predObj.centers_list);
      }
    }
    else
      predObj.centers_list.push_back(predObj.center);

    if (predObj.centers_list.size() > 3)
    {
      predObj.bDirection   = true;
      predObj.center.pos.a = (predObj.centers_list.at(predObj.centers_list.size() - 1).pos.a
                              + predObj.centers_list.at(predObj.centers_list.size() - 2).pos.a
                              + predObj.centers_list.at(predObj.centers_list.size() - 3).pos.a)
                             / 3.0;
    }
    else if (predObj.centers_list.size() > 2)
    {
      predObj.center.pos.a = (predObj.centers_list.at(predObj.centers_list.size() - 1).pos.a + predObj.centers_list.at(predObj.centers_list.size() - 2).pos.a) / 2.0;
    }
    else if (predObj.centers_list.size() > 1)
    {
      predObj.center.pos.a = predObj.centers_list.at(predObj.centers_list.size() - 1).pos.a;
    }

    m_filter.predict();
    m_filter.statePre.copyTo(m_filter.statePost);
    m_filter.errorCovPre.copyTo(m_filter.errorCovPost);

    prev_a     = currA;
    prev_y     = predObj.center.pos.y;
    prev_x     = predObj.center.pos.x;
    prev_v     = currV;
    prev_accel = currAccel;

    forget_time -= _dt;
    m_iLife++;
  }

  // update position of center, angle, path;
  void UpdateAssociateOnly(double _dt, const PlannerHNS::DetectedObject &oldObj, PlannerHNS::DetectedObject &predObj)
  {
    if (predObj.centers_list.size() > PREV_TRACK_SIZE)
      predObj.centers_list.erase(predObj.centers_list.begin() + 0);

    if (predObj.centers_list.size() > 1)
    { // cur-last
      double diff_y = predObj.center.pos.y - predObj.centers_list.at(predObj.centers_list.size() - 1).pos.y;
      double diff_x = predObj.center.pos.x - predObj.centers_list.at(predObj.centers_list.size() - 1).pos.x;
      if (hypot(diff_y, diff_x) > 0.1)
      {
        predObj.centers_list.push_back(predObj.center);
        PlannerHNS::PlanningHelpers::SmoothPath(predObj.centers_list, PREV_TRACK_SMOOTH_DATA, PREV_TRACK_SMOOTH_SMOOTH, 0.1);
        PlannerHNS::PlanningHelpers::CalcAngleAndCost(predObj.centers_list);
      }
    }
    else
      predObj.centers_list.push_back(predObj.center);

    if (predObj.centers_list.size() > 3)
    {
      predObj.center.pos.a = (predObj.centers_list.at(predObj.centers_list.size() - 1).pos.a
                              + predObj.centers_list.at(predObj.centers_list.size() - 2).pos.a
                              + predObj.centers_list.at(predObj.centers_list.size() - 3).pos.a)
                             / 3.0;
    }
    else if (predObj.centers_list.size() > 2)
    {
      predObj.center.pos.a = (predObj.centers_list.at(predObj.centers_list.size() - 1).pos.a
                              + predObj.centers_list.at(predObj.centers_list.size() - 2).pos.a)
                             / 2.0;
    }
    else if (predObj.centers_list.size() > 1)
    {
      predObj.center.pos.a = predObj.centers_list.at(predObj.centers_list.size() - 1).pos.a;
    }

    if (m_iLife > MinAppearanceCount)
    {
      predObj.bDirection = true;
    }
    else
    {
      predObj.bDirection = false;
    }

    m_iLife++;
  }

  virtual ~KFTrackV() {}
};

// Divide interest region into several circles.
class InterestCircle
{
 public:
  int id;
  double radius;
  double forget_time; /* If an obj is detected in this region/circle, it will be removed from track list
                         if it disappears for this time
                       */
  std::vector<KFTrackV *> pTrackers;

  InterestCircle(int _id)
  {
    id          = _id;
    radius      = 0;
    forget_time = NEVER_GORGET_TIME; // never forget
  }
};

//???
class CostRecordSet
{
 public:
  int i_obj;
  int i_track;
  double cost;
  double size_diff;
  double width_diff;
  double length_diff;
  double height_diff;
  double angle_diff;
  double distance_diff;
  CostRecordSet(int obj_index, int track_index, double _distance_diff, double _size_diff,
                double _width_diff, double _length_diff, double _height_diff, double _angle_diff)
  {
    i_obj         = obj_index;
    i_track       = track_index;
    size_diff     = _size_diff;
    angle_diff    = _angle_diff;
    distance_diff = _distance_diff;
    width_diff    = _width_diff;
    length_diff   = _length_diff;
    height_diff   = _height_diff;
    cost          = 0;
  }

  CostRecordSet()
  {
    i_obj         = -1;
    i_track       = -1;
    size_diff     = 0;
    angle_diff    = 0;
    distance_diff = 0;
    width_diff    = 0;
    length_diff   = 0;
    height_diff   = 0;
    cost          = 0;
  }
};

class SimpleTracker
{
 public:
  std::vector<PlannerHNS::DetectedObject> m_DetectedObjects; // store newly detected objs from perception
  std::vector<InterestCircle *> m_InterestRegions;           // store sub-regions

  std::vector<std::pair<PlannerHNS::WayPoint, PlannerHNS::WayPoint>> m_MatchList; // pair<track obj, new obj>

  void DoOneStep(const PlannerHNS::WayPoint &currPose,
                 const std::vector<PlannerHNS::DetectedObject> &obj_list,
                 const TRACKING_TYPE &type = SIMPLE_TRACKER);

  SimpleTracker();
  virtual ~SimpleTracker();
  void InitSimpleTracker();
  void InitializeInterestRegions(std::vector<InterestCircle *> &regions);

 public:
  double m_dt;                       // time(cur - track)
  double m_MAX_ASSOCIATION_DISTANCE; // ony when distance smaller this num, match two objs
  bool m_bUseCenterOnly;             // unused
  double m_MaxKeepTime;              /* This varible is used to initialize forget-time of sub-regions. */
  bool m_bFirstCall;                 // true: if DoOneStep hasn't been called
  int m_nMinTrustAppearances;        // if times we detect this obj more than this num, we can calculate its velocity
  double m_Horizon;                  // reduis of interest region.
  double m_CirclesResolution;        /* Region will be divided into several circle sub-regions. This varible
                                        decides the num of sub-regions.
                                     */

  double m_MAX_ASSOCIATION_SIZE_DIFF;  // size limit when associate objs
  double m_MAX_ASSOCIATION_ANGLE_DIFF; // angle limit when associate objs
  bool m_bEnableStepByStep;            //???

 private:
  std::vector<CostRecordSet> m_CostsLists; // cost of matches
  std::vector<KFTrackV *> m_Tracks;        // unused
  std::vector<KFTrackV> m_TrackSimply;     // tracked objs
  timespec m_TrackTimer;                   // time when last track(ignore process time)
  long iTracksNumber;                      // num of objs that we have tracked and will be use as id of new tracked obj
  PlannerHNS::WayPoint m_PrevState;
  std::vector<KFTrackV> newObjects;     // matching result for current frame and will be used to update tracked objs
  void AssociateAndTrack();             // a: a is similiar with b
  void AssociateDistanceOnlyAndTrack(); // b: a&b belong to CONTOUR_TRACKER
  void AssociateSimply();
  void AssociateToRegions(KFTrackV &detectedObject);
  void CleanOldTracks();
  void AssociateOnly();
  void MergeObjectAndTrack(KFTrackV &track, PlannerHNS::DetectedObject &obj);
  int InsidePolygon(const std::vector<PlannerHNS::GPSPoint> &polygon, const PlannerHNS::GPSPoint &p);

  void MatchClosest();
  void MatchWithDistanceOnly();
  void MatchClosestCost();
};

} // namespace ContourTrackerNS

#endif /* SimpleTracker_H_ */
