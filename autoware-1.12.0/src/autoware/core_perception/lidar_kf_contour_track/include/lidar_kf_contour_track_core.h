/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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

#ifndef KF_CONTOUR_TRACKER_CORE
#define KF_CONTOUR_TRACKER_CORE

// ROS includes
#include <ros/ros.h>

#include "vector_map_msgs/PointArray.h"
#include "vector_map_msgs/LaneArray.h"
#include "vector_map_msgs/NodeArray.h"
#include "vector_map_msgs/StopLineArray.h"
#include "vector_map_msgs/DTLaneArray.h"
#include "vector_map_msgs/LineArray.h"
#include "vector_map_msgs/AreaArray.h"
#include "vector_map_msgs/SignalArray.h"
#include "vector_map_msgs/StopLine.h"
#include "vector_map_msgs/VectorArray.h"

#include "op_planner/RoadNetwork.h"
#include "op_planner/PlannerCommonDef.h"
#include "op_planner/MatrixOperations.h"
#include "SimpleTracker.h"
#include "PolygonGenerator.h"

#include <autoware_msgs/CloudClusterArray.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>

namespace ContourTrackerNS
{

class PerceptionParams
{
 public:
  double VehicleWidth;        // self-size
  double VehicleLength;       // self-size
  double DetectionRadius;     // find points from map that locate in this range from cur pose
  double MinObjSize;          // range of valid size
  double MaxObjSize;          // range of valid size
  double nQuarters;           // Divide plane XOY into sectors by angle. This is num of sectors.
  double PolygonRes;          // length of polygons' sides should small than this value
  TRACKING_TYPE trackingType; // 0 association only , 1 simple tracking, 2 contour based tracking
  bool bEnableSimulation;     // ???
  bool bEnableStepByStep;     // ???
  bool bEnableLogging;        // whether write log
  bool bEnableTTC;            // ???
  bool bEnableLaneChange;     // ???

  PerceptionParams()
  {
    VehicleWidth      = 0;
    VehicleLength     = 0;
    DetectionRadius   = 0;
    MinObjSize        = 0;
    MaxObjSize        = 0;
    nQuarters         = 0;
    PolygonRes        = 0;
    trackingType      = SIMPLE_TRACKER;
    bEnableStepByStep = false;
    bEnableSimulation = false;
    bEnableLogging    = false;
    bEnableTTC        = false;
    bEnableLaneChange = false;
  }
};

class ContourTracker
{
 protected:
  std::vector<PlannerHNS::DetectedObject> m_OriginalClusters; // process clusters and store objs here
  autoware_msgs::DetectedObjectArray m_OutPutResults;
  bool bNewClusters; //??? unused
  PlannerHNS::WayPoint m_CurrentPos;
  bool bNewCurrentPos; // true: when reveive pos
  PerceptionParams m_Params;
  SimpleTracker m_ObstacleTracking; // track all objs

  // Visualization Section
  int m_nDummyObjPerRep;
  int m_nDetectedObjRepresentations;
  std::vector<visualization_msgs::MarkerArray> m_DetectedPolygonsDummy;
  std::vector<visualization_msgs::MarkerArray> m_DetectedPolygonsActual;
  visualization_msgs::MarkerArray m_DetectedPolygonsAllMarkers;
  visualization_msgs::MarkerArray m_DetectionCircles;

  std::vector<visualization_msgs::MarkerArray> m_MatchingInfoDummy;
  std::vector<visualization_msgs::MarkerArray> m_MatchingInfoActual;

  visualization_msgs::MarkerArray m_TTC_Path;
  visualization_msgs::Marker m_TTC_Info;

  std::vector<std::string> m_LogData; // store info of clusters
  PlannerHNS::MAP_SOURCE_TYPE m_MapType;
  std::string m_MapPath;         // path that map is saved
  PlannerHNS::RoadNetwork m_Map; // store map
  bool bMap;                     // whether map exit
  double m_MapFilterDistance;    // to judge whether a point locate on lanes

  std::vector<PlannerHNS::Lane *> m_ClosestLanesList; // get lanes from map

  int m_nOriginalPoints;        // num of points of all clusters
  int m_nContourPoints;         // num of points that consist polygons of clusters
  double m_FilteringTime;       // time to judge whether every cluster locates on lanes(in map)
  double m_PolyEstimationTime;  // time to generate poly for all clusters
  double m_tracking_time;       // track time for this frame
  double m_dt;                  // time betweent consecutive frame
  struct timespec m_loop_timer; // timer to record time betweent consecutive frame

  // ROS subscribers
  ros::NodeHandle nh;

  // define publishers
  ros::Publisher pub_AllTrackedObjects;

  ros::Publisher pub_DetectedPolygonsRviz;
  ros::Publisher pub_TrackedObstaclesRviz;
  ros::Publisher pub_TTC_PathRviz;

  // define subscribers.
  ros::Subscriber sub_cloud_clusters;
  ros::Subscriber sub_current_pose;

  // Callback function for subscriber.
  void callbackGetCloudClusters(const autoware_msgs::CloudClusterArrayConstPtr &msg);
  void callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg);

  // Helper Functions
  void VisualizeLocalTracking();
  void ImportCloudClusters(const autoware_msgs::CloudClusterArrayConstPtr &msg, std::vector<PlannerHNS::DetectedObject> &originalClusters);
  bool IsCar(const PlannerHNS::DetectedObject &obj, const PlannerHNS::WayPoint &currState, PlannerHNS::RoadNetwork &map);
  void CalculateTTC(const std::vector<PlannerHNS::DetectedObject> &objs, const PlannerHNS::WayPoint &currState, PlannerHNS::RoadNetwork &map);
  void GetFrontTrajectories(std::vector<PlannerHNS::Lane *> &lanes, const PlannerHNS::WayPoint &currState, const double &max_distance, std::vector<std::vector<PlannerHNS::WayPoint>> &trajectories);
  void ReadNodeParams();
  void ReadCommonParams();
  void LogAndSend();

 public:
  ContourTracker();
  ~ContourTracker();
  void MainLoop();

  // Mapping Section

  UtilityHNS::MapRaw m_MapRaw;

  ros::Subscriber sub_lanes;
  ros::Subscriber sub_points;
  ros::Subscriber sub_dt_lanes;
  ros::Subscriber sub_intersect;
  ros::Subscriber sup_area;
  ros::Subscriber sub_lines;
  ros::Subscriber sub_stop_line;
  ros::Subscriber sub_signals;
  ros::Subscriber sub_vectors;
  ros::Subscriber sub_curbs;
  ros::Subscriber sub_edges;
  ros::Subscriber sub_way_areas;
  ros::Subscriber sub_cross_walk;
  ros::Subscriber sub_nodes;

  void callbackGetVMLanes(const vector_map_msgs::LaneArray &msg);
  void callbackGetVMPoints(const vector_map_msgs::PointArray &msg);
  void callbackGetVMdtLanes(const vector_map_msgs::DTLaneArray &msg);
  void callbackGetVMIntersections(const vector_map_msgs::CrossRoadArray &msg);
  void callbackGetVMAreas(const vector_map_msgs::AreaArray &msg);
  void callbackGetVMLines(const vector_map_msgs::LineArray &msg);
  void callbackGetVMStopLines(const vector_map_msgs::StopLineArray &msg);
  void callbackGetVMSignal(const vector_map_msgs::SignalArray &msg);
  void callbackGetVMVectors(const vector_map_msgs::VectorArray &msg);
  void callbackGetVMCurbs(const vector_map_msgs::CurbArray &msg);
  void callbackGetVMRoadEdges(const vector_map_msgs::RoadEdgeArray &msg);
  void callbackGetVMWayAreas(const vector_map_msgs::WayAreaArray &msg);
  void callbackGetVMCrossWalks(const vector_map_msgs::CrossWalkArray &msg);
  void callbackGetVMNodes(const vector_map_msgs::NodeArray &msg);
};

} // namespace ContourTrackerNS

#endif // KF_CONTOUR_TRACKER_CORE
