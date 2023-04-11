| 属性                   | 类型                      | 作用                                             |
| ---------------------- | ------------------------- | ------------------------------------------------ |
| mSensor                | eSensor                   | 输入传感器类型                                   |
| mpVocabulary           | ORBVocabulary*            | ORB特征，用来特征匹配与定位                      |
| mpKeyFrameDatabase     | KeyFrameDatabase*         | 关键帧，用于重定位和闭环                         |
| mpMap                  | Map*                      | 地图                                             |
| mpTracker              | Tracking*                 | 决定何时插入关键帧，如果track失败，重定位        |
| mpLocalMapper          | LocalMapping*             | 管理局部地图，完成局部BA                         |
| mpLoopCloseer          | LoopClosing*              | 每一帧都查看是否闭环                             |
| mpViewer               | Viewer*                   | 画地图和相机位姿， Pangolin库                    |
| mpFrameDrawer          | FrameDrawer               | 同上                                             |
| mpMapDrawer            | MapDrawer                 | 同上                                             |
| mptLocalMapping        | std::thread*              | mapping线程， tracking是系统的主线程，创造system |
| mptLoopClosing         | std::thread*              | loop线程                                         |
| mptViewer              | std::thread*              | viewer线程                                       |
| mbReset/mMutexReset    | bool/ std::mutex          | reset 标签                                       |
| mbActivateLocaionMode  | bool                      | 改变模式标签，是定位模式还是slam模式             |
| mbDeactiv............. | bool                      |                                                  |
| mTrackingState         | int                       | Tracking state                                   |
| mTrackedMapPoints      | std::vector<MapPoint*>    | 在追踪的地图向量                                 |
| mTrackedKeyPointSun    | std::vector< cv:KeyPoint> | 追踪的关键点                                     |

* Tracking.h

| 属性                      | 类型                       | 作用              |
| ------------------------- | -------------------------- | ----------------- |
| mState                    | eTrackingState             | 当前帧追踪状态    |
| mLastPorcessedState       | eTrackingState             | 上一帧追踪状态    |
| mSensor                   | int                        | 输入传感器        |
| mCurrentFrame             | Frame                      | 当前帧            |
| mbOnlyTracking            | bool                       | slam模式定位/slam |
| **单目专用：**            |                            |                   |
| mImGray                   | cv                         | 当前帧灰度图      |
| mvIniLastMatches          | std::vector<int>           |                   |
| mvInitMatches             | std::vector<int>           |                   |
| mvbPrevMatched            | std::vector<<cv::Point2f>> |                   |
| mvIniP3D                  | std::vecotr<<cv::Point3f>> |                   |
| mInitialFrame             | Frame                      | 初始帧            |
| mpInitializer             | Initializer*               | 单目初始器        |
| **恢复相机轨迹**          |                            |                   |
| mlRelativeFramePoses      | list<<cv::Mat>>            | pose的变换        |
| mlpReferences             | list<keyFrame*>            | 参考的关键帧      |
| mlFrameTimes              | list<double>               |                   |
| mlbLost                   | list<bool>                 |                   |
| **other Thread Pointers** |                            |                   |
| mpLocalMapper             | LocalMapping*              |                   |
| mpLoopClosing             | LoopClosing*               |                   |
| **ORB**                   |                            |                   |
| mpORBextractorLeft/right  | ORBextracrtor*             |                   |
| mpIniORBextractor         | ORBextractor*              |                   |
| **Bow**                   |                            |                   |
| mpORBVocabulary;          | ORBVocabulary*             | ORB训练好的词袋库 |
| mpKeyFrameDB              | KeyFrameDatabase*          | 关键帧的词袋库    |
| **Local Map**             |                            |                   |
| mpReferenceKF             | KeyFrame*                  |                   |
| mvpLocalKeyFrames         | std::vector<KeyFrame*>     |                   |
| mvpLocalMapPointS         | std::vector<MapPoint*>     |                   |
| **System**                |                            |                   |
| mpSystem                  | System*                    |                   |
| **Drawers**               |                            |                   |
| mpViewer                  | Viewer*                    |                   |
| mpFrameDrawer             | FrameDrawer*               |                   |
| mpMapDrawer               | MapDrawer*                 |                   |
| **Map**                   |                            |                   |
| mpMap                     | Map*                       |                   |

| tracking.h  属性       | 类型            | 作用                    |
| ---------------------- | --------------- | ----------------------- |
| mK                     | cv::Mat         | 内参                    |
| mDistCoef              | cv::Mat         | 畸变参数                |
| **new KeyFrame rules** |                 |                         |
| mMinFrames/mMaxFrames  | int             | 根据fps设定，关键帧选择 |
| **other**              |                 |                         |
| mThDepth               | float           | 近的点和远的点阈值      |
| mDepthMapFactor        | float           | 对与RGBD用              |
| mnMatchInliers         | int             | 当前帧匹配的内点        |
| mVelocity              | cv::Mat         | 运动模型                |
| mbRGB                  | bool            | BGR/RGB                 |
| mlpTemporalPoints      | list<MapPoint*> |                         |
| mpLastKeyFrame;        | KeyFrame*       |                         |
| mLastFrame;            | Frame           |                         |
| mnLastKeyFrameId       | unsigned int    | 上一个关键帧的id        |
| mnLastRelocFrameId;    | unsigned int    | 上一个重定位帧的id      |
|                        |                 |                         |
|                        |                 |                         |
|                        |                 |                         |
|                        |                 |                         |

