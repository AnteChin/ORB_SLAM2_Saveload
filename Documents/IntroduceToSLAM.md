## System Architecture

![orb-slam2 threads](threads.png)

ORB-SLAM2 contains three threads. <br>

(1) **Tracking**
This part extracts the ORB features from the image, perform pose estimation based on the previous frame, or initialize the pose through global relocation. Then, track the reconstructed local map, optimize the pose, and determine the new key frames according to some rules.

(2) **LocalMapping**
This part includes inserting keyframes, validating recently filtered map points and filtering them, and then generating new map points. Using local bundle adjustments (Local BA) to modify and filtering the inserted keyframes to remove redundant keyframes finally.

(3) **LoopClosing**
This part is divided into two processes: Closed loop detection and closed loop correction. The closed loop detection is first detected using WOB, and then calculate the similar transformation by the Sim3 algorithm. Closed-loop correction is closed-loop fusion and graph optimization of Essential Graph.

## Naming rules

In ORB-SLAM2, the variable naming rules are:
| prefix | meaning |
|--------|---------|
| m | member variable|
| p | pointer|
| b | boolean|
| v | vector |
| n | int |
| s | set |
| l | list|
| e | enum|
| kf| KeyFrame(a class defined in this project)|

## Example Program

We choose an ros example to show how to use orb-slam2 with ros. The source file is [ros_stereo](../ROS/Drone_SLAM/src/ros_stereo.cc).

1. Init ros
    * `ros::init(argc, argv, "RGBD");`
    * `ros::start();`
2. Create SLAM System
    * `ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO, true, true);`
3. Caputer images
    * `ImageGrabber igb(&SLAM);`
    * Get image data by subscribering resource from ros node. `message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/zed/zed_node/left/image_rect_color", 1);`
4. Close SLAM
5. Save

## Create SLAM
> `ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO, true, true);`<br>
> See details in [System](../src/System.cc)

1. set camera type and read setting file 
2. load vocabulary, if the format of vocabulary is txt, it will be converted to bin.
3. create **Key Frame Database**
    
    `mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary)`
4. create **Map**

    `mpMap = new Map();` <br>
    `mpMap` is equals to the two windows you see when the program is running. It contains whole pose data and map data.`MapPoint` and `KeyFrame` are contained in `mpMap`.
5. create **Drawers**. They are 2 windows and used by the Viewer
    
    `mpFrameDrawer = new FrameDrawer(mpMap, bReuseMap);` (keyframe display)<br>
    `mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);` (map display)
6. Initialize the **Tracking Thread**

    `mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer, mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);`
7. Initialize the **Local Mapping Thread** and launch

    `mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR);`<br>
	`mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run, mpLocalMapper);`
8. Initialize the **Loop Closing Thread** and launch

    `mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);`<br>
	`mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);`
9. Set pointers between threads

    In this step, a function `mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());` is called. In tracking thread you can see more details.

## Tracking thread

Enrty: When creating SLAM system, a tracking thread will be created. You can see this in SLAM constructor(in [System](../src/System.cc) file). A pointer named `mpTracker` in SLAM class is used to represent the tracking system. This thread lives in main thread.

**Dataflow**: `main` in [ros_stereo](../ROS/Drone_SLAM/src/ros_stereo.cc) -> `GrabStereo()` in [ros_stereo](../ROS/Drone_SLAM/src/ros_stereo.cc) -> `TrackStereo()` in [System](../src/System.cc) -> `GrabImageStereo()` in [Tracking](../src/Tracking.cc) -> `Track()` in [Tracking](../src/Tracking.cc)
* In `Track()` it will check if the stereo is initialized. If not it will call a call a function to initialize it.
* The assignment of variable `mCurrentFrame` happens in `GrabImageStereo()`. It uses the images from `TrackStereo()` and change them to gray.
* `TrackStereo()` uses the images come from `GrabStereo()`.
* `GrabStereo()` uses the images subscribed from ros.

### Track
Tracking involes three model:
1. **Tracking with motion model**

    Track MapPoints which show up in last frame.This model is used for object that is in uniform motion so that the thread can use previous pose and velocity to estimate current pose. Invalid for object that moves casully. The function is `matcher.SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, ...)`.
2. **Tracking with reference key frame**

    The thread will match current frame with the previous KeyFrame. Bag of words, aka BoW, is used for accelerating.
3. **Relocalization**

    When both upper two function failed, system need to relocalization. The entrance is `bOK = Relocalization();`. At that time, system need to match all key frames to make sure if it can find proper position.

If camera is moving, the thread will use motion model to estimate the pose of current frame based on last frame. If it is invalid, the thread will choose reference key frame model. If it doesn't work well, the thread will enlarge the matching range of reference key frame to gain more matches. If both these two models fail, tracking lost.

These three models are aim to get an aproximate value of camera pose. It will be used in local map to get more match points. And Bundle Adjustment will be used to optimize the pose.

### Initialization
Initialization in `Tracking::StereoInitialization()` function. This function is invoked in `Tracking::Track()` function when the thread finds the system didn't initialize.

ORB-SLAM2 contains a file call `Initializer.h`. However, it is only used by Mono camera. Initializations for Mono and Stereo are different and this documentation only introduces Stereo. This is because Mono cannot calculate depth and its scale is uncertain, which means extra computations are needed.

The process in this function is:

1. Find a frame that contians more than 500 key points as the first key frame.
2. Initialize this frame and set initial pose.
3. Generate the initial KeyFrame `pkFini` based on current frame(KeyFrame are different with Frame).
4. Insert `pkFini` into the map.
5. For every key points `i` in current frame:

        z = depth data in i
        if z is positive (which means i is a good point)
            get 3D coodinate x3D of i by unproject
            create Map Point pNewMP by x3D
            add atributes to pNewMp
            add pNewMP into pkFini
            compute descriptor of pNewMp and update
            add pNewMp to mpMap(map in tracking thread)
            links current frame and pNewMp
6. Insert `pkFini` into local map
7. Record
8. Draw map and visuilization

### Track step:
Per Frame -> extract ORB feature points -> Position estimation `R` & `t` based on the previous frame -> track local map and optimize pose -> decide if add key frame

### GrabImageStereo
> Input: images from left and right sight, timestamp<br> Output: Rotation Matrix for world to camera coordinate 

Converting images to gray, discard all color data. It create a frame based on two gray images from left and right sight, and assign it to `mCurrentFrame`. In function `Track()`, the Rotation Martix will be created and assign to `mCurrentFrame`, and then the function return it.

## Frame and KeyFrame

Every image is stored as a Frame. In Frmae, the image is divided into grid and you can change the parameters of grid in file `Frame.h`.

Every Frame contains 3 private matrixs, which respectively represent the rotation matrix from world to camera and camera to world, and a translation matrix. Frame also contains a public parameter named `mTcw` to represent the camera pose. Upper 3 private matrixs are assigned by function `SetPose` and `UpdatePoseMatrices`.

In ORB-SLAM2, Frame and KeyFrame are two different classes. Only frames that satisfy some condition will be created as key frames. KeyFrame contains Frame, 3D points of map and BoW. 

There is a pointer inside KeyFrames points to mpMap, which is stored in Tracking thread and used to reprensent the map you see, and a pointer points to mpKeyFrameDB, which is stored in Tracking thread as well. 

## Map

### MapPoints
MapPoints captured in one key frame are held as a point set. We need to record id of each MapPoint in the every frame. Meanwhile, one MapPoint can be captured in several frames, we need to record this MapPoint's id in thoes frames.

MapPoint need to complete two calculation.
1. `pNewMP->ComputeDistinctiveDescriptors();`
    
    Select the most discriminant descriptor in feature points that detect this MapPoint, amd set it as the descriptor of the map point;

2. `pNewMP->UpdateNormalAndDepth();`

    UpDate one MapPoint's normal and depth. Preparing for merging in the later stages.


### Save and Load Map
The project use [serialization](https://www.boost.org/doc/libs/1_70_0/libs/serialization/doc/archives.html) to generate a binary file. [This page](https://theboostcpplibraries.com/boost.serialization-archive) desribe details about this libiray.

Then in your cpp file, you need to set the fifth boolean parameter of slam constructor. If it's ture it means saving map is valid.
