//
//  ViewController.m
//  VisionSeg
//
//  Created by FloodSurge on 15/7/14.
//  Copyright (c) 2015年 FloodSurge. All rights reserved.
//

#import "ViewController.h"
#import <AVFoundation/AVFoundation.h>
#import <opencv2/opencv.hpp>

#import <opencv2/imgproc/types_c.h>
#import <opencv2/imgcodecs/ios.h>
#import <opencv2/imgproc/imgproc_c.h>
#import <opencv2/videoio/cap_ios.h>

#import "CompressiveTracker.h"
#import "CMT.h"
#import "TLD.h"
#import "color_tracker.h"
#import "Camshift.h"

#import "StruckTracker.h"
#import "Config.h"


#define RATIO  640.0/568.0
#define HOUGH_DETECT_TIME 10
#define HOUGH_TRACK_TIME 0
#define HOUGH_TRACK_DIEDAI 200
#define JUDGE_CENTER_THRESHOLD 0.5
#define JUDGE_RADIUS_THRESHOLD 1
#define RADIUS_RATE 1.25
#define TEXT_NUM_OFFSET 10
#define TEXT_SIZE 0.35
#define COLOR_NON_SELECTED_REC Scalar(0,255,0)

#define _HOUGH_TRACK_MODE_CMT
//#define _HOUGH_TRACK_MODE_CT

using namespace cv;
using namespace cv::colortracker;
using namespace std;
using namespace tld;


typedef enum {
    CT_TRACKER,
    TLD_TRACKER,
    CMT_TRACKER,
    COLOR_TRACKER,
    CAMSHIFT_TRACKER,
    STRUCK_TRACKER,
    HOUGH_TRACKER,
}TrackType;

unsigned int hough_cnt = 0;
bool cmtReset = true;
vector<Vec3f> circles;
vector<cv::Rect> circ_box;
vector<int> circles_init_y;


#ifdef _HOUGH_TRACK_MODE_CMT
  vector<cmt::CMT *> circ_trackers;
#endif
#ifdef _HOUGH_TRACK_MODE_CT
  vector<CompressiveTracker *> circ_trackers;
#endif

@interface ViewController ()<CvVideoCameraDelegate>
{
    CGPoint rectLeftTopPoint;
    CGPoint rectRightDownPoint
    ;

    TrackType trackType;

    // CT Tracker
    CompressiveTracker *ctTracker;
    cv::Rect selectBox;
    cv::Rect initCTBox;
    cv::Rect box;
    bool beginInit;
    bool startTracking;

    // CMT Tracker
    cmt::CMT *cmtTracker;

    // TLD Tracker
    tld::TLD *tldTracker;

    // Color Tracker
    ColorTracker *colorTracker;

    // Camshift Tracker
    Camshift *camshift;

    // Struck Tracker
    StruckTracker *struckTracker;


}
@property (weak, nonatomic) IBOutlet UIImageView *imageView;
@property (nonatomic,strong) CvVideoCamera *videoCamera;
@end

@implementation ViewController

- (void)viewDidLoad {
    [super viewDidLoad];
    // Do any additional setup after loading the view, typically from a nib.

    self.imageView.frame = CGRectMake(0, 0, 568, 480 * 568/640.0);
    self.videoCamera = [[CvVideoCamera alloc] initWithParentView:self.imageView];
    self.videoCamera.delegate = self;
    self.videoCamera.defaultAVCaptureDevicePosition = AVCaptureDevicePositionBack;
    self.videoCamera.defaultAVCaptureSessionPreset = AVCaptureSessionPreset640x480;
    self.videoCamera.defaultAVCaptureVideoOrientation = AVCaptureVideoOrientationLandscapeLeft;
    self.videoCamera.defaultFPS = 30;
    [self.videoCamera start];

    rectLeftTopPoint = CGPointZero;
    rectRightDownPoint = CGPointZero;

    beginInit = false;
    startTracking = false;

    trackType = CMT_TRACKER;
}

- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

- (void)reset
{
    startTracking = false;
    beginInit = false;

    rectLeftTopPoint.x = 0;
    rectRightDownPoint.x = 0;
}

- (IBAction)COLOR:(id)sender
{
    trackType = COLOR_TRACKER;
    [self reset];
}

- (IBAction)CT:(id)sender
{
    trackType = CT_TRACKER;
    [self reset];
}
- (IBAction)TLD:(id)sender
{
    trackType = TLD_TRACKER;
    [self reset];
}

- (IBAction)CMT:(id)sender
{
    trackType = CMT_TRACKER;
    [self reset];
}
- (IBAction)Camshift:(id)sender {
    trackType = CAMSHIFT_TRACKER;
    [self reset];
}

- (IBAction)Struck:(id)sender
{
    trackType = STRUCK_TRACKER;
    [self reset];
}

- (IBAction)Hough:(id)sender{
    trackType = HOUGH_TRACKER;
    hough_cnt = 0;
    circles.clear();
    circ_box.clear();
    circles_init_y.clear();
    beginInit = true;
    startTracking = false;
    [self reset];
}

- (IBAction)Next:(id)sender{
    cmtReset = true;
}

- (void)touchesBegan:(NSSet *)touches withEvent:(UIEvent *)event
{
    startTracking = false;
    beginInit = false;
    UITouch *aTouch  = [touches anyObject];
    rectLeftTopPoint = [aTouch locationInView:self.imageView];

    NSLog(@"touch in :%f,%f",rectLeftTopPoint.x,rectLeftTopPoint.y);
    rectRightDownPoint = CGPointZero;
    selectBox = cv::Rect(rectLeftTopPoint.x * RATIO,rectLeftTopPoint.y * RATIO,0,0);
}

- (void)touchesMoved:(NSSet *)touches withEvent:(UIEvent *)event
{
    UITouch *aTouch  = [touches anyObject];
    rectRightDownPoint = [aTouch locationInView:self.imageView];

    //NSLog(@"touch move :%f,%f",rectRightDownPoint.x,rectRightDownPoint.y);


}

- (void)touchesEnded:(NSSet *)touches withEvent:(UIEvent *)event
{
    UITouch *aTouch  = [touches anyObject];
    rectRightDownPoint = [aTouch locationInView:self.imageView];

    NSLog(@"touch end :%f,%f",rectRightDownPoint.x,rectRightDownPoint.y);
    selectBox.width = abs(rectRightDownPoint.x * RATIO - selectBox.x);
    selectBox.height = abs(rectRightDownPoint.y * RATIO - selectBox.y);
    beginInit = true;
    initCTBox = selectBox;

}

- (void)processImage:(cv::Mat &)image
{
    if (rectLeftTopPoint.x != 0 && rectLeftTopPoint.y != 0 && rectRightDownPoint.x != 0 && rectRightDownPoint.y != 0 && !beginInit && !startTracking) {

        rectangle(image, cv::Point(rectLeftTopPoint.x * RATIO,rectLeftTopPoint.y * RATIO), cv::Point(rectRightDownPoint.x * RATIO,rectRightDownPoint.y * RATIO), Scalar(0,0,255));
    }

    switch (trackType) {
        case CT_TRACKER:
            [self compressiveTracking:image];
            break;
        case CMT_TRACKER:
            [self cmtTracking:image];
            break;
        case TLD_TRACKER:
            [self tldTracking:image];
            break;
        case COLOR_TRACKER:
            [self colorTracking:image];
        case CAMSHIFT_TRACKER:
            [self camshiftTracking:image];
        case STRUCK_TRACKER:
            [self struckTracking:image];
        case HOUGH_TRACKER:
            [self houghTracking:image];
        default:
            break;
    }

}


- (void)houghTracking:(cv::Mat &)image
{
  static int track_itor = 0;
  NSLog(@"ALEPH_DEBUG: Hough Count = %d\n", hough_cnt++);
  NSLog(@"ALEPH_DEBUG: time = %f\n", [[NSDate date] timeIntervalSince1970]);
  Mat img_gray, img_gray_hough;
  /*转为灰度图*/
  cvtColor(image,img_gray,CV_RGB2GRAY);
  /*侦测阶段*/
  if ( hough_cnt < HOUGH_DETECT_TIME ){
    /*转为另一份灰度图*/
    cvtColor(image,img_gray_hough,CV_RGB2GRAY);
    /*模糊化，去除噪点*/
    GaussianBlur( img_gray_hough, img_gray_hough, cv::Size(9, 9), 2, 2 );
    /*找圆*/
    vector<Vec3f> t_circles;
    HoughCircles( img_gray_hough, t_circles, CV_HOUGH_GRADIENT, 1, img_gray_hough.rows/18, 200, 5, 0, 20 );
    /*与之前找到的合并*/
    bool bj = true;
    for (int j = 0; j< (int)t_circles.size(); j++){
      bj = true;
      for (int i = 0; i < (int)circles.size() && bj; i ++) {
        NSLog(@"ALEPH_DEBUG: j=%d, i=%d, V1 = %f, J1 = %f, V2 = %f, J2 = %f\n",j , i, sqrt(((circles[i][0]-t_circles[j][0])*(circles[i][0]-t_circles[j][0]))+((circles[i][1]-t_circles[j][1])*(circles[i][1]-t_circles[j][1]))), circles[i][2]/JUDGE_CENTER_THRESHOLD, fabs((circles[i][2] - t_circles[j][2])/circles[i][2]), (double)JUDGE_RADIUS_THRESHOLD);
        if(sqrt(((circles[i][0]-t_circles[j][0])*(circles[i][0]-t_circles[j][0]))+((circles[i][1]-t_circles[j][1])*(circles[i][1]-t_circles[j][1]))) < circles[i][2]/JUDGE_CENTER_THRESHOLD && fabs((circles[i][2] - t_circles[j][2])/circles[i][2]) < JUDGE_RADIUS_THRESHOLD ) {
          NSLog(@"ALEPH_DEBUG: ==MATCHED==\n");
          bj = false;
          circles[i][0] = t_circles[j][0];
          circles[i][1] = t_circles[j][1];
          circles[i][2] = t_circles[j][2];
          circ_box[i].x = circles[i][0] - RADIUS_RATE*circles[i][2];
          circ_box[i].y = circles[i][1] - RADIUS_RATE*circles[i][2];
          circ_box[i].width = circ_box[i].height = 2*RADIUS_RATE*circles[i][2];
          circles_init_y[i] = t_circles[j][1];
        }
      }
      /*匹配失败，找到了新的圆*/
      if (bj){
        NSLog(@"ALEPH_DEBUG: ==INSERT==\n");
        circles.push_back(*new Vec3f(t_circles[j][0],t_circles[j][1],t_circles[j][2]));
        circ_box.push_back(cv::Rect(t_circles[j][0] - RADIUS_RATE*t_circles[j][2], t_circles[j][1] - RADIUS_RATE*t_circles[j][2], 2*RADIUS_RATE*t_circles[j][2], 2*RADIUS_RATE*t_circles[j][2]));
        circles_init_y.push_back(t_circles[j][1]);
      }
    }
    NSLog(@"ALEPH_DEBUG: circles.size = %d\n", (int)circles.size());
    putText(image, cv::String([[NSString stringWithFormat:@"%d", (int)circles.size()] UTF8String]), cv::Point(100,100), FONT_HERSHEY_SIMPLEX, 0.5, cvScalar(0,255,255));
    beginInit = true;
  }
  /*追踪阶段*/
  else {
    NSLog(@"ALEPH_DEBUG: ==TRACKING==\n");
    NSLog(@"ALEPH_DEBUG: circles.size = %d\n", (int)circles.size());
    if (beginInit) {
      NSLog(@"ALEPH_DEBUG: ==INIT TRACK==\n");
      /*vector重置*/
      if (circ_trackers.size() != 0) {
        for(track_itor = 0; track_itor < circ_trackers.size(); track_itor++) {
          if(NULL != circ_trackers[track_itor]) {
            delete circ_trackers[track_itor];
            circ_trackers[track_itor] = NULL;
          }
        }
        circ_trackers.clear();
      }
      /*vector填充*/
      int tracklen = (int)circ_box.size();
      for(track_itor = 0; track_itor < tracklen; track_itor++) {
        NSLog(@"ALEPH_DEBUG: --init tracker %d--\n", track_itor);
        #ifdef _HOUGH_TRACK_MODE_CMT
          circ_trackers.push_back(new cmt::CMT());
          circ_trackers[track_itor] -> initialize(img_gray, circ_box[track_itor]);
        #endif
        #ifdef _HOUGH_TRACK_MODE_CT
          circ_trackers.push_back(new CompressiveTracker);
          circ_trackers[track_itor] -> init(img_gray, circ_box[track_itor]);
        #endif
      }
      NSLog(@"track init!");
      startTracking = true;
      beginInit = false;
    }
    if (startTracking) {
      NSLog(@"track process...");
      int tracklen = (int)circ_box.size();
      for(track_itor = 0; track_itor < tracklen; track_itor++) {
        #ifdef _HOUGH_TRACK_MODE_CMT
          circ_trackers[track_itor]->processFrame(img_gray);
          double y = 0;
          for(size_t i = 0; i < circ_trackers[track_itor]->points_active.size(); i++){
            circle(image, circ_trackers[track_itor]->points_active[i], 2, Scalar(255,0,0));
            y += circ_trackers[track_itor]->points_active[i].y;
          }
          y /= circ_trackers[track_itor]->points_active.size();
          RotatedRect rect = circ_trackers[track_itor]->bb_rot;
          Point2f vertices[4];
          rect.points(vertices);
          for (int i = 0; i < 4; i++) {
            line(image, vertices[i], vertices[(i+1)%4], COLOR_NON_SELECTED_REC);
          }
          //putText(image, cv::String([[NSString stringWithFormat:@"%d", track_itor] UTF8String]), cv::Point(vertices[0].x, vertices[0].y - TEXT_NUM_OFFSET), FONT_HERSHEY_SIMPLEX, TEXT_SIZE, cvScalar(0,255,255));
          putText(  image,  /*输出图像*/
                    cv::String([[NSString stringWithFormat:@"%d/%d", (int)y, circles_init_y[track_itor]] UTF8String]),  /*输出字符串*/
                    cv::Point(vertices[0].x, vertices[0].y - TEXT_NUM_OFFSET),  /*输出位置，字符串的左下角位于这个点*/
                    FONT_HERSHEY_SIMPLEX, /*字体*/
                    TEXT_SIZE,  /*字体大小*/
                    cvScalar(0,255,255) /*颜色，顺序为BGR*/
                );
        #endif

        #ifdef _HOUGH_TRACK_MODE_CT
          circ_trackers[track_itor]->processFrame(img_gray, circ_box[track_itor]);
          rectangle(image, circ_box[track_itor], COLOR_NON_SELECTED_REC,1);
          putText(image, cv::String([[NSString stringWithFormat:@"%d", track_itor] UTF8String]), cv::Point(circ_box[track_itor].x, circ_box[track_itor].y - TEXT_NUM_OFFSET), FONT_HERSHEY_SIMPLEX, TEXT_SIZE, cvScalar(0,255,255));
        #endif
      }
    }
  }
}



- (void)struckTracking:(cv::Mat &)image
{
    Mat img_gray;
    cvtColor(image,img_gray,CV_BGR2GRAY);
    //cv::flip(img_gray, img_gray, 1);
    if (beginInit) {
        startTracking = true;
        beginInit = false;

        NSString *path = [[NSBundle mainBundle] pathForResource:@"config" ofType:@"txt"];
        string configPath = [path UTF8String];
        Config conf(configPath);
        cout << conf << endl;
        srand(conf.seed);
        if (struckTracker != NULL) {
            delete struckTracker;
        }

        struckTracker = new StruckTracker(conf);

        FloatStruckRect floatRect;
        floatRect.Set(initCTBox.x, initCTBox.y, initCTBox.width, initCTBox.height);
        struckTracker->Initialise(img_gray, floatRect);
        NSLog(@"Struck tracker init");
    }

    if (startTracking) {

        NSLog(@"Struck Tracker process...");
        struckTracker->Track(img_gray);
        FloatStruckRect bb = struckTracker->GetBB();
        cv::Rect bb_rot;
        bb_rot.x = (int)bb.XMin();
        bb_rot.y = (int)bb.YMin();
        bb_rot.width = (int)bb.Width();
        bb_rot.height = (int)bb.Height();

        rectangle(image, bb_rot, Scalar(125,255,0),1);
    }
}

- (void)camshiftTracking:(cv::Mat &)image
{
    if (beginInit) {
        if (camshift != NULL) {
            delete camshift;
        };
        camshift = new Camshift();
        camshift->initialize(image, initCTBox);
        NSLog(@"Camshift track init!");
        startTracking = true;
        beginInit = false;
    }

    if (startTracking) {
        NSLog(@"Camshift Track process...");
        camshift->processFrame(image);

        RotatedRect rect = camshift->objectBox;
        Point2f vertices[4];
        rect.points(vertices);
        for (int i = 0; i < 4; i++)
        {
            line(image, vertices[i], vertices[(i+1)%4], Scalar(255,0,255));
        }
    }
}

- (void)colorTracking:(cv::Mat &)image
{
    if (beginInit) {
        ColorTrackerParameters params;
        params.visualization = 0;
        params.init_pos.x = initCTBox.x + initCTBox.width/2;
        params.init_pos.y = initCTBox.y + initCTBox.height/2;
        params.wsize = initCTBox.size();

        if (colorTracker != NULL) {
            delete colorTracker;
        }

        colorTracker = new ColorTracker(params);
        colorTracker->init_tracking();

        NSLog(@"color track init!");
        startTracking = true;
        beginInit = false;
    }

    if (startTracking) {
        NSLog(@"color track process...");
        cv::Rect rect = colorTracker->track_frame(image);


    }


}


- (void)tldTracking:(cv::Mat &)image
{
    Mat img_gray;
    cvtColor(image,img_gray,CV_BGR2GRAY);
    if (beginInit) {
        if (tldTracker != NULL) {
            tldTracker->release();
            delete tldTracker;
        }

        tldTracker = new tld::TLD();
        tldTracker->detectorCascade->imgWidth = img_gray.cols;
        tldTracker->detectorCascade->imgHeight = img_gray.rows;
        tldTracker->detectorCascade->imgWidthStep = img_gray.step;

        tldTracker->selectObject(img_gray, &initCTBox);
        NSLog(@"tld track init!");
        startTracking = true;
        beginInit = false;
    }

    if (startTracking) {
        NSLog(@"tld process...");
        tldTracker->processImage(image);

        if (tldTracker->currBB != NULL) {

            rectangle(image, *tldTracker->currBB, Scalar(0,255,0),1);
        }

    }
}


- (void)cmtTracking:(cv::Mat &)image
{
    Mat img_gray;
    cvtColor(image,img_gray,CV_RGB2GRAY);

    if (beginInit) {
        if (cmtTracker != NULL) {
            delete cmtTracker;
        }
        cmtTracker = new cmt::CMT();
        cmtTracker->initialize(img_gray,initCTBox);
        NSLog(@"cmt track init!");
        startTracking = true;
        beginInit = false;
    }

    if (startTracking) {
        NSLog(@"cmt process...");
        cmtTracker->processFrame(img_gray);

        for(size_t i = 0; i < cmtTracker->points_active.size(); i++)
        {
            circle(image, cmtTracker->points_active[i], 2, Scalar(255,0,0));
        }


        RotatedRect rect = cmtTracker->bb_rot;
        Point2f vertices[4];
        rect.points(vertices);
        for (int i = 0; i < 4; i++)
        {
            line(image, vertices[i], vertices[(i+1)%4], Scalar(255,0,255));
        }


    }
}

- (void)compressiveTracking:(cv::Mat &)image
{
    Mat img_gray;
    cvtColor(image,img_gray,CV_RGB2GRAY);
    // 初始化选择框，初始化ct算法
    if (beginInit)
    {
        startTracking=true;
        if (ctTracker != NULL) {
            delete ctTracker;
        }
        ctTracker = new CompressiveTracker;
        ctTracker->init(img_gray,initCTBox);
        NSLog(@"init CT Box: %d,%d,%d,%d",initCTBox.x,initCTBox.y,initCTBox.width,initCTBox.height);
        box = initCTBox;
        rectangle(image, initCTBox, Scalar(0,0,255),1);
        beginInit =false;
    }

    //  采用ct算法进行跟踪
    if (startTracking)
    {
        /** if (box.size())
         {

         }*/
        ctTracker->processFrame(img_gray, box);

        rectangle(image, box, Scalar(0,0,255),1);

    }
}

@end
