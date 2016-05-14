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
#define HOUGH_TRACK_TIME 200
#define JUDGE_CENTER_THRESHOLD 1
#define JUDGE_RADIUS_THRESHOLD 1

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
  static int trackCnt = 0;
  static int track_itor = 0;
  NSLog(@"ALEPH_DEBUG: Hough Count = %d\n", hough_cnt++);
  Mat img_gray, img_gray_hough;
  cvtColor(image,img_gray,CV_RGB2GRAY);
  if ( hough_cnt < HOUGH_DETECT_TIME ){
    cvtColor(image,img_gray_hough,CV_RGB2GRAY);
    GaussianBlur( img_gray_hough, img_gray_hough, cv::Size(9, 9), 2, 2 );
    vector<Vec3f> t_circles;
    HoughCircles( img_gray_hough, t_circles, CV_HOUGH_GRADIENT, 1, img_gray_hough.rows/8, 200, 5, 0, 20 );
    bool bj = true;
    for (int j = 0; j< (int)t_circles.size(); j++){
      bj = true;
      for (int i = 0; i < (int)circles.size() && bj; i ++) {
        NSLog(@"ALEPH_DEBUG: j=%d, i=%d, V1 = %f, J1 = %f, V2 = %f, J2 = %f\n",j , i, sqrt(((circles[i][0]-t_circles[j][0])*(circles[i][0]-t_circles[j][0]))+((circles[i][1]-t_circles[j][1])*(circles[i][1]-t_circles[j][1]))), circles[i][2]/JUDGE_CENTER_THRESHOLD, fabs(circles[i][2] - t_circles[j][2]), (double)JUDGE_RADIUS_THRESHOLD);
        if(sqrt(((circles[i][0]-t_circles[j][0])*(circles[i][0]-t_circles[j][0]))+((circles[i][1]-t_circles[j][1])*(circles[i][1]-t_circles[j][1]))) < circles[i][2]/JUDGE_CENTER_THRESHOLD && fabs(circles[i][2] - t_circles[j][2]) < JUDGE_RADIUS_THRESHOLD ){
          NSLog(@"ALEPH_DEBUG: ==MATCHED==\n");
          bj = false;
          circles[i][0] = t_circles[j][0];
          circles[i][1] = t_circles[j][1];
          circles[i][2] = t_circles[j][2];
          //t_circles.erase(t_circles.begin()+j);
          //j--;
        }
      }
      if (bj){
        NSLog(@"ALEPH_DEBUG: ==INSTERT==\n");
        circles.push_back(*new Vec3f(t_circles[j][0],t_circles[j][1],t_circles[j][2]));
      }
    }
    NSLog(@"ALEPH_DEBUG: circles.size() = %d\n", (unsigned int)circles.size());
    putText(image, cv::String([[NSString stringWithFormat:@"%d", (int)circles.size()] UTF8String]), cv::Point(100,100), FONT_HERSHEY_SIMPLEX, 0.5, cvScalar(0,255,255));
  }
  else {
    NSLog(@"ALEPH_DEBUG: ==TRACKING==\n");
    NSLog(@"ALEPH_DEBUG: track_itor = %d, circles.size = %d\n", track_itor, (int)circles.size());
    NSLog(@"ALEPH_DEBUG: trackCnt = %d\n", trackCnt);
    if(track_itor < (int)circles.size()) {
      trackCnt++;
      NSLog(@"ALEPH_DEBUG: beginInit = %d\n", beginInit);
      if (beginInit) {
        if (cmtTracker != NULL) {
          delete cmtTracker;
        }
        cv::Point center(cvRound(circles[track_itor][0]), cvRound(circles[track_itor][1]));
        int radius = cvRound(circles[track_itor][2]);
        initCTBox = cv::Rect((center.x - 2*radius), (center.y - 2*radius), 0, 0);
        initCTBox.width = 4*radius;
        initCTBox.height= 4*radius;
        cmtTracker = new cmt::CMT();
        cmtTracker->initialize(img_gray,initCTBox);
        NSLog(@"cmt track init!");
        startTracking = true;
        beginInit = false;
        NSLog(@"init HOUGH Box: %d,%d,%d,%d",initCTBox.x,initCTBox.y,initCTBox.width,initCTBox.height);
      }

      rectangle(image, cv::Point(initCTBox.x, initCTBox.y), cv::Point((initCTBox.x + initCTBox.width), (initCTBox.y + initCTBox.height)), Scalar(0,0,255));

      if (startTracking) {
        NSLog(@"cmt process...");
        cmtTracker->processFrame(img_gray);

        for(size_t i = 0; i < cmtTracker->points_active.size(); i++){
          circle(image, cmtTracker->points_active[i], 2, Scalar(255,0,0));
        }

        RotatedRect rect = cmtTracker->bb_rot;
        Point2f vertices[4];
        rect.points(vertices);
        for (int i = 0; i < 4; i++) {
          line(image, vertices[i], vertices[(i+1)%4], Scalar(255,0,255));
        }
      }

      if (trackCnt >= HOUGH_TRACK_TIME){
        beginInit = true;
        startTracking = false;
        trackCnt = 0;
      }
    }
  }








  for( size_t i = 0; i < circles.size(); i++ )
  {
    cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
    int radius = cvRound(circles[i][2]);
    // circle center
    circle( image, center, 3, Scalar(0,255,0), -1, 8, 0 );
    // circle outline
    circle( image, center, radius, Scalar(0,0,255), 3, 8, 0 );
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
