- (void)houghTracking:(cv::Mat &)image
{
  static int track_itor = 0;
  static int tracklen = 0;
  hough_cnt++;
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
      /*对每个点，查看是否有与上一帧匹配的点，若匹配上则认为是同一个点*/
      for (int i = 0; i < (int)circles.size() && bj; i ++) {
        if(sqrt(((circles[i][0]-t_circles[j][0])*(circles[i][0]-t_circles[j][0]))+((circles[i][1]-t_circles[j][1])*(circles[i][1]-t_circles[j][1]))) < circles[i][2]/JUDGE_CENTER_THRESHOLD && fabs((circles[i][2] - t_circles[j][2])/circles[i][2]) < JUDGE_RADIUS_THRESHOLD ) {
          /*如果找到了相同的点，则更新老点的信息*/
          bj = false;
          circles[i][0] = t_circles[j][0];
          circles[i][1] = t_circles[j][1];
          circles[i][2] = t_circles[j][2];
          circ_box[i].x = circles[i][0] - RADIUS_RATE*circles[i][2];
          circ_box[i].y = circles[i][1] - RADIUS_RATE*circles[i][2];
          circ_box[i].width = circ_box[i].height = 2*RADIUS_RATE*circles[i][2];
          circles_curr_y[i] = t_circles[j][1];
          circles_last_time[i] = [[NSDate date] timeIntervalSince1970];
        }
      }
      /*匹配失败，找到了新的圆*/
      if (bj){
        circles.push_back(*new Vec3f(t_circles[j][0],t_circles[j][1],t_circles[j][2]));
        circ_box.push_back(cv::Rect(t_circles[j][0] - RADIUS_RATE*t_circles[j][2], t_circles[j][1] - RADIUS_RATE*t_circles[j][2], 2*RADIUS_RATE*t_circles[j][2], 2*RADIUS_RATE*t_circles[j][2]));
        circles_init_y.push_back(t_circles[j][1]);
        circles_curr_y.push_back(t_circles[j][1]);
        circles_init_time.push_back([[NSDate date] timeIntervalSince1970]);
        circles_last_time.push_back([[NSDate date] timeIntervalSince1970]);
      }
    }
    /*识别阶段结束，开始追踪*/
    beginInit = true;
  }
  /*追踪阶段*/
  else {
    if (beginInit) {
      /*清空追踪器*/
      if (circ_trackers.size() != 0) {
        for(track_itor = 0; track_itor < circ_trackers.size(); track_itor++) {
          if(NULL != circ_trackers[track_itor]) {
            delete circ_trackers[track_itor];
            circ_trackers[track_itor] = NULL;
          }
        }
        circ_trackers.clear();
      }
      /*初始化迭代器*/
      track_itor = 0;
      /*计算圆的个数*/
      tracklen = (int)circ_box.size();
      /*结束初始化开始阶段*/
      beginInit = false;
      /*开始追踪*/
      startTracking = true;
    }
    if (show_box.size() < circ_box.size()) {
      /*依次初始化每个追踪器*/
      /*由于每次初始化时有时间间隔，因此依据前几次识别的y坐标预估出此时的y坐标*/
      circ_box[track_itor].y += ([[NSDate date] timeIntervalSince1970] - circles_last_time[track_itor]) * ((circles_curr_y[track_itor] - circles_init_y[track_itor]) / (circles_last_time[track_itor] - circles_init_time[track_itor]));
      /*存储初始化完成的点*/
      show_box.push_back(circ_box[track_itor]);
      /*创建追踪对象*/
      circ_trackers.push_back(new cmt::CMT());
      /*这个结构体用于计算速度*/
      struct trackY tempty;
      for(int ti = 0; ti < TRACK_Y_SIZE; ti++) {
        tempty.y[ti] = 0;
        tempty.t[ti] = 0;
      }
      circles_track_y.push_back(tempty);
      /*初始化追踪对象*/
      circ_trackers[track_itor] -> initialize(img_gray, circ_box[track_itor]);
      /*初始化下一个油滴*/
      track_itor++;
    }
    /*初始化成功，开始正式追踪*/
    if (startTracking) {
    	/*计算追踪对象个数*/
      int tracklen = (int)show_box.size();
      /*依次更新每个对象的位置*/
      for(track_itor = 0; track_itor < tracklen; track_itor++) {
      	/*处理图像*/
        circ_trackers[track_itor]->processFrame(img_gray);
        /*计算此时的y坐标*/
        double y = 0;
        for(size_t i = 0; i < circ_trackers[track_itor]->points_active.size(); i++){
          circle(image, circ_trackers[track_itor]->points_active[i], 2, Scalar(255,0,0));
          y += circ_trackers[track_itor]->points_active[i].y;
        }
        y /= circ_trackers[track_itor]->points_active.size();
        /*更新短时间内的速度*/
        for(int ti = TRACK_Y_SIZE - 2; ti >=0 ; ti--) {
          circles_track_y[track_itor].y[ti+1] = circles_track_y[track_itor].y[ti];
          circles_track_y[track_itor].t[ti+1] = circles_track_y[track_itor].t[ti];
        }
        circles_track_y[track_itor].y[0] = y;
        circles_track_y[track_itor].t[0] = [[NSDate date] timeIntervalSince1970];
        /*用逐差法计算短时间内的速度*/
        double v = 0;
        for(int ti = 0; ti < TRACK_Y_SIZE/2; ti++ ){
          v += (circles_track_y[track_itor].y[ti] - circles_track_y[track_itor].y[ti+(TRACK_Y_SIZE/2)])/(circles_track_y[track_itor].t[ti] - circles_track_y[track_itor].t[ti+(TRACK_Y_SIZE/2)]);
        }
        v /= (TRACK_Y_SIZE/2);
        v = v / SCREEN_PIXELS * SCREEN_SCALE ;
        /*计算出包围的框*/
        RotatedRect rect = circ_trackers[track_itor]->bb_rot;
        Point2f vertices[4];
        rect.points(vertices);
        /*判断速度，并用相应的颜色画出框*/
        for (int i = 0; i < 4; i++) {
          if ( v >= V_MIN && v <= V_MAX ) {
            line(image, vertices[i], vertices[(i+1)%4], COLOR_SELECTED_REC);
          }
          else {
            line(image, vertices[i], vertices[(i+1)%4], COLOR_NON_SELECTED_REC);
          }
        }
        /*显示速度*/
        putText(  image,  /*输出图像*/
                  cv::String([[NSString stringWithFormat:@"%.1f", v*1000] UTF8String]),  /*输出字符串*/
                  cv::Point(vertices[0].x, vertices[0].y - TEXT_NUM_OFFSET),  /*输出位置，字符串的左下角位于这个点*/
                  FONT_HERSHEY_SIMPLEX, /*字体*/
                  TEXT_SIZE,  /*字体大小*/
                  cvScalar(0,255,255) /*颜色，顺序为BGR*/
              );
      }
    }
  }
}