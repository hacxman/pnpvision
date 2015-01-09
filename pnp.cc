#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "code/GHT.cpp"

using namespace std;
using namespace cv;

char* source_window = "Source image";

class Machine {
  private:
    float x,y,z;
    float sx, sy, sz;

    Rect b_pos;
  public:
    Rect cam_rect;
    vector<Point2f> points;
    vector<Point2f> points2;
    float dpiX, dpiY;
    Mat board;
    Machine(float x0, float y0, float z0,
        float sx, float sy, float sz,
        Rect cr) : x(x0), y(y0), z(z0),
                   sx(sx), sy(sy), sz(sz),
                   dpiX(150), dpiY(150), cam_rect(cr) {};
    void moveX(float cm) {x+= ((x+cm) > b_pos.x+b_pos.width-cam_rect.width) || (x+cm < b_pos.x+cam_rect.width) ? 0: cm;};
    void moveY(float cm) {y+= ((y+cm) > b_pos.y+b_pos.height-cam_rect.height) || (y+cm < b_pos.y+cam_rect.height) ? 0: cm;};
    void moveZ(float cm) {z+=cm;};
    Point2f getPos() {return Point2f(x,y); };

    void renderMachine() {
      int dpi = 35;
      Mat mach = Mat::zeros(sx*dpi/2.54, sy*dpi/2.54, board.type());
      float bsx = (dpi / 2.54) / (board.size().width / b_pos.width);
      float bsy = (dpi / 2.54) / (board.size().height / b_pos.height);
      Rect roi(b_pos.x * dpi/2.54, b_pos.y * dpi/2.54,
         b_pos.width * dpi/2.54, b_pos.height * dpi/2.54 );
      Mat destRoi = mach(roi);
      Mat resized_board;
      resize(board, resized_board,
          Size(board.size().width*bsx, board.size().height*bsy), 0,0, INTER_CUBIC);

      resized_board.copyTo(destRoi);
      cvtColor(mach, mach, CV_GRAY2RGB);

      for (auto it = points2.begin(); it != points2.end(); it++) {
        Point2i p(((*it).x) * dpi/2.54, (*it).y * dpi/2.54);
        auto _ii = it; _ii++;
        if (_ii==points2.end()) _ii=it;
        line(mach, (*(_ii))*(dpi/2.54), p, Scalar(255,0,0));
      }
      if (points.size()>500000) points.erase(points.begin());
      if (points2.size()>500000) points2.erase(points2.begin());
      for (auto it = points.begin(); it != points.end(); it++) {
        Point2i p(((*it).x) * dpi/2.54, (*it).y * dpi/2.54);
        auto _ii = it; _ii++;
        if (_ii==points.end()) _ii=it;
        line(mach, (*(_ii))*(dpi/2.54), p, Scalar(0,0,255));
      }

      line(mach, Point2f(x*dpi/2.54, 0), Point2f(x*dpi/2.54, y*dpi/2.54), Scalar(255,0,255));
      line(mach, Point2f(0, y*dpi/2.54), Point2f(x*dpi/2.54, y*dpi/2.54), Scalar(255,0,255));
      circle(mach, Point2f(x*dpi/2.54, y*dpi/2.54), 3, Scalar(255,0,255));
      imshow("machine", mach);
      //waitKey();
    }

    void addBoard(Mat pic, Rect pos) {b_pos = pos; board = pic;};

//    Mat drawBox(Rect b) {
//      int c_s_x = cam_rect.width * (board.size().width / b_pos.width);
//      int c_s_y = cam_rect.height * (board.size().height / b_pos.height);
//      int o_s_x = cam_rect.width * dpiX;
//      int o_s_y = cam_rect.height * dpiY;
//
//      Point2f pt_center = Point2f(board.size().width/b_pos.width * (x - b_pos.x), board.size().height/b_pos.height * (y - b_pos.y));
//      Mat o = board.clone();
//      rectangle(o, pt_center - Point2f(o_s_x / 2, o_s_y / 2), pt_center + Point2f(o_s_x / 2, o_s_y / 2));
//      return o;
//    }

    Mat getFrameAt(float posx, float posy) {
      int c_s_x = cam_rect.width * (board.size().width / b_pos.width);
      int c_s_y = cam_rect.height * (board.size().height / b_pos.height);
      int o_s_x = cam_rect.width * dpiX / 2.54;
      int o_s_y = cam_rect.height * dpiY / 2.54;
      Mat src = board;
//      cout << c_s_x << " " << c_s_y << endl;
//      cout << o_s_x << " " << o_s_y << endl;
      Mat warp_dst = Mat::zeros( c_s_x, c_s_y, src.type() );
      Mat warp_out = Mat::zeros( o_s_x, o_s_y, src.type() );
      Point2f pt_center = Point2f(board.size().width/b_pos.width * (posx - b_pos.x), board.size().height/b_pos.height * (posy - b_pos.y));
      cout << pt_center <<endl;
      getRectSubPix(board, Size(c_s_x, c_s_y), pt_center, warp_dst);
      resize(warp_dst, warp_out, Size(o_s_x, o_s_y), 0,0, INTER_CUBIC);

      return warp_out;
    }

    Mat getCamFrame() {
      return getFrameAt(x,y);
    }
    void goCenter() {x=sx/2; y=sy/2;};
};

int main(int argc, char *argv[]) {
  //start in center, in 1m^3
  srand(time(0));
  float __r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
  float __r2 = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
  Machine m = Machine(57+(__r*3),55+(__r2*3),50, 100, 100, 100, Rect(-1, 1, 2, 2));
  //Point2f my_pos(m.getPos().x, m.getPos().y);
  Point2f my_pos(-1,-1);
  Mat src_gray, src = imread(argv[1], 1);

  cvtColor(src, src_gray, CV_BGR2GRAY);
  Rect board_rect = Rect(45, 45, 16*2, 9*2);
  m.addBoard(src_gray, board_rect);

//  namedWindow( source_window, CV_WINDOW_AUTOSIZE );

  Mat old_frame;
  Point2f g(1,0);
  while (true) {
    m.renderMachine();
    switch (waitKey(1)) {
      case 27: exit(0);
      case 'w': m.moveY(-0.03); break;
      case 's': m.moveY(0.03); break;
      case 'a': m.moveX(-0.03); break;
      case 'd': m.moveX(0.03); break;
    }

    Mat warp_out = m.getCamFrame();
    Mat to_match; cvtColor(warp_out, to_match, CV_GRAY2RGB);
    if (my_pos.x < 0) {
      //resize(to_match, to_match,
      //    Size(128, 128), 0.5,0.5, INTER_CUBIC);


    Mat_<float> lala(warp_out.size());
    matchTemplate(src, to_match, lala, CV_TM_CCOEFF_NORMED);
    double minv; Point minIdx;
    minMaxLoc(lala, NULL, &minv, NULL, &minIdx);
    imshow("lala", lala);
//    cout<<"LALA";
//    cout << minIdx<<endl;

        my_pos = Point2f(board_rect.x+((float)minIdx.x/src.size().width)*board_rect.width+m.cam_rect.width/2,
                     board_rect.y+((float)minIdx.y/src.size().height)*board_rect.height+m.cam_rect.height/2);
/*      std::vector<std::vector<Vec2i>> mmm = runMatching(to_match, src);
      if (mmm.size() > 0 && mmm[mmm.size()/2].size() > 0) {
        Point2f _p(mmm[mmm.size()/2][mmm[mmm.size()/2].size()/2]);
        //Point2f _p(mmm[0][0]);
        _p = Point2f(board_rect.x+(_p.x/src.size().width)*board_rect.width,
                     board_rect.y+(_p.y/src.size().height)*board_rect.height);
        //Point2f _p2(mmm[mmm.size()/2+1][mmm[0].size()/2+1]);
//        Point2f _p2(mmm[mmm.size()-1][mmm[0].size()-1]);
//        _p2 = Point2f(board_rect.x+(_p.x/src.size().width)*board_rect.width,
//                     board_rect.y+(_p.y/src.size().height)*board_rect.height);
        cout << _p << endl;
        my_pos = _p;
      }*/

      old_frame = warp_out.clone();
    }
//    cout << mmm[to_match.size().width][to_match.size().height] << endl;
    if (old_frame.size() != warp_out.size()) old_frame = warp_out.clone();
//    imshow(source_window, src_gray);
    vector<Point2f> pts;
    vector<Point2f> old_pts;
    //imshow("asdfasd", old_frame);
    goodFeaturesToTrack(warp_out, old_pts, 640, 0.35, 4);
//    old_pts.insert(old_pts.end(), pts.begin(), pts.end());
    vector<unsigned char> status;
    vector<float> err;
    vector<Mat> pyr_a, pyr_b;
    buildOpticalFlowPyramid(old_frame, pyr_a, Size(22,22), 6);
    old_frame = warp_out.clone();
    buildOpticalFlowPyramid(warp_out, pyr_b, Size(22,22), 6);
    try {
      calcOpticalFlowPyrLK(pyr_a, pyr_b, old_pts, pts, status, err);
    } catch (Exception &e) {
      cout << e.what() << endl;
//      m.goCenter();
      my_pos = Point2f(-1,-1);
      //my_pos = Point2f(50, 50);
    }
//    calcOpticalFlowFarneback(old_frame, warp_out, pts, 0.5, 4, 5, 8, 5,1.5,OPTFLOW_FARNEBACK_GAUSSIAN);
    auto it3=status.begin();
    auto it4=err.begin();
    int cnt;
    cnt = 0;
    Point2f acc = Point2f(0,0);
    float global_err = 0;
    float last_err = 10000;
    for (vector<Point2f>::iterator it=old_pts.begin(), it2=pts.begin() ; it != old_pts.end(); it++, it2++, it3++, it4++) {
      //circle(warp_out, *it, 5, 0);
      //circle(warp_out, *it, 5, 0);
      if (*it3) {
        if (last_err > (*it4)) {
          line(warp_out, *it, *it2, 0, 1);
          Point2f _p= (*it - *it2);
          if (isnan(_p.x) || isnan(_p.y)) continue;
          acc += _p;
          cnt++;
          cout << "err:"; cout << (*it4) << endl;
          global_err += (*it4);
          last_err = (*it4);
        }
      }
    }
    global_err /= cnt;
    acc = Point2f((acc.x/cnt) / (m.dpiX/2.54),
        (acc.y/cnt) / (m.dpiY/2.54));
    cout << "GLOBAL VECT:" << acc << endl;
    if (!(isnan(acc.x) || isnan(acc.y))) {
      my_pos += acc;
    }
    cout << "GLOBAL POS:" << my_pos << endl;
    cout << "GLOBAL err:" << global_err<< endl;


    Mat my_view = m.getFrameAt(my_pos.x, my_pos.y);
    Mat_<float> lala(my_view.size());
    matchTemplate(src_gray, my_view, lala, CV_TM_CCOEFF_NORMED);
    double minv; Point minIdx;
    minMaxLoc(lala, &minv, NULL, NULL, &minIdx);
    if (global_err > 20) {waitKey(3); my_pos = Point2f(-1,-1);};
    m.points.push_back(my_pos);
    m.points2.push_back(m.getPos());
    old_pts = pts;
    for (vector<unsigned char>::iterator it=status.begin(); it != status.end(); it++) {
      cout << ((*it) ? "*" : " ");
    }
    cout << "|" << endl;
    cout << old_pts << endl;
//      Canny(warp_out, warp_out, 15, 70);
    //imshow("warp_out", warp_out);
//    Mat bn = m.board.clone();
//    rectangle(bn, pt_center - Point2f(c_s_x / 2, c_s_y / 2), pt_center + Point2f(c_s_x / 2, c_s_y / 2), 255, 2);
//    imshow("chujo", bn);
    //old_frame = warp_out.clone();
    float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    if (r < 0.05) {
      float r1 = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
      float r2 = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
      g = Point2f(r1-0.5, r2-0.5);
    }
    r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    m.moveX(g.x*0.15+0.02*(r-0.5));
    r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    m.moveY(g.y*0.15+0.02*(r-0.5));


  };

  return EXIT_SUCCESS;
};
