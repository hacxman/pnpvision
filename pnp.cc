#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace std;
using namespace cv;

char* source_window = "Source image";

class Machine {
  private:
    float x,y,z;
    float sx, sy, sz;
    float dpiX, dpiY;

    Rect cam_rect;
    Rect b_pos;
  public:
    Mat board;
    Machine(float x0, float y0, float z0,
        float sx, float sy, float sz,
        Rect cr) : x(x0), y(y0), z(z0),
                   sx(sx), sy(sy), sz(sz),
                   dpiX(150), dpiY(150), cam_rect(cr) {};
    void moveX(float cm) {x+=cm;};
    void moveY(float cm) {y+=cm;};
    void moveZ(float cm) {z+=cm;};

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

    Mat getCamFrame() {
      int c_s_x = cam_rect.width * (board.size().width / b_pos.width);
      int c_s_y = cam_rect.height * (board.size().height / b_pos.height);
      int o_s_x = cam_rect.width * dpiX;
      int o_s_y = cam_rect.height * dpiY;
      Mat src = board;
//      cout << c_s_x << " " << c_s_y << endl;
//      cout << o_s_x << " " << o_s_y << endl;
      Mat warp_dst = Mat::zeros( c_s_x, c_s_y, src.type() );
      Mat warp_out = Mat::zeros( o_s_x, o_s_y, src.type() );
      Point2f pt_center = Point2f(board.size().width/b_pos.width * (x - b_pos.x), board.size().height/b_pos.height * (y - b_pos.y));
      cout << pt_center <<endl;
      getRectSubPix(board, Size(c_s_x, c_s_y), pt_center, warp_dst);
      resize(warp_dst, warp_out, Size(o_s_x, o_s_y), 0,0, INTER_CUBIC);

      return warp_out;
    }
};

int main(int argc, char *argv[]) {
  //start in center, in 1m^3
  Machine m = Machine(50,50,50, 100, 100, 100, Rect(-1, 1, 1.5, 1.5));
  Mat src_gray, src = imread(argv[1], 1);

  cvtColor(src, src_gray, CV_BGR2GRAY);
  m.addBoard(src_gray, Rect(45, 45, 2*16, 2*9));

//  namedWindow( source_window, CV_WINDOW_AUTOSIZE );


  Mat old_frame;
  while (true) {
    Mat warp_out = m.getCamFrame();
    if (old_frame.size() != warp_out.size()) old_frame = warp_out.clone();
//    imshow(source_window, src_gray);
    vector<Point2f> pts;
    vector<Point2f> old_pts;
    imshow("asdfasd", old_frame);
    goodFeaturesToTrack(warp_out, old_pts, 640, 0.55, 4);
//    old_pts.insert(old_pts.end(), pts.begin(), pts.end());
    vector<unsigned char> status;
    vector<float> err;

    calcOpticalFlowPyrLK(old_frame, warp_out, old_pts, pts, status, err);
    auto it3=status.begin();
    auto it4=err.begin();
    int cnt = 0;
    Point2f acc = Point2f(0,0);
    for (vector<Point2f>::iterator it=old_pts.begin(), it2=pts.begin() ; it != old_pts.end(); it++, it2++, it3++, it4++) {
      //circle(warp_out, *it, 5, 0);
      //circle(warp_out, *it, 5, 0);
      if (*it3) {
        line(warp_out, *it, *it2, 0, 1);
        acc += (*it - *it2);
        cnt++;
        cout << "err:"; cout << (*it4) << endl;
      }
    }
    acc = Point2f(acc.x/cnt, acc.y/cnt);
    cout << "GLOBAL VECT:" << acc << endl;
    old_pts = pts;
    for (vector<unsigned char>::iterator it=status.begin(); it != status.end(); it++) {
      cout << ((*it) ? "*" : " ");
    }
    cout << "|" << endl;
    cout << old_pts << endl;
//      Canny(warp_out, warp_out, 15, 70);
    imshow("chuj", warp_out);
//    Mat bn = m.board.clone();
//    rectangle(bn, pt_center - Point2f(c_s_x / 2, c_s_y / 2), pt_center + Point2f(c_s_x / 2, c_s_y / 2), 255, 2);
//    imshow("chujo", bn);
    old_frame = warp_out.clone();

    m.moveX(0.01);

    switch (waitKey(1000)) {
      case 27: exit(0);
      case 'w': m.moveY(-0.03); break;
      case 's': m.moveY(0.03); break;
      case 'a': m.moveX(-0.03); break;
      case 'd': m.moveX(0.03); break;
    }
  };

  return EXIT_SUCCESS;
};
