
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <sstream>

#include <opencv2/opencv.hpp>
#include <libobsensor/ObSensor.hpp>

int main(){
  try{
    ob::Context ctx;
    auto list=ctx.queryDeviceList();
    if(!list || list->deviceCount()==0){ std::cerr << "no Orbbec device" << std::endl; return 2; }
    ob::Pipeline pipe(list->getDevice(0));
    auto profiles=pipe.getStreamProfileList(OB_SENSOR_DEPTH);
    auto profile=profiles->getProfile(0)->as<ob::VideoStreamProfile>();
    auto cfg=std::make_shared<ob::Config>();
    cfg->enableStream(profile);
    pipe.start(cfg);

    const std::string win="Orbbec Depth Fast Preview";
    cv::namedWindow(win, cv::WINDOW_NORMAL);
    cv::resizeWindow(win, 960, 600);
    auto t0=std::chrono::steady_clock::now();
    int frames=0;
    double fps=0.0;
    while(true){
      auto fs=pipe.waitForFrames(1000);
      if(!fs || !fs->depthFrame()) continue;
      auto depth=fs->depthFrame();
      int w=(int)depth->width(), h=(int)depth->height();
      float scale=depth->getValueScale();
      cv::Mat d16(h,w,CV_16UC1,(void*)depth->data());
      cv::Mat d8, color;
      // Fixed close-range visualization for tabletop tools. Values above 1200mm saturate.
      d16.convertTo(d8, CV_8U, 255.0/1200.0);
      cv::applyColorMap(d8, color, cv::COLORMAP_JET);
      color.setTo(cv::Scalar(0,0,0), d16 == 0);
      frames++;
      auto now=std::chrono::steady_clock::now();
      double dt=std::chrono::duration<double>(now-t0).count();
      if(dt >= 1.0){ fps=frames/dt; frames=0; t0=now; }
      std::ostringstream text;
      text << w << "x" << h << " depth-only " << fps << " fps  q/ESC quit";
      cv::putText(color, text.str(), cv::Point(20,32), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255,255,255), 2);
      cv::imshow(win, color);
      int key=cv::waitKey(1);
      if(key == 27 || key == 'q' || key == 'Q') break;
    }
    pipe.stop();
    return 0;
  }catch(const ob::Error &e){ std::cerr << "ob_error " << e.getMessage() << std::endl; return 10; }
  catch(const std::exception &e){ std::cerr << "std_error " << e.what() << std::endl; return 11; }
}
