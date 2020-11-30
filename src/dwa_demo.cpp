
#include<iostream>
#include<vector>
#include<array>
#include<cmath>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include "utility.h"
#include "dwa.h"
#include "stm32_control.h"
cv::Point2i cv_offset(    float x, float y, int image_width=200, int image_height=200)
{
    cv::Point2i output;
    output.x = int(x * 10) + image_width/2;
    output.y = image_height - int(y * 10) - image_height/3;
    return output;
};

//如果目标里障碍物太近 极容易导致原地画圈
int dwa_loop(float meters){
    State start{0.0, 0.0, 1, 0.0, 0.0};
	Point goal{0.0,meters};
	if(meters <= 10.0){
   		 goal.x_ = 0.0; 
		 goal.y_ = meters;
		}
    Obstacle ob{
                        
                        {5.0, 4.0},
                        {5.0, 5.0},
                        //{2.0, 5.0},
                       // {1.0, 5.0},
                        {-5.0, 4.0},
                        {-5.0, 6.0},
                        
                };

    Control u{0.0, 0.0};
    Config config;
    Traj traj;
    traj.push_back(start);

    bool terminal = false;

   // cv::namedWindow("dwa", cv::WINDOW_NORMAL);
    int count = 0;

    Dwa dwa_demo(start, goal, ob, config);
    static int ccn=0;
    cv::Mat final_canvas;
    Traj ltraj;
    State x;
	Obstacle dyn_ob;
		cv::VideoWriter writer;
	writer.open("out_dwa.avi",CV_FOURCC('M', 'J', 'P', 'G'),
        30, //不进行跟踪，定位，只显示、录制时的帧率
        cv::Size(500,500),
        true);
    if (!writer.isOpened())
    {
        return 0;
    }
    while(!dwa_demo.stepOnceToGoal(&ltraj, &x,&dyn_ob)){
        traj.push_back(x);
	printf("control (v,w) (%.1f,%.1f)\n ",dwa_demo.calculated_u.v_,dwa_demo.calculated_u.w_);
    cmd_send2(dwa_demo.calculated_u.v_, dwa_demo.calculated_u.w_);
	if(ccn ++ >= 50)
	{
	 Point obs{1.0,5.0};
	 dwa_demo.obs_.push_back(obs);
	}
        // visualization
        cv::Mat bg(500,500, CV_8UC3, cv::Scalar(255,255,255));
        cv::circle(bg, cv_offset(goal.x_, goal.y_, bg.cols, bg.rows),
                   3, cv::Scalar(255,0,0), 5);
				   
        for(unsigned int j=0; j<dyn_ob.size(); j++){
            cv::circle(bg, cv_offset(dyn_ob[j].x_, dyn_ob[j].y_, bg.cols, bg.rows),
                       5, cv::Scalar(0,0,0), -1);
        }
        for(unsigned int j=0; j<ltraj.size(); j++){
            cv::circle(bg, cv_offset(ltraj[j].x_, ltraj[j].y_, bg.cols, bg.rows),
                       2, cv::Scalar(0,255,0), -1);
        }
        cv::circle(bg, cv_offset(x.x_, x.y_, bg.cols, bg.rows),
                   3, cv::Scalar(0,0,255), 5);
// toDegree degree = radian/pi*180;
		printf("car state:(%.1f,%.1f,%.1f),%.1f \n",x.x_,x.y_,x.theta_,x.theta_/3.14*180);
        cv::arrowedLine(
                bg,
                cv_offset(x.x_, x.y_, bg.cols, bg.rows),
                cv_offset(x.x_ + std::cos(x.theta_), x.y_ + std::sin(x.theta_), bg.cols, bg.rows),
                cv::Scalar(255,0,255),
                2);

		
       // cv::imwrite("dwa.jpg", bg);
         

        // std::string int_count = std::to_string(count);
        // cv::imwrite("/home/pi/dwa/"+int_count+".jpg", bg);
        writer.write(bg);
        count++;
        final_canvas = bg;
    }


    if (!final_canvas.empty()) {
        for(unsigned int j=0; j<traj.size(); j++){
            cv::circle(final_canvas, cv_offset(traj[j].x_, traj[j].y_, final_canvas.cols, final_canvas.rows),
                       2, cv::Scalar(0,0,255), -1);
        }

        cv::imwrite("dwa2.jpg", final_canvas);
        
    }
    writer.release();
    return 1;



}