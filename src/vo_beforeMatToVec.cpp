#include <opencv2/core/utility.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <math.h>
#include <gmpxx.h>
#include <gmp.h>
#include <boost/multiprecision/cpp_int.hpp>

#include "vo.hpp"
#include "MYORB.h"

#define USE_MYORB 1// 1: MYORB, 0 : opencv ORB
#define MUL pow(2.0, 24)

using namespace cv;
using namespace std;
namespace mp = boost::multiprecision;
//
const int maxLineDiff = 30;
const int sobelSize = 3;
const double sobelScale = 1./8.;

const bool pyramid_on = false;
const int feature_iter_num = 2;
const int feature_corr_num = 12;

const double max_value_1 = 9223372036854775808.0; // 64bits = 2^63
//const double max_value_1 = 36893488147419103232.0; // 66bits = 2^65
//const double max_value_2 = 73786976294838206464.0; // 67bits = 2^66
//const double max_value_2 = 147573952589676412928.0; // 68bits = 2^67
const double max_value_2 = 1180591620717411303424.0; // 71bits = 2^70
//const double max_value_3 = 37778931862957161709568.0; // 76bits = 2^75
const double max_value_3 = pow(2.0, 127); // 128bits = 2^127
const double max_value_4 = pow(2.0, 70); 
const double max_value_5 = pow(2.0, 178); 
//const double max_value_6 = pow(2.0, 80); 
const double max_value_6 = pow(2.0, 59); 
const double max_value_3d = pow(2.0, 41); 
const double max_value_lsm_f1 = pow(2.0, 41); 
const double max_value_lsm_f2 = pow(2.0, 41); 
const double max_value_lsm_f3 = pow(2.0, 41); 
const double max_value_lsm_f4 = pow(2.0, 45); 
const double max_value_lsm_f5 = pow(2.0, 51); //key 
const double max_value_theta = pow(2.0, 34); //for design_ware sincos 

double trunc(double num){
	//return (num<0)?ceil(num):floor(num);
	return floor(num);
}

double trunc_3d(double num){
       if(abs(num) > max_value_3d)
       {
           cout << "num: " << num << endl;
           cout << "In trunc_3d " << endl;
           exit(1);
       }
       if(num > 0)
	   return floor(num);
       else
	   return ceil(num);
}

double trunc_lsm_f1(double num){
       if(abs(num) > max_value_lsm_f1)
       {
           cout << "num: " << num << endl;
           cout << "In trunc_lsm_f1 " << endl;
           exit(1);
       }
       if(num > 0)
	   return floor(num);
       else
	   return ceil(num);
}

double trunc_lsm_f2(double num){
       if(abs(num) > max_value_lsm_f2)
       {
           cout << "num: " << num << endl;
           cout << "In trunc_lsm_f2 " << endl;
           exit(1);
       }
       if(num > 0)
	   return floor(num);
       else
	   return ceil(num);
}

double trunc_lsm_f3(double num){
       if(abs(num) > max_value_lsm_f3)
       {
           cout << "num: " << num << endl;
           cout << "In trunc_lsm_f3 " << endl;
           exit(1);
       }
       if(num > 0)
	   return floor(num);
       else
	   return ceil(num);
}

double trunc_lsm_f4(double num){
       if(abs(num) > max_value_lsm_f4)
       {
           cout << "num: " << num << endl;
           cout << "In trunc_lsm_f4 " << endl;
           exit(1);
       }
       if(num > 0)
	   return floor(num);
       else
	   return ceil(num);
}

double trunc_lsm_f5(double num){
       if(abs(num) > max_value_lsm_f5)
       {
           cout << "num: " << num << endl;
           cout << "In trunc_lsm_f5 " << endl;
           exit(1);
       }
       if(num > 0)
	   return floor(num);
       else
	   return ceil(num);
}

double trunc_theta(double num){
       if(abs(num) > max_value_theta)
       {
           cout << "num: " << num << endl;
           cout << "theta " << endl;
           exit(1);
       }
       if(num > 0)
	   return floor(num);
       else
	   return ceil(num);
}

double trunc1(double num){
       if(abs(num) > max_value_1)
       {
           cout << "num: " << num << endl;
           exit(1);
       }
       if(num > 0)
	   return floor(num);
       else
	   return ceil(num);
}

double trunc2(double num){
       if(abs(num) > max_value_2)
       {
           cout << "trunc2 num: " << num << endl;
           exit(1);
       }
       if(num > 0)
	   return floor(num);
       else
	   return ceil(num);
}

double trunc3(double num){
       if(abs(num) > max_value_3)
       {
           cout << "trunc3 num: " << num << endl;
           exit(1);
       }
       if(num > 0)
	   return floor(num);
       else
	   return ceil(num);
}

double trunc4(double num){
       if(abs(num) > max_value_4)
       {
           cout << "trunc4 num: " << num << endl;
           exit(1);
       }
       if(num > 0)
	   return floor(num);
       else
	   return ceil(num);
}

double trunc5(double num){
       if(abs(num) > max_value_5)
       {
           cout << "trunc5 num: " << num << endl;
           exit(1);
       }
       if(num > 0)
	   return floor(num);
       else
	   return ceil(num);
}

double trunc6(double num){
       if(abs(num) > max_value_6)
       {
           cout << "trunc6 num: " << num << endl;
           exit(1);
       }
       if(num > 0)
	   return floor(num);
       else
	   return ceil(num);
}

static inline
void setDefaultIterCounts(Mat& iterCounts)
{
    if(pyramid_on)
        //iterCounts = Mat(Vec4i(7,7,7,10));
        //iterCounts = Mat(Vec4i(7,7,7,7));
        iterCounts = Mat(Vec4i(7,7,7,7));
    else
        //iterCounts = Mat(Vec4i(7));
        iterCounts = Mat(Vec4i(8));
        //iterCounts = Mat(Vec4i(10));
        //iterCounts = Mat(Vec4i(15));
        //iterCounts = Mat(Vec4i(3));
}

static inline
void setDefaultMinGradientMagnitudes(Mat& minGradientMagnitudes)
{
    if(pyramid_on)
        minGradientMagnitudes = Mat(Vec4f(10,10,10,10));
    else
        minGradientMagnitudes = Mat(Vec4f(10));
}

static
void normalsComputer(const Mat& points3d, int rows, int cols, Mat & maskNormal, Mat & normals) 
{
    normals.create(points3d.size(), CV_MAKETYPE(points3d.depth(), 3));
    maskNormal = Mat(points3d.size(), CV_8UC1, Scalar(0));
//   int a = 1;     //pylin
//   ofstream u_index("./u_index.txt"); //pylin
//   ofstream v_index("./v_index.txt"); //pylin
//   for (int y = 0; y < rows; ++y)
//   {
//     for (int x = 0; x < cols; ++x)
//     {
//         u_index << hex << setw(3) << setfill('0') << x << endl;   //pylin
//         v_index << hex << setw(3) << setfill('0') << y << endl;   //pylin
//     }
//   }
    int test_valid = 1;
    ifstream if_0("./debug_unit_n_x_SW.txt");
    bool check_near_0to1 = if_0.good();
    ifstream if_1("./debug_normal_1_p_z_0_SW.txt");
    bool check_near_1to2 = if_1.good();

    if (check_near_0to1 == 0) {
        // ofstream of_0("./debug_unit_n_x_SW.txt");
        for (int y = 0; y < rows - 1; ++y)
        {
            for (int x = 0; x < cols - 1; ++x)
            {
        	    Vec3d du = points3d.at<Vec3d>(y,x+1) - points3d.at<Vec3d>(y,x);
        	    Vec3d dv = points3d.at<Vec3d>(y+1,x) - points3d.at<Vec3d>(y,x);
                //normals.at<Vec3d>(y,x) = du.cross(dv); //MUL^2
                normals.at<Vec3d>(y,x)[0] = trunc2(du[1] * dv[2] / MUL) - trunc2(du[2] * dv[1] / MUL);
                normals.at<Vec3d>(y,x)[1] = trunc2(du[2] * dv[0] / MUL) - trunc2(du[0] * dv[2] / MUL);
                normals.at<Vec3d>(y,x)[2] = trunc2(du[0] * dv[1] / MUL) - trunc2(du[1] * dv[0] / MUL);

                // of_0 << long(points3d.at<Vec3d>(y,x)[0]) << endl; //p_x_0
                // of_0 << long(points3d.at<Vec3d>(y,x)[1]) << endl; //p_y_0
                // of_0 << long(points3d.at<Vec3d>(y,x)[2]) << endl; //p_z_0
                // of_0 << long(points3d.at<Vec3d>(y,x+1)[0]) << endl; //p_x_u
                // of_0 << long(points3d.at<Vec3d>(y,x+1)[1]) << endl; //p_y_u
                // of_0 << long(points3d.at<Vec3d>(y,x+1)[2]) << endl; //p_z_u
                // of_0 << long(points3d.at<Vec3d>(y+1,x)[0]) << endl; //p_x_v
                // of_0 << long(points3d.at<Vec3d>(y+1,x)[1]) << endl; //p_y_v
                // of_0 << long(points3d.at<Vec3d>(y+1,x)[2]) << endl; //p_z_v
                // of_0 << long(du[0]) << endl; //du_x
                // of_0 << long(du[1]) << endl; //du_y
                // of_0 << long(du[2]) << endl; //du_z
                // of_0 << long(dv[0]) << endl; //dv_x
                // of_0 << long(dv[1]) << endl; //dv_y
                // of_0 << long(dv[2]) << endl; //dv_z
                // of_0 << long(normals.at<Vec3d>(y,x)[0]) << endl;    //normal_0_x
                // of_0 << long(normals.at<Vec3d>(y,x)[1]) << endl;    //normal_0_y
                // of_0 << long(normals.at<Vec3d>(y,x)[2]) << endl;    //normal_0_z

                // if(a == 1){  //pylin
                //     cout << "----------------------------------------------------------------" << endl;
                //     cout << "a = " << long(du[0]) << ", " << long(du[1]) << ", " << long(du[2]) << endl;
                //     cout << "b = " << long(dv[0]) << ", " << long(dv[1]) << ", " << long(dv[2]) << endl;
                //     cout << "normals_x = " << long(normals.at<Vec3d>(y,x)[0]) << endl;
                //     cout << "normals_y = " << long(normals.at<Vec3d>(y,x)[1]) << endl;
                //     cout << "normals_z = " << long(normals.at<Vec3d>(y,x)[2]) << endl;
                //     cout << "----------------------------------------------------------------" << endl;
                //     a = 0;
                // }

                if(normals.at<Vec3d>(y,x)[0] != 0 || normals.at<Vec3d>(y,x)[1] != 0 || normals.at<Vec3d>(y,x)[2] != 0)
                {
                    maskNormal.at<uchar>(y,x) = 255;
                    double norm = trunc2(sqrt(normals.at<Vec3d>(y,x)[0]*normals.at<Vec3d>(y,x)[0] + normals.at<Vec3d>(y,x)[1]*normals.at<Vec3d>(y,x)[1] +normals.at<Vec3d>(y,x)[2]*normals.at<Vec3d>(y,x)[2])); //MUL^2
                        // of_0 << long(normals.at<Vec3d>(y,x)[0]) << endl;    //normal_0_x
                        // of_0 << long(normals.at<Vec3d>(y,x)[1]) << endl;    //normal_0_y
                        // of_0 << long(normals.at<Vec3d>(y,x)[2]) << endl;    //normal_0_z
                        
                        normals.at<Vec3d>(y,x)[0] = trunc1(normals.at<Vec3d>(y,x)[0] * MUL / norm);
                        normals.at<Vec3d>(y,x)[1] = trunc1(normals.at<Vec3d>(y,x)[1] * MUL / norm);
                        normals.at<Vec3d>(y,x)[2] = trunc1(normals.at<Vec3d>(y,x)[2] * MUL / norm);

                        // of_0 << long(normals.at<Vec3d>(y,x)[0]) << endl;    //unit_n_x
                        // of_0 << long(normals.at<Vec3d>(y,x)[1]) << endl;    //unit_n_y
                        // of_0 << long(normals.at<Vec3d>(y,x)[2]) << endl;    //unit_n_z
                        // of_0 << long(norm) << endl;    //norm
                }
            }
        }
    }
    else if (check_near_1to2 == 0) {
        // ofstream of_1("./debug_normal_1_p_z_0_SW.txt");
        for (int y = 0; y < rows - 1; ++y)
        {
            for (int x = 0; x < cols - 1; ++x)
            {
	            Vec3d du = points3d.at<Vec3d>(y,x+1) - points3d.at<Vec3d>(y,x);
	            Vec3d dv = points3d.at<Vec3d>(y+1,x) - points3d.at<Vec3d>(y,x);
                //normals.at<Vec3d>(y,x) = du.cross(dv); //MUL^2
                normals.at<Vec3d>(y,x)[0] = trunc2(du[1] * dv[2] / MUL) - trunc2(du[2] * dv[1] / MUL);
                normals.at<Vec3d>(y,x)[1] = trunc2(du[2] * dv[0] / MUL) - trunc2(du[0] * dv[2] / MUL);
                normals.at<Vec3d>(y,x)[2] = trunc2(du[0] * dv[1] / MUL) - trunc2(du[1] * dv[0] / MUL);

                // of_1 << long(points3d.at<Vec3d>(y,x)[0]) << endl; //p_x_0
                // of_1 << long(points3d.at<Vec3d>(y,x)[1]) << endl; //p_y_0
                // of_1 << long(points3d.at<Vec3d>(y,x)[2]) << endl; //p_z_0
                // of_1 << long(points3d.at<Vec3d>(y,x+1)[0]) << endl; //p_x_u
                // of_1 << long(points3d.at<Vec3d>(y,x+1)[1]) << endl; //p_y_u
                // of_1 << long(points3d.at<Vec3d>(y,x+1)[2]) << endl; //p_z_u
                // of_1 << long(points3d.at<Vec3d>(y+1,x)[0]) << endl; //p_x_v
                // of_1 << long(points3d.at<Vec3d>(y+1,x)[1]) << endl; //p_y_v
                // of_1 << long(points3d.at<Vec3d>(y+1,x)[2]) << endl; //p_z_v

                // if(a == 1){  //pylin
                //     cout << "----------------------------------------------------------------" << endl;
                //     cout << "a = " << long(du[0]) << ", " << long(du[1]) << ", " << long(du[2]) << endl;
                //     cout << "b = " << long(dv[0]) << ", " << long(dv[1]) << ", " << long(dv[2]) << endl;
                //     cout << "normals_x = " << long(normals.at<Vec3d>(y,x)[0]) << endl;
                //     cout << "normals_y = " << long(normals.at<Vec3d>(y,x)[1]) << endl;
                //     cout << "normals_z = " << long(normals.at<Vec3d>(y,x)[2]) << endl;
                //     cout << "----------------------------------------------------------------" << endl;
                //     a = 0;
                // }
                if(normals.at<Vec3d>(y,x)[0] != 0 || normals.at<Vec3d>(y,x)[1] != 0 || normals.at<Vec3d>(y,x)[2] != 0)
                {
                    maskNormal.at<uchar>(y,x) = 255;
                    double norm = trunc2(sqrt(normals.at<Vec3d>(y,x)[0]*normals.at<Vec3d>(y,x)[0] + normals.at<Vec3d>(y,x)[1]*normals.at<Vec3d>(y,x)[1] +normals.at<Vec3d>(y,x)[2]*normals.at<Vec3d>(y,x)[2])); //MUL^2
                        normals.at<Vec3d>(y,x)[0] = trunc1(normals.at<Vec3d>(y,x)[0] * MUL / norm);
                        normals.at<Vec3d>(y,x)[1] = trunc1(normals.at<Vec3d>(y,x)[1] * MUL / norm);
                        normals.at<Vec3d>(y,x)[2] = trunc1(normals.at<Vec3d>(y,x)[2] * MUL / norm);
                }
            }
        }
    }
    else {
        // ofstream of_2("./debug_normal_2_SW.txt");
    for (int y = 0; y < rows - 1; ++y)
    {
        for (int x = 0; x < cols - 1; ++x)
        {
    	    Vec3d du = points3d.at<Vec3d>(y,x+1) - points3d.at<Vec3d>(y,x);
    	    Vec3d dv = points3d.at<Vec3d>(y+1,x) - points3d.at<Vec3d>(y,x);
            //normals.at<Vec3d>(y,x) = du.cross(dv); //MUL^2
            normals.at<Vec3d>(y,x)[0] = trunc2(du[1] * dv[2] / MUL) - trunc2(du[2] * dv[1] / MUL);
            normals.at<Vec3d>(y,x)[1] = trunc2(du[2] * dv[0] / MUL) - trunc2(du[0] * dv[2] / MUL);
            normals.at<Vec3d>(y,x)[2] = trunc2(du[0] * dv[1] / MUL) - trunc2(du[1] * dv[0] / MUL);

            // if(a == 1){  //pylin
            //     cout << "----------------------------------------------------------------" << endl;
            //     cout << "a = " << long(du[0]) << ", " << long(du[1]) << ", " << long(du[2]) << endl;
            //     cout << "b = " << long(dv[0]) << ", " << long(dv[1]) << ", " << long(dv[2]) << endl;
            //     cout << "normals_x = " << long(normals.at<Vec3d>(y,x)[0]) << endl;
            //     cout << "normals_y = " << long(normals.at<Vec3d>(y,x)[1]) << endl;
            //     cout << "normals_z = " << long(normals.at<Vec3d>(y,x)[2]) << endl;
            //     cout << "----------------------------------------------------------------" << endl;
            //     a = 0;
            // }

            if(normals.at<Vec3d>(y,x)[0] != 0 || normals.at<Vec3d>(y,x)[1] != 0 || normals.at<Vec3d>(y,x)[2] != 0)
            {
                maskNormal.at<uchar>(y,x) = 255;
                double norm = trunc2(sqrt(normals.at<Vec3d>(y,x)[0]*normals.at<Vec3d>(y,x)[0] + normals.at<Vec3d>(y,x)[1]*normals.at<Vec3d>(y,x)[1] +normals.at<Vec3d>(y,x)[2]*normals.at<Vec3d>(y,x)[2])); //MUL^2
                    normals.at<Vec3d>(y,x)[0] = trunc1(normals.at<Vec3d>(y,x)[0] * MUL / norm);
                    normals.at<Vec3d>(y,x)[1] = trunc1(normals.at<Vec3d>(y,x)[1] * MUL / norm);
                    normals.at<Vec3d>(y,x)[2] = trunc1(normals.at<Vec3d>(y,x)[2] * MUL / norm);
            }
        }
    }
    }
}

RgbdFrame::RgbdFrame() : ID(-1)
{}

RgbdFrame::RgbdFrame(const Mat& image_in, const Mat& depth_in, const Mat& mask_in, const Mat& normals_in, int ID_in)
    : ID(ID_in), image(image_in), depth(depth_in), mask(mask_in), normals(normals_in)
{}

RgbdFrame::~RgbdFrame()
{}

void RgbdFrame::release()
{
    ID = -1;
    image.release();    //丟掉image的ptr
    depth.release();    //丟掉depth的ptr
    mask.release();     //丟掉mask的ptr
    normals.release();  //丟掉normals的ptr
    cloud.release();    //丟掉cloud的ptr
}

OdometryFrame::OdometryFrame() : RgbdFrame()
{}

OdometryFrame::OdometryFrame(const Mat& image_in, const Mat& depth_in, const Mat& mask_in, const Mat& normals_in, int ID_in)
    : RgbdFrame(image_in, depth_in, mask_in, normals_in, ID_in)
{}

void OdometryFrame::release()
{
    RgbdFrame::release();
    releasePyramids();
}

void OdometryFrame::releasePyramids()
{
    dI_dx.release();        //丟掉dI_dx的ptr
    dI_dy.release();        //丟掉dI_dy的ptr
    maskDepth.release();    //丟掉maskDepth的ptr
    maskText.release();     //丟掉maskText的ptr
    maskNormal.release();   //丟掉maskNormal的ptr
}


Odometry::Odometry() :
    minDepth(DEFAULT_MIN_DEPTH()),              //OdometryFrame::DEFAULT_MIN_DEPTH() == 0 meter
    maxDepth(DEFAULT_MAX_DEPTH()),              //OdometryFrame::DEFAULT_MAX_DEPTH() == 4*5000 meter
    maxDepthDiff(DEFAULT_MAX_DEPTH_DIFF()),     //OdometryFrame::DEFAULT_MAX_DEPTH_DIFF() == 0.07*5000 meter
    maxPointsPart(DEFAULT_MAX_POINTS_PART()),   //OdometryFrame::DEFAULT_MAX_POINTS_PART() == 1 of [0 1]
    maxTranslation(DEFAULT_MAX_TRANSLATION()),  //OdometryFrame::DEFAULT_MAX_TRANSLATION() == 0.15*5000 meter
    maxRotation(DEFAULT_MAX_ROTATION())         //OdometryFrame::DEFAULT_MAX_ROTATION() == 15 degree

{
    setDefaultIterCounts(iterCounts);
    setDefaultMinGradientMagnitudes(minGradientMagnitudes);
}

Odometry::Odometry(const Mat& _cameraMatrix,
                   double _minDepth, 
                   double _maxDepth, 
                   double _maxDepthDiff,
                   const std::vector<int>& _iterCounts,
                   const std::vector<double>& _minGradientMagnitudes,
                   double _maxPointsPart) :
                   minDepth(_minDepth), 
                   maxDepth(_maxDepth), 
                   maxDepthDiff(_maxDepthDiff),
                   iterCounts(Mat(_iterCounts).clone()),
                   minGradientMagnitudes(Mat(_minGradientMagnitudes).clone()),
                   maxPointsPart(_maxPointsPart),
                   cameraMatrix(_cameraMatrix),
                   maxTranslation(DEFAULT_MAX_TRANSLATION()), 
                   maxRotation(DEFAULT_MAX_ROTATION())
{
    if(iterCounts.empty() || minGradientMagnitudes.empty())
    {
        setDefaultIterCounts(iterCounts);
        setDefaultMinGradientMagnitudes(minGradientMagnitudes);
    }
}

template<typename T>
void
depthTo3dNoMask(const cv::Mat& in_depth, const cv::Mat_<T>& K, cv::Mat& points3d)
{
  const T fx = K(0, 0);
  const T fy = K(1, 1);
  const T ox = K(0, 2);
  const T oy = K(1, 2);

  // Build z
  cv::Mat_<T> z_mat;
  z_mat = in_depth;

  for (int y = 0; y < in_depth.rows; ++y)
  {
    cv::Vec<T, 3>* point = points3d.ptr<cv::Vec<T, 3> >(y);
    const T* depth = z_mat[y];
    for (int x = 0; x < in_depth.cols; ++x, ++point, ++depth)
    {
        T z = *depth;
        (*point)[0] = trunc_3d(trunc_3d(trunc_3d(x*MUL - trunc_3d(ox*MUL)) * z / trunc_3d(fx*MUL)) * MUL);
        (*point)[1] = trunc_3d(trunc_3d(trunc_3d(y*MUL - trunc_3d(oy*MUL)) * z / trunc_3d(fy*MUL)) * MUL);
        (*point)[2] = trunc_3d(z * MUL);
        //(*point)[0] = trunc(trunc(x*MUL - ox*MUL) * z / (fx*MUL));
        //(*point)[1] = trunc(trunc(y*MUL - oy*MUL) * z / (fy*MUL));
        //(*point)[2] = z;
        //if(x==340 && y==240 && z!=0)
        //{
        //    //cout << fixed << fx << endl;
        //    //cout << fixed << fy << endl;
        //    //cout << fixed << ox << endl;
        //    //cout << fixed << oy << endl;
        //    //cout << fixed << x << endl;
        //    //cout << fixed << y << endl;
        //    //cout << fixed << z << endl;
        //    //cout << fixed << trunc_3d(y*MUL - trunc_3d(oy*MUL)) * z << endl;
        //    //cout << fixed << trunc_3d(trunc_3d(y*MUL - trunc_3d(oy*MUL)) * z / trunc_3d(fy*MUL)) << endl;
        //    //cout << fixed << trunc_3d(trunc_3d(trunc_3d(y*MUL - trunc_3d(oy*MUL)) * z / trunc_3d(fy*MUL)) * MUL) << endl;
        //    //exit(1);
        //}
    }
  }
}

void
depthTo3d(InputArray depth_in, InputArray K_in, OutputArray points3d_out)
{
  cv::Mat depth = depth_in.getMat();
  cv::Mat K = K_in.getMat();
  CV_Assert(K.cols == 3 && K.rows == 3 && K.depth() == CV_64F);
  CV_Assert(depth.type() == CV_64FC1);

//   //↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ pylin
//   ifstream debug_idepth0("./debug_depth_in_0.txt");
//   bool debug_check_0 = debug_idepth0.good();
//   ifstream debug_idepth1("./debug_depth_in_1.txt");
//   bool debug_check_1 = debug_idepth1.good();
//   ifstream debug_idepth2("./debug_depth_in_2.txt");
//   bool debug_check_2 = debug_idepth2.good();
//   ifstream debug_idepth3("./debug_depth_in_3.txt");
//   bool debug_check_3 = debug_idepth2.good();
//   ifstream debug_idepth4("./debug_depth_in_4.txt");
//   bool debug_check_4 = debug_idepth2.good();
//   if (debug_check_0 == 0) {
//     ofstream debug_odepth0("./debug_depth_in_0.txt");
//     for (int y = 0; y < depth_in.rows(); y++){
//         for (int x = 0; x < depth_in.cols(); x++){
//             debug_odepth0 << hex << int(depth.at<double>(y,x)) << endl;
//         }
//     }
//   }
//   else if (debug_check_1 == 0) {
//     ofstream debug_odepth1("./debug_depth_in_1.txt");
//     for (int y = 0; y < depth_in.rows(); y++){
//         for (int x = 0; x < depth_in.cols(); x++){
//             debug_odepth1 << hex << int(depth.at<double>(y,x)) << endl;
//         }
//     }
//   }
//   else if (debug_check_2 == 0) {
//     ofstream debug_odepth2("./debug_depth_in_2.txt");
//     for (int y = 0; y < depth_in.rows(); y++){
//         for (int x = 0; x < depth_in.cols(); x++){
//             debug_odepth2 << hex << int(depth.at<double>(y,x)) << endl;
//         }
//     }
//   }
//   else if (debug_check_3 == 0) {
//     ofstream debug_odepth3("./debug_depth_in_3.txt");
//     for (int y = 0; y < depth_in.rows(); y++){
//         for (int x = 0; x < depth_in.cols(); x++){
//             debug_odepth3 << hex << int(depth.at<double>(y,x)) << endl;
//         }
//     }
//   }
//   else if (debug_check_4 == 0) {
//     ofstream debug_odepth4("./debug_depth_in_4.txt");
//     for (int y = 0; y < depth_in.rows(); y++){
//         for (int x = 0; x < depth_in.cols(); x++){
//             debug_odepth4 << hex << int(depth.at<double>(y,x)) << endl;
//         }
//     }
//   }
//   //↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑ pylin

  // Create 3D points in one go.
  points3d_out.create(depth.size(), CV_MAKETYPE(K.depth(), 3));
  cv::Mat points3d = points3d_out.getMat();
  depthTo3dNoMask<double>(depth, K, points3d);
}

static
void MaskGen(const Mat& mask, 
             const Mat& Depth, 
             double minDepth,   //OdometryFrame::DEFAULT_MIN_DEPTH() == 0 meter
             double maxDepth,   //OdometryFrame::DEFAULT_MAX_DEPTH() == 4*5000 meter
             const Mat& Normal_mask,
             Mat& maskDepth)
{
    minDepth = std::max(0.0, minDepth);

    if(!maskDepth.empty())
    {
        CV_Assert(maskDepth.size() == Depth.size());
        CV_Assert(maskDepth.type() == CV_8UC1);
    }
    else
    {
        //Mat maskDepth;
        if(mask.empty())
            maskDepth = Mat(Depth.size(), CV_8UC1, Scalar(255));
        else
            maskDepth = mask.clone();

        Mat levelDepth = Depth.clone();
        //patchNaNs(levelDepth, 0);

        maskDepth &= (levelDepth > minDepth) & (levelDepth < maxDepth);
     
        maskDepth &= Normal_mask;
    }

//   //↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ pylin
//   ifstream debug_imaskdepth0("./debug_maskdepth0.txt");
//   bool debug_check_0 = debug_imaskdepth0.good();
//   ifstream debug_imaskdepth1("./debug_maskdepth1.txt");
//   bool debug_check_1 = debug_imaskdepth1.good();
//   ifstream debug_imaskdepth2("./debug_maskdepth2.txt");
//   bool debug_check_2 = debug_imaskdepth2.good();
//   int a;
//   if (debug_check_0 == 0){
//     ofstream debug_omaskdepth0("./debug_check_maskdepth_0_SW.txt"); //pylin
//     for (int y = 0; y < Depth.rows; y++){
//         for (int x = 0; x < Depth.cols; x++){
//             if(int(maskDepth.at<uchar>(y, x)) == 255) a = 1;
//             else a = 0;
//             debug_omaskdepth0 << a << endl;
//         }
//     }
//   }
//   else if (debug_check_1 == 0){
//     ofstream debug_omaskdepth1("./debug_check_maskdepth_1_SW.txt"); //pylin
//     for (int y = 0; y < Depth.rows; y++){
//         for (int x = 0; x < Depth.cols; x++){
//             if(int(maskDepth.at<uchar>(y, x)) == 255) a = 1;
//             else a = 0;
//             debug_omaskdepth1 << a << endl;
//         }
//     }
//   }
//   else if (debug_check_2 == 0){
//     ofstream debug_omaskdepth2("./debug_check_maskdepth_2_SW.txt"); //pylin
//     for (int y = 0; y < Depth.rows; y++){
//         for (int x = 0; x < Depth.cols; x++){
//             if(int(maskDepth.at<uchar>(y, x)) == 255) a = 1;
//             else a = 0;
//             debug_omaskdepth2 << a << endl;
//         }
//     }
//   }
//   //↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑ pylin
  

}

static
void TexturedMaskGen(const Mat& dI_dx, 
                     const Mat& dI_dy,
                     const double& minGradientMagnitudes, 
                     const Mat& Mask, 
                     double maxPointsPart,
                     Mat& texturedMask)
{
    if(!texturedMask.empty())
    {
        CV_Assert(texturedMask.size() == dI_dx.size());
        CV_Assert(texturedMask.type() == CV_8UC1);
    }
    else
    {
        const double sobelScale2_inv = 1.0 / (double)(sobelScale * sobelScale);     //sobelScale = 1./8.
        const double minScaledGradMagnitude2 = minGradientMagnitudes * minGradientMagnitudes * sobelScale2_inv;
        Mat texturedMask_pre(dI_dx.size(), CV_8UC1, Scalar(0));
        for(int y = 0; y < dI_dx.rows; y++)
        {
            const short *dIdx_row = dI_dx.ptr<short>(y);
            const short *dIdy_row = dI_dy.ptr<short>(y);
            uchar *texturedMask_row = texturedMask_pre.ptr<uchar>(y);
            for(int x = 0; x < dI_dx.cols; x++)
            {
                double magnitude2 = static_cast<double>(dIdx_row[x] * dIdx_row[x] + dIdy_row[x] * dIdy_row[x]);
                if(magnitude2 >= minScaledGradMagnitude2)
                    texturedMask_row[x] = 255;
            }
        }
        texturedMask = texturedMask_pre & Mask;

    }
}

static inline
void checkImage(const Mat& image)
{
    if(image.empty())
        CV_Error(Error::StsBadSize, "Image is empty.");
    if(image.type() != CV_8UC1)
        CV_Error(Error::StsBadSize, "Image type has to be CV_8UC1.");
}

static inline
void checkDepth(const Mat& depth, const Size& imageSize)
{
    if(depth.empty())
        CV_Error(Error::StsBadSize, "Depth is empty.");
    if(depth.size() != imageSize)
        CV_Error(Error::StsBadSize, "Depth has to have the size equal to the image size.");
    if(depth.type() != CV_64FC1)
        CV_Error(Error::StsBadSize, "Depth type has to be CV_64FC1.");
}

static inline
void checkMask(const Mat& mask, const Size& imageSize)
{
    if(!mask.empty())
    {
        if(mask.size() != imageSize)
            CV_Error(Error::StsBadSize, "Mask has to have the size equal to the image size.");
        if(mask.type() != CV_8UC1)
            CV_Error(Error::StsBadSize, "Mask type has to be CV_8UC1.");
    }
}

static inline
void checkNormals(const Mat& normals, const Size& depthSize)
{
    if(normals.size() != depthSize)
        CV_Error(Error::StsBadSize, "Normals has to have the size equal to the depth size.");
    if(normals.type() != CV_64FC3)
        CV_Error(Error::StsBadSize, "Normals type has to be CV_64FC3.");
}

static
int computeCorresps_hw(const Mat& K, 
                       const Mat& K_inv, 
                       const Mat& Rt,
                       const Mat& depth0,       //dstFrame
                       const Mat& validMask0,   //dstFrame_maskdepth
                       const Mat& depth1,       //srcFrame
                       const Mat& selectMask1,  //srcFrame_maskdepth
                       double maxDepthDiff,     //maxDepthDiff = 350
                       Mat& _corresps,
                       Mat& normal0,
                       Mat& debug) //pylin
{
    const double fx = K.at<double>(0,0);
    const double fy = K.at<double>(1,1);
    const double ox = K.at<double>(0,2);
    const double oy = K.at<double>(1,2);
    const double * Rt_ptr = Rt.ptr<const double>();
    Mat corresps(depth1.size(), CV_16SC2, Scalar::all(-1)); //depth1.size() = [640 x 480] (width x height)
    Rect r(0, 0, depth1.cols, depth1.rows); //Rect_ (左上角的x座標, 左上角的y座標, _Tp _width, _Tp _height)

    /*if (debug.at<double>(0,0) == 1){
        cout << "selectMask1.cols = " << selectMask1.cols << endl;
    }
    debug.at<double>(0,0) = 0;*/
    
    int correspCount = 0;

    // pylin↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
    int test_valid = 1;
    ifstream if_0("./debug_check_proj_d1_u_r_SW.txt");
    bool check_near_0to1 = if_0.good();
    ifstream if_1("./debug_check_near_1to2.txt");
    bool check_near_1to2 = if_1.good();
    if (check_near_0to1 == 0) {
        // ofstream of_0("./debug_check_proj_d1_u_r_SW.txt");
    int store_index = 32;
    int load_v_origin = 0;
    int load_index_tmp = 0;
    int load_index_w = 0;
    for(int v1 = 0; v1 < depth1.rows; v1++)
    {
        const double *depth1_row = depth1.ptr<double>(v1);
        const uchar *mask1_row = selectMask1.ptr<uchar>(v1);    //selectMask1.size() = [640 x 480]
        for(int u1 = 0; u1 < depth1.cols; u1++)
        {   
            double d1 = depth1_row[u1];
            if(test_valid)
            {
                //p1_x = (u1 - ox)*d1/fx
                double p1_x = trunc_3d(trunc_3d(trunc_3d(u1*MUL - trunc_3d(ox*MUL)) * d1 / trunc_3d(fx*MUL)) * MUL);
                //p1_y = (v1 - oy)*d1/fy
                double p1_y = trunc_3d(trunc_3d(trunc_3d(v1*MUL - trunc_3d(oy*MUL)) * d1 / trunc_3d(fy*MUL)) * MUL);
                //p1_z = d1
                double p1_z = trunc_3d(d1 * MUL);
                //[tp1] = [Rt] * [p]
                double tp1_x = trunc_lsm_f1(p1_x * Rt_ptr[0] / MUL) + trunc_lsm_f1(p1_y * Rt_ptr[1] / MUL) + trunc_lsm_f1(p1_z * Rt_ptr[2]  / MUL) + Rt_ptr[3] ;
                double tp1_y = trunc_lsm_f1(p1_x * Rt_ptr[4] / MUL) + trunc_lsm_f1(p1_y * Rt_ptr[5] / MUL) + trunc_lsm_f1(p1_z * Rt_ptr[6]  / MUL) + Rt_ptr[7] ;
                double tp1_z = trunc_lsm_f1(p1_x * Rt_ptr[8] / MUL) + trunc_lsm_f1(p1_y * Rt_ptr[9] / MUL) + trunc_lsm_f1(p1_z * Rt_ptr[10] / MUL) + Rt_ptr[11];

                // of_0 << long(p1_x) << endl;
                // of_0 << long(p1_y) << endl;
                // of_0 << long(p1_z) << endl;
                // of_0 << long(tp1_x) << endl;
                // of_0 << long(tp1_y) << endl;
                // of_0 << long(tp1_z) << endl;

                if(test_valid)
                {
                    //u0 = fx*tp1_x/tp1_z + ox
                    int u0 = trunc_lsm_f2( (trunc_lsm_f2(trunc_lsm_f2(fx * MUL) * tp1_x / tp1_z) + trunc_lsm_f2(ox * MUL)) / MUL);
                    //v0 = fy*tp1_y/tp1_z + oy
                    int v0 = trunc_lsm_f2( (trunc_lsm_f2(trunc_lsm_f2(fy * MUL) * tp1_y / tp1_z) + trunc_lsm_f2(oy * MUL)) / MUL);
                    
                    if (v0 == -1577) v0 = 1023;
                    load_v_origin = (store_index >= 32) ? (store_index - 32) : store_index + 31;
                    load_index_tmp = (v0 < 480) ? ((v0 >= v1) ? ((v0 - v1 <= 30) ? load_v_origin + v0 - v1 : 127) : ((v1 - v0 <= 30) ? load_v_origin + v0 - v1 : 127)) : 127;
                    load_index_w = (load_index_tmp == 127) ? 127 :
                                   ((load_index_tmp < 0)  ? load_index_tmp + 63 : 
                                   ((load_index_tmp > 62) ? load_index_tmp - 63 : load_index_tmp));
                    // of_0 << long(load_index_w) << endl;

                    // if(u0 == -845){
                    //     of_0 << 1023 << endl;
                    // }
                    // else {
                    //     of_0 << int(u0) << endl;
                    // }

                    // if(v0 == -1577){
                    //     of_0 << 1023 << endl;
                    // }
                    // else {
                    //     of_0 << int(v0) << endl;
                    // }

                    if(r.contains(Point(u0,v0))) //查看rect有沒有包含Point(u0, v0) -> 確認v0沒有超出範圍(480)
                    {
                        double d0 = depth0.at<double>(v0,u0);
                        bool test_check_near = std::abs(v1 - v0) <= maxLineDiff;
                        // of_0 << test_check_near << endl;

                        bool test_check_depth = std::abs(tp1_z - trunc1(d0*MUL)) <= trunc1(maxDepthDiff*MUL);
                        if (test_check_near) {
                            // of_0 << long(test_check_depth) << endl;
                            // of_0 << long(d0) << endl;    //proj_d1
                            // if (u0 < 639 && v0 < 479) of_0 << long(depth0.at<double>(v0,u0+1)) << endl; //proj_d1_u
                            // if (u0 < 639 && v0 < 479) of_0 << long(depth0.at<double>(v0+1,u0)) << endl; //proj_d1_v
                            // else of_0 << 0 << endl; //proj_d1_u or proj_d1_v 
                        }
                        // else of_0 << 65535 << endl;  //proj_d1
                        // else of_0 << 0 << endl; //proj_d1_u or proj_d1_v
                        // if (u0 == 639 || v0 == 479) of_0 << 0 << endl;
                        // else if (long(validMask0.at<uchar>(v0, u0)) == 255) of_0 << 1 << endl;
                        // else of_0 << 0 << endl;
                        if(validMask0.at<uchar>(v0, u0) && std::abs(tp1_z - trunc1(d0*MUL)) <= trunc1(maxDepthDiff*MUL) && std::abs(v1 - v0) <= maxLineDiff){
                            // of_0 << long(u1) << endl; //match_u0
                            // of_0 << long(v1) << endl; //match_v0
                            // of_0 << long(d1) << endl; //match_d0
                            // of_0 << long(u0) << endl; //match_u1
                            // of_0 << long(v0) << endl; //match_v1
                            // of_0 << long(d0) << endl; //match_d1
                            
                            // of_0 << long(normal0.at<Vec3d>(v0,u0)[0]) << endl; //match_n1_x
                            // of_0 << long(normal0.at<Vec3d>(v0,u0)[1]) << endl; //match_n1_y
                            // of_0 << long(normal0.at<Vec3d>(v0,u0)[2]) << endl; //match_n1_z

                        }
                        
                        
                        
                        
                    }
                    // else of_0 << 65535 << endl;  //proj_d1
                    // else of_0 << 0 << endl; //proj_d1_u or proj_d1_v
                    
                }
            }
            if (u1 == 639) {
                if (store_index == 62) store_index = 0;
                else store_index++;
            }
        }
    }
    }
    else if (check_near_1to2 == 0) {
        // ofstream of_1("./debug_check_near_1to2.txt");
    for(int v1 = 0; v1 < depth1.rows; v1++)
    {
        const double *depth1_row = depth1.ptr<double>(v1);
        const uchar *mask1_row = selectMask1.ptr<uchar>(v1);    //selectMask1.size() = [640 x 480]
        for(int u1 = 0; u1 < depth1.cols; u1++)
        {
            double d1 = depth1_row[u1];
            if(test_valid)
            {
                //p1_x = (u1 - ox)*d1/fx
                double p1_x = trunc_3d(trunc_3d(trunc_3d(u1*MUL - trunc_3d(ox*MUL)) * d1 / trunc_3d(fx*MUL)) * MUL);
                //p1_y = (v1 - oy)*d1/fy
                double p1_y = trunc_3d(trunc_3d(trunc_3d(v1*MUL - trunc_3d(oy*MUL)) * d1 / trunc_3d(fy*MUL)) * MUL);
                //p1_z = d1
                double p1_z = trunc_3d(d1 * MUL);
                //[tp1] = [Rt] * [p]
                double tp1_x = trunc_lsm_f1(p1_x * Rt_ptr[0] / MUL) + trunc_lsm_f1(p1_y * Rt_ptr[1] / MUL) + trunc_lsm_f1(p1_z * Rt_ptr[2]  / MUL) + Rt_ptr[3] ;
                double tp1_y = trunc_lsm_f1(p1_x * Rt_ptr[4] / MUL) + trunc_lsm_f1(p1_y * Rt_ptr[5] / MUL) + trunc_lsm_f1(p1_z * Rt_ptr[6]  / MUL) + Rt_ptr[7] ;
                double tp1_z = trunc_lsm_f1(p1_x * Rt_ptr[8] / MUL) + trunc_lsm_f1(p1_y * Rt_ptr[9] / MUL) + trunc_lsm_f1(p1_z * Rt_ptr[10] / MUL) + Rt_ptr[11];

                if(test_valid)
                {
                    //u0 = fx*tp1_x/tp1_z + ox
                    int u0 = trunc_lsm_f2( (trunc_lsm_f2(trunc_lsm_f2(fx * MUL) * tp1_x / tp1_z) + trunc_lsm_f2(ox * MUL)) / MUL);
                    //v0 = fy*tp1_y/tp1_z + oy
                    int v0 = trunc_lsm_f2( (trunc_lsm_f2(trunc_lsm_f2(fy * MUL) * tp1_y / tp1_z) + trunc_lsm_f2(oy * MUL)) / MUL);

                    bool test_check_near = std::abs(v1 - v0) <= maxLineDiff;
                    // of_1 << test_check_near << endl;
                }
            }
        }
    }
    }
    // pylin↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑

    for(int v1 = 0; v1 < depth1.rows; v1++)
    {
        const double *depth1_row = depth1.ptr<double>(v1);
        const uchar *mask1_row = selectMask1.ptr<uchar>(v1);    //selectMask1.size() = [640 x 480]
        for(int u1 = 0; u1 < depth1.cols; u1++)
        {
            double d1 = depth1_row[u1];
            if(mask1_row[u1])
            {
                //p1_x = (u1 - ox)*d1/fx
                double p1_x = trunc_3d(trunc_3d(trunc_3d(u1*MUL - trunc_3d(ox*MUL)) * d1 / trunc_3d(fx*MUL)) * MUL);
                //p1_y = (v1 - oy)*d1/fy
                double p1_y = trunc_3d(trunc_3d(trunc_3d(v1*MUL - trunc_3d(oy*MUL)) * d1 / trunc_3d(fy*MUL)) * MUL);
                //p1_z = d1
                double p1_z = trunc_3d(d1 * MUL);
                //[tp1] = [Rt] * [p]
                double tp1_x = trunc_lsm_f1(p1_x * Rt_ptr[0] / MUL) + trunc_lsm_f1(p1_y * Rt_ptr[1] / MUL) + trunc_lsm_f1(p1_z * Rt_ptr[2]  / MUL) + Rt_ptr[3] ;
                double tp1_y = trunc_lsm_f1(p1_x * Rt_ptr[4] / MUL) + trunc_lsm_f1(p1_y * Rt_ptr[5] / MUL) + trunc_lsm_f1(p1_z * Rt_ptr[6]  / MUL) + Rt_ptr[7] ;
                double tp1_z = trunc_lsm_f1(p1_x * Rt_ptr[8] / MUL) + trunc_lsm_f1(p1_y * Rt_ptr[9] / MUL) + trunc_lsm_f1(p1_z * Rt_ptr[10] / MUL) + Rt_ptr[11];

                // //↓↓↓↓↓--pylin--↓↓↓↓↓
                // if(debug.at<double>(0,0) == 1){
                //     cout << "mask1_row[" << u1 << "] = " << mask1_row[u1] << endl;
                //     cout << "p1_x = " << p1_x << " = " << "[u1(" << u1 << ")-ox(" << ox << ")]*d1(" << d1 <<")/fx(" << fx << ")" << endl;
                //     cout << "p1_y = " << p1_y << " = " << "[v1(" << u1 << ")-oy(" << ox << ")]*d1(" << d1 <<")/fy(" << fx << ")" << endl;
                //     cout << "p1_z = d1 = " << d1 << endl;
                //     cout << "[tp1] = [Rt] * [p]" << endl;
                //     cout << "tp1_x = " << tp1_x << " = Rt[0,0](" << Rt_ptr[0] << ")*p1_x(" << p1_x << ") + Rt[0,1](" << Rt_ptr[1] << ")*p1_y(" << p1_y << ") + Rt[0,2](" << Rt_ptr[2] << ")*p1_z(" << p1_z << ") + Rt[0,3](" << Rt_ptr[3] << ")" << endl;
                //     cout << "tp1_y = " << tp1_y << " = Rt[1,0](" << Rt_ptr[4] << ")*p1_x(" << p1_x << ") + Rt[1,1](" << Rt_ptr[5] << ")*p1_y(" << p1_y << ") + Rt[1,2](" << Rt_ptr[6] << ")*p1_z(" << p1_z << ") + Rt[1,3](" << Rt_ptr[7] << ")" << endl;
                //     cout << "tp1_z = " << tp1_z << " = Rt[2,0](" << Rt_ptr[8] << ")*p1_x(" << p1_x << ") + Rt[2,1](" << Rt_ptr[9] << ")*p1_y(" << p1_y << ") + Rt[2,2](" << Rt_ptr[10] << ")*p1_z(" << p1_z << ") + Rt[2,3](" << Rt_ptr[11] << ")" << endl;
                // }
                // //↑↑↑↑↑--pylin--↑↑↑↑↑

                if(tp1_z > 0)
                {
                    //u0 = fx*tp1_x/tp1_z + ox
                    int u0 = trunc_lsm_f2( (trunc_lsm_f2(trunc_lsm_f2(fx * MUL) * tp1_x / tp1_z) + trunc_lsm_f2(ox * MUL)) / MUL);
                    //v0 = fy*tp1_y/tp1_z + oy
                    int v0 = trunc_lsm_f2( (trunc_lsm_f2(trunc_lsm_f2(fy * MUL) * tp1_y / tp1_z) + trunc_lsm_f2(oy * MUL)) / MUL);

                    // //↓↓↓↓↓--pylin--↓↓↓↓↓
                    // if(debug.at<double>(0,0) == 1){
                    //     cout << "u0 = " << u0 << " = fx(" << fx << ")*tp1_x(" << tp1_x << ")/tp1_z(" << tp1_z << ") + ox(" << ox << ")" << endl;
                    //     cout << "v0 = " << v0 << " = fy(" << fy << ")*tp1_y(" << tp1_y << ")/tp1_z(" << tp1_z << ") + oy(" << oy << ")" << endl;
                    // }
                    // //↑↑↑↑↑--pylin--↑↑↑↑↑

                    // cout << "test begin" << endl;
                    // cout << u1 << " " << v1 << " "<< u0 << " "<< v0 << endl;
                    // cout << "test end" << endl;
                    if(r.contains(Point(u0,v0))) //查看rect有沒有包含Point(u0, v0) -> 確認v0沒有超出範圍(480)
                    {
                        double d0 = depth0.at<double>(v0,u0);

                        //if (mask==1 && |z1-z0|<350 && |v1-v0|<30 )
                        if(validMask0.at<uchar>(v0, u0) && std::abs(tp1_z - trunc1(d0*MUL)) <= trunc1(maxDepthDiff*MUL) && std::abs(v1 - v0) <= maxLineDiff)    //maxLineDiff = 30
                        {
                            /*
                            Vec2s& c = corresps.at<Vec2s>(v0,u0);
                            if(c[0] != -1)
                            {
                                int exist_u1 = c[0], exist_v1 = c[1];

                                double exist_p1_x = trunc_3d(trunc_3d(trunc_3d(exist_u1*MUL - trunc_3d(ox*MUL)) * d1 / trunc_3d(fx*MUL)) * MUL);
                                double exist_p1_y = trunc_3d(trunc_3d(trunc_3d(exist_v1*MUL - trunc_3d(oy*MUL)) * d1 / trunc_3d(fy*MUL)) * MUL);
                                double exist_p1_z = trunc_3d(depth1.at<double>(exist_v1,exist_u1) * MUL);
                                double exist_tp1_z = trunc_lsm_f1(exist_p1_x * Rt_ptr[8] / MUL) + trunc_lsm_f1(exist_p1_y * Rt_ptr[9] / MUL) + trunc_lsm_f1(exist_p1_z * Rt_ptr[10] / MUL) + Rt_ptr[11];
                                if(tp1_z > exist_tp1_z)
                                    continue;
                            }
                            else
                                correspCount++;

                            c = Vec2s((short)u1, (short)v1);
                            */
                            Vec2s& c = corresps.at<Vec2s>(v1,u1);
                            c = Vec2s((short)u0, (short)v0); //short stand for 16-bit signed integer
                            correspCount++;

                            // //↓↓↓↓↓--pylin--↓↓↓↓↓
                            // if(debug.at<double>(0,0) == 1){
                            //     cout << "dstFrameMask(" << v0 << "," << u0 << ") & |tp1_z(" << tp1_z << ")-dstFrameDepth(" << v0 << "," << u0 << ")(" << trunc1(d0*MUL) << ")|<=maxDepthDiff(350) & |v1(" << v1 << ")-v0(" << v0 << ")|<=maxLineDiff(30)" << endl;
                            //     cout << "correspCount = " << correspCount << " : " << c << endl;
                            // }
                            // //↑↑↑↑↑--pylin--↑↑↑↑↑

                        }

                        // //↓↓↓↓↓--pylin--↓↓↓↓↓
                        // else{
                        //     if(debug.at<double>(0,0) == 1){
                        //         cout << "dstFrameMask(" << v0 << "," << u0 << ") = " << validMask0.at<uchar>(v0, u0) << endl;
                        //         cout << "|tp1_z(" << tp1_z << ")-dstFrameDepth(" << v0 << "," << u0 << ")(" << trunc1(d0*MUL) << ")| > maxDepthDiff(350)" << endl;
                        //         cout << "|v1(" << v1 << ")-v0(" << v0 << ")| > maxLineDiff(30)" << endl;
                        //     }
                        // }
                        // //↑↑↑↑↑--pylin--↑↑↑↑↑
                    }
                }
            }
        }
    }
    // cout << "correspCount = " << correspCount << endl;
    /*
    int v_max_corr = 0;
    _corresps.create(correspCount, 1, CV_32SC4);
    Vec4i * corresps_ptr = _corresps.ptr<Vec4i>();
    for(int v0 = 0, i = 0; v0 < corresps.rows; v0++)
    {
        const Vec2s* corresps_row = corresps.ptr<Vec2s>(v0);
        for(int u0 = 0; u0 < corresps.cols; u0++)
        {
            const Vec2s& c = corresps_row[u0];
            if(c[0] != -1)
            {
                //corresps_ptr[i++] = Vec4i(u0,v0,c[0],c[1]);
                corresps_ptr[i++] = Vec4i(c[0],c[1],u0,v0);
                int v_diff = abs(v0-c[1]);
                if(v_diff > v_max_corr)
                    v_max_corr = v_diff;
            }
        }
    }
    */
    int v_max_corr = 0;
    _corresps.create(correspCount, 1, CV_32SC4);    //.create(rows, cols, type)
    Vec4i * corresps_ptr = _corresps.ptr<Vec4i>();

    //↓↓↓↓↓--pylin--↓↓↓↓↓
    ifstream ifile_1("./debug_corresps_1to2.txt");
    bool check_1to2 = ifile_1.good();
    ifstream ifile_2("./debug_corresps_2to3.txt");
    bool check_2to3 = ifile_2.good();
    if (check_1to2 == 0) {
        // ofstream ofile_1("./debug_corresps_1to2.txt");
    //↑↑↑↑↑--pylin--↑↑↑↑↑

    for(int v0 = 0, i = 0; v0 < corresps.rows; v0++)
    {
        const Vec2s* corresps_row = corresps.ptr<Vec2s>(v0);
        for(int u0 = 0; u0 < corresps.cols; u0++)
        {
            const Vec2s& c = corresps_row[u0];
            if(c[0] != -1)
            {
                corresps_ptr[i++] = Vec4i(u0,v0,c[0],c[1]); //0:src ; c:dst

                // //↓↓↓↓↓--pylin--↓↓↓↓↓
                // ofile_1 << int(u0) << endl;
                // //↑↑↑↑↑--pylin--↑↑↑↑↑

                //corresps_ptr[i++] = Vec4i(c[0],c[1],u0,v0);
                int v_diff = abs(v0-c[1]);
                if(v_diff > v_max_corr)
                    v_max_corr = v_diff;
            }
        }
    }
    }
    else if (check_2to3 == 0) {
        // ofstream ofile_2("./debug_corresps_2to3.txt");
    for(int v0 = 0, i = 0; v0 < corresps.rows; v0++)
    {
        const Vec2s* corresps_row = corresps.ptr<Vec2s>(v0);
        for(int u0 = 0; u0 < corresps.cols; u0++)
        {
            const Vec2s& c = corresps_row[u0];
            if(c[0] != -1)
            {
                corresps_ptr[i++] = Vec4i(u0,v0,c[0],c[1]); //0:src ; c:dst

                // //↓↓↓↓↓--pylin--↓↓↓↓↓
                // ofile_2 << int(u0) << endl;
                // //↑↑↑↑↑--pylin--↑↑↑↑↑

                //corresps_ptr[i++] = Vec4i(c[0],c[1],u0,v0);
                int v_diff = abs(v0-c[1]);
                if(v_diff > v_max_corr)
                    v_max_corr = v_diff;
            }
        }
    }
    }
    else {
    for(int v0 = 0, i = 0; v0 < corresps.rows; v0++)
    {
        const Vec2s* corresps_row = corresps.ptr<Vec2s>(v0);
        for(int u0 = 0; u0 < corresps.cols; u0++)
        {
            const Vec2s& c = corresps_row[u0];
            if(c[0] != -1)
            {
                corresps_ptr[i++] = Vec4i(u0,v0,c[0],c[1]); //0:src ; c:dst
                //corresps_ptr[i++] = Vec4i(c[0],c[1],u0,v0);
                int v_diff = abs(v0-c[1]);
                if(v_diff > v_max_corr)
                    v_max_corr = v_diff;
            }
        }
    }
    }
    return v_max_corr;
}

static
int computeCorresps(const Mat& K, const Mat& K_inv, const Mat& Rt,
                     const Mat& depth0, const Mat& validMask0,
                     const Mat& depth1, const Mat& selectMask1, double maxDepthDiff,
                     Mat& _corresps)
{
    CV_Assert(K.type() == CV_64FC1);
    CV_Assert(K_inv.type() == CV_64FC1);
    CV_Assert(Rt.type() == CV_64FC1);

    Mat corresps(depth1.size(), CV_16SC2, Scalar::all(-1));

    Rect r(0, 0, depth1.cols, depth1.rows);
    Mat Kt = Rt(Rect(3,0,1,3)).clone();
    //Kt = K * Kt;
    double * Kt_ptr = Kt.ptr<double>();
    double fx = trunc1(K.at<double>(0, 0) * MUL);
    double fy = trunc1(K.at<double>(1, 1) * MUL);
    double cx = trunc1(K.at<double>(0, 2) * MUL);
    double cy = trunc1(K.at<double>(1, 2) * MUL);
    Kt_ptr[0] = trunc1(Kt_ptr[0] * fx / MUL) + trunc1(Kt_ptr[2] * cx / MUL);
    Kt_ptr[1] = trunc1(Kt_ptr[1] * fy / MUL) + trunc1(Kt_ptr[2] * cy / MUL);
    Kt_ptr[2] = Kt_ptr[2];
    double fx_inv = trunc1(MUL * MUL / fx);
    double fy_inv = trunc1(MUL * MUL / fy);
    //double cx_inv = -cx / fx;
    //double cy_inv = -cy / fy;

    AutoBuffer<double> buf(3 * (depth1.cols + depth1.rows));
    double *KRK_inv0_u1 = buf;
    double *KRK_inv1_v1_plus_KRK_inv2 = KRK_inv0_u1 + depth1.cols;
    double *KRK_inv3_u1 = KRK_inv1_v1_plus_KRK_inv2 + depth1.rows;
    double *KRK_inv4_v1_plus_KRK_inv5 = KRK_inv3_u1 + depth1.cols;
    double *KRK_inv6_u1 = KRK_inv4_v1_plus_KRK_inv5 + depth1.rows;
    double *KRK_inv7_v1_plus_KRK_inv8 = KRK_inv6_u1 + depth1.cols;
    {
        Mat R = Rt(Rect(0,0,3,3)).clone();

        //Mat KRK_inv = K * R * K_inv;
        //Mat K_inv2 = Mat::eye(3, 3, CV_64FC1);
        //K_inv2.at<double>(0,0) = fx_inv; 
        //K_inv2.at<double>(1,1) = fy_inv; 
        //K_inv2.at<double>(0,2) = cx_inv; 
        //K_inv2.at<double>(1,2) = cy_inv; 
        //Mat KRK_inv2 = K * R * K_inv2;
        Mat KRK_inv = Mat::eye(3, 3, CV_64FC1);
        double r00 = R.at<double>(0,0);
        double r01 = R.at<double>(0,1);
        double r02 = R.at<double>(0,2);
        double r10 = R.at<double>(1,0);
        double r11 = R.at<double>(1,1);
        double r12 = R.at<double>(1,2);
        double r20 = R.at<double>(2,0);
        double r21 = R.at<double>(2,1);
        double r22 = R.at<double>(2,2);
        KRK_inv.at<double>(0,0) = r00 + trunc1((r20 * cx * fx_inv / MUL) / MUL); 
        KRK_inv.at<double>(0,1) = trunc1((r01 * fx * fy_inv / MUL) / MUL) + trunc1((r21 * cx * fy_inv / MUL) / MUL); 
        KRK_inv.at<double>(0,2) = -trunc1(r00 * cx / MUL) - trunc1(((r01 * fx * cy * fy_inv / MUL) / MUL) / MUL) + trunc1(r02 * fx / MUL) - trunc1(((r20 * cx * cx * fx_inv / MUL) / MUL) / MUL) - trunc1(((r21 * cx * cy * fy_inv / MUL) / MUL) / MUL) + trunc1(r22 * cx / MUL); 
        KRK_inv.at<double>(1,0) = trunc1((r10 * fy * fx_inv / MUL) / MUL) + trunc1((r20 * cy * fx_inv / MUL) / MUL); 
        KRK_inv.at<double>(1,1) = r11 + trunc1((r21 * cy * fy_inv / MUL) / MUL); 
        KRK_inv.at<double>(1,2) = -trunc1(((r10 * cx * fy * fx_inv / MUL) / MUL) / MUL) - trunc1(r11 * cy / MUL) + trunc1(r12 * fy / MUL) - trunc1(((r20 * cx * cy * fx_inv / MUL) / MUL) / MUL) - trunc1(((r21 * cy * cy * fy_inv / MUL) / MUL) / MUL) + trunc1(r22 * cy / MUL) ; 
        KRK_inv.at<double>(2,0) = trunc1(r20 * fx_inv / MUL); 
        KRK_inv.at<double>(2,1) = trunc1(r21 * fy_inv / MUL); 
        KRK_inv.at<double>(2,2) = - trunc1((r20 * cx * fx_inv / MUL) / MUL) - trunc1((r21 * cy * fy_inv / MUL) / MUL) + r22; 

        const double * KRK_inv_ptr = KRK_inv.ptr<const double>();
        for(int u1 = 0; u1 < depth1.cols; u1++)
        {
            KRK_inv0_u1[u1] = (double)(KRK_inv_ptr[0] * u1);
            KRK_inv3_u1[u1] = (double)(KRK_inv_ptr[3] * u1);
            KRK_inv6_u1[u1] = (double)(KRK_inv_ptr[6] * u1);
        }

        for(int v1 = 0; v1 < depth1.rows; v1++)
        {
            KRK_inv1_v1_plus_KRK_inv2[v1] = (double)(KRK_inv_ptr[1] * v1 + KRK_inv_ptr[2]);
            KRK_inv4_v1_plus_KRK_inv5[v1] = (double)(KRK_inv_ptr[4] * v1 + KRK_inv_ptr[5]);
            KRK_inv7_v1_plus_KRK_inv8[v1] = (double)(KRK_inv_ptr[7] * v1 + KRK_inv_ptr[8]);
        }
    }

    int correspCount = 0;
    for(int v1 = 0; v1 < depth1.rows; v1++)
    {
        const double *depth1_row = depth1.ptr<double>(v1);
        const uchar *mask1_row = selectMask1.ptr<uchar>(v1);
        for(int u1 = 0; u1 < depth1.cols; u1++)
        {
            double d1 = depth1_row[u1];
            if(mask1_row[u1])
            {
                CV_DbgAssert(!cvIsNaN(d1));
                double transformed_d1 = static_cast<double>(d1 * (KRK_inv6_u1[u1] + KRK_inv7_v1_plus_KRK_inv8[v1]) +
                                                          Kt_ptr[2]);
                if(transformed_d1 > 0)
                {
                    //double transformed_d1_inv = trunc(MUL * MUL / transformed_d1);
                    int u0 = cvRound((d1 * (KRK_inv0_u1[u1] + KRK_inv1_v1_plus_KRK_inv2[v1]) + Kt_ptr[0]) / transformed_d1);
                    int v0 = cvRound((d1 * (KRK_inv3_u1[u1] + KRK_inv4_v1_plus_KRK_inv5[v1]) + Kt_ptr[1]) / transformed_d1);

                    if(r.contains(Point(u0,v0)))
                    {
                        double d0 = depth0.at<double>(v0,u0);
                        if(validMask0.at<uchar>(v0, u0) && std::abs(transformed_d1 - trunc1(d0*MUL)) <= trunc1(maxDepthDiff*MUL) && std::abs(v1 - v0) <= maxLineDiff)
                        {
                            CV_DbgAssert(!cvIsNaN(d0));
                            Vec2s& c = corresps.at<Vec2s>(v0,u0);
                            if(c[0] != -1)
                            {
                                int exist_u1 = c[0], exist_v1 = c[1];

                                double exist_d1 = (double)(depth1.at<double>(exist_v1,exist_u1) *
                                    (KRK_inv6_u1[exist_u1] + KRK_inv7_v1_plus_KRK_inv8[exist_v1]) + Kt_ptr[2]);

                                if(transformed_d1 > exist_d1)
                                    continue;
                            }
                            else
                                correspCount++;

                            c = Vec2s((short)u1, (short)v1);
                        }
                    }
                }
            }
        }
    }

    int v_max_corr = 0;
    _corresps.create(correspCount, 1, CV_32SC4);
    Vec4i * corresps_ptr = _corresps.ptr<Vec4i>();
    for(int v0 = 0, i = 0; v0 < corresps.rows; v0++)
    {
        const Vec2s* corresps_row = corresps.ptr<Vec2s>(v0);
        for(int u0 = 0; u0 < corresps.cols; u0++)
        {
            const Vec2s& c = corresps_row[u0];
            if(c[0] != -1)
            {
                //corresps_ptr[i++] = Vec4i(u0,v0,c[0],c[1]);
                corresps_ptr[i++] = Vec4i(c[0],c[1],u0,v0);
                int v_diff = abs(v0-c[1]);
                if(v_diff > v_max_corr)
                    v_max_corr = v_diff;
            }
        }
    }
    // cout << "begin" << endl;
    // cout << _corresps << endl;
    // cout << "end" << endl;
    //exit(0);
    return v_max_corr;
}

typedef
void (*CalcRgbdEquationCoeffsPtr)(double*, double, double, const Point3d&, double, double);

typedef
void (*CalcICPEquationCoeffsPtr)(double*, const Point3d&, const Vec3d&);

typedef
void (*CalcFeatureXEquationCoeffsPtr)(double*, const Point3d&, double);

typedef
void (*CalcFeatureYEquationCoeffsPtr)(double*, const Point3d&, double);


static
void calcRgbdLsmMatrices(const Mat& image0, //src
                         const Mat& cloud0, //src
                         const Mat& Rt,
                         const Mat& image1, //dst
                         const Mat& dI_dx1, //dst
                         const Mat& dI_dy1, //dst
                         const Mat& corresps, 
                         double fx, double fy, 
                         double sobelScaleIn,   //sobelScaleIn = 1./8.
                         Mat& AtA, Mat& AtB, 
                         CalcRgbdEquationCoeffsPtr func, 
                         int transformDim)  //transformDim = 6
{
    AtA = Mat(transformDim, transformDim, CV_64FC1, Scalar(0)); // size of AtA = 6*6
    AtB = Mat(transformDim, 1, CV_64FC1, Scalar(0));    // size of AtB = 6*1
    double* AtB_ptr = AtB.ptr<double>();

    const int correspsCount = corresps.rows;

    CV_Assert(Rt.type() == CV_64FC1);
    const double * Rt_ptr = Rt.ptr<const double>();

    AutoBuffer<double> diffs(correspsCount);
    double* diffs_ptr = diffs;

    const Vec4i* corresps_ptr = corresps.ptr<Vec4i>();

    double sigma = 0;
    for(int correspIndex = 0; correspIndex < corresps.rows; correspIndex++)
    {
         const Vec4i& c = corresps_ptr[correspIndex];
         int u0 = c[0], v0 = c[1];  //src
         int u1 = c[2], v1 = c[3];  //dst

         diffs_ptr[correspIndex] = static_cast<double>(static_cast<int>(image0.at<uchar>(v0,u0)) -
                                                      static_cast<int>(image1.at<uchar>(v1,u1)));   // diffs = RGB_src - RGB_dst
    //      std::cout << "====================test=======================" << diffs_ptr[0] <<  std::endl;
    //      std::cout << static_cast<int>(image0.at<uchar>(v0,u0)) <<  std::endl;
    //      std::cout << static_cast<int>(image1.at<uchar>(v1,u1)) <<  std::endl;
	//  exit(1);
         sigma += diffs_ptr[correspIndex] * diffs_ptr[correspIndex]; // sigma = sum(diffs^2)
    }
    sigma = trunc1(std::sqrt(trunc1(sigma/correspsCount))); //sigma: diff的方均根
    //cout << "sigma: " << sigma << endl;

    std::vector<double> A_buf(transformDim);
    double* A_ptr = &A_buf[0];

    for(int correspIndex = 0; correspIndex < corresps.rows; correspIndex++)
    {
         const Vec4i& c = corresps_ptr[correspIndex];
         int u0 = c[0], v0 = c[1];  //src
         int u1 = c[2], v1 = c[3];  //dst

         double w = sigma + std::abs(diffs_ptr[correspIndex]);  // w = sigma + |diff|
         w = w > DBL_EPSILON ? 1./w : 1.;   //DBL_EPSILON表double的min正值, w = 1/(sigma + |diff|)

         double w_sobelScale = w * sobelScaleIn;    //sobelScaleIn = 1./8.; w_sobelScale = 1/[8*(sigma + |diff|)]

         const Point3d& p0 = cloud0.at<Point3d>(v0,u0);
         Point3d tp0; // from src
         tp0.x = trunc1(p0.x * Rt_ptr[0] / MUL) + trunc1(p0.y * Rt_ptr[1] / MUL) + trunc1(p0.z * Rt_ptr[2]  / MUL) + Rt_ptr[3] ;
         tp0.y = trunc1(p0.x * Rt_ptr[4] / MUL) + trunc1(p0.y * Rt_ptr[5] / MUL) + trunc1(p0.z * Rt_ptr[6]  / MUL) + Rt_ptr[7] ;
         tp0.z = trunc1(p0.x * Rt_ptr[8] / MUL) + trunc1(p0.y * Rt_ptr[9] / MUL) + trunc1(p0.z * Rt_ptr[10] / MUL) + Rt_ptr[11];

         //func(A_ptr,
         //     w_sobelScale * dI_dx1.at<short int>(v1,u1),
         //     w_sobelScale * dI_dy1.at<short int>(v1,u1),
         //     tp0, fx, fy);
         double invz  = 1. / tp0.z;
         double invzw  = invz * w_sobelScale;   //invzw = 1/{tp0.z*[8*(sigma + |diff|)]}
         double tmp_v0 = trunc1(dI_dx1.at<short int>(v1,u1) * trunc1(fx * MUL) * MUL * invzw);
         double tmp_v1 = trunc1(dI_dy1.at<short int>(v1,u1) * trunc1(fy * MUL) * MUL * invzw);
         double tmp_v2 = trunc1(-(tmp_v0 * tp0.x + tmp_v1 * tp0.y) * invz);

         A_ptr[0] = trunc1((-tp0.z * tmp_v1 + tp0.y * tmp_v2) / MUL);
         A_ptr[1] = trunc1(( tp0.z * tmp_v0 - tp0.x * tmp_v2) / MUL);
         A_ptr[2] = trunc1((-tp0.y * tmp_v0 + tp0.x * tmp_v1) / MUL);
         A_ptr[3] = tmp_v0;
         A_ptr[4] = tmp_v1;
         A_ptr[5] = tmp_v2;

        for(int y = 0; y < transformDim; y++)
        {
            double* AtA_ptr = AtA.ptr<double>(y);
            for(int x = y; x < transformDim; x++)
                AtA_ptr[x] += trunc2(A_ptr[y] * A_ptr[x] / MUL);

            AtB_ptr[y] += trunc2(A_ptr[y] * diffs_ptr[correspIndex] * w);
        }
    }

    for(int y = 0; y < transformDim; y++)
        for(int x = y+1; x < transformDim; x++)
            AtA.at<double>(x,y) = AtA.at<double>(y,x);
}

static
void calcICPLsmMatrices(const Mat& cloud0,   //src
                        const Mat& Rt,
                        const Mat& cloud1,   //dst
                        const Mat& normals1, //dst_normal
                        const Mat& corresps,
                        Mat& AtA, Mat& AtB, 
                        CalcICPEquationCoeffsPtr func, 
                        int transformDim,
                        int iter,
                        Mat sigma_debug)
{
    AtA = Mat(transformDim, transformDim, CV_64FC1, Scalar(0)); // size of AtA = 6*6
    AtB = Mat(transformDim, 1, CV_64FC1, Scalar(0));    //size of AtB = 6*1
    double* AtB_ptr = AtB.ptr<double>();

    const int correspsCount = corresps.rows;

    CV_Assert(Rt.type() == CV_64FC1);
    const double * Rt_ptr = Rt.ptr<const double>();

    AutoBuffer<long> diffs(correspsCount);
    long * diffs_ptr = diffs;

    AutoBuffer<Point3d> transformedPoints0(correspsCount);
    Point3d * tps0_ptr = transformedPoints0;

    const Vec4i* corresps_ptr = corresps.ptr<Vec4i>();

    long sigma = 0;
    ifstream if_0("./debug_check_diff_SW.txt");
    bool check_0 = if_0.good();
    if (check_0 == 0) {
        // ofstream of_0("./debug_check_diff_SW.txt");
    for(int correspIndex = 0; correspIndex < corresps.rows; correspIndex++)
    {
        const Vec4i& c = corresps_ptr[correspIndex];
        int u0 = c[0], v0 = c[1];   //src
        int u1 = c[2], v1 = c[3];   //dst

        const Point3d& p0 = cloud0.at<Point3d>(v0,u0);

        // of_0 << long(p0.x) << endl; //p0_x
        // of_0 << long(p0.y) << endl; //p0_y
        // of_0 << long(p0.z) << endl; //p0_z
        const Point3d& p1 = cloud1.at<Point3d>(v1,u1);
        // of_0 << long(p1.x) << endl; //p1_x
        // of_0 << long(p1.y) << endl; //p1_y
        // of_0 << long(p1.z) << endl; //p1_z

        Point3d tp0;
        tp0.x = trunc4(p0.x * Rt_ptr[0] / MUL) + trunc4(p0.y * Rt_ptr[1] / MUL) + trunc4(p0.z * Rt_ptr[2]  / MUL) + Rt_ptr[3] ;
        tp0.y = trunc4(p0.x * Rt_ptr[4] / MUL) + trunc4(p0.y * Rt_ptr[5] / MUL) + trunc4(p0.z * Rt_ptr[6]  / MUL) + Rt_ptr[7] ;
        tp0.z = trunc4(p0.x * Rt_ptr[8] / MUL) + trunc4(p0.y * Rt_ptr[9] / MUL) + trunc4(p0.z * Rt_ptr[10] / MUL) + Rt_ptr[11];

        // of_0 << long(tp0.x) << endl; //tp0_x
        // of_0 << long(tp0.y) << endl; //tp0_y
        // of_0 << long(tp0.z) << endl; //tp0_z

        Vec3d n1 = normals1.at<Vec3d>(v1, u1);
        Point3d v = cloud1.at<Point3d>(v1,u1) - tp0; //MUL

        // of_0 << long(n1[0]) << endl; //n1_x
        // of_0 << long(n1[1]) << endl; //n1_y
        // of_0 << long(n1[2]) << endl; //n1_z
        // of_0 << long(v.x) << endl; //v_x
        // of_0 << long(v.y) << endl; //v_y
        // of_0 << long(v.z) << endl; //v_z

        tps0_ptr[correspIndex] = tp0; //MUL
        mp::cpp_int n1_x_mp(n1[0]);
        mp::cpp_int n1_y_mp(n1[1]);
        mp::cpp_int n1_z_mp(n1[2]);
        mp::cpp_int v_x_mp(v.x);
        mp::cpp_int v_y_mp(v.y);
        mp::cpp_int v_z_mp(v.z);
        mp::cpp_int diff_mp;
        diff_mp = (n1_x_mp * v_x_mp) + (n1_y_mp * v_y_mp) + (n1_z_mp * v_z_mp);
        diffs_ptr[correspIndex] = diff_mp.convert_to<long>();
        // diffs_ptr[correspIndex] = n1[0] * v.x + n1[1] * v.y + n1[2] * v.z; //MUL^2
        //std::cout << "====================test=======================" << diffs_ptr[0] <<  std::endl;
        //exit(1);
        sigma += diffs_ptr[correspIndex] * diffs_ptr[correspIndex]; //MUL^4

        // of_0 << v_x_mul_n1_x << endl;  //v_x_mul_n1_x
        // of_0 << v_y_mul_n1_y << endl;  //v_y_mul_n1_y
        // of_0 << v_z_mul_n1_z << endl;  //v_z_mul_n1_z
        // of_0 << diffs_ptr[correspIndex] << endl;   //diff
    }
    }

    else {
    for(int correspIndex = 0; correspIndex < corresps.rows; correspIndex++)
    {
        const Vec4i& c = corresps_ptr[correspIndex];
        int u0 = c[0], v0 = c[1];   //src
        int u1 = c[2], v1 = c[3];   //dst

        const Point3d& p0 = cloud0.at<Point3d>(v0,u0);
        Point3d tp0;
        tp0.x = trunc4(p0.x * Rt_ptr[0] / MUL) + trunc4(p0.y * Rt_ptr[1] / MUL) + trunc4(p0.z * Rt_ptr[2]  / MUL) + Rt_ptr[3] ;
        tp0.y = trunc4(p0.x * Rt_ptr[4] / MUL) + trunc4(p0.y * Rt_ptr[5] / MUL) + trunc4(p0.z * Rt_ptr[6]  / MUL) + Rt_ptr[7] ;
        tp0.z = trunc4(p0.x * Rt_ptr[8] / MUL) + trunc4(p0.y * Rt_ptr[9] / MUL) + trunc4(p0.z * Rt_ptr[10] / MUL) + Rt_ptr[11];

        Vec3d n1 = normals1.at<Vec3d>(v1, u1);
        Point3d v = cloud1.at<Point3d>(v1,u1) - tp0; //MUL

        tps0_ptr[correspIndex] = tp0; //MUL
        mp::cpp_int n1_x_mp(n1[0]);
        mp::cpp_int n1_y_mp(n1[1]);
        mp::cpp_int n1_z_mp(n1[2]);
        mp::cpp_int v_x_mp(v.x);
        mp::cpp_int v_y_mp(v.y);
        mp::cpp_int v_z_mp(v.z);
        mp::cpp_int diff_mp;
        diff_mp = (n1_x_mp * v_x_mp) + (n1_y_mp * v_y_mp) + (n1_z_mp * v_z_mp);
        diffs_ptr[correspIndex] = diff_mp.convert_to<long>();
        // diffs_ptr[correspIndex] = n1[0] * v.x + n1[1] * v.y + n1[2] * v.z; //MUL^2
        //std::cout << "====================test=======================" << diffs_ptr[0] <<  std::endl;
        //exit(1);
        sigma += diffs_ptr[correspIndex] * diffs_ptr[correspIndex]; //MUL^4
    }
    }

    sigma = trunc3(std::sqrt(trunc5(sigma/correspsCount))); //MUL^2

    long tmp;
    tmp = long(sigma_debug.at<double>(0,0));
    sigma_debug.at<double>(0,0) = sigma;
    sigma = tmp;

    // cout << "Rt " << iter << " : " << endl;   //pylin
    // cout << Rt << endl;   //pylin
    // cout << "sigma " << iter << " : " << long(sigma) << endl;    //pylin
    // cout << endl;   //pylin

    std::vector<long> A_buf(transformDim);
    long* A_ptr = &A_buf[0];

    // //↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ pylin
    // Mat debug;  //pylin
    // debug = Mat(1, 1, CV_64FC1, Scalar(1));    //pylin
    // ofstream debug_ICP("./debug_form_AtA_B.txt");      //pylin
    // //↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑ pylin
    mp::cpp_int AtA_00("0");
    mp::cpp_int AtA_01("0");
    mp::cpp_int AtA_02("0");
    mp::cpp_int AtA_03("0");
    mp::cpp_int AtA_04("0");
    mp::cpp_int AtA_05("0");
    mp::cpp_int AtA_11("0");
    mp::cpp_int AtA_12("0");
    mp::cpp_int AtA_13("0");
    mp::cpp_int AtA_14("0");
    mp::cpp_int AtA_15("0");
    mp::cpp_int AtA_22("0");
    mp::cpp_int AtA_23("0");
    mp::cpp_int AtA_24("0");
    mp::cpp_int AtA_25("0");
    mp::cpp_int AtA_33("0");
    mp::cpp_int AtA_34("0");
    mp::cpp_int AtA_35("0");
    mp::cpp_int AtA_44("0");
    mp::cpp_int AtA_45("0");
    mp::cpp_int AtA_55("0");
    mp::cpp_int AtB_mp[6];
    mp::cpp_int set0("0");
    for (int i = 0; i < 6; i++){
        AtB_mp[i] = set0;
    }

    ifstream if_1("./debug_check_Mat_00_SW1.txt");
    bool check_0to1 = if_1.good();
    if (check_0to1 == 0) {
        // ofstream of_1("./debug_check_test4_SW.txt");
        // ofstream of_2("./debug_check_A1_SW.txt");
        // ofstream of_3("./debug_check_A2_SW.txt");
        // ofstream of_4("./debug_check_A3_SW.txt");
        // ofstream of_5("./debug_check_A4_SW.txt");
        // ofstream of_6("./debug_check_A5_SW.txt");
        // ofstream of_7("./debug_check_diff_div_w_SW.txt");

        // ofstream of_00("./debug_check_A0_mul_A0_SW.txt");
        // ofstream of_10("./debug_check_A0_mul_A1_SW.txt");
        // ofstream of_20("./debug_check_A0_mul_A2_SW.txt");
        // ofstream of_30("./debug_check_A0_mul_A3_SW.txt");
        // ofstream of_40("./debug_check_A0_mul_A4_SW.txt");
        // ofstream of_50("./debug_check_A0_mul_A5_SW.txt");
        // ofstream of_11("./debug_check_A1_mul_A1_SW.txt");
        // ofstream of_21("./debug_check_A1_mul_A2_SW.txt");
        // ofstream of_31("./debug_check_A1_mul_A3_SW.txt");
        // ofstream of_41("./debug_check_A1_mul_A4_SW.txt");
        // ofstream of_51("./debug_check_A1_mul_A5_SW.txt");
        // ofstream of_22("./debug_check_A2_mul_A2_SW.txt");
        // ofstream of_32("./debug_check_A2_mul_A3_SW.txt");
        // ofstream of_42("./debug_check_A2_mul_A4_SW.txt");
        // ofstream of_52("./debug_check_A2_mul_A5_SW.txt");
        // ofstream of_33("./debug_check_A3_mul_A3_SW.txt");
        // ofstream of_43("./debug_check_A3_mul_A4_SW.txt");
        // ofstream of_53("./debug_check_A3_mul_A5_SW.txt");
        // ofstream of_44("./debug_check_A4_mul_A4_SW.txt");
        // ofstream of_54("./debug_check_A4_mul_A5_SW.txt");
        // ofstream of_55("./debug_check_A5_mul_A5_SW.txt");
        // ofstream of_0("./debug_check_A0_mul_diff_div_w_SW.txt");
        // ofstream of_1("./debug_check_A1_mul_diff_div_w_SW.txt");
        // ofstream of_2("./debug_check_A2_mul_diff_div_w_SW.txt");
        // ofstream of_3("./debug_check_A3_mul_diff_div_w_SW.txt");
        // ofstream of_4("./debug_check_A4_mul_diff_div_w_SW.txt");
        // ofstream of_5("./debug_check_A5_mul_diff_div_w_SW.txt");

        ofstream of_00("./debug_check_Mat_00_SW.txt");
        ofstream of_10("./debug_check_Mat_10_SW.txt");
        ofstream of_20("./debug_check_Mat_20_SW.txt");
        ofstream of_30("./debug_check_Mat_30_SW.txt");
        ofstream of_40("./debug_check_Mat_40_SW.txt");
        ofstream of_50("./debug_check_Mat_50_SW.txt");
        ofstream of_11("./debug_check_Mat_11_SW.txt");
        ofstream of_21("./debug_check_Mat_21_SW.txt");
        ofstream of_31("./debug_check_Mat_31_SW.txt");
        ofstream of_41("./debug_check_Mat_41_SW.txt");
        ofstream of_51("./debug_check_Mat_51_SW.txt");
        ofstream of_22("./debug_check_Mat_22_SW.txt");
        ofstream of_32("./debug_check_Mat_32_SW.txt");
        ofstream of_42("./debug_check_Mat_42_SW.txt");
        ofstream of_52("./debug_check_Mat_52_SW.txt");
        ofstream of_33("./debug_check_Mat_33_SW.txt");
        ofstream of_43("./debug_check_Mat_43_SW.txt");
        ofstream of_53("./debug_check_Mat_53_SW.txt");
        ofstream of_44("./debug_check_Mat_44_SW.txt");
        ofstream of_54("./debug_check_Mat_54_SW.txt");
        ofstream of_55("./debug_check_Mat_55_SW.txt");
        ofstream of_0("./debug_check_Vec_0_SW.txt");
        ofstream of_1("./debug_check_Vec_1_SW.txt");
        ofstream of_2("./debug_check_Vec_2_SW.txt");
        ofstream of_3("./debug_check_Vec_3_SW.txt");
        ofstream of_4("./debug_check_Vec_4_SW.txt");
        ofstream of_5("./debug_check_Vec_5_SW.txt");
    for(int correspIndex = 0; correspIndex < corresps.rows; correspIndex++)
    {
        const Vec4i& c = corresps_ptr[correspIndex];
        int u1 = c[2], v1 = c[3]; // dst

        long w = sigma + std::abs(diffs_ptr[correspIndex]); //MUL^2
        //w = w > DBL_EPSILON ? 1./w : 1.;
        // of_1 << long(w) << endl; //w

        //func(A_ptr, tps0_ptr[correspIndex], normals1.at<Vec3d>(v1, u1) * w);
        mp::cpp_int A_mp[6];
        mp::cpp_int n1_x_mp(normals1.at<Vec3d>(v1, u1)[0]);
        mp::cpp_int n1_y_mp(normals1.at<Vec3d>(v1, u1)[1]);
        mp::cpp_int n1_z_mp(normals1.at<Vec3d>(v1, u1)[2]);
        mp::cpp_int tp0_x_mp(tps0_ptr[correspIndex].x);
        mp::cpp_int tp0_y_mp(tps0_ptr[correspIndex].y);
        mp::cpp_int tp0_z_mp(tps0_ptr[correspIndex].z);
        mp::cpp_int MUL_mp(MUL);
        mp::cpp_int w_mp(w);
        mp::cpp_int diff_mp(diffs_ptr[correspIndex]);
        mp::cpp_int diff_div_w_mp;
        long diff_div_w;

        diff_div_w_mp = diff_mp * MUL_mp / w_mp;
        diff_div_w = diff_div_w_mp.convert_to<long>();
        // of_7 << diff_div_w << endl;

        A_mp[0] = ((tp0_y_mp * n1_z_mp) - (tp0_z_mp * n1_y_mp)) * MUL_mp / w_mp;
        A_mp[1] = ((tp0_z_mp * n1_x_mp) - (tp0_x_mp * n1_z_mp)) * MUL_mp / w_mp;
        A_mp[2] = ((tp0_x_mp * n1_y_mp) - (tp0_y_mp * n1_x_mp)) * MUL_mp / w_mp;
        A_ptr[0] = A_mp[0].convert_to<long>();
        A_ptr[1] = A_mp[1].convert_to<long>();
        A_ptr[2] = A_mp[2].convert_to<long>();
        // A_ptr[0] =trunc4((-tps0_ptr[correspIndex].z * normals1.at<Vec3d>(v1, u1)[1] + tps0_ptr[correspIndex].y * normals1.at<Vec3d>(v1, u1)[2]) * MUL / w);
        // A_ptr[1] =trunc4(( tps0_ptr[correspIndex].z * normals1.at<Vec3d>(v1, u1)[0] - tps0_ptr[correspIndex].x * normals1.at<Vec3d>(v1, u1)[2]) * MUL / w);
        // A_ptr[2] =trunc4((-tps0_ptr[correspIndex].y * normals1.at<Vec3d>(v1, u1)[0] + tps0_ptr[correspIndex].x * normals1.at<Vec3d>(v1, u1)[1]) * MUL / w);
        // A_ptr[0] =trunc4(-tps0_ptr[correspIndex].z * normals1.at<Vec3d>(v1, u1)[1] * MUL /  w) + trunc4(tps0_ptr[correspIndex].y * normals1.at<Vec3d>(v1, u1)[2] * MUL / w);
        // A_ptr[1] =trunc4( tps0_ptr[correspIndex].z * normals1.at<Vec3d>(v1, u1)[0] * MUL /  w) - trunc4(tps0_ptr[correspIndex].x * normals1.at<Vec3d>(v1, u1)[2] * MUL / w);
        // A_ptr[2] =trunc4(-tps0_ptr[correspIndex].y * normals1.at<Vec3d>(v1, u1)[0] * MUL /  w) + trunc4(tps0_ptr[correspIndex].x * normals1.at<Vec3d>(v1, u1)[1] * MUL / w);
        A_ptr[3] =trunc4(normals1.at<Vec3d>(v1, u1)[0] * MUL * MUL / w);
        A_ptr[4] =trunc4(normals1.at<Vec3d>(v1, u1)[1] * MUL * MUL / w);
        A_ptr[5] =trunc4(normals1.at<Vec3d>(v1, u1)[2] * MUL * MUL / w);
        A_mp[3] = mp::cpp_int(A_ptr[3]);
        A_mp[4] = mp::cpp_int(A_ptr[4]);
        A_mp[5] = mp::cpp_int(A_ptr[5]);
        mp::cpp_int r1, r1_tmp;
        mp::cpp_int r2;

        for(int y = 0; y < transformDim; y++)
        {
            double* AtA_ptr = AtA.ptr<double>(y);
            for(int x = y; x < transformDim; x++){
                // AtA_ptr[x] += trunc4(A_ptr[y] * A_ptr[x] / MUL);
                r1 = (A_mp[y] * A_mp[x]) >> 24;
                if (y==0 && x==0) AtA_00 = AtA_00 + r1;
                else if (y==0 && x==1) AtA_01 = AtA_01 + r1;
                else if (y==0 && x==2) AtA_02 = AtA_02 + r1;
                else if (y==0 && x==3) AtA_03 = AtA_03 + r1;
                else if (y==0 && x==4) AtA_04 = AtA_04 + r1;
                else if (y==0 && x==5) AtA_05 = AtA_05 + r1;
                else if (y==1 && x==1) AtA_11 = AtA_11 + r1;
                else if (y==1 && x==2) AtA_12 = AtA_12 + r1;
                else if (y==1 && x==3) AtA_13 = AtA_13 + r1;
                else if (y==1 && x==4) AtA_14 = AtA_14 + r1;
                else if (y==1 && x==5) AtA_15 = AtA_15 + r1;
                else if (y==2 && x==2) AtA_22 = AtA_22 + r1;
                else if (y==2 && x==3) AtA_23 = AtA_23 + r1;
                else if (y==2 && x==4) AtA_24 = AtA_24 + r1;
                else if (y==2 && x==5) AtA_25 = AtA_25 + r1;
                else if (y==3 && x==3) AtA_33 = AtA_33 + r1;
                else if (y==3 && x==4) AtA_34 = AtA_34 + r1;
                else if (y==3 && x==5) AtA_35 = AtA_35 + r1;
                else if (y==4 && x==4) AtA_44 = AtA_44 + r1;
                else if (y==4 && x==5) AtA_45 = AtA_45 + r1;
                else if (y==5 && x==5) AtA_55 = AtA_55 + r1;
                AtA_ptr[x] = long(AtA_ptr[x]) + r1.convert_to<long>();
            }
            r2 = (A_mp[y] * diff_div_w_mp) >> 24;
            // AtB_ptr[y] += trunc4(A_ptr[y] * diffs_ptr[correspIndex] / w);
            AtB_mp[y] = AtB_mp[y] + r2;
            AtB_ptr[y] = trunc4(AtB_ptr[y] + trunc4(r2.convert_to<long>()));
        }
        // debug.at<double>(0,0) = 0; // pylin
        // AtA.at<int64>(0,0) = AtA_00.convert_to<long>();
        // AtA.at<double>(0,1) = AtA_01.convert_to<long>();
        // AtA.at<double>(0,2) = AtA_02.convert_to<long>();
        // AtA.at<double>(0,3) = AtA_03.convert_to<long>();
        // AtA.at<double>(0,4) = AtA_04.convert_to<long>();
        // AtA.at<double>(0,5) = AtA_05.convert_to<long>();
        // AtA.at<long>(1,1) = AtA_11.convert_to<long>();
        // AtA.at<double>(1,2) = AtA_12.convert_to<long>();
        // AtA.at<double>(1,3) = AtA_13.convert_to<long>();
        // AtA.at<double>(1,4) = AtA_14.convert_to<long>();
        // AtA.at<double>(1,5) = AtA_15.convert_to<long>();
        // AtA.at<double>(2,2) = AtA_22.convert_to<long>();
        // AtA.at<double>(2,3) = AtA_23.convert_to<long>();
        // AtA.at<double>(2,4) = AtA_24.convert_to<long>();
        // AtA.at<double>(2,5) = AtA_25.convert_to<long>();
        // AtA.at<double>(3,3) = AtA_33.convert_to<long>();
        // AtA.at<double>(3,4) = AtA_34.convert_to<long>();
        // AtA.at<double>(3,5) = AtA_35.convert_to<long>();
        // AtA.at<double>(4,4) = AtA_44.convert_to<long>();
        // AtA.at<double>(4,5) = AtA_45.convert_to<long>();
        // AtA.at<double>(5,5) = AtA_55.convert_to<long>();
        // of_00 << AtA_00 << endl;
        // of_10 << AtA_01 << endl;
        // of_20 << AtA_02 << endl;
        // of_30 << AtA_03 << endl;
        // of_40 << AtA_04 << endl;
        // of_50 << AtA_05 << endl;
        // of_11 << AtA_11 << endl;
        // of_21 << AtA_12 << endl;
        // of_31 << AtA_13 << endl;
        // of_41 << AtA_14 << endl;
        // of_51 << AtA_15 << endl;
        // of_22 << AtA_22 << endl;
        // of_32 << AtA_23 << endl;
        // of_42 << AtA_24 << endl;
        // of_52 << AtA_25 << endl;
        // of_33 << AtA_33 << endl;
        // of_43 << AtA_34 << endl;
        // of_53 << AtA_35 << endl;
        // of_44 << AtA_44 << endl;
        // of_54 << AtA_45 << endl;
        // of_55 << AtA_55 << endl;
        // of_0 << long( AtB_ptr[0] ) << endl;
        // of_1 << long( AtB_ptr[1] ) << endl;
        // of_2 << long( AtB_ptr[2] ) << endl;
        // of_3 << long( AtB_ptr[3] ) << endl;
        // of_4 << long( AtB_ptr[4] ) << endl;
        // of_5 << long( AtB_ptr[5] ) << endl;

        of_00 << long( AtA.at<double>(0,0) ) << endl;
        of_10 << long( AtA.at<double>(0,1) ) << endl;
        of_20 << long( AtA.at<double>(0,2) ) << endl;
        of_30 << long( AtA.at<double>(0,3) ) << endl;
        of_40 << long( AtA.at<double>(0,4) ) << endl;
        of_50 << long( AtA.at<double>(0,5) ) << endl;
        of_11 << long( AtA.at<double>(1,1) ) << endl;
        of_21 << long( AtA.at<double>(1,2) ) << endl;
        of_31 << long( AtA.at<double>(1,3) ) << endl;
        of_41 << long( AtA.at<double>(1,4) ) << endl;
        of_51 << long( AtA.at<double>(1,5) ) << endl;
        of_22 << long( AtA.at<double>(2,2) ) << endl;
        of_32 << long( AtA.at<double>(2,3) ) << endl;
        of_42 << long( AtA.at<double>(2,4) ) << endl;
        of_52 << long( AtA.at<double>(2,5) ) << endl;
        of_33 << long( AtA.at<double>(3,3) ) << endl;
        of_43 << long( AtA.at<double>(3,4) ) << endl;
        of_53 << long( AtA.at<double>(3,5) ) << endl;
        of_44 << long( AtA.at<double>(4,4) ) << endl;
        of_54 << long( AtA.at<double>(4,5) ) << endl;
        of_55 << long( AtA.at<double>(5,5) ) << endl;
        of_0 << long( AtB_ptr[0] ) << endl;
        of_1 << long( AtB_ptr[1] ) << endl;
        of_2 << long( AtB_ptr[2] ) << endl;
        of_3 << long( AtB_ptr[3] ) << endl;
        of_4 << long( AtB_ptr[4] ) << endl;
        of_5 << long( AtB_ptr[5] ) << endl;
    }
    }

    else {
    for(int correspIndex = 0; correspIndex < corresps.rows; correspIndex++)
    {
        const Vec4i& c = corresps_ptr[correspIndex];
        int u1 = c[2], v1 = c[3]; // dst

        long w = sigma + std::abs(diffs_ptr[correspIndex]); //MUL^2
        //w = w > DBL_EPSILON ? 1./w : 1.;

        //func(A_ptr, tps0_ptr[correspIndex], normals1.at<Vec3d>(v1, u1) * w);
        mp::cpp_int A0_mp, A1_mp, A2_mp;
        mp::cpp_int n1_x_mp(normals1.at<Vec3d>(v1, u1)[0]);
        mp::cpp_int n1_y_mp(normals1.at<Vec3d>(v1, u1)[1]);
        mp::cpp_int n1_z_mp(normals1.at<Vec3d>(v1, u1)[2]);
        mp::cpp_int tp0_x_mp(tps0_ptr[correspIndex].x);
        mp::cpp_int tp0_y_mp(tps0_ptr[correspIndex].y);
        mp::cpp_int tp0_z_mp(tps0_ptr[correspIndex].z);
        mp::cpp_int MUL_mp(MUL);
        mp::cpp_int w_mp(w);
        A0_mp = ((tp0_y_mp * n1_z_mp) - (tp0_z_mp * n1_y_mp)) * MUL_mp / w_mp;
        A1_mp = ((tp0_z_mp * n1_x_mp) - (tp0_x_mp * n1_z_mp)) * MUL_mp / w_mp;
        A2_mp = ((tp0_x_mp * n1_y_mp) - (tp0_y_mp * n1_x_mp)) * MUL_mp / w_mp;
        A_ptr[0] = A0_mp.convert_to<long>();
        A_ptr[1] = A1_mp.convert_to<long>();
        A_ptr[2] = A2_mp.convert_to<long>();
        // A_ptr[0] =trunc4(-tps0_ptr[correspIndex].z * normals1.at<Vec3d>(v1, u1)[1] * MUL /  w) + trunc4(tps0_ptr[correspIndex].y * normals1.at<Vec3d>(v1, u1)[2] * MUL / w);
        // A_ptr[1] =trunc4( tps0_ptr[correspIndex].z * normals1.at<Vec3d>(v1, u1)[0] * MUL /  w) - trunc4(tps0_ptr[correspIndex].x * normals1.at<Vec3d>(v1, u1)[2] * MUL / w);
        // A_ptr[2] =trunc4(-tps0_ptr[correspIndex].y * normals1.at<Vec3d>(v1, u1)[0] * MUL /  w) + trunc4(tps0_ptr[correspIndex].x * normals1.at<Vec3d>(v1, u1)[1] * MUL / w);
        A_ptr[3] =trunc4(normals1.at<Vec3d>(v1, u1)[0] * MUL * MUL / w);
        A_ptr[4] =trunc4(normals1.at<Vec3d>(v1, u1)[1] * MUL * MUL / w);
        A_ptr[5] =trunc4(normals1.at<Vec3d>(v1, u1)[2] * MUL * MUL / w);

        for(int y = 0; y < transformDim; y++)
        {
            double* AtA_ptr = AtA.ptr<double>(y);
            for(int x = y; x < transformDim; x++){
                AtA_ptr[x] += trunc4(A_ptr[y] * A_ptr[x] / MUL);
            }
            AtB_ptr[y] += trunc4(A_ptr[y] * diffs_ptr[correspIndex] / w);
        }
    }
    }

    for(int y = 0; y < transformDim; y++)
        for(int x = y+1; x < transformDim; x++)
            AtA.at<double>(x,y) = AtA.at<double>(y,x);
}

void calcFeatureLsmMatrices(const int iter, 
                            const Mat& cloud0, 
                            const Mat& Rt,
                            const Mat& corresps, 
                            double fx, double fy, double cx, double cy,
                            Mat& AtA, Mat& AtB, 
                            CalcFeatureXEquationCoeffsPtr func_x, 
                            CalcFeatureYEquationCoeffsPtr func_y, 
                            int transformDim)
{
    AtA = Mat(transformDim, transformDim, CV_64FC1, Scalar(0)); //transformDim = 6
    AtB = Mat(transformDim, 1, CV_64FC1, Scalar(0));            //transformDim = 6
    double* AtB_ptr = AtB.ptr<double>();

    const int correspsCount = corresps.rows;  //匹配點數量

    CV_Assert(Rt.type() == CV_64FC1);
    const double * Rt_ptr = Rt.ptr<const double>();

    AutoBuffer<double> diffs_x(correspsCount);
    AutoBuffer<double> diffs_y(correspsCount);
    double* diffs_x_ptr = diffs_x;
    double* diffs_y_ptr = diffs_y;

    AutoBuffer<Point3d> transformedPoints0(correspsCount);
    Point3d * tps0_ptr = transformedPoints0;

    const Vec4i* corresps_ptr = corresps.ptr<Vec4i>();

    double sigma_x = 0;
    double sigma_y = 0;
    for(int correspIndex = 0; correspIndex < corresps.rows; correspIndex++)
    {
        const Vec4i& c = corresps_ptr[correspIndex];
        int u0 = c[0], v0 = c[1];
        int u1 = c[2], v1 = c[3];
    
        const Point3d& p0 = cloud0.at<Point3d>(v0,u0);
        Point3d tp0;
        // [P'] = [R|t][P] -- p.166(7.37)
        tp0.x = trunc_lsm_f1(p0.x * Rt_ptr[0] / MUL) + trunc_lsm_f1(p0.y * Rt_ptr[1] / MUL) + trunc_lsm_f1(p0.z * Rt_ptr[2]  / MUL) + Rt_ptr[3] ;
        tp0.y = trunc_lsm_f1(p0.x * Rt_ptr[4] / MUL) + trunc_lsm_f1(p0.y * Rt_ptr[5] / MUL) + trunc_lsm_f1(p0.z * Rt_ptr[6]  / MUL) + Rt_ptr[7] ;
        tp0.z = trunc_lsm_f1(p0.x * Rt_ptr[8] / MUL) + trunc_lsm_f1(p0.y * Rt_ptr[9] / MUL) + trunc_lsm_f1(p0.z * Rt_ptr[10] / MUL) + Rt_ptr[11];
        //mpz_t p0_x;
        //mpz_t p0_y;
        //mpz_t p0_z;
        //mpz_t Rt_4;
        //mpz_t Rt_5;
        //mpz_t Rt_6;
        //mpz_t MUL_gmp;
        //mpz_init_set_d(MUL_gmp, MUL);
        //mpz_init_set_d(p0_x, p0.x);
        //mpz_init_set_d(p0_y, p0.y);
        //mpz_init_set_d(p0_z, p0.z);
        //mpz_init_set_d(Rt_4, Rt_ptr[4]);
        //mpz_init_set_d(Rt_5, Rt_ptr[5]);
        //mpz_init_set_d(Rt_6, Rt_ptr[6]);
        //mpz_mul(Rt_4,p0_x,Rt_4);
        //mpz_mul(Rt_5,p0_y,Rt_5);
        //mpz_mul(Rt_6,p0_z,Rt_6);
        //mpz_div(Rt_4,Rt_4,MUL_gmp);
        //mpz_div(Rt_5,Rt_5,MUL_gmp);
        //mpz_div(Rt_6,Rt_6,MUL_gmp);
        //double test4 = mpz_get_d(Rt_4);
        //double test5 = mpz_get_d(Rt_5);
        //double test6 = mpz_get_d(Rt_6);
        //tp0.y = trunc_lsm_f1(test4) + trunc_lsm_f1(test5) + trunc_lsm_f1(test6) + Rt_ptr[7] ;
        //cout << "p0_x " << fixed << p0.x << endl;
        //cout << "Rt_4 " << fixed << Rt_ptr[4] << endl;
        //cout << "Rt_5 " << fixed << Rt_ptr[5] << endl;
        //cout << "Rt_6 " << fixed << Rt_ptr[6] << endl;
        //cout << "tp0.y " << fixed << tp0.y << endl;
        //exit(1);
        //int p2d_x = cvRound( (trunc1(trunc1(fx * MUL) * tp0.x / tp0.z) + trunc1(cx * MUL)) / MUL);
        //int p2d_y = cvRound( (trunc1(trunc1(fy * MUL) * tp0.y / tp0.z) + trunc1(cy * MUL)) / MUL);

        // u = fx*X'/Z' + cx ; v = fy*Y'/Z' + cy  --  p.166(7.39)、(7.40)
        int p2d_x = trunc_lsm_f2( (trunc_lsm_f2(trunc_lsm_f2(fx * MUL) * tp0.x / tp0.z) + trunc_lsm_f2(cx * MUL)) / MUL);
        int p2d_y = trunc_lsm_f2( (trunc_lsm_f2(trunc_lsm_f2(fy * MUL) * tp0.y / tp0.z) + trunc_lsm_f2(cy * MUL)) / MUL);

        tps0_ptr[correspIndex] = tp0;
        //diffs_x_ptr[correspIndex] = p2d_x - u1;
        //diffs_y_ptr[correspIndex] = p2d_y - v1;
        diffs_x_ptr[correspIndex] = u1 - p2d_x;
        diffs_y_ptr[correspIndex] = v1 - p2d_y;
        sigma_x += diffs_x_ptr[correspIndex] * diffs_x_ptr[correspIndex];
        sigma_y += diffs_y_ptr[correspIndex] * diffs_y_ptr[correspIndex];
   
        //debug
        //cout << "u0 " << u0 << endl;
        //cout << "v0 " << v0 << endl;
        //cout << "u1 " << u1 << endl;
        //cout << "v1 " << v1 << endl;
        //int z0 = trunc_lsm_f1(p0.z/MUL);
        //cout << "z0 " << hex << z0 << endl;
        ///cout << "Rt[0] "  << fixed << Rt_ptr[0] << endl;
        ///cout << "Rt[1] "  << fixed << Rt_ptr[1] << endl;
        ///cout << "Rt[2] "  << fixed << Rt_ptr[2] << endl;
        ///cout << "Rt[3] "  << fixed << Rt_ptr[3] << endl;
        ///cout << "Rt[4] "  << fixed << Rt_ptr[4] << endl;
        ///cout << "Rt[5] "  << fixed << Rt_ptr[5] << endl;
        ///cout << "Rt[6] "  << fixed << Rt_ptr[6] << endl;
        ///cout << "Rt[7] "  << fixed << Rt_ptr[7] << endl;
        ///cout << "Rt[8] "  << fixed << Rt_ptr[8] << endl;
        ///cout << "Rt[9] "  << fixed << Rt_ptr[9] << endl;
        ///cout << "Rt[10] " << fixed << Rt_ptr[10] << endl;
        ///cout << "Rt[11] " << fixed << Rt_ptr[11] << endl;
        ///cout << "p0.x " << setw(14) << setprecision(0) << fixed << tp0.x << endl;
        ///cout << "p0.y " << setw(14) << setprecision(0) << fixed << tp0.y << endl;
        ///cout << "p0.z " << setw(14) << setprecision(0) << fixed << tp0.z << endl;
        ///if(iter==1)
        ///    exit(1);
        //cout << "x:" << setw(4) << p2d_x << endl;
        //cout << "y:" << setw(4) << p2d_y << endl;
        //cout << "x:" << setw(4) << u1 - p2d_x << endl;
        //cout << "y:" << setw(4) << v1 - p2d_y << endl;
    }
    //if(iter==1)
    //  exit(1);

    sigma_x = trunc_lsm_f2(std::sqrt(trunc_lsm_f2(sigma_x/correspsCount)));
    sigma_y = trunc_lsm_f2(std::sqrt(trunc_lsm_f2(sigma_y/correspsCount)));

    std::vector<double> A_buf_x(transformDim);  //transformDim = 6
    std::vector<double> A_buf_y(transformDim);  //transformDim = 6
    double* A_ptr_x = &A_buf_x[0];
    double* A_ptr_y = &A_buf_y[0];
    for(int correspIndex = 0; correspIndex < corresps.rows; correspIndex++)
    {
        double w_x = sigma_x + std::abs(diffs_x_ptr[correspIndex]);
        double w_y = sigma_y + std::abs(diffs_y_ptr[correspIndex]);
        w_x = w_x > DBL_EPSILON ? 1./w_x : 1.;
        w_y = w_y > DBL_EPSILON ? 1./w_y : 1.;

        //func_x(A_ptr_x, tps0_ptr[correspIndex], fx * w_x);
        double z_squared = trunc3(tps0_ptr[correspIndex].z * tps0_ptr[correspIndex].z);
        // -fx*X'*Y'/(Z'^2)  --  p.167 (7.45)
        A_ptr_x[0] = -( trunc_lsm_f3( trunc_lsm_f3(fx * MUL) * tps0_ptr[correspIndex].x * tps0_ptr[correspIndex].y / z_squared ) );
        // fx + (fx*X'*X')/(Z'^2)  --  p.167 (7.45)
        A_ptr_x[1] = trunc_lsm_f3(fx * MUL) + trunc_lsm_f3( trunc_lsm_f3(fx * MUL) * tps0_ptr[correspIndex].x * tps0_ptr[correspIndex].x / z_squared);
        // -fx*Y'/Z'  --  p.167 (7.45)
        A_ptr_x[2] = -( trunc_lsm_f3( trunc_lsm_f3(fx * MUL) * tps0_ptr[correspIndex].y / tps0_ptr[correspIndex].z ) );
        // fx/Z'  --  p.167 (7.45)
        A_ptr_x[3] = trunc_lsm_f3( trunc_lsm_f3(fx * MUL) * MUL / tps0_ptr[correspIndex].z );
        // 0  --  p.167 (7.45)
        A_ptr_x[4] = 0;
        // -fx*X'/(Z'^2)  --  p.167 (7.45)
        A_ptr_x[5] = -( trunc_lsm_f3(trunc_lsm_f3(fx * MUL)  * tps0_ptr[correspIndex].x * MUL  / z_squared) );

        //func_y(A_ptr_y, tps0_ptr[correspIndex], fy * w_y);
        // fy - (fy*Y'*Y')/(Z'^2)  --  p.167 (7.45)
        A_ptr_y[0] = -trunc_lsm_f3(fy * MUL) - trunc_lsm_f3(trunc_lsm_f3(fy * MUL) * tps0_ptr[correspIndex].y * tps0_ptr[correspIndex].y /z_squared);
        //A_ptr_y[1] = trunc_lsm_f3(trunc_lsm_f3(fy * MUL) * tps0_ptr[correspIndex].x * tps0_ptr[correspIndex].x / z_squared);
        // fy*X'*Y'/(Z'^2)  --  p.167 (7.45)
        A_ptr_y[1] = trunc_lsm_f3(trunc_lsm_f3(fy * MUL) * tps0_ptr[correspIndex].x * tps0_ptr[correspIndex].y / z_squared);
        // -fy*X'/Z'  --  p.167 (7.45)
        A_ptr_y[2] = trunc_lsm_f3(trunc_lsm_f3(fy * MUL) * tps0_ptr[correspIndex].x / tps0_ptr[correspIndex].z);
        // 0  --  p.167 (7.45)
        A_ptr_y[3] = 0;
        // fy/Z'  --  p.167 (7.45)
        A_ptr_y[4] = trunc_lsm_f3(trunc_lsm_f3(fy * MUL) * MUL / tps0_ptr[correspIndex].z);
        // -fy*Y'/(Z'^2)  --  p.167 (7.45)
        A_ptr_y[5] = -trunc_lsm_f3(trunc_lsm_f3(fy * MUL) * tps0_ptr[correspIndex].y * MUL /z_squared);
        
        for(int y = 0; y < transformDim; y++)   //transformDim = 6
        {
            double* AtA_ptr = AtA.ptr<double>(y);
            for(int x = y; x < transformDim; x++)   //transformDim = 6
            {
                //AtA_ptr[x] += trunc1(A_ptr_x[y] * A_ptr_x[x] * w_x * w_x / MUL) + trunc1(A_ptr_y[y] * A_ptr_y[x] * w_y * w_y / MUL);
                //HW friendly
                //AtA_ptr[x] += trunc_lsm_f5(trunc_lsm_f4(A_ptr_x[y] * A_ptr_x[x] / MUL) + trunc_lsm_f4(A_ptr_y[y] * A_ptr_y[x] / MUL));
                //double miss precision
                mpz_t A_ptr_x_x;
                mpz_t A_ptr_x_y;
                mpz_t A_ptr_y_x;
                mpz_t A_ptr_y_y;
                mpz_t MUL_gmp;
                mpz_t AtA_gmp;
                mpz_init_set_d(A_ptr_x_x, A_ptr_x[x]);
                mpz_init_set_d(A_ptr_x_y, A_ptr_x[y]);
                mpz_init_set_d(A_ptr_y_x, A_ptr_y[x]);
                mpz_init_set_d(A_ptr_y_y, A_ptr_y[y]);
                mpz_init_set_d(MUL_gmp, MUL);
                mpz_init_set_d(AtA_gmp, AtA_ptr[x]);
                mpz_mul(A_ptr_x_x,A_ptr_x_x,A_ptr_x_y);
                mpz_div(A_ptr_x_x,A_ptr_x_x,MUL_gmp);
                mpz_mul(A_ptr_y_y,A_ptr_y_x,A_ptr_y_y);
                mpz_div(A_ptr_y_y,A_ptr_y_y,MUL_gmp);
                mpz_add(A_ptr_x_x,A_ptr_x_x,A_ptr_y_y);
                mpz_add(AtA_gmp,AtA_gmp,A_ptr_x_x);
                AtA_ptr[x] = mpz_get_d(AtA_gmp);
                mpz_clear(A_ptr_x_x);
                mpz_clear(A_ptr_x_y);
                mpz_clear(A_ptr_y_x);
                mpz_clear(A_ptr_y_y);
                mpz_clear(MUL_gmp);
                mpz_clear(AtA_gmp);
            } 
            //AtB_ptr[y] += trunc1(A_ptr_x[y] * w_x * w_x * diffs_x_ptr[correspIndex]) + trunc1(A_ptr_y[y] * w_y * w_y * diffs_y_ptr[correspIndex]);
            //HW friendly
            //AtB_ptr[y] += trunc_lsm_f5(trunc_lsm_f4(A_ptr_x[y] * diffs_x_ptr[correspIndex]) + trunc_lsm_f4(A_ptr_y[y] * diffs_y_ptr[correspIndex]));
             mpz_t A_ptr_x_gmp;
             mpz_t A_ptr_y_gmp;
             mpz_t diffs_x_gmp;
             mpz_t diffs_y_gmp;
             mpz_t AtB_gmp;
            mpz_init_set_d(A_ptr_x_gmp, A_ptr_x[y]);
            mpz_init_set_d(A_ptr_y_gmp, A_ptr_y[y]);
            mpz_init_set_d(diffs_x_gmp, diffs_x_ptr[correspIndex]);
            mpz_init_set_d(diffs_y_gmp, diffs_y_ptr[correspIndex]);
            mpz_mul(diffs_x_gmp, diffs_x_gmp, A_ptr_x_gmp);
            mpz_mul(diffs_y_gmp, diffs_y_gmp, A_ptr_y_gmp);
            mpz_add(diffs_x_gmp, diffs_x_gmp, diffs_y_gmp);
            mpz_init_set_d(AtB_gmp, AtB_ptr[y]);
            mpz_add(AtB_gmp, AtB_gmp, diffs_x_gmp);
            AtB_ptr[y] = mpz_get_d(AtB_gmp);
            mpz_clear(A_ptr_x_gmp);
            mpz_clear(A_ptr_y_gmp);
            mpz_clear(diffs_x_gmp);
            mpz_clear(diffs_y_gmp);
            mpz_clear(AtB_gmp);
        }
        //cout << "idx " << correspIndex << endl;
        //cout << "Ax[0] " << setw(13) << setprecision(0) << fixed << A_ptr_x[0] << endl;
        //cout << "Ax[1] " << setw(13) << setprecision(0) << fixed << A_ptr_x[1] << endl;
        //cout << "Ax[2] " << setw(13) << setprecision(0) << fixed << A_ptr_x[2] << endl;
        //cout << "Ax[3] " << setw(13) << setprecision(0) << fixed << A_ptr_x[3] << endl;
        //cout << "Ax[4] " << setw(13) << setprecision(0) << fixed << A_ptr_x[4] << endl;
        //cout << "Ax[5] " << setw(13) << setprecision(0) << fixed << A_ptr_x[5] << endl;
        //cout << "Ay[0] " << setw(13) << setprecision(0) << fixed << A_ptr_y[0] << endl;
        //cout << "Ay[1] " << setw(13) << setprecision(0) << fixed << A_ptr_y[1] << endl;
        //cout << "Ay[2] " << setw(13) << setprecision(0) << fixed << A_ptr_y[2] << endl;
        //cout << "Ay[3] " << setw(13) << setprecision(0) << fixed << A_ptr_y[3] << endl;
        //cout << "Ay[4] " << setw(13) << setprecision(0) << fixed << A_ptr_y[4] << endl;
        //cout << "Ay[5] " << setw(13) << setprecision(0) << fixed << A_ptr_y[5] << endl;
        //cout << "AtA[0][1]_tmpx_pre " << setw(17) << setprecision(0) << fixed << A_ptr_x[0] * A_ptr_x[1] << endl;
        //cout << "AtA[0][1]_tmpy_pre " << setw(17) << setprecision(0) << fixed << A_ptr_y[0] * A_ptr_y[1] << endl;
        //cout << "AtA[0][1]_tmpx " << setw(17) << setprecision(0) << fixed << trunc_lsm_f4(A_ptr_x[0] * A_ptr_x[1] / MUL) << endl;
        //cout << "AtA[0][1]_tmpy " << setw(17) << setprecision(0) << fixed << trunc_lsm_f4(A_ptr_y[0] * A_ptr_y[1] / MUL) << endl;
        //cout << "AtA[0][1] " << setw(17) << setprecision(0) << fixed << AtA.at<double>(0,1) << endl;
        //cout << "diffs_x " << setw(17) << setprecision(0) << fixed << diffs_x_ptr[correspIndex] << endl;
        //cout << "diffs_y " << setw(17) << setprecision(0) << fixed << diffs_y_ptr[correspIndex] << endl;
        //cout << "AtB[0] " << setw(17) << setprecision(0) << fixed << AtB_ptr[0] << endl;
        //exit(1);
    }
    //exit(1);

    for(int y = 0; y < transformDim; y++)       //transformDim = 6
        for(int x = y+1; x < transformDim; x++) //transformDim = 6
            AtA.at<double>(x,y) = AtA.at<double>(y,x);

    //cout << "AtA[0][0] " << setw(13) << setprecision(0) << fixed << AtA.at<double>(0,0) << endl;
    //cout << "AtA[1][0] " << setw(13) << setprecision(0) << fixed << AtA.at<double>(1,0) << endl;
    //cout << "AtA[2][0] " << setw(13) << setprecision(0) << fixed << AtA.at<double>(2,0) << endl;
    //cout << "AtA[3][0] " << setw(13) << setprecision(0) << fixed << AtA.at<double>(3,0) << endl;
    //cout << "AtA[4][0] " << setw(13) << setprecision(0) << fixed << AtA.at<double>(4,0) << endl;
    //cout << "AtA[5][0] " << setw(13) << setprecision(0) << fixed << AtA.at<double>(5,0) << endl;
    //cout << "AtA[0][1] " << setw(13) << setprecision(0) << fixed << AtA.at<double>(0,1) << endl;
    //cout << "AtA[1][1] " << setw(13) << setprecision(0) << fixed << AtA.at<double>(1,1) << endl;
    //cout << "AtA[2][1] " << setw(13) << setprecision(0) << fixed << AtA.at<double>(2,1) << endl;
    //cout << "AtA[3][1] " << setw(13) << setprecision(0) << fixed << AtA.at<double>(3,1) << endl;
    //cout << "AtA[4][1] " << setw(13) << setprecision(0) << fixed << AtA.at<double>(4,1) << endl;
    //cout << "AtA[5][1] " << setw(13) << setprecision(0) << fixed << AtA.at<double>(5,1) << endl;
    //cout << "AtA[0][2] " << setw(13) << setprecision(0) << fixed << AtA.at<double>(0,2) << endl;
    //cout << "AtA[1][2] " << setw(13) << setprecision(0) << fixed << AtA.at<double>(1,2) << endl;
    //cout << "AtA[2][2] " << setw(13) << setprecision(0) << fixed << AtA.at<double>(2,2) << endl;
    //cout << "AtA[3][2] " << setw(13) << setprecision(0) << fixed << AtA.at<double>(3,2) << endl;
    //cout << "AtA[4][2] " << setw(13) << setprecision(0) << fixed << AtA.at<double>(4,2) << endl;
    //cout << "AtA[5][2] " << setw(13) << setprecision(0) << fixed << AtA.at<double>(5,2) << endl;
    //cout << "AtA[0][3] " << setw(13) << setprecision(0) << fixed << AtA.at<double>(0,3) << endl;
    //cout << "AtA[1][3] " << setw(13) << setprecision(0) << fixed << AtA.at<double>(1,3) << endl;
    //cout << "AtA[2][3] " << setw(13) << setprecision(0) << fixed << AtA.at<double>(2,3) << endl;
    //cout << "AtA[3][3] " << setw(13) << setprecision(0) << fixed << AtA.at<double>(3,3) << endl;
    //cout << "AtA[4][3] " << setw(13) << setprecision(0) << fixed << AtA.at<double>(4,3) << endl;
    //cout << "AtA[5][3] " << setw(13) << setprecision(0) << fixed << AtA.at<double>(5,3) << endl;
    //cout << "AtA[0][4] " << setw(13) << setprecision(0) << fixed << AtA.at<double>(0,4) << endl;
    //cout << "AtA[1][4] " << setw(13) << setprecision(0) << fixed << AtA.at<double>(1,4) << endl;
    //cout << "AtA[2][4] " << setw(13) << setprecision(0) << fixed << AtA.at<double>(2,4) << endl;
    //cout << "AtA[3][4] " << setw(13) << setprecision(0) << fixed << AtA.at<double>(3,4) << endl;
    //cout << "AtA[4][4] " << setw(13) << setprecision(0) << fixed << AtA.at<double>(4,4) << endl;
    //cout << "AtA[5][4] " << setw(13) << setprecision(0) << fixed << AtA.at<double>(5,4) << endl;
    //cout << "AtA[0][5] " << setw(13) << setprecision(0) << fixed << AtA.at<double>(0,5) << endl;
    //cout << "AtA[1][5] " << setw(13) << setprecision(0) << fixed << AtA.at<double>(1,5) << endl;
    //cout << "AtA[2][5] " << setw(13) << setprecision(0) << fixed << AtA.at<double>(2,5) << endl;
    //cout << "AtA[3][5] " << setw(13) << setprecision(0) << fixed << AtA.at<double>(3,5) << endl;
    //cout << "AtA[4][5] " << setw(13) << setprecision(0) << fixed << AtA.at<double>(4,5) << endl;
    //cout << "AtA[5][5] " << setw(13) << setprecision(0) << fixed << AtA.at<double>(5,5) << endl;
    //cout << "AtB[0] " << setw(13) << setprecision(0) << fixed << AtB_ptr[0] << endl;
    //cout << "AtB[1] " << setw(13) << setprecision(0) << fixed << AtB_ptr[1] << endl;
    //cout << "AtB[2] " << setw(13) << setprecision(0) << fixed << AtB_ptr[2] << endl;
    //cout << "AtB[3] " << setw(13) << setprecision(0) << fixed << AtB_ptr[3] << endl;
    //cout << "AtB[4] " << setw(13) << setprecision(0) << fixed << AtB_ptr[4] << endl;
    //cout << "AtB[5] " << setw(13) << setprecision(0) << fixed << AtB_ptr[5] << endl;
    //exit(1);
}

static
bool solveSystem(const Mat& AtA, const Mat& AtB, double detThreshold, Mat& x)  //output ksi(x); determinantThreshold(detThreshold) = 1e-6
{
    //double det = determinant(AtA);

    //if(fabs (det) < detThreshold || cvIsNaN(det) || cvIsInf(det))
    //    return false;

    //solve(AtA, AtB, x, DECOMP_CHOLESKY);
    int rows = AtA.rows;    //rows = 6
    int cols = AtA.cols;    //cols = 6
    Mat A = AtA.clone();    //6*6
    Mat B = AtB.clone();    //6*1
    //cout << "AtA.10 = " << A.at<double>(1, 0) << endl;

    for(int k = 0; k < rows; k++)
    {
        for(int m = 0; m < k; m++) //dkk = akk - lkm * lkm * dmm  = akk - lkm * umk 
        {
            A.at<double>(k, k) = A.at<double>(k, k) - trunc6((A.at<double>(k, m) * A.at<double>(m, k)) / MUL);
        }
 
        for(int i = k+1; i < cols; i++)
        {
            for(int m = 0; m < k; m++) //uki = aki - lkm * umi
            {
                 A.at<double>(k, i) = A.at<double>(k, i) - trunc6((A.at<double>(m, i) * A.at<double>(k, m)) / MUL);
            }
            if(fabs(A.at<double>(k, k)) <= DBL_EPSILON) //fabs表FP的abs ; DBL_EPSILON表double的min正值
                return false;
         
            //lik = uki / dkk 
            A.at<double>(i, k) = trunc6(A.at<double>(k, i) * MUL / A.at<double>(k, k));
        }
    }

    //cout << "AtA " << AtA << endl;
    //cout << "A " << A << endl;
    //cout << "B " << B << endl;
    for(int i = 0; i < rows; i++)
    {
        for(int k = 0; k < i; k++)
        {
            B.at<double>(i, 0) = B.at<double>(i, 0) - trunc6((A.at<double>(i, k) * B.at<double>(k, 0)) / MUL);
        }
    }

    for(int i = rows-1; i >= 0; i--)
    {
        if(fabs(A.at<double>(i, i)) <= DBL_EPSILON)
            return false;
        B.at<double>(i, 0) = trunc6(B.at<double>(i, 0) * MUL / A.at<double>(i, i));
        for(int k = i+1; k < rows; k++)
        {
            B.at<double>(i, 0) = B.at<double>(i, 0) - trunc6((A.at<double>(k, i) * B.at<double>(k, 0)) / MUL);
        }
    }
    //cout << "B " << B << endl;
    //exit(1);

    x = B;
    //cout << "AtA " << AtA << endl;
    //cout << "AtB " << AtB << endl;
    //cout << "A " << A << endl;
    //cout << "B " << B << endl;
    //for(int i = 0; i < rows; i++)
    //{
    //        //x.at<double>(i, 0) = x.at<double>(i, 0) / MUL;
    //        if(isnan(x.at<double>(i, 0)))
    //            exit(1);
    //}
    //cout << "x " << x << endl;
    //cout << "A*x " << AtA*x << endl;
    //cout << "B " << AtB << endl;
    //exit(1);

    return true;
}

static
bool testDeltaTransformation(const Mat& deltaRt, double maxTranslation, double maxRotation)
{
    double translation = norm(deltaRt(Rect(3, 0, 1, 3)));

    Mat rvec;
    Rodrigues(deltaRt(Rect(0,0,3,3)), rvec);

    double rotation = norm(rvec) * 180. / CV_PI;

    return translation <= maxTranslation && rotation <= maxRotation;
}

static
bool computeProjectiveMatrix(const Mat& ksi, Mat& Rt, double maxTranslation, double maxRotation)
{
    CV_Assert(ksi.size() == Size(1,6) && ksi.type() == CV_64FC1);

#ifdef HAVE_EIGEN3_HERE  //HAVE_EIGEN3_HERE is not defined
    const double* ksi_ptr = ksi.ptr<const double>();
    Eigen::Matrix<double,4,4> twist, g;
    twist << 0.,          -ksi_ptr[2], ksi_ptr[1],  ksi_ptr[3],
             ksi_ptr[2],  0.,          -ksi_ptr[0], ksi_ptr[4],
             -ksi_ptr[1], ksi_ptr[0],  0,           ksi_ptr[5],
             0.,          0.,          0.,          0.;
    g = twist.exp();

    eigen2cv(g, Rt);
#else

    // TODO: check computeProjectiveMatrix when there is not eigen library,
    //       because it gives less accurate pose of the camera
    Rt = Mat::eye(4, 4, CV_64FC1);
    Rt = Rt * MUL;

    //Mat R = Rt(Rect(0,0,3,3));
    Mat rvec = ksi.rowRange(0,3);
    double translation = norm(ksi.rowRange(3,6));
    if(translation > trunc1(maxTranslation*MUL))
        return false;
    double rotation = norm(rvec) * 180. / CV_PI;
    if(rotation > trunc1(maxRotation*MUL))
        return false;
 

    //Rodrigues(rvec, R);

    Rt.at<double>(0,3) = ksi.at<double>(3);
    Rt.at<double>(1,3) = ksi.at<double>(4);
    Rt.at<double>(2,3) = ksi.at<double>(5);

    Mat rvec_fix = rvec.clone();
    Point3d r;
    r.x = rvec_fix.at<double>(0);
    r.y = rvec_fix.at<double>(1);
    r.z = rvec_fix.at<double>(2);
    double theta = trunc_theta(norm(r));

    //CORDIC
    //vector<double> atan_table;
    //for(int k = 0; k < 21; k++)
    //    //atan_table.push_back(atan(pow(2.0,(-1.0*k)))*180/M_PI); 
    //    atan_table.push_back(atan(pow(2.0,(-1.0*k)))); 
    //double An = sqrt(2.0);
    //for(int k = 1; k < 20; k++)
    //    An *= sqrt(1 + pow(2.0, (-2.0*k)));        
    //double x = 1.0/An;
    //double y = 0.0;
    //double z = theta;
    //double d;
    //for(int k = 0; k < 21; k++)
    //{
    //    if(z<=0)
    //        d = -1;
    //    else
    //        d = 1;
    //    double tmp_x = x - (y * d * pow(2.0, (-1.0*k)));
    //    double tmp_y = y + (x * d * pow(2.0, (-1.0*k)));
    //    x = tmp_x;
    //    y = tmp_y;
    //    z = z -(d * atan_table[k]);
    //}
    //double c = x;
    //double s = y;
    double c = trunc1(cos(theta/MUL) * MUL);
    double s = trunc1(sin(theta/MUL) * MUL);

    double c1 = 1.*MUL - c;
    //double c1 = 1. - c;
    //double itheta = theta ? 1./theta : 0.;
    
    //r *= itheta;
    r.x = trunc1(r.x * MUL / theta);
    r.y = trunc1(r.y * MUL / theta);
    r.z = trunc1(r.z * MUL / theta);
    
    //Matx33d rrt( r.x*r.x, r.x*r.y, r.x*r.z, r.x*r.y, r.y*r.y, r.y*r.z, r.x*r.z, r.y*r.z, r.z*r.z );
    Matx33d rrt( trunc1(r.x*r.x/MUL), trunc1(r.x*r.y/MUL), trunc1(r.x*r.z/MUL),
                 trunc1(r.x*r.y/MUL), trunc1(r.y*r.y/MUL), trunc1(r.y*r.z/MUL), 
                 trunc1(r.x*r.z/MUL), trunc1(r.y*r.z/MUL), trunc1(r.z*r.z/MUL) );
    Matx33d r_x(    0, -r.z,  r.y,
                  r.z,    0, -r.x,
                 -r.y,  r.x,    0 );
    // R = cos(theta)*I + (1 - cos(theta))*r*rT + sin(theta)*[r_x]
    //Matx33d R_fix = c*Matx33d::eye() + c1*rrt + s*r_x;
    //Rt.at<double>(0,0) = R_fix(0,0);
    //Rt.at<double>(1,0) = R_fix(1,0);
    //Rt.at<double>(2,0) = R_fix(2,0);
    //Rt.at<double>(0,1) = R_fix(0,1);
    //Rt.at<double>(1,1) = R_fix(1,1);
    //Rt.at<double>(2,1) = R_fix(2,1);
    //Rt.at<double>(0,2) = R_fix(0,2);
    //Rt.at<double>(1,2) = R_fix(1,2);
    //Rt.at<double>(2,2) = R_fix(2,2);
    Rt.at<double>(0,0) = c + trunc1(c1*rrt(0,0)/MUL) + trunc1(s*r_x(0,0)/MUL);
    Rt.at<double>(1,0) = trunc1(c1*rrt(1,0)/MUL) + trunc1(s*r_x(1,0)/MUL);
    Rt.at<double>(2,0) = trunc1(c1*rrt(2,0)/MUL) + trunc1(s*r_x(2,0)/MUL);
    Rt.at<double>(0,1) = trunc1(c1*rrt(0,1)/MUL) + trunc1(s*r_x(0,1)/MUL);
    Rt.at<double>(1,1) = c + trunc1(c1*rrt(1,1)/MUL) + trunc1(s*r_x(1,1)/MUL);
    Rt.at<double>(2,1) = trunc1(c1*rrt(2,1)/MUL) + trunc1(s*r_x(2,1)/MUL);
    Rt.at<double>(0,2) = trunc1(c1*rrt(0,2)/MUL) + trunc1(s*r_x(0,2)/MUL);
    Rt.at<double>(1,2) = trunc1(c1*rrt(1,2)/MUL) + trunc1(s*r_x(1,2)/MUL);
    Rt.at<double>(2,2) = c + trunc1(c1*rrt(2,2)/MUL) + trunc1(s*r_x(2,2)/MUL);
    //for(int i = 0; i < Rt.rows; i++)
    //{
    //    for(int j = 0; j < Rt.cols; j++)
    //    {
    //        Rt.at<double>(i, j) = Rt.at<double>(i, j) / MUL;
    //    }
    //}
    //cout << "R_fix: " << R_fix <<endl;
    //cout << "ksi: " << ksi <<endl;
    //cout << "Rt: " << Rt <<endl;
    //cout << "translation: " << translation <<endl;
    //cout << "theta: " << theta <<endl;
    //cout << "theta/MUL: " << theta/MUL <<endl;
    //cout << "cos: " << c <<endl;
    //cout << "sin: " << s <<endl;
    //cout << "1 - cos: " << c1 <<endl;
    //exit(1);
    return true;

#endif
}

static inline
void calcRgbdEquationCoeffs(double* C, double dIdx, double dIdy, const Point3d& p3d, double fx, double fy)
{
    double invz  = 1. / p3d.z,
           v0 = dIdx * fx * invz,
           v1 = dIdy * fy * invz,
           v2 = -(v0 * p3d.x + v1 * p3d.y) * invz;

    C[0] = -p3d.z * v1 + p3d.y * v2;
    C[1] =  p3d.z * v0 - p3d.x * v2;
    C[2] = -p3d.y * v0 + p3d.x * v1;
    C[3] = v0;
    C[4] = v1;
    C[5] = v2;
}

static inline
void calcICPEquationCoeffs(double* C, const Point3d& p0, const Vec3d& n1)
{
    C[0] = -p0.z * n1[1] + p0.y * n1[2];
    C[1] =  p0.z * n1[0] - p0.x * n1[2];
    C[2] = -p0.y * n1[0] + p0.x * n1[1];
    C[3] = n1[0];
    C[4] = n1[1];
    C[5] = n1[2];
}

static inline
void calcFeatureXEquationCoeffs(double* C, const Point3d& p3d, double fx)
{
    double invz  = 1. / p3d.z;

    C[0] = -(fx * p3d.x * p3d.y * invz * invz);
    C[1] = fx + fx * p3d.x * p3d.x * invz * invz;
    C[2] = -(fx * p3d.y * invz);
    C[3] = fx * invz;
    C[4] = 0;
    C[5] = -(fx * p3d.x * invz * invz);
}

static inline
void calcFeatureYEquationCoeffs(double* C, const Point3d& p3d, double fy)
{
    double invz  = 1. / p3d.z;

    C[0] = -fy - (fy * p3d.y * p3d.y * invz * invz);
    //C[1] = fy * p3d.x * p3d.x * invz * invz;
    C[1] = fy * p3d.x * p3d.y * invz * invz;
    C[2] = fy * p3d.x * invz;
    C[3] = 0;
    C[4] = fy * invz;
    C[5] = -(fy * p3d.y * invz * invz);
}

Size Odometry::prepareFrameCache(Ptr<OdometryFrame>& frame, int cacheType) const
{
    
    checkImage(frame->image);

    checkDepth(frame->depth, frame->image.size());
    
    checkMask(frame->mask, frame->image.size());

    depthTo3d(frame->depth, cameraMatrix, frame->cloud);

    normalsComputer(frame->cloud, frame->depth.rows, frame->depth.cols, frame->maskNormal, frame->normals);
    checkNormals(frame->normals, frame->depth.size());

    MaskGen(frame->mask, frame->depth, minDepth, maxDepth,
            frame->maskNormal, frame->maskDepth);

    Sobel(frame->image, frame->dI_dx, CV_16S, 1, 0, sobelSize);     //sobelSize = 3
    Sobel(frame->image, frame->dI_dy, CV_16S, 0, 1, sobelSize);     //sobelSize = 3

    // //↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ pylin
    // ofstream debug_image("./debug_image.txt"); 
    // ofstream debug_dI_dx("./debug_dI_dx.txt"); 
    // for (int y = 0; y < frame->image.rows; ++y)
    // {
    //     for (int x = 0; x < frame->image.cols; ++x)
    //     {
    //         debug_image << short(frame->image.at<uint8_t>(y,x)) << endl;
    //         debug_dI_dx << frame->dI_dx.at<short int>(y,x) << endl;
    //     }
    // }
    // //↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑ pylin

    /*void Sobel( 
        InputArray src, //表示输入的灰度图像
        OutputArray dst, //表示输出的梯度
        int depth, //輸出梯度的資料型態，需大於輸入圖像的資料型態
        int dx,   //對x向取梯度
        int dy,   //對y向取梯度
        int ksize = 3,  //Sobel算子的kernel大小, 須為1,3,5,7. 默認為3
        double scale = 1, 
        double delta = 0,
        int borderType = BORDER_DEFAULT );
    */
    std::vector<int> minGradientMagnitudes_vec = minGradientMagnitudes;
    TexturedMaskGen(frame->dI_dx, frame->dI_dy,
                    minGradientMagnitudes_vec[0], frame->maskDepth,
                    maxPointsPart, frame->maskText);

    return frame->image.size();
}

bool Odometry::compute(Ptr<OdometryFrame>& srcFrame, Ptr<OdometryFrame>& dstFrame, Mat& Rt, int& v_max, Mat& sigma_debug, const Mat& initRt) const
{
    Mat debug;  //pylin
    debug = Mat(1, 1, CV_64FC1, Scalar(1));    //pylin

    Size srcSize = prepareFrameCache(srcFrame, OdometryFrame::CACHE_SRC);   
    Size dstSize = prepareFrameCache(dstFrame, OdometryFrame::CACHE_DST);

    if(srcSize != dstSize)
        CV_Error(Error::StsBadSize, "srcFrame and dstFrame have to have the same size (resolution).");  //確認input正確

    int transformDim = 6;
    //set function ptr
    CalcRgbdEquationCoeffsPtr rgbdEquationFuncPtr = calcRgbdEquationCoeffs;
    CalcICPEquationCoeffsPtr icpEquationFuncPtr = calcICPEquationCoeffs;
    CalcFeatureXEquationCoeffsPtr featureXEquationFuncPtr = calcFeatureXEquationCoeffs;
    CalcFeatureYEquationCoeffsPtr featureYEquationFuncPtr = calcFeatureYEquationCoeffs;


    //initialize parameters
    std::vector<int> iterCounts_vec = iterCounts;

    const int minOverdetermScale = 20;
    const int minCorrespsCount = minOverdetermScale * transformDim;  // 20*6

    std::vector<Mat> pyramidCameraMatrix;

    Mat resultRt = initRt.empty() ? Mat::eye(4,4,CV_64FC1)*MUL : initRt.clone();
    Mat currRt, ksi;

    bool isOk = false;
    {
        const Mat& levelCameraMatrix = cameraMatrix;
        const Mat& levelCameraMatrix_inv = levelCameraMatrix.inv(DECOMP_SVD);
        const Mat& srcLevelDepth = srcFrame->depth;
        const Mat& dstLevelDepth = dstFrame->depth;

        const double fx = levelCameraMatrix.at<double>(0,0);
        const double fy = levelCameraMatrix.at<double>(1,1);
        const double cx = levelCameraMatrix.at<double>(0,2);
        const double cy = levelCameraMatrix.at<double>(1,2);
        const double determinantThreshold = 1e-6;

        Mat AtA_rgbd, AtB_rgbd, AtA_icp, AtB_icp;
        Mat corresps_rgbd, corresps_icp;

        // Run transformation search on current level iteratively.
        // for(int iter = 0; iter < iterCounts_vec[0]; iter ++)    //iterCounts_vec[0] = 8
        for(int iter = 0; iter < 3; iter ++)  //pylin test
        {
            Mat AtA(transformDim, transformDim, CV_64FC1, Scalar(0)), AtB(transformDim, 1, CV_64FC1, Scalar(0)); //transformDim = 6
            if(iter >= feature_iter_num){  //feature_iter_num = 2
                //Mat resultRt_inv = resultRt.inv(DECOMP_SVD);
                /*
                //int v_rgbd = computeCorresps(levelCameraMatrix, levelCameraMatrix_inv, 
                int v_rgbd = computeCorresps_hw(levelCameraMatrix, levelCameraMatrix_inv, 
                                             //resultRt_inv, srcLevelDepth, srcFrame->maskDepth, dstLevelDepth, dstFrame->maskText,
                                             //resultRt, dstLevelDepth, dstFrame->maskDepth, srcLevelDepth, srcFrame->maskText,
                                             resultRt, dstLevelDepth, dstFrame->maskDepth, srcLevelDepth, srcFrame->maskDepth,
                                             maxDepthDiff, corresps_rgbd);
                if (v_rgbd > v_max)
                    v_max = v_rgbd;
                //int v_icp = computeCorresps(levelCameraMatrix, levelCameraMatrix_inv, 
                int v_icp = computeCorresps_hw(levelCameraMatrix, levelCameraMatrix_inv, 
                                            //resultRt_inv, srcLevelDepth, srcFrame->maskDepth, dstLevelDepth, dstFrame->maskNormal,
                                            resultRt, dstLevelDepth, dstFrame->maskDepth, srcLevelDepth, srcFrame->maskDepth,
                                            maxDepthDiff, corresps_icp);
                
                if (v_icp > v_max)
                    v_max = v_icp;
                */
                int v_rgbd = computeCorresps_hw(levelCameraMatrix, 
                                                levelCameraMatrix_inv, 
                                                resultRt, 
                                                dstLevelDepth, 
                                                dstFrame->maskDepth, 
                                                srcLevelDepth, 
                                                srcFrame->maskDepth,
                                                maxDepthDiff, 
                                                corresps_rgbd, 
                                                dstFrame->normals,
                                                debug); //pylin debug
                corresps_icp = corresps_rgbd;
                if (v_rgbd > v_max)
                    v_max = v_rgbd;

               //cout << "corresps_rgbd " << corresps_rgbd.size() << endl;
               //cout << "corresps_icp " << corresps_icp.size() << endl;

                if(corresps_rgbd.rows >= minCorrespsCount)
                {
                    calcRgbdLsmMatrices(srcFrame->image, srcFrame->cloud, resultRt, //srcFrame->image is grey
                                        dstFrame->image, dstFrame->dI_dx, dstFrame->dI_dy,
                                        corresps_rgbd, fx, fy, sobelScale,
                                        AtA_rgbd, AtB_rgbd, rgbdEquationFuncPtr, transformDim);

                    //AtA += AtA_rgbd / MUL;
                    //AtB += AtB_rgbd / MUL;
                    AtA += AtA_rgbd;
                    AtB += AtB_rgbd;
                }

                if(corresps_icp.rows >= minCorrespsCount)
                {   
                    calcICPLsmMatrices(srcFrame->cloud, 
                                       resultRt,
                                       dstFrame->cloud, 
                                       dstFrame->normals,
                                       corresps_icp, 
                                       AtA_icp, AtB_icp, 
                                       icpEquationFuncPtr, 
                                       transformDim,
                                       iter,
                                       sigma_debug ); // pylin
                    //AtA += AtA_icp / MUL;
                    //AtB += AtB_icp / MUL;
                    AtA += AtA_icp;
                    AtB += AtB_icp;
                }
            }
            else 
            {
               ///////////////////////////////////////////////
               #if !USE_MYORB //USE_MYORB = 1  //# == Macro, 在compiler time判斷條件
               
               std::vector<KeyPoint> keypoints_1, keypoints_2;
               Mat descriptors_1, descriptors_2;
               Ptr<FeatureDetector> detector = ORB::create();
               //Ptr<FeatureDetector> detector = FastFeatureDetector::create();
               Ptr<DescriptorExtractor> descriptor = ORB::create();            

               detector->detect ( srcFrame->image ,keypoints_1 );
               detector->detect ( dstFrame->image ,keypoints_2 );
               descriptor->compute ( srcFrame->image, keypoints_1, descriptors_1 );
               descriptor->compute ( dstFrame->image, keypoints_2, descriptors_2 );

               Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );
               vector<DMatch> matches;
               matcher->match ( descriptors_1, descriptors_2, matches );

               double min_dist=10000, max_dist=0;
               for ( int i = 0; i < descriptors_1.rows; i++ )
               {
                   double dist = matches[i].distance;
                   if ( dist < min_dist ) min_dist = dist;
                   if ( dist > max_dist ) max_dist = dist;
               }

               std::vector< DMatch > good_matches;
               for ( int i = 0; i < descriptors_1.rows; i++ )
               {
                   if ( matches[i].distance <= max ( 2*min_dist, 30.0 ) )
                   {
                       if(srcFrame->maskDepth.at<uchar>(keypoints_1[matches[i].queryIdx].pt.y, keypoints_1[matches[i].queryIdx].pt.x)){
                           if(abs(keypoints_1[matches[i].queryIdx].pt.y - keypoints_2[matches[i].trainIdx].pt.y) <= maxLineDiff) 
                               good_matches.push_back ( matches[i] );
                       }
                   }
               }

               //BFMatcher matcher;
               //std::vector<vector< DMatch >> matches;
               //matcher.knnMatch(descriptors_1, descriptors_2, matches, 2);
               //
               //std::vector< DMatch > good_matches;
               //for (int i = 0; i < matches.size(); i ++) {
               //    double rejectRatio = 0.8;
               //    if (matches[i][0].distance / matches[i][1].distance > rejectRatio)
               //        continue;
               //    if(srcFrame->pyramidMask[level].at<uchar>(keypoints_1[matches[i][0].queryIdx].pt.y, keypoints_1[matches[i][0].queryIdx].pt.x))
               //        good_matches.push_back(matches[i][0]);
               //}

               Mat corresps_feature;
               corresps_feature.create(good_matches.size(), 1, CV_32SC4);
               Vec4i * corresps_feature_ptr = corresps_feature.ptr<Vec4i>();
               for(int idx = 0, i = 0; idx < good_matches.size(); idx++)
               {
                   //if(abs(keypoints_1[good_matches[idx].queryIdx].pt.y - keypoints_2[good_matches[idx].trainIdx].pt.y) <= maxLineDiff) 
                       corresps_feature_ptr[i++] = Vec4i(keypoints_1[good_matches[idx].queryIdx].pt.x, keypoints_1[good_matches[idx].queryIdx].pt.y, 
                                                         keypoints_2[good_matches[idx].trainIdx].pt.x, keypoints_2[good_matches[idx].trainIdx].pt.y);
               }
               //cout << "corresps " << corresps_feature << endl;
               //cout << "corresps " << corresps_feature.size() << endl;
               //exit(1);

               #else
               // ============ Use Frank's ORB to do feature matching ============
               ////#define FAST_N                          9
               ////#define FAST_threshold                  20
               ////#define FAST_orientation_patch_size     7
               ////#define FAST_scorethreshold             80
               ////#define FAST_edgethreshold              31
               ////#define keypoints_num                   500
               ////#define MATCH_threshold                 30
               ////#define DISPLAY                         false
               ////#define FIXED                           false
               ////#define TESTBENCH                       false
               ////// Image pyramid
               ////#define FAST_nlevels                    4
               ////#define FAST_scaling                    2
               ////
               ////// cout << srcFrame->image.rows << " " << srcFrame->image.cols << endl;
               ////MYORB orb(FAST_N, FAST_threshold, FAST_orientation_patch_size, FAST_scorethreshold, FAST_edgethreshold, keypoints_num, MATCH_threshold, FAST_nlevels, FAST_scaling, srcFrame->image, dstFrame->image, DISPLAY, FIXED, TESTBENCH);
               #define FAST_N                          9
               #define FAST_threshold                  20
               #define FAST_orientation_patch_size     7
               #define FAST_scorethreshold             80
               #define FAST_edgethreshold              31
               #define keypoints_num                   500
               #define MATCH_threshold                 30
               #define DISPLAY                         false
               #define FIXED                           true
               #define DEBUG                           false
               #define TESTBENCH                       false
               MYORB orb(FAST_N, FAST_threshold, FAST_orientation_patch_size, FAST_scorethreshold, FAST_edgethreshold, keypoints_num, MATCH_threshold, srcFrame->image, dstFrame->image, srcFrame->depth, dstFrame->depth, DISPLAY, FIXED, DEBUG, TESTBENCH);
               std::vector<DMatch> matches = orb.Matching();

               // Delete matches without depth information
               std::vector<DMatch> good_matches; //good_matches: 保留有depth的matches
               for ( int i = 0; i < matches.size(); i++ )
               {
                   KeyPoint k = orb.POINT_k1(matches[i].trainIdx);
                   KeyPoint k2 = orb.POINT_k2(matches[i].queryIdx);
                   if(srcFrame->maskDepth.at<uchar>(k.pt.y, k.pt.x))
                   {
                       if(abs(k.pt.y - k2.pt.y) <= maxLineDiff) 
                           good_matches.push_back(matches[i]);
                   }
               }
               //cout << good_matches.size() << endl;

               Mat corresps_feature;
               corresps_feature.create(good_matches.size(), 1, CV_32SC4);
               Vec4i * corresps_feature_ptr = corresps_feature.ptr<Vec4i>();
               for(int idx = 0, i = 0; idx < good_matches.size(); idx++)
               {
                   KeyPoint k1 = orb.POINT_k1(good_matches[i].trainIdx);
                   KeyPoint k2 = orb.POINT_k2(good_matches[i].queryIdx);
                   corresps_feature_ptr[i++] = Vec4i(k1.pt.x, k1.pt.y, k2.pt.x, k2.pt.y);  //後面沒用到
               }
               #endif
               // ================================================================

               //if(corresps_feature.rows >= minCorrespsCount)
               if(corresps_feature.rows >= feature_corr_num) //feature_corr_num = 12
               {
                   Mat AtA_feature, AtB_feature;
                   calcFeatureLsmMatrices(iter, srcFrame->cloud, resultRt,
                                         corresps_feature, fx, fy, cx, cy,
                                         AtA_feature, AtB_feature, featureXEquationFuncPtr, featureYEquationFuncPtr, transformDim);

                   AtA += AtA_feature;
                   AtB += AtB_feature;
               }
            }
            //cout << "iter " << iter << endl;
            //cout << "AtA " << AtA << endl;
            //cout << "AtB " << AtB << endl;
            //if(iter==1)
            //    exit(1);
            bool solutionExist = solveSystem(AtA, AtB, determinantThreshold, ksi);  //output ksi; determinantThreshold = 1e-6
            if(!solutionExist) //if no soulution
                break;

            bool testDelta = computeProjectiveMatrix(ksi, currRt, maxTranslation, maxRotation);
            if(!testDelta)  //if solution is useless
                break;
            //resultRt = currRt * resultRt;
            Mat currRt_cp = currRt.clone();
            //cout << "resultRt ori " << resultRt << endl;
            //cout << "currRt " << currRt << endl;

//GMP
//purpose: high precision calculation

            mpz_t curr_00; //mpz_t: 高精度的數字
            mpz_t curr_01;
            mpz_t curr_02;
            mpz_t curr_03;
            mpz_t curr_10;
            mpz_t curr_11;
            mpz_t curr_12;
            mpz_t curr_13;
            mpz_t curr_20;
            mpz_t curr_21;
            mpz_t curr_22;
            mpz_t curr_23;
            mpz_t curr_30;
            mpz_t curr_31;
            mpz_t curr_32;
            mpz_t curr_33;
            mpz_t result_00;
            mpz_t result_10;
            mpz_t result_20;
            mpz_t result_30;
            mpz_t result_01;
            mpz_t result_11;
            mpz_t result_21;
            mpz_t result_31;
            mpz_t result_02;
            mpz_t result_12;
            mpz_t result_22;
            mpz_t result_32;
            mpz_t result_03;
            mpz_t result_13;
            mpz_t result_23;
            mpz_t result_33;
            mpz_t result_temp;
            mpz_t result_final_00;
            mpz_t result_final_01;
            mpz_t result_final_02;
            mpz_t result_final_03;
            mpz_t result_final_10;
            mpz_t result_final_11;
            mpz_t result_final_12;
            mpz_t result_final_13;
            mpz_t result_final_20;
            mpz_t result_final_21;
            mpz_t result_final_22;
            mpz_t result_final_23;
            mpz_t result_final_30;
            mpz_t result_final_31;
            mpz_t result_final_32;
            mpz_t result_final_33;
            mpz_t MUL_gmp;
            mpz_init_set_d(MUL_gmp, MUL);   //mpz_init_set_d: 設定初始值
            mpz_init_set_d(curr_00, currRt_cp.at<double>(0,0));
            mpz_init_set_d(curr_01, currRt_cp.at<double>(0,1));
            mpz_init_set_d(curr_02, currRt_cp.at<double>(0,2));
            mpz_init_set_d(curr_03, currRt_cp.at<double>(0,3));
            mpz_init_set_d(curr_10, currRt_cp.at<double>(1,0));
            mpz_init_set_d(curr_11, currRt_cp.at<double>(1,1));
            mpz_init_set_d(curr_12, currRt_cp.at<double>(1,2));
            mpz_init_set_d(curr_13, currRt_cp.at<double>(1,3));
            mpz_init_set_d(curr_20, currRt_cp.at<double>(2,0));
            mpz_init_set_d(curr_21, currRt_cp.at<double>(2,1));
            mpz_init_set_d(curr_22, currRt_cp.at<double>(2,2));
            mpz_init_set_d(curr_23, currRt_cp.at<double>(2,3));
            mpz_init_set_d(curr_30, currRt_cp.at<double>(3,0));
            mpz_init_set_d(curr_31, currRt_cp.at<double>(3,1));
            mpz_init_set_d(curr_32, currRt_cp.at<double>(3,2));
            mpz_init_set_d(curr_33, currRt_cp.at<double>(3,3));
            mpz_init_set_d(result_00, resultRt.at<double>(0,0));
            mpz_init_set_d(result_10, resultRt.at<double>(1,0));
            mpz_init_set_d(result_20, resultRt.at<double>(2,0));
            mpz_init_set_d(result_30, resultRt.at<double>(3,0));
            mpz_init_set_d(result_01, resultRt.at<double>(0,1));
            mpz_init_set_d(result_11, resultRt.at<double>(1,1));
            mpz_init_set_d(result_21, resultRt.at<double>(2,1));
            mpz_init_set_d(result_31, resultRt.at<double>(3,1));
            mpz_init_set_d(result_02, resultRt.at<double>(0,2));
            mpz_init_set_d(result_12, resultRt.at<double>(1,2));
            mpz_init_set_d(result_22, resultRt.at<double>(2,2));
            mpz_init_set_d(result_32, resultRt.at<double>(3,2));
            mpz_init_set_d(result_03, resultRt.at<double>(0,3));
            mpz_init_set_d(result_13, resultRt.at<double>(1,3));
            mpz_init_set_d(result_23, resultRt.at<double>(2,3));
            mpz_init_set_d(result_33, resultRt.at<double>(3,3));
            mpz_mul(result_temp,curr_00,result_00);
            mpz_div(result_final_00,result_temp,MUL_gmp);
            mpz_mul(result_temp,curr_01,result_10);
            mpz_div(result_temp,result_temp,MUL_gmp);
            mpz_add(result_final_00,result_final_00,result_temp);
            mpz_mul(result_temp,curr_02,result_20);
            mpz_div(result_temp,result_temp,MUL_gmp);
            mpz_add(result_final_00,result_final_00,result_temp);
            mpz_mul(result_temp,curr_03,result_30);
            mpz_div(result_temp,result_temp,MUL_gmp);
            mpz_add(result_final_00,result_final_00,result_temp);
            resultRt.at<double>(0,0) = mpz_get_d(result_final_00);
            mpz_mul(result_temp,curr_00,result_01);
            mpz_div(result_final_01,result_temp,MUL_gmp);
            mpz_mul(result_temp,curr_01,result_11);
            mpz_div(result_temp,result_temp,MUL_gmp);
            mpz_add(result_final_01,result_final_01,result_temp);
            mpz_mul(result_temp,curr_02,result_21);
            mpz_div(result_temp,result_temp,MUL_gmp);
            mpz_add(result_final_01,result_final_01,result_temp);
            mpz_mul(result_temp,curr_03,result_31);
            mpz_div(result_temp,result_temp,MUL_gmp);
            mpz_add(result_final_01,result_final_01,result_temp);
            resultRt.at<double>(0,1) = mpz_get_d(result_final_01);
            mpz_mul(result_temp,curr_00,result_02);
            mpz_div(result_final_02,result_temp,MUL_gmp);
            mpz_mul(result_temp,curr_01,result_12);
            mpz_div(result_temp,result_temp,MUL_gmp);
            mpz_add(result_final_02,result_final_02,result_temp);
            mpz_mul(result_temp,curr_02,result_22);
            mpz_div(result_temp,result_temp,MUL_gmp);
            mpz_add(result_final_02,result_final_02,result_temp);
            mpz_mul(result_temp,curr_03,result_32);
            mpz_div(result_temp,result_temp,MUL_gmp);
            mpz_add(result_final_02,result_final_02,result_temp);
            resultRt.at<double>(0,2) = mpz_get_d(result_final_02);
            mpz_mul(result_temp,curr_00,result_03);
            mpz_div(result_final_03,result_temp,MUL_gmp);
            mpz_mul(result_temp,curr_01,result_13);
            mpz_div(result_temp,result_temp,MUL_gmp);
            mpz_add(result_final_03,result_final_03,result_temp);
            mpz_mul(result_temp,curr_02,result_23);
            mpz_div(result_temp,result_temp,MUL_gmp);
            mpz_add(result_final_03,result_final_03,result_temp);
            mpz_mul(result_temp,curr_03,result_33);
            mpz_div(result_temp,result_temp,MUL_gmp);
            mpz_add(result_final_03,result_final_03,result_temp);
            resultRt.at<double>(0,3) = mpz_get_d(result_final_03);
            mpz_mul(result_temp,curr_10,result_00);
            mpz_div(result_final_10,result_temp,MUL_gmp);
            mpz_mul(result_temp,curr_11,result_10);
            mpz_div(result_temp,result_temp,MUL_gmp);
            mpz_add(result_final_10,result_final_10,result_temp);
            mpz_mul(result_temp,curr_12,result_20);
            mpz_div(result_temp,result_temp,MUL_gmp);
            mpz_add(result_final_10,result_final_10,result_temp);
            mpz_mul(result_temp,curr_13,result_30);
            mpz_div(result_temp,result_temp,MUL_gmp);
            mpz_add(result_final_10,result_final_10,result_temp);
            resultRt.at<double>(1,0) = mpz_get_d(result_final_10);
            mpz_mul(result_temp,curr_10,result_01);
            mpz_div(result_final_11,result_temp,MUL_gmp);
            mpz_mul(result_temp,curr_11,result_11);
            mpz_div(result_temp,result_temp,MUL_gmp);
            mpz_add(result_final_11,result_final_11,result_temp);
            mpz_mul(result_temp,curr_12,result_21);
            mpz_div(result_temp,result_temp,MUL_gmp);
            mpz_add(result_final_11,result_final_11,result_temp);
            mpz_mul(result_temp,curr_13,result_31);
            mpz_div(result_temp,result_temp,MUL_gmp);
            mpz_add(result_final_11,result_final_11,result_temp);
            resultRt.at<double>(1,1) = mpz_get_d(result_final_11);
            mpz_mul(result_temp,curr_10,result_02);
            mpz_div(result_final_12,result_temp,MUL_gmp);
            mpz_mul(result_temp,curr_11,result_12);
            mpz_div(result_temp,result_temp,MUL_gmp);
            mpz_add(result_final_12,result_final_12,result_temp);
            mpz_mul(result_temp,curr_12,result_22);
            mpz_div(result_temp,result_temp,MUL_gmp);
            mpz_add(result_final_12,result_final_12,result_temp);
            mpz_mul(result_temp,curr_13,result_32);
            mpz_div(result_temp,result_temp,MUL_gmp);
            mpz_add(result_final_12,result_final_12,result_temp);
            resultRt.at<double>(1,2) = mpz_get_d(result_final_12);
            mpz_mul(result_temp,curr_10,result_03);
            mpz_div(result_final_13,result_temp,MUL_gmp);
            mpz_mul(result_temp,curr_11,result_13);
            mpz_div(result_temp,result_temp,MUL_gmp);
            mpz_add(result_final_13,result_final_13,result_temp);
            mpz_mul(result_temp,curr_12,result_23);
            mpz_div(result_temp,result_temp,MUL_gmp);
            mpz_add(result_final_13,result_final_13,result_temp);
            mpz_mul(result_temp,curr_13,result_33);
            mpz_div(result_temp,result_temp,MUL_gmp);
            mpz_add(result_final_13,result_final_13,result_temp);
            resultRt.at<double>(1,3) = mpz_get_d(result_final_13);
            mpz_mul(result_temp,curr_20,result_00);
            mpz_div(result_final_20,result_temp,MUL_gmp);
            mpz_mul(result_temp,curr_21,result_10);
            mpz_div(result_temp,result_temp,MUL_gmp);
            mpz_add(result_final_20,result_final_20,result_temp);
            mpz_mul(result_temp,curr_22,result_20);
            mpz_div(result_temp,result_temp,MUL_gmp);
            mpz_add(result_final_20,result_final_20,result_temp);
            mpz_mul(result_temp,curr_23,result_30);
            mpz_div(result_temp,result_temp,MUL_gmp);
            mpz_add(result_final_20,result_final_20,result_temp);
            resultRt.at<double>(2,0) = mpz_get_d(result_final_20);
            mpz_mul(result_temp,curr_20,result_01);
            mpz_div(result_final_21,result_temp,MUL_gmp);
            mpz_mul(result_temp,curr_21,result_11);
            mpz_div(result_temp,result_temp,MUL_gmp);
            mpz_add(result_final_21,result_final_21,result_temp);
            mpz_mul(result_temp,curr_22,result_21);
            mpz_div(result_temp,result_temp,MUL_gmp);
            mpz_add(result_final_21,result_final_21,result_temp);
            mpz_mul(result_temp,curr_23,result_31);
            mpz_div(result_temp,result_temp,MUL_gmp);
            mpz_add(result_final_21,result_final_21,result_temp);
            resultRt.at<double>(2,1) = mpz_get_d(result_final_21);
            mpz_mul(result_temp,curr_20,result_02);
            mpz_div(result_final_22,result_temp,MUL_gmp);
            mpz_mul(result_temp,curr_21,result_12);
            mpz_div(result_temp,result_temp,MUL_gmp);
            mpz_add(result_final_22,result_final_22,result_temp);
            mpz_mul(result_temp,curr_22,result_22);
            mpz_div(result_temp,result_temp,MUL_gmp);
            mpz_add(result_final_22,result_final_22,result_temp);
            mpz_mul(result_temp,curr_23,result_32);
            mpz_div(result_temp,result_temp,MUL_gmp);
            mpz_add(result_final_22,result_final_22,result_temp);
            resultRt.at<double>(2,2) = mpz_get_d(result_final_22);
            mpz_mul(result_temp,curr_20,result_03);
            mpz_div(result_final_23,result_temp,MUL_gmp);
            mpz_mul(result_temp,curr_21,result_13);
            mpz_div(result_temp,result_temp,MUL_gmp);
            mpz_add(result_final_23,result_final_23,result_temp);
            mpz_mul(result_temp,curr_22,result_23);
            mpz_div(result_temp,result_temp,MUL_gmp);
            mpz_add(result_final_23,result_final_23,result_temp);
            mpz_mul(result_temp,curr_23,result_33);
            mpz_div(result_temp,result_temp,MUL_gmp);
            mpz_add(result_final_23,result_final_23,result_temp);
            resultRt.at<double>(2,3) = mpz_get_d(result_final_23);
            //mpz_clear(curr_00);
            //mpz_clear(curr_01);
            //mpz_clear(curr_02);
            //mpz_clear(curr_03);
            //mpz_clear(curr_10);
            //mpz_clear(curr_11);
            //mpz_clear(curr_12);
            //mpz_clear(curr_13);
            //mpz_clear(curr_20);
            //mpz_clear(curr_21);
            //mpz_clear(curr_22);
            //mpz_clear(curr_23);
            //mpz_clear(curr_30);
            //mpz_clear(curr_31);
            //mpz_clear(curr_32);
            //mpz_clear(curr_33);
            //mpz_clear(result_00);
            //mpz_clear(result_10);
            //mpz_clear(result_20);
            //mpz_clear(result_30);
            //mpz_clear(result_01);
            //mpz_clear(result_11);
            //mpz_clear(result_21);
            //mpz_clear(result_31);
            //mpz_clear(result_02);
            //mpz_clear(result_12);
            //mpz_clear(result_22);
            //mpz_clear(result_32);
            //mpz_clear(result_03);
            //mpz_clear(result_13);
            //mpz_clear(result_23);
            //mpz_clear(result_33);
            //mpz_clear(result_temp);
            //mpz_clear(result_final_00);
            //mpz_clear(result_final_01);
            //mpz_clear(result_final_02);
            //mpz_clear(result_final_03);
            //mpz_clear(result_final_10);
            //mpz_clear(result_final_11);
            //mpz_clear(result_final_12);
            //mpz_clear(result_final_13);
            //mpz_clear(result_final_20);
            //mpz_clear(result_final_21);
            //mpz_clear(result_final_22);
            //mpz_clear(result_final_23);
            //mpz_clear(result_final_30);
            //mpz_clear(result_final_31);
            //mpz_clear(result_final_32);
            //mpz_clear(result_final_33);
            //mpz_clear(MUL_gmp);
            ////mpz_mul(result_temp,curr_30,result_00);
            ////mpz_div(result_final_30,result_temp,MUL_gmp);
            ////mpz_mul(result_temp,curr_31,result_10);
            ////mpz_div(result_temp,result_temp,MUL_gmp);
            ////mpz_add(result_final_30,result_final_30,result_temp);
            ////mpz_mul(result_temp,curr_32,result_20);
            ////mpz_div(result_temp,result_temp,MUL_gmp);
            ////mpz_add(result_final_30,result_final_30,result_temp);
            ////mpz_mul(result_temp,curr_33,result_30);
            ////mpz_div(result_temp,result_temp,MUL_gmp);
            ////mpz_add(result_final_30,result_final_30,result_temp);
            ////resultRt.at<double>(3,0) = mpz_get_d(result_final_30);
            ////cout << "test " << endl;
            ////mpz_mul(result_temp,curr_30,result_01);
            ////mpz_div(result_final_31,result_temp,MUL_gmp);
            ////mpz_mul(result_temp,curr_31,result_11);
            ////mpz_div(result_temp,result_temp,MUL_gmp);
            ////mpz_add(result_final_31,result_final_31,result_temp);
            ////mpz_mul(result_temp,curr_32,result_21);
            ////mpz_div(result_temp,result_temp,MUL_gmp);
            ////mpz_add(result_final_31,result_final_31,result_temp);
            ////mpz_mul(result_temp,curr_33,result_31);
            ////mpz_div(result_temp,result_temp,MUL_gmp);
            ////mpz_add(result_final_31,result_final_31,result_temp);
            ////resultRt.at<double>(3,1) = mpz_get_d(result_final_31);
            ////cout << "test1 " << endl;
            ////mpz_mul(result_temp,curr_30,result_02);
            ////mpz_div(result_final_32,result_temp,MUL_gmp);
            ////mpz_mul(result_temp,curr_31,result_12);
            ////mpz_div(result_temp,result_temp,MUL_gmp);
            ////mpz_add(result_final_32,result_final_32,result_temp);
            ////mpz_mul(result_temp,curr_32,result_22);
            ////mpz_div(result_temp,result_temp,MUL_gmp);
            ////mpz_add(result_final_32,result_final_32,result_temp);
            ////mpz_mul(result_temp,curr_33,result_32);
            ////mpz_div(result_temp,result_temp,MUL_gmp);
            ////mpz_add(result_final_32,result_final_32,result_temp);
            ////resultRt.at<double>(3,2) = mpz_get_d(result_final_32);
            ////cout << "test1 " << endl;
            ////mpz_mul(result_temp,curr_30,result_03);
            ////cout << "test1 " << endl;
            ////mpz_div(result_final_33,result_temp,MUL_gmp);
            ////cout << "test1 " << endl;
            ////mpz_mul(result_temp,curr_31,result_13);
            ////cout << "test1 " << endl;
            ////mpz_div(result_temp,result_temp,MUL_gmp);
            ////cout << "test1 " << endl;
            ////mpz_add(result_final_33,result_final_33,result_temp);
            ////cout << "test1 " << endl;
            ////mpz_mul(result_temp,curr_32,result_23);
            ////mpz_div(result_temp,result_temp,MUL_gmp);
            ////mpz_add(result_final_33,result_final_33,result_temp);
            ////mpz_mul(result_temp,curr_33,result_33);
            ////cout << "test1 " << endl;
            ////mpz_div(result_temp,result_temp,MUL_gmp);
            ////mpz_add(result_final_33,result_final_33,result_temp);
            ////resultRt.at<double>(3,3) = mpz_get_d(result_final_33);
            ////cout << "test " << endl;
            //resultRt.at<double>(0,0)=trunc1(currRt.at<double>(0,0)*resultRt.at<double>(0,0)/MUL)+trunc1(currRt.at<double>(0,1)*resultRt.at<double>(1,0)/MUL)+
            //                         trunc1(currRt.at<double>(0,2)*resultRt.at<double>(2,0)/MUL)+trunc1(currRt.at<double>(0,3)*resultRt.at<double>(3,0)/MUL); 
            //resultRt.at<double>(0,1)=trunc1(currRt.at<double>(0,0)*resultRt.at<double>(0,1)/MUL)+trunc1(currRt.at<double>(0,1)*resultRt.at<double>(1,1)/MUL)+
            //                         trunc1(currRt.at<double>(0,2)*resultRt.at<double>(2,1)/MUL)+trunc1(currRt.at<double>(0,3)*resultRt.at<double>(3,1)/MUL); 
            //resultRt.at<double>(0,2)=trunc1(currRt.at<double>(0,0)*resultRt.at<double>(0,2)/MUL)+trunc1(currRt.at<double>(0,1)*resultRt.at<double>(1,2)/MUL)+
            //                         trunc1(currRt.at<double>(0,2)*resultRt.at<double>(2,2)/MUL)+trunc1(currRt.at<double>(0,3)*resultRt.at<double>(3,2)/MUL); 
            //resultRt.at<double>(0,3)=trunc1(currRt.at<double>(0,0)*resultRt.at<double>(0,3)/MUL)+trunc1(currRt.at<double>(0,1)*resultRt.at<double>(1,3)/MUL)+
            //                         trunc1(currRt.at<double>(0,2)*resultRt.at<double>(2,3)/MUL)+trunc1(currRt.at<double>(0,3)*resultRt.at<double>(3,3)/MUL); 
            //resultRt.at<double>(1,0)=trunc1(currRt.at<double>(1,0)*resultRt.at<double>(0,0)/MUL)+trunc1(currRt.at<double>(1,1)*resultRt.at<double>(1,0)/MUL)+
            //                         trunc1(currRt.at<double>(1,2)*resultRt.at<double>(2,0)/MUL)+trunc1(currRt.at<double>(1,3)*resultRt.at<double>(3,0)/MUL); 
            //resultRt.at<double>(1,1)=trunc1(currRt.at<double>(1,0)*resultRt.at<double>(0,1)/MUL)+trunc1(currRt.at<double>(1,1)*resultRt.at<double>(1,1)/MUL)+
            //                         trunc1(currRt.at<double>(1,2)*resultRt.at<double>(2,1)/MUL)+trunc1(currRt.at<double>(1,3)*resultRt.at<double>(3,1)/MUL); 
            //resultRt.at<double>(1,2)=trunc1(currRt.at<double>(1,0)*resultRt.at<double>(0,2)/MUL)+trunc1(currRt.at<double>(1,1)*resultRt.at<double>(1,2)/MUL)+
            //                         trunc1(currRt.at<double>(1,2)*resultRt.at<double>(2,2)/MUL)+trunc1(currRt.at<double>(1,3)*resultRt.at<double>(3,2)/MUL); 
            //resultRt.at<double>(1,3)=trunc1(currRt.at<double>(1,0)*resultRt.at<double>(0,3)/MUL)+trunc1(currRt.at<double>(1,1)*resultRt.at<double>(1,3)/MUL)+
            //                         trunc1(currRt.at<double>(1,2)*resultRt.at<double>(2,3)/MUL)+trunc1(currRt.at<double>(1,3)*resultRt.at<double>(3,3)/MUL); 
            //resultRt.at<double>(2,0)=trunc1(currRt.at<double>(2,0)*resultRt.at<double>(0,0)/MUL)+trunc1(currRt.at<double>(2,1)*resultRt.at<double>(1,0)/MUL)+
            //                         trunc1(currRt.at<double>(2,2)*resultRt.at<double>(2,0)/MUL)+trunc1(currRt.at<double>(2,3)*resultRt.at<double>(3,0)/MUL); 
            //resultRt.at<double>(2,1)=trunc1(currRt.at<double>(2,0)*resultRt.at<double>(0,1)/MUL)+trunc1(currRt.at<double>(2,1)*resultRt.at<double>(1,1)/MUL)+
            //                         trunc1(currRt.at<double>(2,2)*resultRt.at<double>(2,1)/MUL)+trunc1(currRt.at<double>(2,3)*resultRt.at<double>(3,1)/MUL); 
            //resultRt.at<double>(2,2)=trunc1(currRt.at<double>(2,0)*resultRt.at<double>(0,2)/MUL)+trunc1(currRt.at<double>(2,1)*resultRt.at<double>(1,2)/MUL)+
            //                         trunc1(currRt.at<double>(2,2)*resultRt.at<double>(2,2)/MUL)+trunc1(currRt.at<double>(2,3)*resultRt.at<double>(3,2)/MUL); 
            //resultRt.at<double>(2,3)=trunc1(currRt.at<double>(2,0)*resultRt.at<double>(0,3)/MUL)+trunc1(currRt.at<double>(2,1)*resultRt.at<double>(1,3)/MUL)+
            //                         trunc1(currRt.at<double>(2,2)*resultRt.at<double>(2,3)/MUL)+trunc1(currRt.at<double>(2,3)*resultRt.at<double>(3,3)/MUL); 
            //resultRt.at<double>(3,0)=trunc1(currRt.at<double>(3,0)*resultRt.at<double>(0,0)/MUL)+trunc1(currRt.at<double>(3,1)*resultRt.at<double>(1,0)/MUL)+
            //                         trunc1(currRt.at<double>(3,2)*resultRt.at<double>(2,0)/MUL)+trunc1(currRt.at<double>(3,3)*resultRt.at<double>(3,0)/MUL); 
            //resultRt.at<double>(3,1)=trunc1(currRt.at<double>(3,0)*resultRt.at<double>(0,1)/MUL)+trunc1(currRt.at<double>(3,1)*resultRt.at<double>(1,1)/MUL)+
            //                         trunc1(currRt.at<double>(3,2)*resultRt.at<double>(2,1)/MUL)+trunc1(currRt.at<double>(3,3)*resultRt.at<double>(3,1)/MUL); 
            //resultRt.at<double>(3,2)=trunc1(currRt.at<double>(3,0)*resultRt.at<double>(0,2)/MUL)+trunc1(currRt.at<double>(3,1)*resultRt.at<double>(1,2)/MUL)+
            //                         trunc1(currRt.at<double>(3,2)*resultRt.at<double>(2,2)/MUL)+trunc1(currRt.at<double>(3,3)*resultRt.at<double>(3,2)/MUL); 
            //resultRt.at<double>(3,3)=trunc1(currRt.at<double>(3,0)*resultRt.at<double>(0,3)/MUL)+trunc1(currRt.at<double>(3,1)*resultRt.at<double>(1,3)/MUL)+
            //                         trunc1(currRt.at<double>(3,2)*resultRt.at<double>(2,3)/MUL)+trunc1(currRt.at<double>(3,3)*resultRt.at<double>(3,3)/MUL); 
            // cout << "resultRt " << resultRt << endl;
            //if(iter==1)
            //  exit(1);
            isOk = true;    //在回圈內
        }
        //exit(1);
    }

    //cout << "v_max" << v_max << endl;
    Rt = resultRt;

    //應可用Mat的method
    for(int i = 0; i < Rt.rows; i++)
    {
        for(int j = 0; j < Rt.cols; j++)
        {
            Rt.at<double>(i, j) = Rt.at<double>(i, j) / MUL;
        }
    }

    //if(isOk)
    //{
    //    //Mat deltaRt;
    //    //if(initRt.empty())
    //    //    deltaRt = resultRt;
    //    //else
    //    //    deltaRt = resultRt * initRt.inv(DECOMP_SVD);

    //    //isOk = testDeltaTransformation(deltaRt, maxTranslation, maxRotation);
    //    isOk = testDeltaTransformation(resultRt, maxTranslation, maxRotation);
    //}

    return isOk;
}


