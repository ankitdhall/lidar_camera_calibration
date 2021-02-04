#pragma GCC diagnostic warning "-Wuninitialized"
/*****************************
Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
********************************/
#include <aruco/boarddetector.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <cstdlib>
#include <ctime>
#include <cassert>
#include <fstream>
using namespace std;
using namespace cv;
namespace aruco
{

  BoardDetector::BoardDetector(bool setYPerpendicular)
    : _setYPerpendicular(setYPerpendicular),
      _areParamsSet(false),
      _markerSize(0)
  {}
  /**
   * Use if you plan to let this class to perform marker detection too
   */
  void BoardDetector::setParams(const BoardConfiguration &bc,const CameraParameters &cp, float markerSizeMeters)
  {
    _camParams = cp;
    _markerSize = markerSizeMeters;
    _bconf = bc;
    _areParamsSet = true;
  }

  void BoardDetector::setParams(const BoardConfiguration &bc)
  {
    _bconf = bc;
    _areParamsSet = true;
  }


  float  BoardDetector::detect(const cv::Mat &im)throw (cv::Exception)
  {
    _mdetector.detect(im,_vmarkers);

    float res;

    if (_camParams.isValid())
      res=detect(_vmarkers,_bconf,_boardDetected,_camParams.CameraMatrix,_camParams.Distorsion,_markerSize);
    else res=detect(_vmarkers,_bconf,_boardDetected);
    return res;
  }

  float BoardDetector::detect ( const vector<Marker> &detectedMarkers,const  BoardConfiguration &BConf, Board &Bdetected,const CameraParameters &cp, float markerSizeMeters ) throw ( cv::Exception )
  {
    return detect ( detectedMarkers, BConf,Bdetected,cp.CameraMatrix,cp.Distorsion,markerSizeMeters );
  }

  float BoardDetector::detect ( const vector<Marker> &detectedMarkers,const  BoardConfiguration &BConf, Board &Bdetected, Mat camMatrix,Mat distCoeff,float markerSizeMeters ) throw ( cv::Exception )
  {
    if (BConf.size()==0) throw cv::Exception(8881,"BoardDetector::detect","Invalid BoardConfig that is empty",__FILE__,__LINE__);
    if (BConf[0].size()<2) throw cv::Exception(8881,"BoardDetector::detect","Invalid BoardConfig that is empty 2",__FILE__,__LINE__);
    //compute the size of the markers in meters, which is used for some routines(mostly drawing)
    float ssize;
    if ( BConf.mInfoType==BoardConfiguration::PIX && markerSizeMeters>0 ) ssize=markerSizeMeters;
    else if ( BConf.mInfoType==BoardConfiguration::METERS )
    {
      ssize=cv::norm ( BConf[0][0]-BConf[0][1] );
    }

    // cout<<"markerSizeMeters="<<markerSizeMeters<<endl;
    Bdetected.clear();
    ///find among detected markers these that belong to the board configuration
    for ( unsigned int i=0;i<detectedMarkers.size();i++ )
    {
      int idx=BConf.getIndexOfMarkerId(detectedMarkers[i].id);
      if (idx!=-1) {
        Bdetected.push_back ( detectedMarkers[i] );
        Bdetected.back().ssize=ssize;
      }
    }
    //copy configuration
    Bdetected.conf=BConf;
    //

    bool hasEnoughInfoForRTvecCalculation=false;
    if ( Bdetected.size() >=1 )
    {
      if ( camMatrix.rows!=0 )
      {
        if ( markerSizeMeters>0 && BConf.mInfoType==BoardConfiguration::PIX ) hasEnoughInfoForRTvecCalculation=true;
        else if ( BConf.mInfoType==BoardConfiguration::METERS ) hasEnoughInfoForRTvecCalculation=true;
      }
    }

    //calculate extrinsic if there is information for that
    if ( hasEnoughInfoForRTvecCalculation )
    {
      //calculate the size of the markers in meters if expressed in pixels
      double marker_meter_per_pix=0;
      if ( BConf.mInfoType==BoardConfiguration::PIX ) marker_meter_per_pix=markerSizeMeters /  cv::norm ( BConf[0][0]-BConf[0][1] );
      else marker_meter_per_pix=1;//to avoind interferring the process below

      // now, create the matrices for finding the extrinsics
      Mat objPoints ( 4*Bdetected.size(),3,CV_32FC1 );
      Mat imagePoints ( 4*Bdetected.size(),2,CV_32FC1 );
      //size in meters of inter-marker distance

      for ( size_t i=0;i<Bdetected.size();i++ )
      {
        int idx=Bdetected.conf.getIndexOfMarkerId(Bdetected[i].id);
        assert(idx!=-1);
        for ( int p=0;p<4;p++ )
        {
          imagePoints.at<float> ( ( i*4 ) +p,0 ) =Bdetected[i][p].x;
          imagePoints.at<float> ( ( i*4 ) +p,1 ) =Bdetected[i][p].y;
          const aruco::MarkerInfo &Minfo=Bdetected.conf.getMarkerInfo( Bdetected[i].id);

          objPoints.at<float> ( ( i*4 ) +p,0 ) = Minfo[p].x*marker_meter_per_pix;
          objPoints.at<float> ( ( i*4 ) +p,1 ) = Minfo[p].y*marker_meter_per_pix;
          objPoints.at<float> ( ( i*4 ) +p,2 ) = Minfo[p].z*marker_meter_per_pix;
          // 		cout<<objPoints.at<float>( (i*4)+p,0)<<" "<<objPoints.at<float>( (i*4)+p,1)<<" "<<objPoints.at<float>( (i*4)+p,2)<<endl;
        }
      }
      if (distCoeff.total()==0) distCoeff=cv::Mat::zeros(1,4,CV_32FC1 );

      cv::Mat rvec,tvec;
      cv::solvePnP(objPoints,imagePoints,camMatrix,distCoeff,rvec,tvec );
      rvec.convertTo(Bdetected.Rvec,CV_32FC1);
      tvec.convertTo(Bdetected.Tvec,CV_32FC1);
      //now, rotate 90 deg in X so that Y axis points up
      if (_setYPerpendicular)
        rotateXAxis ( Bdetected.Rvec );
      //         cout<<Bdetected.Rvec.at<float>(0,0)<<" "<<Bdetected.Rvec.at<float>(1,0)<<" "<<Bdetected.Rvec.at<float>(2,0)<<endl;
      //         cout<<Bdetected.Tvec.at<float>(0,0)<<" "<<Bdetected.Tvec.at<float>(1,0)<<" "<<Bdetected.Tvec.at<float>(2,0)<<endl;
    }

    float prob=float( Bdetected.size() ) /double ( Bdetected.conf.size() );
    return prob;
  }

  void BoardDetector::rotateXAxis ( Mat &rotation )
  {
    cv::Mat R ( 3,3,CV_32FC1 );
    Rodrigues ( rotation, R );
    //create a rotation matrix for x axis
    cv::Mat RX=cv::Mat::eye ( 3,3,CV_32FC1 );
    float angleRad=-M_PI/2;
    RX.at<float> ( 1,1 ) =cos ( angleRad );
    RX.at<float> ( 1,2 ) =-sin ( angleRad );
    RX.at<float> ( 2,1 ) =sin ( angleRad );
    RX.at<float> ( 2,2 ) =cos ( angleRad );
    //now multiply
    R=R*RX;
    //finally, the the rodrigues back
    Rodrigues ( R,rotation );

  }

}

