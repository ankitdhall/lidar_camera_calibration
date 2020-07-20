/*****************************
Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.
CV_
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
#include <aruco/markerdetector.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <fstream>
#include <aruco/arucofidmarkers.h>
#include <valarray>
using namespace std;
using namespace cv;

namespace aruco
{
  /************************************
 *
 *
 *
 *
 ************************************/
  MarkerDetector::MarkerDetector()
  {
    _doErosion=false;
    _enableCylinderWarp=false;
    _thresMethod=ADPT_THRES;
    _thresParam1=_thresParam2=7;
    _cornerMethod=LINES;
    _markerWarpSize=56;
    _speed=0;
    markerIdDetector_ptrfunc=aruco::FiducidalMarkers::detect;
    pyrdown_level=0; // no image reduction
    _minSize=0.04;
    _maxSize=0.5;
  }
  /************************************
 *
 *
 *
 *
 ************************************/

  MarkerDetector::~MarkerDetector()
  {

  }

  /************************************
 *
 *
 *
 *
 ************************************/
  void MarkerDetector::setDesiredSpeed ( int val )
  {
    if ( val<0 ) val=0;
    else if ( val>3 ) val=2;

    _speed=val;
    switch ( _speed )
    {

    case 0:
      _markerWarpSize=56;
      _cornerMethod=SUBPIX;
      _doErosion=true;
      break;

    case 1:
    case 2:
      _markerWarpSize=28;
      _cornerMethod=NONE;
      break;
    };
  }

  /************************************
 *
 *
 *
 *
 ************************************/
  void MarkerDetector::detect ( const  cv::Mat &input,std::vector<Marker> &detectedMarkers, CameraParameters camParams ,float markerSizeMeters ,bool setYPerpendicular) throw ( cv::Exception )
  {
    detect ( input, detectedMarkers,camParams.CameraMatrix ,camParams.Distorsion,  markerSizeMeters ,setYPerpendicular);
  }


  /************************************
 *
 * Main detection function. Performs all steps
 *
 *
 ************************************/
  void MarkerDetector::detect ( const  cv::Mat &input,vector<Marker> &detectedMarkers,Mat camMatrix ,Mat distCoeff ,float markerSizeMeters ,bool setYPerpendicular) throw ( cv::Exception )
  {


    //it must be a 3 channel image
    if ( input.type() ==CV_8UC3 )   cv::cvtColor ( input,grey,cv::COLOR_BGR2GRAY );
    else     grey=input;


    //     cv::cvtColor(grey,_ssImC ,CV_GRAY2BGR); //DELETE

    //clear input data
    detectedMarkers.clear();


    cv::Mat imgToBeThresHolded=grey;
    double ThresParam1=_thresParam1,ThresParam2=_thresParam2;
    //Must the image be downsampled before continue processing?
    if ( pyrdown_level!=0 )
    {
      reduced=grey;
      for ( int i=0;i<pyrdown_level;i++ )
      {
        cv::Mat tmp;
        cv::pyrDown ( reduced,tmp );
        reduced=tmp;
      }
      int red_den=pow ( 2.0f,pyrdown_level );
      imgToBeThresHolded=reduced;
      ThresParam1/=float ( red_den );
      ThresParam2/=float ( red_den );
    }

    ///Do threshold the image and detect contours
    thresHold ( _thresMethod,imgToBeThresHolded,thres,ThresParam1,ThresParam2 );
    //an erosion might be required to detect chessboard like boards

    if ( _doErosion )
    {
      erode ( thres,thres2,cv::Mat() );
      thres2=thres;
      //         cv::bitwise_xor ( thres,thres2,thres );
    }
    //find all rectangles in the thresholdes image
    vector<MarkerCandidate > MarkerCanditates;
    detectRectangles ( thres,MarkerCanditates );
    //if the image has been downsampled, then calcualte the location of the corners in the original image
    if ( pyrdown_level!=0 )
    {
      float red_den=pow ( 2.0f,pyrdown_level );
      float offInc= ( ( pyrdown_level/2. )-0.5 );
      for ( unsigned int i=0;i<MarkerCanditates.size();++i ) {
        for ( int c=0;c<4;c++ )
        {
          MarkerCanditates[i][c].x=MarkerCanditates[i][c].x*red_den+offInc;
          MarkerCanditates[i][c].y=MarkerCanditates[i][c].y*red_den+offInc;
        }
        //do the same with the the contour points
        for ( size_t c=0;c<MarkerCanditates[i].contour.size();++c )
        {
          MarkerCanditates[i].contour[c].x=MarkerCanditates[i].contour[c].x*red_den+offInc;
          MarkerCanditates[i].contour[c].y=MarkerCanditates[i].contour[c].y*red_den+offInc;
        }
      }
    }

    ///identify the markers
    _candidates.clear();
    for ( unsigned int i=0;i<MarkerCanditates.size();++i )
    {
      //Find proyective homography
      Mat canonicalMarker;
      bool resW=false;
      if (_enableCylinderWarp)
        resW=warp_cylinder( grey,canonicalMarker,Size ( _markerWarpSize,_markerWarpSize ),MarkerCanditates[i] );
      else  resW=warp ( grey,canonicalMarker,Size ( _markerWarpSize,_markerWarpSize ),MarkerCanditates[i] );
      if (resW) {
        int nRotations;
        int id= ( *markerIdDetector_ptrfunc ) ( canonicalMarker,nRotations );
        if ( id!=-1 )
        {
          if(_cornerMethod==LINES) refineCandidateLines( MarkerCanditates[i] ); // make LINES refinement before lose contour points
          detectedMarkers.push_back ( MarkerCanditates[i] );
          detectedMarkers.back().id=id;
          //sort the points so that they are always in the same order no matter the camera orientation
          std::rotate ( detectedMarkers.back().begin(),detectedMarkers.back().begin() +4-nRotations,detectedMarkers.back().end() );
        }
        else _candidates.push_back ( MarkerCanditates[i] );
      }

    }


    ///refine the corner location if desired
    if ( detectedMarkers.size() >0 && _cornerMethod!=NONE && _cornerMethod!=LINES )
    {
      vector<Point2f> Corners;
      for ( unsigned int i=0;i<detectedMarkers.size();++i )
        for ( int c=0;c<4;c++ )
          Corners.push_back ( detectedMarkers[i][c] );

      if ( _cornerMethod==HARRIS )
        findBestCornerInRegion_harris ( grey, Corners,7 );
      else if ( _cornerMethod==SUBPIX )
        cornerSubPix ( grey, Corners,cv::Size ( 5,5 ), cv::Size ( -1,-1 )   ,cv::TermCriteria (  cv::TermCriteria::EPS | cv::TermCriteria::COUNT,3,0.05 ) );

      //copy back
      for ( unsigned int i=0;i<detectedMarkers.size();++i )
        for ( int c=0;c<4;c++ )     detectedMarkers[i][c]=Corners[i*4+c];
    }
    //sort by id
    std::sort ( detectedMarkers.begin(),detectedMarkers.end() );
    //there might be still the case that a marker is detected twice because of the double border indicated earlier,
    //detect and remove these cases
    vector<bool> toRemove ( detectedMarkers.size(),false );
    for ( int i=0;i<int ( detectedMarkers.size() )-1;++i )
    {
      if ( detectedMarkers[i].id==detectedMarkers[i+1].id && !toRemove[i+1] )
      {
        //deletes the one with smaller perimeter
        if ( perimeter ( detectedMarkers[i] ) >perimeter ( detectedMarkers[i+1] ) ) toRemove[i+1]=true;
        else toRemove[i]=true;
      }
    }
    //remove the markers marker
    removeElements ( detectedMarkers, toRemove );

    ///detect the position of detected markers if desired
    if ( camMatrix.rows!=0  && markerSizeMeters>0 )
    {
      for ( unsigned int i=0;i<detectedMarkers.size();i++ )
        detectedMarkers[i].calculateExtrinsics ( markerSizeMeters,camMatrix,distCoeff,setYPerpendicular );
    }
  }


  /************************************
 *
 * Crucial step. Detects the rectangular regions of the thresholded image
 *
 *
 ************************************/
  void  MarkerDetector::detectRectangles ( const cv::Mat &thresold,vector<std::vector<cv::Point2f> > &MarkerCanditates )
  {
    vector<MarkerCandidate>  candidates;
    detectRectangles(thresold,candidates);
    //create the output
    MarkerCanditates.resize(candidates.size());
    for (size_t i=0;i<MarkerCanditates.size();i++)
      MarkerCanditates[i]=candidates[i];
  }

  void MarkerDetector::detectRectangles(const cv::Mat &thresImg,vector<MarkerCandidate> & OutMarkerCanditates)
  {
    vector<MarkerCandidate>  MarkerCanditates;
    //calcualte the min_max contour sizes
    unsigned int minSize=_minSize*std::max(thresImg.cols,thresImg.rows)*4;
    unsigned int maxSize=_maxSize*std::max(thresImg.cols,thresImg.rows)*4;
    std::vector<std::vector<cv::Point> > contours2;
    std::vector<cv::Vec4i> hierarchy2;

    thresImg.copyTo ( thres2 );
    cv::findContours ( thres2 , contours2, hierarchy2, cv::RETR_TREE, cv::CHAIN_APPROX_NONE );
    vector<Point>  approxCurve;
    ///for each contour, analyze if it is a paralelepiped likely to be the marker

    for ( unsigned int i=0;i<contours2.size();i++ )
    {


      //check it is a possible element by first checking is has enough points
      if ( minSize< contours2[i].size() &&contours2[i].size()<maxSize  )
      {
        //approximate to a poligon
        approxPolyDP (  contours2[i]  ,approxCurve , double ( contours2[i].size() ) *0.05 , true );
        // 				drawApproxCurve(copy,approxCurve,Scalar(0,0,255));
        //check that the poligon has 4 points
        if ( approxCurve.size() ==4 )
        {

          //  	   drawContour(input,contours2[i],Scalar(255,0,225));
          //  		  namedWindow("input");
          //  		imshow("input",input);
          //  	 	waitKey(0);
          //and is convex
          if ( isContourConvex ( Mat ( approxCurve ) ) )
          {
            // 					      drawApproxCurve(input,approxCurve,Scalar(255,0,255));
            // 						//ensure that the   distace between consecutive points is large enough
            float minDist=1e10;
            for ( int j=0;j<4;j++ )
            {
              float d= std::sqrt ( ( float ) ( approxCurve[j].x-approxCurve[ ( j+1 ) %4].x ) * ( approxCurve[j].x-approxCurve[ ( j+1 ) %4].x ) +
                                   ( approxCurve[j].y-approxCurve[ ( j+1 ) %4].y ) * ( approxCurve[j].y-approxCurve[ ( j+1 ) %4].y ) );
              // 		norm(Mat(approxCurve[i]),Mat(approxCurve[(i+1)%4]));
              if ( d<minDist ) minDist=d;
            }
            //check that distance is not very small
            if ( minDist>10 )
            {
              //add the points
              // 	      cout<<"ADDED"<<endl;
              MarkerCanditates.push_back ( MarkerCandidate() );
              MarkerCanditates.back().idx=i;
              for ( int j=0;j<4;j++ )
              {
                MarkerCanditates.back().push_back ( Point2f ( approxCurve[j].x,approxCurve[j].y ) );
              }
            }
          }
        }
      }
    }

    // 		 		  namedWindow("input");
    //  		imshow("input",input);
    //  						waitKey(0);
    ///sort the points in anti-clockwise order
    valarray<bool> swapped(false,MarkerCanditates.size());//used later
    for ( unsigned int i=0;i<MarkerCanditates.size();i++ )
    {

      //trace a line between the first and second point.
      //if the thrid point is at the right side, then the points are anti-clockwise
      double dx1 = MarkerCanditates[i][1].x - MarkerCanditates[i][0].x;
      double dy1 =  MarkerCanditates[i][1].y - MarkerCanditates[i][0].y;
      double dx2 = MarkerCanditates[i][2].x - MarkerCanditates[i][0].x;
      double dy2 = MarkerCanditates[i][2].y - MarkerCanditates[i][0].y;
      double o = ( dx1*dy2 )- ( dy1*dx2 );

      if ( o  < 0.0 )		 //if the third point is in the left side, then sort in anti-clockwise order
      {
        swap ( MarkerCanditates[i][1],MarkerCanditates[i][3] );
        swapped[i]=true;
        //sort the contour points
        //  	    reverse(MarkerCanditates[i].contour.begin(),MarkerCanditates[i].contour.end());//????

      }
    }

    /// remove these elements whise corners are too close to each other
    //first detect candidates

    vector<pair<int,int>  > TooNearCandidates;
    for ( unsigned int i=0;i<MarkerCanditates.size();i++ )
    {
      // 	cout<<"Marker i="<<i<<MarkerCanditates[i]<<endl;
      //calculate the average distance of each corner to the nearest corner of the other marker candidate
      for ( unsigned int j=i+1;j<MarkerCanditates.size();j++ )
      {
        float dist=0;
        for ( int c=0;c<4;c++ )
          dist+= sqrt ( ( MarkerCanditates[i][c].x-MarkerCanditates[j][c].x ) * ( MarkerCanditates[i][c].x-MarkerCanditates[j][c].x ) + ( MarkerCanditates[i][c].y-MarkerCanditates[j][c].y ) * ( MarkerCanditates[i][c].y-MarkerCanditates[j][c].y ) );
        dist/=4;
        //if distance is too small
        if ( dist< 10 )
        {
          TooNearCandidates.push_back ( pair<int,int> ( i,j ) );
        }
      }
    }

    //mark for removal the element of  the pair with smaller perimeter
    valarray<bool> toRemove ( false,MarkerCanditates.size() );
    for ( unsigned int i=0;i<TooNearCandidates.size();i++ )
    {
      if ( perimeter ( MarkerCanditates[TooNearCandidates[i].first ] ) >perimeter ( MarkerCanditates[ TooNearCandidates[i].second] ) )
        toRemove[TooNearCandidates[i].second]=true;
      else toRemove[TooNearCandidates[i].first]=true;
    }

    //remove the invalid ones
    //     removeElements ( MarkerCanditates,toRemove );
    //finally, assign to the remaining candidates the contour
    OutMarkerCanditates.reserve(MarkerCanditates.size());
    for (size_t i=0;i<MarkerCanditates.size();i++) {
      if (!toRemove[i]) {
        OutMarkerCanditates.push_back(MarkerCanditates[i]);
        OutMarkerCanditates.back().contour=contours2[ MarkerCanditates[i].idx];
        if (swapped[i] && _enableCylinderWarp )//if the corners where swapped, it is required to reverse here the points so that they are in the same order
          reverse(OutMarkerCanditates.back().contour.begin(),OutMarkerCanditates.back().contour.end());//????
      }
    }

  }

  /************************************
 *
 *
 *
 *
 ************************************/
  void MarkerDetector::thresHold ( int method,const Mat &grey_m,Mat &out,double param1,double param2 ) throw ( cv::Exception )
  {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
    if (param1==-1) param1=_thresParam1;
    if (param2==-1) param2=_thresParam2;
#pragma GCC diagnostic pop 

    if ( grey_m.type() !=CV_8UC1 )     throw cv::Exception ( 9001,"grey_m.type()!=CV_8UC1","MarkerDetector::thresHold",__FILE__,__LINE__ );
    switch ( method )
    {
    case FIXED_THRES:
      cv::threshold ( grey_m, out, param1,255, cv::THRESH_BINARY_INV );
      break;
    case ADPT_THRES://currently, this is the best method
      //ensure that _thresParam1%2==1
      if ( param1<3 ) param1=3;
      else if ( ( ( int ) param1 ) %2 !=1 ) param1= ( int ) ( param1+1 );

      cv::adaptiveThreshold ( grey_m,out,255,ADAPTIVE_THRESH_MEAN_C,cv::THRESH_BINARY_INV,param1,param2 );
      break;
    case CANNY:
    {
      //this should be the best method, and generally it is.
      //However, some times there are small holes in the marker contour that makes
      //the contour detector not to find it properly
      //if there is a missing pixel
      cv::Canny ( grey_m, out, 10, 220 );
      //I've tried a closing but it add many more points that some
      //times makes this even worse
      // 			  Mat aux;
      // 			  cv::morphologyEx(thres,aux,MORPH_CLOSE,Mat());
      // 			  out=aux;
    }
      break;
    }
  }
  /************************************
 *
 *
 *
 *
 ************************************/
  bool MarkerDetector::warp ( Mat &in,Mat &out,Size size, vector<Point2f> points ) throw ( cv::Exception )
  {

    if ( points.size() !=4 )    throw cv::Exception ( 9001,"point.size()!=4","MarkerDetector::warp",__FILE__,__LINE__ );
    //obtain the perspective transform
    Point2f  pointsRes[4],pointsIn[4];
    for ( int i=0;i<4;i++ ) pointsIn[i]=points[i];
    pointsRes[0]= ( Point2f ( 0,0 ) );
    pointsRes[1]= Point2f ( size.width-1,0 );
    pointsRes[2]= Point2f ( size.width-1,size.height-1 );
    pointsRes[3]= Point2f ( 0,size.height-1 );
    Mat M=getPerspectiveTransform ( pointsIn,pointsRes );
    cv::warpPerspective ( in, out,  M, size,cv::INTER_NEAREST );
    return true;
  }

  void findCornerPointsInContour(const vector<cv::Point2f>& points,const vector<cv::Point> &contour,vector<int> &idxs)
  {
    assert(points.size()==4);
    int idxSegments[4]={-1,-1,-1,-1};
    //the first point coincides with one
    cv::Point points2i[4];
    for (int i=0;i<4;i++) {
      points2i[i].x=points[i].x;
      points2i[i].y=points[i].y;
    }

    for (size_t i=0;i<contour.size();++i) {
      if (idxSegments[0]==-1)
        if (contour[i]==points2i[0]) idxSegments[0]=i;
      if (idxSegments[1]==-1)
        if (contour[i]==points2i[1]) idxSegments[1]=i;
      if (idxSegments[2]==-1)
        if (contour[i]==points2i[2]) idxSegments[2]=i;
      if (idxSegments[3]==-1)
        if (contour[i]==points2i[3]) idxSegments[3]=i;
    }
    idxs.resize(4);
    for (int i=0;i<4;i++) idxs[i]=idxSegments[i];
  }

  int findDeformedSidesIdx(const vector<cv::Point> &contour,const vector<int> &idxSegments)
  {
    float distSum[4]={0,0,0,0};

    for (int i=0;i<3;i++) {
      cv::Point p1=contour[ idxSegments[i]];
      cv::Point p2=contour[ idxSegments[i+1]];
      float inv_den=1./ sqrt(float(( p2.x-p1.x)*(p2.x-p1.x)+ (p2.y-p1.y)*(p2.y-p1.y)));
      //   d=|v^^·r|=(|(x_2-x_1)(y_1-y_0)-(x_1-x_0)(y_2-y_1)|)/(sqrt((x_2-x_1)^2+(y_2-y_1)^2)).
      //         cerr<<"POSS="<<idxSegments[i]<<" "<<idxSegments[i+1]<<endl;
      for (int j=idxSegments[i];j<idxSegments[i+1];++j) {
        float dist=std::fabs( float(  (p2.x-p1.x)*(p1.y-contour[j].y)-  (p1.x-contour[j].x)*(p2.y-p1.y)) )*inv_den;
        distSum[i]+=dist;
        //             cerr<< dist<<" ";
        //             cv::rectangle(_ssImC,contour[j],contour[j],colors[i],-1);
      }
      distSum[i]/=float(idxSegments[i+1]-idxSegments[i]);
      //         cout<<endl<<endl;
    }


    //for the last one
    cv::Point p1=contour[ idxSegments[0]];
    cv::Point p2=contour[ idxSegments[3]];
    float inv_den=1./ std::sqrt(float(( p2.x-p1.x)*(p2.x-p1.x)+ (p2.y-p1.y)*(p2.y-p1.y)));
    //   d=|v^^·r|=(|(x_2-x_1)(y_1-y_0)-(x_1-x_0)(y_2-y_1)|)/(sqrt((x_2-x_1)^2+(y_2-y_1)^2)).
    for (int j=0;j<idxSegments[0];++j)
      distSum[3]+=std::fabs(   float((p2.x-p1.x)*(p1.y-contour[j].y)-  (p1.x-contour[j].x)*(p2.y-p1.y)))*inv_den;
    for (int j=idxSegments[3];j<static_cast<int>(contour.size());++j)
      distSum[3]+=std::fabs(   float((p2.x-p1.x)*(p1.y-contour[j].y)-  (p1.x-contour[j].x)*(p2.y-p1.y)))*inv_den;

    distSum[3]/=float(  idxSegments[0]+  (contour.size()-idxSegments[3]));
    //now, get the maximum
    /*    for (int i=0;i<4;i++)
            cout<<"DD="<<distSum[i]<<endl;*/
    //check the two combinations to see the one with higher error
    if ( distSum[0]+distSum[2]> distSum[1]+distSum[3])
      return 0;
    else return 1;
  }

  void setPointIntoImage(cv::Point2f &p,cv::Size s) {
    if (p.x<0) p.x=0;
    else if (p.x>=s.width )p.x=s.width-1;
    if (p.y<0)p.y=0;
    else if (p.y>=s.height)p.y=s.height-1;

  }

  void setPointIntoImage(cv::Point  &p,cv::Size s) {
    if (p.x<0) p.x=0;
    else if (p.x>=s.width )p.x=s.width-1;
    if (p.y<0)p.y=0;
    else if (p.y>=s.height)p.y=s.height-1;

  }
  /************************************
 *
 *
 *
 *
 ************************************/
  bool MarkerDetector::warp_cylinder ( Mat &in,Mat &out,Size size, MarkerCandidate& mcand ) throw ( cv::Exception )
  {

    if ( mcand.size() !=4 )    throw cv::Exception ( 9001,"point.size()!=4","MarkerDetector::warp",__FILE__,__LINE__ );

    //check first the real need for cylinder warping
    //     cout<<"im="<<mcand.contour.size()<<endl;

    //     for (size_t i=0;i<mcand.contour.size();i++) {
    //         cv::rectangle(_ssImC ,mcand.contour[i],mcand.contour[i],cv::Scalar(111,111,111),-1 );
    //     }
    //     mcand.draw(imC,cv::Scalar(0,255,0));
    //find the 4 different segments of the contour
    vector<int> idxSegments;
    findCornerPointsInContour(mcand,mcand.contour,idxSegments);
    //let us rearrange the points so that the first corner is the one whith smaller idx
    int minIdx=0;
    for (int i=1;i<4;i++)
      if (idxSegments[i] <idxSegments[minIdx]) minIdx=i;
    //now, rotate the points to be in this order
    std::rotate(idxSegments.begin(),idxSegments.begin()+minIdx,idxSegments.end());
    std::rotate(mcand.begin(),mcand.begin()+minIdx,mcand.end());

    //     cout<<"idxSegments="<<idxSegments[0]<< " "<<idxSegments[1]<< " "<<idxSegments[2]<<" "<<idxSegments[3]<<endl;
    //now, determine the sides that are deformated by cylinder perspective
    int defrmdSide=findDeformedSidesIdx(mcand.contour,idxSegments);
    //     cout<<"Def="<<defrmdSide<<endl;

    //instead of removing perspective distortion  of the rectangular region
    //given by the rectangle, we enlarge it a bit to include the deformed parts
    Point2f enlargedRegion[4];
    for (int i=0;i<4;i++) enlargedRegion[i]=mcand[i];
    if (defrmdSide==0) {
      enlargedRegion[0]=mcand[0]+(mcand[3]-mcand[0])*1.2;
      enlargedRegion[1]=mcand[1]+(mcand[2]-mcand[1])*1.2;
      enlargedRegion[2]=mcand[2]+(mcand[1]-mcand[2])*1.2;
      enlargedRegion[3]=mcand[3]+(mcand[0]-mcand[3])*1.2;
    }
    else {
      enlargedRegion[0]=mcand[0]+(mcand[1]-mcand[0])*1.2;
      enlargedRegion[1]=mcand[1]+(mcand[0]-mcand[1])*1.2;
      enlargedRegion[2]=mcand[2]+(mcand[3]-mcand[2])*1.2;
      enlargedRegion[3]=mcand[3]+(mcand[2]-mcand[3])*1.2;
    }
    for (size_t i=0;i<4;i++)
      setPointIntoImage(enlargedRegion[i],in.size());

    /*
        cv::Scalar colors[4]={cv::Scalar(0,0,255),cv::Scalar(255,0,0),cv::Scalar(0,255,0),cv::Scalar(111,111,0)};
        for (int i=0;i<4;i++) {
            cv::rectangle(_ssImC,mcand.contour[idxSegments[i]]-cv::Point(2,2),mcand.contour[idxSegments[i]]+cv::Point(2,2),colors[i],-1 );
            cv::rectangle(_ssImC,enlargedRegion[i]-cv::Point2f(2,2),enlargedRegion[i]+cv::Point2f(2,2),colors[i],-1 );

        }*/
    //     cv::imshow("imC",_ssImC);


    //calculate the max distance from each contour point the line of the corresponding segment it belongs to
    //     calculate
    //      cv::waitKey(0);
    //check that the region is into image limits
    //obtain the perspective transform
    Point2f  pointsRes[4];

    cv::Size enlargedSize=size;
    enlargedSize.width+=2*enlargedSize.width*0.2;
    pointsRes[0]= ( Point2f ( 0,0 ) );
    pointsRes[1]= Point2f ( enlargedSize.width-1,0 );
    pointsRes[2]= Point2f ( enlargedSize.width-1,enlargedSize.height-1 );
    pointsRes[3]= Point2f ( 0,enlargedSize.height-1 );
    //rotate to ensure that deformed sides are in the horizontal axis when warping
    if (defrmdSide==0) rotate(pointsRes,pointsRes+1,pointsRes+4);
    cv::Mat imAux,imAux2(enlargedSize,CV_8UC1);
    Mat M=getPerspectiveTransform ( enlargedRegion,pointsRes );
    cv::warpPerspective ( in, imAux,  M, enlargedSize,cv::INTER_NEAREST);

    //now, transform all points to the new image
    vector<cv::Point> pointsCO(mcand.contour.size());
    assert(M.type()==CV_64F);
    assert(M.cols==3 && M.rows==3);
    //     cout<<M<<endl;
    double *mptr=M.ptr<double>(0);
    imAux2.setTo(cv::Scalar::all(0));


    for (size_t i=0;i<mcand.contour.size();i++) {
      float inX=mcand.contour[i].x;
      float inY=mcand.contour[i].y;
      float w= inX * mptr[6]+inY * mptr[7]+mptr[8];
      cv::Point2f pres;
      pointsCO[i].x=( (inX * mptr[0]+inY* mptr[1]+mptr[2])/w)+0.5;
      pointsCO[i].y=( (inX * mptr[3]+inY* mptr[4]+mptr[5])/w)+0.5;
      //make integers
      setPointIntoImage(pointsCO[i],imAux.size());//ensure points are into image limits
      // 	cout<<"p="<<pointsCO[i]<<" "<<imAux.size().width<<" "<<imAux.size().height<<endl;
      imAux2.at<uchar>(pointsCO[i].y,pointsCO[i].x)=255;
      if (pointsCO[i].y>0)
        imAux2.at<uchar>(pointsCO[i].y-1,pointsCO[i].x)=255;
      if (pointsCO[i].y<imAux2.rows-1 )
        imAux2.at<uchar>(pointsCO[i].y+1,pointsCO[i].x)=255;
    }

    cv::Mat outIm(enlargedSize,CV_8UC1);
    outIm.setTo(cv::Scalar::all(0));
    //now, scan in lines to determine the required displacement
    for (int y=0;y<imAux2.rows;y++) {
      uchar *_offInfo=imAux2.ptr<uchar>(y);
      int start=-1,end=-1;
      //determine the start and end of markerd regions
      for (int x=0;x<imAux.cols;x++) {
        if (_offInfo[x]) {
          if (start==-1) start=x;
          else end=x;
        }
      }
      //       cout<<"S="<<start<<" "<<end<<" "<<end-start<<" "<<(size.width>>1)<<endl;
      //check that the size is big enough and
      assert(start!=-1 && end!=-1 && (end-start)> size.width>>1);
      uchar *In_image=imAux.ptr<uchar>(y);
      uchar *Out_image=outIm.ptr<uchar>(y);
      memcpy(Out_image,In_image+start,imAux.cols-start );
    }


    //     cout<<"SS="<<mcand.contour.size()<<" "<<pointsCO.size()<<endl;
    //get the central region with the size specified
    cv::Mat centerReg=outIm(cv::Range::all(),cv::Range(0,size.width));
    out=centerReg.clone();
    //     cv::perspectiveTransform(mcand.contour,pointsCO,M);
    //draw them
    //     cv::imshow("out2",out);
    //     cv::imshow("imm",imAux2);
    //     cv::waitKey(0);
    return true;
  }
  /************************************
 *
 *
 *
 *
 ************************************/
  bool MarkerDetector::isInto ( Mat &contour,vector<Point2f> &b )
  {

    for ( size_t i=0;i<b.size();i++ )
      if ( pointPolygonTest ( contour,b[i],false ) >0 ) return true;
    return false;
  }
  /************************************
 *
 *
 *
 *
 ************************************/
  int MarkerDetector:: perimeter ( vector<Point2f> &a )
  {
    int sum=0;
    for ( size_t i=0;i<a.size();i++ )
    {
      int i2= ( i+1 ) %a.size();
      sum+= sqrt ( ( a[i].x-a[i2].x ) * ( a[i].x-a[i2].x ) + ( a[i].y-a[i2].y ) * ( a[i].y-a[i2].y ) ) ;
    }
    return sum;
  }


  /**
 *
 *
 */
  void MarkerDetector::findBestCornerInRegion_harris ( const cv::Mat  & grey_m,vector<cv::Point2f> &  Corners,int blockSize )
  {
    int halfSize=blockSize/2;
    for ( size_t i=0;i<Corners.size();i++ )
    {
      //check that the region is into the image limits
      cv::Point2f min ( Corners[i].x-halfSize,Corners[i].y-halfSize );
      cv::Point2f max ( Corners[i].x+halfSize,Corners[i].y+halfSize );
      if ( min.x>=0  &&  min.y>=0 && max.x<grey_m.cols && max.y<grey_m.rows )
      {
        cv::Mat response;
        cv::Mat subImage ( grey_m,cv::Rect ( Corners[i].x-halfSize,Corners[i].y-halfSize,blockSize ,blockSize ) );
        vector<Point2f> corners2;
        goodFeaturesToTrack ( subImage, corners2, 10, 0.001, halfSize );
        float minD=9999;
        int bIdx=-1;
        cv::Point2f Center ( halfSize,halfSize );
        for ( size_t j=0;j<corners2.size();j++ )
        {
          float dist=cv::norm ( corners2[j]-Center );
          if ( dist<minD )
          {
            minD=dist;
            bIdx=j;
          }
          if ( minD<halfSize ) Corners[i]+= ( corners2[bIdx]-Center );
        }
      }
    }
  }


  /**
 *
 *
 */
  void MarkerDetector::refineCandidateLines(MarkerDetector::MarkerCandidate& candidate)
  {
    // search corners on the contour vector
    vector<size_t> cornerIndex;
    cornerIndex.resize(4);
    for(size_t j=0; j<candidate.contour.size(); ++j) {
      for(size_t k=0; k<4; ++k) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
        if(candidate.contour[j].x==candidate[k].x && candidate.contour[j].y==candidate[k].y) {
          cornerIndex[k] = j;
        }
#pragma GCC diagnostic pop 
      }
    }

    // contour pixel in inverse order or not?
    bool inverse;
    if( (cornerIndex[1] > cornerIndex[0]) && (cornerIndex[2]>cornerIndex[1] || cornerIndex[2]<cornerIndex[0]) )
      inverse = false;
    else if( cornerIndex[2]>cornerIndex[1] && cornerIndex[2]<cornerIndex[0] )
      inverse = false;
    else inverse = true;


    // get pixel vector for each line of the marker
    int inc = 1;
    if(inverse) inc = -1;

    vector<std::vector<cv::Point> > contourLines;
    contourLines.resize(4);
    for(unsigned int l=0; l<4; ++l) {
      for(int j=(int)cornerIndex[l]; j!=(int)cornerIndex[(l+1)%4]; j+=inc) {
        if(j==(int)candidate.contour.size() && !inverse) j=0;
        else if(j==0 && inverse) j=candidate.contour.size()-1;
        contourLines[l].push_back(candidate.contour[j]);
        if(j==(int)cornerIndex[(l+1)%4]) break; // this has to be added because of the previous ifs
      }

    }

    // interpolate marker lines
    vector<Point3f> lines;
    lines.resize(4);
    for(unsigned int j=0; j<lines.size(); ++j)
      interpolate2Dline(contourLines[j], lines[j]);

    // get cross points of lines
    vector<Point2f> crossPoints;
    crossPoints.resize(4);
    for(unsigned int i=0; i<4; ++i)
      crossPoints[i] = getCrossPoint( lines[(i-1)%4], lines[i] );

    // reassing points
    for(unsigned int j=0; j<4; ++j)
      candidate[j] = crossPoints[j];
  }


  /**
 */
  void MarkerDetector::interpolate2Dline( const std::vector< Point >& inPoints, Point3f& outLine)
  {

    float minX, maxX, minY, maxY;
    minX = maxX = inPoints[0].x;
    minY = maxY = inPoints[0].y;
    for(unsigned int i=1; i<inPoints.size(); ++i)  {
      if(inPoints[i].x < minX) minX = inPoints[i].x;
      if(inPoints[i].x > maxX) maxX = inPoints[i].x;
      if(inPoints[i].y < minY) minY = inPoints[i].y;
      if(inPoints[i].y > maxY) maxY = inPoints[i].y;
    }

    // create matrices of equation system
    Mat A(inPoints.size(),2,CV_32FC1, Scalar(0));
    Mat B(inPoints.size(),1,CV_32FC1, Scalar(0));
    Mat X;

    
    
    if( maxX-minX > maxY-minY ) {
      // Ax + C = y
      for (size_t i=0; i<inPoints.size(); ++i) {

        A.at<float>(i, 0) = inPoints[i].x;
        A.at<float>(i, 1) = 1.;
        B.at<float>(i, 0) = inPoints[i].y;

      }

      // solve system
      solve(A,B,X, DECOMP_SVD);
      // return Ax + By + C
      outLine = Point3f(X.at<float>(0,0), -1., X.at<float>(1,0));
    }
    else {
      // By + C = x
      for (size_t i=0; i<inPoints.size(); ++i) {

        A.at<float>(i, 0) = inPoints[i].y;
        A.at<float>(i, 1) = 1.;
        B.at<float>(i, 0) = inPoints[i].x;

      }

      // solve system
      solve(A,B,X, DECOMP_SVD);
      // return Ax + By + C
      outLine = Point3f(-1., X.at<float>(0,0), X.at<float>(1,0));
    }

  }

  /**
 */
  Point2f MarkerDetector::getCrossPoint(const cv::Point3f& line1, const cv::Point3f& line2)
  {

    // create matrices of equation system
    Mat A(2,2,CV_32FC1, Scalar(0));
    Mat B(2,1,CV_32FC1, Scalar(0));
    Mat X;

    A.at<float>(0, 0) = line1.x;
    A.at<float>(0, 1) = line1.y;
    B.at<float>(0, 0) = -line1.z;

    A.at<float>(1, 0) = line2.x;
    A.at<float>(1, 1) = line2.y;
    B.at<float>(1, 0) = -line2.z;

    // solve system
    solve(A,B,X, DECOMP_SVD);
    return Point2f(X.at<float>(0,0), X.at<float>(1,0));

  }








  /************************************
 *
 *
 *
 *
 ************************************/
  void MarkerDetector::drawAllContours ( Mat input, std::vector<std::vector<cv::Point> > &contours )
  {
    drawContours ( input,  contours, -1,Scalar ( 255,0,255 ) );
  }

  /************************************
 *
 *
 *
 *
 ************************************/
  void MarkerDetector:: drawContour ( Mat &in,vector<Point>  &contour,Scalar color )
  {
    for ( size_t i=0;i<contour.size();++i )
    {
      cv::rectangle ( in,contour[i],contour[i],color );
    }
  }

  void  MarkerDetector:: drawApproxCurve ( Mat &in,vector<Point>  &contour,Scalar color )
  {
    for ( size_t i=0;i<contour.size();++i )
    {
      cv::line ( in,contour[i],contour[ ( i+1 ) %contour.size() ],color );
    }
  }
  /************************************
 *
 *
 *
 *
 ************************************/

  void MarkerDetector::draw ( Mat out,const vector<Marker> &markers )
  {
    for ( size_t i=0;i<markers.size();++i )
    {
      cv::line ( out,markers[i][0],markers[i][1],cv::Scalar ( 255,0,0 ),2,LINE_AA );
      cv::line ( out,markers[i][1],markers[i][2],cv::Scalar ( 255,0,0 ),2,LINE_AA );
      cv::line ( out,markers[i][2],markers[i][3],cv::Scalar ( 255,0,0 ),2,LINE_AA );
      cv::line ( out,markers[i][3],markers[i][0],cv::Scalar ( 255,0,0 ),2,LINE_AA );
    }
  }
  /* Attempt to make it faster than in opencv. I could not :( Maybe trying with SSE3...
void MarkerDetector::warpPerspective(const cv::Mat &in,cv::Mat & out, const cv::Mat & M,cv::Size size)
{
   //inverse the matrix
   out.create(size,in.type());
   //convert to float to speed up operations
   const double *m=M.ptr<double>(0);
   float mf[9];
   mf[0]=m[0];mf[1]=m[1];mf[2]=m[2];
   mf[3]=m[3];mf[4]=m[4];mf[5]=m[5];
   mf[6]=m[6];mf[7]=m[7];mf[8]=m[8];

   for(int y=0;y<out.rows;y++){
     uchar *_ptrout=out.ptr<uchar>(y);
     for(int x=0;x<out.cols;x++){
   //get the x,y position
   float den=1./(x*mf[6]+y*mf[7]+mf[8]);
   float ox= (x*mf[0]+y*mf[1]+mf[2])*den;
   float oy= (x*mf[3]+y*mf[4]+mf[5])*den;
   _ptrout[x]=in.at<uchar>(oy,ox);
     }
   }
}
*/

  /************************************
 *
 *
 *
 *
 ************************************/

  void MarkerDetector::glGetProjectionMatrix ( CameraParameters &  CamMatrix,cv::Size orgImgSize, cv::Size size,double proj_matrix[16],double gnear,double gfar,bool invert )
  throw ( cv::Exception )
  {
    cerr<<"MarkerDetector::glGetProjectionMatrix . This a deprecated function. Use CameraParameters::glGetProjectionMatrix instead. "<<__FILE__<<" "<<__LINE__<<endl;
    CamMatrix.glGetProjectionMatrix ( orgImgSize,size,proj_matrix,gnear,gfar,invert );
  }

  /************************************
*
*
*
*
************************************/

  void MarkerDetector::setMinMaxSize(float min ,float max )throw(cv::Exception)
  {
    if (min<=0 || min>1) throw cv::Exception(1," min parameter out of range","MarkerDetector::setMinMaxSize",__FILE__,__LINE__);
    if (max<=0 || max>1) throw cv::Exception(1," max parameter out of range","MarkerDetector::setMinMaxSize",__FILE__,__LINE__);
    if (min>max) throw cv::Exception(1," min>max","MarkerDetector::setMinMaxSize",__FILE__,__LINE__);
    _minSize=min;
    _maxSize=max;
  }

}

