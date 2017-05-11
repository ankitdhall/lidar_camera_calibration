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
/****
 * Select the n best markers according to their distance. In other words, selects the n farthert markers in terms of hamming distance
 *
 *
 *
 *****/

#include <opencv2/highgui/highgui.hpp>
#include <aruco/aruco.h>
#include <iostream>
#include <limits>
#include <aruco/arucofidmarkers.h>
using namespace cv;
using namespace std;

int HammDist_(const cv::Mat &m1,const cv::Mat & m2)
{
  
    int dist=0;
    for (int y=0;y<5;y++)
        for (int x=0;x<5;x++)
            if (m1.at<uchar>(y,x)!=m2.at<uchar>(y,x)) dist++;
    return dist;

}
Mat rotate(Mat  in)
{
    Mat out;
    in.copyTo(out);
    for (int i=0;i<in.rows;i++)
    {
        for (int j=0;j<in.cols;j++)
        {
            out.at<uchar>(i,j)=in.at<uchar>(in.cols-j-1,i);
        }
    }
    return out;
}


int HammDist(const cv::Mat &m1,const cv::Mat & m2)
{
    cv::Mat mc=m1.clone();
    int minD=std::numeric_limits<int>::max();
    for(int i=0;i<4;i++){
      int dist=HammDist_(mc,m2);
      if( dist<minD) minD=dist;
      mc= rotate(mc);
    }
    return minD;

}

int entropy(const cv::Mat &marker)
{
  
  //the entropy is calcualte for each bin as the number of elements different from it in its sourroundings
  int totalEntropy=0;
    for (int y=0;y<5;y++)
        for (int x=0;x<5;x++){
	    int minX=max(x-1,0);
	    int maxX=min(x+1,5);
	    int minY=max(y-1,0);
	    int maxY=min(y+1,5);
	    
	    for(int yy=minY;yy<maxY;yy++)
	      for(int xx=minX;xx<maxX;xx++)
		  if (marker.at<uchar>(y,x)!=marker.at<uchar>(yy,xx)) totalEntropy++;
	}
     
    return totalEntropy;
}

int main(int argc,char **argv)
{
    try {
        if (argc<4) {

            //You can also use ids 2000-2007 but it is not safe since there are a lot of false positives.
            cerr<<"Usage: nofMarkers outbasename size [minimum_entropy(9,25)]"<<endl;
            return -1;
        }
        
      
        //create a vector with all markers
        int minimimEntropy=0;
	if(argc>=5) minimimEntropy=atoi(argv[4]);
        vector<cv::Mat> markers;
	vector<int> ventropy;
        for (int i=0;i<1024;i++){
            markers.push_back(aruco::FiducidalMarkers::getMarkerMat(i) );
 	    ventropy.push_back(entropy( aruco::FiducidalMarkers::getMarkerMat(i) ));
	}
	  cout<<"Calculating distance matrix"<<endl;
        //create a matrix with all distances
        cv::Mat distances=cv::Mat::zeros(1024,1024,CV_32SC1);
        for (int i=0;i<1024;i++)
            for (int j=i+1;j<1024;j++)
                distances.at<int>(i,j)=distances.at<int>(j,i)= HammDist (  markers[i],markers[j]);
	cout<<"done"<<endl;
	    //
        int nMarkers=atoi(argv[1]);
        //select the first marker
        vector<bool> usedMarkers(1024,false);
 
	
	
        vector<int> selectedMarkers;
	//select the masker with higher entropy first
	int bestEntr=0;
	for(size_t i=0;i<ventropy.size();i++)
	  if (ventropy[i]>ventropy[bestEntr]) bestEntr=i;
        selectedMarkers.push_back(bestEntr);
        usedMarkers[bestEntr]=true;
	
	//remove these with low entropy. Not very elegnat. Other day I will improve the algorithm
	//to make it multiobjective
	for(size_t i=0;i<ventropy.size();i++)
	  if (ventropy[i]<minimimEntropy) usedMarkers[i]=true;
	  
	cout<<"Max Entroy in ="<<bestEntr<<" "<<ventropy[bestEntr]<<endl;
        //add new markers according to the distance added to the global
        for (int i=1;i<nMarkers;i++) {
	  int bestMarker=-1;
	  int shorterDist=0;
            //select as new marker the one that maximizes mean distance to
            for (int j=0;j<distances.cols;j++) {
                if (!usedMarkers[j]) {
                    int minDist=std::numeric_limits< int >::max();
                    for (size_t k=0;k<selectedMarkers.size();k++)
			if (distances.at<int> ( selectedMarkers[k], j)<minDist) minDist=distances.at<int> ( selectedMarkers[k], j);
// 		    cout<<"j="<<j<<" "<<distSum<<"|"<<flush;
		    if (minDist>shorterDist){ 
		      shorterDist=minDist;
		      bestMarker=j;
		    }
                }
            }
            if (bestMarker!=-1 && shorterDist>1 ){
	      selectedMarkers.push_back(bestMarker);
	      usedMarkers[bestMarker]=true;
// 	      cout<<"Best id="<<bestMarker<<" dist="<<farthestDist<< endl;
	    }
	    else {cerr<<"COUDL NOT ADD ANY MARKER"<<endl;exit(0);}
//             char c;cin>>c;
        }
        
        sort(selectedMarkers.begin(),selectedMarkers.end());
        for(size_t i=0;i<selectedMarkers.size();i++){
	  char name[1024];
	  sprintf(name,"%s%d.png",argv[2],selectedMarkers[i]);
// 	  cout<<"name="<<name<<endl;
	  cout<<selectedMarkers[i]<<" "<<flush;
	  Mat markerImage=aruco::FiducidalMarkers::createMarkerImage(selectedMarkers[i],atoi(argv[3]));
	  imwrite(name,markerImage);
	}
	cout<<endl;
	//print the minimim distance between any two  elements
	int minDist=std::numeric_limits<int>::max();
	for(size_t i=0;i<selectedMarkers.size()-1;i++)
	  for(size_t j=i+1;j<selectedMarkers.size();j++){
// 	    cout<<" d=" << selectedMarkers[i]<<" "<<selectedMarkers[j]<<"->"<<distances.at<int> ( selectedMarkers[i], selectedMarkers[j])<<endl;
	    if (distances.at<int> ( selectedMarkers[i], selectedMarkers[j]) < minDist) minDist=distances.at<int> ( selectedMarkers[i], selectedMarkers[j]);
	    
	  }
	    
	
   cout<<"Min Dist="<<minDist<<endl;

    }
    catch (std::exception &ex)
    {
        cout<<ex.what()<<endl;
    }

}

