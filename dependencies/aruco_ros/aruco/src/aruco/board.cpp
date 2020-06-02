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
#include <aruco/board.h>
#include <fstream>
using namespace std;
using namespace cv;
namespace aruco
{

/**
*
*
*/
BoardConfiguration::BoardConfiguration()
{
    mInfoType=NONE;
}
/**
*
*
*/
BoardConfiguration::BoardConfiguration ( const BoardConfiguration  &T ): vector<MarkerInfo>(T)
{
//     MarkersInfo=T.MarkersInfo;
    mInfoType=T.mInfoType;
}

/**
*
*
*/
BoardConfiguration & BoardConfiguration ::operator=(const BoardConfiguration  &T) {
//     MarkersInfo=T.MarkersInfo;
    vector<MarkerInfo>::operator=(T);
    mInfoType=T.mInfoType;
    return *this;
}
/**
*
*
*/
void BoardConfiguration::saveToFile ( string sfile ) throw ( cv::Exception )
{

    cv::FileStorage fs ( sfile,cv::FileStorage::WRITE );
    saveToFile(fs);

}
/**Saves the board info to a file
*/
void BoardConfiguration::saveToFile(cv::FileStorage &fs)throw (cv::Exception) {
    fs<<"aruco_bc_nmarkers"<< ( int ) size();
    fs<<"aruco_bc_mInfoType"<< ( int ) mInfoType;
    fs<<"aruco_bc_markers"<<"[";
    for ( size_t i=0;i<size();i++ )
    {
        fs << "{:" << "id" << at(i).id ;

        fs<<"corners"<< "[:";
        for (size_t c=0;c<at(i).size();++c)
            fs<<at(i)[c];
        fs<<"]";
        fs <<  "}";
    }
    fs << "]";
}

/**
*
*
*/
void BoardConfiguration::readFromFile ( string sfile ) throw ( cv::Exception )
{
    cv::FileStorage fs ( sfile,cv::FileStorage::READ );
    readFromFile(fs);

}


/**Reads board info from a file
*/
void BoardConfiguration::readFromFile(cv::FileStorage &fs)throw (cv::Exception)
{
    int aux=0;
    //look for the nmarkers
    if ( fs["aruco_bc_nmarkers"].name() !="aruco_bc_nmarkers" )
        throw cv::Exception ( 81818,"BoardConfiguration::readFromFile","invalid file type" ,__FILE__,__LINE__ );
    fs["aruco_bc_nmarkers"]>>aux;
    resize ( aux );
    fs["aruco_bc_mInfoType"]>>mInfoType;
    cv::FileNode markers=fs["aruco_bc_markers"];
    int i=0;
    for (FileNodeIterator it = markers.begin();it!=markers.end();++it,i++) {
        at(i).id=(*it)["id"];
        FileNode FnCorners=(*it)["corners"];
        for (FileNodeIterator itc = FnCorners.begin();itc!=FnCorners.end();++itc) {
            vector<float> coordinates3d;
            (*itc)>>coordinates3d;
            if(coordinates3d.size()!=3)
	       throw cv::Exception ( 81818,"BoardConfiguration::readFromFile","invalid file type 3" ,__FILE__,__LINE__ );
	    cv::Point3f point(coordinates3d[0],coordinates3d[1],coordinates3d[2]);
            at(i).push_back(point);
        }
    }
}

/**
 */
int BoardConfiguration::getIndexOfMarkerId(int id)const
{
 
 for(size_t i=0;i<size();i++)
   if( at(i).id==id)return i;
 return -1;   
}

/**
 */
const MarkerInfo& BoardConfiguration::getMarkerInfo(int id)const throw (cv::Exception)
{
 for(size_t i=0;i<size();i++)
   if( at(i).id ==id) return at(i);
 throw cv::Exception(111,"BoardConfiguration::getMarkerInfo","Marker with the id given is not found",__FILE__,__LINE__);
  
}


/**
 */
void Board::glGetModelViewMatrix ( double modelview_matrix[16] ) throw ( cv::Exception )
{
    //check if paremeters are valid
    bool invalid=false;
    for ( int i=0;i<3 && !invalid ;i++ )
    {
      invalid |= std::isnan(Tvec.at<float>(i,0));
      invalid |= std::isnan(Rvec.at<float>(i,0));
    }
    if ( invalid ) throw cv::Exception ( 9002,"extrinsic parameters are not set","Marker::getModelViewMatrix",__FILE__,__LINE__ );
    Mat Rot ( 3,3,CV_32FC1 ),Jacob;
    Rodrigues ( Rvec, Rot, Jacob );

    double para[3][4];
    for ( int i=0;i<3;i++ )
        for ( int j=0;j<3;j++ ) para[i][j]=Rot.at<float> ( i,j );
    //now, add the translation
    para[0][3]=Tvec.at<float> ( 0,0 );
    para[1][3]=Tvec.at<float> ( 1,0 );
    para[2][3]=Tvec.at<float> ( 2,0 );
    double scale=1;

    modelview_matrix[0 + 0*4] = para[0][0];
    // R1C2
    modelview_matrix[0 + 1*4] = para[0][1];
    modelview_matrix[0 + 2*4] = para[0][2];
    modelview_matrix[0 + 3*4] = para[0][3];
    // R2
    modelview_matrix[1 + 0*4] = para[1][0];
    modelview_matrix[1 + 1*4] = para[1][1];
    modelview_matrix[1 + 2*4] = para[1][2];
    modelview_matrix[1 + 3*4] = para[1][3];
    // R3
    modelview_matrix[2 + 0*4] = -para[2][0];
    modelview_matrix[2 + 1*4] = -para[2][1];
    modelview_matrix[2 + 2*4] = -para[2][2];
    modelview_matrix[2 + 3*4] = -para[2][3];
    modelview_matrix[3 + 0*4] = 0.0;
    modelview_matrix[3 + 1*4] = 0.0;
    modelview_matrix[3 + 2*4] = 0.0;
    modelview_matrix[3 + 3*4] = 1.0;
    if ( scale > 0.0 )
    {
        modelview_matrix[12] *= scale;
        modelview_matrix[13] *= scale;
        modelview_matrix[14] *= scale;
    }


}


/****
 *
 */
void Board::OgreGetPoseParameters ( double position[3], double orientation[4] ) throw ( cv::Exception )
{
    //check if paremeters are valid
    bool invalid=false;
    for ( int i=0;i<3 && !invalid ;i++ )
    {
      invalid |= std::isnan(Tvec.at<float>(i,0));
      invalid |= std::isnan(Rvec.at<float>(i,0));
    }
    if ( invalid ) throw cv::Exception ( 9003,"extrinsic parameters are not set","Marker::getModelViewMatrix",__FILE__,__LINE__ );

    // calculate position vector
    position[0] = -Tvec.ptr<float> ( 0 ) [0];
    position[1] = -Tvec.ptr<float> ( 0 ) [1];
    position[2] = +Tvec.ptr<float> ( 0 ) [2];

    // now calculare orientation quaternion
    cv::Mat Rot ( 3,3,CV_32FC1 );
    cv::Rodrigues ( Rvec, Rot );

    // calculate axes for quaternion
    double stAxes[3][3];
    // x axis
    stAxes[0][0] = -Rot.at<float> ( 0,0 );
    stAxes[0][1] = -Rot.at<float> ( 1,0 );
    stAxes[0][2] = +Rot.at<float> ( 2,0 );
    // y axis
    stAxes[1][0] = -Rot.at<float> ( 0,1 );
    stAxes[1][1] = -Rot.at<float> ( 1,1 );
    stAxes[1][2] = +Rot.at<float> ( 2,1 );
    // for z axis, we use cross product
    stAxes[2][0] = stAxes[0][1]*stAxes[1][2] - stAxes[0][2]*stAxes[1][1];
    stAxes[2][1] = - stAxes[0][0]*stAxes[1][2] + stAxes[0][2]*stAxes[1][0];
    stAxes[2][2] = stAxes[0][0]*stAxes[1][1] - stAxes[0][1]*stAxes[1][0];

    // transposed matrix
    double axes[3][3];
    axes[0][0] = stAxes[0][0];
    axes[1][0] = stAxes[0][1];
    axes[2][0] = stAxes[0][2];

    axes[0][1] = stAxes[1][0];
    axes[1][1] = stAxes[1][1];
    axes[2][1] = stAxes[1][2];

    axes[0][2] = stAxes[2][0];
    axes[1][2] = stAxes[2][1];
    axes[2][2] = stAxes[2][2];

    // Algorithm in Ken Shoemake's article in 1987 SIGGRAPH course notes
    // article "Quaternion Calculus and Fast Animation".
    double fTrace = axes[0][0]+axes[1][1]+axes[2][2];
    double fRoot;

    if ( fTrace > 0.0 )
    {
        // |w| > 1/2, may as well choose w > 1/2
        fRoot = sqrt ( fTrace + 1.0 );  // 2w
        orientation[0] = 0.5*fRoot;
        fRoot = 0.5/fRoot;  // 1/(4w)
        orientation[1] = ( axes[2][1]-axes[1][2] ) *fRoot;
        orientation[2] = ( axes[0][2]-axes[2][0] ) *fRoot;
        orientation[3] = ( axes[1][0]-axes[0][1] ) *fRoot;
    }
    else
    {
        // |w| <= 1/2
        static unsigned int s_iNext[3] = { 1, 2, 0 };
        unsigned int i = 0;
        if ( axes[1][1] > axes[0][0] )
            i = 1;
        if ( axes[2][2] > axes[i][i] )
            i = 2;
        unsigned int j = s_iNext[i];
        unsigned int k = s_iNext[j];

        fRoot = sqrt ( axes[i][i]-axes[j][j]-axes[k][k] + 1.0 );
        double* apkQuat[3] = { &orientation[1], &orientation[2], &orientation[3] };
        *apkQuat[i] = 0.5*fRoot;
        fRoot = 0.5/fRoot;
        orientation[0] = ( axes[k][j]-axes[j][k] ) *fRoot;
        *apkQuat[j] = ( axes[j][i]+axes[i][j] ) *fRoot;
        *apkQuat[k] = ( axes[k][i]+axes[i][k] ) *fRoot;
    }


}


/**Save this from a file
  */
void Board::saveToFile(string filePath)throw(cv::Exception)
{
 cv::FileStorage fs ( filePath,cv::FileStorage::WRITE );
  
    fs<<"aruco_bo_rvec"<< Rvec;
    fs<<"aruco_bo_tvec"<< Tvec;
    //now, the markers
    fs<<"aruco_bo_nmarkers"<< ( int )  size();
    fs<<"aruco_bo_markers"<< "[";
    for ( size_t i=0;i<size();++i )
    {
        fs << "{:" << "id" << at(i).id ;
        fs<<"corners"<< "[:";
        for (size_t c=0;c<at(i).size();++c)
	      fs<<at(i)[c];
        fs<<"]";
        fs <<  "}";
    }
    fs << "]";
  //save configuration file
  conf.saveToFile(fs);
 
 
 
//  readFromFile(fs);

}
/**Read  this from a file
 */
void Board::readFromFile(string filePath)throw(cv::Exception)
{
 cv::FileStorage fs ( filePath,cv::FileStorage::READ );
    if ( fs["aruco_bo_nmarkers"].name() !="aruco_bo_nmarkers" )
        throw cv::Exception ( 81818,"Board::readFromFile","invalid file type:" ,__FILE__,__LINE__ );

    
    
    int aux=0;
    //look for the nmarkers
    fs["aruco_bo_nmarkers"]>>aux;
    resize ( aux );
    fs["aruco_bo_rvec"]>> Rvec;
    fs["aruco_bo_tvec"]>> Tvec;
    
    cv::FileNode markers=fs["aruco_bo_markers"];
    int i=0;
    for (FileNodeIterator it = markers.begin();it!=markers.end();++it,i++) {
        at(i).id=(*it)["id"];	
	int ncorners=(*it)["ncorners"];
	at(i).resize(ncorners);
        FileNode FnCorners=(*it)["corners"];
        int c=0;
        for (FileNodeIterator itc = FnCorners.begin();itc!=FnCorners.end();++itc,c++) {
            vector<float> coordinates2d;
            (*itc)>>coordinates2d;
	    if(coordinates2d.size()!=2) 
	      throw cv::Exception ( 81818,"Board::readFromFile","invalid file type 2" ,__FILE__,__LINE__ );
	    cv::Point2f point;
            point.x=coordinates2d[0];
            point.y=coordinates2d[1];
	    at(i).push_back(point);
        }
    }
    
 conf.readFromFile(fs);
 
 
}

    /** 
 */
void BoardConfiguration::getIdList(std::vector< int >& ids, bool append) const
{
 if (!append) ids.clear();
 for(size_t i=0;i<size();i++)
   ids.push_back(at(i).id);
}
};
