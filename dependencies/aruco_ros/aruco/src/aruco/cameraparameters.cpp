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
#include <aruco/cameraparameters.h>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;
namespace aruco
{


CameraParameters::CameraParameters() {
    CameraMatrix=cv::Mat();
    Distorsion=cv::Mat();
    CamSize=cv::Size(-1,-1);
}
/**Creates the object from the info passed
 * @param cameraMatrix 3x3 matrix (fx 0 cx, 0 fy cy, 0 0 1)
 * @param distorsionCoeff 4x1 matrix (k1,k2,p1,p2)
 * @param size image size
 */
CameraParameters::CameraParameters(cv::Mat cameraMatrix,cv::Mat distorsionCoeff,cv::Size size) throw(cv::Exception) {
    setParams(cameraMatrix,distorsionCoeff,size);
}
/**
 */
CameraParameters::CameraParameters(const CameraParameters &CI) {
    CI.CameraMatrix.copyTo(CameraMatrix);
    CI.Distorsion.copyTo(Distorsion);
    CamSize=CI.CamSize;
}

/**
*/
CameraParameters & CameraParameters::operator=(const CameraParameters &CI) {
    CI.CameraMatrix.copyTo(CameraMatrix);
    CI.Distorsion.copyTo(Distorsion);
    CamSize=CI.CamSize;
    return *this;
}
/**
 */
void CameraParameters::setParams(cv::Mat cameraMatrix,cv::Mat distorsionCoeff,cv::Size size) throw(cv::Exception)
{
    if (cameraMatrix.rows!=3 || cameraMatrix.cols!=3)
        throw cv::Exception(9000,"invalid input cameraMatrix","CameraParameters::setParams",__FILE__,__LINE__);
    cameraMatrix.convertTo(CameraMatrix,CV_32FC1);
    if (  distorsionCoeff.total()<4 ||  distorsionCoeff.total()>=7 )
        throw cv::Exception(9000,"invalid input distorsionCoeff","CameraParameters::setParams",__FILE__,__LINE__);
    cv::Mat auxD;

    distorsionCoeff.convertTo( Distorsion,CV_32FC1);

//     Distorsion.create(1,4,CV_32FC1);
//     for (int i=0;i<4;i++)
//         Distorsion.ptr<float>(0)[i]=auxD.ptr<float>(0)[i];

    CamSize=size;

}

/**
*/
cv::Point3f CameraParameters::getCameraLocation(cv::Mat Rvec,cv::Mat Tvec)
{
    cv::Mat m33(3,3,CV_32FC1);
    cv::Rodrigues(Rvec, m33)  ;

    cv::Mat m44=cv::Mat::eye(4,4,CV_32FC1);
    for (int i=0;i<3;i++)
        for (int j=0;j<3;j++)
            m44.at<float>(i,j)=m33.at<float>(i,j);

    //now, add translation information
    for (int i=0;i<3;i++)
        m44.at<float>(i,3)=Tvec.at<float>(0,i);
    //invert the matrix
    m44.inv();
    return  cv::Point3f( m44.at<float>(0,0),m44.at<float>(0,1),m44.at<float>(0,2));

}

/**Reads the camera parameters from file
 */
void CameraParameters::readFromFile(string path)throw(cv::Exception)
{

    ifstream file(path.c_str());
    if (!file)  throw cv::Exception(9005,"could not open file:"+path,"CameraParameters::readFromFile",__FILE__,__LINE__);
//Create the matrices
    Distorsion=cv::Mat::zeros(4,1,CV_32FC1);
    CameraMatrix=cv::Mat::eye(3,3,CV_32FC1);
    char line[1024];
    while (!file.eof()) {
        file.getline(line,1024);
        char cmd[20];
        float fval;
        if ( sscanf(line,"%s = %f",cmd,&fval)==2) {
            string scmd(cmd);
            if (scmd=="fx") CameraMatrix.at<float>(0,0)=fval;
            else if (scmd=="cx") CameraMatrix.at<float>(0,2)=fval;
            else if (scmd=="fy") CameraMatrix.at<float>(1,1)=fval;
            else if (scmd=="cy") CameraMatrix.at<float>(1,2)=fval;
            else if (scmd=="k1") Distorsion.at<float>(0,0)=fval;
            else if (scmd=="k2") Distorsion.at<float>(1,0)=fval;
            else if (scmd=="p1") Distorsion.at<float>(2,0)=fval;
            else if (scmd=="p2") Distorsion.at<float>(3,0)=fval;
            else if (scmd=="width") CamSize.width=fval;
            else if (scmd=="height") CamSize.height=fval;
        }
    }
}
/**Saves this to a file
  */
void CameraParameters::saveToFile(string path,bool inXML)throw(cv::Exception)
{
    if (!isValid())  throw cv::Exception(9006,"invalid object","CameraParameters::saveToFile",__FILE__,__LINE__);
    if (!inXML) {
        ofstream file(path.c_str());
        if (!file)  throw cv::Exception(9006,"could not open file:"+path,"CameraParameters::saveToFile",__FILE__,__LINE__);
        file<<"# Aruco 1.0 CameraParameters"<<endl;
        file<<"fx = "<<CameraMatrix.at<float>(0,0)<<endl;
        file<<"cx = "<<CameraMatrix.at<float>(0,2)<<endl;
        file<<"fy = "<<CameraMatrix.at<float>(1,1)<<endl;
        file<<"cy = "<<CameraMatrix.at<float>(1,2)<<endl;
        file<<"k1 = "<<Distorsion.at<float>(0,0)<<endl;
        file<<"k2 = "<<Distorsion.at<float>(1,0)<<endl;
        file<<"p1 = "<<Distorsion.at<float>(2,0)<<endl;
        file<<"p2 = "<<Distorsion.at<float>(3,0)<<endl;
        file<<"width = "<<CamSize.width<<endl;
        file<<"height = "<<CamSize.height<<endl;
    }
    else {
        cv::FileStorage fs(path,cv::FileStorage::WRITE);
        fs<<"image_width" << CamSize.width;
        fs<<"image_height" << CamSize.height;
        fs<<"camera_matrix" << CameraMatrix;
        fs<<"distortion_coefficients" <<Distorsion;
    }
}

/**Adjust the parameters to the size of the image indicated
 */
void CameraParameters::resize(cv::Size size)throw(cv::Exception)
{
    if (!isValid())  throw cv::Exception(9007,"invalid object","CameraParameters::resize",__FILE__,__LINE__);
    if (size==CamSize) return;
    //now, read the camera size
    //resize the camera parameters to fit this image size
    float AxFactor= float(size.width)/ float(CamSize.width);
    float AyFactor= float(size.height)/ float(CamSize.height);
    CameraMatrix.at<float>(0,0)*=AxFactor;
    CameraMatrix.at<float>(0,2)*=AxFactor;
    CameraMatrix.at<float>(1,1)*=AyFactor;
    CameraMatrix.at<float>(1,2)*=AyFactor;
}

/****
 *
 *
 *
 *
 */
void CameraParameters::readFromXMLFile(string filePath)throw(cv::Exception)
{
    cv::FileStorage fs(filePath, cv::FileStorage::READ);
    int w=-1,h=-1;
    cv::Mat MCamera,MDist;

    fs["image_width"] >> w;
    fs["image_height"] >> h;
    fs["distortion_coefficients"] >> MDist;
    fs["camera_matrix"] >> MCamera;

    if (MCamera.cols==0 || MCamera.rows==0)throw cv::Exception(9007,"File :"+filePath+" does not contains valid camera matrix","CameraParameters::readFromXML",__FILE__,__LINE__);
    if (w==-1 || h==0)throw cv::Exception(9007,"File :"+filePath+" does not contains valid camera dimensions","CameraParameters::readFromXML",__FILE__,__LINE__);

    if (MCamera.type()!=CV_32FC1) MCamera.convertTo(CameraMatrix,CV_32FC1);
    else CameraMatrix=MCamera;

    if (MDist.total()<4) throw cv::Exception(9007,"File :"+filePath+" does not contains valid distortion_coefficients","CameraParameters::readFromXML",__FILE__,__LINE__);
    //convert to 32 and get the 4 first elements only
    cv::Mat mdist32;
    MDist.convertTo(mdist32,CV_32FC1);
    Distorsion.create(1,4,CV_32FC1);
    for (int i=0;i<4;i++)
        Distorsion.ptr<float>(0)[i]=mdist32.ptr<float>(0)[i];

    CamSize.width=w;
    CamSize.height=h;
}
/****
 *
 */
void CameraParameters::glGetProjectionMatrix( cv::Size orgImgSize, cv::Size size,double proj_matrix[16],double gnear,double gfar,bool invert   )throw(cv::Exception)
{

    if (cv::countNonZero(Distorsion)!=0) std::cerr<< "CameraParameters::glGetProjectionMatrix :: The camera has distortion coefficients " <<__FILE__<<" "<<__LINE__<<endl;
    if (isValid()==false) throw cv::Exception(9100,"invalid camera parameters","CameraParameters::glGetProjectionMatrix",__FILE__,__LINE__);

    //Deterime the rsized info
    double Ax=double(size.width)/double(orgImgSize.width);
    double Ay=double(size.height)/double(orgImgSize.height);
    double _fx=CameraMatrix.at<float>(0,0)*Ax;
    double _cx=CameraMatrix.at<float>(0,2)*Ax;
    double _fy=CameraMatrix.at<float>(1,1)*Ay;
    double _cy=CameraMatrix.at<float>(1,2)*Ay;
    double cparam[3][4] =
    {
        {
            _fx,  0,  _cx,  0
        },
        {0,          _fy,  _cy, 0},
        {0,      0,      1,      0}
    };

    argConvGLcpara2( cparam, size.width, size.height, gnear, gfar, proj_matrix, invert );

}

/*******************
 *
 *
 *******************/
double CameraParameters::norm( double a, double b, double c )
{
    return( sqrt( a*a + b*b + c*c ) );
}

/*******************
 *
 *
 *******************/

double CameraParameters::dot( double a1, double a2, double a3,
                              double b1, double b2, double b3 )
{
    return( a1 * b1 + a2 * b2 + a3 * b3 );
}

/*******************
 *
 *
 *******************/

void CameraParameters::argConvGLcpara2( double cparam[3][4], int width, int height, double gnear, double gfar, double m[16], bool invert )throw(cv::Exception)
{

    double   icpara[3][4];
    double   trans[3][4];
    double   p[3][3], q[4][4];
    int      i, j;

    cparam[0][2] *= -1.0;
    cparam[1][2] *= -1.0;
    cparam[2][2] *= -1.0;

    if ( arParamDecompMat(cparam, icpara, trans) < 0 )
        throw cv::Exception(9002,"parameter error","MarkerDetector::argConvGLcpara2",__FILE__,__LINE__);

    for ( i = 0; i < 3; i++ )
    {
        for ( j = 0; j < 3; j++ )
        {
            p[i][j] = icpara[i][j] / icpara[2][2];
        }
    }
    q[0][0] = (2.0 * p[0][0] / width);
    q[0][1] = (2.0 * p[0][1] / width);
    q[0][2] = ((2.0 * p[0][2] / width)  - 1.0);
    q[0][3] = 0.0;

    q[1][0] = 0.0;
    q[1][1] = (2.0 * p[1][1] / height);
    q[1][2] = ((2.0 * p[1][2] / height) - 1.0);
    q[1][3] = 0.0;

    q[2][0] = 0.0;
    q[2][1] = 0.0;
    q[2][2] = (gfar + gnear)/(gfar - gnear);
    q[2][3] = -2.0 * gfar * gnear / (gfar - gnear);

    q[3][0] = 0.0;
    q[3][1] = 0.0;
    q[3][2] = 1.0;
    q[3][3] = 0.0;

    for ( i = 0; i < 4; i++ )
    {
        for ( j = 0; j < 3; j++ )
        {
            m[i+j*4] = q[i][0] * trans[0][j]
                       + q[i][1] * trans[1][j]
                       + q[i][2] * trans[2][j];
        }
        m[i+3*4] = q[i][0] * trans[0][3]
                   + q[i][1] * trans[1][3]
                   + q[i][2] * trans[2][3]
                   + q[i][3];
    }

    if (!invert)
    {
        m[13]=-m[13] ;
        m[1]=-m[1];
        m[5]=-m[5];
        m[9]=-m[9];
    }

}
/*******************
 *
 *
 *******************/

int CameraParameters::arParamDecompMat( double source[3][4], double cpara[3][4], double trans[3][4] )throw(cv::Exception)
{
    int       r, c;
    double    Cpara[3][4];
    double    rem1, rem2, rem3;

    if ( source[2][3] >= 0 )
    {
        for ( r = 0; r < 3; r++ )
        {
            for ( c = 0; c < 4; c++ )
            {
                Cpara[r][c] = source[r][c];
            }
        }
    }
    else
    {
        for ( r = 0; r < 3; r++ )
        {
            for ( c = 0; c < 4; c++ )
            {
                Cpara[r][c] = -(source[r][c]);
            }
        }
    }

    for ( r = 0; r < 3; r++ )
    {
        for ( c = 0; c < 4; c++ )
        {
            cpara[r][c] = 0.0;
        }
    }
    cpara[2][2] = norm( Cpara[2][0], Cpara[2][1], Cpara[2][2] );
    trans[2][0] = Cpara[2][0] / cpara[2][2];
    trans[2][1] = Cpara[2][1] / cpara[2][2];
    trans[2][2] = Cpara[2][2] / cpara[2][2];
    trans[2][3] = Cpara[2][3] / cpara[2][2];

    cpara[1][2] = dot( trans[2][0], trans[2][1], trans[2][2],
                       Cpara[1][0], Cpara[1][1], Cpara[1][2] );
    rem1 = Cpara[1][0] - cpara[1][2] * trans[2][0];
    rem2 = Cpara[1][1] - cpara[1][2] * trans[2][1];
    rem3 = Cpara[1][2] - cpara[1][2] * trans[2][2];
    cpara[1][1] = norm( rem1, rem2, rem3 );
    trans[1][0] = rem1 / cpara[1][1];
    trans[1][1] = rem2 / cpara[1][1];
    trans[1][2] = rem3 / cpara[1][1];

    cpara[0][2] = dot( trans[2][0], trans[2][1], trans[2][2],
                       Cpara[0][0], Cpara[0][1], Cpara[0][2] );
    cpara[0][1] = dot( trans[1][0], trans[1][1], trans[1][2],
                       Cpara[0][0], Cpara[0][1], Cpara[0][2] );
    rem1 = Cpara[0][0] - cpara[0][1]*trans[1][0] - cpara[0][2]*trans[2][0];
    rem2 = Cpara[0][1] - cpara[0][1]*trans[1][1] - cpara[0][2]*trans[2][1];
    rem3 = Cpara[0][2] - cpara[0][1]*trans[1][2] - cpara[0][2]*trans[2][2];
    cpara[0][0] = norm( rem1, rem2, rem3 );
    trans[0][0] = rem1 / cpara[0][0];
    trans[0][1] = rem2 / cpara[0][0];
    trans[0][2] = rem3 / cpara[0][0];

    trans[1][3] = (Cpara[1][3] - cpara[1][2]*trans[2][3]) / cpara[1][1];
    trans[0][3] = (Cpara[0][3] - cpara[0][1]*trans[1][3]
                   - cpara[0][2]*trans[2][3]) / cpara[0][0];

    for ( r = 0; r < 3; r++ )
    {
        for ( c = 0; c < 3; c++ )
        {
            cpara[r][c] /= cpara[2][2];
        }
    }

    return 0;
}


/******
 *
 */
void CameraParameters::OgreGetProjectionMatrix(cv::Size orgImgSize, cv::Size size, double proj_matrix[16], double gnear, double gfar, bool invert) throw(cv::Exception)
{
    double temp_matrix[16];
    (*this).glGetProjectionMatrix(orgImgSize, size, temp_matrix, gnear, gfar, invert);
    proj_matrix[0]=-temp_matrix[0];
    proj_matrix[1]=-temp_matrix[4];
    proj_matrix[2]=-temp_matrix[8];
    proj_matrix[3]=temp_matrix[12];

    proj_matrix[4]=-temp_matrix[1];
    proj_matrix[5]=-temp_matrix[5];
    proj_matrix[6]=-temp_matrix[9];
    proj_matrix[7]=temp_matrix[13];

    proj_matrix[8]=-temp_matrix[2];
    proj_matrix[9]=-temp_matrix[6];
    proj_matrix[10]=-temp_matrix[10];
    proj_matrix[11]=temp_matrix[14];

    proj_matrix[12]=-temp_matrix[3];
    proj_matrix[13]=-temp_matrix[7];
    proj_matrix[14]=-temp_matrix[11];
    proj_matrix[15]=temp_matrix[15];
}




};
