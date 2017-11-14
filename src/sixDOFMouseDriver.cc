#include <array>
#include <ctime>
#include <hpp/interactive/sixDOFMouseDriver.hh>
#include <stdlib.h>
#include <stdio.h>
//#include <libudev.h>
#include <locale.h>
#include <unistd.h>
#include <hpp/interactive/dhdc.h>
#include <hpp/interactive/drdc.h>
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <hpp/interactive/Data.hh>
#include <hpp/interactive/zmq.hh>

#include <array>
#include <limits>


boost::thread* SixDOFMouseDriver::interactiveDeviceThread_ = 0;
int SixDOFMouseDriver::fd_ = 0;
char SixDOFMouseDriver::data_[14];
short int SixDOFMouseDriver::deviceValues_[6];
double SixDOFMouseDriver::deviceValuesNormalized_[6];
se3::SE3 SixDOFMouseDriver::transformation_;
boost::mutex SixDOFMouseDriver::mutex_;
double SixDOFMouseDriver::linear_speed_;
double SixDOFMouseDriver::angular_speed_;
double SixDOFMouseDriver::rotation_threshold_;
bool SixDOFMouseDriver::has_moved_;
bool SixDOFMouseDriver::recale_;
double SixDOFMouseDriver::limits_[6];
double SixDOFMouseDriver::K_[3];
double SixDOFMouseDriver::K_off_[3];
// x, y, z for side, front, up vectors ; in this order
float SixDOFMouseDriver::cameraVectors_[9];
short int SixDOFMouseDriver::type_;
zmq::socket_t* SixDOFMouseDriver::socket_;
zmq::context_t* SixDOFMouseDriver::context_;

Eigen::Vector3d SixDOFMouseDriver::deviceForce_;
Eigen::Vector3d SixDOFMouseDriver::deviceTorque_;
Eigen::Vector3d SixDOFMouseDriver::contact_normal_;
Eigen::Vector3d SixDOFMouseDriver::user_force_;
bool SixDOFMouseDriver::in_contact_;
using namespace std;

typedef std::array<float, 3> float3;
typedef std::array<float3, 3> float3x3;

const float PI = 3.14159265358979323846264f;

bool closeEnough(const float& a, const float& b, const float& epsilon = std::numeric_limits<float>::epsilon()) {
    return (epsilon > std::abs(a - b));
}

float3 eulerAngles(const float3x3& R) {

    //check for gimbal lock
    if (closeEnough(R[0][2], -1.0f)) {
        float x = 0; //gimbal lock, value of x doesn't matter
        float y = PI / 2;
        float z = x + atan2(R[1][0], R[2][0]);
        return { x, y, z };
    } else if (closeEnough(R[0][2], 1.0f)) {
        float x = 0;
        float y = -PI / 2;
        float z = -x + atan2(-R[1][0], -R[2][0]);
        return { x, y, z };
    } else { //two solutions exist
        float x1 = -asin(R[0][2]);
        float x2 = PI - x1;

        float y1 = atan2(R[1][2] / cos(x1), R[2][2] / cos(x1));
        float y2 = atan2(R[1][2] / cos(x2), R[2][2] / cos(x2));

        float z1 = atan2(R[0][1] / cos(x1), R[0][0] / cos(x1));
        float z2 = atan2(R[0][1] / cos(x2), R[0][0] / cos(x2));

        //choose one solution to return
        //for example the "shortest" rotation
        if ((std::abs(x1) + std::abs(y1) + std::abs(z1)) <= (std::abs(x2) + std::abs(y2) + std::abs(z2))) {
            return { x1, y1, z1 };
        } else {
            return { x2, y2, z2 };
        }
    }
}



Eigen::Matrix3f quat2Mat(float x, float y, float z, float w){
  ::Eigen::Matrix3f ret;

  ret(0, 0) = 1 - 2*(float)(pow(y, 2) + pow(z, 2));
  ret(0, 1) = 2*x*y - 2*z*w;
  ret(0, 2) = 2*x*z + 2*y*w;
  ret(1, 0) = 2*x*y + 2*z*w;
  ret(1, 1) = 1 - 2*(float)(pow(x, 2) + pow(z, 2));
  ret(1, 2) = 2*y*z - 2*x*w;
  ret(2, 0) = 2*x*z - 2*y*w;
  ret(2, 1) = 2*y*z + 2*x*w;
  ret(2, 2) = 1 - 2*(float)(pow(x, 2) + pow(y, 2));

  return ret;
}

void normalizeQuat(double& w, double& x, double& y, double& z){
  double mag = sqrt(pow(w,2)+pow(x,2)+pow(y,2)+pow(z,2));
  w = w/mag; x = x/mag; y = y/mag; z = z/mag;
}
static void toEulerianAngle(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw)
{
  double ysqr = q.y() * q.y();

  // roll (x-axis rotation)
  double t0 = +2.0 * (q.w() * q.x() + q.y() * q.z());
  double t1 = +1.0 - 2.0 * (q.x() * q.x() + ysqr);
  roll = std::atan2(t0, t1);

  // pitch (y-axis rotation)
  double t2 = +2.0 * (q.w() * q.y() - q.z() * q.x());
  t2 = t2 > 1.0 ? 1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  pitch = std::asin(t2);

  // yaw (z-axis rotation)
  double t3 = +2.0 * (q.w() * q.z() + q.x() * q.y());
  double t4 = +1.0 - 2.0 * (ysqr + q.z() * q.z());  
  yaw = std::atan2(t3, t4);
}
static Eigen::Quaterniond toQuaternion(double pitch, double roll, double yaw)
{
  Eigen::Quaterniond q;
  double t0 = std::cos(yaw * 0.5);
  double t1 = std::sin(yaw * 0.5);
  double t2 = std::cos(roll * 0.5);
  double t3 = std::sin(roll * 0.5);
  double t4 = std::cos(pitch * 0.5);
  double t5 = std::sin(pitch * 0.5);

  q.w() = t0 * t2 * t4 + t1 * t3 * t5;
  q.x() = t0 * t3 * t4 - t1 * t2 * t5;
  q.y() = t0 * t2 * t5 + t1 * t3 * t4;
  q.z() = t1 * t2 * t4 - t0 * t3 * t5;
  return q;
}
float3 quat2Euler(float q0, float q1, float q2, float q3)
{
  return
  {
    atan2(2 * (q0*q1 + q2*q3), 1 - 2 * (q1*q1 + q2*q2)),
      asin( 2 * (q0*q2 - q3*q1)),
      atan2(2 * (q0*q3 + q1*q2), 1 - 2 * (q2*q2 + q3*q3))
  };
}

void euler2Quat(double psi, double theta, double phi, double* quat){
  psi/=2; theta/=2; phi/=2;

  quat[0] = cos(psi) * cos(theta) * cos(phi) - sin(psi) * sin(theta) * sin(phi);//w
  quat[1] = sin(psi) * sin(theta) * cos(phi) + cos(psi) * cos(theta) * cos(phi);//x
  quat[2] = sin(psi) * cos(theta) * cos(phi) + cos(psi) * sin(theta) * sin(phi);//y
  quat[3] = cos(psi) * sin(theta) * cos(phi) - sin(psi) * cos(theta) * sin(phi);//z
}
  
SixDOFMouseDriver::SixDOFMouseDriver(){

}

const Eigen::Vector3d& SixDOFMouseDriver::getContactNormal(){
  return SixDOFMouseDriver::contact_normal_;
}
const Eigen::Vector3d& SixDOFMouseDriver::getUserForce(){
  return SixDOFMouseDriver::user_force_;
}
bool SixDOFMouseDriver::userInContact(){
  return in_contact_;
}

const se3::SE3& SixDOFMouseDriver::getTransformation(){
    mutex_.lock(); // TODO deux fois le mutex dans le code !
    const se3::SE3& trans = SixDOFMouseDriver::transformation_;
	mutex_.unlock();
	return trans;
}
const se3::SE3& SixDOFMouseDriver::getTransformationNoMutex(){
	return SixDOFMouseDriver::transformation_;
}

void SixDOFMouseDriver::setForceAndTorque(Eigen::Vector3d force, Eigen::Vector3d torque){
  SixDOFMouseDriver::deviceForce_=force;
  SixDOFMouseDriver::deviceTorque_=torque;
}

void SixDOFMouseDriver::setCameraVectors(float a1, float a2, float a3,
                             float b1, float b2, float b3,
                             float c1, float c2, float c3){

    /*
    cout << "in setcameravect " << a1 << " " << a2 << " " << a3 <<
            " " << b1 << " " << b2 << " " << b3 << " " <<
            c1 << " " << c2 << " " << c3 << endl;
    //*/

    SixDOFMouseDriver::cameraVectors_[0] = a1;
    SixDOFMouseDriver::cameraVectors_[1] = a2;
    SixDOFMouseDriver::cameraVectors_[2] = a3;
    SixDOFMouseDriver::cameraVectors_[3] = b1;
    SixDOFMouseDriver::cameraVectors_[4] = b2;
    SixDOFMouseDriver::cameraVectors_[5] = b3;
    SixDOFMouseDriver::cameraVectors_[6] = c1;
    SixDOFMouseDriver::cameraVectors_[7] = c2;
    SixDOFMouseDriver::cameraVectors_[8] = c3;
}

void SixDOFMouseDriver::setLinearSpeed(double sp){
	SixDOFMouseDriver::linear_speed_ = sp;
}

void SixDOFMouseDriver::setAngularSpeed(double sp){
	SixDOFMouseDriver::angular_speed_ = sp;
}

void SixDOFMouseDriver::setRotationThreshold(double th){
	SixDOFMouseDriver::rotation_threshold_ = th;
}


inline void expMap (const se3::SE3::Vector3 & omega, se3::SE3::Matrix3 & R){
	double theta = omega.norm ();
    se3::SE3::Vector3 u ((omega / (float)theta));

	double s0, c0;
	s0 = sin(theta);
	c0 = cos(theta);

	double v0 = 1 - c0;

	R(0,0) = (float)(u(0) * u(0) * v0 + c0);
	R(0,1) = (float)(u(0) * u(1) * v0 - u(2) * s0);
	R(0,2) = (float)(u(0) * u(2) * v0 + u(1) * s0);

	R(1,0) = (float)(u(0) * u(1) * v0 + u(2) * s0);
	R(1,1) = (float)(u(1) * u(1) * v0 + c0);
	R(1,2) = (float)(u(1) * u(2) * v0 - u(0) * s0);

	R(2,0) = (float)(u(0) * u(2) * v0 - u(1) * s0);
	R(2,1) = (float)(u(1) * u(2) * v0 + u(0) * s0);
	R(2,2) = (float)(u(2) * u(2) * v0 + c0);
}


inline se3::SE3::Matrix3 operatorHat(const se3::SE3::Vector3& v){
  se3::SE3::Matrix3 m;

  m <<  0   ,   -v[2],    v[1],
        v[2],   0,       -v[0],
        -v[1],  v[0],     0;

  return m;
}

void SixDOFMouseDriver::InitPosition(double* translation)
{

    transformation_.translation()[0] = (float)translation[0];
    transformation_.translation()[1] = (float)translation[1];
    transformation_.translation()[2] = (float)translation[2];
}

void SixDOFMouseDriver::InitRotation(double* ro)
{
  Eigen::Quaternionf qq((float)ro[0], (float)ro[1], (float)ro[2], (float)ro[3]);
  qq.normalize();
  Eigen::Matrix3f temp =quat2Mat(qq.x(),qq.y(),qq.z(),qq.w());
  transformation_.rotation(temp);
}


void SixDOFMouseDriver::MouseInit(short int type, const double* bounds)
{
  type_=type;
  if (type_==1)
  {
    fd_ = open("/dev/hidraw0", O_RDONLY);
    if (fd_ < 0) {
      perror("Unable to open interactive device");
      abort();
    }
    else {
      std::cout << "device file descriptor : " << fd_ << "\n";
    }
    // init status
    SixDOFMouseDriver::has_moved_ = false;
    // init position
    //*
    transformation_.translation()[0] = 0;
    transformation_.translation()[1] = 0;
    transformation_.translation()[2] = 1;
    //*/
    // init rotation
    transformation_.rotation().setIdentity();
    // init speed
    SixDOFMouseDriver::linear_speed_ = 20;
    SixDOFMouseDriver::angular_speed_ = 40;
    // init axes
    SixDOFMouseDriver::cameraVectors_[0] = 1;
    SixDOFMouseDriver::cameraVectors_[1] = 0;
    SixDOFMouseDriver::cameraVectors_[2] = 0;
    SixDOFMouseDriver::cameraVectors_[3] = 0;
    SixDOFMouseDriver::cameraVectors_[4] = 1;
    SixDOFMouseDriver::cameraVectors_[5] = 0;
    SixDOFMouseDriver::cameraVectors_[6] = 0;
    SixDOFMouseDriver::cameraVectors_[7] = 0;
    SixDOFMouseDriver::cameraVectors_[8] = 1;
    // execute thread 
    //void* arg = 0;
    SixDOFMouseDriver::interactiveDeviceThread_ = 
      new boost::thread(boost::thread(SixDOFMouseDriver::ReadMouse, bounds));
  }
  if (type_==2)
  {
    int major, minor, release, revision;
    dhdGetSDKVersion (&major, &minor, &release, &revision);
    printf ("Force Dimension - Force Sensor Emulation %d.%d.%d.%d\n", major, minor, release, revision);
    printf ("(C) 2001-2015 Force Dimension\n");
    printf ("All Rights Reserved.\n\n");
    // open the first available device
    if (drdOpen () < 0) {
      printf ("error: cannot open device (%s)\n", dhdErrorGetLastStr ());
      dhdSleep (2.0);
    }
    else
    {
      // print out device identifier
      if (!drdIsSupported()) {
        printf ("unsupported device\n");
        printf ("exiting...\n");
        dhdSleep (2.0);
        drdClose (); 
      }
      printf ("%s haptic device detected\n\n", dhdGetSystemName());
      // initialize if necessary
      //if (!drdIsInitialized() && (drdAutoInit() < 0)) {
        //printf ("error: initialization failed (%s)\n", dhdErrorGetLastStr ());
        //dhdSleep (2.0);
      //}
      dhdEnableForce (DHD_ON);
      sleep(3);
      //long int iteration = 0;
      // init status
      SixDOFMouseDriver::has_moved_ = false;
      // init position
      //*
      transformation_.translation()[0] = 0;
      transformation_.translation()[1] = 0;
      transformation_.translation()[2] = 0;
      //*/
      // init rotation
      transformation_.rotation().setIdentity();
      // init speed
      SixDOFMouseDriver::linear_speed_ = 1;
      SixDOFMouseDriver::angular_speed_ = 500;
      // init axes
      SixDOFMouseDriver::cameraVectors_[0] = 1;
      SixDOFMouseDriver::cameraVectors_[1] = 0;
      SixDOFMouseDriver::cameraVectors_[2] = 0;
      SixDOFMouseDriver::cameraVectors_[3] = 0;
      SixDOFMouseDriver::cameraVectors_[4] = 1;
      SixDOFMouseDriver::cameraVectors_[5] = 0;
      SixDOFMouseDriver::cameraVectors_[6] = 0;
      SixDOFMouseDriver::cameraVectors_[7] = 0;
      SixDOFMouseDriver::cameraVectors_[8] = 1;
      limits_[0]=-0.1318;
      limits_[1]=0.06213;
      limits_[2]=-0.0724;
      limits_[3]=0.0346;
      limits_[4]=-0.125;
      limits_[5]=0.1448;
      cout << "bounds ";
      for (int i=0; i<6; i++) cout << bounds[i] << " ";
      cout << endl;
      cout << "limits ";
      for (int i=0; i<6; i++) cout << limits_[i] << " ";
      cout << endl;
      for (int i=0; i<3; i++){
        K_off_[i]=(bounds[2*i+1]*limits_[2*i]-limits_[2*i+1]*bounds[2*i])/(limits_[2*i]-limits_[2*i+1]);
        K_[i] = (bounds[2*i]-K_off_[i])/limits_[2*i];
        cout << "K/KOFF " << K_[i] << " " << K_off_[i] <<endl;
      }

      //nouvelle version
      
      for (int i=0; i<3; i++)
        K_off_[i] = 0;
      /*
         for (int i=0; i<6; i++)
         bounds_[i] = bounds[i];
         bounds_[0] = bounds[2];
         bounds_[1] = bounds[3];
      //*/

      // init bras : durée de chute 
      for (int i=0; i< 1000; i++) getData(NULL);
      // execute thread 
      //void* arg = 0;
      SixDOFMouseDriver::interactiveDeviceThread_ = 
        new boost::thread(boost::thread(SixDOFMouseDriver::ReadMouse, bounds));
    }
  }
  if (type_==3)
  {
    // init status
    SixDOFMouseDriver::has_moved_ = false;
    // init position
    //*
    transformation_.translation()[0] = 0;
    transformation_.translation()[1] = 0;
    transformation_.translation()[2] = 0;
    //*/
    // init rotation
    transformation_.rotation().setIdentity();
    // init speed
    //SixDOFMouseDriver::linear_speed_ = 20;
    //SixDOFMouseDriver::angular_speed_ = 40;
    // init axes
    SixDOFMouseDriver::cameraVectors_[0] = 1;
    SixDOFMouseDriver::cameraVectors_[1] = 0;
    SixDOFMouseDriver::cameraVectors_[2] = 0;
    SixDOFMouseDriver::cameraVectors_[3] = 0;
    SixDOFMouseDriver::cameraVectors_[4] = 1;
    SixDOFMouseDriver::cameraVectors_[5] = 0;
    SixDOFMouseDriver::cameraVectors_[6] = 0;
    SixDOFMouseDriver::cameraVectors_[7] = 0;
    SixDOFMouseDriver::cameraVectors_[8] = 1;

    //ref.recv(&request);
    //Data *recv_config = (Data *)(request.data());
    //int array_size = recv_config->getCollisionArraySize();
    //std::cout << std::setw(6) << "Array Size: " << array_size << std::endl;
    //for (int j=0; j<3; j++){
      //cout << "pos"<<j<<" "<<recv_config->getObjCooridnate().getPosition()[j] << endl;
      //cout << "force"<<j<< recv_config->getArmCF().getForceByIndex(j) << endl;
    //}
    //cout << "message reçu ex\n";

    sleep(6);
    // execute thread 
    SixDOFMouseDriver::interactiveDeviceThread_ = 
      new boost::thread(boost::thread(SixDOFMouseDriver::ReadMouse, bounds));

  }

}

// read interactice device thread function
void SixDOFMouseDriver::ReadMouse(const double* bounds_)
{
  se3::SE3::Vector3 pos, rot, axei, local, temp;
  double bounds[6];
  for (int i=0; i<6; i++) bounds[i]=bounds_[i];
  // infinite loop
  cout << "ReadDevice thread...\n";
  void* arg;

  // TODO n'ouvrir qu'en cas de type_==3
  //zmq::context_t contextpp (1);
  //zmq::socket_t socket (contextpp, ZMQ_PULL);
  //zmq_unbind (socket, "tcp://*:5555");
  //zmq_disconnect (socket, "tcp://*:5555");
  //socket.bind ("tcp://*:5555");
  /*if(0)while(1){
    cout << "while\n";
    zmq::message_t request;

    //  Wait for next request from client
    socket.recv (&request);
    Data *recv_config = (Data *)(request.data());
    // output the size of received collision array
    int array_size = recv_config->getCollisionArraySize();
    std::cout << std::setw(6) << "Array Size: " << array_size << std::endl;
    std::cout<<*(recv_config->getCollisionPairByIndex(0).getPositionOnObj1())<<" "<<*(recv_config->getCollisionPairByIndex(0).getPositionOnObj2())<<std::endl;
    for (int j=0; j<3; j++){
      cout << "pos"<<j<<" "<<recv_config->getObjCooridnate().getPosition()[j] << endl;
      //cout << "force"<<j<< recv_config->getArmCF().getForceByIndex(j) << endl;
    }   

  }//*/


  while (1){
    //cout << "ReadDevice thread...while\n";
    getData(arg);
    //mutex_.unlock();
    se3::SE3 temp_trans = transformation_;
    if (type_==1)
    {
      /////////////////////////////////////////////////////////
      // translation
      float divideFactor = (float)SixDOFMouseDriver::linear_speed_;// dt
      /*
         cout << "in the driver cam vect ";
         for (int i=0; i<9; i++)
         cout << SixDOFMouseDriver::cameraVectors_[i] << " ";
         cout << "\r";
      //*/
      //*
      for (int i=0; i<3; i++){
        pos[i] =
          (float) SixDOFMouseDriver::deviceValuesNormalized_[0]/divideFactor*SixDOFMouseDriver::cameraVectors_[i] -
          (float) SixDOFMouseDriver::deviceValuesNormalized_[1]/divideFactor*SixDOFMouseDriver::cameraVectors_[i+3] -
          (float) SixDOFMouseDriver::deviceValuesNormalized_[2]/divideFactor*SixDOFMouseDriver::cameraVectors_[i+6];
      }
      //*/
      /*
      // bounds limits ---------------- TODO : provoque un effet de bords avec les rotations
      if (pos[0]+transformation_.translation()[0]<bounds[0]){
      pos[0] = 0;
      cout << "pos[0]+transformation_.translation()[0]<bounds[0]\n";
      cout << pos[0]<< "<" << bounds[0]<< "\n";
      }
      if (pos[0]+transformation_.translation()[0]>bounds[1]){
      pos[0] = 0;
      cout << "pos[0]+transformation_.translation()[0]>bounds[1]\n";
      cout << pos[0] << ">" << bounds[1] << "\n";
      }
      if (pos[1]+transformation_.translation()[1]<bounds[2]){
      pos[1] = 0;
      cout << "pos[1]+transformation_.translation()[1]<bounds[2]\n";
      cout << pos[1] << "<" << bounds[2] << "\n";
      }
      if (pos[1]+transformation_.translation()[1]>bounds[3]){
      pos[1] = 0;
      cout << "pos[1]+transformation_.translation()[1]>bounds[3]\n";
      cout << pos[1] << ">" << bounds[3] << "\n";
      }
      if (pos[2]+transformation_.translation()[2]<bounds[4]){
      pos[2] = 0;
      cout << "pos[2]+transformation_.translation()[2]<bounds[4]\n";
      cout << pos[2] << "<" << bounds[4] << "\n";
      }
      if (pos[2]+transformation_.translation()[2]>bounds[5]){
      pos[2] = 0;
      cout << "pos[2]+transformation_.translation()[2]>bounds[5]\n";
      cout << pos[2] << ">" << bounds[5] << "\n";
      }
      // -------------------------------------------------------
      //*/
      temp_trans.translation(pos + transformation_.translation());
      

      /////////////////////////////////////////////////////////
      /*
         for (int i=0; i<3; i++){
         rot[i] =
         (float) SixDOFMouseDriver::deviceValuesNormalized_[3]*M_PI
       *SixDOFMouseDriver::cameraVectors_[i] -
       (float) SixDOFMouseDriver::deviceValuesNormalized_[4]*M_PI
       *SixDOFMouseDriver::cameraVectors_[i+3] -
       (float) SixDOFMouseDriver::deviceValuesNormalized_[5]*M_PI
       *SixDOFMouseDriver::cameraVectors_[i+6];
       }
      //cout << "rotation " << rot[0] << " " << rot[1] << " " << rot[2] << endl;
      double norme = sqrt(pow(rot[0], 2)+ pow(rot[1], 2)+pow(rot[2], 2));
      //cout << "norme " << norme << endl;
      rot[0] = rot[0]/norme;
      rot[1] = rot[1]/norme;
      rot[1] = rot[2]/norme;
      /*/
      /* // réorienter selon caméra
         Eigen::Vector3f res;
         res << SixDOFMouseDriver::deviceValuesNormalized_[3] ,
         SixDOFMouseDriver::deviceValuesNormalized_[4] ,
         SixDOFMouseDriver::deviceValuesNormalized_[5];
         cout << matrot << endl;
         res = matrot.inverse() * res;
      // res 0 tangage res1 roulis
      //cout << "rotation " << rot[0] << " " << rot[1] << " " << rot[2] << endl;
      // rotation
      //*/
      //* // sans modif, version initiale
      rot[0] = (float) (SixDOFMouseDriver::deviceValuesNormalized_[3]* M_PI);
      rot[1] = (float) (SixDOFMouseDriver::deviceValuesNormalized_[4]* M_PI);
      rot[2] = (float) (SixDOFMouseDriver::deviceValuesNormalized_[5]* M_PI);
      //cout << "ancienne meth "<< rot[0] << " " << rot[1] << " " << rot[2] << endl ;
      //*/
      /*
      // réorienter selon précédente position
      Eigen::Matrix3f matrot;
      matrot = transformation_.rotation();
      Eigen::Vector3f res;
      res << SixDOFMouseDriver::deviceValuesNormalized_[3]* M_PI ,
      SixDOFMouseDriver::deviceValuesNormalized_[4]* M_PI ,
      SixDOFMouseDriver::deviceValuesNormalized_[5]* M_PI;
      res = matrot * res;
      rot[0] = res(0);
      rot[1] = -res(1);
      rot[2] = -res(2);
      cout << "rotation " << rot[0] << " " << rot[1] << " " << rot[2] << endl << endl;
      //*/

      /////////////////////////////////////////////////////////
      // integrate rotations
      double threshold;
      if (type_==1) threshold = 0.5; // anciennement 0.5 0.2
      if (type_==2) threshold = 0.03; // anciennement 0.5 0.2
      divideFactor = (float)SixDOFMouseDriver::angular_speed_;
      // rotation variations
      se3::SE3::Vector3 v_local (0., 0., 0.);
      // threshold for rotations
      if (std::abs(rot[0]) > threshold)
        v_local[0] = (float)(rot[0]-threshold)/(float)divideFactor;
      if (std::abs(rot[1]) > threshold)
        v_local[1] = -(float)(rot[1]-threshold)/(float)divideFactor;
      if (std::abs(rot[2]) > threshold)
        v_local[2] = (float)(rot[2]-threshold)/(float)divideFactor;
      Eigen::Matrix3f matrot;

      //*
      matrot <<
        (abs(SixDOFMouseDriver::cameraVectors_[3]) < 1e-7 ? 0 : SixDOFMouseDriver::cameraVectors_[3]) ,
        (abs(SixDOFMouseDriver::cameraVectors_[4]) < 1e-7 ? 0 : SixDOFMouseDriver::cameraVectors_[4]) ,
        (abs(SixDOFMouseDriver::cameraVectors_[5]) < 1e-7 ? 0 : SixDOFMouseDriver::cameraVectors_[5]) ,
        (abs(SixDOFMouseDriver::cameraVectors_[6]) < 1e-7 ? 0 : SixDOFMouseDriver::cameraVectors_[6]) ,
        (abs(SixDOFMouseDriver::cameraVectors_[7]) < 1e-7 ? 0 : SixDOFMouseDriver::cameraVectors_[7]) ,
        (abs(SixDOFMouseDriver::cameraVectors_[8]) < 1e-7 ? 0 : SixDOFMouseDriver::cameraVectors_[8]) ,
        (abs(SixDOFMouseDriver::cameraVectors_[0]) < 1e-7 ? 0 : SixDOFMouseDriver::cameraVectors_[0]) ,
        (abs(SixDOFMouseDriver::cameraVectors_[1]) < 1e-7 ? 0 : SixDOFMouseDriver::cameraVectors_[1]) ,
        (abs(SixDOFMouseDriver::cameraVectors_[2]) < 1e-7 ? 0 : SixDOFMouseDriver::cameraVectors_[2]) ;
      //*/

      /*
         matrot << SixDOFMouseDriver::cameraVectors_[0] ,
         SixDOFMouseDriver::cameraVectors_[1] ,
         SixDOFMouseDriver::cameraVectors_[2] ,
         SixDOFMouseDriver::cameraVectors_[3] ,
         SixDOFMouseDriver::cameraVectors_[4] ,
         SixDOFMouseDriver::cameraVectors_[5] ,
         SixDOFMouseDriver::cameraVectors_[6] ,
         SixDOFMouseDriver::cameraVectors_[7] ,
         SixDOFMouseDriver::cameraVectors_[8];
      //*/
      //cout << "matrot " << matrot.transpose() << endl;
      //v_local = transformation_.rotation().transpose() * v_local; // ça marcheoie biengue
      //v_local = matrot.transpose() * v_local;

      se3::SE3::Matrix3 dR;
      // threshold
      if (v_local.norm () < 1e-8) dR.setIdentity();
      else expMap(v_local, dR);

      //cout << dR << endl << endl;

      se3::SE3::Matrix3 R_new (temp_trans.rotation () * dR);
      temp_trans.rotation(R_new);

    }
    if (type_==2)
    {
      for (int i=0; i<3; i++)pos[i]=(float)deviceValuesNormalized_[i];
      //cout << "nouv pos= " << pos[0] << " " << pos[1] << " " << pos[2] << "\r";
      for (int i=0; i<3; i++){
        pos[i] =
          (float) SixDOFMouseDriver::deviceValuesNormalized_[0]*
            SixDOFMouseDriver::cameraVectors_[i] -
          (float) SixDOFMouseDriver::deviceValuesNormalized_[1]*
            SixDOFMouseDriver::cameraVectors_[i+3] -
          (float) SixDOFMouseDriver::deviceValuesNormalized_[2]*
            SixDOFMouseDriver::cameraVectors_[i+6];
      }
      temp_trans.translation(pos);
      //double ori[3][3]; // orientation
      //dhdGetOrientationFrame(ori);
      //se3::SE3::Matrix3 Rot;
      //Rot  << (float)ori[0][0], (float)ori[0][1], (float)ori[0][2], (float)ori[1][0], 
        //(float)ori[1][1], (float)ori[1][2], (float)ori[2][0],
        //(float)ori[2][1], (float)ori[2][2];
      //Eigen::Matrix3f mat = Rot;
      //mat = 20*mat;
      //Eigen::Quaternionf q(mat);
      //q.normalize();

      double quat[4];

      // TODO à déplacer au bon endroit

      double bornes[3][2]; // limites du bras
      double d[3]; // dynamique par axe
      double angles[3]; // position lues en degrés
      double val[3]; // valeur normalisée
      double res[3]; // resultat : angles d'eulers adaptés
      double k = 2; // nombre de révolutions
      bornes[0][0]=-99.8687;
      bornes[0][1]=135.75;
      bornes[1][0]=-69.5698;
      bornes[1][1]=69.8842;
      bornes[2][0]=-17.0917;
      bornes[2][1]=184.128;
      dhdGetOrientationDeg(angles+0, angles+1, angles+2);
      for (int i=0; i<3;i++){
        d[i] = bornes[i][1]-bornes[i][0];
      //if(angles[i]<0) val[i] = fabs(bornes[i][0])-fabs(angles[i]);
      //else val[i] = angles[i] - fabs(bornes[i][0]);
        val[i]=angles[i]-bornes[i][0];
        //cout <<" "<<i<<"angles="<<angles[i]<<"borne "<< bornes[i][0]<< " val="<<val[i];
        //double aaa = val[i]/d[i];
        res[i] = k * 2 * M_PI * ((val[i]/d[i])-(1/2)); 
      }
      //cout << endl;

      //cout << "les angles " << res[0]<<" "<<res[1]<<" "<<res[2]<<endl; 
      euler2Quat(res[0], res[1], res[2], quat);
      
      //float3 r;
      //r = quat2Euler(q.w(),q.x(),q.y(),q.z());
      //r = quat2Euler(quat[0], quat[1], quat[2], quat[3]);
  
      //cout << " re les angles "<<angles[0]<<" "<<angles[1]<<" "<<angles[2]<<endl; 
      Eigen::Quaternionf qq((float)quat[0], (float)quat[1], (float)quat[2], (float)quat[3]);
      qq.normalize();
      Eigen::Matrix3f temp =quat2Mat(qq.x(),qq.y(),qq.z(),qq.w());
      temp_trans.rotation(temp);
      //Rot.setZero();
      //temp_trans.rotation(Rot);
    }
    if (type_==3)
    {

      float p[3], r[4], f[3], n[3];
      zmq::message_t request;

      //  Wait for next request from client
      socket_->recv (&request);
      Data *recv_config = (Data *)(request.data());
      // output the size of received collision array
      int array_size = recv_config->getCollisionArraySize();
      in_contact_ = array_size; 
      //cout << "user in contact" << in_contact_ << endl;
      //std::cout << std::setw(6) << "Array Size: " << array_size << std::endl;
      //std::cout<<*(recv_config->getCollisionPairByIndex(0).getPositionOnObj1())<<" "<<*(recv_config->getCollisionPairByIndex(0).getPositionOnObj2())<<std::endl;
      memcpy(f, recv_config->getArmCF().getForce(), 3*sizeof(float));
      memcpy(n,recv_config->getCollisionPairByIndex(0).getNormalOnObj1(),3*sizeof(float));
      user_force_<<f[0], f[1], f[2];
      contact_normal_<<n[0], n[1], n[2];
      contact_normal_.normalize();
      //cout << "XXXXXXX cn="<<contact_normal_.transpose()<<endl;
    //cout << "array size " << array_size<< endl;
    if(0)for (int i=0; i<array_size; i++){
      cout << "pt obj1 ";
      for (int j=0; j<3; j++)
      cout << recv_config->getCollisionPairByIndex(i).getPositionOnObj1()[j] << " ";
      cout << "pt obj2 ";
      for (int j=0; j<3; j++)
      cout << recv_config->getCollisionPairByIndex(i).getPositionOnObj2()[j] << " ";
      cout << "n  obj1 ";
      for (int j=0; j<3; j++)
      cout << recv_config->getCollisionPairByIndex(i).getNormalOnObj1()[0] << " ";
      cout << "n obj2 ";
      for (int j=0; j<3; j++)
      cout << recv_config->getCollisionPairByIndex(i).getNormalOnObj1()[0] << " ";

      
    }

      //cout << "force ";
      //for (int j=0; j<3; j++){
        //cout << f[j]<< " ";}
      //cout << endl;
      
      /*
      for (int j=0; j<3; j++){
        p[j] = recv_config->getObjCooridnate().getPosition()[j];
        //if (p[j]<1e-9)p[j] = 0;
        cout << "pos"<<j<<" "<<p[j]<<" ";
        //pos[j] = p[j];
        //cout << "force"<<j<< recv_config->getArmCF().getForceByIndex(j) << endl;
      }
      cout << endl;
      //*/

      //cout << "p 0 " << p[0] << " 1 " << p[1] << " 2 " << p[2] << endl;
      //cout << "pos 0 " << pos[0] << " 1 " << pos[1] << " 2 " << pos[2] << endl;
      //if (p[j]<1e-9)p[j] = 0;
      //cout << "pos"<<j<<" "<<p[j]<<" ";
      //pos[j] = p[j];


  
    

      // position ///////////////////////////////////////////////////////
      // pour le mur
      //pos[0] = recv_config->getObjCooridnate().getPosition()[0]+(float)1;
      //pos[1] = recv_config->getObjCooridnate().getPosition()[2];
      //pos[2] = recv_config->getObjCooridnate().getPosition()[1];//-1;
      // pour la voiture
      //pos[0] = -recv_config->getObjCooridnate().getPosition()[0];
      //pos[1] = recv_config->getObjCooridnate().getPosition()[1];
      //pos[2] = recv_config->getObjCooridnate().getPosition()[2];
 

     // pour le tore
      pos[0] = -0.0035-recv_config->getObjCooridnate().getPosition()[0];
      pos[1] = 0.0035+recv_config->getObjCooridnate().getPosition()[1];
      pos[2] = 0.0035+recv_config->getObjCooridnate().getPosition()[2];
      pos[0] = 0.216-recv_config->getObjCooridnate().getPosition()[0];
      pos[1] = -0.030+recv_config->getObjCooridnate().getPosition()[1];
      pos[2] = +0.002+recv_config->getObjCooridnate().getPosition()[2];
      pos[0] = -0.009-recv_config->getObjCooridnate().getPosition()[0];
      pos[1] = +0.005+recv_config->getObjCooridnate().getPosition()[1];
      pos[2] = +recv_config->getObjCooridnate().getPosition()[2];


      temp_trans.translation(pos);
      //cout<<"pos";for(int j=0;j<3;j++){cout<<" "<<pos[j];} cout<<endl;


      //rotation /////////////////////////////////////////////////
      memcpy(r, recv_config->getObjCooridnate().getQuaternion(), 4*sizeof(float));
      //cout<<"quat recu ";for(int j=-3;j<4;j++){cout<<"rot "<<j<<" "<<r[j]<<" ";}
      //cout << endl;
      Eigen::Quaterniond qq(r[0], -r[1], r[2],  r[3]);
      qq.normalize();
      Eigen::Matrix3f temp =quat2Mat(qq.x(),qq.y(),qq.z(),qq.w());
      temp_trans.rotation(temp);

     } 


    /////////////////////////////////////////////////////////
    // apply configuration
    //mutex_.lock();
    transformation_ = temp_trans;
    //cout << transformation_.translation().transpose() << endl;
    SixDOFMouseDriver::has_moved_ = true;
  }
}

// read data from device and fill class members
void SixDOFMouseDriver::getData(void* arg)
{
  //cout << "getData\n";
  if (type_==1)
  {
    union val{
      char swap[2];
      short int value;
    };
    val v;
    // 6D mouse sends one 7 byte position and one 7 byte orientation frame
    memset(SixDOFMouseDriver::data_, 0x0, 14);
    // read position
    if (read(SixDOFMouseDriver::fd_, SixDOFMouseDriver::data_, 7) != 7)
      std::cout << "read error" << std::endl;
    if (SixDOFMouseDriver::data_[0] == 1){
      //conversion to big endian
      for (int i = 0; i<3; i++){
        v.swap[0] = SixDOFMouseDriver::data_[1+2*i];
        v.swap[1] = SixDOFMouseDriver::data_[1+2*i+1];
        // divide by 512 for normalization
        SixDOFMouseDriver::deviceValues_[i] = v.value;
        SixDOFMouseDriver::deviceValuesNormalized_[i] = (float)v.value/512;
      }
    }
    // read orientation
    if (read(SixDOFMouseDriver::fd_, SixDOFMouseDriver::data_+7, 7) != 7)
      std::cout << "read error" << std::endl;
    if (SixDOFMouseDriver::data_[7] == 2){
      //conversion to big endian
      for (int i = 0; i<3; i++){
        v.swap[0] = SixDOFMouseDriver::data_[8+2*i];
        v.swap[1] = SixDOFMouseDriver::data_[8+2*i+1];
        // divide by 512 for normalization
        SixDOFMouseDriver::deviceValues_[3+i] = v.value;
        SixDOFMouseDriver::deviceValuesNormalized_[3+i] = (float)v.value/512;
      }
    }
  }
  if (type_==2)
  {
    int fact = 50;
    double   posX, posY, posZ;
    posX = posY = posZ = 0.0;
    //Eigen::Vector3d deviceForce;
    //Eigen::Vector3d deviceTorque;
    Eigen::Vector3d devicePos;
    Eigen::Matrix3d deviceRot;
    dhdEnableForce (DHD_ON);
    if (dhdGetButtonMask()){
      cout << "RECALE\n";
      dhdGetPosition(&posX, &posY, &posZ);
      K_off_[0] = deviceValuesNormalized_[0]-posY*fact;
      K_off_[1] = deviceValuesNormalized_[1]+posZ*fact;
      K_off_[2] = deviceValuesNormalized_[2]+posX*fact;
      recale_ = true;
    }
    else recale_ = false;
    if (!recale_){
      //dhdGetForce(&posX, &posY, &posZ);
      dhdGetPosition(&posX, &posY, &posZ);
      //cout << "pos " << posX << " " << posY << " " << posZ;
      SixDOFMouseDriver::deviceValuesNormalized_[0] = posY*fact+K_off_[0];
      SixDOFMouseDriver::deviceValuesNormalized_[1] = -posZ*fact+K_off_[1];
      SixDOFMouseDriver::deviceValuesNormalized_[2] = -posX*fact+K_off_[2];
    }

    //else{
      //for (int i=0; i<6; i++) deviceValuesNormalized_[i] = 0;
    //}
    // compute forces and torques
    //deviceForce_.setZero ();
    //deviceTorque_.setZero ();
    // send forces to device
    //dhdSetForceAndTorqueAndGripperForce (deviceForce_(0), deviceForce_(1),  deviceForce_(2),deviceTorque_(0), deviceTorque_(1), deviceTorque_(2), 0.0);
  }
  if(type_==3)
  {

    
    
    //zmq::message_t request;
    //socket.recv (&request);
    //Data *recv_config = (Data *)(request.data());

    //zmq_msg_t msg;
    //int rc = zmq_msg_init (&msg);
    //rc = zmq_msg_recv (&msg, (void*)socket_, 0);
    //Data *recv_config = (Data *)(msg.data());


    //zmq::message_t request;

    //  Wait for next request from client

    //  Wait for next request from client
    //socket_->recv (&request);
    //Data *recv_config = (Data *)(request.data());
    //int array_size = recv_config->getCollisionArraySize();
    //std::cout << std::setw(6) << "Array Size: " << array_size << std::endl;
    //for (int j=0; j<3; j++){
      //cout << "pos"<<j<<" "<<recv_config->getObjCooridnate().getPosition()[j] << endl;
      //cout << "force"<<j<< recv_config->getArmCF().getForceByIndex(j) << endl;
    //}
    //cout << "message reçu\n";
  }
    //cout << "fin getData\n";


  //	//print values
  //	printf("rawdata position\n");
  //	for (int i = 0; i<7; ++i)
  //		printf("%02hhX ", SixDOFMouseDriver::data_[i]);
  //	printf("\n");

  //	//print values
  //	printf("rawdata orientation\n");
  //	for (int i = 8; i<14; ++i){
  //		printf("%02hhX;", SixDOFMouseDriver::data_[i]);
  //	}
  //	printf("\n");

  //	printf("formatted values\n");
  //	for (int i = 0; i<14; ++i)
  //		printf("%02hhX ", SixDOFMouseDriver::data_[i]);
  //	printf("\n");
  //	printf("integer values\n");
  //	for (int i = 3; i<6; ++i) 
  //		std::cout << SixDOFMouseDriver::deviceValues_[i] << " ";
  //	std::cout << std::endl;
  //printf("float values\n");
  //for (int  i = 0; i<6; ++i)
  //std::cout << SixDOFMouseDriver::deviceValuesNormalized_[i] << "\t\t\t";
  //std::cout << "\n";

  //for (int i = 0; i<6; i++)SixDOFMouseDriver::deviceValuesNormalized_[i]=0;

}


void SixDOFMouseDriver::SetConnexion(){
  context_ = new zmq::context_t(1);
  socket_ = new zmq::socket_t(*context_, ZMQ_PULL);
  socket_->bind("tcp://*:5555");
}


    /*
       pos[0] =
       (float) SixDOFMouseDriver::deviceValuesNormalized_[0]/divideFactor*SixDOFMouseDriver::cameraVectors_[0] -
       (float) SixDOFMouseDriver::deviceValuesNormalized_[1]/divideFactor*SixDOFMouseDriver::cameraVectors_[3] -
       (float) SixDOFMouseDriver::deviceValuesNormalized_[2]/divideFactor*SixDOFMouseDriver::cameraVectors_[6];
       pos[1] =
       (float) SixDOFMouseDriver::deviceValuesNormalized_[0]/divideFactor*SixDOFMouseDriver::cameraVectors_[1] -
       (float) SixDOFMouseDriver::deviceValuesNormalized_[1]/divideFactor*SixDOFMouseDriver::cameraVectors_[4] -
       (float) SixDOFMouseDriver::deviceValuesNormalized_[2]/divideFactor*SixDOFMouseDriver::cameraVectors_[7];
       pos[2] =
       (float) SixDOFMouseDriver::deviceValuesNormalized_[0]/divideFactor*SixDOFMouseDriver::cameraVectors_[2] -
       (float) SixDOFMouseDriver::deviceValuesNormalized_[1]/divideFactor*SixDOFMouseDriver::cameraVectors_[5] -
       (float) SixDOFMouseDriver::deviceValuesNormalized_[2]/divideFactor*SixDOFMouseDriver::cameraVectors_[8];
    //*/







      //Eigen::Vector3d v;
      //Eigen::Quaterniond p, np, pi, rqq;

      //v<<1,0,0;
      //p.w() = 0;
      //p.vec() = v;
      //np.w() = 0;
      //np.vec() = -v;
      //pi = p.inverse();
      //rqq = np * qq * pi;
      //cout<<"np x "<<np.x()<<" y "<<np.y()<<" z "<<np.z()<<" w "<<np.w()<<endl;
      //cout<<"pi x "<<pi.x()<<" y "<<pi.y()<<" z "<<pi.z()<<" w "<<pi.w()<<endl;


      //rqq = rqq * pi; 

      //cout<<"rqat x "<<rqq.x()<<" y "<<rqq.y()<<" z "<<rqq.z()<<" w "<<rqq.w()<<endl;
      //temp =quat2Mat(rqq.x(),rqq.y(),rqq.z(),rqq.w());

      //Eigen::Vector3d rotatedV = rqq.vec();


      //origine = qq;
      //mirroir(0.0,1.0,0.0,0.0); 
      //origine<<qq.x(), qq.y(), qq.z();
      //mirroir<<1, 0, 0;
      //retourne =  -mirroir.transpose() * origine;
      //mirroir = mirroir.inverse();
      /*
      double roll, pitch, yaw;
      toEulerianAngle(qq, roll, pitch, yaw);
      //float3 eu;
      //eu = quat2Euler(qq.w(),qq.x(),qq.y(),qq.z());
      //roll = yaw = 0;
      roll = roll/(2*M_PI)*360; 
      pitch = pitch/(2*M_PI)*360; 
      yaw = yaw/(2*M_PI)*360; 
      cout<<"eulers r "<<roll<<" p "<<pitch<<" y "<<yaw<<endl;
      //cout<<"eulerv r"<<eu[0]<<" p"<<eu[1]<<" y"<<eu[2]<<endl;

      //cout << "quat eigen2 " << qq2.x()<<" "<<qq2.y()<<" "<<qq2.z()<<" "<<qq2.w()<<endl;

      float3x3 convert;
      convert[0][0] = temp(0,0);
      convert[0][1] = temp(1,0);
      convert[0][2] = temp(2,0);
      convert[1][0] = temp(0,1);
      convert[1][1] = temp(1,1);
      convert[1][2] = temp(2,1);
      convert[2][0] = temp(0,2);
      convert[2][1] = temp(1,2);
      convert[2][2] = temp(2,2);
      float3 rt = eulerAngles(convert);

      //cout<<"euler2 r "<<rt[1]<<" p "<<rt[0]<<" y "<<rt[2]<<endl;
      //if(std::abs(rt[0])<1e-9)rt[0]=0; 
      //if(std::abs(rt[1])<1e-9)rt[1]=0; 
      //if(std::abs(rt[2])<1e-9)rt[2]=0; 

      //roll = pitch = yaw = 0;

      roll = rt[0];
      pitch = -rt[1];
      yaw = rt[2]; 

      //cout<<"euler3 r "<<roll<<" p "<<pitch<<" y "<<yaw<<endl<<endl;

      Eigen::Quaterniond qq2(toQuaternion(roll, pitch, yaw));
      qq2.normalize();
      //cout<<"quat2x "<<qq2.x()<<" y "<<qq2.y()<<" z "<<qq2.z()<<" w "<<qq2.w()<<endl;
      //*/
      
      //temp =quat2Mat(qq2.x(),qq2.y(),qq2.z(),qq2.w());
