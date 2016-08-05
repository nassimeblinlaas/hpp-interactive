/* Linux hid driver for 6Dmouse by immersion model 3Dconnexion
 * Written by Nassime BLIN 2015 for Gepetto LAAS-CNRS LGP-INPT
 * 
 * usage :
 * 1) plug mouse, /dev/hidraw0~3 will appear
 * 2) find the right device using cat or other (usually hidraw2)
 * 3) sudo chmod 664 /dev/hidraw2
 * 4) write correct name of device in sixDOFMouseDriver.cc::88
 *
 * call MouseInit() first
 *
 * */

#include <boost/thread/thread.hpp>

#include <linux/hidraw.h>
#include <fcntl.h>

#include <iostream>
#include <math.h>

#include <gepetto/viewer/corba/se3.hh> // se3 dependancy

#ifndef SIXDOFMOUSEDRIVER
#define SIXDOFMOUSEDRIVER

class SixDOFMouseDriver{
	public :
		SixDOFMouseDriver();
		// to call for init
        static void MouseInit(short int type, double* bounds);
        static void InitPosition(double* translation);
		// blocking if no data
        static const se3::SE3& getTransformation();
		// return false before first data
		static bool HasMoved(){return has_moved_;};

		static void setLinearSpeed(double sp);
		static void setAngularSpeed(double sp);
		static void setRotationThreshold(double th);
	
        static void setCameraVectors(float a1, float a2, float a3,
                                     float b1, float b2, float b3,
                                     float c1, float c2, float c3);

	private :
		static void getData();
       		static void ReadMouse(double* bounds);
		
		static int fd_; // hid file descriptor
		static boost::thread* interactiveDeviceThread_;			
		static boost::mutex mutex_;
		static double linear_speed_; // not a real speed, only a division factor for input data
		static double angular_speed_; // same
		static double rotation_threshold_;
		
		static char data_[14]; // raw data
		static short int deviceValues_[6]; // formatted data
		static double deviceValuesNormalized_[6]; // normalized data
        static se3::SE3 transformation_;
		static bool has_moved_;

        static float cameraVectors_[9];
                static short int type_;// device type 1 6D mouse 2 sigma7
};


#endif // SIXDOFMOUSEDRIVER
