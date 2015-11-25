/* Linux hid driver for 6Dmouse by immersion model 3Dconnexion
 * Written by Nassime BLIN 2015 for Gepetto LAAS-CNRS LGP-INPT
 * 
 * usage :
 * 1) plug mouse, /dev/hidraw0~3 will appear
 * 2) find the right device using cat or ohter (usually hidraw2)
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

#include <gepetto/viewer/corba/client.hh> // se3 dependancy

#ifndef SIXDOFMOUSEDRIVER
#define SIXDOFMOUSEDRIVER

class SixDOFMouseDriver{
	public :
		SixDOFMouseDriver();
		// to call for init
		static void MouseInit();
		// blocking if no data
		static const se3::SE3& getTransformation();
		// return false before first data
		static bool HasMoved(){return has_moved_;};

		static void setLinearSpeed(double sp);
		static void setAngularSpeed(double sp);
		static void setRotationThreshold(double th);
	
	private :
		static void getData();
		static void ReadMouse(void* arg);
		
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
};


#endif // SIXDOFMOUSEDRIVER
