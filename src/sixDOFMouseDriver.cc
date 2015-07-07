
#include <hpp/interactive/sixDOFMouseDriver.hh>
#include <stdlib.h>
#include <stdio.h>
#include <libudev.h>
#include <locale.h>
#include <unistd.h>

#include <iostream>

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


using namespace std;

SixDOFMouseDriver::SixDOFMouseDriver(){

}

const se3::SE3& SixDOFMouseDriver::getTransformation(){
	mutex_.lock();
	const se3::SE3& trans = SixDOFMouseDriver::transformation_;
	mutex_.unlock();
	return trans;
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

	m << 	0		, -v[2], 	v[1],
			v[2]	, 		0, -v[0],
			-v[1]	,  v[0], 		0	;

	return m;
}

void SixDOFMouseDriver::MouseInit()
{
	struct udev *udev;
	struct udev_enumerate *enumerate;
	struct udev_list_entry *devices, *dev_list_entry;
	struct udev_device *dev;
	struct udev_monitor *mon;
	int fd;

//	udev = udev_new();
//	mon = udev_monitor_new_from_netlink(udev, "udev");
//	udev_monitor_filter_add_match_subsystem_devtype(mon, "hidraw", NULL);
//	udev_monitor_enable_receiving(mon);
//	fd = udev_monitor_get_fd(mon);
//
//	enumerate = udev_enumerate_new(udev);
//	udev_enumerate_add_match_subsystem(enumerate, "hidraw");
//	udev_enumerate_scan_devices(enumerate);
//	devices = udev_enumerate_get_list_entry(enumerate);




	// TODO
	/*   Open the Device with non-blocking reads. In real life,
	 *         don't use a hard coded path; use libudev instead. */
	fd_ = open("/dev/hidraw2", O_RDONLY);

	if (fd_ < 0) {
		perror("Unable to open interactive device");
		abort();
	}
	else {
		std::cout << "device file descriptor : " << fd_ << "\n";
	}

	SixDOFMouseDriver::linear_speed_ = 10;
	SixDOFMouseDriver::angular_speed_ = 20;

	transformation_.rotation().setIdentity();
	// execute thread 
	void* arg = 0;
	SixDOFMouseDriver::interactiveDeviceThread_ = 
		new boost::thread(boost::thread(SixDOFMouseDriver::ReadMouse, arg));


}

// read interactice device thread function
void SixDOFMouseDriver::ReadMouse(void* arg)
{
	se3::SE3::Vector3 pos, rot, axei, local, temp;

	// infinite loop
	while (1){
		getData();
		mutex_.unlock();

		/////////////////////////////////////////////////////////
		// translation
		float divideFactor = SixDOFMouseDriver::linear_speed_;// dt
		pos[0] = (float) -SixDOFMouseDriver::deviceValuesNormalized_[0]/divideFactor;
		pos[1] = (float) SixDOFMouseDriver::deviceValuesNormalized_[1]/divideFactor;
		pos[2] = (float) -SixDOFMouseDriver::deviceValuesNormalized_[2]/divideFactor;
		
		// limits for maze problem TODO to remove ----------------
		if (pos[2]+transformation_.translation()[2]<0) pos[2] = 0;
		if (pos[2]+transformation_.translation()[2]>1) pos[2] = 0;
		// -------------------------------------------------------

		se3::SE3 temp_trans = transformation_;
		temp_trans.translation(pos + transformation_.translation());

		/////////////////////////////////////////////////////////
		// rotation
		rot[0] = (float) (SixDOFMouseDriver::deviceValuesNormalized_[3]* M_PI);
		rot[1] = (float) (SixDOFMouseDriver::deviceValuesNormalized_[4]* M_PI);
		rot[2] = (float) (SixDOFMouseDriver::deviceValuesNormalized_[5]* M_PI);

		/////////////////////////////////////////////////////////
		// integrate rotations
		double threshold = 0.5;
		divideFactor = SixDOFMouseDriver::angular_speed_;
		se3::SE3::Vector3 v_local (0., 0., 0.);

		// threshold for rotations
		if (std::abs(rot[0]) > threshold)	v_local[0] = rot[0]/(float)divideFactor;
		if (std::abs(rot[1]) > threshold)	v_local[1] = rot[1]/(float)divideFactor;
		if (std::abs(rot[2]) > threshold)	v_local[2] = rot[2]/(float)divideFactor;

		se3::SE3::Matrix3 dR;
		// threshold
		if (v_local.norm () < 1e-8) dR.setIdentity();
		else expMap(v_local, dR);

		se3::SE3::Matrix3 R_new (temp_trans.rotation () * dR);
		temp_trans.rotation(R_new);

		/////////////////////////////////////////////////////////
		// apply configuration
		mutex_.lock();
		transformation_ = temp_trans;
	}
}

// read data from device and fill class members
void SixDOFMouseDriver::getData()
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
//	printf("float values\n");
//	for (int  i = 3; i<6; ++i) 
//		std::cout << SixDOFMouseDriver::deviceValuesNormalized_[i] << ";";
//	std::cout << std::endl;

}
