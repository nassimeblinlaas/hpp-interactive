
#include <hpp/interactive/sixDOFMouseDriver.hh>
#include <stdlib.h>
#include <stdio.h>
//#include <libudev.h>
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
bool SixDOFMouseDriver::has_moved_;
// x, y, z for side, front, up vectors ; in this order
float SixDOFMouseDriver::cameraVectors_[9];


using namespace std;

SixDOFMouseDriver::SixDOFMouseDriver(){

}

const se3::SE3& SixDOFMouseDriver::getTransformation(){
    mutex_.lock(); // TODO deux fois le mutex dans le code !
	const se3::SE3& trans = SixDOFMouseDriver::transformation_;
	mutex_.unlock();
	return trans;
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

	m << 	0		, -v[2], 	v[1],
			v[2]	, 		0, -v[0],
			-v[1]	,  v[0], 		0	;

	return m;
}

void SixDOFMouseDriver::MouseInit(double* bounds)
{

    // open device
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

    // init status
    SixDOFMouseDriver::has_moved_ = false;

    // init position
    transformation_.rotation().setIdentity();
    /*transformation_.translation()[0] = 5;
    transformation_.translation()[1] = 5;
    transformation_.translation()[2] = 5;*/

    // init speed
	SixDOFMouseDriver::linear_speed_ = 10;
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

// read interactice device thread function
void SixDOFMouseDriver::ReadMouse(double* bounds_)
{
	se3::SE3::Vector3 pos, rot, axei, local, temp;
    double bounds[6];

    //*
    for (int i=0; i<6; i++)
        bounds[i] = bounds_[i];
    //*/

	// infinite loop
	while (1){
        //cout << "infinite ReadMouse loop thread\n";
		getData();
		mutex_.unlock();

		/////////////////////////////////////////////////////////
		// translation
		float divideFactor = SixDOFMouseDriver::linear_speed_;// dt

        /*
        cout << "in the driver ";
        for (int i=0; i<9; i++)
            cout << SixDOFMouseDriver::cameraVectors_[i] << " ";
        cout << endl;
        //*/


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

        //*
        for (int i=0; i<3; i++){
            pos[i] =
             (float) SixDOFMouseDriver::deviceValuesNormalized_[0]/divideFactor*SixDOFMouseDriver::cameraVectors_[i] -
             (float) SixDOFMouseDriver::deviceValuesNormalized_[1]/divideFactor*SixDOFMouseDriver::cameraVectors_[i+3] -
             (float) SixDOFMouseDriver::deviceValuesNormalized_[2]/divideFactor*SixDOFMouseDriver::cameraVectors_[i+6];
        }
        //*/
		
        //*
        // bounds limits ---------------- TODO : provoque un effet de bords avec les rotations
        if (pos[0]+transformation_.translation()[0]<bounds[0]) pos[0] = 0;
        if (pos[0]+transformation_.translation()[0]>bounds[1]) pos[0] = 0;
        if (pos[1]+transformation_.translation()[1]<bounds[2]) pos[1] = 0;
        if (pos[1]+transformation_.translation()[1]>bounds[3]) pos[1] = 0;
        if (pos[2]+transformation_.translation()[2]<bounds[4]) pos[2] = 0;
        if (pos[2]+transformation_.translation()[2]>bounds[5]) pos[2] = 0;
		// -------------------------------------------------------
        //*/

		se3::SE3 temp_trans = transformation_;
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
        rot[2] = rot[2]/norme;
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
        double threshold = 0.5; // anciennement 0.5 0.2
        divideFactor = SixDOFMouseDriver::angular_speed_;
        // rotation variations
		se3::SE3::Vector3 v_local (0., 0., 0.);


        // threshold for rotations
        if (std::abs(rot[0]) > threshold)	v_local[0] = (rot[0]-threshold)/(float)divideFactor;
        if (std::abs(rot[1]) > threshold)	v_local[1] = -(rot[1]-threshold)/(float)divideFactor;
        if (std::abs(rot[2]) > threshold)	v_local[2] = (rot[2]-threshold)/(float)divideFactor;

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

        cout << "matrot " << matrot.transpose() << endl;

        v_local = transformation_.rotation().transpose() * v_local; // ça marcheoie biengue
        //v_local = matrot.transpose() * v_local;

		se3::SE3::Matrix3 dR;
		// threshold
		if (v_local.norm () < 1e-8) dR.setIdentity();
		else expMap(v_local, dR);

        //cout << dR << endl << endl;


		se3::SE3::Matrix3 R_new (temp_trans.rotation () * dR);
		temp_trans.rotation(R_new);

		/////////////////////////////////////////////////////////
		// apply configuration
		mutex_.lock();
		transformation_ = temp_trans;
        SixDOFMouseDriver::has_moved_ = true;
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
//    printf("float values\n");
//    for (int  i = 1; i<6; ++i)
//        std::cout << SixDOFMouseDriver::deviceValuesNormalized_[i] << ";";
//    std::cout << std::endl;

    /*
    struct udev *udev;
    struct udev_enumerate *enumerate;
    struct udev_list_entry *devices, *dev_list_entry;
    struct udev_device *dev;
    struct udev_monitor *mon;
    int fd;
    //*/
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

}
