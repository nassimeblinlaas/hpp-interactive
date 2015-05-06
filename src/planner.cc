//
// Copyright (c) 2014 CNRS
// Authors: Nassime Blin
//
// This file is part of hpp-core
// hpp-core is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-core is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#include <boost/tuple/tuple.hpp>
#include <boost/thread/thread.hpp>
#include <hpp/util/debug.hh>
#include <hpp/model/configuration.hh>
#include <hpp/model/device.hh>

#include <hpp/model/joint.hh>
#include <hpp/model/device.hh>
#include <hpp/model/object-factory.hh>

#include <hpp/core/config-projector.hh>
#include <hpp/interactive/planner.hh>
#include <hpp/core/node.hh>
#include <hpp/core/edge.hh>
#include <hpp/core/path.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/steering-method.hh>
#include <hpp/core/basic-configuration-shooter.hh>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>


#include <linux/types.h>
#include <linux/input.h>
#include <linux/hidraw.h>
/*   Unix */
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <iostream>
#include <fstream>
#include <iomanip>
#include <math.h>


#include <gepetto/viewer/corba/client.hh>
#include <hpp/core/problem.hh>

using namespace std;

namespace hpp {
	namespace interactive {

		typedef se3::SE3::Vector3 Vector3;
		typedef se3::SE3::Matrix3 Matrix3;
		RoadmapPtr_t Planner::roadmap_i;
		boost::thread* Planner::interactiveDeviceThread_ = 0;
		int Planner::fd_ = 0;
		char Planner::data_[14];
		short int Planner::deviceValues_[6];
		float Planner::deviceValuesNormalized_[6];
		const Problem* Planner::problem_i = 0;
		Configuration_t Planner::actual_configuration_;
		ConfigurationPtr_t Planner::actual_configuration_ptr_;


		using model::displayConfig;

		inline void expMap (const Vector3 & omega, Matrix3 & R)
		{
			double theta = omega.norm ();
			Vector3 u (omega / theta);

			double s0, c0;
			s0 = sin(theta);
			c0 = cos(theta);

			double v0 = 1 - c0;

			R(0,0) = u(0) * u(0) * v0 + c0;
			R(0,1) = u(0) * u(1) * v0 - u(2) * s0;
			R(0,2) = u(0) * u(2) * v0 + u(1) * s0;

			R(1,0) = u(0) * u(1) * v0 + u(2) * s0;
			R(1,1) = u(1) * u(1) * v0 + c0;
			R(1,2) = u(1) * u(2) * v0 - u(0) * s0;

			R(2,0) = u(0) * u(2) * v0 - u(1) * s0;
			R(2,1) = u(1) * u(2) * v0 + u(0) * s0;
			R(2,2) = u(2) * u(2) * v0 + c0;
		}


		PlannerPtr_t Planner::createWithRoadmap
			(const Problem& problem, const RoadmapPtr_t& roadmap)
			{
				Planner* ptr = new Planner (problem, roadmap);
				Planner::roadmap_i = roadmap;


				//-------------------------------------------------------
				const Problem& pb = problem;	
				Planner::problem_i = &pb;
				//-------------------------------------------------------
				ConfigurationPtr_t config (new Configuration_t ((hpp::model::size_type)7));
				(*config)[0] = 0;
				(*config)[1] = 0;
				(*config)[2] = 0;
				(*config)[3] = 1;
				(*config)[4] = 0;
				(*config)[5] = 0;
				(*config)[6] = 0;
				actual_configuration_ptr_ = config;
				//-------------------------------------------------------

				return PlannerPtr_t (ptr);
			}

		PlannerPtr_t Planner::create (const Problem& problem)
		{
			Planner* ptr = new Planner (problem);
			return PlannerPtr_t (ptr);
		}

		Planner::Planner (const Problem& problem):
			PathPlanner (problem),
			configurationShooter_ (new BasicConfigurationShooter (problem.robot ())),
			qProj_ (problem.robot ()->configSize ())
		{
		}

		Planner::Planner (const Problem& problem,
				const RoadmapPtr_t& roadmap) :
			PathPlanner (problem, roadmap),
			configurationShooter_ (new BasicConfigurationShooter (problem.robot ())),
			qProj_ (problem.robot ()->configSize ())
		{
			InteractiveDeviceInit();
		}

		void Planner::InteractiveDeviceInit()
		{

			//			cout << " Planner::Interac" << endl;
			std::cout << "init interactive device\n";

			// TODO
			/*   Open the Device with non-blocking reads. In real life,
			 *         don't use a hard coded path; use libudev instead. */
			this->fd_ = open("/dev/hidraw2", O_RDONLY);

			if (this->fd_ < 0) {
				perror("Unable to open interactive device");
				abort();
			}
			else {
				std::cout << "device file descriptor : " << this->fd_ << "\n";
			}

			//			const	ConfigurationPtr_t config;a
			//ConfigurationPtr_t config = problem_.initConfig ();
			//const ConfigurationPtr_t cconf = config;

			//problem_.initConfig(cconf);	

			//this->problem_.initConfig(config);
			//hpp::core::Problem::initConfig

			// execute thread
			void* arg = 0;
			Planner::interactiveDeviceThread_ = 
				new boost::thread(boost::thread(Planner::ReadInteractiveDevice, arg));

		}



		se3::SE3::Matrix3 operatorHat(se3::SE3::Vector3 v){
			se3::SE3::Matrix3 m;

			m << 	0			, -v[2], 	v[1],
				v[2]	, 		0, -v[0],
				-v[1]	,  v[0], 		0	;

			return m;
		}



		// read interactice device thread function
		void Planner::ReadInteractiveDevice(void* arg)
		{
			std::cout << "read interactive device thread beginning\n";
			std::ofstream myfile;
			myfile.open ("/tmp/out.csv");

			int i = (int) arg; i = 0; i++;
			float f = (float) 0.1;

			// add landmark to viewer
			graphics::corbaServer::ClientCpp p;
			p.createGroup ("scene_hpp_");
			gepetto::corbaserver::Color color;
			color[0] = 1;
			color[1] = 1;
			color[2] = 1;
			color[3] = 1.;
			p.addBox ("scene_hpp_/curseur", f/10,f/10,f/10, color);
			p.addLandmark("scene_hpp_/curseur", 1.);
			p.addSceneToWindow ("scene_hpp_", 0);
			//p.addSphere ("scene_hpp_/curseur2", 1., color);
			//p.addLandmark("scene_hpp_/curseur2", 2.);








			se3::SE3::Vector3 pos, rot, axei, local, temp;
			se3::SE3 transformation = 0;
			unsigned int refresh = 1;
			double dt = 0.5;

			bool unique = false;
			// infinite loop
			while (1){
				getData(myfile);
				//if (unique) break; // permet de couper le programme après la première lecture pour debug
				unique = true;


				/////////////////////////////////////////////////////////
				// translation
				float divideFactor = 10;// dt
				pos[0] = -Planner::deviceValuesNormalized_[0]/divideFactor;
				pos[1] = Planner::deviceValuesNormalized_[1]/divideFactor;
				pos[2] = -Planner::deviceValuesNormalized_[2]/divideFactor;
				transformation.translation(pos + transformation.translation());

				(*actual_configuration_ptr_)[0] = transformation.translation()[0];
				(*actual_configuration_ptr_)[1] = transformation.translation()[1];
				(*actual_configuration_ptr_)[2] = transformation.translation()[2];
				(*actual_configuration_ptr_)[3] = 1;
				(*actual_configuration_ptr_)[4] = 0;
				(*actual_configuration_ptr_)[5] = 0;
				(*actual_configuration_ptr_)[6] = 0;
				//				cout << "act conf" << endl 
				//					<< (*actual_configuration_ptr_) << endl;

				/////////////////////////////////////////////////////////
				// rotation
				rot[0] = Planner::deviceValuesNormalized_[3]* M_PI;
				rot[1] = Planner::deviceValuesNormalized_[4]* M_PI;
				rot[2] = Planner::deviceValuesNormalized_[5]* M_PI;
				//cout << "rotation" << endl << rot.transpose() << endl;

				/////////////////////////////////////////////////////////
				// integrate rotations
				double threshold = 0.5;
				divideFactor = 20;
				se3::SE3::Vector3 v_local (0., 0., 0.);
				if (abs(rot[0]) > threshold ) v_local[0] = rot[0]/divideFactor;
				if (abs(rot[1]) > threshold ) v_local[1] = rot[1]/divideFactor;
				if (abs(rot[2]) > threshold ) v_local[2] = rot[2]/divideFactor;


				Matrix3 dR;
				if (v_local.norm () < 1e-8)
					dR.setIdentity();
				else
					expMap(v_local, dR);

				Matrix3 R_new (transformation.rotation () * dR);


				/////////////////////////////////////////////////////////
				/////////////////////////////////////////////////////////
				// apply configuration
				transformation.rotation(R_new);		
				p.applyConfiguration("scene_hpp_/curseur", transformation);	
				p.refresh();

			}

		}


		// read data from device and fill class members
		void Planner::getData(std::ofstream& file)
		{

			union val{
				char swap[2];
				short int value;
			};
			val v;

			// 6D mouse sends 1 position frame then 1 orientation frame
			memset(Planner::data_, 0x0, 14);

			// read position
			if (read(Planner::fd_, Planner::data_, 7) != 7)
				std::cout << "read error" << std::endl;


			//			//print values
			//			printf("rawdata position\n");
			//			for (int i = 0; i<7; ++i)
			//				printf("%02hhX ", Planner::data_[i]);
			//			printf("\n");
			if (Planner::data_[0] == 1){
				//conversion to big endian
				for (int i = 0; i<3; i++){
					v.swap[0] = Planner::data_[1+2*i];
					v.swap[1] = Planner::data_[1+2*i+1];
					Planner::deviceValues_[i] = v.value;
					Planner::deviceValuesNormalized_[i] = (float)v.value/512;
				}
			}

			// read orientation
			if (read(Planner::fd_, Planner::data_+7, 7) != 7)
				std::cout << "read error" << std::endl;

			if (Planner::data_[7] == 2){
				//conversion to big endian
				for (int i = 0; i<3; i++){
					v.swap[0] = Planner::data_[8+2*i];
					v.swap[1] = Planner::data_[8+2*i+1];
					// divide by 512 for normalization
					Planner::deviceValues_[3+i] = v.value;
					Planner::deviceValuesNormalized_[3+i] = (float)v.value/512;
				}
			}

			//			//print values
			//			printf("rawdata orientation\n");
			//			for (int i = 8; i<14; ++i){
			//	//			file << Planner::data_[i] << ";" ;
			//				printf("%02hhX;", Planner::data_[i]);
			//			}
			////			file << std::endl;
			//			printf("\n");
			//			printf("formatted values\n");
			//	for (int i = 0; i<14; ++i)
			//				printf("%02hhX ", Planner::data_[i]);
			//				printf("\n");
			//	printf("integer values\n");
			//			for (int i = 3; i<6; ++i) 
			//				std::cout << Planner::deviceValues_[i] << " ";
			//			std::cout << std::endl;
			//			printf("float values\n");
			//			for (int  i = 3; i<6; ++i) 
			//				std::cout << Planner::deviceValuesNormalized_[i] << ";";
			////				file << Planner::deviceValuesNormalized_[i] << ";";
			////
			//			std::cout << std::endl;
			//*/ 
			//			file << ";" << std::endl;


		}


		void Planner::init (const PlannerWkPtr_t& weak)
		{
			PathPlanner::init (weak);
			weakPtr_ = weak;
		}



		bool belongs (const ConfigurationPtr_t& q, const Nodes_t& nodes)
		{
			for (Nodes_t::const_iterator itNode = nodes.begin ();
					itNode != nodes.end (); ++itNode) {
				if (*((*itNode)->configuration ()) == *q) return true;
			}
			return false;
		}

		PathPtr_t Planner::extend (const NodePtr_t& near,
				const ConfigurationPtr_t& target)
		{
			const SteeringMethodPtr_t& sm (problem ().steeringMethod ());
			const ConstraintSetPtr_t& constraints (sm->constraints ());
			if (constraints) {
				ConfigProjectorPtr_t configProjector (constraints->configProjector ());
				if (configProjector) {
					configProjector->projectOnKernel (*(near->configuration ()), *target,
							qProj_);
				} else {
					qProj_ = *target;
				}
				if (constraints->apply (qProj_)) {
					return (*sm) (*(near->configuration ()), qProj_);
				} else {
					return PathPtr_t ();
				}
			}
			return (*sm) (*(near->configuration ()), *target);
		}


		void Planner::oneStep ()
		{
			std::cout << "one step\n";
			sleep(3);
			typedef boost::tuple <NodePtr_t, ConfigurationPtr_t, PathPtr_t>
				DelayedEdge_t;
			typedef std::vector <DelayedEdge_t> DelayedEdges_t;


			DelayedEdges_t delayedEdges;
			DevicePtr_t robot (problem ().robot ());
			PathValidationPtr_t pathValidation (problem ().pathValidation ());
			Nodes_t newNodes;
			PathPtr_t validPath, path;
			// Pick a random node
			ConfigurationPtr_t q_rand = configurationShooter_->shoot ();
			//
			// First extend each connected component toward q_rand
			//
			for (ConnectedComponents_t::const_iterator itcc =
					roadmap ()->connectedComponents ().begin ();
					itcc != roadmap ()->connectedComponents ().end (); ++itcc) {
				// Find nearest node in roadmap
				value_type distance;
//				(*actual_configuration_ptr_)[0] = (*q_rand)[0];
//				(*actual_configuration_ptr_)[1] = (*q_rand)[1];
//				(*actual_configuration_ptr_)[2] = (*q_rand)[2];
//				(*actual_configuration_ptr_)[3] = (*q_rand)[3];
//				(*actual_configuration_ptr_)[4] = (*q_rand)[4];
//				(*actual_configuration_ptr_)[5] = (*q_rand)[5];
//				(*actual_configuration_ptr_)[6] = (*q_rand)[6];
				cout << "extend to " << *actual_configuration_ptr_ << endl;

				*q_rand = *actual_configuration_ptr_;

				NodePtr_t near = roadmap ()->nearestNode (q_rand, *itcc, distance);
				path = extend (near, q_rand);	// modif nassime
				if (path) {
					bool pathValid = pathValidation->validate (path, false, validPath);
					// Insert new path to q_near in roadmap
					value_type t_final = validPath->timeRange ().second;
					if (t_final != path->timeRange ().first) {
						ConfigurationPtr_t q_new (new Configuration_t
								((*validPath) (t_final)));
						if (!pathValid || !belongs (q_new, newNodes)) {
							newNodes.push_back (roadmap ()->addNodeAndEdges
									(near, q_new, validPath));
						} else {
							// Store edges to add for later insertion.
							// Adding edges while looping on connected components is indeed
							// not recommended.
							delayedEdges.push_back (DelayedEdge_t (near, q_new, validPath));
						}
					}
				}
			}
			// Insert delayed edges
			for (DelayedEdges_t::const_iterator itEdge = delayedEdges.begin ();
					itEdge != delayedEdges.end (); ++itEdge) {
				const NodePtr_t& near = itEdge-> get <0> ();
				const ConfigurationPtr_t& q_new = itEdge-> get <1> ();
				const PathPtr_t& validPath = itEdge-> get <2> ();
				NodePtr_t newNode = roadmap ()->addNode (q_new);
				roadmap ()->addEdge (near, newNode, validPath);
				interval_t timeRange = validPath->timeRange ();
				roadmap ()->addEdge (newNode, near, validPath->extract
						(interval_t (timeRange.second ,
												 timeRange.first)));
			}

			//
			// Second, try to connect new nodes together
			//
			const SteeringMethodPtr_t& sm (problem ().steeringMethod ());
			for (Nodes_t::const_iterator itn1 = newNodes.begin ();
					itn1 != newNodes.end (); ++itn1) {
				for (Nodes_t::const_iterator itn2 = boost::next (itn1);
						itn2 != newNodes.end (); ++itn2) {
					ConfigurationPtr_t q1 ((*itn1)->configuration ());
					ConfigurationPtr_t q2 ((*itn2)->configuration ());
					assert (*q1 != *q2);
					path = (*sm) (*q1, *q2);
					if (path && pathValidation->validate (path, false, validPath)) {
						roadmap ()->addEdge (*itn1, *itn2, path);
						interval_t timeRange = path->timeRange ();
						roadmap ()->addEdge (*itn2, *itn1, path->extract
								(interval_t (timeRange.second,
														 timeRange.first)));
					}
				}
			}
		}

		void Planner::configurationShooter
			(const ConfigurationShooterPtr_t& shooter)
			{
				configurationShooter_ = shooter;
			}


	} // namespace interactive
} // namespace hpp


