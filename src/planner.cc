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

#include <hpp/interactive/sixDOFMouseDriver.hh>
#include <hpp/corbaserver/client.hh>
#include <hpp/corbaserver/robot.hh>

using namespace std;
using hpp::model::displayConfig;

namespace hpp {
	namespace interactive {

		// typedefs
		typedef se3::SE3::Vector3 Vector3;
		typedef se3::SE3::Matrix3 Matrix3;
		
		// static variables
		Configuration_t Planner::actual_configuration_;
		ConfigurationPtr_t Planner::actual_configuration_ptr_;
		boost::mutex Planner::mutex_;
		double Planner::random_prob_;
		short int Planner::iteration_; // unused

		// gloobal variables
		graphics::corbaServer::ClientCpp p;

		// functions
		bool belongs (const ConfigurationPtr_t& q, const Nodes_t& nodes);
		void InteractiveDeviceThread();




		PlannerPtr_t Planner::createWithRoadmap
			(const Problem& problem, const RoadmapPtr_t& roadmap)
			{
				Planner* ptr = new Planner (problem, roadmap);
				Planner::random_prob_ = 0.6; // 0 all human 1 all machine
				return PlannerPtr_t (ptr);
			}






Planner::Planner (const Problem& problem,
				const RoadmapPtr_t& roadmap) :
			PathPlanner (problem, roadmap),
			configurationShooter_ (new BasicConfigurationShooter (problem.robot ())),
			qProj_ (problem.robot ()->configSize ())
		{
			std::cout << "read interactive device thread beginning\n";

			std::ofstream myfile;
			ConfigurationPtr_t config (new Configuration_t ((hpp::model::size_type)7));
			(*config)[0] = 0;
			(*config)[1] = 0;
			(*config)[2] = 0;
			(*config)[3] = 1;
			(*config)[4] = 0;
			(*config)[5] = 0;
			(*config)[6] = 0;

			Planner::actual_configuration_ptr_ = config;

			float f = (float) 0.1;

			// add landmark to viewer
			p.createGroup ("scene_hpp_");
			gepetto::corbaserver::Color color;
			color[0] = 1;	color[1] = 1;	color[2] = 1;	color[3] = 1.;

			p.addBox ("scene_hpp_/curseur", f/10,f/10,f/10, color);
			p.addLandmark("scene_hpp_/curseur", 1.);
			p.addSceneToWindow ("scene_hpp_", 0);

//			hpp::corbaserver::Client::robot_ ;
		
			ConfigurationPtr_t q_rand = configurationShooter_->shoot (); // décale le rand initial
			SixDOFMouseDriver::MouseInit();
			boost::thread th(InteractiveDeviceThread);


		}


		void InteractiveDeviceThread(){

			while(1){

				// get transformation from device
				se3::SE3 trans_temp = SixDOFMouseDriver::getTransformation();

				// apply transformation to the cursor
				p.applyConfiguration("scene_hpp_/curseur", trans_temp);

				//					cout << "dans le planneuur " << 
				//						trans_temp << endl;
				//						//	(*actual_configuration_ptr_)[0] =
				//						trans_temp.translation()[0] << " " <<
				//						//		(*actual_configuration_ptr_)[1] =
				//						trans_temp.translation()[1] << " " << 
				//						//		(*actual_configuration_ptr_)[2] =
				//						trans_temp.translation()[2] << endl;

				//					setActConf(0, trans_temp.translation()[0]);
				//					setActConf(1, trans_temp.translation()[1]);
				//					setActConf(2, trans_temp.translation()[2]);

				// save current transformation in the planner's memory
				(*Planner::actual_configuration_ptr_)[0] =
					trans_temp.translation()[0];
				(*Planner::actual_configuration_ptr_)[1] =
					trans_temp.translation()[1];
				(*Planner::actual_configuration_ptr_)[2] =
					trans_temp.translation()[2];

				Eigen::Matrix3f a(trans_temp.rotation());

//			Eigen::Quaternionf q(a);
//			Eigen::Quaternionf q(fcl::Quaternion3f::toRotation(trans_temp.rotation()));
//				std::cout << "mat " << a << std::endl;
//				std::cout << "quat " << q.norm() << " " << q.x() << " " << q.y() << " " << q.z() << " " << std::endl;
//				(*Planner::actual_configuration_ptr_)[3] = q.norm();
//				(*Planner::actual_configuration_ptr_)[4] = q.x();
//				(*Planner::actual_configuration_ptr_)[5] = q.y();
//				(*Planner::actual_configuration_ptr_)[6] = q.z();

				// show modifications on screen
				p.refresh();
			}
		}


		
		
/* 
 * 
*/
		void Planner::oneStep ()
		{
			
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
//				cout << "extend to " << *actual_configuration_ptr_ << endl;

				double rando = rand();
				rando = rando / RAND_MAX;
				
				if ( rando < Planner::random_prob_)
				{
					// keep random config
//					usleep(1000);
				}
				else{
					//mutex_.lock();
					*q_rand = *Planner::actual_configuration_ptr_;
					//mutex_.unlock();
				}
				NodePtr_t near = roadmap ()->nearestNode (q_rand, *itcc, distance);
				path = extend (near, q_rand);
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
			//#aorthey owns your code _°°°_ 
		}



		void Planner::configurationShooter
			(const ConfigurationShooterPtr_t& shooter)
			{
				configurationShooter_ = shooter;
			}




	} // namespace interactive
} // namespace hpp


