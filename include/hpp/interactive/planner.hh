// Copyright (c) 2014 CNRS
// Authors: Nassime BLIN
//
// This file is part of hpp-interactive
// hpp-interactive is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-interactive is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-interactive  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_INTERACTIVE_PLANNER_HH
#define HPP_INTERACTIVE_PLANNER_HH

# include <vector>
# include <boost/tuple/tuple.hpp>
# include <boost/thread/thread.hpp>
# include <hpp/interactive/fwd.hh>
# include <hpp/core/path-planner.hh>
#include <boost/thread/mutex.hpp>

#include <iostream>
#include <fstream>

#include <gepetto/viewer/corba/client.hh>

namespace hpp {
  namespace interactive {
    //using core::PathPlanner;
    /// Generic implementation of RRT algorithm
    class HPP_INTERACTIVE_DLLAPI Planner : public core::PathPlanner
    {
    public:
      /// Return shared pointer to new object.
      static PlannerPtr_t createWithRoadmap
	( const Problem& problem,  const RoadmapPtr_t& roadmap);
      /// Return shared pointer to new object.
      static PlannerPtr_t create ( const Problem& problem);
      /// One step of extension.
      virtual void oneStep ();
      /// Set configuration shooter.
      void configurationShooter (const ConfigurationShooterPtr_t& shooter);
			
			void InteractiveDeviceInit();
			static void ReadInteractiveDevice(void* arg);
			static void getData();

			void setActConf(int n, double val){actual_configuration_[n]=val;};
			static ConfigurationPtr_t actual_configuration_ptr_;

    protected:
      /// Constructor
      Planner (const Problem& problem, const RoadmapPtr_t& roadmap);
      /// Constructor with roadmap
      Planner (const Problem& problem);
      /// Store weak pointer to itself
      void init (const PlannerWkPtr_t& weak);
      /// Extend a node in the direction of a configuration
      /// \param near node in the roadmap,
      /// \param target target configuration
      virtual PathPtr_t extend (const NodePtr_t& near,
				const ConfigurationPtr_t& target);

			/*/
			const RoadmapPtr_t& getRoadmap(){
				return roadmap_;
			};*/

    private:
      ConfigurationShooterPtr_t configurationShooter_;
      mutable Configuration_t qProj_;
      PlannerWkPtr_t weakPtr_;

			// file descriptor for interactive device
			static int fd_;
			// memory buffer for interactive device
			static char data_[14];
			// thread for reading interactive device
			static boost::thread* interactiveDeviceThread_;			
			// converted position and orientation values table
			static short int deviceValues_[6];
			static float deviceValuesNormalized_[6];
			static RoadmapPtr_t roadmap_i;
			static const Problem* problem_i;
			static Configuration_t actual_configuration_;

			static double random_prob_;
			static short int iteration_;
		
			static boost::mutex mutex_;	

    };



  } // namespace interactive
} // namespace hpp
#endif // HPP_INTERACTIVE_PLANNER_HH

