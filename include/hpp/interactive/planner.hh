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
			static void getData(std::ofstream& file);

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

			const RoadmapPtr_t& getRoadmap(){
				return roadmap_;
			};

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
			static ConfigurationPtr_t actual_configuration_ptr_;

		int bravo;	

    };



  } // namespace interactive
} // namespace hpp
#endif // HPP_INTERACTIVE_PLANNER_HH




















// test pour classe templatée
//   namespace interactive {
//
//		template <typename T> class Planner;
//
//		template <typename T>
//		struct PlannerPtr_t
//		{
//			typedef boost::shared_ptr<Planner<T> > type;
//		};
//
//		template<typename T>
//		class HPP_CORE_DLLAPI Planner : public T
//		{
//    public:
//      /// Return shared pointer to new object.
//      static typename PlannerPtr_t<T>::type create
//				(const Problem& problem);
//			/// Return shared pointer to new object.
//      static typename PlannerPtr_t<T>::type createWithRoadmap
//				(const Problem& problem, const RoadmapPtr_t& roadmap);
//      /// One step of extension.
//      virtual void oneStep ();
//      /// Set configuration shooter.
//      void configurationShooter (const ConfigurationShooterPtr_t& shooter);
//    protected:
//      // Constructor
//      Planner (const Problem& problem, const RoadmapPtr_t& roadmap);
//			// Store weak pointer to itself
//			void init (const PlannerWkPtr_t& weak);
//    private:
//      PlannerWkPtr_t weakPtr_;
//			ConfigurationShooterPtr_t configurationShooter_;
//
////      typedef boost::tuple <NodePtr_t, ConfigurationPtr_t, PathPtr_t>
////	DelayedEdge_t;
////      typedef std::vector <DelayedEdge_t> DelayedEdges_t;
//
//    }; // class Planner
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//			// coding inside header for template and compilation reason
//
//
//			// Constructor
//			template<typename T>
//			Planner<T>::Planner
//				(const Problem& problem, const RoadmapPtr_t& roadmap) :
//					PathPlanner(problem, roadmap),
//			configurationShooter_( new BasicConfigurationShooter (problem.robot()) )
//			{
//			}
//
//			template<typename T>
//			typename PlannerPtr_t<T>::type Planner<T>::create
//				(const Problem& problem)
//			{
//				Planner* ptr = new Planner (problem);
//				return PlannerPtr_t<T>(ptr);;;
//			}
//
//
//
//	 } // namespace interactive
