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
#include <hpp/fcl/distance.h>

# include <hpp/core/path-planner.hh>

namespace hpp {
  namespace interactive {
    /// \addtogroup path_planning
    /// \{

    /// Generic implementation of RRT algorithm
    class HPP_INTERACTIVE_DLLAPI Planner : public core::PathPlanner
    {
    public:
      /// Return shared pointer to new object.
      static PlannerPtr_t createWithRoadmap
	(const Problem& problem, const RoadmapPtr_t& roadmap);
      /// Return shared pointer to new object.
      static PlannerPtr_t create (const Problem& problem);
      /// One step of extension.
      virtual void oneStep ();
      /// Set configuration shooter.
      void configurationShooter (const ConfigurationShooterPtr_t& shooter);

      void InteractiveDeviceThread();
      fcl::DistanceResult FindNearestObstacle();
      void ShowBounds();

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
    private:
      ConfigurationShooterPtr_t configurationShooter_;
      mutable Configuration_t qProj_;
      PlannerWkPtr_t weakPtr_;

      ConfigurationPtr_t actual_configuration_ptr_;
      bool random_prob_;
      graphics::corbaServer::Client client_;
      static short int nb_lauchs; // static because value kept

      boost::mutex robot_mutex_;
      bool exist_obstacle_;
      bool contact_activated_;  // enabling contact algorithm
      bool mode_contact_; // entering contact mode
      double quat_[4]; 

      fcl::Vec3f org_;    // orig pt référence pour gram schmidt
      fcl::Vec3f obj_;    // point de l'objet le plus proche de l'obstacle
      float distance_;    // distance centre du robot
      float distances_[3];   // distances centre du robot
      boost::mutex distance_mutex_; // protège l'accès à distance_
      fcl::CollisionObject* o2ptr;
      double repere_local_[3][3];
      ::Eigen::Vector3f NewMinBounds;
      ::Eigen::Vector3f NewMaxBounds;
      ::Eigen::Vector3f min;
      ::Eigen::Vector3f max;
      short int iteration_;
      short int type_; //device type 1 mouse 2 sigma7

    };
    /// \}
  } // namespace interactive 
} // namespace hpp
#endif // HPP_INTERACTIVE_PLANNER_HH

