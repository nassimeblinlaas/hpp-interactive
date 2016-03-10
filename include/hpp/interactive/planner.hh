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

namespace hpp {
  namespace interactive {
    //using core::PathPlanner;
    /// Generic implementation of RRT algorithm
    class HPP_INTERACTIVE_DLLAPI Planner : public core::PathPlanner
    {
        public:

        /// Return shared pointer to new object.
        static PlannerPtr_t createWithRoadmap
        (const Problem& problem,  const RoadmapPtr_t& roadmap);
        /// Return shared pointer to new object.
        static PlannerPtr_t create ( const Problem& problem);
        /// One step of extension.
        virtual void oneStep ();
        /// Set configuration shooter.
        void configurationShooter (const ConfigurationShooterPtr_t& shooter);

        void InteractiveDeviceInit();
        static void ReadInteractiveDevice(void* arg);
        static void getData();

        //void setActConf(int n, double val){actual_configuration_[n]=val;};

        // TODO geters/setters etmettre en private
        static ConfigurationPtr_t actual_configuration_ptr_;
        // true if an obstacle exist between robot and goal -> contact mode
        static bool exist_obstacle_;
        // repère local au plan tangeant au point le plus proche de l'obstacle
        static double repere_local_[3][3];

        Configuration_t GetActualConfiguration() const {
            return actual_configuration_;
        }

        fcl::DistanceResult FindNearestObstacle();

        protected:
        /// Constructor
        Planner (const Problem& problem, const RoadmapPtr_t& roadmap);
        /// Constructor with roadmap
        Planner (const Problem& problem);
        /// Store weak pointer to itself
        void init (const PlannerWkPtr_t& weak);
        /// Extend a node in the direction of a configuration
        /// \param near node in the roadmap,
        ///
        /// \param target target configuration
        virtual PathPtr_t extend (const NodePtr_t& near,
        const ConfigurationPtr_t& target);





        private:

        void ShowBounds();



        // void InteractiveDeviceThread(void* arg);

        ConfigurationShooterPtr_t configurationShooter_;
        mutable Configuration_t qProj_;
        PlannerWkPtr_t weakPtr_;


        // memory buffer for interactive device
        static char data_[14];
        // thread for reading interactive device
        static boost::thread* interactiveDeviceThread_;
        // converted position and orientation values table
        static short int deviceValues_[6];
        // values between 0 and 1
        static float deviceValuesNormalized_[6];

        //static RoadmapPtr_t roadmap_i;
        //static const Problem* problem_i;
        static Configuration_t actual_configuration_;

        // machine probability to shoot (= 1 - human probability)
        static double random_prob_;

    public :    // todo mettre le thread dans la classe pour enlever le public du mode
        //contact mode
        static bool mode_contact_;

        // unused
        static short int iteration_;
        static boost::mutex mutex_;
        // file descriptor for interactive device
        static int fd_;

    };



  } // namespace interactive
} // namespace hpp
#endif // HPP_INTERACTIVE_PLANNER_HH

