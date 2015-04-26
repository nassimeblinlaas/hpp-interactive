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

#ifndef HPP_INTERACTIVE_FWD_HH
# define HPP_INTERACTIVE_FWD_HH

# include <hpp/interactive/config.hh>
# include <hpp/core/fwd.hh>

namespace hpp {
  namespace interactive {
    HPP_PREDEF_CLASS (Planner);
    typedef boost::shared_ptr <Planner> PlannerPtr_t;

    typedef core::Problem Problem;
    typedef core::Roadmap Roadmap;
    typedef core::RoadmapPtr_t RoadmapPtr_t;
    typedef core::ConfigurationShooterPtr_t ConfigurationShooterPtr_t;
    typedef core::ConfigurationPtr_t ConfigurationPtr_t;
    typedef core::NodePtr_t NodePtr_t;
    typedef core::Configuration_t Configuration_t;
    typedef core::PathPtr_t PathPtr_t;
    typedef core::BasicConfigurationShooter BasicConfigurationShooter;
    typedef core::Nodes_t Nodes_t;
    typedef core::SteeringMethodPtr_t SteeringMethodPtr_t;
    typedef core::ConstraintSetPtr_t ConstraintSetPtr_t;
    typedef core::ConfigProjectorPtr_t ConfigProjectorPtr_t;
    typedef core::DevicePtr_t DevicePtr_t;
    typedef core::PathValidationPtr_t PathValidationPtr_t;
    typedef core::ConnectedComponents_t ConnectedComponents_t;
    typedef core::value_type value_type;
    typedef core::interval_t interval_t;
  } // namespace interactive
} // namespace hpp

#endif // HPP_INTERACTIVE_FWD_HH
