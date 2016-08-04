
//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux
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
#include <hpp/util/debug.hh>
#include <hpp/model/configuration.hh>
#include <hpp/model/device.hh>
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
#include <hpp/corbaserver/client.hh>
#include <gepetto/viewer/corba/client.hh>
#include <hpp/interactive/sixDOFMouseDriver.hh>
#include <hpp/model/collision-object.hh>

#include <hpp/interactive/gram-schmidt.hh>

typedef se3::SE3::Vector3 Vector3;
typedef se3::SE3::Matrix3 Matrix3;

::Eigen::Matrix3f quat2Mat(float x, float y, float z, float w){
  ::Eigen::Matrix3f ret;

  ret(0, 0) = 1 - 2*(float)(pow(y, 2) + pow(z, 2));
  ret(0, 1) = 2*x*y - 2*z*w;
  ret(0, 2) = 2*x*z + 2*y*w;
  ret(1, 0) = 2*x*y + 2*z*w;
  ret(1, 1) = 1 - 2*(float)(pow(x, 2) + pow(z, 2));
  ret(1, 2) = 2*y*z - 2*x*w;
  ret(2, 0) = 2*x*z - 2*y*w;
  ret(2, 1) = 2*y*z + 2*x*w;
  ret(2, 2) = 1 - 2*(float)(pow(x, 2) + pow(y, 2));

  return ret;
}

void euler2Quat(double psi, double theta, double phi, double* quat){
  psi/=2; theta/=2; phi/=2;

  quat[0] = cos(psi) * cos(theta) * cos(phi) + sin(psi) * sin(theta) * sin(phi);
  quat[1] = sin(psi) * cos(theta) * cos(phi) - cos(psi) * sin(theta) * sin(phi);
  quat[2] = cos(psi) * sin(theta) * cos(phi) + sin(psi) * cos(theta) * sin(phi);
  quat[3] = cos(psi) * cos(theta) * sin(phi) - sin(psi) * sin(theta) * cos(phi);
}

// normaliser quaternion
void normalizeQuat(double& w, double& x, double& y, double& z){
  double mag = sqrt(pow(w,2)+pow(x,2)+pow(y,2)+pow(z,2));
  w = w/mag; x = x/mag; y = y/mag; z = z/mag;
}



short int signe (double x) {
  return ((x < 0) ? -1 : 1);
}

namespace hpp {
  namespace interactive {
    using model::displayConfig;
    using namespace std;
    short int nb_launchs = 0;

    // returns an fcl distance request structure
    fcl::DistanceResult Planner::FindNearestObstacle(){
      fcl::DistanceRequest request(true, 0, 0, fcl::GST_INDEP);
      fcl::DistanceResult result;
      result.clear();
      fcl::CollisionObject* robot_proche;
      fcl::CollisionObject* obstacle_proche;
      fcl::CollisionObject* obst_temp;
      fcl::CollisionObject* robot_temp;
      hpp::core::ObjectVector_t liste = this->problem().collisionObstacles();
      double min_dist = 999;
      for (hpp::core::ObjectVector_t::iterator it_obst = liste.begin();it_obst!=liste.end();++it_obst){
        obst_temp = &*(*it_obst)->fcl();
        for (hpp::model::ObjectIterator it_rob = this->problem().robot()->objectIterator(hpp::model::COLLISION);
            !it_rob.isEnd(); ++it_rob){
          robot_temp = &*(*it_rob)->fcl();
          result.clear();
          fcl::distance(obst_temp, robot_temp, request, result);
          //cout << (*it_obst)->name() << "/" << (*it_rob)->name() << " " << result.min_distance << endl;
          if (result.min_distance<min_dist){
            if(result.min_distance==-1){
              1;//collision = true; TODO vérifier que tout va bien
            }

            robot_proche = robot_temp;
            obstacle_proche = obst_temp;
            min_dist = result.min_distance;
          }
        }
      }

      fcl::distance(obstacle_proche, robot_proche, request, result);
      return result;
    }

    PlannerPtr_t Planner::createWithRoadmap
    (const Problem& problem, const RoadmapPtr_t& roadmap)
    {
      Planner* ptr = new Planner (problem, roadmap);
      return PlannerPtr_t (ptr);
    }

    PlannerPtr_t Planner::create (const Problem& problem)
    {
      Planner* ptr = new Planner (problem);
      return PlannerPtr_t (ptr);
    }

    Planner::Planner (const Problem& problem):
      PathPlanner (problem),
      configurationShooter_ (problem.configurationShooter()),
      qProj_ (problem.robot ()->configSize ()),
      client_(0, NULL)
    {
    }

    Planner::Planner (const Problem& problem,
					const RoadmapPtr_t& roadmap) :
      PathPlanner (problem, roadmap),
      configurationShooter_ (problem.configurationShooter()),
      qProj_ (problem.robot ()->configSize ()),
      client_(0, NULL)
    {

      nb_launchs++;
      type_ = 2; //device type 1 mouse 2 sigma7
      random_prob_ = 0; // 0 all human  1 all machine
      mode_contact_ = false;
      // adding interactive robot and positionning it
      client_.connect();
      cout << "adding landmark to viewer\n";
      //string robot_name = "/hpp/src/hpp_tutorial/urdf/robot_mesh.urdf"; contact_activated_ = true;
      string robot_name = "/hpp/src/hpp_tutorial/urdf/robot_3angles.urdf"; contact_activated_ = true;
      //string robot_name = "/hpp/src/hpp_tutorial/urdf/robot_chaise.urdf"; contact_activated_ = true;
      float f = (float) 0.1;
      gepetto::corbaserver::Color color;
      color[0] = 1; color[1] = 1; color[2] = 1; color[3] = 1.;
      client_.gui()->addBox ("0_scene_hpp_/curseur", f/10,f/10,f/10, color);
      //client_.gui()->addSceneToWindow ("0_scene_hpp_", 0);
      client_.gui()->addLandmark("0_scene_hpp_/curseur", 1.);
      client_.gui()->addURDF("0_scene_hpp_/robot_interactif", robot_name.data() ,"/hpp/install/share");
      ::gepetto::corbaserver::Transform tr;
      tr[0] = 1; tr[1] = 1; tr[2] = 1;
      client_.gui()->applyConfiguration("0_scene_hpp_/curseur", tr);
      client_.gui()->applyConfiguration("0_scene_hpp_/robot_interactif", tr);
      this->problem().robot()->computeForwardKinematics();
      client_.gui()->refresh();
      ConfigurationPtr_t q_rand = configurationShooter_->shoot ();
      // init mouse driver
      double bounds[6] = {
        this->problem().robot()->rootJoint()->lowerBound(0),
        this->problem().robot()->rootJoint()->upperBound(0),
        this->problem().robot()->rootJoint()->lowerBound(1),
        this->problem().robot()->rootJoint()->upperBound(1),
        this->problem().robot()->rootJoint()->lowerBound(2),
        this->problem().robot()->rootJoint()->upperBound(2)
      };
      SixDOFMouseDriver::MouseInit(type_, bounds);
      ConfigurationPtr_t config (new Configuration_t ((hpp::model::size_type)7));
      (*config)[0] = 1;
      (*config)[1] = 1;
      (*config)[2] = 1;
      (*config)[3] = 1;
      (*config)[4] = 0;
      (*config)[5] = 0;
      (*config)[6] = 0;
      actual_configuration_ptr_ = config;
      // launch interactive thread 
      
      if (nb_launchs<2)
        boost::thread th(boost::bind( &Planner::InteractiveDeviceThread,this));
    }

    void Planner::InteractiveDeviceThread(){
      gepetto::corbaserver::Color color;
      color[0] = 1; color[1] = 1; color[2] = 1; color[3] = 1.;
      int index_lignes = 0;

      bool init = false;
      while(1){
        if (SixDOFMouseDriver::HasMoved() && nb_launchs<2){
          if (!init){
            const ConfigurationPtr_t initConfig_ = this->problem().initConfig();
            //double translations[3] = {
              //(*initConfig_)[0],
              //(*initConfig_)[1],
              //(*initConfig_)[2]
            //};
            //SixDOFMouseDriver::InitPosition(translations);
            init = true;
          }

          // get data from 6D mouse
          se3::SE3 trans_temp = SixDOFMouseDriver::getTransformation();
          // conversion matrice -> quaternion
          Eigen::Matrix3f mat = trans_temp.rotation();
          Eigen::Quaternionf quat(mat);
          // normaliser quaternion
          //quat_[0]=quat.w();quat_[1]=quat.x();quat_[2]=quat.y();quat_[3]=quat.z();
          //normalizeQuat(quat_[0],quat_[1], quat_[2], quat_[3]);
          double mag = sqrt(pow(quat.w(),2)+pow(quat.x(),2)+pow(quat.y(),2)+pow(quat.z(),2));
          ::gepetto::corbaserver::Transform tr;
          if(type_==1){
            tr[0] = trans_temp.translation()[0];
            tr[1] = trans_temp.translation()[1];
            tr[2] = trans_temp.translation()[2];
          }
          if (type_==2){
            Eigen::Vector3f translate;
            translate << trans_temp.translation()[0], 
                      trans_temp.translation()[1],
                      trans_temp.translation()[2];
            Eigen::Matrix3f rot90x;
            Eigen::Matrix3f rot90y;
            Eigen::Matrix3f rot90z;
            double angle = 3.14159265/2;
            rot90z << cos(angle), -sin(angle), 0, 
                   sin(angle), cos(angle),  0,
                   0,          0,           1;
            rot90x << 1, 0, 0,
                   0, cos(angle), -sin(angle),
                   0, sin(angle), cos(angle);
            rot90y << cos(angle), 0, sin(angle),
                   0, 1, 0,
                   -sin(angle), 0, cos(angle);  


            //translate = translate.transpose() * rot90y;
            tr[0] = translate[0]; // version sigma7
            tr[1] = translate[1];
            tr[2] = translate[2];
            //tr[0] = trans_temp.translation()[0];
            //tr[1] = trans_temp.translation()[1];
            //tr[2] = trans_temp.translation()[2];
          }
          tr[3] = (float)quat.w()/(float)mag;
          tr[4] = (float)quat.x()/(float)mag;
          tr[5] = (float)quat.y()/(float)mag;
          tr[6] = (float)quat.z()/(float)mag;

          // save current transfo-rmation in the planner's memory
          (*Planner::actual_configuration_ptr_)[0] = tr[0];
          (*Planner::actual_configuration_ptr_)[1] = tr[1];
          (*Planner::actual_configuration_ptr_)[2] = tr[2];
          (*Planner::actual_configuration_ptr_)[3] = tr[3];
          (*Planner::actual_configuration_ptr_)[4] = tr[4];
          (*Planner::actual_configuration_ptr_)[5] = tr[5];
          (*Planner::actual_configuration_ptr_)[6] = tr[6];
          //(*Planner::actual_configuration_ptr_)[3] = quat_[0];
          //(*Planner::actual_configuration_ptr_)[4] = quat_[1];
          //(*Planner::actual_configuration_ptr_)[5] = quat_[2];
          //(*Planner::actual_configuration_ptr_)[6] = quat_[3];
          /*
             cout << "config curseur        " <<
          //    trans_temp << endl;
          //    //  (*actual_configuration_ptr_)[0] =
          trans_temp.translation()[0] << " " <<
          //    (*actual_configuration_ptr_)[1] =
          trans_temp.translation()[1] << " " <<
          //    (*actual_configuration_ptr_)[2] =
          trans_temp.translation()[2] << endl;
          //*/

          // afficher le robot
          client_.gui()->applyConfiguration("0_scene_hpp_/robot_interactif", tr);
          client_.gui()->applyConfiguration("0_scene_hpp_/curseur", tr);

          // get camera vectors to align cursor with viewer
          unsigned long id = client_.gui()->getWindowID("window_hpp_");
          gepetto::corbaserver::floatSeq* CamVects;
          unsigned short int dim = 4;
          CamVects = new gepetto::corbaserver::floatSeq();
          CamVects->length(dim);
          CamVects = client_.gui()->getCameraVectors((float)id, "");
          ::Eigen::Matrix3f camMat = quat2Mat(CamVects->get_buffer()[0],CamVects->get_buffer()[1],
              CamVects->get_buffer()[2],CamVects->get_buffer()[3]);
          SixDOFMouseDriver::setCameraVectors(
              camMat(0,0), camMat(0,1), camMat(0,2),
              camMat(1,0), camMat(1,1), camMat(1,2),
              camMat(2,0), camMat(2,1), camMat(2,2)
              );



          // using solveanddisplay relaunches planner -> anti core dump protection
          if (nb_launchs<2){
            //*
            // caler le robot au niveau du curseur
            robot_mutex_.lock();                // TODO mutex inoptimal !
            hpp::model::Configuration_t sauv = problem().robot()->currentConfiguration();
            hpp::model::Configuration_t in = sauv;
            in[0] = trans_temp.translation()[0];
            in[1] = trans_temp.translation()[1];
            in[2] = trans_temp.translation()[2];
            in[0] = tr[0];
            in[1] = tr[1];
            in[2] = tr[2];
            in[3] = tr[3];
            in[4] = tr[4];
            in[5] = tr[5];
            in[6] = tr[6];
            hpp::model::ConfigurationIn_t in_t(in);
            problem().robot()->currentConfiguration(in_t);
            problem().robot()->computeForwardKinematics();

            /*
               cout << " robot " <<
               (*arg_->problem().robot()->objectIterator(hpp::model::COLLISION))->name() << " tr "
               << (*arg_->problem().robot()->objectIterator(hpp::model::COLLISION))->getTransform().getTranslation()
               << endl;
            //*/

            // méthode de recherche du plus proche obst par itération
            fcl::DistanceResult result;
            bool collision = false;
            result = FindNearestObstacle();
            collision = result.min_distance == -1 ? true : false;

            mode_contact_ = false;
            robot_mutex_.unlock();

            //*
            // //////////////////////////////////////////////////////////////////
            if (contact_activated_ && !collision){

              //cout << " dist obstacle " << result.min_distance << std::endl;

              // enregistrer les coordonnées des extrémités du segment robot/obstacle
              // point sur le robot
              gepetto::corbaserver::Position v = {
                (float)result.nearest_points[0][0],
                (float)result.nearest_points[0][1],
                (float)result.nearest_points[0][2]};
              // point sur l'obstacle
              gepetto::corbaserver::Position w = {
                (float)result.nearest_points[1][0],
                (float)result.nearest_points[1][1],
                (float)result.nearest_points[1][2]};

              //const gepetto::corbaserver::Position* v = &v_;
              //const gepetto::corbaserver::Position* w = &w_;

              // algorithme de GRAM-SCHMIDT
              if (result.min_distance != -1){
                exist_obstacle_ = true;


                Mat33f A;
                // normale

                A.col[0] = Vec3f(w[0]-v[0], w[1]-v[1], w[2]-v[2]);

                // distance du centre de l'objet à sa surface
                if (distance_mutex_.try_lock()) // TODO mutex inoptimal
                {


                  Eigen::Vector3f normal(
                      (float)(result.nearest_points[0][0] - result.nearest_points[1][0]),
                      (float)(result.nearest_points[0][1] - result.nearest_points[1][1]),
                      (float)(result.nearest_points[0][2] - result.nearest_points[1][2])
                      );

                  //cout << "normale " << normal(0)<< " " << normal(1) << " "<< normal(2) << endl;
                  for (int i=0; i< 3; i++){
                    if (abs(normal(i))<(1e-10)) normal(i) = 0;
                  }
                  //cout << "normale " << normal(0)<< " " << normal(1) << " "<< normal(2) << endl;

                  float norm_of_normal = sqrt( pow(normal(0), 2) +
                      pow(normal(1), 2) +
                      pow(normal(2), 2) );
                  //cout << "norm of normale " << norm_of_normal << endl;
                  normal(0) = normal(0)/norm_of_normal;
                  normal(1) = normal(1)/norm_of_normal;
                  normal(2) = normal(2)/norm_of_normal;
                  //cout << "normale " << normal(0)<< " " << normal(1) << " "<< normal(2) << endl;
                  Eigen::Vector3f d_com_near_point(
                      (float)result.nearest_points[1][0] - trans_temp.translation()[0],
                      (float)result.nearest_points[1][1] - trans_temp.translation()[1],
                      (float)result.nearest_points[1][2] - trans_temp.translation()[2]);
                  //cout << "d_com_near_point " << d_com_near_point(0)<< " " << d_com_near_point(1) << " "<< d_com_near_point(2) << endl;
                  distances_[0] = (float) (0.0 + normal(0) * d_com_near_point(0));
                  distances_[1] = (float) (0.0 + normal(1) * d_com_near_point(1));
                  distances_[2] = (float) (0.0 + normal(2) * d_com_near_point(2));
                  //cout << "distances_ " << distances_[0]  << " " << distances_[1] << " " <<
                  //         distances_[2] << endl;



                  distance_ = sqrt(
                      pow((float)result.nearest_points[1][0] - trans_temp.translation()[0], 2) +
                      pow((float)result.nearest_points[1][1] - trans_temp.translation()[1], 2) +
                      pow((float)result.nearest_points[1][2] - trans_temp.translation()[2], 2));
                  //distance_ = 0.05;


                  distance_mutex_.unlock();
                }




                // vecteur aléatoire 1
                double rando1 = rand(), rando2 = rand(), rando3 = rand();
                rando1 = rando1 / RAND_MAX; rando2 = rando2 / RAND_MAX; rando3 = rando3 / RAND_MAX;
                A.col[1] = Vec3f((float)rando1,(float)rando2, (float)rando3);

                // vecteur aléatoire 2
                rando1 = rand(); rando2 = rand(); rando3 = rand();
                rando1 = rando1 / RAND_MAX; rando2 = rando2 / RAND_MAX; rando3 = rando3 / RAND_MAX;
                A.col[2] = Vec3f((float)rando1,(float)rando2, (float)rando3);


                // calcule la matrice de rotation MGS si on n'est pas déjà en mode contact
                if (!Planner::mode_contact_)
                  modified_gram_schmidt(MGS, A);

                //print_mat("MGS", MGS);



                // afficher le repère local // //////////////////////////////////////////
                string nom_ligne = "0_scene_hpp_/ligne";
                string ind = boost::lexical_cast<std::string>(index_lignes);
                nom_ligne += ind;

                if (index_lignes > 0){
                  //cout << "index " << index_lignes << " obj à cacher " << nom_ligne << endl;
                  client_.gui()->setVisibility(nom_ligne.c_str(), "OFF");
                  string axe = nom_ligne +='a';
                  client_.gui()->setVisibility(axe.c_str(), "OFF");
                  axe = nom_ligne +='b';
                  client_.gui()->setVisibility(axe.c_str(), "OFF");
                }

                index_lignes++;
                nom_ligne = "0_scene_hpp_/ligne";
                ind = boost::lexical_cast<std::string>(index_lignes);
                nom_ligne += ind;
                client_.gui()->addLine(nom_ligne.c_str(), v, w, &color[0]);

                // afficher les deux axes manquants du repère
                w[0] = v[0] + MGS.col[1].v[0];
                w[1] = v[1] + MGS.col[1].v[1];
                w[2] = v[2] + MGS.col[1].v[2];
                string axe = nom_ligne +='a';
                client_.gui()->addLine(axe.c_str(), v, w, &color[0]);
                w[0] = v[0] + MGS.col[2].v[0];
                w[1] = v[1] + MGS.col[2].v[1];
                w[2] = v[2] + MGS.col[2].v[2];
                axe = nom_ligne +='b';
                client_.gui()->addLine(axe.c_str(), v, w, &color[0]);
                // //////////////////////////////////////////////////////////////



                ::Eigen::Matrix3f MGS_;
                MGS_ << MGS.col[0].v[0],
                     MGS.col[1].v[0],
                     MGS.col[2].v[0],
                     MGS.col[0].v[1],
                     MGS.col[1].v[1],
                     MGS.col[2].v[1],
                     MGS.col[0].v[2],
                     MGS.col[1].v[2],
                     MGS.col[2].v[2];

                NewMinBounds = MGS_*min;
                NewMaxBounds = MGS_*max;


                //cout << "d=" << result.min_distance << " \n";//<< std::endl;
                if (result.min_distance<0.15 && !Planner::mode_contact_){
                  //if (0){
                  //cout << "distance inférieure à 0.15" << std::endl;
                  //std::cout << " pt0 " << result.nearest_points[0] <<
                  //             " pt1 " << result.nearest_points[1] << std::endl;


                  // sur l'obstacle
                  org_ = result.nearest_points[0];
                  //org_mutex_.unlock();

                  // sur le robot
                  obj_ = result.nearest_points[1];

                  Planner::mode_contact_ = true;
                }

                }// fin gram schmidt

                //}// fin for paires de collision

            }// fin activation contact
            //*/
          } // fin nb launch
          //*/

          // show modifications on screen
          client_.gui()->refresh();

        }// fin if has_moved_
      }// fin while(1) 

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


    /// This method performs one step of RRT extension as follows
    ///  1. a random configuration "q_rand" is shot,
    ///  2. for each connected component,
    ///    2.1. the closest node "q_near" is chosen,
    ///    2.2. "q_rand" is projected first on the tangent space of the
    ///         non-linear constraint at "q_near", this projection yields
    ///         "q_tmp", then "q_tmp" is projected on the non-linear constraint
    ///         manifold as "q_proj" (method extend)
    ///    2.3. the steering method is called between "q_near" and "q_proj" that
    ///         returns "path",
    ///    2.4. a valid connected part of "path", called "validPath" starting at
    ///         "q_near" is extracted, if "path" is valid (collision free),
    ///         the full "path" is returned, "q_new" is the end configuration of
    ///         "validPath",
    ///    2.5  a new node containing "q_new" is added to the connected
    ///         component and a new edge is added between nodes containing
    ///         "q_near" and "q_new".
    ///  3. Try to connect new nodes together using the steering method and
    ///     the current PathValidation instance.
    ///
    ///  Note that edges are actually added to the roadmap after step 2 in order
    ///  to avoid iterating on the list of connected components while modifying
    ///  this list.

    void Planner::oneStep ()
    {
      robot_mutex_.lock();
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
      double rando = rand();
      rando = rando / RAND_MAX;
      // keep random config
      if (rando > Planner::random_prob_)
      {
        if (Planner::mode_contact_){
          cout << rando << " contact q \n";
          Matrix3 rot;
          rot(0,0) = MGS.col[0].v[0];
          rot(0,1) = MGS.col[0].v[1];
          rot(0,2) = MGS.col[0].v[2];
          rot(1,0) = MGS.col[1].v[0];
          rot(1,1) = MGS.col[1].v[1];
          rot(1,2) = MGS.col[1].v[2];
          rot(2,0) = MGS.col[2].v[0];
          rot(2,1) = MGS.col[2].v[1];
          rot(2,2) = MGS.col[2].v[2];
          // garder z à zéro
          Vector3 val(0, (float)(*q_rand)[0], (float)(*q_rand)[2]);
          //cout << "rot " << rot << endl;
          //cout << "val " << val.transpose() << endl;
          //std::cout << "one step distance centre/surf " << distance_ << std::endl;
          //cout << "org " << org_[1] << " obj " << obj_[1]
          //     << " signe org-obj " << signe(org_[1]-obj_[1]) << endl;

          distance_mutex_.try_lock();
          Vector3 org(
              (float)org_[0]+signe(obj_[0]-org_[0])*distances_[0],
              (float)org_[1]+signe(obj_[1]-org_[1])*distances_[1],
              (float)org_[2]+signe(obj_[2]-org_[2])*distances_[2]
              );
          //cout << "org " << org.transpose() << endl;
          val = rot.transpose()*val + org;
          //cout << "nouveau val " << val.transpose() << endl;

          // gros hack
          //bool proche = false;
          while(1){
            double dist;
            dist = sqrt(
                pow(val[0]-(*Planner::actual_configuration_ptr_)[0], 2)+
                pow(val[1]-(*Planner::actual_configuration_ptr_)[1], 2)+
                pow(val[2]-(*Planner::actual_configuration_ptr_)[2], 2)
                );
            if (dist<2){
              break;
            }
            else{
              //cout << "retente\n";
              q_rand = configurationShooter_->shoot ();
              val[0] = 0;
              val[1] = (float)(*q_rand)[1];
              val[2] = (float)(*q_rand)[2];
              Vector3 re_org((float)org_[0]+signe(obj_[0]-org_[0])*distances_[0],
                  (float)org_[1]+signe(obj_[1]-org_[1])*distances_[1],
                  (float)org_[2]+signe(obj_[2]-org_[2])*distances_[2]
                  );
              val = rot.transpose()*val + re_org;
            }
          }
          distance_mutex_.unlock();

          (*q_rand)[0] = val[0];
          (*q_rand)[1] = val[1];
          (*q_rand)[2] = val[2];
          // fixer rotation
          (*q_rand)[3] = (*Planner::actual_configuration_ptr_)[3];
          (*q_rand)[4] = (*Planner::actual_configuration_ptr_)[4];
          (*q_rand)[5] = (*Planner::actual_configuration_ptr_)[5];
          (*q_rand)[6] = (*Planner::actual_configuration_ptr_)[6];

          Planner::iteration_++;
          if(Planner::iteration_ == 5){
            Planner::mode_contact_ = false;
            // ancien emplacement de distance_mutex_.unlock();
          }

  
        }
        else{
          *q_rand = *actual_configuration_ptr_; 
        }
      }

      for (ConnectedComponents_t::const_iterator itcc =
	     roadmap ()->connectedComponents ().begin ();
	   itcc != roadmap ()->connectedComponents ().end (); ++itcc) {
	// Find nearest node in roadmap
	value_type distance;
	NodePtr_t near = roadmap ()->nearestNode (q_rand, *itcc, distance);
	path = extend (near, q_rand);
	if (path) {
	  core::PathValidationReportPtr_t report;
	  bool pathValid = pathValidation->validate (path, false, validPath,
						     report);
	  // Insert new path to q_near in roadmap
	  value_type t_final = validPath->timeRange ().second;
	  if (t_final != path->timeRange ().first) {
	    ConfigurationPtr_t q_new (new Configuration_t
				      (validPath->end ()));
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
	  core::PathValidationReportPtr_t report;
	  if (path && pathValidation->validate (path, false, validPath,
						report)) {
	    roadmap ()->addEdge (*itn1, *itn2, path);
	    interval_t timeRange = path->timeRange ();
	    roadmap ()->addEdge (*itn2, *itn1, path->extract
				 (interval_t (timeRange.second,
					      timeRange.first)));
	  }
	}
      }
    robot_mutex_.unlock();
    }

    void Planner::configurationShooter
    (const ConfigurationShooterPtr_t& shooter)
    {
      configurationShooter_ = shooter;
    }


  } // namespace core
} // namespace hpp


          /*
             cout << "config curseur        " <<
          //    trans_temp << endl;
          //    //  (*actual_configuration_ptr_)[0] =
          trans_temp.translation()[0] << " " <<
          //    (*actual_configuration_ptr_)[1] =
          trans_temp.translation()[1] << " " <<
          //    (*actual_configuration_ptr_)[2] =
          trans_temp.translation()[2] << endl;
          //*/
