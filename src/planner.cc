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

#include <hpp/interactive/dhdc.h>
#include <hpp/interactive/drdc.h>
#include <hpp/interactive/gram-schmidt.hh>
#include <math.h>
#include <pthread.h>

typedef se3::SE3::Vector3 Vector3;
typedef se3::SE3::Matrix3 Matrix3;

extern Eigen::Matrix3f quat2Mat(float x, float y, float z, float w);
extern void euler2Quat(double psi, double theta, double phi, double* quat);
extern void normalizeQuat(double& w, double& x, double& y, double& z);

short int signe (double x) {
  return ((x < 0) ? -1 : 1);
}


namespace hpp {
  namespace interactive {
    using model::displayConfig;
    using namespace std;
    short int nb_launchs = 0;

    fcl::Vec3f org_temp;
    fcl::Vec3f prev_org_temp;
    fcl::Vec3f obj_temp;
    Eigen::Vector3f d_com_near_point_;
    double d_;      
    bool change_obst_;

    Eigen::Matrix3f camMat;
    Eigen::Vector3d deviceForce;
    Eigen::Vector3d deviceTorque;
    Eigen::Vector3f normal;
    
    boost::mutex normal_mutex;








void* ForceFeedback(void*){

  //clock_t begin, end, end2 ;
  Eigen::Vector3f temp, pos, force_vec, obst, normale;//, signes;
  Eigen::Vector3d obst_d;
  double err, prev_err, force, gain, D, kP, kD;
  bool force_feed = false;
  //long int iteration = 0;
  gain = 50;
  //double max=15;max++;
  sleep(2);
  while(1){
    obst.setZero();
    pos.setZero();
    temp.setZero();
    normal_mutex.lock();
    normale = normal;
    normal_mutex.unlock();
    //iteration++;
    pos = SixDOFMouseDriver::getTransformationNoMutex().translation();
    for (int i=0; i<3; i++){
      //signes[i]=signe(org_temp[i]-pos[i]);
      obst[i]=(float)org_temp[i]; 
    }
    // départ du plan P
    temp=obst-d_*normale;
    // équation du plan P: Ax+By+Cz+D=0;
    double aa = temp[0]*normale[0];
    double ab = temp[1]*normale[1];
    double ac = temp[2]*normale[2];
    //D = aa + ab + ac;
    //cout <<" aa "<<aa<<"="<<temp[0]<<"*"<<normale[0];
    //cout <<" ab "<<ab<<"="<<temp[1]<<"*"<<normale[1];
    //cout <<" ac "<<ac<<"="<<temp[2]<<"*"<<normale[2];
    //D = -D;
    D=-(temp[0]*normale[0]+temp[1]*normale[1]+temp[2]*normale[2]);
    // position du point proche de l'objet par rapport à P
    err = (pos[0]+d_com_near_point_[0])*normale[0]+
      (pos[1]+d_com_near_point_[1])*normale[1]+
      (pos[2]+d_com_near_point_[2])*normale[2]+D;
    //end = clock();
    //dt = double(end - begin) / CLOCKS_PER_SEC;
    //begin = clock();
    kP = err * gain;
    //kD = 0.250*gain*(err-prev_err);
    //prev_err = err;
    force = kP;//+kD;
    force_feed = err > 0;
    force_vec = normale*(float)force;
    if (force_feed){
      //force_vec.setZero();
    //cout << "pos="<<pos.transpose()<<" obst="<<obst.transpose()<<" n="<<normal.transpose()<<" temp="<<temp.transpose()<<" D="<<D<<" err="<< err <<" force="<<force_vec.transpose()<<"!!!!!!!!!!!!!"<<endl; 
      //cout << "force="<<force<<"=kP:"<<kP<<"+kD:"<<kD<<"\n";
      //t = clock();
      cout << "!!!!!!!!!!!!!!!!!!!!\n";
    }
    else{
      force_vec.setZero();
    //cout << "pos="<<pos.transpose()<<" obst="<<obst.transpose()<<" n="<<normal.transpose()<<" temp="<<temp.transpose()<<" D="<<D<<" err="<< err <<" force="<<force_vec.transpose()<<endl; 
      //cout <<"zero\n";
      cout << endl;
    }
    D = 0;
    //force_vec[1] = force_vec[0] = 0;

    /////// cout /////////////////////////////////////////////////////////////////////
    //FILE * pFile;
    //pFile = fopen ("log.csv","w");
    //double t;
    //double dt; 
    //clock_t begin, end;
    //fprintf (pFile, "t;pos[0];pos[1];pos[2];obst[0];obst[1];obst[2];normale[0];normale[1];normale[2];temp[0];temp[1];temp[2];D;err;force_vec[0];force_vec[1];force_vec[2]\n");
    //cout << normal.transpose()<< endl;
    //cout << " err="<< err <<" force="<<force_vec.transpose()<<endl; 
    //t = clock();
    //cout << "t="<<elapsed_secs<<endl;
    //if(iteration%10==1)
    //cout << "err="<<err<<"=Somme des pos="<<pos.transpose()<<"*normale="<<normale.transpose()<<"+D="<<D<<endl;
    //cout <<"pos "<<pos.transpose()<< " sgn " << signes[0]<<" "<<signes[1]<<" "<<signes[2] << " longs "<< d_com_near_point_.transpose()<< " d " << d_<< " errv1 " << err;
    //err = pos[0]*normal[0]+pos[1]*normal[1]+pos[2]*normal[2]+D;
    //cout <<"\terrv2 " << err;
    //err = (pos[0]+d_com_near_point_[0]*signes[0])*normal[0]+
      //(pos[1]+d_com_near_point_[1]*signes[1])*normal[1]+
      //(pos[2]+d_com_near_point_[2]*signes[2])*normal[2]+D;
    //cout <<"\terrv3 " << err<<endl;
      //cout << "obst="<<obst[1]<<" temp="<<temp[1]<<" pos="<<pos[1]<<" err"<<err<<" force="<<force_vec.transpose()<<" n "<<normal.transpose()<<endl;
    //fprintf (pFile, "%f;%f;%f;%f\n",t,pos[1],err*100,force_vec[1]);
    //fprintf (pFile, "%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f\n",t,pos[0],pos[1],pos[2],obst[0],obst[1],obst[2],normal[0],normal[1],normal[2],temp[0],temp[1],temp[2],D,err*100,force_vec[0],force_vec[1],force_vec[2]);
    ////////////////////////////////////////////////////////////////////////////////////

    //force_vec.setZero();
    dhdSetForce(force_vec[0], force_vec[1], -force_vec[2]);
  }
  return NULL;
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
      d_ = 0.10; // distance entrée mode contact
      Planner::mode_contact_ = false;
      change_obst_ = false;
      //force_feedback_=false;
      // adding interactive robot and positionning it
      client_.connect();
      cout << "adding landmark to viewer\n";
      //string robot_name = "/hpp/src/hpp_tutorial/urdf/robot_mesh.urdf"; contact_activated_ = true;
      //string robot_name = "/hpp/src/hpp_tutorial/urdf/robot_3angles.urdf"; contact_activated_ = true;
      string robot_name = "/hpp/src/hpp_tutorial/urdf/robot_L.urdf"; contact_activated_ = true;
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
      const double bounds[6] = {
        this->problem().robot()->rootJoint()->lowerBound(0),
        this->problem().robot()->rootJoint()->upperBound(0),
        this->problem().robot()->rootJoint()->lowerBound(1),
        this->problem().robot()->rootJoint()->upperBound(1),
        this->problem().robot()->rootJoint()->lowerBound(2),
        this->problem().robot()->rootJoint()->upperBound(2)
      };
      SixDOFMouseDriver::MouseInit(type_, bounds);
      ShowBounds();
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
      
      // using solveanddisplay relaunches planner -> anti core dump protection
      if (nb_launchs<2){
        boost::thread th1(boost::bind( &Planner::InteractiveDeviceThread,this));
        //boost::thread th2(boost::bind( &Planner::ActuateArm,this));
        pthread_t          handle;
        pthread_create (&handle, NULL, ForceFeedback, NULL);
        struct sched_param sp; 
        memset (&sp, 0, sizeof(struct sched_param));
        sp.sched_priority = 10; 
        pthread_setschedparam (handle, SCHED_RR, &sp);

      }
    }


    void Planner::InteractiveDeviceThread(){
      gepetto::corbaserver::Color color;
      color[0] = 1; color[1] = 1; color[2] = 1; color[3] = 1.;
      int index_lignes = 0;
      sleep(1);//TODO ceci pour régler le pb init curseur viteuf, à changer
      bool init = false;
      cout << "InteractiveDevice thread...\n";
      while(!SixDOFMouseDriver::HasMoved());
      while(nb_launchs<2){
        if (!init){
          const ConfigurationPtr_t initConfig_ = this->problem().initConfig();
          //TODO régler le pb init curseur ici en prenant les infos du script python
          //double translations[3] = {  //TODO
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
          //double angle = 3.14159265/2;
          //rot90z << cos(angle), -sin(angle), 0, 
          //sin(angle), cos(angle),  0,
          //0,          0,           1;
          //rot90x << 1, 0, 0,
          //0, cos(angle), -sin(angle),
          //0, sin(angle), cos(angle);
          //rot90y << cos(angle), 0, sin(angle),
          //0, 1, 0,
          //-sin(angle), 0, cos(angle);  
          //translate = translate.transpose() * rot90y;
          tr[0] = translate[0]; // version sigma7
          tr[1] = translate[1];
          tr[2] = translate[2];
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
        /*
           cout << "config curseur        " <<
        //    trans_temp << endl;
        //    //  (*actual_configuration_ptr_)[0] =
        tr[0] << " " <<
        //    (*actual_configuration_ptr_)[1] =
        tr[1] << " " <<
        //    (*actual_configuration_ptr_)[2] =
        tr[2] << endl;
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
        camMat = quat2Mat(CamVects->get_buffer()[0],CamVects->get_buffer()[1],
            CamVects->get_buffer()[2],CamVects->get_buffer()[3]);
        SixDOFMouseDriver::setCameraVectors(
            camMat(0,0), camMat(0,1), camMat(0,2),
            camMat(1,0), camMat(1,1), camMat(1,2),
            camMat(2,0), camMat(2,1), camMat(2,2)
            );



        //*
        // caler le robot au niveau du curseur
        // TODO bug sur mutex je pense // TODO bug avéré : blocage à l'init parfois
        // le bug est dû au problème init curseur : le contact s'active puis le robot s'éloigne
        // directement ensuite le "gros hack" essaye de rester dans une petite distance et tourne
        // à l'infini
        robot_mutex_.lock();                // TODO mutex inoptimal !
        //cout << "robot_mutex_lock device thread\n";
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
           (*problem().robot()->objectIterator(hpp::model::COLLISION))->name() << " tr "
           << (*problem().robot()->objectIterator(hpp::model::COLLISION))->getTransform().getTranslation()
           << endl;
        //*/

        // méthode de recherche du plus proche obst par itération
        fcl::DistanceResult result;
        result.min_distance = 999;
        bool collision = false;
        result = FindNearestObstacle();
        collision = result.min_distance == -1 ? true : false;
        //cout << "result.min_distance " << result.min_distance << endl;
        org_temp = result.nearest_points[0];
        change_obst_ = (org_temp-prev_org_temp).norm()>0.001;
        //change_obst_ = org_temp!=prev_org_temp;
        if(result.min_distance==-1)org_temp=prev_org_temp;
        prev_org_temp=org_temp;
        
        obj_temp = result.nearest_points[1];
        dist_cont_ = result.min_distance;
        robot_mutex_.unlock();
        //cout << "robot_mutex_unlock device thread\n";
        
        //cout << "ligne 365  " << org_temp << "\t" << obj_temp << endl;
        //*
        // //////////////////////////////////////////////////////////////////
        if (contact_activated_ && !collision && !Planner::mode_contact_){

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

          // algorithme de GRAM-SCHMIDT
          if (result.min_distance != -1){
            exist_obstacle_ = true;

            Mat33f A;
            // normale
            A.col[0] = Vec3f(w[0]-v[0], w[1]-v[1], w[2]-v[2]);
            // distance du centre de l'objet à sa surface
            if (distance_mutex_.try_lock()) // TODO mutex inoptimal
            {
              normal_mutex.lock();
              normal={
                  (float)(result.nearest_points[0][0] - result.nearest_points[1][0]),
                  (float)(result.nearest_points[0][1] - result.nearest_points[1][1]),
                  (float)(result.nearest_points[0][2] - result.nearest_points[1][2])
                  };
              normal.normalize();
              normal_mutex.unlock();
              //cout << "normale " << normal(0)<< " " << normal(1) << " "<< normal(2) << endl;
              //for (int i=0; i< 3; i++)
                //if (abs(normal(i))<(1e-10)) normal(i) = 0;
              //cout << "normale " << normal(0)<< " " << normal(1) << " "<< normal(2) << endl;
              //double norm_of_normal = sqrt( pow(normal(0), 2) +
                  //pow(normal(1), 2) +
                  //pow(normal(2), 2) );
              //cout << "norm of normale " << norm_of_normal << endl;
              //normal(0) = normal(0)/(float)norm_of_normal;
              //normal(1) = normal(1)/(float)norm_of_normal;
              //normal(2) = normal(2)/(float)norm_of_normal;
              //cout << "normale " << normal(0)<< " " << normal(1) << " "<< normal(2) << endl;
              d_com_near_point_ = {
                  (float)result.nearest_points[1][0] - trans_temp.translation()[0],
                  (float)result.nearest_points[1][1] - trans_temp.translation()[1],
                  (float)result.nearest_points[1][2] - trans_temp.translation()[2]};
              //cout << "d_com_near_point " << d_com_near_point(0)<< " " << d_com_near_point(1) << " "<< d_com_near_point(2) << endl;
              distances_[0] = (float) (0.0 + normal(0) * d_com_near_point_(0));
              distances_[1] = (float) (0.0 + normal(1) * d_com_near_point_(1));
              distances_[2] = (float) (0.0 + normal(2) * d_com_near_point_(2));
              //cout<<"distances_ "<<distances_[0]<<" "<<distances_[1]<<" "<<distances_[2]<<endl;
              distance_ = (float)sqrt(
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
            if (result.min_distance<d_){
              //force_feedback_ = true;
              if (!Planner::mode_contact_){
                //cout << "distance inférieure à 0.15" << std::endl;
                //std::cout << " pt0 " << result.nearest_points[0] <<
                //             " pt1 " << result.nearest_points[1] << std::endl;
                // sur l'obstacle
                org_ = result.nearest_points[0];
                // sur le robot
                obj_ = result.nearest_points[1];
                iteration_ = 0;
                //cout << "contact activated\n";
                Planner::mode_contact_ = true;
              }
            }
            //else force_feedback_ = false; // TODO rechanger la condition
          }// fin gram schmidt
        }// fin activation contact
        // show modifications on screen
        client_.gui()->refresh();
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

    void Planner::oneStep ()
    {
        //cout << "one step\n";
      robot_mutex_.lock();
      //cout << "robot_mutex_lock one step\n";
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
      if (rando > Planner::random_prob_ || Planner::mode_contact_)
      {
        if (Planner::mode_contact_){
          //cout << rando << " contact q \n";
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
          //cout << "will distancemutex try lock\n";
          distance_mutex_.try_lock();
          //cout << "trylock ok\n";
          Vector3 org(
              (float)org_[0]+signe(obj_[0]-org_[0])*distances_[0],
              (float)org_[1]+signe(obj_[1]-org_[1])*distances_[1],
              (float)org_[2]+signe(obj_[2]-org_[2])*distances_[2]
              );
          //cout << "org " << org.transpose() << endl;
          val = rot.transpose()*val + org;
          //cout << "nouveau val " << val.transpose() << endl;

          // gros hack
          // bool proche = false;
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
          //cout <<"fin hack wile 1\n";

          (*q_rand)[0] = val[0];
          (*q_rand)[1] = val[1];
          (*q_rand)[2] = val[2];
          // fixer rotation
          (*q_rand)[3] = (*Planner::actual_configuration_ptr_)[3];
          (*q_rand)[4] = (*Planner::actual_configuration_ptr_)[4];
          (*q_rand)[5] = (*Planner::actual_configuration_ptr_)[5];
          (*q_rand)[6] = (*Planner::actual_configuration_ptr_)[6];

          Planner::iteration_++;
          //cout << "iteration contact " << iteration_ << endl;
          if(Planner::iteration_ == 5){
            Planner::mode_contact_ = false;
            // ancien emplacement de distance_mutex_.unlock();
          }
        }
        else{ // mode interactif
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
    //cout << "robot_mutex_unlock one step\n";
    }

    void Planner::configurationShooter
    (const ConfigurationShooterPtr_t& shooter)
    {
      configurationShooter_ = shooter;
    }

    // afficher bornes du problème
    void Planner::ShowBounds(){
        gepetto::corbaserver::Color color_rouge;
        color_rouge[0]=(float)1;color_rouge[1]=(float)0.2;
        color_rouge[2]=(float)0;color_rouge[3]=(float)1;
        std::cout << "joint bounds " <<
                     this->problem().robot()->rootJoint()->lowerBound(0) << " " <<
                     this->problem().robot()->rootJoint()->upperBound(0) << " " <<
                     this->problem().robot()->rootJoint()->lowerBound(1) << " " <<
                     this->problem().robot()->rootJoint()->upperBound(1) << " " <<
                     this->problem().robot()->rootJoint()->lowerBound(2) << " " <<
                     this->problem().robot()->rootJoint()->upperBound(2) << " " <<
        std::endl;

        float mx = (float)this->problem().robot()->rootJoint()->lowerBound(0);
        float my = (float)this->problem().robot()->rootJoint()->lowerBound(1);
        float mz = (float)this->problem().robot()->rootJoint()->lowerBound(2);
        float Mx = (float)this->problem().robot()->rootJoint()->upperBound(0);
        float My = (float)this->problem().robot()->rootJoint()->upperBound(1);
        float Mz = (float)this->problem().robot()->rootJoint()->upperBound(2);

        min << mx, my, mz;
        max << Mx, My, Mz;

        const gepetto::corbaserver::Position A = {(float)mx, (float)my, (float)mz};
        const gepetto::corbaserver::Position B = {(float)Mx, (float)my, (float)mz};
        const gepetto::corbaserver::Position C = {(float)Mx, (float)My, (float)mz};
        const gepetto::corbaserver::Position D = {(float)mx, (float)My, (float)mz};
        const gepetto::corbaserver::Position E = {(float)mx, (float)my, (float)Mz};
        const gepetto::corbaserver::Position F = {(float)Mx, (float)my, (float)Mz};
        const gepetto::corbaserver::Position G = {(float)Mx, (float)My, (float)Mz};
        const gepetto::corbaserver::Position H = {(float)mx, (float)My, (float)Mz};

        string borne = "0_scene_hpp_/borne";
        borne +="i";
        client_.gui()->addLine(borne.c_str(), A, B, &color_rouge[0]);borne +="i";
        client_.gui()->addLine(borne.c_str(), B, C, &color_rouge[0]);borne +="i";
        client_.gui()->addLine(borne.c_str(), C, D, &color_rouge[0]);borne +="i";
        client_.gui()->addLine(borne.c_str(), D, A, &color_rouge[0]);borne +="i";
        client_.gui()->addLine(borne.c_str(), E, F, &color_rouge[0]);borne +="i";
        client_.gui()->addLine(borne.c_str(), F, G, &color_rouge[0]);borne +="i";
        client_.gui()->addLine(borne.c_str(), G, H, &color_rouge[0]);borne +="i";
        client_.gui()->addLine(borne.c_str(), H, E, &color_rouge[0]);borne +="i";
        client_.gui()->addLine(borne.c_str(), A, E, &color_rouge[0]);borne +="i";
        client_.gui()->addLine(borne.c_str(), B, F, &color_rouge[0]);borne +="i";
        client_.gui()->addLine(borne.c_str(), C, G, &color_rouge[0]);borne +="i";
        client_.gui()->addLine(borne.c_str(), D, H, &color_rouge[0]);

        client_.gui()->refresh();
}

    // returns an fcl distance request structure
    fcl::DistanceResult Planner::FindNearestObstacle(){
      fcl::DistanceRequest request(true, 0, 0, fcl::GST_INDEP);
      fcl::DistanceResult result;
      result.clear();
      fcl::CollisionObject* robot_proche=0;
      fcl::CollisionObject* obstacle_proche=0;
      fcl::CollisionObject* obst_temp=0;
      fcl::CollisionObject* robot_temp=0;
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
          if (floor(1000*result.min_distance)/1000<min_dist){
            if(result.min_distance==-1){
              1;//collision = true; TODO vérifier que tout va bien
            }

            // TODO grave : un problème sur l'itération, les point objet change problème résolu : istance calculée depuis l'un ou l'autre corps de l'objet
            robot_proche = robot_temp;
            obstacle_proche = obst_temp;
            min_dist = result.min_distance;
          }
        }
      }

      fcl::distance(obstacle_proche, robot_proche, request, result);
      if (result.min_distance <d_&&result.min_distance!=-1) {
        force_feedback_ = true;
        //obj_ffb_ = result.nearest_points[1];
      }
      //cout << result.nearest_points[0] << " " << result.nearest_points[1] << endl;
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






//void* ForceFeedback_n(void*){
//
  ////clock_t begin, end, end2 ;
  //Eigen::Vector3f signes, pos, force_vec, obst, normale;
  //double err, obj, force, gain, max;
  //gain = 100;
  //obj = 0.3;
  //max=15;max++;
  //sleep(3);
  //while(1){
    //pos = SixDOFMouseDriver::getTransformationNoMutex().translation();
    //for (int i=0; i<3; i++){
      //signes[i]=signe(org_temp[i]-pos[i]);
      //obst[i]=(float)org_temp[i]-signes[i]*(d_com_near_point_[i]+(float)d_); 
     // 
    //}
    //err = (pos-obst).norm();
    //if(err>0) force = err*gain;
    //else force=0;
   // 
    //normale = (pos-obst) / (float)err;
    //force_vec = (float)force * normale;
    ////if (force>max)force=max*signe(force);
    ////else force = 0;
    //cout << "obj="<<obj<<" pos="<<pos.transpose()<<" err="<<err<<" force="<<force<<" n"<<normale<<" f_vec"<<force_vec<<endl; 
    ////force_fd[0] = force;
    ////force_fd.setZero();
    //force_vec[0]=force_vec[2]=0;
    //dhdSetForce(force_vec[0], force_vec[1], -force_vec[2]);
   // 
//
  //}
  //return NULL;
//}






//void* _ForceFeedback(void*){
//
  //clock_t begin, end, end2 ;
  //Eigen::Vector3f signes, pos, force_fd;
  //double tmps, err, obj, force, gain, max;
  //gain = 100;
  //obj = 0.3;
  //max=15;
  //sleep(3);
  //while(1){
    //for (int i=0; i<3; i++) signes[i]=signe(org_temp[i]-pos[i]);
    //obj=org_temp[0]+signe[0]*(d_com_near_point_[0]+d_); 
    //pos = SixDOFMouseDriver::getTransformationNoMutex().translation();
    //err = pos[0]-obj;
    //if (err>0){
      //force = err*gain;
      //if (force>max)force=max*signe(force);
    //}
    //else force = 0;
    //cout << "obj="<<obj<<" pos="<<pos.transpose()<<" err="<<err<<" force="<<force<<endl; 
    //force_fd[0] = force;
    ////force_fd.setZero();
    //dhdSetForce(force_fd[0], force_fd[1], force_fd[2]);
   // 
//
  //}
//}











////void Planner::ForceFeedback(){
//void* ForceFeedback_(void*){
  //cout << "ForceFeedback thread...\n";
  //sleep(1);
//
  //Eigen::Vector3d force_fd, prev_force_fd, torque_fd;
  //Eigen::Vector3f pos;
  //fcl::Vec3f obj_ffb, signes, normale, force_vec, p_0;
  //double force, prev_force;
  //double gain=3000;
  //double force_max = 15;
  //bool force_feed = false;
  //double pen, pen_prev, pen_diff, delta_t;
  //FILE * pFile;
  //pFile = fopen ("log.csv","w");
  //clock_t begin, end, end2 ;
  //clock_t e1, e2;
 // 
  //double elapsed_secs;
  //long int iteration=0;
  ////double a, b, c;
  //e1=e2= clock();
  //pen_prev = 0;
  //fprintf (pFile, "t;posx;posy;posz;1000*p;1000*dp/dt;P;D;f\n");
  //while(1){
    //iteration++;
    //begin = clock();
    //pos = SixDOFMouseDriver::getTransformationNoMutex().translation();
    ////end = clock();
    ////double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    ////if (force_feed) cout << "temps get=\t\t" <<elapsed_secs<<endl; 
    //for (int i=0; i<3; i++) signes[i]=signe(org_temp[i]-pos[i]);
    //obj_ffb= {pos[0]+d_com_near_point_[0], pos[1]+d_com_near_point_[1], pos[2]+d_com_near_point_[2]}; 
    //p_0 = org_temp - obj_ffb; // vecteur obj->robot
    //double p_0_norm = p_0.norm();
    //force_feed = p_0_norm < d_ ? true : false;
    ////if (!force_feed&&iteration%15==1) cout <<"pos "<<pos.transpose()<<"+dist " << d_com_near_point_[0]<<" "<< d_com_near_point_[1]<<" "<< d_com_near_point_[2]<<" " << "* sgn " << signes << "=obj "<<obj_ffb<< " obj-obst " << org_temp << "=p_0 "<<p_0<<" p_0_norm "<<p_0_norm<<endl;
//
    //cout << "obst"<<org_temp<<" obj"<<obj_ffb<<" obst-obj"<<p_0<<" norme"<<p_0_norm<<"\t\tFFB"<<force_feed<<endl;
    ////end2 = clock();
    ////elapsed_secs = double(end2 - end) / CLOCKS_PER_SEC;
    ////if (force_feed) cout << "temps dist=\t\t" <<elapsed_secs<<endl; 
    //if(force_feed){
      //// terme proportionnel
      //pen = d_ - p_0_norm;
      //// derme dérivatif
      //pen_diff = (pen-pen_prev);
      //pen_prev = pen;
      //e2 = clock();
      //delta_t = e2 - e1;
      //e1 = e2;
      //double P = pen*gain;
      //double D = pen_diff/delta_t*300*gain;
      //// total 
      //force = P;// + D;
      //// normalisé 
      //normale = p_0 / p_0_norm;
      //force_vec = force * normale;
     // 
      ////end = clock();
      ////elapsed_secs = double(end - end2) / CLOCKS_PER_SEC;
      ////cout << "temps force=\t\t" <<elapsed_secs<<endl; 
      ////cout << setprecision(5) << setw(5) << "pen (" << pen << "*"<<gain<<") *n " <<normale<<"="<<force_vec<<endl;
     // 
      ////if(iteration%1==1)fprintf (pFile, "obst;%f;%f;%f;obj;%f;%f;%f;p_0;%f;%f;%f;p0norm;%f;pen;%f;force;%f;%f;%f;\n",org_[0],org_[1],org_[2],obj_ffb[0],obj_ffb[1],obj_ffb[2],p_0[0],p_0[1],p_0[2],p_0.norm(),pen,force_vec[0],force_vec[1],force_vec[2]);
      //double t = clock();
      ////cout <<"t="<<t<<"\tpos="<<pos.transpose()<<"\tpen="<<pen<<"\tdpen"<<pen_diff<<"\tdt"<<delta_t<<"\tpen_der/dt="<<pen_diff/delta_t<<"\tP="<<P<<"\tD="<<D<<"\tforce="<<force<<endl;
      ////if(iteration%10==1)
      //fprintf (pFile, "%lf;%lf;%lf;%lf;%lf;%lf;%lf,%f,%f\n",t,pos[0],pos[1],pos[2],pen,pen_diff/delta_t,P,D,force);
      ////force_fd.setZero();
//
      //// 0 avant arrière x rouge // 1 droite gauche y vert // 2 bas haut z bleu 
      //force_fd[0] = force_vec[0];
      //if (fabs(force_fd[0])>force_max)
      //force_fd[0]=force_max*signes[0];
      //if(force_fd[0]==-prev_force_fd[0])force_fd[0]=prev_force_fd[0];
////
      //force_fd[1] = force_vec[1];
      //if (fabs(force_fd[1])>force_max)
      //force_fd[1]=force_max*signes[0];
      //if(force_fd[1]==-prev_force_fd[1])force_fd[1]=prev_force_fd[1];
////
      //force_fd[2] = -force_vec[2];
      //if (fabs(force_fd[2])>force_max)
      //force_fd[2]=force_max*signes[0];
      //if(force_fd[2]==-prev_force_fd[2])force_fd[2]=prev_force_fd[2];
    //}
    //else{
      //force_fd.setZero();
      //torque_fd.setZero();
      ////a=b=c=0;
    //}
    //torque_fd.setZero();
    ////force_fd.setZero();
    ////end2 = clock();
    ////elapsed_secs = double(end2 - end) / CLOCKS_PER_SEC;
    ////if (force_feed) cout << "temps save=\t\t" <<elapsed_secs<<endl; 
    //for (int i=0; i<3; i++)force_fd[i]=(force_fd[i]+prev_force_fd[i])/2; 
    ////deviceForce = force_fd;
    ////deviceTorque = torque_fd;  
    //double ff[7];
    //ff[0] = force_fd[0]; 
    //ff[1] = force_fd[1]; 
    //ff[2] = force_fd[2]; 
    //ff[3] = torque_fd[3];
    //ff[4] = torque_fd[4];
    //ff[5] = torque_fd[5];
    //ff[6] = 0;
    //end = clock();
    //dhdSetForce(force_fd[0], force_fd[1], force_fd[2]);
    //end2 = clock();
    //elapsed_secs = double(end2 - end) / CLOCKS_PER_SEC;
    ////if (force_feed) cout << "temps envoi=\t\t" <<elapsed_secs<<endl; 
    ////dhdSetForceAndTorqueAndGripperForce (force_fd[0], force_fd[1], force_fd[2], torque_fd[0], torque_fd[1], torque_fd[2], 0.0);
    ////SixDOFMouseDriver::setForceAndTorque(force_fd, torque_fd);
    ////dhdSleep(0.1);
    //prev_force_fd=force_fd;
    ////end = clock();
    ////elapsed_secs = double(end - end2) / CLOCKS_PER_SEC;
    ////if (force_feed) cout << "temps envoi=\t\t" <<elapsed_secs<<endl; 
    ////elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    ////if (force_feed) cout << "temps total=\t\t" <<elapsed_secs<<endl<<endl; 
  //} 
  //fclose (pFile);
  //return 0;
//}

/*void lisser(Eigen::Vector3d force_fd, Eigen::Vector3d prev, Eigen::Vector3d torque_fd){
  bool condi = false;
  bool cond[3];
  //double f[3];
  short int signei[3];
  for (int i=0;i<3;i++){
    cond[i] = false;
    //f[i]=prev[i];
    signei[i] = signe(force_fd[i]-prev[i]);
  }
  while(!condi){
    for (int i=0;i<3;i++){
      cond[i]=(prev[i]!=force_fd[i]);
      prev[i]+=signei[i];
    }
    SixDOFMouseDriver::setForceAndTorque(prev, torque_fd);
    condi = cond[0] || cond[1] || cond[2];
  }
}*/
