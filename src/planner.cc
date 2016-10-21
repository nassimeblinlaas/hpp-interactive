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
#include <Eigen/Geometry>
#include <algorithm>


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

    //////////// var glob
    short int nb_launchs = 0;
    fcl::Vec3f org_temp;
    fcl::Vec3f prev_org_temp;
    fcl::Vec3f obj_temp;
    Eigen::Vector3f d_com_near_point_[3];
    double d_;      
    bool change_obst_;
    bool collision_[3];
    Eigen::Matrix3f camMat;
    Eigen::Vector3f normal[3];

    boost::mutex normal_mutex;
    boost::mutex results_mutex;

    graphics::corbaServer::Client* client_ptr;

    vector<fcl::DistanceResult> results;
    vector<fcl::DistanceResult> results_cpy;
    vector<fcl::DistanceResult> prev_results;
    //////////// var glob

    void* ForceFeedback(void*){
      graphics::corbaServer::Client& client_ref(*client_ptr);
      double dnn[7];
      Eigen::Vector3f temp, pos, force_vec, prev_force_vec, obst, obst2, normale, normale2, signes, signes2;
      Eigen::Vector3d obst_d;
      Eigen::Matrix3d center;
      double err, err2, force, force2, gain, D, D2, kP;
      bool force_feed = false;
      bool force_feed2 = false;
      gain = 10;
      double max=5;
      drdStart(); 
      bool base, wrist, grip;
      base = wrist = grip = false;

      drdRegulatePos  (false);
      drdRegulateGrip (false);
      drdRegulateRot  (false);

      bool prev_ff = false;
      sleep(3);
      long int iteration = 0;
      Eigen::Vector3f d_com_near_point;
      d_com_near_point.setZero();
      obst.setZero();
      pos.setZero();
      temp.setZero();
      while(1){
        iteration++;

        results_mutex.lock();
        results_cpy = results;
        results_mutex.unlock();
        pos = SixDOFMouseDriver::getTransformationNoMutex().translation();
        for (int i=0; i<3; i++){
          obst[i]=(float)results_cpy[0].nearest_points[0][i];//(float)org_temp[i];
          obst2[i]=(float)results_cpy[1].nearest_points[0][i];
        }
        normal_mutex.lock();
        normale = normal[0];
        normale2 = normal[1];
        normal_mutex.unlock();
        // départ du plan P
        temp=obst-d_*normale;
        // équation du plan P: Ax+By+Cz+D=0;
        D=-(temp[0]*normale[0]+temp[1]*normale[1]+temp[2]*normale[2]);
        // départ du plan P2
        temp=obst2-d_*normale2;
        // équation du plan P: Ax+By+Cz+D=0;
        D2=-(temp[0]*normale2[0]+temp[1]*normale2[1]+temp[2]*normale2[2]);
        // position du point proche de l'objet par rapport à P
        //cout << "ffd " << (float)results_cpy[0].nearest_points[1][0]<< " " <<(float)results_cpy[0].nearest_points[1][1]<<" " << (float)results_cpy[0].nearest_points[1][2]<< " dcom " << d_com_near_point.transpose()<<endl;
 
        err = (pos[0]+d_com_near_point_[0][0])*normale[0]+
          (pos[1]+d_com_near_point_[0][1])*normale[1]+
          (pos[2]+d_com_near_point_[0][2])*normale[2]+D;
        err2 = (pos[0]+d_com_near_point_[1][0])*normale2[0]+
          (pos[1]+d_com_near_point_[1][1])*normale2[1]+
          (pos[2]+d_com_near_point_[1][2])*normale2[2]+D2;
        //cout << "ffd1 " << pos[0]<< " " <<pos[1]<<" " << pos[2]<< " dcom " << d_com_near_point_.transpose()<<" err " << err << endl;
  
       //err = (pos[0]+d_com_near_point[0])*normale[0]+
          //(pos[1]+d_com_near_point[1])*normale[1]+
          //(pos[2]+d_com_near_point[2])*normale[2]+D;
        //cout << " dcom_ " << d_com_near_point_[0].transpose() << " dcom " << err << endl;
        //if (iteration%200==0)cout << "ffd2 near point" << (float)results_cpy[0].nearest_points[1][0]<< " " <<(float)results_cpy[0].nearest_points[1][1]<<" " << (float)results_cpy[0].nearest_points[1][2]<< " dcom " << d_com_near_point.transpose()<< " err "<<err<<endl;
        kP = err * gain;
        force = kP;
        force_feed = err > 0;
        force2 = err2 * gain;
        force_feed2 = err2 > 0;
        cout << "obst1 " << obst.transpose() << " obst2 " << obst2.transpose() << endl;
        cout << "err1 " << err << " err2 " << err2 << " ffd1 " << force_feed << " ffd2 " << force_feed2 << endl;
        //cout << "obst="<<obst.transpose()<<" pos="<<pos.transpose()<<" D="<<D<<" err="<<err<<" force="<<force<<" n "<<normale.transpose()<<" f_vec"<<force_vec.transpose()<<endl; 
        //cout << force_feed<<endl;

        if (force_feed != prev_ff)
        {
          drdRegulateRot(force_feed);
          prev_ff = force_feed;
          //if (prev_ff){
            //string nom_ligne = "0_scene_hpp_/ligne_force";
            //string ind = boost::lexical_cast<std::string>(iteration);
            //nom_ligne += ind;
            //client_ref.gui()->setVisibility(nom_ligne.c_str(), "OFF");
          //} 
        }


        if (force_feed){
          
          /////
          //string nom_ligne = "0_scene_hpp_/ligne_force";
          //string ind = boost::lexical_cast<std::string>(iteration);
          //nom_ligne += ind;
          //if (iteration > 0){
          //cout << "index " << iteration << " obj à cacher " << nom_ligne << endl;
          //client_ref.gui()->setVisibility(nom_ligne.c_str(), "OFF");
          //}
          //iteration++;
          //nom_ligne = "0_scene_hpp_/ligne_force";
          //ind = boost::lexical_cast<std::string>(iteration);
          //nom_ligne += ind;
          //gepetto::corbaserver::Color color;
          //color[0] = 0.3; color[1] = 0.7; color[2] = 0.2; color[3] = 1.;
          //gepetto::corbaserver::Position v = {obst[0], obst[1], obst[2]};
          //Eigen::Vector3f t = normal * kP*10;
          //gepetto::corbaserver::Position w = {t[0], t[1], t[2]};
          //client_ref.gui()->addLine(nom_ligne.c_str(), v, w, &color[0]);

          force_vec.setZero();
          force_vec = normale*(float)force;

          //cout << "ffb " << force_vec.transpose();
          //if (change_obst_) iteration = 0; 
          if (force_feed2){
            force_vec += normale2*(float)force2;
            //cout << " ffb2 " << force_vec.transpose();
          }
          
          // résultante
          normale +=normale2;
          normale.normalize();
          //cout << endl;
          //force_vec.setZero();
          // protection 1 : seuillage
          //for (int i = 0 ;i<3; i++) if (force_vec[i]>max){force_vec[i]=signes[i]*max; /*cout<<"max dépassé sur "<<i<<endl;*/}
          // protection 2 : lissage
          // bah ça fait pas grand chose
          //for (int i=0; i<3;i++){
            //double diff = force_vec(i)-prev_force_vec(i);
            //if(fabs(diff)>1){
              ////cout << " diff sur "<<i << " force "<<force_vec.transpose()<<" prevf "<<prev_force_vec.transpose(); 
              //force_vec[i] = prev_force_vec(i)+0.02*signe(diff);
              ////cout << " change " <<force_vec.transpose()<<endl; 
            //}
            //if (fabs(force_vec(i))<1e-5) force_vec[i]=0;
          //}
          //prev_force_vec = force_vec;
          /////////////////////////////////////////////////////
        }
        else{
        //if(collision_[0])d_com_near_point = {
          //(float)results_cpy[0].nearest_points[1][0] - pos[0],
          //(float)results_cpy[0].nearest_points[1][1] - pos[1],
          //(float)results_cpy[0].nearest_points[1][2] - pos[2]};
          //iteration = 0;
          force_vec.setZero();
          for (int i=0; i<3; i++) signes[i]=signe((float)results_cpy[0].nearest_points[0][i]-pos[i]);
          for (int i=0; i<3; i++) signes2[i]=signe((float)results_cpy[1].nearest_points[0][i]-pos[i]);
        }

        dnn[0] = force_vec(0);
        dnn[1] = force_vec(1);
        dnn[2] = -force_vec(2);
        drdSetForceAndTorqueAndGripperForce (dnn);  // avec blocage des rots

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
      client_ptr = &client_;
      nb_launchs++;
      type_ = 2; //device type 1 mouse 2 sigma7
      random_prob_ = 0; // 0 all human  1 all machine
      d_ = 0.11; // distance entrée mode contact
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
        //if (type_==2){
        if (1){
          pthread_t          handle;
          pthread_create (&handle, NULL, ForceFeedback, NULL);
          struct sched_param sp; 
          memset (&sp, 0, sizeof(struct sched_param));
          sp.sched_priority = 10; 
          pthread_setschedparam (handle, SCHED_RR, &sp);
        }
      }
    }


    void Planner::InteractiveDeviceThread(){
      gepetto::corbaserver::Color color;
      color[0] = 1; color[1] = 1; color[2] = 1; color[3] = 1.;
      int index_lignes = 0;
      int index_lignes2 = 0;
      bool double_contact = true; 
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
          //Eigen::Vector3f translate;
          //translate << trans_temp.translation()[0], 
                    //trans_temp.translation()[1],
                    //trans_temp.translation()[2];
          //Eigen::Matrix3f rot90x;
          //Eigen::Matrix3f rot90y;
          //Eigen::Matrix3f rot90z;
          //tr[0] = translate[0]; // version sigma7
          //tr[1] = translate[1];
          //tr[2] = translate[2];
          tr[0] = trans_temp.translation()[0];
          tr[1] = trans_temp.translation()[1];
          tr[2] = trans_temp.translation()[2];
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

        collision_[0] = false;
        vector<fcl::DistanceResult> results_temp;
        FindNearestObstacle(results_temp);
        results_mutex.lock();
        results.clear();
        results = results_temp;
        fcl::DistanceResult result = results[0];
        results_mutex.unlock();
        //result.min_distance = 999;
        collision_[0] = result.min_distance == -1 ? true : false;
        collision_[1] = results[1].min_distance == -1 ? true : false;
        //cout << "result.min_distance " << result.min_distance << endl;
        org_temp = result.nearest_points[0];
        change_obst_ = (org_temp-prev_org_temp).norm()>0.001;
        //change_obst_ = org_temp!=prev_org_temp;
        if(result.min_distance==-1)org_temp=prev_org_temp;
        if(results[0].min_distance==-1)results[0]=prev_results[0];
        if(results[1].min_distance==-1)results[1]=prev_results[1];
        prev_org_temp=org_temp;
        prev_results = results; 
        obj_temp = result.nearest_points[1];
        dist_cont_ = result.min_distance;
        robot_mutex_.unlock();
        //cout << "robot_mutex_unlock device thread\n";
        bool contact[3];
        for (int i=0; i<3; i++)contact[i] = results_temp[i].min_distance<d_;


        AfficherReperes(contact, results_temp); 
        

        //*
        // //////////////////////////////////////////////////////////////////
        if (contact_activated_ && !collision_[0] && !Planner::mode_contact_){
          index_lignes2 = 0;

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
              normal[0]={
                (float)(result.nearest_points[0][0] - result.nearest_points[1][0]),
                (float)(result.nearest_points[0][1] - result.nearest_points[1][1]),
                (float)(result.nearest_points[0][2] - result.nearest_points[1][2])
              };
              normal[0].normalize();
              normal[1]={
                (float)(results[1].nearest_points[0][0] - results[1].nearest_points[1][0]),
                (float)(results[1].nearest_points[0][1] - results[1].nearest_points[1][1]),
                (float)(results[1].nearest_points[0][2] - results[1].nearest_points[1][2])
              };
              normal[1].normalize();
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
              //cout <<"là. near point "<<(float)result.nearest_points[1][0]<<"."<<(float)result.nearest_points[1][1]<<"."<<(float)result.nearest_points[1][2]<<".pos "<< trans_temp.translation()[0]<<"."<<trans_temp.translation()[1]<<trans_temp.translation()[2]<<" dcom "<<d_com_near_point_[0].transpose()<<endl;

              d_com_near_point_[0] = {
                (float)results[0].nearest_points[1][0] - trans_temp.translation()[0],
                (float)results[0].nearest_points[1][1] - trans_temp.translation()[1],
                (float)results[0].nearest_points[1][2] - trans_temp.translation()[2]};
              //cout << "d_com_near_point " << d_com_near_point(0)<< " " << d_com_near_point(1) << " "<< d_com_near_point(2) << endl;
              distances_[0] = (float) (0.0 + normal[0](0) * d_com_near_point_[0](0));
              distances_[1] = (float) (0.0 + normal[0](1) * d_com_near_point_[0](1));
              distances_[2] = (float) (0.0 + normal[0](2) * d_com_near_point_[0](2));
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

            if (double_contact){
              

            }


            //::Eigen::Matrix3f MGS_;
            //MGS_ << MGS.col[0].v[0],
                 //MGS.col[1].v[0],
                 //MGS.col[2].v[0],
                 //MGS.col[0].v[1],
                 //MGS.col[1].v[1],
                 //MGS.col[2].v[1],
                 //MGS.col[0].v[2],
                 //MGS.col[1].v[2],
                 //MGS.col[2].v[2];
            //NewMinBounds = MGS_*min;
            //NewMaxBounds = MGS_*max;

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
//*
    void Planner::AfficherReperes(bool contact[3], vector<fcl::DistanceResult> results){
      se3::SE3 trans_temp = SixDOFMouseDriver::getTransformation();
      gepetto::corbaserver::Color color;
      color[0] = 1; color[1] = 1; color[2] = 1; color[3] = 1.;
      static int index_lignes=0;
      static int index_lignes2=0;
      // point sur le robot
      gepetto::corbaserver::Position v = {
        (float)results[0].nearest_points[0][0],
        (float)results[0].nearest_points[0][1],
        (float)results[0].nearest_points[0][2]};
      // point sur l'obstacle
      gepetto::corbaserver::Position w = {
        (float)results[0].nearest_points[1][0],
        (float)results[0].nearest_points[1][1],
        (float)results[0].nearest_points[1][2]};
      //cout << index_lignes<<endl;
      //cout << "index " << index_lignes << " obj à cacher " << nom_ligne << endl;

      // toujours tenter d'effacer le repère 2
      string nom_ligne = "0_scene_hpp_/ligne";
      string ind = boost::lexical_cast<std::string>(index_lignes);
      nom_ligne += ind;
      ind = boost::lexical_cast<std::string>(index_lignes2);
      nom_ligne += ind ;
      nom_ligne +='c';
      //cout << " obj à cacher " << nom_ligne << endl;
      client_.gui()->setVisibility(nom_ligne.c_str(), "OFF");
      //}
      if (!contact [0])
      {
        // effacer l'ancien repère 1 si pas de contact sinon, le garder
        nom_ligne = "0_scene_hpp_/ligne";
        ind = boost::lexical_cast<std::string>(index_lignes);
        nom_ligne += ind;
        client_.gui()->setVisibility(nom_ligne.c_str(), "OFF");
        string axe = nom_ligne +='a';
        client_.gui()->setVisibility(axe.c_str(), "OFF");
        axe = nom_ligne +='b';
        client_.gui()->setVisibility(axe.c_str(), "OFF");

        index_lignes++;
        // afficher le repère local // //////////////////////////////////////////
        nom_ligne = "0_scene_hpp_/ligne";
        ind = boost::lexical_cast<std::string>(index_lignes);
        nom_ligne += ind;
        client_.gui()->addLine(nom_ligne.c_str(), v, w, &color[0]);
        // afficher les deux axes manquants du repère
        w[0] = v[0] + MGS.col[1].v[0];
        w[1] = v[1] + MGS.col[1].v[1];
        w[2] = v[2] + MGS.col[1].v[2];
        axe = nom_ligne +='a';
        client_.gui()->addLine(axe.c_str(), v, w, &color[0]);

        w[0] = v[0] + MGS.col[2].v[0];
        w[1] = v[1] + MGS.col[2].v[1];
        w[2] = v[2] + MGS.col[2].v[2];
        axe = nom_ligne +='b';
        client_.gui()->addLine(axe.c_str(), v, w, &color[0]);
        //axe = nom_ligne +='c';
        //client_.gui()->addLine(axe.c_str(), v2, w2, &color[0]);
        // //////////////////////////////////////////////////////////////

      } 
      if (contact[0] && !contact[1]){
        //d_com_near_point_[0] = {
          //(float)results[0].nearest_points[1][0] - trans_temp.translation()[0],
          //(float)results[0].nearest_points[1][1] - trans_temp.translation()[1],
          //(float)results[0].nearest_points[1][2] - trans_temp.translation()[2]};
        gepetto::corbaserver::Position v2 = {
          (float)results[1].nearest_points[0][0],
          (float)results[1].nearest_points[0][1],
          (float)results[1].nearest_points[0][2]};
        gepetto::corbaserver::Position w2 = {
          (float)results[1].nearest_points[1][0],
          (float)results[1].nearest_points[1][1],
          (float)results[1].nearest_points[1][2]};
        d_com_near_point_[1] = {
          (float)results[1].nearest_points[1][0] - trans_temp.translation()[0],
          (float)results[1].nearest_points[1][1] - trans_temp.translation()[1],
          (float)results[1].nearest_points[1][2] - trans_temp.translation()[2]};
        // afficher le repère local // //////////////////////////////////////////
        nom_ligne = "0_scene_hpp_/ligne";
        ind = boost::lexical_cast<std::string>(index_lignes);
        nom_ligne += ind;
        client_.gui()->addLine(nom_ligne.c_str(), v, w, &color[0]);
        nom_ligne = "0_scene_hpp_/ligne";
        ind = boost::lexical_cast<std::string>(index_lignes);
        nom_ligne += ind ;
        ind = boost::lexical_cast<std::string>(index_lignes2);
        nom_ligne += ind ;
        nom_ligne +='c';
        client_.gui()->setVisibility(nom_ligne.c_str(), "OFF");
        //if (index_lignes > 0){
        //}
        index_lignes2++;
        nom_ligne = "0_scene_hpp_/ligne";
        ind = boost::lexical_cast<std::string>(index_lignes);
        nom_ligne += ind;
        ind = boost::lexical_cast<std::string>(index_lignes2);
        nom_ligne += ind ;
        nom_ligne +='c';
        //cout << " obj à afficher " << nom_ligne << endl;
        //if (index_lignes > 0){
        client_.gui()->addLine(nom_ligne.c_str(), v2, w2, &color[0]);
        //// //////////////////////////////////////////////////////////////

      }
    }//*/

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


    bool sortDistances (fcl::DistanceResult i,fcl::DistanceResult j) {
      return (i.min_distance<j.min_distance); 
    }

    // returns an fcl distance request structure
    bool Planner::FindNearestObstacle(vector<fcl::DistanceResult>&vect){
      // méthode de recherche du plus proche obst par itération
      //std::vector<std::pair<fcl::DistanceResult, std::pair<fcl::CollisionObject*, fcl::CollisionObject*> > vect;
      //std::vector<fcl::DistanceResult> vect;
      fcl::DistanceRequest request(true, 0, 0, fcl::GST_INDEP);
      fcl::DistanceResult result;
      result.clear();
      fcl::CollisionObject* robot_proche=0;
      fcl::CollisionObject* obstacle_proche=0;
      fcl::CollisionObject* obst_temp=0;
      fcl::CollisionObject* robot_temp=0;
      fcl::DistanceResult res[3];
      hpp::core::ObjectVector_t liste = this->problem().collisionObstacles();
      double min_dist = 999;
      string impr="";
      for (hpp::core::ObjectVector_t::iterator it_obst = liste.begin();it_obst!=liste.end();++it_obst){
        obst_temp = &*(*it_obst)->fcl();
        for (hpp::model::ObjectIterator it_rob = this->problem().robot()->objectIterator(hpp::model::COLLISION);
            !it_rob.isEnd(); ++it_rob){
          robot_temp = &*(*it_rob)->fcl();
          result.clear();
          fcl::distance(obst_temp, robot_temp, request, result);
          //if(result.min_distance<5)
          result.min_distance=(floor(1000*result.min_distance)/1000);
          impr+=(*it_obst)->name() + "/" + (*it_rob)->name() + " " + boost::lexical_cast<std::string>(result.min_distance)+"\n";
          //if (floor(1000*result.min_distance)/1000<min_dist){
          //if (1){ // TODO attention ceci enlève la correctio du bug ci dessous
                    // je crois que c'est re-bon
            //if(result.min_distance!=-1){
            //}
            vect.push_back(result); 

            // TODO grave : un problème sur l'itération, le point objet change problème résolu : distance calculée depuis l'un ou l'autre corps de l'objet
            //robot_proche = robot_temp;
            //obstacle_proche = obst_temp;
            //min_dist = result.min_distance;
          //}
        }
      }
      std::sort (vect.begin(), vect.end(), sortDistances);
      //cout << impr<<endl;
      //for (int i=0;i<vect.size();i++) cout << vect[i].min_distance<<endl;
      //cout << endl;
      //fcl::distance(obstacle_proche, robot_proche, request, result);
      //cout << "vect";
      //for (int i = 0; i< vect.size(); i++) cout << " " << vect[i].min_distance;
      //cout << endl;
      //if (result.min_distance <d_&&result.min_distance!=-1) {
      //force_feedback_ = true;
      //obj_ffb_ = result.nearest_points[1];
      //}
      //cout << result.nearest_points[0] << " " << result.nearest_points[1] << endl;
      //vect[0]=result;
      return true;
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
              //axe = nom_ligne +='c';
              //client_.gui()->setVisibility(axe.c_str(), "OFF");
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
            //axe = nom_ligne +='c';
            //client_.gui()->addLine(axe.c_str(), v2, w2, &color[0]);
            // //////////////////////////////////////////////////////////////
            //*/
