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

#include <boost/lexical_cast.hpp>

#include <hpp/model/joint.hh>
#include <hpp/model/object-factory.hh>
#include <hpp/model/distance-result.hh>
#include <hpp/model/fwd.hh>

#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/distance.h>

#include <hpp/util/pointer.hh>
#include <hpp/model/config.hh>
#include <hpp/model/fwd.hh>

#include <hpp/core/config-projector.hh>
#include <hpp/interactive/planner.hh>
#include <hpp/core/node.hh>
#include <hpp/core/edge.hh>
#include <hpp/core/path.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/path-validation-report.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/steering-method.hh>
#include <hpp/core/basic-configuration-shooter.hh>
#include <hpp/core/discretized-collision-checking.hh>
#include <hpp/core/collision-validation.hh>
#include <hpp/model/collision-object.hh>
#include <hpp/corbaserver/client.hh>
#include <gepetto/viewer/group-node.h>
#include <gepetto/viewer/window-manager.h>
#include <gepetto/viewer/roadmap-viewer.h>
#include <boost/thread/mutex.hpp>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/LU>
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
#include <gepetto/viewer/corba/se3.hh>


#include <hpp/core/problem.hh>

#include <hpp/interactive/sixDOFMouseDriver.hh>
#include <hpp/interactive/gram-schmidt.hh>
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
    bool Planner::exist_obstacle_;
    double Planner::repere_local_[3][3];
    bool Planner::mode_contact_; // let one step know if in contact


    // global variables
    short int nb_launchs = 0; // solve and display relance le planneur qui plante à cause du device


    //graphics::corbaServer::ClientCpp p; // deprecated but used to show cursor

    graphics::corbaServer::Client client(0, NULL);
    fcl::Vec3f org_;    // origine du point de référence pour gram schmidt
    fcl::Vec3f obj_;    // point de l'objet le plus proche de l'obstacle
    float distance_;    // distance centre du robot
    float distances_[3];   // distances centre du robot
    boost::mutex distance_mutex_; // protège par mutex l'accès à distance_
    fcl::CollisionObject* o2ptr;
    boost::mutex robot_mutex_;

    bool contact_activated;


    ::Eigen::Vector3f NewMinBounds;
    ::Eigen::Vector3f NewMaxBounds;

    ::Eigen::Vector3f min;
    ::Eigen::Vector3f max;


    // functions
    bool belongs (const ConfigurationPtr_t& q, const Nodes_t& nodes);
    void InteractiveDeviceThread(void* arg);

    // //////////////////////////////////////////////////////////////////////////////////////////

    /// \brief The Vec3f struct
    ///


    ::Eigen::Matrix3f quat2Mat(float x, float y, float z, float w){
        ::Eigen::Matrix3f ret;

            ret(0, 0) = 1 - 2*(pow(y, 2) + pow(z, 2));
                ret(0, 1) = 2*x*y - 2*z*w;
                    ret(0, 2) = 2*x*z + 2*y*w;
            ret(1, 0) = 2*x*y + 2*z*w;
                ret(1, 1) = 1 - 2*(pow(x, 2) + pow(z, 2));
                    ret(1, 2) = 2*y*z - 2*x*w;
            ret(2, 0) = 2*x*z - 2*y*w;
                ret(2, 1) = 2*y*z + 2*x*w;
                    ret(2, 2) = 1 - 2*(pow(x, 2) + pow(y, 2));

        return ret;
    }

    void euler2Quat(float psi, float theta, float phi, float* quat){
        psi/=2; theta/=2; phi/=2;

        quat[0] = cos(psi) * cos(theta) * cos(phi) + sin(psi) * sin(theta) * sin(phi);
        quat[1] = sin(psi) * cos(theta) * cos(phi) - cos(psi) * sin(theta) * sin(phi);
        quat[2] = cos(psi) * sin(theta) * cos(phi) + sin(psi) * cos(theta) * sin(phi);
        quat[3] = cos(psi) * cos(theta) * sin(phi) - sin(psi) * sin(theta) * cos(phi);
    }

    // normaliser quaternion
    void normalizeQuat(float& w, float& x, float& y, float& z){
        float mag = sqrt(pow(w,2)+pow(x,2)+pow(y,2)+pow(z,2));
        w = w/mag; x = x/mag; y = y/mag; z = z/mag;
    }



    short int signe (double x) {
        return ((x < 0) ? -1 : 1);
    }

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



        //client.gui()->addLine();

        const gepetto::corbaserver::Position A = {(float)mx, (float)my, (float)mz};
        const gepetto::corbaserver::Position B = {(float)Mx, (float)my, (float)mz};
        const gepetto::corbaserver::Position C = {(float)Mx, (float)My, (float)mz};
        const gepetto::corbaserver::Position D = {(float)mx, (float)My, (float)mz};
        const gepetto::corbaserver::Position E = {(float)mx, (float)my, (float)Mz};
        const gepetto::corbaserver::Position F = {(float)Mx, (float)my, (float)Mz};
        const gepetto::corbaserver::Position G = {(float)Mx, (float)My, (float)Mz};
        const gepetto::corbaserver::Position H = {(float)mx, (float)My, (float)Mz};

        //const gepetto::corbaserver::Position *A_ = &A[0];
        //const gepetto::corbaserver::Position *B_ = &B[0];
        //const gepetto::corbaserver::Position *C_ = &C[0];
        //const gepetto::corbaserver::Position *D_ = &D[0];
        //const gepetto::corbaserver::Position *E_ = &E[0];
        //const gepetto::corbaserver::Position *F_ = &F[0];
        //const gepetto::corbaserver::Position *G_ = &G[0];
        //const gepetto::corbaserver::Position *H_ = &H[0];

        string borne = "0_scene_hpp_/borne";
        borne +="i";
        client.gui()->addLine(borne.c_str(), A, B, &color_rouge[0]);borne +="i";
        client.gui()->addLine(borne.c_str(), B, C, &color_rouge[0]);borne +="i";
        client.gui()->addLine(borne.c_str(), C, D, &color_rouge[0]);borne +="i";
        client.gui()->addLine(borne.c_str(), D, A, &color_rouge[0]);borne +="i";
        client.gui()->addLine(borne.c_str(), E, F, &color_rouge[0]);borne +="i";
        client.gui()->addLine(borne.c_str(), F, G, &color_rouge[0]);borne +="i";
        client.gui()->addLine(borne.c_str(), G, H, &color_rouge[0]);borne +="i";
        client.gui()->addLine(borne.c_str(), H, E, &color_rouge[0]);borne +="i";
        client.gui()->addLine(borne.c_str(), A, E, &color_rouge[0]);borne +="i";
        client.gui()->addLine(borne.c_str(), B, F, &color_rouge[0]);borne +="i";
        client.gui()->addLine(borne.c_str(), C, G, &color_rouge[0]);borne +="i";
        client.gui()->addLine(borne.c_str(), D, H, &color_rouge[0]);

        client.gui()->refresh();
    }

    // //////////////////////////////////////////////////////////////////////////////////////////


    PlannerPtr_t Planner::createWithRoadmap
        (const Problem& problem, const RoadmapPtr_t& roadmap)
    {
        cout << "create with roadmap\n";
        Planner* ptr = new Planner (problem, roadmap);


        return PlannerPtr_t (ptr);
    }

    PlannerPtr_t Planner::create (const Problem& problem)
    {
        cout << "create\n";
        Planner* ptr = new Planner (problem);
        return PlannerPtr_t (ptr);
    }


    Planner::Planner (const Problem& problem,	const RoadmapPtr_t& roadmap) :
        PathPlanner (problem, roadmap),
        //configurationShooter_ (new BasicConfigurationShooter (problem.robot ())), // ancienne version avant grosse MAJ à vérifier
        configurationShooter_ (BasicConfigurationShooter::create (problem.robot ())),
        qProj_ (problem.robot ()->configSize ())
        {

            // configuratio
            //Planner::random_prob_ = 1; // 0 all human  1 all machine
            Planner::random_prob_ = 0.5; // 0 all human  1 all machine

            string robot_name = "/hpp/src/hpp_tutorial/urdf/robot_L.urdf"; contact_activated = true;
            //string robot_name = "/hpp/src/hpp_tutorial/urdf/robot_3angles.urdf"; contact_activated = true;


            nb_launchs++;
            std::cout << "read interactive device thread beginning\n";

            std::ofstream myfile;



            cout << "init config[0]=" << endl;
            //cout << this->problem().initConfig().get()[0] << endl;






            float f = (float) 0.1;

            cout << "adding landmark to viewer\n";
            client.connect();

            // add landmark to viewer

            //client.gui()->createGroup ("0_scene_hpp_");

            gepetto::corbaserver::Color color;
            color[0] = 1;	color[1] = 1;	color[2] = 1;	color[3] = 1.;
            cout << client.gui()->getSceneList()->get_buffer() << endl;
            cout << "adding landmark to viewer2\n";


            client.gui()->addBox ("0_scene_hpp_/curseur", f/10,f/10,f/10, color);
            cout << "adding landmark to viewer3\n";
            client.gui()->addSceneToWindow ("0_scene_hpp_", 0);
            cout << "adding landmark to viewer4\n";

            client.gui()->addLandmark("0_scene_hpp_/curseur", 1.);
            cout << "adding landmark to viewer5\n";
            client.gui()->addURDF("0_scene_hpp_/robot_interactif", robot_name.data() ,"");
            cout << "adding landmark to viewer6\n";



            /* // problème init curseur pas ici
            se3::SE3 trans_temp = SixDOFMouseDriver::getTransformation();
            ::gepetto::corbaserver::Transform tr;
            tr[0] = trans_temp.translation()[0];
            tr[1] = trans_temp.translation()[1];
            tr[2] = trans_temp.translation()[2];
            client.gui()->applyConfiguration("scene_hpp_/robot_interactif", tr);
            client.gui()->applyConfiguration("scene_hpp_/curseur", tr);
            //*/

            ConfigurationPtr_t q_rand = configurationShooter_->shoot (); // décale le rand initial


            double bounds[6] = {
                this->problem().robot()->rootJoint()->lowerBound(0),
                this->problem().robot()->rootJoint()->upperBound(0),
                this->problem().robot()->rootJoint()->lowerBound(1),
                this->problem().robot()->rootJoint()->upperBound(1),
                this->problem().robot()->rootJoint()->lowerBound(2),
                this->problem().robot()->rootJoint()->upperBound(2)
            };


            SixDOFMouseDriver::MouseInit(bounds);

            ShowBounds();

            ConfigurationPtr_t config (new Configuration_t ((hpp::model::size_type)7));

            // on vient ici dans un second temps donc la mauvaise init n'est pas là !!!!!!!!!!!
            //*
            (*config)[0] = 0;
            (*config)[1] = 0;
            (*config)[2] = 0;
            (*config)[3] = 1;
            (*config)[4] = 0;
            (*config)[5] = 0;
            (*config)[6] = 0;
            Planner::actual_configuration_ptr_ = config;
            //*/
            //config = this->problem().initConfig();



            boost::thread th(&InteractiveDeviceThread, this);

        }




    // //////////////////////////////////////////////////////////////////////////////////////////
    void InteractiveDeviceThread(void* arg){

        Planner* arg_ = (Planner*) arg;

        gepetto::corbaserver::Color color;
        color[0] = 1;	color[1] = 1;	color[2] = 1;	color[3] = 1.;

        int index_lignes = 0;



        bool init = false;
    while(1){




    if (SixDOFMouseDriver::HasMoved()){

        if (!init){
            const ConfigurationPtr_t initConfig_ = arg_->problem().initConfig();
            double translations[3] = {
                (*initConfig_)[0],
                (*initConfig_)[1],
                (*initConfig_)[2]
            };
            SixDOFMouseDriver::InitPosition(translations);
            init = true;
        }



        // get transformation from device
        //cout << "get transfo... inter device thread\n";
        se3::SE3 trans_temp = SixDOFMouseDriver::getTransformation();

        // conversion matrice -> quaternion
        Eigen::Matrix3f mat = trans_temp.rotation();
        Eigen::Quaternionf quat(mat);
        // normaliser quaternion
        double mag = sqrt(pow(quat.w(),2)+pow(quat.x(),2)+pow(quat.y(),2)+pow(quat.z(),2));
        ::gepetto::corbaserver::Transform tr;

        tr[0] = trans_temp.translation()[0];
        tr[1] = trans_temp.translation()[1];
        tr[2] = trans_temp.translation()[2];
        tr[3] = (float)quat.w()/(float)mag;
        tr[4] = (float)quat.x()/(float)mag;
        tr[5] = (float)quat.y()/(float)mag;
        tr[6] = (float)quat.z()/(float)mag;


        // save current transfo-rmation in the planner's memory
        (*Planner::actual_configuration_ptr_)[0] =
                        trans_temp.translation()[0];
        (*Planner::actual_configuration_ptr_)[1] =
                        trans_temp.translation()[1];
        (*Planner::actual_configuration_ptr_)[2] =
                        trans_temp.translation()[2];
        (*Planner::actual_configuration_ptr_)[3] = tr[3];
        (*Planner::actual_configuration_ptr_)[4] = tr[4];
        (*Planner::actual_configuration_ptr_)[5] = tr[5];
        (*Planner::actual_configuration_ptr_)[6] = tr[6];


        // afficher le robot
        client.gui()->applyConfiguration("0_scene_hpp_/robot_interactif", tr);
        client.gui()->applyConfiguration("0_scene_hpp_/curseur", tr);

        // get camera vectors to align cursor with viewer
        unsigned long id = client.gui()->getWindowID("window_hpp_");
        gepetto::corbaserver::floatSeq* CamVects;
        unsigned short int dim = 4;
        CamVects = new gepetto::corbaserver::floatSeq();
        CamVects->length(dim);
        CamVects = client.gui()->getCameraVectors((float)id, "");

        Matrix3 camMat = quat2Mat(CamVects->get_buffer()[0],CamVects->get_buffer()[1],
                CamVects->get_buffer()[2],CamVects->get_buffer()[3]);

        SixDOFMouseDriver::setCameraVectors(
            camMat(0,0), camMat(0,1), camMat(0,2),
            camMat(1,0), camMat(1,1), camMat(1,2),
            camMat(2,0), camMat(2,1), camMat(2,2)
                    );


        /*
        cout << "config curseur        " <<
        //    trans_temp << endl;
        //    //	(*actual_configuration_ptr_)[0] =
            trans_temp.translation()[0] << " " <<
            //		(*actual_configuration_ptr_)[1] =
            trans_temp.translation()[1] << " " <<
            //		(*actual_configuration_ptr_)[2] =
            trans_temp.translation()[2] << endl;
        //*/





        // using solveanddisplay relaunches planner -> anti core dump protection
        if (nb_launchs<2){


        //*
        // caler le robot au niveau du curseur
        robot_mutex_.lock();                // TODO mutex inoptimal !
        hpp::model::Configuration_t sauv = arg_->problem().robot()->currentConfiguration();
        hpp::model::Configuration_t in = sauv;
        in[0] = trans_temp.translation()[0];
        in[1] = trans_temp.translation()[1];
        in[2] = trans_temp.translation()[2];
        in[3] = tr[3];
        in[4] = tr[4];
        in[5] = tr[5];
        in[6] = tr[6];
        hpp::model::ConfigurationIn_t in_t(in);
        arg_->problem().robot()->currentConfiguration(in_t);
        arg_->problem().robot()->computeForwardKinematics();

        /*
    cout << " robot " <<
            (*arg_->problem().robot()->objectIterator(hpp::model::COLLISION))->name() << " tr "
            << (*arg_->problem().robot()->objectIterator(hpp::model::COLLISION))->getTransform().getTranslation()
            << endl;
    //*/



    // méthode de recherche du plus proche obst par itération
    fcl::DistanceResult result;
    bool collision = false;
    result = arg_->FindNearestObstacle();
    collision = result.min_distance == -1 ? true : false;


    robot_mutex_.unlock();



    // remettre le robot là où il était, inutile ?
    //arg_->problem().robot()->currentConfiguration(sauv);
    //arg_->problem().robot()->computeForwardKinematics();

    // //////////////////////////////////////////////////////////////////
 if (contact_activated && !collision){

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





    //* algorithme de GRAM-SCHMIDT
    if (result.min_distance != -1){
        arg_->exist_obstacle_ = true;


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

            /*
            std::cout << "distance centre/surf " << distance_ << std::endl;
            cout << "alt method " << distances_[0] << " " << distances_[1] << " " <<
                    distances_[2] <<
                    " norm " << norm_alt << endl <<
                    " pt0 " << result.nearest_points[0] <<
                         " pt1 " << result.nearest_points[1] <<
                         " \ncurseur " << trans_temp.translation() <<
                         endl << endl;
                    //*/
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
        string nom_ligne = "scene_hpp_/ligne";
        string ind = boost::lexical_cast<std::string>(index_lignes);
        nom_ligne += ind;
        //*
        if (index_lignes > 0){
            //cout << "index " << index_lignes << " obj à cacher " << nom_ligne << endl;
            client.gui()->setVisibility(nom_ligne.c_str(), "OFF");
            string axe = nom_ligne +='a';
            client.gui()->setVisibility(axe.c_str(), "OFF");
            axe = nom_ligne +='b';
            client.gui()->setVisibility(axe.c_str(), "OFF");
        }
        //*/
        index_lignes++;
        nom_ligne = "scene_hpp_/ligne";
        ind = boost::lexical_cast<std::string>(index_lignes);
        nom_ligne += ind;
        client.gui()->addLine(nom_ligne.c_str(), v, w, &color[0]);

        // afficher les deux axes manquants du repère
        w[0] = v[0] + MGS.col[1].v[0];
        w[1] = v[1] + MGS.col[1].v[1];
        w[2] = v[2] + MGS.col[1].v[2];
        string axe = nom_ligne +='a';
        client.gui()->addLine(axe.c_str(), v, w, &color[0]);
        w[0] = v[0] + MGS.col[2].v[0];
        w[1] = v[1] + MGS.col[2].v[1];
        w[2] = v[2] + MGS.col[2].v[2];
        axe = nom_ligne +='b';
        client.gui()->addLine(axe.c_str(), v, w, &color[0]);
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
            Planner::iteration_ = 0;
        }

    }// fin gram schmidt//*/

//}// fin for paires de collision

        }// fin activation contact

        } // fin nb launch
        //*/

        // show modifications on screen
        client.gui()->refresh();

        }// fin if has_moved_
        }// fin while(1)
    }
    // //////////////////////////////////////////////////////////////////////////////////////////




    /*
     *
     */
    void Planner::oneStep ()
    {
        static int i;
        typedef boost::tuple <NodePtr_t, ConfigurationPtr_t, PathPtr_t>	DelayedEdge_t;
        typedef std::vector <DelayedEdge_t> DelayedEdges_t;

        robot_mutex_.lock();

        DelayedEdges_t delayedEdges;
        //DevicePtr_t robot (problem ().robot ());
        //robot_mutex_.unlock();

        PathValidationPtr_t pathValidation (problem ().pathValidation ());
        Nodes_t newNodes;
        PathPtr_t validPath, path;
        // Pick a random node
        ConfigurationPtr_t q_rand = configurationShooter_->shoot ();

        //*
        // ////////////////////////////////////////////////////////////////////////////
        // decide whether to keep a random config or choose manual configuration from device
        double rando = rand();
        rando = rando / RAND_MAX;
        // keep random config
        if ( (rando < Planner::random_prob_) || (Planner::mode_contact_) ) // todo : réorganiser condition
        {
            if (Planner::mode_contact_){
                //cout << "mode contact " << ++i << Planner::iteration_ << std::endl;
                /* // bounds limitations, not working
                if(Planner::iteration_ == 0){
                    this->problem().robot()->rootJoint()->lowerBound(0, NewMinBounds[0]);
                    this->problem().robot()->rootJoint()->upperBound(0, NewMaxBounds[0]);
                    this->problem().robot()->rootJoint()->lowerBound(1, NewMinBounds[1]);
                    this->problem().robot()->rootJoint()->upperBound(1, NewMaxBounds[1]);
                    this->problem().robot()->rootJoint()->lowerBound(2, NewMinBounds[2]);
                    this->problem().robot()->rootJoint()->upperBound(2, NewMaxBounds[2]);
                }
                // bounds limitation, not working
                this->problem().robot()->rootJoint()->lowerBound(0, min[0]);
                this->problem().robot()->rootJoint()->upperBound(0, max[0]);
                this->problem().robot()->rootJoint()->lowerBound(1, min[1]);
                this->problem().robot()->rootJoint()->upperBound(1, max[1]);
                this->problem().robot()->rootJoint()->lowerBound(2, min[2]);
                this->problem().robot()->rootJoint()->upperBound(2, max[2]);
                //*/

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

                //* gros hack
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
                //*/
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
            //else cout << "pas contact\n";
        }
        else{
            //mutex_.lock();
            *q_rand = *Planner::actual_configuration_ptr_;
            //mutex_.unlock();
        }


        // ////////////////////////////////////////////////////////////////////////////
        //*/

        //
        // First extend each connected component toward q_rand
        //
        for (ConnectedComponents_t::const_iterator itcc =
            roadmap ()->connectedComponents ().begin ();
            itcc != roadmap ()->connectedComponents ().end (); ++itcc) {

            // Find nearest node in roadmap
            value_type distance;

            NodePtr_t near = roadmap ()->nearestNode (q_rand, *itcc, distance);
            path = extend (near, q_rand);
            if (path) {
                core::PathValidationReportPtr_t report;
                bool pathValid = pathValidation->validate (path, false, validPath, report);

                // Insert new path to q_near in roadmap
                value_type t_final = validPath->timeRange ().second;
                if (t_final != path->timeRange ().first) {
                    ConfigurationPtr_t q_new (new Configuration_t	((*validPath) (t_final)));
                    if (!pathValid || !belongs (q_new, newNodes)) {
                    newNodes.push_back (roadmap ()->addNodeAndEdges(near, q_new, validPath));
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
            roadmap ()->addEdge (newNode, near,
                validPath->extract (interval_t (timeRange.second , timeRange.first)));
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
                if (path && pathValidation->validate (path, false, validPath, report)) {
                    roadmap ()->addEdge (*itn1, *itn2, path);
                    interval_t timeRange = path->timeRange ();
                    roadmap ()->addEdge (*itn2, *itn1,
                        path->extract(interval_t (timeRange.second, timeRange.first)));
                }
            }
        }


    // ancien emplacement de
        robot_mutex_.unlock();
    }



    Planner::Planner (const Problem& problem):
        PathPlanner (problem),
        //configurationShooter_ (new BasicConfigurationShooter (problem.robot ())),
        configurationShooter_ (BasicConfigurationShooter::create (problem.robot ())),
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
                configProjector->projectOnKernel (
                    *(near->configuration ()), *target,	qProj_);
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

// ///////////////////////////////////////////////////////////////
// bornes du problème
/*
gepetto::corbaserver::Color color_rouge;
color_rouge[0]=1;color_rouge[1]=0.2;color_rouge[2]=0;color_rouge[3]=1;
std::cout << "joint bounds " <<
             arg_->problem().robot()->rootJoint()->lowerBound(0) << " " <<
             arg_->problem().robot()->rootJoint()->upperBound(0) << " " <<
             arg_->problem().robot()->rootJoint()->lowerBound(1) << " " <<
             arg_->problem().robot()->rootJoint()->upperBound(1) << " " <<
             arg_->problem().robot()->rootJoint()->lowerBound(2) << " " <<
             arg_->problem().robot()->rootJoint()->upperBound(2) << " " <<
std::endl;

double mx = arg_->problem().robot()->rootJoint()->lowerBound(0);
double my = arg_->problem().robot()->rootJoint()->lowerBound(1);
double mz = arg_->problem().robot()->rootJoint()->lowerBound(2);
double Mx = arg_->problem().robot()->rootJoint()->upperBound(0);
double My = arg_->problem().robot()->rootJoint()->upperBound(1);
double Mz = arg_->problem().robot()->rootJoint()->upperBound(2);

min << mx, my, mz;
max << Mx, My, Mz;

graphics::corbaServer::ClientCpp::value_type A[3] = {(float)mx, (float)my, (float)mz};
graphics::corbaServer::ClientCpp::value_type B[3] = {(float)Mx, (float)my, (float)mz};
graphics::corbaServer::ClientCpp::value_type C[3] = {(float)Mx, (float)My, (float)mz};
graphics::corbaServer::ClientCpp::value_type D[3] = {(float)mx, (float)My, (float)mz};
graphics::corbaServer::ClientCpp::value_type E[3] = {(float)mx, (float)my, (float)Mz};
graphics::corbaServer::ClientCpp::value_type F[3] = {(float)Mx, (float)my, (float)Mz};
graphics::corbaServer::ClientCpp::value_type G[3] = {(float)Mx, (float)My, (float)Mz};
graphics::corbaServer::ClientCpp::value_type H[3] = {(float)mx, (float)My, (float)Mz};

graphics::corbaServer::ClientCpp::value_type *A_ = &A[0];
graphics::corbaServer::ClientCpp::value_type *B_ = &B[0];
graphics::corbaServer::ClientCpp::value_type *C_ = &C[0];
graphics::corbaServer::ClientCpp::value_type *D_ = &D[0];
graphics::corbaServer::ClientCpp::value_type *E_ = &E[0];
graphics::corbaServer::ClientCpp::value_type *F_ = &F[0];
graphics::corbaServer::ClientCpp::value_type *G_ = &G[0];
graphics::corbaServer::ClientCpp::value_type *H_ = &H[0];

string borne = "scene_hpp_/borne";
borne +="i";
p.addLine(borne.c_str(), A_, B_, &color_rouge[0]);borne +="i";
p.addLine(borne.c_str(), B_, C_, &color_rouge[0]);borne +="i";
p.addLine(borne.c_str(), C_, D_, &color_rouge[0]);borne +="i";
p.addLine(borne.c_str(), D_, A_, &color_rouge[0]);borne +="i";
p.addLine(borne.c_str(), E_, F_, &color_rouge[0]);borne +="i";
p.addLine(borne.c_str(), F_, G_, &color_rouge[0]);borne +="i";
p.addLine(borne.c_str(), G_, H_, &color_rouge[0]);borne +="i";
p.addLine(borne.c_str(), H_, E_, &color_rouge[0]);borne +="i";
p.addLine(borne.c_str(), A_, E_, &color_rouge[0]);borne +="i";
p.addLine(borne.c_str(), B_, F_, &color_rouge[0]);borne +="i";
p.addLine(borne.c_str(), C_, G_, &color_rouge[0]);borne +="i";
p.addLine(borne.c_str(), D_, H_, &color_rouge[0]);

p.refresh();

//*/
// ///////////////////////////////////////////////////////////////

/* // afficher des repères à chaque config -> inutile
// créer un nom unique
string nom = "scene_hpp_/curseur";
string ind = boost::lexical_cast<std::string>((*q_rand)[0]);
nom += ind;
// une couleur
gepetto::corbaserver::Color color;
color[0] = 1;	color[1] = 1;	color[2] = 1;	color[3] = 1.;
// ajouter une boîte puis un repère
float f = (float) 0.1;
p.addBox (nom.c_str(), f/10,f/10,f/10, color);
p.addLandmark(nom.c_str(), 1.);
// mettre à jour la conf du repère
se3::SE3 conf;
conf.translation()[0] = (float)(*q_rand)[0];
conf.translation()[1] = (float)(*q_rand)[1];
conf.translation()[2] = (float)(*q_rand)[2];
Matrix3 rotation = quat2Mat((float)(*q_rand)[3], (float)(*q_rand)[4], (float)(*q_rand)[5],   (float)(*q_rand)[6]);
conf.rotation(rotation);
p.applyConfiguration(nom.c_str(), conf);
//p.setVisibility(nom.c_str(), "OFF");
p.refresh();
//*/

/*
// méthode 2 (et propre) et identique en résultats
PathValidationPtr_t pV2 = arg_->problem().pathValidation();
hpp::core::ObjectVector_t liste = arg_->problem().collisionObstacles();
cout << "méthode 2, liste des éléments ";
fcl::DistanceRequest req(true, 0, 0, fcl::GST_INDEP);
fcl::DistanceResult res;
res.clear();
for (hpp::core::ObjectVector_t::iterator it = liste.begin(); it != liste.end(); ++it){
    cout << " elem " << (*it)->name();
    fcl::CollisionObject o1 = *(*it)->fcl();
    cout << " rob " << (*arg_->problem().robot()->objectIterator(hpp::model::COLLISION))->name();
    fcl::CollisionObject o2 = *(*arg_->problem().robot()->objectIterator(hpp::model::COLLISION))->fcl();
    fcl::distance(&o1, &o2, req, res);

    cout << " dist " << res.min_distance << std::endl;
}
//*/

//*
// méthode 2 (et propre) et identique en résultats
//PathValidationPtr_t pV2 = arg_->problem().pathValidation();
//hpp::core::ObjectVector_t liste = arg_->problem().collisionObstacles();
//cout << "\nméthode 2, liste des éléments ";
//int i=0;
//for (hpp::core::ObjectVector_t::iterator it = liste.begin(); it != liste.end(); ++it){
    //i++;
    //cout << "itération " << i << endl;

// virtual bool currentConfiguration (ConfigurationIn_t configuration)


/*/ méthode 1 et plantoire
// root joint car robot monodimensionnel
hpp::model::CollisionObjectPtr_t robot_geom_ptr =
        *(arg_->problem().robot()->rootJoint()->linkedBody()->innerObjects
            (hpp::model::COLLISION).begin());
fcl::CollisionObject o1 = *robot_geom_ptr->fcl();

fcl::CollisionObject o2 = o1;

// const core::ObjectVector_t& lst = arg_->problem().collisionObstacles();
// begin car un seul obstacle
hpp::model::CollisionObjectPtr_t obst_geom_ptr =
        *(arg_->problem().collisionObstacles().begin());
fcl::CollisionObject o2 = *obst_geom_ptr->fcl();

//*/

//            PathValidationPtr_t pV = (arg_->problem().pathValidation());
//            core::DiscretizedCollisionChecking* DCC = (core::DiscretizedCollisionChecking*)(&*pV);
//            core::CollisionValidation* cV = (core::CollisionValidation*)(&*DCC->getConfigValid());
//            core::CollisionPairs_t cP = cV->getCollisionPairs();

            //

            //int index_paires=-1;
            //for (core::CollisionPairs_t::const_iterator itCol = cP.begin ();
            // itCol != cP.end (); ++itCol){

            // pas de boucle car un seul obstacle
            //{
                //core::CollisionPair_t paire = *itCol;

                // Given two objects o1 and o2
                //fcl::CollisionObject* o1 = paire.first->fcl().get();
                //fcl::CollisionObject* o2 = paire.second->fcl().get();
                //cout << "obj1 " << o1->getTranslation() << "\t";
                //cout << "obj2 " << o2->getTranslation() << "\t";


                // cout bla bla
                //std::cout << "test paires collision n°" << ++index_paires << " ";
                //std::cout << paire.first->name() << " " << paire.second->name() ;
                //cout << (result.min_distance == -1 ? "\t" : "");
                //std::cout << " dist=" << result.min_distance << std::endl;




                //double (*rep)[3] = arg_->repere_local_;
