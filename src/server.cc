


#include <hpp/util/pointer.hh>
#include <hpp/core/basic-configuration-shooter.hh>
#include <hpp/core/config-validations.hh>
#include <hpp/core/connected-component.hh>
#include <hpp/core/constraint-set.hh>
#include <hpp/core/path-planner.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/problem-solver.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/steering-method.hh>
#include <hpp/corbaserver/server.hh>
#include <hpp/core/fwd.hh>
#include <hpp/core/distance-between-objects.hh>
#include <iostream>

#include "hpp/interactive/planner.hh"


using namespace std;

// main function of the server
int main (int argc, const char* argv[])
{
  // create a ProblemSolver instance.
  // This class is a container that does the interface between hpp-core library
  // and component to be running in a middleware like CORBA or ROS.
  hpp::core::ProblemSolverPtr_t problemSolver = 
	hpp::core::ProblemSolver::create();


  // Add the new planner type in order to be able to select it from python
  // client.
	problemSolver->addPathPlannerType (
			"interactive", hpp::interactive::Planner::createWithRoadmap);
  // Create the CORBA server.
  hpp::corbaServer::Server server (problemSolver, argc, argv, true);
  // Start the CORBA server.
  server.startCorbaServer ();
  cout << "serveur démarré\n";
  // Wait for CORBA requests.
  server.processRequest (true);
  cout << "wait corba req\n";
}
/*
char** pt = new char*[argc];

  for (int i=0; i< argc; i++){
    pt[i] = new char[256];
    int j=0;
    cout << "copie de argv[" << i << "]=";
    while(argv[i][j] != '\0'){
      pt[i][j] = argv[i][j];
      j++;
    }   
    cout << pt[i] << endl;
  }

  // pour changer le premier argument
  
  // pour changer la dernière lettre du premeir argument
  int i = 0;
  cout << "cherche fin argv[0]\n";
  while(argv[0][i] != '\0'){
    i++;
  }
  cout << endl;
  pt[0][i] = 'u';
  pt[0][i+1] = '\0';

  const char * strs[] = {pt[0]};
  const char **p = strs;
  cout << p[0] << endl;*/
