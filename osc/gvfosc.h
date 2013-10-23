#ifndef GVFOSC_H
#define GVFOSC_H


///////////////////////////////////////////////////////////////////////
//
//  GVFOSC - Gesture Variation Follower OSC
//
//  
//  Copyright (C) 2013 Baptiste Caramiaux, Goldsmiths College, University of London
//
//  The GVF library is under the GNU Lesser General Public License (LGPL v3)
//  version: 09-2013
//
//  The interfacing with OSC has been realized by Marije Baalman
//  (c) 2013 - Marije Baalman, STEIM (www.steim.org)
//
//
///////////////////////////////////////////////////////////////////////

#include <lo/lo.h>

#include <Eigen/Core>
#include <vector>
#include <map>

#include "GestureVariationFollower.h"

#include <iostream>
#include <fstream>
#include <string>


enum {STATE_CLEAR, STATE_LEARNING, STATE_FOLLOWING};

class GvfInstance{
public: 
  GvfInstance( int statespace_dim );
  GvfInstance( int statespace_dim, int num_particles, float tolerance, float resampling_threshold );
  ~GvfInstance();
  
  int learnNew();
  void learnExisting( int id );
  void follow();
  void restart();
  void clear();
  void clearExisting( int id );
  
  int newdata( int size, std::vector<float> data );

  void saveVocabulary( std::string filename );
  void loadVocabulary( std::string filename );
  
  void setTolerance( float stdnew );
  void setNbOfParticles( int nb );
  void setResamplingThreshold( float rtnew );
  void setSpreadingMeans( std::vector<float> means );
  void setSpreadingRanges( std::vector<float> ranges );
  void setAdaptationSpeed( std::vector<float> as );
  void setTranslate( int tr );
  
//   std::map<int,std::vector<std::pair<float,float> > > *refmap;
//   float sp, sv, sr, ss; // pos,vel,rot,scal,observation
  int pdim;
  Eigen::VectorXf sigs;
  Eigen::VectorXf mpvrs;
  Eigen::VectorXf rpvrs;
  
  GestureVariationFollower * gvf;

  int translate;
  std::vector<float> vect_0_l;
  std::vector<float> vect_0_d;
  
  int state;
  int numberOfGestures;
  int currentToBeLearned;

  int restarted_l;
  int restarted_d;
};

//// end header

int create_handler(const char *path, const char *types, lo_arg **argv,
		    int argc, void *data, void *user_data);
int delete_handler(const char *path, const char *types, lo_arg **argv,
		    int argc, void *data, void *user_data);
int save_handler(const char *path, const char *types, lo_arg **argv,
		    int argc, void *data, void *user_data);
int load_handler(const char *path, const char *types, lo_arg **argv,
		    int argc, void *data, void *user_data);

int translate_handler(const char *path, const char *types, lo_arg **argv,
		    int argc, void *data, void *user_data);
int tolerance_handler(const char *path, const char *types, lo_arg **argv,
		    int argc, void *data, void *user_data);
int particles_handler(const char *path, const char *types, lo_arg **argv,
		    int argc, void *data, void *user_data);
int resampling_handler(const char *path, const char *types, lo_arg **argv,
		    int argc, void *data, void *user_data);
int spreadingmeans_handler(const char *path, const char *types, lo_arg **argv,
		    int argc, void *data, void *user_data);
int spreadingranges_handler(const char *path, const char *types, lo_arg **argv,
		    int argc, void *data, void *user_data);
int adaption_handler(const char *path, const char *types, lo_arg **argv,
		    int argc, void *data, void *user_data);

int data_handler(const char *path, const char *types, lo_arg **argv,
		    int argc, void *data, void *user_data);
int learnnew_handler(const char *path, const char *types, lo_arg **argv,
		    int argc, void *data, void *user_data);
int relearn_handler(const char *path, const char *types, lo_arg **argv,
		    int argc, void *data, void *user_data);
int follow_handler(const char *path, const char *types, lo_arg **argv,
		    int argc, void *data, void *user_data);
int restart_handler(const char *path, const char *types, lo_arg **argv,
		    int argc, void *data, void *user_data);
int clear_handler(const char *path, const char *types, lo_arg **argv,
		    int argc, void *data, void *user_data);
int cleargesture_handler(const char *path, const char *types, lo_arg **argv,
		    int argc, void *data, void *user_data);


void error(int num, const char *m, const char *path);

// int info_handler(const char *path, const char *types, lo_arg **argv,
// 		    int argc, void *data, void *user_data);
int generic_handler(const char *path, const char *types, lo_arg **argv,
		    int argc, void *data, void *user_data);
int quit_handler(const char *path, const char *types, lo_arg **argv, int argc,
		 void *data, void *user_data);



#endif