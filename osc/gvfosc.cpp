
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
//
//
///////////////////////////////////////////////////////////////////////

#include <lo/lo.h>

#include <Eigen/Core>
#include <vector>
#include <map>

#include "GestureVariationFollower.h"
#include "globalutilities.h"

#include <iostream>
#include <fstream>
#include <string>


enum {STATE_CLEAR, STATE_LEARNING, STATE_FOLLOWING};

class GvfInstance{
public: 
  GvfInstance();
  ~GvfInstance();
  
  void learnNew();
  void learnExisting( int id );
  void follow();
  void restart();
  void clear();
  void clearGesture( int id );
  
  void newdata( int size, std::vector<float> *data );

  void saveVocabulary( std::string filename );
  void loadVocabulary( std::string filename );
  
  void setTolerance( float stdnew );
  void setNbOfParticles( int nb );
  void setResamplingThreshold( float rtnew );
  void setSpreadingMeans( vector<float> *means );
  void setSpreadingRanges( vector<float> *ranges );
  void setAdaptationSpeed( vector<float> *as );
  void setTranslate( int tr );
  
//   std::map<int,std::vector<std::pair<float,float> > > *refmap;
//   float sp, sv, sr, ss; // pos,vel,rot,scal,observation
  int pdim;
  Eigen::VectorXf sigs;
  Eigen::VectorXf mpvrs;
  Eigen::VectorXf rpvrs;

  int translate;
  std::vector<float> vect_0_l;
  std::vector<float> vect_0_d;
  
  int state;
  int lastReferenceLearned;
  int currentToBeLearned;

  int restarted_l;
  int restarted_d;
};

//// end header

/// start classfile
GvfInstance::GvfInstance( int num_particles, int statespace_dim, float tolerance, float resampling_threshold ){
  
  // maybe I should pass these too, or set later?
  float sp = 0.00001;
  float sv = 0.0001;
  float ss = 0.0001;
  float sr = 0.0000001;

  pdim = statespace_dim; // e.g. 4
  
  sigs = Eigen::VectorVf( pdim );
  sigs << sp, sv, ss, sr;
  
//   refmap = new std::map<int,std::vector<std::pair<float,float> > >;
  
//   int num_particles = 2000;
  gvf = new GestureVariationFollower( num_particles, sigs, 1./(tolerance*tolerance), resampling_threshold, 0. ); // what are the arguments?
  // number of particles - state space dimensions stddev?significants?, global tolerance/variance, resampling threshold, student distribution parameter nu
  
  // maybe I should pass these too, or set later?
  mpvrs = Eigen::VectorVf( pdim );
  mpvrs << 0.05, 1.0, 1.0, 0.0;
  rpvrs = Eigen::VectorVf( pdim );
  rpvrs << 0.1, 0.4, 0.3, 0.0;
  
  restarted_l = 1;
  restarted_d = 1;
  
  state = STATE_CLEAR;
  lastReferenceLearned = -1;
  currentToBeLearned = -1;
  translate = 1;
}

GvfInstance::~GvfInstance(){
  if( gvf != NULL)
    delete gvf;
//   if( refmap != NULL)
//     delete refmap;
}

void GvfInstance::clearExisting( int id ){
//   currentToBeLearned=id;
  if(id > lastReferenceLearned){
    post("reference %d not known", id);
    // inform about template not available
    return;
  }
  post("clearing reference %d", id);
  gvf->clearTemplate( id );
  // inform about clearing template
}

void GvfInstance::learnExisting( int id ){
  currentToBeLearned=id;
  if(id > lastReferenceLearned){
    post("you need to learn reference %d first", lastReferenceLearned);
    // inform about template not available
    return;
  }
  post("modifying reference %d", id);
  gvf->clearTemplate( id );
  
  state = LEARNING;
  restarted_l=1;
  
  // inform about changing template
}

void GvfInstance::learnNew(){
  currentToBeLearned = lastReferenceLearned;
  gvf->addTemplate();
  lastReferenceLearned++;
  state = LEARNING;
  restarted_l=1;
    // inform about number of templates
}

void GvfInstance::follow(){
  if(lastReferenceLearned>0){
      gvf->spreadParticles( mpvrs, rpvrs );
      state = STATE_FOLLOWING;
      // inform about now in following mode
  } else {
      post("no reference has been learned");
      // inform about no references there
      return;
  }
}

//FIXME: pass in vector with data
void GvfInstance::newdata( int size, std::vector<float> *data ){
  if ( state == STATE_CLEAR ){
     post("what am I supposed to do ... I'm in standby!");
     // inform about nothing to do, or just do nothing.
     return;
  }
  else if ( state == STATE_LEARNING ){
    std::vector<float> vect(size);
    if ( translate ){
	if ( restarted_l==1 ){
	  for ( int k=0; k<size; k++ ){
	    vect[k] = data[k];
	  }
	  vect_0_l = vect;
	  restarted_l = 0;
	} else {
	  for ( int k=0; k<size; k++ ){
	    vect[k] = data[k] - vect_0_l[k];
	  }
	}
    } else { // no auto-translate
      for ( int k=0; k<size; k++ ){
	vect[k] = data[k];
      }
    }
    gvf->fillTemplate( currentToBeLearned, vect );
  }
  else if ( state == STATE_FOLLOWING ){
    std::vector<float> vect(size);
    if ( translate ){
	if ( restarted_d==1 ){
	  for ( int k=0; k<size; k++ ){
	    vect[k] = data[k];
	  }
	  vect_0_d = vect;
	  restarted_d = 0;
	} else {
	  for ( int k=0; k<size; k++ ){
	    vect[k] = data[k] - vect_0_d[k];
	  }
	}
    } else { // no auto-translate
      for ( int k=0; k<size; k++ ){
	vect[k] = data[k];
      }
    }
    gvf->infer( vect );
    Eigen::MatrixXf statu = gvf->getEstimatedStatus();
    Eigen::VectorXf gprob = gvf->getGestureConditionnalProbabilities();
    
    // phase, speed, scaling, rotation
    // statu.rows() -> number of gestures; statu(j,0 .. 3 )
    //FIXME: send osc outputs:
    // per gesture known ---- probability, phase, speed, scaling, rotation
    // /gesture/variation ... gprob(j) statu(j,0) statu(j,1) statu(j,2) statu(j,3)    
  }
}

void GvfInstance::restart(){
    restarted_l = 1;
    if ( state == STATE_FOLLOWING ){
      restarted_d = 1;
      gvf->spreadParticles( mpvrs, rpvrs );
    }
}

void GvfInstance::saveVocabulary( std::string filename ){
    gvf->saveTemplates( filename );
}

void GvfInstance::loadVocabulary( std::string filename ){
    gvf->loadTemplates( filename );
    lastReferenceLearned = gvf->getNbOfTemplates();
}

void GvfInstance::clear(){
    lastReferenceLearned = -1;
    gvf->clear();
    restarted_l = 1;
    restarted_d = 1;
    state = STATE_CLEAR;
}

void GvfInstance::setTolerance( float stdnew ){
    if ( stdnew == 0.0 ){
	stdnew = 0.1;
    }
    gvf->setIcovSingleValue( 1 / ( stdnew*stdnew ) ); // why taking power of two, if we take square root again?
}

void GvfInstance::setNbOfParticles( int nb ){
    gvf->setNbOfParticles( nb );
}

void GvfInstance::setResamplingThreshold( float rtnew ){
    int cNS = gvf->getNbOfParticles();
    if( rtnew >= cNS ){
	rtnew = floor( cNS/2 ); // why half of particles, and why not check for larger than half of particles?
    }
    gvf->setResamplingThreshold( rtnew );
}

void GvfInstance::setSpreadingMeans( vector<float> *means ){
  mpvrs = Eigen::VectorXf(pdim);
  for ( int k=0; k<pdim; k++ ){
      mpvrs << means[k];
  }
}

void GvfInstance::setSpreadingRanges( vector<float> *ranges ){
  rpvrs = Eigen::VectorXf(pdim);
  for ( int k=0; k<pdim; k++ ){
      fpvrs << ranges[k];
  }
}

void GvfInstance::setAdaptationSpeed( vector<float> *as ){
  gvf->setAdaptSpeed( *as ); // which dimension should this have? ---> ok, also pdim, this is the same as the sig that is passed in earlier on
}

void GvfInstance::setTranslate( int tr ){
  translate = tr;
}

//// END GESTURE FOLLOWER INSTANCE

// START OF osc PART
int done = 0;
int paused = 0;

lo_address t;
lo_server s;
lo_server_thread st;

void error(int num, const char *m, const char *path);
int info_handler(const char *path, const char *types, lo_arg **argv,
		    int argc, void *data, void *user_data);
int pause_handler(const char *path, const char *types, lo_arg **argv,
		    int argc, void *data, void *user_data);


int generic_handler(const char *path, const char *types, lo_arg **argv,
		    int argc, void *data, void *user_data);
int quit_handler(const char *path, const char *types, lo_arg **argv, int argc,
		 void *data, void *user_data);
