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
// (c) 2013 - Marije Baalman, STEIM (www.steim.org)
//
//
///////////////////////////////////////////////////////////////////////

#include <unistd.h>
#include "gvfosc.h"

GvfInstance::GvfInstance( int statespace_dim ){
  
  // maybe I should pass these too, or set later?
  float sp = 0.00001;
  float sv = 0.0001;
  float ss = 0.0001;
  float sr = 0.0000001;
  float tolerance = 0.15;

  pdim = statespace_dim; // e.g. 4
  
  sigs = Eigen::VectorXf( pdim );
  sigs << sp, sv, ss, sr;
    
  gvf = new GestureVariationFollower( 2000, sigs, 1./(tolerance*tolerance), 500, 0. ); // what are the arguments?
  // number of particles - state space dimensions stddev?significants?, global tolerance/variance, resampling threshold, student distribution parameter nu
  
  // maybe I should pass these too, or set later?
  mpvrs = Eigen::VectorXf( pdim );
  mpvrs << 0.05, 1.0, 1.0, 0.0;
  rpvrs = Eigen::VectorXf( pdim );
  rpvrs << 0.1, 0.4, 0.3, 0.0;
  
  restarted_l = 1;
  restarted_d = 1;
  
  state = STATE_CLEAR;
  numberOfGestures = 0;
  currentToBeLearned = -1;
  translate = 1;
}

/// start classfile
GvfInstance::GvfInstance( int statespace_dim, int num_particles, float tolerance, float resampling_threshold ){
  
  // maybe I should pass these too, or set later?
  float sp = 0.00001;
  float sv = 0.0001;
  float ss = 0.0001;
  float sr = 0.0000001;

  pdim = statespace_dim; // e.g. 4
  
  sigs = Eigen::VectorXf( pdim );
  sigs << sp, sv, ss, sr;
  
//   refmap = new std::map<int,std::vector<std::pair<float,float> > >;
  
//   int num_particles = 2000;
  gvf = new GestureVariationFollower( num_particles, sigs, 1./(tolerance*tolerance), resampling_threshold, 0. ); // what are the arguments?
  // number of particles - state space dimensions stddev?significants?, global tolerance/variance, resampling threshold, student distribution parameter nu
  
  // maybe I should pass these too, or set later?
  mpvrs = Eigen::VectorXf( pdim );
  mpvrs << 0.05, 1.0, 1.0, 0.0;
  rpvrs = Eigen::VectorXf( pdim );
  rpvrs << 0.1, 0.4, 0.3, 0.0;
  
  restarted_l = 1;
  restarted_d = 1;
  
  state = STATE_CLEAR;
  numberOfGestures = 0;
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
  if(id >= numberOfGestures){
    printf("reference %d not known", id);
    // inform about template not available
    return;
  }
  printf("clearing reference %d", id);
  gvf->clearTemplate( id );
  // inform about clearing template
}

void GvfInstance::learnExisting( int id ){
  currentToBeLearned=id;
  if(id >= numberOfGestures){
    printf("you need to learn reference %d first", numberOfGestures);
    // inform about template not available
    return;
  }
  printf("modifying reference %d", id);
  gvf->clearTemplate( id );
  
  state = STATE_LEARNING;
  restarted_l=1;
  
  // inform about changing template
}

int GvfInstance::learnNew(){
  currentToBeLearned = numberOfGestures;
  gvf->addTemplate();
  numberOfGestures++;
  state = STATE_LEARNING;
  restarted_l=1;
    // inform about number of templates
  return currentToBeLearned;
}

void GvfInstance::follow(){
  if(numberOfGestures>0){
      gvf->spreadParticles( mpvrs, rpvrs );
      state = STATE_FOLLOWING;
      // inform about now in following mode
  } else {
      printf("no reference has been learned");
      // inform about no references there
      return;
  }
}

int GvfInstance::newdata( int size, std::vector<float> data ){
  if ( state == STATE_CLEAR ){
     printf("what am I supposed to do ... I'm in standby!");
     // inform about nothing to do, or just do nothing.
     return -1;
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
    return 0;
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
    return 1;
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
    numberOfGestures = gvf->getNbOfTemplates();
}

void GvfInstance::clear(){
    numberOfGestures = 0;
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
    gvf->setNumberOfParticles( nb );
}

void GvfInstance::setResamplingThreshold( float rtnew ){
    int cNS = gvf->getNbOfParticles();
    if( rtnew >= cNS ){
	rtnew = floor( cNS/2 ); // why half of particles, and why not check for larger than half of particles?
    }
    gvf->setResamplingThreshold( rtnew );
}

void GvfInstance::setSpreadingMeans( std::vector<float> means ){
  mpvrs = Eigen::VectorXf(pdim);
  for ( int k=0; k<pdim; k++ ){
      mpvrs << means[k];
  }
}

void GvfInstance::setSpreadingRanges( std::vector<float> ranges ){
  rpvrs = Eigen::VectorXf(pdim);
  for ( int k=0; k<pdim; k++ ){
      rpvrs << ranges[k];
  }
}

void GvfInstance::setAdaptationSpeed( std::vector<float> as ){
  gvf->setAdaptSpeed( as ); // which dimension should this have? ---> ok, also pdim, this is the same as the sig that is passed in earlier on
}

void GvfInstance::setTranslate( int tr ){
  translate = tr;
}

//// END GESTURE FOLLOWER INSTANCE

// START OF osc PART
int done = 0;

lo_address t;
lo_server s;
// lo_server_thread st;

typedef std::map<int, GvfInstance* > gvf_map_t;

gvf_map_t followers;

GvfInstance * getFollower( int id ){
  /// find in map
  // return NULL if not found, else return the found one
  GvfInstance * thisone = followers.find( id )->second;
  return thisone;  
}

void createFollower( int id, int pdim ){
    GvfInstance * newgvf = new GvfInstance( pdim ); // set some default values!
    followers[ id ] = newgvf;
}

void destroyFollower( int id ){
    GvfInstance * thisone = getFollower( id );
    if ( thisone != NULL ){
	followers.erase( id );
	delete thisone;
    }
}

void sendEstimation( int id ){
    GvfInstance * thisone = getFollower( id );
    if ( thisone == NULL ){
      return;
    }
    GestureVariationFollower * thisgvf = thisone->gvf;
    Eigen::MatrixXf statu = thisgvf->getEstimatedStatus();
    Eigen::VectorXf gprob = thisgvf->getGestureConditionnalProbabilities();
    
    // phase, speed, scaling, rotation
    // statu.rows() -> number of gestures; statu(j,0 .. 3 )
    //FIXME: send osc outputs:
    // per gesture known ---- probability, phase, speed, scaling, rotation
    // /gesture/variation ... gprob(j) statu(j,0) statu(j,1) statu(j,2) statu(j,3)    
    
    lo_bundle b = lo_bundle_new( LO_TT_IMMEDIATE );

    for(int j = 0; j < statu.rows(); j++){
      lo_message m = lo_message_new();
      lo_message_add_int32( m, id ); // follower
      lo_message_add_int32( m, j );  // gesture
//       lo_message_add_float( m, gprob( j, 0 ) ); // probability
      // iterate over other dimensions:
      for ( int k=0; k<statu.cols(); k++ ){
	lo_message_add_float( m, statu(j,k) );      
      }
      lo_bundle_add_message( b, "/gesture/estimation", m );      
    }
    lo_message m = lo_message_new();
    lo_message_add_int32( m, id ); // follower
   for(int j = 0; j < statu.rows(); j++){
      lo_message_add_float( m, gprob( j, 0 ) ); // probability
      // iterate over other dimensions:      
    }
    lo_bundle_add_message( b, "/gesture/probabilities", m );      
    if ( lo_send_bundle_from( t, s, b )  == -1 ){
	printf("gesture update: OSC error %d: %s\n", lo_address_errno(t), lo_address_errstr(t) );
    }
//     lo_bundle_free_messages(b);
    lo_bundle_free(b);
}

int create_handler(const char *path, const char *types, lo_arg **argv, int argc,
		 void *data, void *user_data)
{
  int followerid = argv[0]->i;
    // check if ID already exists, send out warning that it exists
  if ( getFollower( followerid ) == NULL ){
    // create a new gesture follower with a certain ID, and dimensions
    createFollower( followerid, argv[1]->i );
    lo_send_from( t, s, LO_TT_IMMEDIATE, "/gvf/create/succes", "i", followerid );
  } else {
    lo_send_from( t, s, LO_TT_IMMEDIATE, "/gvf/create/error", "is", followerid, "follower with this id already exists" );
  }
  return 0;
}

int delete_handler(const char *path, const char *types, lo_arg **argv, int argc,
		 void *data, void *user_data)
{
  int followerid = argv[0]->i;
  if ( getFollower( followerid ) == NULL ){
    // send out warning
    lo_send_from( t, s, LO_TT_IMMEDIATE, "/gvf/delete/error", "is", followerid, "follower with this id does not exist" );
  } else {
    destroyFollower( followerid );
    lo_send_from( t, s, LO_TT_IMMEDIATE, "/gvf/delete/succes", "i", followerid );
  }
  return 0;
}

int save_handler(const char *path, const char *types, lo_arg **argv, int argc,
		 void *data, void *user_data)
{
  int followerid = argv[0]->i;
  GvfInstance * follower = getFollower( followerid );
  if ( follower == NULL ){
    lo_send_from( t, s, LO_TT_IMMEDIATE, "/gvf/save/error", "is", followerid, "follower with this id does not exist" );
  } else {
//     follower->saveVocabulary( argv[1]->s );
    lo_send_from( t, s, LO_TT_IMMEDIATE, "/gvf/save/succes", "i", followerid );
  }
  return 0;
}

int load_handler(const char *path, const char *types, lo_arg **argv, int argc,
		 void *data, void *user_data)
{
  int followerid = argv[0]->i;
  GvfInstance * follower = getFollower( followerid );
  if ( follower == NULL ){
    lo_send_from( t, s, LO_TT_IMMEDIATE, "/gvf/load/error", "is", followerid, "follower with this id does not exist" );
  } else {
//     std::string filename = argv[1]->s;
//     follower->loadVocabulary( filename );
    lo_send_from( t, s, LO_TT_IMMEDIATE, "/gvf/load/succes", "i", followerid );
  }
  return 0;
}

int translate_handler(const char *path, const char *types, lo_arg **argv, int argc,
		 void *data, void *user_data)
{
  int followerid = argv[0]->i;
  GvfInstance * follower = getFollower( followerid );
  if ( follower == NULL ){
    lo_send_from( t, s, LO_TT_IMMEDIATE, "/gvf/translate/error", "is", followerid, "follower with this id does not exist" );
  } else {
    follower->setTranslate( argv[1]->i );
    lo_send_from( t, s, LO_TT_IMMEDIATE, "/gvf/translate/succes", "i", followerid );
  }
  return 0;
}

int tolerance_handler(const char *path, const char *types, lo_arg **argv, int argc,
		 void *data, void *user_data)
{
  int followerid = argv[0]->i;
  GvfInstance * follower = getFollower( followerid );
  if ( follower == NULL ){
    lo_send_from( t, s, LO_TT_IMMEDIATE, "/gvf/tolerance/error", "is", followerid, "follower with this id does not exist" );
  } else {
    follower->setTolerance( argv[1]->f );
    lo_send_from( t, s, LO_TT_IMMEDIATE, "/gvf/tolerance/succes", "i", followerid );
  }
  return 0;
}

int particles_handler(const char *path, const char *types, lo_arg **argv, int argc,
		 void *data, void *user_data)
{
  int followerid = argv[0]->i;
  GvfInstance * follower = getFollower( followerid );
  if ( follower == NULL ){
    lo_send_from( t, s, LO_TT_IMMEDIATE, "/gvf/particles/error", "is", followerid, "follower with this id does not exist" );
  } else {
    follower->setNbOfParticles( argv[1]->i );
    lo_send_from( t, s, LO_TT_IMMEDIATE, "/gvf/particles/succes", "i", followerid );
  }
  return 0;
}

int resampling_handler(const char *path, const char *types, lo_arg **argv, int argc,
		 void *data, void *user_data)
{
  int followerid = argv[0]->i;
  GvfInstance * follower = getFollower( followerid );
  if ( follower == NULL ){
    lo_send_from( t, s, LO_TT_IMMEDIATE, "/gvf/resamplingthreshold/error", "is", followerid, "follower with this id does not exist" );
  } else {
    follower->setResamplingThreshold( argv[1]->f );
    lo_send_from( t, s, LO_TT_IMMEDIATE, "/gvf/resamplingthreshold/succes", "i", followerid );
  }
  return 0;
}

int spreadingmeans_handler(const char *path, const char *types, lo_arg **argv, int argc,
		 void *data, void *user_data)
{
  int followerid = argv[0]->i;
  GvfInstance * follower = getFollower( followerid );
  if ( follower == NULL ){
    lo_send_from( t, s, LO_TT_IMMEDIATE, "/gvf/spreadingmeans/error", "is", followerid, "follower with this id does not exist" );
  } else {
    std::vector<float> values( argc-1 );
    for ( int k=1; k<argc; k++ ){
      values[k-1] = argv[k]->f;
    }
    follower->setSpreadingRanges( values );
    lo_send_from( t, s, LO_TT_IMMEDIATE, "/gvf/spreadingmeans/succes", "i", followerid );
  }
  return 0;
}

int spreadingranges_handler(const char *path, const char *types, lo_arg **argv, int argc,
		 void *data, void *user_data)
{
  int followerid = argv[0]->i;
  GvfInstance * follower = getFollower( followerid );
  if ( follower == NULL ){
    lo_send_from( t, s, LO_TT_IMMEDIATE, "/gvf/spreadingranges/error", "is", followerid, "follower with this id does not exist" );
  } else {
    std::vector<float> values( argc-1 );
    for ( int k=1; k<argc; k++ ){
      values[k-1] = argv[k]->f;
    }
    follower->setSpreadingRanges( values );
    lo_send_from( t, s, LO_TT_IMMEDIATE, "/gvf/spreadingranges/succes", "i", followerid );
  }
  return 0;
}

int adaption_handler(const char *path, const char *types, lo_arg **argv, int argc,
		 void *data, void *user_data)
{
  int followerid = argv[0]->i;
  GvfInstance * follower = getFollower( followerid );
  if ( follower == NULL ){
    lo_send_from( t, s, LO_TT_IMMEDIATE, "/gvf/adaptationspeed/error", "is", followerid, "follower with this id does not exist" );
  } else {
    std::vector<float> values( argc-1 );
    for ( int k=1; k<argc; k++ ){
      values[k-1] = argv[k]->f;
    }
    follower->setAdaptationSpeed( values );
    lo_send_from( t, s, LO_TT_IMMEDIATE, "/gvf/adaptationspeed/succes", "i", followerid );
  }
  return 0;
}


int data_handler(const char *path, const char *types, lo_arg **argv, int argc,
		 void *data, void *user_data)
{
  int followerid = argv[0]->i;
  GvfInstance * follower = getFollower( followerid );
  if ( follower == NULL ){
    lo_send_from( t, s, LO_TT_IMMEDIATE, "/gesture/data/error", "is", followerid, "follower with this id does not exist" );
  } else {
    std::vector<float> values( argc-1 );
    for ( int k=1; k<argc; k++ ){
      values[k-1] = argv[k]->f;
    }
    int res = follower->newdata( argc-1, values );
    
    if ( res == 1 ){
      sendEstimation( followerid );
    } else if ( res == -1 ){
      lo_send_from( t, s, LO_TT_IMMEDIATE, "/gesture/data/error", "is", followerid, "follower with this id does not have anything to do yet" );      
    }
//     lo_send_from( t, s, LO_TT_IMMEDIATE, "/gvf/data/succes", "i", followerid );
  }
  return 0;
}

int learnnew_handler(const char *path, const char *types, lo_arg **argv, int argc,
		 void *data, void *user_data)
{
  int followerid = argv[0]->i;
  GvfInstance * follower = getFollower( followerid );
  if ( follower == NULL ){
    lo_send_from( t, s, LO_TT_IMMEDIATE, "/gesture/learn/error", "is", followerid, "follower with this id does not exist" );
  } else {
    int gestureid = follower->learnNew();
    lo_send_from( t, s, LO_TT_IMMEDIATE, "/gesture/learn/succes", "ii", followerid, gestureid );
  }
  return 0;
}

int relearn_handler(const char *path, const char *types, lo_arg **argv, int argc,
		 void *data, void *user_data)
{
  int followerid = argv[0]->i;
  GvfInstance * follower = getFollower( followerid );
  if ( follower == NULL ){
    lo_send_from( t, s, LO_TT_IMMEDIATE, "/gesture/learn/error", "is", followerid, "follower with this id does not exist" );
  } else {
    // could return the gesture id
    follower->learnExisting( argv[1]->i );
    lo_send_from( t, s, LO_TT_IMMEDIATE, "/gesture/learn/succes", "i", followerid );
  }
  return 0;
}


int clear_handler(const char *path, const char *types, lo_arg **argv, int argc,
		 void *data, void *user_data)
{
  int followerid = argv[0]->i;
  GvfInstance * follower = getFollower( followerid );
  if ( follower == NULL ){
    lo_send_from( t, s, LO_TT_IMMEDIATE, "/gesture/clear/error", "is", followerid, "follower with this id does not exist" );
  } else {
    follower->clear();
    lo_send_from( t, s, LO_TT_IMMEDIATE, "/gesture/clear/succes", "i", followerid );
  }
  return 0;
}

int cleargesture_handler(const char *path, const char *types, lo_arg **argv, int argc,
		 void *data, void *user_data)
{
  int followerid = argv[0]->i;
  GvfInstance * follower = getFollower( followerid );
  if ( follower == NULL ){
    lo_send_from( t, s, LO_TT_IMMEDIATE, "/gesture/clear/error", "is", followerid, "follower with this id does not exist" );
  } else {
    int gestureid = argv[1]->i;
    follower->clearExisting( gestureid );
    lo_send_from( t, s, LO_TT_IMMEDIATE, "/gesture/clear/succes", "ii", followerid, gestureid );
  }
  return 0;
}


int follow_handler(const char *path, const char *types, lo_arg **argv, int argc,
		 void *data, void *user_data)
{
  int followerid = argv[0]->i;
  GvfInstance * follower = getFollower( followerid );
  if ( follower == NULL ){
    lo_send_from( t, s, LO_TT_IMMEDIATE, "/gesture/follow/error", "is", followerid, "follower with this id does not exist" );
  } else {
    follower->follow();
    lo_send_from( t, s, LO_TT_IMMEDIATE, "/gesture/follow/succes", "i", followerid );
  }
  return 0;
}

int restart_handler(const char *path, const char *types, lo_arg **argv, int argc,
		 void *data, void *user_data)
{
  int followerid = argv[0]->i;
  GvfInstance * follower = getFollower( followerid );
  if ( follower == NULL ){
    lo_send_from( t, s, LO_TT_IMMEDIATE, "/gesture/restart/error", "is", followerid, "follower with this id does not exist" );
  } else {
    follower->restart();
    lo_send_from( t, s, LO_TT_IMMEDIATE, "/gesture/restart/succes", "i", followerid );
  }
  return 0;
}

/* catch any incoming messages and display them. returning 1 means that the
 * message has not been fully handled and the server should try other methods */
int generic_handler(const char *path, const char *types, lo_arg **argv,
		    int argc, void *data, void *user_data)
{
    int i;

    printf("path: <%s>\n", path);
    for (i=0; i<argc; i++) {
	printf("arg %d '%c' ", i, types[i]);
// 	lo_arg_pp(types[i], argv[i]);
	printf("\n");
    }
    printf("\n");
    fflush(stdout);

    return 1;
}

int quit_handler(const char *path, const char *types, lo_arg **argv, int argc,
		 void *data, void *user_data)
{
    done = 1;
    printf("gvfosc: allright, that's it, I quit\n");
    fflush(stdout);

    return 0;
}

void error(int num, const char *msg, const char *path)
{
     printf("liblo server error %d in path %s: %s\n", num, path, msg);
     fflush(stdout);
}

int main(int argc, char** argv)
{

    char *myport = "57151";
    char *dest_port = "57120";
    char *dest_ip = "127.0.0.1";

    if ( argc == 4 ){
      dest_ip = argv[3];
      myport = argv[2];
      dest_port = argv[1];
    } else if ( argc == 3 ){
      myport = argv[2];
      dest_port = argv[1];
    } else if ( argc == 2 ){
      dest_port = argv[1];      
    }

    printf("============================================================================\n");
    printf("gvfosc - v0.1 - osc controllable gesture variation follower\n");
    printf("                     (c) 2013, Marije Baalman\n");
    printf("                http://www.nescivi.eu/gvfosc\n");
    printf("Written using gvf and liblo\n");
    printf("This is free software released under the GNU/General Public License\n");
    printf("start with \"gvfosc <target_port> <recv_port> <target_ip>\" \n");
    printf("============================================================================\n\n");
    printf("Listening to port: %s\n", myport );
    printf("Sending to ip and port: %s %s\n", dest_ip, dest_port );
    fflush(stdout);

//     print_help();

    /* create liblo addres */
    t = lo_address_new(dest_ip, dest_port); // change later to use other host

    /// there's not really a need for a separate thread in this case, so I could just make a blocking server
//     lo_server_thread st = lo_server_thread_new(myport, error);
    lo_server s = lo_server_new( myport, error );

    lo_server_add_method( s, "/gvf/create", "ii", create_handler, NULL);
    lo_server_add_method( s, "/gvf/delete", "i", delete_handler, NULL);
    lo_server_add_method( s, "/gvf/save", "is", save_handler, NULL);
    lo_server_add_method( s, "/gvf/load", "is", load_handler, NULL);
    
    lo_server_add_method( s, "/gvf/translate", "ii", translate_handler, NULL);
    lo_server_add_method( s, "/gvf/tolerance", "if", tolerance_handler, NULL);
    lo_server_add_method( s, "/gvf/particles", "ii", particles_handler, NULL);
    lo_server_add_method( s, "/gvf/resamplingthreshold", "if", resampling_handler, NULL);
    lo_server_add_method( s, "/gvf/spreadingmeans", NULL, spreadingmeans_handler, NULL);
    lo_server_add_method( s, "/gvf/spreadingranges", NULL, spreadingranges_handler, NULL);
    lo_server_add_method( s, "/gvf/adaptationspeed", NULL, adaption_handler, NULL);
    
    lo_server_add_method( s, "/gesture/data", NULL, data_handler, NULL);
    lo_server_add_method( s, "/gesture/learn", "i", learnnew_handler, NULL);
    lo_server_add_method( s, "/gesture/learn", "ii", relearn_handler, NULL);
    lo_server_add_method( s, "/gesture/follow", "i", follow_handler, NULL);
    lo_server_add_method( s, "/gesture/restart", "i", restart_handler, NULL);    
    lo_server_add_method( s, "/gesture/clear", "i", clear_handler, NULL);
    lo_server_add_method( s, "/gesture/clear", "ii", cleargesture_handler, NULL);
        
    lo_server_add_method( s, "/gvfosc/quit", "", quit_handler, NULL);
    lo_server_add_method( s, NULL, NULL, generic_handler, NULL);

    /// there's not really a need for a separate thread in this case, so I could just make a blocking server
//     lo_server_thread_start(st);
 
//     lo_server s = lo_server_thread_get_server( st );

    lo_send_from( t, s, LO_TT_IMMEDIATE, "/gvfosc/started", "" );

        
    while( !done ){
      lo_server_recv(s);
//       usleep(1000);
    }

    lo_send_from( t, s, LO_TT_IMMEDIATE, "/gvfosc/quit", "" );
//     lo_server_thread_free( st );
    lo_server_free( s );
    lo_address_free( t );

    return 0;
}
