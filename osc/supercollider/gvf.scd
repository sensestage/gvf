OSCFunc.trace( true );

n = NetAddr.new( "127.0.0.1", 57151 );

// create a gesture follower:
n.sendMsg( "/gvf/create", 1, 4 );

// learn a gesture:
n.sendMsg( "/gesture/learn", 1 );
// returns /gesture/learn/succes follower gesture id

n.sendMsg( "/gesture/data", 1, 0.2, 0.4, 0.2 );
n.sendMsg( "/gesture/data", 1, 0.25, 0.45, 0.25 );

n.sendMsg( "/gesture/follow", 1 );

n.sendMsg( "/gesture/data", 1, 0.2, 0.4, 0.2 );
n.sendMsg( "/gesture/data", 1, 0.25, 0.45, 0.25 );
