  //state machine that lets us keep track of what we command the robot to do 
  
  //these are just enumerations of the possible states.  They should really be defines--they're just taking up space
  #define TRANSLATE  1
  #define ROTATE     2
  #define STOPPED    3
  
  //the variable that actually holds the modes.  Start out not moving
  int mode = STOPPED; 


  //number of loops the robot goes through with its errors under a threshhold before it decides it's completed an action
  #define STABLE_COUNT 100
