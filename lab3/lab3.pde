 /**
 **********************************************************************************************************************
 * @file       Haptic_Physics_Template.pde
 * @author     Steve Ding, Colin Gallacher
 * @version    V3.0.0
 * @date       27-September-2018
 * @brief      Base project template for use with pantograph 2-DOF device and 2-D physics engine
 *             creates a blank world ready for creation
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */
 
 
 
 /* library imports *****************************************************************************************************/ 
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
/* end library imports *************************************************************************************************/  



/* scheduler definition ************************************************************************************************/ 
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/ 



/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;

byte              widgetOneID                         = 5;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           rendering_force                     = false;
/* end device block definition *****************************************************************************************/



/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/ 



/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerCentimeter                 = 40.0;

/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           pos_ee                              = new PVector(0, 0);
PVector           f_ee                                = new PVector(0, 0); 

/* World boundaries */
FWorld            world;
float             worldWidth                          = 24.0;  
float             worldHeight                         = 14.0; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;

float dRed ;
float dBlue;

float lasttimecheck;
float timeinterval ;
float t            ;


/* Initialization of virtual tool */
HVirtualCoupling  s;

/* Define areas*/
FBox wall  ;
FBox div   ;
FBox kill  ;
FBox glass1;
FBox glass2;
FBox glass3;
FBox   l1;
FBox   l2;
FBox   l3;
//FBox   field1;
//FBox   field2;
//FBox   field0;
//FBox   field3;
//FBox   field4;
FBox trap    ;
FCircle coil ;
FBox battery;
FBox field;


/* end elements definition *********************************************************************************************/



/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(965, 575);
  
  
  /* device setup */
  
  /**  
   * The board declaration needs to be changed depending on which USB serial port the Haply board is connected.
   * In the base example, a connection is setup to the first detected serial device, this parameter can be changed
   * to explicitly state the serial port will look like the following for different OS:
   *
   *      windows:      haplyBoard = new Board(this, "COM10", 0);
   *      linux:        haplyBoard = new Board(this, "/dev/ttyUSB0", 0);
   *      mac:          haplyBoard = new Board(this, "/dev/cu.usbmodem1411", 0);
   */
  haplyBoard          = new Board(this, Serial.list()[0], 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();
  
  println(Serial.list());
  
  widgetOne.set_mechanism(pantograph);
  
  widgetOne.add_actuator(1, CCW, 2);
  widgetOne.add_actuator(2, CW, 1);
 
  widgetOne.add_encoder(1, CCW, 241, 10752, 2);
  widgetOne.add_encoder(2, CW, -61, 10752, 1);
  
  widgetOne.device_set_parameters();
  
  
  /* 2D physics scaling and world creation */
  hAPI_Fisica.init(this); 
  hAPI_Fisica.setScale(pixelsPerCentimeter); 
  world               = new FWorld();
  
  
  /* Glasses Setup */
  
  wall = new FBox(2.5,0.25);
  wall.setPosition(worldWidth/2-9.5, worldHeight/2-1.5);
  wall.setNoFill();
  wall.setNoStroke();
  wall.setStatic(true);
  world.add(wall);
  
  wall = new FBox(2.5,0.25);
  wall.setPosition(worldWidth/2-6.5, worldHeight/2-1.5);
  wall.setNoFill();
  wall.setNoStroke();
  wall.setStatic(true);  
  world.add(wall);
  
  wall = new FBox(2.5,0.25);
  wall.setPosition(worldWidth/2-3.5, worldHeight/2-1.5);
  wall.setNoFill();
  wall.setNoStroke();
  wall.setStatic(true);  
  world.add(wall);
  
  wall = new FBox(0.25,3.5);
  wall.setPosition(worldWidth/2-10.625, worldHeight/2-3.375);
  wall.setNoFill();
  wall.setNoStroke();
  wall.setStatic(true);  
  world.add(wall);
  
  wall = new FBox(0.25,3.5);
  wall.setPosition(worldWidth/2-7.625, worldHeight/2-3.375);
  wall.setNoFill()    ;
  wall.setNoStroke()  ;
  wall.setStatic(true);  
  world.add(wall)     ;
  
  wall = new FBox(0.25,3.5);
  wall.setPosition(worldWidth/2-4.625, worldHeight/2-3.375);
  wall.setNoFill()    ;
  wall.setNoStroke()  ;
  wall.setStatic(true); 
  world.add(wall)     ;
  
  wall = new FBox(0.25,3.5);
  wall.setPosition(worldWidth/2-8.375, worldHeight/2-3.375);
  wall.setNoFill()    ;
  wall.setNoStroke()  ;
  wall.setStatic(true);
  world.add(wall)     ;
  
  wall = new FBox(0.25,3.5);
  wall.setPosition(worldWidth/2-5.375, worldHeight/2-3.375);
  wall.setNoFill()    ;
  wall.setNoStroke()  ;
  wall.setStatic(true);
  world.add(wall)     ;
  
  wall = new FBox(0.25,3.5);
  wall.setPosition(worldWidth/2-2.375, worldHeight/2-3.375);
  wall.setNoFill()    ;
  wall.setNoStroke()  ;
  wall.setStatic(true);
  world.add(wall)     ;
  
  
  /* Set HONEY layer */
  l1                  = new FBox(2,3);
  l1.setPosition(worldWidth/2-9.5,worldHeight/2-3.125);
  l1.setNoFill()      ;
  l1.setDensity(100)  ;
  l1.setSensor(true)  ;
  l1.setNoStroke()    ;
  l1.setStatic(true)  ;
  l1.setName("Honey") ;
  world.add(l1)       ;
  
  /* Set OLIVE */
  l2                  = new FBox(2,3);
  l2.setPosition(worldWidth/2-6.5,worldHeight/2-3.125);
  l2.setNoFill()      ;
  l2.setDensity(100)  ;
  l2.setSensor(true)  ;
  l2.setNoStroke()    ;
  l2.setStatic(true)  ;
  l2.setName("Olive") ;
  world.add(l2)       ;
  
  /* Set WATER */
  l3                  = new FBox(2,3);
  l3.setPosition(worldWidth/2-3.5,worldHeight/2-3.125);
  l3.setNoFill()      ;
  l3.setDensity(100)  ;
  l3.setSensor(true)  ;
  l3.setNoStroke()    ;
  l3.setStatic(true)  ;
  l3.setName("Water") ;
  world.add(l3)       ;
  
  
  /* Set Divisions */
  
  div = new FBox(21,0.25);
  div.setPosition(worldWidth/2+0.5, worldHeight/2);
  div.setFill(130,130,130);
  div.setStatic(true);  
  world.add(div);
  
  div = new FBox(0.25,4.5);
  div.setPosition(worldWidth/2, worldHeight/2-2.5);
  div.setFill(130,130,130);
  div.setStatic(true);  
  world.add(div);
  
  kill = new FBox(10.5,1);
  kill.setPosition(worldWidth/2+5.5, worldHeight/2-0.5);
  kill.setNoFill()    ;
  kill.setNoStroke()  ;
  kill.setStatic(true);  
  kill.setSensor(true);
  world.add(kill)     ;

  
  /* Set Electromagnet */
  
  //int n = 20              ;
  //float xO = 3            ;
  //float yO = 8.5          ;
  //float incrementX = 0.250;
  //float incrementY = 0.125;
  //FCircle cirO            ;
  //FCircle cird            ;
  //FCircle cirX            ;
  
  //for (int i = 0; i < n; i++){
   
  // cirO = new FCircle(0.75);
  // cirO.setPosition(xO, yO);
  // cirO.setNoFill()        ;
  // cirO.setStatic(true) ;
  // world.add(cirO)      ;
   
  // cird = new FCircle(0.10);
  // cird.setPosition(xO, yO);
  // cird.setFill(255,0,0)   ;
  // cird.setStatic(true) ;
  // world.add(cird)      ;
   
  // cirX = new FCircle(0.75)  ;
  // cirX.setPosition(xO, yO+3);
  // cirX.setNoFill()          ;
  // cirX.setStatic(true) ;
  // world.add(cirX)      ;
   
  // cirX = new FCircle(0.75)  ;
  // cirX.setPosition(xO, yO+3);
  // cirX.setNoFill()          ;
  // cirX.setStatic(true) ;
  // world.add(cirX)      ;
   
  // cird = new FCircle(0.10);
  // cird.setPosition(xO, yO+3);
  // cird.setFill(0,0,255)   ;
  // cird.setStatic(true) ;
  // world.add(cird)      ;
   
  // xO = xO + 0.75;
   
  //}
  
  
  //field1 = new FBox(5,2);
  //field1.setPosition(worldWidth/2-7, worldHeight/3.5+6);
  //field1.setFill(100,100,100,10);
  //field1.setNoStroke();
  //field1.setSensor(true);
  //field1.setStatic(true);  
  //world.add(field1);
  
  //field0 = new FBox(5,2);
  //field0.setPosition(worldWidth/2-2, worldHeight/3.5+6);
  //field0.setFill(100,100,100,10);
  //field0.setNoStroke();
  //field0.setSensor(true);
  //field0.setStatic(true);  
  //world.add(field0);
  
  //field2 = new FBox(5,2);
  //field2.setPosition(worldWidth/2+3, worldHeight/3.5+6);
  //field2.setFill(100,100,100,10);
  //field2.setNoStroke();
  //field2.setSensor(true);
  //field2.setStatic(true);  
  //world.add(field2);
  
  
  
  /* Set Charged particles */
  
  //pos = new FCircle(0.75);
  //pos.setPosition(worldWidth/2-7, worldHeight/2+3.5);
  //pos.setFill(200,0,0);
  //pos.setStatic(true) ;
  //pos.setSensor(true) ;
  //world.add(pos)      ;
  
  //neg = new FCircle(0.75);
  //neg.setPosition(worldWidth/2+7, worldHeight/2+3.5);
  //neg.setFill(0,0,200);
  //neg.setStatic(true) ;
  //neg.setSensor(true) ;
  //world.add(neg)      ;
  
  //blue = new FCircle(4);
  //blue.setPosition(worldWidth/2+7, worldHeight/2+3.5);
  //blue.setNoFill()       ;
  //blue.setSensor(true)   ;
  //blue.setStroke(0,0,255,50);
  //blue.setStatic(true) ;
  //world.add(blue)      ;
  
  //red = new FCircle(4);
  //red.setPosition(worldWidth/2-7, worldHeight/2+3.5);
  //red.setNoFill()       ;
  //red.setSensor(true)   ;
  //red.setStroke(255,0,0,50);
  //red.setStatic(true) ;
  //world.add(red)      ;
  
  //float HaplyX = s.h_avatar.getX();
  //float HaplyY = s.h_avatar.getY();
  //float pposX = worldWidth/2-7   ;
  //float pposY = worldHeight/2+3.5;
  //float pnegX = worldWidth/2+7   ;
  //float pnegY = worldHeight/2+3.5;
  
  
  /* Magnet & Electromagnet */
  
  battery = new FBox(4,2);
  battery.setPosition(worldWidth/2, worldHeight/2+4.75);
  battery.setNoFill()    ;
  battery.setNoStroke()  ;
  battery.setStatic(true);  
  world.add(battery)     ;
  
  battery = new FBox(0.5,1);
  battery.setPosition(worldWidth/2-2.25, worldHeight/2+4.75);
  battery.setNoFill()    ;
  battery.setNoStroke()  ;
  battery.setStatic(true);  
  world.add(battery)     ;
  
  battery = new FBox(6,0.1);
  battery.setPosition(worldWidth/2-5.5, worldHeight/2+4.75);
  battery.setNoFill()    ;
  battery.setNoStroke()  ;
  battery.setStatic(true);
  world.add(battery)     ;
  
  battery = new FBox(6,0.1);
  battery.setPosition(worldWidth/2+5, worldHeight/2+4.75);
  battery.setNoFill()    ;
  battery.setNoStroke()  ;
  battery.setStatic(true);
  world.add(battery)     ;
  
  battery = new FBox(0.1,2.5);
  battery.setPosition(worldWidth/2-8.5, worldHeight/2+3.5);
  battery.setNoFill()    ;
  battery.setNoStroke()  ;
  battery.setStatic(true);  
  world.add(battery)     ;
  
  battery = new FBox(0.1,2.5);
  battery.setPosition(worldWidth/2+8, worldHeight/2+3.5);
  battery.setNoFill()    ;
  battery.setNoStroke()  ;
  battery.setStatic(true);  
  world.add(battery)     ;
  
  int n = 17;
  float xO = worldWidth/2-8.25;
  float yO = worldHeight/2+1.5;
  
  for (int i = 0; i < n; i++){
    coil = new FCircle(1.5)  ;
    coil.setPosition(xO, yO) ;
    coil.setNoFill()         ;
    coil.setSensor(false)    ;
    coil.setNoStroke()       ;
    coil.setStatic(true)     ;
    world.add(coil)          ;
    
    xO = xO + 1; 
  }
  
  
  /* Timer */
  lasttimecheck = millis();
  timeinterval = 250      ;
  
  
  /* Setup the Virtual Coupling Contact Rendering Technique */
  s                   = new HVirtualCoupling((0.5));
  s.h_avatar.setDensity(2); 
  s.h_avatar.setFill(0,26,255); 
  s.init(world, worldWidth/2, edgeTopLeftY+2); 
  
  /* World conditions setup */
  world.setGravity((0.0), (300.0)); //1000 cm/(s^2)
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(0.5);
  world.setEdgesFriction(1);
  
  world.draw();
  
  /* setup framerate speed */
  frameRate(baseFrameRate);
  
  /* setup simulation thread to run at 1kHz */ 
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}


/* draw section ********************************************************************************************************/

void draw(){
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  
  background(255);
  world.draw()   ;
  
}
/* end draw section ****************************************************************************************************/


int forces = 0;

/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable{
  
  public void run(){
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */
    rendering_force = true;
    
    if(haplyBoard.data_available()){
      widgetOne.device_read_data();
      forces = 0;
      angles.set(widgetOne.get_device_angles()); 
      pos_ee.set(widgetOne.get_device_position(angles.array()));
      pos_ee.set(pos_ee.copy().mult(200));  
    }
    
    s.setToolPosition(edgeTopLeftX+worldWidth/2-(pos_ee).x+2, edgeTopLeftY+(pos_ee).y-7); 
    s.updateCouplingForce()                    ;
    f_ee.set(-s.getVCforceX(), s.getVCforceY());
    f_ee.div(20000)                            ;


/* MAGNETIC FIELD */
 
    //if((s.h_avatar.isTouchingBody(field1)) || (s.h_avatar.isTouchingBody(field2))){
    //  f_ee.x=-1 ;
    //  f_ee.y=0.0;
    //}
    //else if(s.h_avatar.isTouchingBody(field0)){
    //  f_ee.x=-4.0;
    //  f_ee.y=0.0 ;
    //}
    
    // Failed attempts for ATTRACT/REPEL
    
    //if ( (s.h_avatar.getY() > worldHeight/2) && (s.h_avatar.getX() < worldWidth/2) ){
    
    //  if ( (s.h_avatar.getX() <= worldWidth/2-7) && (s.h_avatar.getY() <= worldHeight/2+3) ) {
    //    f_ee.x = -4/(8*dRed);
    //    f_ee.y = 4 /(8*dRed);
    //  }
    //  else if ( (s.h_avatar.getX() >= worldWidth/2-7) && (s.h_avatar.getY() <= worldHeight/2+3) ){
        
    //    f_ee.x = 4/(8*dRed);
    //    f_ee.y = 4/(8*dRed);
    //  }
    //  else if ( (s.h_avatar.getX() <= worldWidth/2-7) && (s.h_avatar.getY() >= worldHeight/2+3) ) {
    //    f_ee.x = -4/(8*dRed);
    //    f_ee.y = -4/(8*dRed);
    //  }
    //  else if ( (s.h_avatar.getX() >= worldWidth/2-7) && (s.h_avatar.getY() >= worldHeight/2+3) ) {
    //    f_ee.x =  4/(8*dRed);
    //    f_ee.y = -4/(8*dRed);
    //  }
    //}
    
    torques.set(widgetOne.set_device_torques(f_ee.array()));
    widgetOne.device_write_torques();
    
    
/* FLUIDS */

    if (s.h_avatar.isTouchingBody(l1)){
      s.h_avatar.setDamping(600);
    }
    else if (s.h_avatar.isTouchingBody(l2)){
      s.h_avatar.setDamping(300);
    }
    else if (s.h_avatar.isTouchingBody(l3)){
      s.h_avatar.setDamping(100);
    }
    else{
      s.h_avatar.setDamping(50);
    }


/* TRAPPED */

    if((s.h_avatar.getX() > worldWidth/2+1) && (s.h_avatar.getY() < worldHeight/2)){
      
      t = millis() - lasttimecheck;
      
      if (millis() > lasttimecheck + timeinterval){
        lasttimecheck = millis()  ;
      
        trap = new FBox(0.4,0.4);
        trap.setPosition(s.h_avatar.getX(), s.h_avatar.getY()-1);
        trap.setFill(100,100,100,50);
        trap.setDensity(500) ;
        trap.setNoStroke()   ;
        trap.setSensor(false);
        trap.setStatic(false);  
        world.add(trap)      ;      
      }
    // If the following is activated, the word TRAPPED might change to WARZONE
    //if(trap.isTouchingBody(kill)){
    //  world.remove(trap);
    //}
    }
    
    world.step(1.0f/1000.0f);
    rendering_force = false;
  }
}
/* end simulation section **********************************************************************************************/



/* helper functions section, place helper functions here ***************************************************************/

/* end helper functions section ****************************************************************************************/
