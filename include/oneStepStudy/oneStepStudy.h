/*
 *  Copyright AIST-CNRS Joint Robotics Laboratory
 *  Author: Nicolas Perrin
 */

#ifndef oneStepStudy_H
#define oneStepStudy_H

#include <MatrixAbstractLayer/MatrixAbstractLayer.h>
#include <dynamicsJRLJapan/dynamicsJRLJapanFactory.h>
//#ifdef WITH_HRP2DYNAMICS
#include <hrp2Dynamics/hrp2OptHumanoidDynamicRobot.h>
//#endif

#include <walkGenJrl/PatternGeneratorInterface.h>
#include <jrlMathTools/jrlConstants.h>
#include "CollisionDetection/HumanoidRobotCollisionDetection.h"
#include <robotDynamics/jrlRobotDynamicsObjectConstructor.h>
#include "filesManipulation.h"


#include <sys/time.h>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <vector>
#include <set>

#define PI 3.14159265359

using namespace::PatternGeneratorJRL;
using namespace std;

enum InitialPoses_t {
  HALF_SITTING_2003,
  TELEOPERATION_2008,
  HWPG_v1,
  HALF_SITTING_2008,
  INTERACTION_2008,
  MODEL_BUILDING_1,
  MODEL_BUILDING_2
};

/* double InitialPoses[7][42] = {

  // 1- With previous half-sitting value
  { 
    0.0, 0.0, -20.0, 40.0, -20.0, 0.0, 0.0, 0.0, -20.0, 40.0, -20.0, 0.0,  // legs

    0.0, 0.0, -23.7, 6.6, // chest and head

    27.0, -5.0, -4.0, -87.0, -4.0, -16.0, 0.0, 20.0, // right arm
    15.0,  10.0, 0.0, -20.0, 0.0, 0.0, 0.0,  10.0, // left arm 

    -20.0, 20.0, -20.0, 20.0, -20.0, // right hand
    -10.0, 10.0, -10.0, 10.0, -10.0  // left hand
  },
  // 2- Nicolas position + New half sitting for the legs
  { 
    0.0, 0.0, -26.0, 50.0, -24.0, 0.0, 0.0, 0.0, -26.0, 50.0, -24.0, 0.0,  // legs

    0.0, 0.0, -23.7, 6.6, // chest and head

    27.0, -5.0, -4.0, -87.0, -4.0, -16.0, 0.0,  20.0, // right arm
    15.0,  10.0, 0.0, -20.0, 0.0, 0.0,  0.0, 10.0, // left arm 

    -20.0, 20.0, -20.0, 20.0, -20.0, // right hand
    -10.0, 10.0, -10.0, 10.0, -10.0  // left hand
  },
  // 3- Test for comparison with PG v1.x
  { 
    0.0, 0.0, -26.0, 50.0, -24.0, 0.0, 0.0, 0.0, -26.0, 50.0, -24.0, 0.0,  // legs

    0.0, 0.0, 0.0, 0.0, // chest and head

    0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0, // right arm
    0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0, // left arm 

    0.0, 0.0, 0.0, 0.0, 0.0, // right hand
    0.0, 0.0, 0.0, 0.0, 0.0  // left hand
  },
  // 4- New Half sitting
  { 
    0.0, 0.0, -26.0, 50.0, -24.0, 0.0, 
    0.0, 0.0, -26.0, 50.0, -24.0, 0.0,  // legs

    0.0, 0.0, 0.0, 0.0, // chest and head

    15.0, -10.0, 0.0, -30.0, 0.0, 0.0, 0.0, 10.0, // right arm
    15.0,  10.0, 0.0, -30.0, 0.0, 0.0, 0.0, 10.0, // left arm 

    -10.0, 10.0, -10.0, 10.0, -10.0,  // right hand
    -10.0, 10.0, -10.0, 10.0, -10.0  // left hand
  },
  // 5- Position for interaction
  {
    0.0 ,  0.0 , -26.0 ,  50.0 , -24 ,   0.0,  
    0.0 ,  0.0 , -26.0 ,  50.0 , -24 ,   0.0 , // legs
    0.0 ,  0.0 ,  // chest
    0.0 ,  0.0 , // head 

    10.0, -18.0, 0.0, -100.0, -18.0, 0.0, 0.0,  10.0, // right arm  
    10.0,  18.0, 0.0, -100.0,  18.0, 0.0, 0.0,  10.0, 
    -10.000004 ,  10.000004 , -10.000004 ,  10.000004 , -10.000004 , // right hand 
    -10.000004 ,  10.000004 , -10.000004 ,  10.000004 , -10.000004 // left hand
  },
  // 6- Initial position for model building 1,
  {
    14.323945,  -6.0363396,  -13.459409,    44.02602,  -30.566611,    6.0363396,
    0.0000001,   7.4859801,  -27.663319,    44.65489,  -16.991579,   -7.4859801,
    0.,    0.,    0.,    0.,    
    12.397718,  -10.000004,    0.,  -29.618538,    0.,    0.,  0.0,    10.0,
    16.536364,   10.000004,    0.,  -29.828011,    0.,    0.,  0.0,    10.0,
    
    -10.0,  10.0, -10.0,  10,   -10.0, 
    -10.0,  10.0, -10.0,  10.0, -10.0 
  },
  // 7- Initial position for model buiding 2
  {
    -7.16197, -7.69299, -16.1787, 44.5201, -28.3415,  7.69299, 
    7.16197,   5.74946, -31.3668, 44.1057, -12.7389, -5.74946,
    
    0., 0., 0., 0., 
    
    12.622 , -10, 0, -29.678 , 0, 0, 0.0,  10, 
    16.7091,  10, 0, -29.7841, 0, 0, 0.0,  10, 
    
    -10.0,  10.0, -10.0,  10,   -10.0, 
    -10.0,  10.0, -10.0,  10.0, -10.0 
  }

};

*/

/*!
 * Small function to concatenate MAL vectors.
 * @param v1 reference to the first MAL vector.
 * @param v2 reference to the second MAL vector.
 * @param v3 reference to the MAL vector which will contain v1.v2
 * after the call to concatvectors(v1,v2,v3).
 * WARNING: v3 must already have the adequate size.
*/
void concatvectors(MAL_VECTOR(,double) & v1, MAL_VECTOR(,double) & v2, MAL_VECTOR(,double) & v3);

/*!
 * Enumeration type to pick between two possible choices (to use later
 * with the function setSteps()): cartesian
 * inputs or polar inputs.
*/
enum COORDINATES_CONVERSION { PolarINPUT, CartesianINPUT };

/*!
 * Structure containing all the parameters set by the user.
 */
struct ModeAndPropertiesToSet 
{
  /*!
   * If generate==1, then initAndGenOpenHRPFiles() will actually
   * modify the files used by OpenHRP.
   * If generate==0, then initAndGenOpenHRPFiles() will not access 
   * (no read, no write) to any file on the disk.
   */
  bool generate;

  /*!
   * The height of the Waist (in meters), which will keep the same
   * value during the steps.
   */
  double bodyHeight;

  /*!
   * Defines which foot will be stable during the first step. 
   * 'L': left foot. 'R': right foot.
   */
  char leftOrRightFirstStable; 

  /*!
   * Defines which type of coordinates are to be used with method setSteps()
   */
  COORDINATES_CONVERSION coordsConversion;

  /*!
   * Vector containing the pairs of bodies that are going to be checked
   * with vclip for collision detection.
   */
  vector<PaireOfBodies> pairsOfBodiesToCheck;  
};

/*!
 * Enumeration type.
 * The user can change the properties through the public member
 * m_modeAndProperties. Then he can activate the changes (the
 * properties are moved to private members). Changes can all be
 * activated simultaneously, but the user can also choose which
 * specific change he wants to activate. To do that he can use
 * activateModeAndPropertiesChanges(prpty), prpty being of type ModeAndPropertiesEnum
 */
enum ModeAndPropertiesEnum { Generate, BodyHeight, LeftOrRightFirstStable, CoordsConversion, PairsOfBodiesToCheck };

/*!
 * Structure containing the information on the result of the last call
 * to runPGAndEvalTrajectory().
 */
struct InfoOnLastRun 
{
  /*!
   * Initial distance to self-collision in cm
   */ 
  double initDistCol; 

  /*!
   * Distance min. to joints limits for legs in degrees
   */
  double distMinJointsLim; 
	
  /*!
   * Distance min. to self-collision in cm
   */
  double distMinSelfCol; 

  /*!
   * Variance of ZMP's deviation in cm^2
   */
  double varZMPdev; 

  /*!
   * Variance of ZMP's x-deviation during one-foot-up phase in cm^2
   */
  double varZMPdevX; 

  /*!
   * Variance of ZMP's y-deviation during one-foot-up phase in cm^2
   */
  double varZMPdevY; 

  /*!
   * ZMP's max. deviation in cm
   */
  double ZMPdevMax; 

  /*!
   * Number of PG iterations
   */
  int PGit; 

  /*!
   * Last configuration of the robot.
   */
  MAL_VECTOR(,double) lastConfig;

  /*!
   * =1 if there was a possible error in the run,
   * =0 if the run was OK.
   */
  bool possibleError;
};


/*!
 * Main class of the package.
 * It is called ConeStepStudy because it was initially intended to
 * study isolated single steps, but now it can actually study isolated
 * sequences of steps.
 * Uses the convention: m_ for public members, mp_ for private members.
 */
class ConeStepStudy {
  
  public:    
  
  /*!
   * Constructor.
   */
  ConeStepStudy();
  
  /*!
   * Destructor.
   */
  ~ConeStepStudy();

  /*!
   * Properties that are to be set by the user.
   */
  ModeAndPropertiesToSet m_modeAndProperties;
  
  /*!
   * Activates all the properties containing in m_modeAndProperties
   * and set by the user.
   */
  void activateModeAndPropertiesChanges();

  /*! 
   * Activates a specific property of m_modeAndProperties
   * @param prpty The property to activate.
   */ 
  void activateModeAndPropertiesChanges(ModeAndPropertiesEnum prpty);
  
  /*! setSteps is used by the user to define the sequence of steps
   *  that will be studied. 
   * @param vect_input Describes the sequence of steps.\n
   *  HERE IS A PRECISE DESCRIPTION OF THE FORMAT USED FOR vect_input:\n
   *           First remark: all angles are in degrees.\n
   *           Let's denote by "FStF" the foot which will be 
   *           stable during first step: the first stable foot.\n
   *           The first 6 parameters are the following:\n
   *                         We consider a plan (the floor) with a x
   *                       axis and a y axis. The orientation of the x
   *                       axis in this plan is 0.\n
   *                         The (0,0) origin of this plan corresponds to
   *                       the position of the waist and the CoM of
   *                       the robot (they are supposed identical).\n
   *                         The 3 first parameters correspond to the
   *                       FStF: \n
   *                       they are respectively the x position, y
   *                       position and (absolute) orientation of the 
   *                       FStF. \n
   *                         The orientation of the FStF will always be
   *                       supposed equal to ZERO ! Hence, the third
   *                       parameter (vect_input[2]) is superfluous:
   *                       it should always be 0 !!  \n
   *                         The 3 next parameters correspond to the
   *                       first swinging foot (FSwF): x position, y
   *                       orientation, (absolute) orientation.\n
   *                         Let's denote by x, y, t, x', y', t', the
   *                       first 6 parameters.\n
   *                         We suppose also that the position of the
   *                       CoM is at the BARYCENTER OF THE INITIAL
   *                       POSITIONS OF THE FEET; therefore we have:
   *                       -x' = x , and -y' = y. HENCE, the 6 first
   *                       parameters can in fact be generated with
   *                       only 3 parameters, and :\n
   *                         x,y,t,x',y',t' = -x',-y',0,x',y',t'\n
   *                           So, we need only the initial position
   *                         and orientation of the FSwF to generate
   *                         the 6 parameters.\n   
   *           The first 6 parameters are all ABSOLUTE coordinates,
   *           but the following parameters are all RELATIVE
   *           coordinates ! What follows is a list of groups of three
   *           parameters, each group corresponding to one step
   *           realized. Therefore, the number of steps is (n-6)/3 if
   *           the vector vect_input is of size n.\n
   *           Let's say that the vector vect_input has the form
   *           -x',-y',0,x',y',t', G_1, G_2, G_3 .... where the G_i
   *           are groups of 3 parameters. G_i corresponds to the ith 
   *           step.\n
   *           Each step has a stable foot and a swinging foot. 
   *           In CARTESIAN_INPUT:\n
   *              If G_i = (x_i, y_i, t_i),    (x_i, y_i) is the
   *              couple of coordinates (in meters) of the vector
   *              between the center of the stable foot and the
   *              swinging foot just after the ith step is finished
   *              (i.e. when the swinging foot touches the
   *              ground). Finally, t_i is the RELATIVE orientation
   *              (in degrees) of the final position of the swinging
   *              foot compared to the orientation of stable
   *              foot. For example, since the orientation of the
   *              first stable foot is 0, the parameter t_1 is the
   *              absolute orientation of the FSwF when the first step
   *              is just finished.\n
   *           In POLAR_INPUT:\n
   *              Similar, but G_i = (o_i, r_i, t_i), where o_i is in
   *              degrees and r_i in meters, (o_i, r_i) corresponding
   *              to the vector (x_i, y_i) described with polar
   *              coordinates.
   */ 
  void setSteps( 
		MAL_VECTOR(,double) vect_input 
		); 

  MAL_VECTOR(,double) getSteps() {

	return mp_sseq;

  };

  MAL_VECTOR(,double) getRadJointValues() {

	return mp_jointsRadValues;

  };
 
  /*!
   * Loads the files needed by the Pattern Generator.
   * Remark: the *.ptc files describing the robot's bodies' geometry
   * are not loaded with loadFiles() but must be in the folder where
   * the program is launched.  
   */ 
  void loadFiles(		 
		 string PCParametersFile_, 
		 string VRMLPath_, 
		 string VRMLFileName_, 
		 string SpecificitiesFileName_, 
		 string LinkJointRank_, 
		 string openHRPxml_		 
		 );

  /*!
   * From the loaded files, builds all the objects that will be used
   * to perform the study of a sequence of steps (which does not need
  to be already defined here).
   */
  void buildNecessaryInternalStructures();

  /*!
   * Initializes the objects created by
   * buildNecessaryInternalStructures() to make the study of the sequence
   * of steps possible.
   * The sequence of steps must have been already defined (with the
   * method setSteps()).
   * If m_modeAndProperties was activated with generate==1, then
   * OpenHRP files are modified so that the sequence of step can be
   * visualized with OpenHRP.
   */ 
  void initAndGenOpenHRPFiles();
    
  /*!
   * runs the Pattern Generator (with some constraints that help to
  get a unique trajectory), executes the sequence of steps and
   * evaluates the trajectory.
   */
  void runPGAndEvalTrajectory(); 

  /*!
   * Prints the results corresponding to the last execution of runPGAndEvalTrajectory().  
   */
  void printResults();
  
  /*! 
   * Contains the information on the result of the last call
   * to runPGAndEvalTrajectory().
   */ 
  InfoOnLastRun m_lastRunInfo;

  /*!
   * Draws steps in a file intendedfor gnuplot. Plot index 0 with
   * "lines", index 1 with "labels", and preferably set no autoscale.
   * @param fb The ofstream corresponding to the file where the data
  will be written.
   */
  void drawSteps(ofstream & fb);


 private: 

  
  bool mp_generate;
  double mp_bodyHeight;
  char mp_leftOrRightFirstStable;
  COORDINATES_CONVERSION mp_coordsConversion;

  vector<PaireOfBodies> mp_pairsOfBodiesToCheck;  
  set<int> mp_setOfBodies;
  vector<int> mp_vectOfBodies;

  MAL_VECTOR(,double) mp_stepsInput;
  MAL_VECTOR(,double) mp_stepsVector;
  MAL_VECTOR(,double) mp_sseq;
  

  string mp_PCParametersFile;
  string mp_VRMLPath;
  string mp_VRMLFileName;
  string mp_SpecificitiesFileName;
  string mp_LinkJointRank; 
  string mp_openHRPxml;

  PatternGeneratorInterface * mp_aPGI;
  Chrp2OptHumanoidDynamicRobot * mp_HDR;
  HumanoidRobotCollisionDetection * mp_aHRCD;

  //coordinates of the initial positions of the ANKLES:
  //first stable ankle:
  double mp_foot1X1;
  double mp_foot1Y1;
  double mp_foot1Theta1;
  //first swinging ankle:
  double mp_foot2X1; 
  double mp_foot2Y1;
  double mp_foot2Theta1;
  
  //initial joints values and configuration:
  MAL_VECTOR(,double) mp_jointsRadValues;
  MAL_VECTOR(,double) mp_currentConfiguration;
  MAL_VECTOR(,double) mp_currentVelocity;
  MAL_VECTOR(,double) mp_currentAcceleration;
  MAL_VECTOR(,double) mp_previousConfiguration;
  MAL_VECTOR(,double) mp_previousVelocity;
  MAL_VECTOR(,double) mp_previousAcceleration;
  MAL_VECTOR(,double) mp_ZMPTarget;

  //number of iterations of the PG:
  int mp_NbOfIt;
  
  /*!
   * Defines the command that will be sent to the Pattern Generator.
   */
  void commandToDo(
		     PatternGeneratorInterface &aPGI_, 
		     MAL_VECTOR(,double));    


};

#endif
