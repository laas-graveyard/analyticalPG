/*
 *  Copyright AIST-CNRS Joint Robotics Laboratory
 *  Author: Nicolas Perrin
 */

#ifndef newPGstepStudy_H
#define newPGstepStudy_H

#include <MatrixAbstractLayer/MatrixAbstractLayer.h>
#include <dynamicsJRLJapan/dynamicsJRLJapanFactory.h>
//#ifdef WITH_HRP2DYNAMICS
#include <hrp2Dynamics/hrp2OptHumanoidDynamicRobot.h>
//#endif

#include <jrlMathTools/jrlConstants.h>
#include "CollisionDetection/HumanoidRobotCollisionDetection.h"
#include <robotDynamics/jrlRobotDynamicsObjectConstructor.h>
#include "filesManipulation.h"
#include "oneStepStudy.h"

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

using namespace std;

/*!
 * Main class of the package.
 * It is called ConeStepStudy because it was initially intended to
 * study isolated single steps, but now it can actually study isolated
 * sequences of steps.
 * Uses the convention: m_ for public members, mp_ for private members.
 */

struct StepFeatures {
	vector<double> comTrajX;
	vector<double> zmpTrajX;
	vector<double> comTrajY;
	vector<double> zmpTrajY;
	vector<double> leftfootXtraj;
	vector<double> leftfootYtraj;
	vector<double> leftfootHeight;
	vector<double> leftfootOrient;
	vector<double> rightfootXtraj;
	vector<double> rightfootYtraj;
	vector<double> rightfootHeight;
	vector<double> rightfootOrient;
	vector<double> waistOrient;	
	double incrTime;
	double zc;
	unsigned int size;
};

class CnewPGstepStudy
{

	public:

		/*!
		 * Constructor.
		 */
		CnewPGstepStudy();

		/*!
		 * Destructor.
		 */
		~CnewPGstepStudy();

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

		MAL_VECTOR(,double) getSteps()
		{
			return mp_sseq;
		};

		MAL_VECTOR(,double) getRadJointValues()
		{

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
		 * runs the new PG (with some constraints that help to
		get a unique trajectory), executes the sequence of steps and
		 * evaluates the trajectory.
		 */
		void runPGAndEvalTrajectory();

		/*!
		 * Draws steps in a file intendedfor gnuplot. Plot index 0 with
		 * "lines", index 1 with "labels", and preferably set no autoscale.
		 * @param fb The ofstream corresponding to the file where the data
		will be written.
		 */
		void drawSteps(ofstream & fb);


		void genCOMtrajectoryOFSTREAM(ofstream & fb, double incrTime, double zc, double g, double delta0, double deltaX, double deltaX2, double t1, double t2, double t3, double t4, double t5);

		void genCOMZMPtrajectory(vector<double>& outputCOM, vector<double>& outputZMP, double incrTime, double zc, double g, double delta0, double deltaX, double deltaX2, double t1, double t2, double t3, double t4, double t5);

		void genFOOTposition(vector<double>& outputX, vector<double>& outputY, double incrTime, double xinit, double yinit, double xend, double yend, double delay, double t1, double t2, double t3, double t4, double t5);

		void genFOOTheight(vector<double>& output, double incrTime, double heightMax, double t1, double t2, double t3, double t4, double t5);

		void genFOOTupDOWNheight(vector<double> & output, double incrTime, double heightMax, double t1, double t2, double t3);

		void genFOOTdownUPheight(vector<double> & output, double incrTime, double heightMax, double t1, double t2, double t3);

		void genFOOTorientation(vector<double>& output, double incrTime, double initOrient, double endOrient, double delay, double t1, double t2, double t3, double t4, double t5);

		void genWAISTorientation(vector<double>& output, double incrTime, double initOrient, double endOrient, double delay, double t1, double t2, double t3, double t4, double t5);



		void genFullBodyConfig(int count, MAL_VECTOR(,double) & jointsRadValues, vector<double> & comTrajX, vector<double> & comTrajY, vector<double> & waistOrient, vector<double> & footXtraj, vector<double> & footYtraj, vector<double> & footOrient, vector<double> & footHeight, double positionXstableFoot, double positionYstableFoot, char leftOrRightFootStable, double zc); 

		void genFullBodyTrajectoryFromStepFeatures(ofstream & fb, ofstream & fbZMP, StepFeatures & stepF); 

		void addStepFeaturesWithSlide(StepFeatures & stepF1, StepFeatures & stepF2, double negativeSlideTime); 

		void produceOneStep(vector<vector<double> > & fb, vector<vector<double> > & fbZMP, double incrTime, double zc, double g, double stepHeight, double t1, double t2, double t3, double t4, double t5, vector<double> vectStep_input, char leftOrRightFootStable);

		void produceOneStep(ofstream & fb, ofstream & fbZMP, double incrTime, double zc, double g, double stepHeight, double t1, double t2, double t3, double t4, double t5, vector<double> vectStep_input, char leftOrRightFootStable);

		void produceOneUPHalfStep(vector<vector<double> > & fb, vector<vector<double> > & fbZMP, double incrTime, double zc, double g, double t1, double t2, double t3, vector<double> vectUPHalfStep_input, char leftOrRightFootStable);

		void produceOneUPHalfStep(ofstream & fb, ofstream & fbZMP, double incrTime, double zc, double g, double t1, double t2, double t3, vector<double> vectUPHalfStep_input, char leftOrRightFootStable);

		void produceOneUPHalfStepFeatures(StepFeatures & stepF, double incrTime, double zc, double g, double t1, double t2, double t3, vector<double> vectStep_input, char leftOrRightFootStable);

		void produceOneDOWNHalfStep(vector<vector<double> > & fb, vector<vector<double> > & fbZMP, double incrTime, double zc, double g, double t1, double t2, double t3, vector<double> vectDOWNHalfStep_input, char leftOrRightFootStable);

		void produceOneDOWNHalfStep(ofstream & fb, ofstream & fbZMP, double incrTime, double zc, double g, double t1, double t2, double t3, vector<double> vectDOWNHalfStep_input, char leftOrRightFootStable);

		void produceOneDOWNHalfStepFeatures(StepFeatures & stepF, double incrTime, double zc, double g, double t1, double t2, double t3, vector<double> vectStep_input, char leftOrRightFootStable);


		void produceSeqSteps(vector<vector<double> > & fb, vector<vector<double> > & fbZMP, double incrTime, double zc, double g, double stepHeight, double t1, double t2, double t3, double t4, double t5, vector<double> vectSteps_input, char leftOrRightFootStable);

		void produceSeqSteps(ofstream & fb, ofstream & fbZMP, double incrTime, double zc, double g, double stepHeight, double t1, double t2, double t3, double t4, double t5, vector<double> vectSteps_input, char leftOrRightFootStable);

		void produceSeqHalfSteps(vector<vector<double> > & fb, vector<vector<double> > & fbZMP, double incrTime, double zc, double g, double t1, double t2, double t3, vector<double> vectSteps_input, char leftOrRightFootStable);

		void produceSeqHalfSteps(ofstream & fb, ofstream & fbZMP, double incrTime, double zc, double g, double t1, double t2, double t3, vector<double> vectSteps_input, char leftOrRightFootStable);

		void produceSeqLinkedHalfSteps(ofstream & fb, ofstream & fbZMP, double incrTime, double zc, double g, double t1, double t2, double t3, vector<double> vectSteps_input, char leftOrRightFootStable, double negativeSlideTime);

		void produceGlobalLinkedCOMZMP(vector<double> & gCOMx, vector<double> & gCOMy, vector<double> & gZMPx, vector<double> & gZMPy, double incrTime, double zc, double g, double t1, double t2, double t3, double t4, double t5, vector<double> vectSteps_input, char leftOrRightFootStable);

		void plotGlobalLinkedCOMZMP(ofstream & fb, double incrTime, double zc, double g, double t1, double t2, double t3, double t4, double t5, vector<double> vectSteps_input, char leftOrRightFootStable);

		void produceGlobalSeparateCOMZMP(vector<double> & gCOMx, vector<double> & gCOMy, vector<double> & gZMPx, vector<double> & gZMPy, double incrTime, double zc, double g, double t1, double t2, double t3, double t4, double t5, vector<double> vectSteps_input, char leftOrRightFootStable);

		void plotGlobalSeparateCOMZMP(ofstream & fb, double incrTime, double zc, double g, double t1, double t2, double t3, double t4, double t5, vector<double> vectSteps_input, char leftOrRightFootStable);

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
};
#endif
