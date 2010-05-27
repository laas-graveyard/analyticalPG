//  Copyright AIST-CNRS Joint Robotics Laboratory
//  Author: Nicolas Perrin

/*! System Headers */
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <vector>
#include <set>

/*! JRL Headers */
#include <jrlMathTools/jrlConstants.h>

#include <MatrixAbstractLayer/MatrixAbstractLayer.h>

#include <robotDynamics/jrlRobotDynamicsObjectConstructor.h>

#include <dynamicsJRLJapan/dynamicsJRLJapanFactory.h>

#include "CollisionDetection/HumanoidRobotCollisionDetection.h"

#include "newPGstepStudy.h"

#include "InverseKinematics.h"

#define DEBUGJOINTS(x)

using namespace std;

CnewPGstepStudy::CnewPGstepStudy()
{
	mp_HDR = NULL;
	mp_aHRCD = NULL;
}


CnewPGstepStudy::~CnewPGstepStudy()
{
	if(mp_HDR != NULL)
	{
		delete mp_HDR;
	}
	if(mp_aHRCD != NULL)
	{
		delete mp_aHRCD;
	}
}


//WARNING: this activates all the values at the same time, so they
//should all be up-to-date.
void CnewPGstepStudy::activateModeAndPropertiesChanges()
{
	mp_generate = m_modeAndProperties.generate;
	mp_bodyHeight = m_modeAndProperties.bodyHeight;
	mp_leftOrRightFirstStable = m_modeAndProperties.leftOrRightFirstStable;
	mp_coordsConversion = m_modeAndProperties.coordsConversion;

	mp_pairsOfBodiesToCheck = m_modeAndProperties.pairsOfBodiesToCheck;

	mp_setOfBodies.clear();
	mp_vectOfBodies.clear();

	for(unsigned int i = 0 ; i<mp_pairsOfBodiesToCheck.size() ; i++ )
	{

		mp_setOfBodies.insert(mp_pairsOfBodiesToCheck[i].body1);
		mp_setOfBodies.insert(mp_pairsOfBodiesToCheck[i].body2);
		mp_pairsOfBodiesToCheck[i].body1++;
		mp_pairsOfBodiesToCheck[i].body2++;

	}

	for( set<int>::const_iterator iter = mp_setOfBodies.begin();
		iter != mp_setOfBodies.end();
		++iter )
	{
		mp_vectOfBodies.insert(mp_vectOfBodies.end(),*iter);
	}

}


//this is for selective activation:
void CnewPGstepStudy::activateModeAndPropertiesChanges(ModeAndPropertiesEnum Selected)
{

	switch (Selected)
	{
		case Generate: mp_generate = m_modeAndProperties.generate;break;
		case BodyHeight: mp_bodyHeight = m_modeAndProperties.bodyHeight;break;
		case LeftOrRightFirstStable: mp_leftOrRightFirstStable = m_modeAndProperties.leftOrRightFirstStable;break;
		case CoordsConversion: mp_coordsConversion = m_modeAndProperties.coordsConversion;
		case PairsOfBodiesToCheck:
		{
			mp_pairsOfBodiesToCheck = m_modeAndProperties.pairsOfBodiesToCheck;
			mp_setOfBodies.clear();
			mp_vectOfBodies.clear();

			for(unsigned int i = 0 ; i<mp_pairsOfBodiesToCheck.size() ; i++ )
			{

				mp_setOfBodies.insert(mp_pairsOfBodiesToCheck[i].body1);
				mp_setOfBodies.insert(mp_pairsOfBodiesToCheck[i].body2);
				mp_pairsOfBodiesToCheck[i].body1++;
				mp_pairsOfBodiesToCheck[i].body2++;

			}

			for( set<int>::const_iterator iter = mp_setOfBodies.begin();
				iter != mp_setOfBodies.end();
				++iter )
			{
				mp_vectOfBodies.insert(mp_vectOfBodies.end(),*iter);
			}
		}
		break;
		default: break;
	}
}


void CnewPGstepStudy::setSteps(MAL_VECTOR(,double) vect_input)
{

	mp_stepsInput = vect_input;

	//change in case of polar input:
	if (mp_coordsConversion == PolarINPUT)
	{
		mp_stepsVector.clear();
		mp_stepsVector = vect_input;

		//from feet to ankles:
		if(mp_leftOrRightFirstStable == 'R')
		{
			mp_stepsVector[0]+=-cos(mp_stepsVector[2]*PI/180-PI/2)*0.035;
			mp_stepsVector[1]+=-sin(mp_stepsVector[2]*PI/180-PI/2)*0.035;
			mp_stepsVector[3]+=-cos(mp_stepsVector[5]*PI/180+PI/2)*0.035;
			mp_stepsVector[4]+=-sin(mp_stepsVector[5]*PI/180+PI/2)*0.035;
		}
		else if(mp_leftOrRightFirstStable == 'L')
		{
			mp_stepsVector[3]+=-cos(mp_stepsVector[5]*PI/180-PI/2)*0.035;
			mp_stepsVector[4]+=-sin(mp_stepsVector[5]*PI/180-PI/2)*0.035;
			mp_stepsVector[0]+=-cos(mp_stepsVector[2]*PI/180+PI/2)*0.035;
			mp_stepsVector[1]+=-sin(mp_stepsVector[2]*PI/180+PI/2)*0.035;
		}

		mp_foot1X1=mp_stepsVector[0];
		mp_foot1Y1=mp_stepsVector[1];
		mp_foot1Theta1=mp_stepsVector[2]*PI/180;
		mp_foot2X1=mp_stepsVector[3];
		mp_foot2Y1=mp_stepsVector[4];
		mp_foot2Theta1=mp_stepsVector[5]*PI/180;

		for(unsigned int i=2; i<mp_stepsVector.size()/3;i++)
		{
			//conversion from polar to cartesian:
			double tmpx = mp_stepsVector[3*i+1]*sin(mp_stepsVector[3*i+0]*PI/180);
			double tmpy = mp_stepsVector[3*i+1]*cos(mp_stepsVector[3*i+0]*PI/180);
			mp_stepsVector[3*i+0] = tmpx;
			mp_stepsVector[3*i+1] = tmpy;
		}
	}
	//and also change in case of cartesian input (conversion feet/ankles...)
	else if (mp_coordsConversion == CartesianINPUT)
	{
		mp_stepsVector = vect_input;

		//from feet to ankles:
		if(mp_leftOrRightFirstStable == 'R')
		{
			mp_stepsVector[0]+=-cos(mp_stepsVector[2]*PI/180-PI/2)*0.035;
			mp_stepsVector[1]+=-sin(mp_stepsVector[2]*PI/180-PI/2)*0.035;
			mp_stepsVector[3]+=-cos(mp_stepsVector[5]*PI/180+PI/2)*0.035;
			mp_stepsVector[4]+=-sin(mp_stepsVector[5]*PI/180+PI/2)*0.035;
		}
		else if(mp_leftOrRightFirstStable == 'L')
		{
			mp_stepsVector[3]+=-cos(mp_stepsVector[5]*PI/180-PI/2)*0.035;
			mp_stepsVector[4]+=-sin(mp_stepsVector[5]*PI/180-PI/2)*0.035;
			mp_stepsVector[0]+=-cos(mp_stepsVector[2]*PI/180+PI/2)*0.035;
			mp_stepsVector[1]+=-sin(mp_stepsVector[2]*PI/180+PI/2)*0.035;
		}

		mp_foot1X1=mp_stepsVector[0];
		mp_foot1Y1=mp_stepsVector[1];
		mp_foot1Theta1=mp_stepsVector[2]*PI/180;
		mp_foot2X1=mp_stepsVector[3];
		mp_foot2Y1=mp_stepsVector[4];
		mp_foot2Theta1=mp_stepsVector[5]*PI/180;
	}
}


void CnewPGstepStudy::loadFiles(
string PCParametersFile_,
string VRMLPath_,
string VRMLFileName_,
string SpecificitiesFileName_,
string LinkJointRank_,
string openHRPxml_
)
{
	mp_PCParametersFile=PCParametersFile_;
	mp_VRMLPath=VRMLPath_;
	mp_VRMLFileName=VRMLFileName_;
	mp_SpecificitiesFileName=SpecificitiesFileName_;
	mp_LinkJointRank=LinkJointRank_;
	mp_openHRPxml=openHRPxml_;
}


void CnewPGstepStudy::buildNecessaryInternalStructures()
{

	string Global=mp_PCParametersFile;
	Global+= " ";
	Global+=mp_VRMLPath;
	Global+= " ";
	Global+=mp_VRMLFileName;
	Global+= " ";
	Global+=mp_SpecificitiesFileName;
	Global+= " ";
	Global+=mp_LinkJointRank;
	std::istringstream strm(Global);

	string humanoidName = "HRP2";//of course, specific to HRP2!

	//see MultiBody.h DynamicMultiBody.h HumanoidDynamicMultiBody.h

	string aPath=mp_VRMLPath;
	string aName=mp_VRMLFileName;

	dynamicsJRLJapan::ObjectFactory aRobotDynamicsObjectConstructor;

	mp_HDR = new Chrp2OptHumanoidDynamicRobot(&aRobotDynamicsObjectConstructor);
	//mp_aDMB = mp_HDR;

	string RobotFileName = mp_VRMLPath+mp_VRMLFileName;
	cout << "Map Joint and rank: " << mp_LinkJointRank << endl;
	dynamicsJRLJapan::parseOpenHRPVRMLFile(*mp_HDR,RobotFileName,
		mp_LinkJointRank,
		mp_SpecificitiesFileName);

	cout << " NbOf Dofs : " << mp_HDR->numberDof()<< std::endl;

	// PaireOfBodies: see HumanoidRobotCollisionDetection.h

	vector<PaireOfBodies> SetOfPairs;

	for(unsigned int i=0;i<mp_pairsOfBodiesToCheck.size();i++)
		SetOfPairs.insert(SetOfPairs.end(),mp_pairsOfBodiesToCheck[i]);

	mp_aHRCD = new HumanoidRobotCollisionDetection();

								 //warning hands are not
	mp_aHRCD->LoadStructureForTheHumanoid();
	//loaded, and the arms not completely

	for(unsigned int i=0;i<SetOfPairs.size();i++)
	{
		mp_aHRCD->InsertACollisionPair(SetOfPairs[i]);
	}

}


void CnewPGstepStudy::initAndGenOpenHRPFiles()
{

	/* */
	string Global=mp_PCParametersFile;
	Global+= " ";
	Global+=mp_VRMLPath;
	Global+= " ";
	Global+=mp_VRMLFileName;
	Global+= " ";
	Global+=mp_SpecificitiesFileName;
	Global+= " ";
	Global+=mp_LinkJointRank;
	std::istringstream strm(Global);
	/* */

	MAL_S3x3_MATRIX(Body_R,double);
	MAL_S3_VECTOR(Body_P,double);
	MAL_S3_VECTOR(Dt,double);
	MAL_S3x3_MATRIX(Foot_R,double);
	MAL_S3_VECTOR(Foot_P,double);
	MAL_VECTOR_DIM(q,double,6);
	MAL_VECTOR_DIM(q_mem,double,6);
	//Initialization of these matrices and vectors for the right leg:

	// body rotation
	MAL_S3x3_MATRIX_CLEAR(Body_R);
	MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 0, 0) = 1.0;
	MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 1, 1) = 1.0;
	MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 2, 2) = 1.0;
	// MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 0, 0) = cos(((mp_foot1Theta1)+(mp_foot2Theta1))/2);
	// MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 1, 0) = sin(((mp_foot1Theta1)+(mp_foot2Theta1))/2);
	// MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 0, 1) = -sin(((mp_foot1Theta1)+(mp_foot2Theta1))/2);
	// MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 1, 1) = cos(((mp_foot1Theta1)+(mp_foot2Theta1))/2);
	// MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 2, 2) = 1.0;

	// body translation : vector (0,0,0)
	MAL_S3_VECTOR_FILL(Body_P,0.0);

	// Dt (leg)
	MAL_S3_VECTOR_ACCESS(Dt,0) = 0.0;
	MAL_S3_VECTOR_ACCESS(Dt,1) = -0.06;
	MAL_S3_VECTOR_ACCESS(Dt,2) = 0.0;

	if(mp_leftOrRightFirstStable == 'R')
	{
		// ankle rotation
		MAL_S3x3_MATRIX_CLEAR(Foot_R);
		MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 0, 0) =  cos(mp_foot1Theta1);
		MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 1, 0) =  sin(mp_foot1Theta1);
		MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 0, 1) =  -sin(mp_foot1Theta1);
		MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 1, 1) =  cos(mp_foot1Theta1);
		MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 2, 2) =  1.0;

		// ankle translation
		MAL_S3_VECTOR_ACCESS(Foot_P, 0) = mp_foot1X1;
		MAL_S3_VECTOR_ACCESS(Foot_P, 1) = mp_foot1Y1;
		MAL_S3_VECTOR_ACCESS(Foot_P, 2) = -mp_bodyHeight;
	}
	else if(mp_leftOrRightFirstStable == 'L')
	{
		// ankle rotation
		MAL_S3x3_MATRIX_CLEAR(Foot_R);
		MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 0, 0) =  cos(mp_foot2Theta1);
		MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 1, 0) =  sin(mp_foot2Theta1);
		MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 0, 1) =  -sin(mp_foot2Theta1);
		MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 1, 1) =  cos(mp_foot2Theta1);
		MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 2, 2) =  1.0;

		// ankle translation
		MAL_S3_VECTOR_ACCESS(Foot_P, 0) = mp_foot2X1;
		MAL_S3_VECTOR_ACCESS(Foot_P, 1) = mp_foot2Y1;
		MAL_S3_VECTOR_ACCESS(Foot_P, 2) = -mp_bodyHeight;
	}

	InverseKinematics::ComputeInverseKinematics2ForLegs(Body_R, Body_P,Dt,Foot_R, Foot_P,q);

	if(mp_generate)
	{

		replace_value_scalar(mp_openHRPxml.c_str(),"RLEG_JOINT0.angle",q[0]);
		replace_value_scalar(mp_openHRPxml.c_str(),"RLEG_JOINT1.angle",q[1]);
		replace_value_scalar(mp_openHRPxml.c_str(),"RLEG_JOINT2.angle",q[2]);
		replace_value_scalar(mp_openHRPxml.c_str(),"RLEG_JOINT3.angle",q[3]);
		replace_value_scalar(mp_openHRPxml.c_str(),"RLEG_JOINT4.angle",q[4]);
		replace_value_scalar(mp_openHRPxml.c_str(),"RLEG_JOINT5.angle",q[5]);

	}
	else
	{

		for(int i=0;i<6;i++)
		{
			q_mem[i]=q[i];
		}

	}

	//update of the values for the left leg:

	MAL_S3_VECTOR_ACCESS(Dt,1) = 0.06;

	if(mp_leftOrRightFirstStable == 'R')
	{
		// ankle rotation
		MAL_S3x3_MATRIX_CLEAR(Foot_R);
		MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 0, 0) =  cos(mp_foot2Theta1);
		MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 1, 0) =  sin(mp_foot2Theta1);
		MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 0, 1) =  -sin(mp_foot2Theta1);
		MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 1, 1) =  cos(mp_foot2Theta1);
		MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 2, 2) =  1.0;

		// ankle translation
		MAL_S3_VECTOR_ACCESS(Foot_P, 0) = mp_foot2X1;
		MAL_S3_VECTOR_ACCESS(Foot_P, 1) = mp_foot2Y1;
		MAL_S3_VECTOR_ACCESS(Foot_P, 2) = -mp_bodyHeight;
	}
	else if(mp_leftOrRightFirstStable == 'L')
	{
		// ankle rotation
		MAL_S3x3_MATRIX_CLEAR(Foot_R);
		MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 0, 0) =  cos(mp_foot1Theta1);
		MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 1, 0) =  sin(mp_foot1Theta1);
		MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 0, 1) =  -sin(mp_foot1Theta1);
		MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 1, 1) =  cos(mp_foot1Theta1);
		MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 2, 2) =  1.0;

		// ankle translation
		MAL_S3_VECTOR_ACCESS(Foot_P, 0) = mp_foot1X1;
		MAL_S3_VECTOR_ACCESS(Foot_P, 1) = mp_foot1Y1;
		MAL_S3_VECTOR_ACCESS(Foot_P, 2) = -mp_bodyHeight;
	}

	//

	InverseKinematics::ComputeInverseKinematics2ForLegs(Body_R, Body_P, Dt, Foot_R, Foot_P, q);

	if(mp_generate)
	{

		replace_value_scalar(mp_openHRPxml.c_str(),"LLEG_JOINT0.angle",q[0]);
		replace_value_scalar(mp_openHRPxml.c_str(),"LLEG_JOINT1.angle",q[1]);
		replace_value_scalar(mp_openHRPxml.c_str(),"LLEG_JOINT2.angle",q[2]);
		replace_value_scalar(mp_openHRPxml.c_str(),"LLEG_JOINT3.angle",q[3]);
		replace_value_scalar(mp_openHRPxml.c_str(),"LLEG_JOINT4.angle",q[4]);
		replace_value_scalar(mp_openHRPxml.c_str(),"LLEG_JOINT5.angle",q[5]);

	}

	//update of the waist position:

	if(mp_generate)
	{
		replace_value_vector3(mp_openHRPxml.c_str(),"WAIST.translation", 0.0, 0.0, mp_bodyHeight+0.105);
	}

	MAL_VECTOR_DIM(rotwaist,double,4);
	rotwaist[0]=cos((mp_foot1Theta1+mp_foot2Theta1)/2);
	rotwaist[1]=0;
	rotwaist[2]=sin((mp_foot1Theta1+mp_foot2Theta1)/2);;
	rotwaist[3]=0;

	if(mp_generate)
	{
		replace_value_vectorN(4, mp_openHRPxml.c_str(),"WAIST.rotation",rotwaist);
	}

	//remark: the InverseKinematics takes into account the position of
	//the ankle so the z-limit for HRP2 is 60cm whereas 10.5cm have to be added to
	//get the actual distance between the ground and the waist root (i
	//suppose here that the HRP2's feet stay flat on the ground)
	//other remark: the ankle "zero" y-translation is 6cm.

	//to get the value from the .xml project file and put it in a vector:

	MAL_VECTOR(,double) joints;
	joints.resize(40);
	mp_jointsRadValues.resize(40);

	if(mp_generate)
	{

		mp_jointsRadValues[0]=get_value_scalar(mp_openHRPxml.c_str(),"RLEG_JOINT0.angle");
		mp_jointsRadValues[1]=get_value_scalar(mp_openHRPxml.c_str(),"RLEG_JOINT1.angle");
		mp_jointsRadValues[2]=get_value_scalar(mp_openHRPxml.c_str(),"RLEG_JOINT2.angle");
		mp_jointsRadValues[3]=get_value_scalar(mp_openHRPxml.c_str(),"RLEG_JOINT3.angle");
		mp_jointsRadValues[4]=get_value_scalar(mp_openHRPxml.c_str(),"RLEG_JOINT4.angle");
		mp_jointsRadValues[5]=get_value_scalar(mp_openHRPxml.c_str(),"RLEG_JOINT5.angle");

		mp_jointsRadValues[6]=get_value_scalar(mp_openHRPxml.c_str(),"LLEG_JOINT0.angle");
		mp_jointsRadValues[7]=get_value_scalar(mp_openHRPxml.c_str(),"LLEG_JOINT1.angle");
		mp_jointsRadValues[8]=get_value_scalar(mp_openHRPxml.c_str(),"LLEG_JOINT2.angle");
		mp_jointsRadValues[9]=get_value_scalar(mp_openHRPxml.c_str(),"LLEG_JOINT3.angle");
		mp_jointsRadValues[10]=get_value_scalar(mp_openHRPxml.c_str(),"LLEG_JOINT4.angle");
		mp_jointsRadValues[11]=get_value_scalar(mp_openHRPxml.c_str(),"LLEG_JOINT5.angle");

		mp_jointsRadValues[12]=get_value_scalar(mp_openHRPxml.c_str(),"CHEST_JOINT0.angle");
		mp_jointsRadValues[13]=get_value_scalar(mp_openHRPxml.c_str(),"CHEST_JOINT1.angle");

		mp_jointsRadValues[14]=get_value_scalar(mp_openHRPxml.c_str(),"HEAD_JOINT0.angle");
		mp_jointsRadValues[15]=get_value_scalar(mp_openHRPxml.c_str(),"HEAD_JOINT1.angle");

		mp_jointsRadValues[16]=get_value_scalar(mp_openHRPxml.c_str(),"RARM_JOINT0.angle");
		mp_jointsRadValues[17]=get_value_scalar(mp_openHRPxml.c_str(),"RARM_JOINT1.angle");
		mp_jointsRadValues[18]=get_value_scalar(mp_openHRPxml.c_str(),"RARM_JOINT2.angle");
		mp_jointsRadValues[19]=get_value_scalar(mp_openHRPxml.c_str(),"RARM_JOINT3.angle");
		mp_jointsRadValues[20]=get_value_scalar(mp_openHRPxml.c_str(),"RARM_JOINT4.angle");
		mp_jointsRadValues[21]=get_value_scalar(mp_openHRPxml.c_str(),"RARM_JOINT5.angle");
		mp_jointsRadValues[22]=get_value_scalar(mp_openHRPxml.c_str(),"RARM_JOINT6.angle");

		mp_jointsRadValues[23]=get_value_scalar(mp_openHRPxml.c_str(),"LARM_JOINT0.angle");
		mp_jointsRadValues[24]=get_value_scalar(mp_openHRPxml.c_str(),"LARM_JOINT1.angle");
		mp_jointsRadValues[25]=get_value_scalar(mp_openHRPxml.c_str(),"LARM_JOINT2.angle");
		mp_jointsRadValues[26]=get_value_scalar(mp_openHRPxml.c_str(),"LARM_JOINT3.angle");
		mp_jointsRadValues[27]=get_value_scalar(mp_openHRPxml.c_str(),"LARM_JOINT4.angle");
		mp_jointsRadValues[28]=get_value_scalar(mp_openHRPxml.c_str(),"LARM_JOINT5.angle");
		mp_jointsRadValues[29]=get_value_scalar(mp_openHRPxml.c_str(),"LARM_JOINT6.angle");

		// We have all the joints' values, except for the hands.
		//RHAND_JOINT0
		//RHAND_JOINT1
		//...
		//LHAND_JOINT3
		//LHAND_JOINT4
		// Hands' joints at zero:

		mp_jointsRadValues[30]=0.0;
		mp_jointsRadValues[31]=0.0;
		mp_jointsRadValues[32]=0.0;
		mp_jointsRadValues[33]=0.0;
		mp_jointsRadValues[34]=0.0;
		mp_jointsRadValues[35]=0.0;
		mp_jointsRadValues[36]=0.0;
		mp_jointsRadValues[37]=0.0;
		mp_jointsRadValues[38]=0.0;
		mp_jointsRadValues[39]=0.0;

	}
	else
	{

		mp_jointsRadValues[0]= q_mem[0];
		mp_jointsRadValues[1]= q_mem[1];
		mp_jointsRadValues[2]= q_mem[2];
		mp_jointsRadValues[3]= q_mem[3];
		mp_jointsRadValues[4]= q_mem[4];
		mp_jointsRadValues[5]= q_mem[5];

		mp_jointsRadValues[6]= q[0];
		mp_jointsRadValues[7]= q[1];
		mp_jointsRadValues[8]= q[2];
		mp_jointsRadValues[9]= q[3];
		mp_jointsRadValues[10]= q[4];
		mp_jointsRadValues[11]= q[5];

		mp_jointsRadValues[12]=0.0;
		mp_jointsRadValues[13]=0.0;

		mp_jointsRadValues[14]=0.0;
		mp_jointsRadValues[15]=0.0;

		mp_jointsRadValues[16]=0.261809;
		mp_jointsRadValues[17]=-0.174533;
		mp_jointsRadValues[18]=0.000000;
		mp_jointsRadValues[19]=-0.523599;
		mp_jointsRadValues[20]=0.000000;
		mp_jointsRadValues[21]=0.000000;
		mp_jointsRadValues[22]=0.174533;

		mp_jointsRadValues[23]=0.261809;;
		mp_jointsRadValues[24]=0.174533;
		mp_jointsRadValues[25]=0.000000;
		mp_jointsRadValues[26]=-0.523599;
		mp_jointsRadValues[27]=0.000000;
		mp_jointsRadValues[28]=0.000000;
		mp_jointsRadValues[29]=0.174533;

		// We have all the joints' values, except for the hands.
		//RHAND_JOINT0
		//RHAND_JOINT1
		//...
		//LHAND_JOINT3
		//LHAND_JOINT4
		// Hands' joints at zero:

		mp_jointsRadValues[30]=0.0;
		mp_jointsRadValues[31]=0.0;
		mp_jointsRadValues[32]=0.0;
		mp_jointsRadValues[33]=0.0;
		mp_jointsRadValues[34]=0.0;
		mp_jointsRadValues[35]=0.0;
		mp_jointsRadValues[36]=0.0;
		mp_jointsRadValues[37]=0.0;
		mp_jointsRadValues[38]=0.0;
		mp_jointsRadValues[39]=0.0;

	}

	for(int k=0;k<40;k++)
	{
		joints[k]=mp_jointsRadValues[k]*180.0/PI;
	}

	//string search =	"MARKPOINT1\n  seq.sendMsg(\":joint-angles";

	//remark: 46=6 (free flyer) + 40 (joints)
	mp_currentConfiguration.resize(46);
	mp_currentVelocity.resize(46);
	mp_currentAcceleration.resize(46);
	mp_previousConfiguration.resize(46) ;
	mp_previousVelocity.resize(46);
	mp_previousAcceleration.resize(46);
	mp_ZMPTarget.resize(3);

	for(int i=0; i<46; i++)
	{
		mp_currentConfiguration[i]=0;
		mp_currentVelocity[i]=0;
		mp_currentAcceleration[i]=0;
		mp_previousConfiguration[i]=0;
		mp_previousVelocity[i]=0;
		mp_previousAcceleration[i]=0;
	}

	mp_ZMPTarget[0]=0;
	mp_ZMPTarget[1]=0;
	mp_ZMPTarget[2]=0;

	MAL_VECTOR(,double) freefly;
	freefly.resize(6);
	for(int i=0;i<6;i++)
	{
		freefly[i]=0;
	}

	freefly[3]=(mp_foot1Theta1+mp_foot2Theta1)/2;

	MAL_VECTOR_DIM(totall,double,46);
	for(int i=0;i<46;i++)
	{
		totall[i]=0;
	}
	concatvectors(freefly,mp_jointsRadValues,totall);

	mp_currentConfiguration=totall;
	mp_HDR->currentConfiguration(totall);
	mp_HDR->computeForwardKinematics();

	MAL_S3_VECTOR(valCOM0,double);
	valCOM0 = mp_HDR->positionCenterOfMass();

	CjrlJoint *aJoint0;
	matrix4d aCurrentM0;

	if(mp_leftOrRightFirstStable == 'R')
		aJoint0 = mp_HDR->rightAnkle();
	if(mp_leftOrRightFirstStable == 'L')
		aJoint0 = mp_HDR->leftAnkle();

	aCurrentM0 = aJoint0->currentTransformation();

	double foot_stay_x = MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM0,0,3)-valCOM0[0];
	double foot_stay_y = MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM0,1,3)-valCOM0[1];
	double foot_stay_theta = 0;
	if(mp_leftOrRightFirstStable == 'R') foot_stay_theta = mp_foot1Theta1;
	if(mp_leftOrRightFirstStable == 'L') foot_stay_theta = mp_foot2Theta1;

	mp_sseq.resize(mp_stepsVector.size()-3);
	mp_sseq[0] = foot_stay_x;
	mp_sseq[1] = foot_stay_y;
	mp_sseq[2] = foot_stay_theta*180/PI;

	for(unsigned int i=3; i<mp_stepsVector.size()-3; i++)
	{
		mp_sseq[i] = mp_stepsVector[i+3];
	}

	//The sequence of steps to do is defined by mp_sseq

	MAL_VECTOR_DIM(qArmr, double, 7);
	MAL_VECTOR_DIM(qArml, double, 7);
	MAL_VECTOR_DIM(aCOMPosition, double,6);
	MAL_VECTOR_DIM(RFP,double,3);
	MAL_VECTOR_DIM(LFP,double,3);

	for(int j=0;j<1;j++)
	{

		//==================Initialization of the upper body posture
		//===A few iterations (or even just 1) to converge

		valCOM0 = mp_HDR->positionCenterOfMass();
		for(int i=0;i<3;i++) aCOMPosition[i]=valCOM0[i];

		aJoint0 = mp_HDR->rightAnkle();
		aCurrentM0 = aJoint0->currentTransformation();
		for(int i=0;i<3;i++) RFP[i]=MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM0,i,3);

		aJoint0 = mp_HDR->leftAnkle();
		aCurrentM0 = aJoint0->currentTransformation();
		for(int i=0;i<3;i++) LFP[i]=MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM0,i,3);

		InverseKinematics::ComputeUpperBodyHeuristicForNormalWalking(qArmr, qArml, aCOMPosition, RFP, LFP);

		mp_jointsRadValues[16]=qArmr[0];
		mp_jointsRadValues[17]=qArmr[1];
		mp_jointsRadValues[18]=qArmr[2];
		mp_jointsRadValues[19]=qArmr[3];
		mp_jointsRadValues[20]=qArmr[4];
		mp_jointsRadValues[21]=qArmr[5];
		mp_jointsRadValues[22]=qArmr[6];

		mp_jointsRadValues[23]=qArml[0];
		mp_jointsRadValues[24]=qArml[1];
		mp_jointsRadValues[25]=qArml[2];
		mp_jointsRadValues[26]=qArml[3];
		mp_jointsRadValues[27]=qArml[4];
		mp_jointsRadValues[28]=qArml[5];
		mp_jointsRadValues[29]=qArml[6];

		for(int k=16;k<30;k++)
		{
			joints[k]=mp_jointsRadValues[k]*180.0/PI;
		}

		//_________________________

		//re-initialization
		concatvectors(freefly,mp_jointsRadValues,totall);

		mp_currentConfiguration=totall;
		mp_HDR->currentConfiguration(totall);
		mp_HDR->computeForwardKinematics();

		//________________________

	}

	mp_NbOfIt=0;

	if(mp_generate)
	{

		replace_value_scalar(mp_openHRPxml.c_str(),"RARM_JOINT0.angle",qArmr[0]);
		replace_value_scalar(mp_openHRPxml.c_str(),"RARM_JOINT1.angle",qArmr[1]);
		replace_value_scalar(mp_openHRPxml.c_str(),"RARM_JOINT2.angle",qArmr[2]);
		replace_value_scalar(mp_openHRPxml.c_str(),"RARM_JOINT3.angle",qArmr[3]);
		replace_value_scalar(mp_openHRPxml.c_str(),"RARM_JOINT4.angle",qArmr[4]);
		replace_value_scalar(mp_openHRPxml.c_str(),"RARM_JOINT5.angle",qArmr[5]);
		replace_value_scalar(mp_openHRPxml.c_str(),"RARM_JOINT6.angle",qArmr[6]);

		replace_value_scalar(mp_openHRPxml.c_str(),"LARM_JOINT0.angle",qArml[0]);
		replace_value_scalar(mp_openHRPxml.c_str(),"LARM_JOINT1.angle",qArml[1]);
		replace_value_scalar(mp_openHRPxml.c_str(),"LARM_JOINT2.angle",qArml[2]);
		replace_value_scalar(mp_openHRPxml.c_str(),"LARM_JOINT3.angle",qArml[3]);
		replace_value_scalar(mp_openHRPxml.c_str(),"LARM_JOINT4.angle",qArml[4]);
		replace_value_scalar(mp_openHRPxml.c_str(),"LARM_JOINT5.angle",qArml[5]);
		replace_value_scalar(mp_openHRPxml.c_str(),"LARM_JOINT6.angle",qArml[6]);

	}

}


void CnewPGstepStudy::runPGAndEvalTrajectory()
{

	//===================================================
	//Is the initial position in self-collision ?

	vector3d aP1;
	matrix3d aR1;

	Vect3 aP2;
	Mat3 aR2;

	VclipPose aX;

	// Take the posture from the multibody configuration.
	vector<CjrlJoint *> aVecOfJoints;
	CjrlJoint *aJoint;
	aVecOfJoints = mp_HDR->jointVector();
	for(int i=0;i<28;i++)
	{
		matrix4d aCurrentM;
		aJoint = aVecOfJoints[i];//mp_aDMB->GetJointFromVRMLID(i);

		aCurrentM = aJoint->currentTransformation();

		aP2[0] = MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,0,3);
		aP2[1] = MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,1,3);
		aP2[2] = MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,2,3);

		aR2[0][0] = MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,0,0);
		aR2[0][1] = MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,0,1);
		aR2[0][2] = MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,0,2);
		aR2[1][0] = MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,1,0);
		aR2[1][1] = MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,1,1);
		aR2[1][2] = MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,1,2);
		aR2[2][0] = MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,2,0);
		aR2[2][1] = MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,2,1);
		aR2[2][2] = MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,2,2);

		aX.set(aR2,aP2);
		mp_aHRCD->SetBodyPose(i+1,aX);

	}

	double r;

	// Update the body posture.
	aX = VclipPose::ID;
	mp_aHRCD->SetBodyPose(0,aX);

	// Check the possible collision.
	r = mp_aHRCD->ComputeSelfCollision();

	double rinit;
	rinit = r;

	//==============================================
	// ''''''''''''''''''
	// initialization of the joints limits vectors
	double minJointsLimits[30] =
	{
								 // right leg
		-45.0, -35.0, -125.0, -2.0, -75.0, -20.0,
								 // left leg
		-30.0, -20.0, -125.0, -2.0, -75.0, -35.0,

		-45.0, -5.0,			 // chest
		-45.0 -30.0,			 // head

								 // right arm
		-180.0, -95.0, -92.0, -137.0, -92.0, -92.0, -60.0,
								 // left arm
		-180.0, -10.0, -92.0, -137.0, -92.0, -92.0, -60.0
	};

	double maxJointsLimits[30] =
	{
								 // right leg
		30.0, 20.0, 42.0, 150.0, 42.0, 35.0,
								 // left leg
		45.0, 35.0, 42.0, 150.0, 42.0, 20.0,

		45.0, 60.0,				 // chest
		45.0, 45.0,				 // head

								 // right arm
		60.0, 10.0, 92.0, 2.0, 92.0, 92.0, 16.0,
								 // left arm
		60.0, 95.0, 92.0, 2.0, 92.0, 92.0, 16.0
	};

	// ''''''''''''''''

	int counter=0;

	double distLimits=10000.0;

	filebuf fpbuf;
	fpbuf.open ("oneStep.pos",ios::out);
	ostream fp(&fpbuf);

	filebuf fp2buf;
	fp2buf.open ("oneStep.zmp",ios::out);
	ostream fp2(&fp2buf);

	while(0) {};

	//------------------------------------------------------

	//double InitDistCol, DistMinJointsLim, DistMinSelfCol;
	//int PGit;

}


void CnewPGstepStudy::drawSteps(ofstream & fb)
{
	mp_stepsInput;
	mp_leftOrRightFirstStable;
	mp_coordsConversion;

	double centre_x;
	double centre_y;
	double abs_orientation;
	char which_foot;

	vector<double> c_x;
	vector<double> c_y;

	for(int i=0; i < (int) (mp_stepsInput.size())/3 ; i++)
	{

		if(i==0)
		{
			centre_x = mp_stepsInput[0];
			centre_y = mp_stepsInput[1];
			abs_orientation = mp_stepsInput[2];
			which_foot = mp_leftOrRightFirstStable;
			c_x.push_back(centre_x);
			c_y.push_back(centre_y);
		}
		else if(i==1)
		{
			centre_x = mp_stepsInput[3];
			centre_y = mp_stepsInput[4];
			abs_orientation = mp_stepsInput[5];
			if(mp_leftOrRightFirstStable == 'L') which_foot = 'R';
			else which_foot = 'L';
			c_x.push_back(centre_x);
			c_y.push_back(centre_y);
		}
		else if(i==2)
		{
			centre_x = mp_stepsInput[0] + mp_stepsInput[6];
			centre_y = mp_stepsInput[1] + mp_stepsInput[7];
			abs_orientation = mp_stepsInput[2] + mp_stepsInput[8];
			which_foot = mp_leftOrRightFirstStable;
			c_x.push_back(centre_x);
			c_y.push_back(centre_y);
		}
		else
		{
			centre_x = centre_x + mp_stepsInput[3*i];
			centre_y = centre_y + mp_stepsInput[3*i+1];
			abs_orientation = abs_orientation + mp_stepsInput[3*i+2];
			if(which_foot == 'L') which_foot = 'R';
			else which_foot = 'L';
			c_x.push_back(centre_x);
			c_y.push_back(centre_y);
		}

		abs_orientation = abs_orientation * PI/180;

		double norm = sqrt(0.065*0.065+0.115*0.115);

		vector<double> cosinuss (4, 0);
		vector<double> sinuss (4, 0);
		vector<double> x (4, 0);
		vector<double> y (4, 0);

		cosinuss[0] = cos(abs_orientation)*0.115/norm - sin(abs_orientation)*0.065/norm;
		sinuss[0] = cos(abs_orientation)*0.065/norm + sin(abs_orientation)*0.115/norm;

		cosinuss[1] = cos(abs_orientation)*(-0.115)/norm - sin(abs_orientation)*0.065/norm;
		sinuss[1] = cos(abs_orientation)*0.065/norm + sin(abs_orientation)*(-0.115)/norm;

		cosinuss[2] = cos(abs_orientation)*(-0.115)/norm - sin(abs_orientation)*(-0.065)/norm;
		sinuss[2] = cos(abs_orientation)*(-0.065)/norm + sin(abs_orientation)*(-0.115)/norm;

		cosinuss[3] = cos(abs_orientation)*0.115/norm - sin(abs_orientation)*(-0.065)/norm;
		sinuss[3] = cos(abs_orientation)*(-0.065)/norm + sin(abs_orientation)*0.115/norm;

		for(int j = 0; j<4; j++)
		{
			x[j] = centre_x + cosinuss[j]*norm;
			y[j] = centre_y + sinuss[j]*norm;
		}

		for(int j = 0; j<4; j++)
		{
			fb << x[j]
				<< " " << y[j]
				<< " " << x[(j+1) % 4] - x[j]
				<< " " << y[(j+1) % 4] - y[j]
				<< endl;
		}
		fb << x[0]
			<< " " << y[0]
			<< " " << x[0]
			<< " " << y[0]
			<< endl << endl;

	}

	double multiplier = 2;
	fb << -multiplier
		<< " " << -multiplier
		<< " " << multiplier
		<< " " << -multiplier
		<< endl;
	fb << multiplier
		<< " " << -multiplier
		<< " " << multiplier
		<< " " << multiplier
		<< endl;
	fb << multiplier
		<< " " << multiplier
		<< " " << -multiplier
		<< " " << multiplier
		<< endl;
	fb << -multiplier
		<< " " << multiplier
		<< " " << -multiplier
		<< " " << -multiplier
		<< endl;
	fb << -multiplier
		<< " " << -multiplier
		<< " " << -multiplier
		<< " " << -multiplier
		<< endl << endl;

	fb << endl;

	char trig = mp_leftOrRightFirstStable;
	for(int i=0; i < (int) (mp_stepsInput.size())/3 ; i++)
	{
		if(i==0)
		{
			fb << c_x[i] << " " << c_y[i] << " " << trig << "#" << endl;
		}
		else if(i==1)
		{
			if(trig == 'R') trig = 'L';
			else trig = 'R';
			fb << c_x[i] << " " << c_y[i] << " " << trig << "~" << endl;
		}
		else
		{
			fb << c_x[i] << " " << c_y[i] << " " << trig << i-1 << endl;
			if(trig == 'R') trig = 'L';
			else trig = 'R';
		}
	}

	fb << endl;
	fb << endl;
}


double w (double t, double g, double zc, double delta0, double deltaX, double t1, double t2, double V, double W)
{

	return(delta0+(V*cosh(sqrt(g/zc)*t1)+W*sinh(sqrt(g/zc)*t1)-6.0*deltaX*zc/pow(t2-t1,2.0)/g)*cosh(sqrt(g/zc)*(t-t1))+(V*sinh(sqrt(g/zc)*t1)*sqrt(g/zc
		)+W*cosh(sqrt(g/zc)*t1)*sqrt(g/zc)+12.0*deltaX*zc/pow(t2-t1,3.0)/g)*sinh(
		sqrt(g/zc)*(t-t1))/sqrt(g/zc)-2.0*deltaX*pow(t-t1,3.0)/pow(t2-t1,3.0)+3.0*
		deltaX*pow(t-t1,2.0)/pow(t2-t1,2.0)-12.0*deltaX*zc*(t-t1)/pow(t2-t1,3.0
		)/g+6.0*deltaX*zc/pow(t2-t1,2.0)/g);

};

double w2 (double t, double g, double zc, double deltaX2, double t2, double t3, double t4, double K2, double V2, double W2)
{

	return(K2+(V2*cosh(sqrt(g/zc)*(t3-t2))+W2*sinh(sqrt(g/zc)*(t3-t2))-6.0*
		deltaX2*zc/pow(t4-t3,2.0)/g)*cosh(sqrt(g/zc)*(t-t3))+(V2*sinh(sqrt(g/zc)*(
		t3-t2))*sqrt(g/zc)+W2*cosh(sqrt(g/zc)*(t3-t2))*sqrt(g/zc)+12.0*deltaX2*zc/
		pow(t4-t3,3.0)/g)*sinh(sqrt(g/zc)*(t-t3))/sqrt(g/zc)-2.0*deltaX2*pow(t-t3,
		3.0)/pow(t4-t3,3.0)+3.0*deltaX2*pow(t-t3,2.0)/pow(t4-t3,2.0)-12.0*deltaX2
		*zc*(t-t3)/pow(t4-t3,3.0)/g+6.0*deltaX2*zc/pow(t4-t3,2.0)/g);

};

double u (double t, double g, double zc, double t2, double K2, double V2, double W2)
{

	return(V2*cosh(sqrt(g/zc)*(t-t2))+W2*sinh(sqrt(g/zc)*(t-t2))+K2);

};

double u2 (double t, double g, double zc, double t4, double K3, double V3, double W3)
{

	return(V3*cosh(sqrt(g/zc)*(t-t4))+W3*sinh(sqrt(g/zc)*(t-t4))+K3);

};

double hZMP (double t, double g, double zc, double delta0, double deltaX, double deltaX2, double t1, double t2, double t3, double t4, double V, double W, double K2, double V2, double W2, double K3, double V3, double W3)
{

	if(t <= t1)
	{
		return delta0;
	}
	else if(t <= t2)
	{
		return (delta0*t1*t1*t1-delta0*t2*t2*t2+2.0*deltaX*t*t*t+deltaX*t1*t1*t1-3.0*deltaX*t*t*t1-3.0*deltaX*t*t*t2+6.0*deltaX*t*t1*t2
			-3.0*delta0*t1*t1*t2+3.0*delta0*t1*t2*t2-3.0*deltaX*t1*t1*t2)/pow(t1-
			t2,3.0);
	}
	else if(t <= t3)
	{
		return K2;
	}
	else if(t <= t4)
	{
		return (-K2*t4*t4*t4+K2*t3*t3*t3+2.0*deltaX2*t*t*t+deltaX2*t3*t3*t3
			-3.0*deltaX2*t*t*t3-3.0*deltaX2*t*t*t4+6.0*deltaX2*t*t3*t4-3.0*
			deltaX2*t4*t3*t3+3.0*K2*t4*t4*t3-3.0*K2*t4*t3*t3)/pow(-t4+t3,3.0);
	}
	else
	{
		return K3;
	}

};

double h (double t, double g, double zc, double delta0, double deltaX, double deltaX2, double t1, double t2, double t3, double t4, double V, double W, double K2, double V2, double W2, double K3, double V3, double W3)
{

	if(t <= t1)
	{
		return V*cosh(sqrt(g/zc)*t)+W*sinh(sqrt(g/zc)*t)+delta0;
	}
	else if(t <= t2)
	{
		return w(t, g, zc, delta0, deltaX, t1, t2, V, W);
	}
	else if(t <= t3)
	{
		return u(t, g, zc, t2, K2, V2, W2);
	}
	else if(t <= t4)
	{
		return w2(t, g, zc, deltaX2, t2, t3, t4, K2, V2, W2);
	}
	else
	{
		return u2(t, g, zc, t4, K3, V3, W3);
	}

};

vector<double> hVinit (double t, double g, double zc, double delta0, double deltaX, double deltaX2, double t1, double t2, double t3, double t4, double t5, double pinit, double vinit)
{

	vector<double> PairToReturn;
	double V = pinit-delta0;
	double W = vinit/sqrt(g/zc);
	double K2 = delta0+(V*cosh(sqrt(g/zc)*t1)+W*sinh(sqrt(g/zc)*t1)-6.0
		*deltaX*zc/pow(t2-t1,2.0)/g)*cosh(sqrt(g/zc)*(t2-t1))+(V*sinh(sqrt(g/zc)*t1
		)*sqrt(g/zc)+W*cosh(sqrt(g/zc)*t1)*sqrt(g/zc)+12.0*deltaX*zc/pow(t2-t1,3.0)
		/g)*sinh(sqrt(g/zc)*(t2-t1))/sqrt(g/zc)+deltaX-6.0*deltaX*zc/pow(t2-t1,2.0)/g-zc/g*((V*
		cosh(sqrt(g/zc)*t1)+W*sinh(sqrt(g/zc)*t1)-6.0*deltaX*zc/pow(t2-t1,2.0)/g)*
		cosh(sqrt(g/zc)*(t2-t1))*g/zc+(V*sinh(sqrt(g/zc)*t1)*sqrt(g/zc)+W*cosh(sqrt(g/
		zc)*t1)*sqrt(g/zc)+12.0*deltaX*zc/pow(t2-t1,3.0)/g)*sinh(sqrt(g/zc)*(t2-t1)
		)*sqrt(g/zc)-6.0*deltaX/pow(t2-t1,2.0));
	double V2 = zc/g*((V*cosh(sqrt(g/zc)*t1)+W*sinh(sqrt(g/zc)*t1)-6.0*deltaX*zc
		/pow(t2-t1,2.0)/g)*cosh(sqrt(g/zc)*(t2-t1))*g/zc+(V*sinh(sqrt(g/zc)*t1)*sqrt(g/
		zc)+W*cosh(sqrt(g/zc)*t1)*sqrt(g/zc)+12.0*deltaX*zc/pow(t2-t1,3.0)/g)*sinh(
		sqrt(g/zc)*(t2-t1))*sqrt(g/zc)-6.0*deltaX/pow(t2-t1,2.0));
	double W2 = ((V*cosh(sqrt(g/zc)*t1)+W*sinh(sqrt(g/zc)*t1)-6.0*deltaX*zc/pow(
		t2-t1,2.0)/g)*sinh(sqrt(g/zc)*(t2-t1))*sqrt(g/zc)+(V*sinh(sqrt(g/zc)*t1)*sqrt(g
		/zc)+W*cosh(sqrt(g/zc)*t1)*sqrt(g/zc)+12.0*deltaX*zc/pow(t2-t1,3.0)/g)*cosh
		(sqrt(g/zc)*(t2-t1))-12.0*deltaX*zc/pow(t2-t1,3.0)/g)/sqrt(g/zc);
	double K3 = K2+(V2*cosh(sqrt(g/zc)*(t3-t2))+W2*sinh(sqrt(g/zc)*(t3-t2)
		)-6.0*deltaX2*zc/pow(t4-t3,2.0)/g)*cosh(sqrt(g/zc)*(t4-t3))+(V2*sinh(sqrt(g
		/zc)*(t3-t2))*sqrt(g/zc)+W2*cosh(sqrt(g/zc)*(t3-t2))*sqrt(g/zc)+12.0*deltaX2
		*zc/pow(t4-t3,3.0)/g)*sinh(sqrt(g/zc)*(t4-t3))/sqrt(g/zc)+deltaX2-6.0*deltaX2*zc/pow(t4-t3,2.0)/g-zc/g*((
		V2*cosh(sqrt(g/zc)*(t3-t2))+W2*sinh(sqrt(g/zc)*(t3-t2))-6.0*deltaX2*zc/pow(
		t4-t3,2.0)/g)*cosh(sqrt(g/zc)*(t4-t3))*g/zc+(V2*sinh(sqrt(g/zc)*(t3-t2))*sqrt(g
		/zc)+W2*cosh(sqrt(g/zc)*(t3-t2))*sqrt(g/zc)+12.0*deltaX2*zc/pow(t4-t3,3.0)/
		g)*sinh(sqrt(g/zc)*(t4-t3))*sqrt(g/zc)-6.0*deltaX2/pow(t4-t3,2.0));
	double V3 = zc/g*((V2*cosh(sqrt(g/zc)*(t3-t2))+W2*sinh(sqrt(g/zc)*(t3-t2))-6.0*
		deltaX2*zc/pow(t4-t3,2.0)/g)*cosh(sqrt(g/zc)*(t4-t3))*g/zc+(V2*sinh(sqrt(g/
		zc)*(t3-t2))*sqrt(g/zc)+W2*cosh(sqrt(g/zc)*(t3-t2))*sqrt(g/zc)+12.0*deltaX2
		*zc/pow(t4-t3,3.0)/g)*sinh(sqrt(g/zc)*(t4-t3))*sqrt(g/zc)-6.0*deltaX2/pow(
		t4-t3,2.0));
	double W3 = ((V2*cosh(sqrt(g/zc)*(t3-t2))+W2*sinh(sqrt(g/zc)*(t3-t2))-6.0*deltaX2*
		zc/pow(t4-t3,2.0)/g)*sinh(sqrt(g/zc)*(t4-t3))*sqrt(g/zc)+(V2*sinh(sqrt(g/
		zc)*(t3-t2))*sqrt(g/zc)+W2*cosh(sqrt(g/zc)*(t3-t2))*sqrt(g/zc)+12.0*deltaX2
		*zc/pow(t4-t3,3.0)/g)*cosh(sqrt(g/zc)*(t4-t3))-12.0*deltaX2*zc/pow(t4-t3,
		3.0)/g)/sqrt(g/zc);
	PairToReturn.push_back(h(t, g, zc, delta0, deltaX, deltaX2, t1, t2, t3, t4, V, W, K2, V2, W2, K3, V3, W3));
	PairToReturn.push_back(hZMP(t, g, zc, delta0, deltaX, deltaX2, t1, t2, t3, t4, V, W, K2, V2, W2, K3, V3, W3));
	return PairToReturn;
};

double hVinitCOMonly (double t, double g, double zc, double delta0, double deltaX, double deltaX2, double t1, double t2, double t3, double t4, double t5, double pinit, double vinit)
{

	double V = pinit-delta0;
	double W = vinit/sqrt(g/zc);
	double K2 = delta0+(V*cosh(sqrt(g/zc)*t1)+W*sinh(sqrt(g/zc)*t1)-6.0
		*deltaX*zc/pow(t2-t1,2.0)/g)*cosh(sqrt(g/zc)*(t2-t1))+(V*sinh(sqrt(g/zc)*t1
		)*sqrt(g/zc)+W*cosh(sqrt(g/zc)*t1)*sqrt(g/zc)+12.0*deltaX*zc/pow(t2-t1,3.0)
		/g)*sinh(sqrt(g/zc)*(t2-t1))/sqrt(g/zc)+deltaX-6.0*deltaX*zc/pow(t2-t1,2.0)/g-zc/g*((V*
		cosh(sqrt(g/zc)*t1)+W*sinh(sqrt(g/zc)*t1)-6.0*deltaX*zc/pow(t2-t1,2.0)/g)*
		cosh(sqrt(g/zc)*(t2-t1))*g/zc+(V*sinh(sqrt(g/zc)*t1)*sqrt(g/zc)+W*cosh(sqrt(g/
		zc)*t1)*sqrt(g/zc)+12.0*deltaX*zc/pow(t2-t1,3.0)/g)*sinh(sqrt(g/zc)*(t2-t1)
		)*sqrt(g/zc)-6.0*deltaX/pow(t2-t1,2.0));
	double V2 = zc/g*((V*cosh(sqrt(g/zc)*t1)+W*sinh(sqrt(g/zc)*t1)-6.0*deltaX*zc
		/pow(t2-t1,2.0)/g)*cosh(sqrt(g/zc)*(t2-t1))*g/zc+(V*sinh(sqrt(g/zc)*t1)*sqrt(g/
		zc)+W*cosh(sqrt(g/zc)*t1)*sqrt(g/zc)+12.0*deltaX*zc/pow(t2-t1,3.0)/g)*sinh(
		sqrt(g/zc)*(t2-t1))*sqrt(g/zc)-6.0*deltaX/pow(t2-t1,2.0));
	double W2 = ((V*cosh(sqrt(g/zc)*t1)+W*sinh(sqrt(g/zc)*t1)-6.0*deltaX*zc/pow(
		t2-t1,2.0)/g)*sinh(sqrt(g/zc)*(t2-t1))*sqrt(g/zc)+(V*sinh(sqrt(g/zc)*t1)*sqrt(g
		/zc)+W*cosh(sqrt(g/zc)*t1)*sqrt(g/zc)+12.0*deltaX*zc/pow(t2-t1,3.0)/g)*cosh
		(sqrt(g/zc)*(t2-t1))-12.0*deltaX*zc/pow(t2-t1,3.0)/g)/sqrt(g/zc);
	double K3 = K2+(V2*cosh(sqrt(g/zc)*(t3-t2))+W2*sinh(sqrt(g/zc)*(t3-t2)
		)-6.0*deltaX2*zc/pow(t4-t3,2.0)/g)*cosh(sqrt(g/zc)*(t4-t3))+(V2*sinh(sqrt(g
		/zc)*(t3-t2))*sqrt(g/zc)+W2*cosh(sqrt(g/zc)*(t3-t2))*sqrt(g/zc)+12.0*deltaX2
		*zc/pow(t4-t3,3.0)/g)*sinh(sqrt(g/zc)*(t4-t3))/sqrt(g/zc)+deltaX2-6.0*deltaX2*zc/pow(t4-t3,2.0)/g-zc/g*((
		V2*cosh(sqrt(g/zc)*(t3-t2))+W2*sinh(sqrt(g/zc)*(t3-t2))-6.0*deltaX2*zc/pow(
		t4-t3,2.0)/g)*cosh(sqrt(g/zc)*(t4-t3))*g/zc+(V2*sinh(sqrt(g/zc)*(t3-t2))*sqrt(g
		/zc)+W2*cosh(sqrt(g/zc)*(t3-t2))*sqrt(g/zc)+12.0*deltaX2*zc/pow(t4-t3,3.0)/
		g)*sinh(sqrt(g/zc)*(t4-t3))*sqrt(g/zc)-6.0*deltaX2/pow(t4-t3,2.0));
	double V3 = zc/g*((V2*cosh(sqrt(g/zc)*(t3-t2))+W2*sinh(sqrt(g/zc)*(t3-t2))-6.0*
		deltaX2*zc/pow(t4-t3,2.0)/g)*cosh(sqrt(g/zc)*(t4-t3))*g/zc+(V2*sinh(sqrt(g/
		zc)*(t3-t2))*sqrt(g/zc)+W2*cosh(sqrt(g/zc)*(t3-t2))*sqrt(g/zc)+12.0*deltaX2
		*zc/pow(t4-t3,3.0)/g)*sinh(sqrt(g/zc)*(t4-t3))*sqrt(g/zc)-6.0*deltaX2/pow(
		t4-t3,2.0));
	double W3 = ((V2*cosh(sqrt(g/zc)*(t3-t2))+W2*sinh(sqrt(g/zc)*(t3-t2))-6.0*deltaX2*
		zc/pow(t4-t3,2.0)/g)*sinh(sqrt(g/zc)*(t4-t3))*sqrt(g/zc)+(V2*sinh(sqrt(g/
		zc)*(t3-t2))*sqrt(g/zc)+W2*cosh(sqrt(g/zc)*(t3-t2))*sqrt(g/zc)+12.0*deltaX2
		*zc/pow(t4-t3,3.0)/g)*cosh(sqrt(g/zc)*(t4-t3))-12.0*deltaX2*zc/pow(t4-t3,
		3.0)/g)/sqrt(g/zc);
	return h(t, g, zc, delta0, deltaX, deltaX2, t1, t2, t3, t4, V, W, K2, V2, W2, K3, V3, W3);
};

double searchVinit (double g, double zc, double delta0, double deltaX, double deltaX2, double t1, double t2, double t3, double t4, double t5, double pinit)
{

	double vinitBmin = -10.0;

	double vinitBmax = 10.0;

	if (
		hVinitCOMonly(t5, g, zc, delta0, deltaX, deltaX2, t1, t2, t3, t4, t5, pinit, vinitBmin) >= delta0 + deltaX + deltaX2
		||
		hVinitCOMonly(t5, g, zc, delta0, deltaX, deltaX2, t1, t2, t3, t4, t5, pinit, vinitBmax) <= delta0 + deltaX + deltaX2
		) return -999;

	while (vinitBmax - vinitBmin > 0.00000001)
	{

		if (hVinitCOMonly(t5, g, zc, delta0, deltaX, deltaX2, t1, t2, t3, t4, t5, pinit, (vinitBmax + vinitBmin) / 2) > delta0 + deltaX + deltaX2)
		{
			vinitBmax = (vinitBmax + vinitBmin) / 2;
		}
		else
		{
			vinitBmin = (vinitBmax + vinitBmin) / 2;
		}

	}

	return (vinitBmax + vinitBmin) / 2;

}


void CnewPGstepStudy::genCOMtrajectoryOFSTREAM(ofstream & fb, double incrTime, double zc, double g, double delta0, double deltaX, double deltaX2, double t1, double t2, double t3, double t4, double t5)
{

	double vinit = searchVinit(g, zc, delta0, deltaX, deltaX2, t1, t2, t3, t4, t5, 0);

	for(double i = 0.0 ; i < t5 ; i += incrTime)
	{

		fb << i << " " << hVinitCOMonly(i, g, zc, delta0, deltaX, deltaX2, t1, t2, t3, t4, t5, 0, vinit) << endl;
	}

}


void CnewPGstepStudy::genCOMZMPtrajectory(vector<double> & outputCOM, vector<double> & outputZMP, double incrTime, double zc, double g, double delta0, double deltaX, double deltaX2, double t1, double t2, double t3, double t4, double t5)
{

	outputCOM.clear();
	outputZMP.clear();

	double vinit = searchVinit(g, zc, delta0, deltaX, deltaX2, t1, t2, t3, t4, t5, 0);

	for(double i = 0.0 ; i < t5 ; i += incrTime)
	{
		//fb << i << " " << hVinit(i, g, zc, delta0, deltaX, deltaX2, t1, t2, t3, t4, t5, vinit) << endl;
		vector<double> ComZmp = hVinit(i, g, zc, delta0, deltaX, deltaX2, t1, t2, t3, t4, t5, 0, vinit);

		outputCOM.push_back(ComZmp[0]);
		outputZMP.push_back(ComZmp[1]);
	}

}


void CnewPGstepStudy::genFOOTposition(vector<double> & outputX, vector<double> & outputY, double incrTime, double xinit, double yinit, double xend, double yend, double delay, double t1, double t2, double t3, double t4, double t5)
{

	outputX.clear();
	outputY.clear();

	for(double i = 0.0 ; i < t5 ; i += incrTime)
	{

		if(i < t2+delay)
		{

			outputX.push_back(xinit);
			outputY.push_back(yinit);

		}
		else if(i < t3-delay)
		{

			outputX.push_back(xinit + (-2/pow(t3-t2-2*delay,3.0)*pow(i-t2-delay,3.0)+3/pow(t3-t2-2*delay,2.0)*pow(i-t2-delay,2.0))*(xend-xinit));
			outputY.push_back(yinit + (-2/pow(t3-t2-2*delay,3.0)*pow(i-t2-delay,3.0)+3/pow(t3-t2-2*delay,2.0)*pow(i-t2-delay,2.0))*(yend-yinit));

		}
		else
		{

			outputX.push_back(xend);
			outputY.push_back(yend);

		}

	}

}


void CnewPGstepStudy::genFOOTheight(vector<double> & output, double incrTime, double heightMax, double t1, double t2, double t3, double t4, double t5)
{

	output.clear();

	for(double i = 0.0 ; i < t5 ; i += incrTime)
	{

		if(i < t2)
		{

			output.push_back(0);

		}
		else if(i < t3)
		{

			output.push_back( 16*heightMax/pow(t3-t2,4.0)*pow(i-t2,4.0)  -  32*heightMax/pow(t3-t2,3.0)*pow(i-t2,3.0)  +  16*heightMax/pow(t3-t2,2.0)*pow(i-t2,2.0));

		}
		else
		{

			output.push_back(0);

		}

	}

}

void CnewPGstepStudy::genFOOTdownUPheight(vector<double> & output, double incrTime, double heightMax, double t1, double t2, double t3)
{

	output.clear();

	for(double i = 0.0 ; i < t3 ; i += incrTime)
	{

		if(i < t2)
		{

			output.push_back(0);

		}
		else
		{

			output.push_back( -2*heightMax/pow(t3-t2,3.0)*pow(i-t2,3.0) + 3*heightMax/pow(t3-t2,2.0)*pow(i-t2,2.0));

		}

	}

}

void CnewPGstepStudy::genFOOTupDOWNheight(vector<double> & output, double incrTime, double heightMax, double t1, double t2, double t3)
{

	output.clear();

	for(double i = 0.0 ; i < t3 ; i += incrTime)
	{

		if(i < t1)
		{

			output.push_back( -2*heightMax/pow(0-t1,3.0)*pow(i-t1,3.0)  +  3*heightMax/pow(0-t1,2.0)*pow(i-t1,2.0));

		}
		else 
		{

			output.push_back(0);

		}

	}

}

void CnewPGstepStudy::genFOOTorientation(vector<double> & output, double incrTime, double initOrient, double endOrient, double delay, double t1, double t2, double t3, double t4, double t5)
{

	output.clear();

	for(double i = 0.0 ; i < t5 ; i += incrTime)
	{

		if(i < t2+delay)
		{

			output.push_back(initOrient);

		}
		else if(i < t3-delay)
		{

			output.push_back( initOrient + (-2/pow(t3-t2,3.0)*pow(i-t2,3.0) + 3/pow(t3-t2,2.0)*pow(i-t2,2.0))*(endOrient - initOrient) );

		}
		else
		{

			output.push_back(endOrient);

		}

	}

}


void CnewPGstepStudy::genWAISTorientation(vector<double> & output, double incrTime, double initOrient, double endOrient, double delay, double t1, double t2, double t3, double t4, double t5)
{

	output.clear();

	for(double i = 0.0 ; i < t5 ; i += incrTime)
	{

		if(i < t2+delay)
		{

			output.push_back(initOrient);

		}
		else if(i < t3-delay)
		{

			output.push_back( initOrient + (-2/pow(t3-t2,3.0)*pow(i-t2,3.0) + 3/pow(t3-t2,2.0)*pow(i-t2,2.0))*(endOrient - initOrient) );

		}
		else
		{

			output.push_back(endOrient);

		}

	}

}

void CnewPGstepStudy::genFullBodyConfig(
	int count, 
	MAL_VECTOR(,double) & jointsRadValues, 
	vector<double> & comTrajX, 
	vector<double> & comTrajY, 
	vector<double> & waistOrient, 
	vector<double> & footXtraj, 
	vector<double> & footYtraj, 
	vector<double> & footOrient, 
	vector<double> & footHeight, 
	double positionXstableFoot,
	double positionYstableFoot,
	char leftOrRightFootStable, 
	double zc) 
{
 
	MAL_S3x3_MATRIX(Body_R,double);
	MAL_S3_VECTOR(Body_P,double);
	MAL_S3_VECTOR(Dt,double);
	MAL_S3x3_MATRIX(Foot_R,double);
	MAL_S3_VECTOR(Foot_P,double);
	MAL_VECTOR_DIM(q,double,6);
	MAL_VECTOR_DIM(q_mem,double,6);
	//Initialization of these matrices and vectors for the right leg:

	// body rotation
	MAL_S3x3_MATRIX_CLEAR(Body_R);
	MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 0, 0) = 1.0;
	MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 1, 1) = 1.0;
	MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 2, 2) = 1.0;
	// MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 0, 0) = cos(((mp_foot1Theta1)+(mp_foot2Theta1))/2);
	// MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 1, 0) = sin(((mp_foot1Theta1)+(mp_foot2Theta1))/2);
	// MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 0, 1) = -sin(((mp_foot1Theta1)+(mp_foot2Theta1))/2);
	// MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 1, 1) = cos(((mp_foot1Theta1)+(mp_foot2Theta1))/2);
	// MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 2, 2) = 1.0;

	// body translation : vector (0,0,0)
	MAL_S3_VECTOR_FILL(Body_P,0.0);

		//we describe the horizontal configuration of the feet at time, relative to the waist (considered as the root position and orientation), with 6 parameters in the style x,y,t1,-x,-y,t2.
		//VERY IMPORTANT REMARK: the positions have been calculated according to the initial orientation of the waist: zero. Therefore a rotation has to be performed in order to deal with that. 

		vector<double> feetVector;
		feetVector.resize(6);

		feetVector[0] = (positionXstableFoot - comTrajX[count])*cos(-waistOrient[count]*PI/180) - (positionYstableFoot - comTrajY[count])*sin(-waistOrient[count]*PI/180);
		feetVector[1] = (positionXstableFoot - comTrajX[count])*sin(-waistOrient[count]*PI/180) + (positionYstableFoot - comTrajY[count])*cos(-waistOrient[count]*PI/180);
		feetVector[2] = -waistOrient[count];
		feetVector[3] = (footXtraj[count] - comTrajX[count])*cos(-waistOrient[count]*PI/180) - (footYtraj[count] - comTrajY[count])*sin(-waistOrient[count]*PI/180);
		feetVector[4] = (footXtraj[count] - comTrajX[count])*sin(-waistOrient[count]*PI/180) + (footYtraj[count] - comTrajY[count])*cos(-waistOrient[count]*PI/180);
		feetVector[5] = footOrient[count] - waistOrient[count];

		//we convert from feet to ankles:
		if(leftOrRightFootStable == 'R')
		{
			feetVector[0]+=-cos(feetVector[2]*PI/180-PI/2)*0.035;
			feetVector[1]+=-sin(feetVector[2]*PI/180-PI/2)*0.035;
			feetVector[3]+=-cos(feetVector[5]*PI/180+PI/2)*0.035;
			feetVector[4]+=-sin(feetVector[5]*PI/180+PI/2)*0.035;
		}
		else if(leftOrRightFootStable == 'L')
		{
			feetVector[3]+=-cos(feetVector[5]*PI/180-PI/2)*0.035;
			feetVector[4]+=-sin(feetVector[5]*PI/180-PI/2)*0.035;
			feetVector[0]+=-cos(feetVector[2]*PI/180+PI/2)*0.035;
			feetVector[1]+=-sin(feetVector[2]*PI/180+PI/2)*0.035;
		}

		double foot1X1=feetVector[0];
		double foot1Y1=feetVector[1];
		double foot1Theta1=feetVector[2]*PI/180;

		double foot2X1=feetVector[3];
		double foot2Y1=feetVector[4];
		double foot2Theta1=feetVector[5]*PI/180;

		//remark: the InverseKinematics takes into account the position of
		//the ankle so the z-limit for HRP2 is 60cm whereas 10.5cm have to be added to
		//get the actual distance between the ground and the waist root (i
		//suppose here that the HRP2's feet stay flat on the ground)
		//other remark: the ankle "zero" y-translation is 6cm.

		//Now we define the joint values for the right leg:

		// Dt (leg)
		MAL_S3_VECTOR_ACCESS(Dt,0) = 0.0;
		MAL_S3_VECTOR_ACCESS(Dt,1) = -0.06;
		MAL_S3_VECTOR_ACCESS(Dt,2) = 0.0;

		if(leftOrRightFootStable == 'R')
		{
			// ankle rotation
			MAL_S3x3_MATRIX_CLEAR(Foot_R);
			MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 0, 0) =  cos(foot1Theta1);
			MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 1, 0) =  sin(foot1Theta1);
			MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 0, 1) =  -sin(foot1Theta1);
			MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 1, 1) =  cos(foot1Theta1);
			MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 2, 2) =  1.0;

			// ankle translation
			MAL_S3_VECTOR_ACCESS(Foot_P, 0) = foot1X1;
			MAL_S3_VECTOR_ACCESS(Foot_P, 1) = foot1Y1;
								 // - 0.105;
			MAL_S3_VECTOR_ACCESS(Foot_P, 2) = -zc;
		}
		else if(leftOrRightFootStable == 'L')
		{
			// ankle rotation
			MAL_S3x3_MATRIX_CLEAR(Foot_R);
			MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 0, 0) =  cos(foot2Theta1);
			MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 1, 0) =  sin(foot2Theta1);
			MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 0, 1) =  -sin(foot2Theta1);
			MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 1, 1) =  cos(foot2Theta1);
			MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 2, 2) =  1.0;

			// ankle translation
			MAL_S3_VECTOR_ACCESS(Foot_P, 0) = foot2X1;
			MAL_S3_VECTOR_ACCESS(Foot_P, 1) = foot2Y1;
								 // - 0.105
			MAL_S3_VECTOR_ACCESS(Foot_P, 2) = -zc + footHeight[count];
		}

		InverseKinematics::ComputeInverseKinematics2ForLegs(Body_R, Body_P,Dt,Foot_R, Foot_P,q);

		for(int k=0;k<6;k++)
		{
			q_mem[k]=q[k];
		}

		//values for the left leg:

		MAL_S3_VECTOR_ACCESS(Dt,1) = 0.06;

		if(leftOrRightFootStable == 'R')
		{
			// ankle rotation
			MAL_S3x3_MATRIX_CLEAR(Foot_R);
			MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 0, 0) =  cos(foot2Theta1);
			MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 1, 0) =  sin(foot2Theta1);
			MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 0, 1) =  -sin(foot2Theta1);
			MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 1, 1) =  cos(foot2Theta1);
			MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 2, 2) =  1.0;

			// ankle translation
			MAL_S3_VECTOR_ACCESS(Foot_P, 0) = foot2X1;
			MAL_S3_VECTOR_ACCESS(Foot_P, 1) = foot2Y1;
			MAL_S3_VECTOR_ACCESS(Foot_P, 2) = -zc + footHeight[count];
		}
		else if(leftOrRightFootStable == 'L')
		{
			// ankle rotation
			MAL_S3x3_MATRIX_CLEAR(Foot_R);
			MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 0, 0) =  cos(foot1Theta1);
			MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 1, 0) =  sin(foot1Theta1);
			MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 0, 1) =  -sin(foot1Theta1);
			MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 1, 1) =  cos(foot1Theta1);
			MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 2, 2) =  1.0;

			// ankle translation
			MAL_S3_VECTOR_ACCESS(Foot_P, 0) = foot1X1;
			MAL_S3_VECTOR_ACCESS(Foot_P, 1) = foot1Y1;
			MAL_S3_VECTOR_ACCESS(Foot_P, 2) = -zc;
		}

		InverseKinematics::ComputeInverseKinematics2ForLegs(Body_R, Body_P, Dt, Foot_R, Foot_P, q);

		jointsRadValues.clear();
		jointsRadValues.resize(42);

		jointsRadValues[0]= q_mem[0];
		jointsRadValues[1]= q_mem[1];
		jointsRadValues[2]= q_mem[2];
		jointsRadValues[3]= q_mem[3];
		jointsRadValues[4]= q_mem[4];
		jointsRadValues[5]= q_mem[5];

		jointsRadValues[6]= q[0];
		jointsRadValues[7]= q[1];
		jointsRadValues[8]= q[2];
		jointsRadValues[9]= q[3];
		jointsRadValues[10]= q[4];
		jointsRadValues[11]= q[5];

		jointsRadValues[12]=0.0;
		jointsRadValues[13]=0.0;

		jointsRadValues[14]=0.0;
		jointsRadValues[15]=0.0;

		jointsRadValues[16]=0.2583087;
		jointsRadValues[17]=-0.174533;
		jointsRadValues[18]=0.000000;
		jointsRadValues[19]=-0.523599;
		jointsRadValues[20]=0.000000;
		jointsRadValues[21]=0.000000;
		jointsRadValues[22]=0.174533;
		jointsRadValues[23]=0.000000;

		jointsRadValues[24]=0.2583087;
		jointsRadValues[25]=0.174533;
		jointsRadValues[26]=0.000000;
		jointsRadValues[27]=-0.523599;
		jointsRadValues[28]=0.000000;
		jointsRadValues[29]=0.000000;
		jointsRadValues[30]=0.174533;
		jointsRadValues[31]=0.000000;
		/*
			 jointsRadValues[16]=0.261809;
			 jointsRadValues[17]=-0.174533;

			 jointsRadValues[18]=0.000000;

			 jointsRadValues[19]=0.000000;
			 jointsRadValues[20]=-0.523599;
			 jointsRadValues[21]=0.000000;
			 jointsRadValues[22]=0.000000;
			 jointsRadValues[23]=0.174533;

			 jointsRadValues[24]=0.261809;
			 jointsRadValues[25]=0.174533;

			 jointsRadValues[26]=0.000000;

			 jointsRadValues[27]=0.000000;
			 jointsRadValues[28]=-0.523599;
			 jointsRadValues[29]=0.000000;
			 jointsRadValues[30]=0.000000;
			 jointsRadValues[31]=0.174533;
		*/

		// We have all the joints' values, except for the hands.
		//RHAND_JOINT0
		//RHAND_JOINT1
		//...
		//LHAND_JOINT3
		//LHAND_JOINT4
		// Hands' joints at zero:

		jointsRadValues[32]=0.0;
		jointsRadValues[33]=0.0;
		jointsRadValues[34]=0.0;
		jointsRadValues[35]=0.0;
		jointsRadValues[36]=0.0;
		jointsRadValues[37]=0.0;
		jointsRadValues[38]=0.0;
		jointsRadValues[39]=0.0;
		jointsRadValues[40]=0.0;
		jointsRadValues[41]=0.0;

}

double CnewPGstepStudy::genFullBodyTrajectoryFromStepFeatures(
	ofstream & fb,
	ofstream & fbZMP,
	bool withSelfCollision,
	Chrp2OptHumanoidDynamicRobot * HDR,
	HumanoidRobotCollisionDetection * aHRCD,
	vector<int> & vectOfBodies,
	StepFeatures & stepF
	) 
{

	double rmin = 999;

	MAL_VECTOR(,double) jointsRadValues;

	for(unsigned int count = 0; count < stepF.size ; count++) {


	MAL_S3x3_MATRIX(Body_R,double);
	MAL_S3_VECTOR(Body_P,double);
	MAL_S3_VECTOR(Dt,double);
	MAL_S3x3_MATRIX(Foot_R,double);
	MAL_S3_VECTOR(Foot_P,double);
	MAL_VECTOR_DIM(q,double,6);
	MAL_VECTOR_DIM(q_mem,double,6);
	//Initialization of these matrices and vectors for the right leg:

	// body rotation
	MAL_S3x3_MATRIX_CLEAR(Body_R);
	MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 0, 0) = 1.0;
	MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 1, 1) = 1.0;
	MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 2, 2) = 1.0;
	// MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 0, 0) = cos(((mp_foot1Theta1)+(mp_foot2Theta1))/2);
	// MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 1, 0) = sin(((mp_foot1Theta1)+(mp_foot2Theta1))/2);
	// MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 0, 1) = -sin(((mp_foot1Theta1)+(mp_foot2Theta1))/2);
	// MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 1, 1) = cos(((mp_foot1Theta1)+(mp_foot2Theta1))/2);
	// MAL_S3x3_MATRIX_ACCESS_I_J(Body_R, 2, 2) = 1.0;

	// body translation : vector (0,0,0)
	MAL_S3_VECTOR_FILL(Body_P,0.0);

		//we describe the horizontal configuration of the feet relative to the waist (considered as the root position and orientation), with 6 parameters: (Left) x,y,t1, (Right) x',y',t2.
		//VERY IMPORTANT REMARK: the positions have been calculated according to the initial orientation of the waist: zero. Therefore a rotation has to be performed in order to deal with that. 

		vector<double> feetVector;
		feetVector.resize(6);

		feetVector[0] = (stepF.leftfootXtraj[count] - stepF.comTrajX[count])*cos(-stepF.waistOrient[count]*PI/180) - (stepF.leftfootYtraj[count] - stepF.comTrajY[count])*sin(-stepF.waistOrient[count]*PI/180);
		feetVector[1] = (stepF.leftfootXtraj[count] - stepF.comTrajX[count])*sin(-stepF.waistOrient[count]*PI/180) + (stepF.leftfootYtraj[count] - stepF.comTrajY[count])*cos(-stepF.waistOrient[count]*PI/180);
		feetVector[2] = stepF.leftfootOrient[count] - stepF.waistOrient[count];
		feetVector[3] = (stepF.rightfootXtraj[count] - stepF.comTrajX[count])*cos(-stepF.waistOrient[count]*PI/180) - (stepF.rightfootYtraj[count] - stepF.comTrajY[count])*sin(-stepF.waistOrient[count]*PI/180);
		feetVector[4] = (stepF.rightfootXtraj[count] - stepF.comTrajX[count])*sin(-stepF.waistOrient[count]*PI/180) + (stepF.rightfootYtraj[count] - stepF.comTrajY[count])*cos(-stepF.waistOrient[count]*PI/180);
		feetVector[5] = stepF.rightfootOrient[count] - stepF.waistOrient[count];

		//we convert from feet to ankles:
		feetVector[3]+=-cos(feetVector[5]*PI/180-PI/2)*0.035;
		feetVector[4]+=-sin(feetVector[5]*PI/180-PI/2)*0.035;
		feetVector[0]+=-cos(feetVector[2]*PI/180+PI/2)*0.035;
		feetVector[1]+=-sin(feetVector[2]*PI/180+PI/2)*0.035;		

		double foot1X1=feetVector[0];
		double foot1Y1=feetVector[1];
		double foot1Theta1=feetVector[2]*PI/180;

		double foot2X1=feetVector[3];
		double foot2Y1=feetVector[4];
		double foot2Theta1=feetVector[5]*PI/180;

		//remark: the InverseKinematics takes into account the position of
		//the ankle so the z-limit for HRP2 is 60cm whereas 10.5cm have to be added to
		//get the actual distance between the ground and the waist root (i
		//suppose here that the HRP2's feet stay flat on the ground)
		//other remark: the ankle "zero" y-translation is 6cm.

		//Now we define the joint values for the right leg:

		// Dt (leg)
		MAL_S3_VECTOR_ACCESS(Dt,0) = 0.0;
		MAL_S3_VECTOR_ACCESS(Dt,1) = -0.06;
		MAL_S3_VECTOR_ACCESS(Dt,2) = 0.0;

			// ankle rotation
			MAL_S3x3_MATRIX_CLEAR(Foot_R);
			MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 0, 0) =  cos(foot2Theta1);
			MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 1, 0) =  sin(foot2Theta1);
			MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 0, 1) =  -sin(foot2Theta1);
			MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 1, 1) =  cos(foot2Theta1);
			MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 2, 2) =  1.0;

			// ankle translation
			MAL_S3_VECTOR_ACCESS(Foot_P, 0) = foot2X1;
			MAL_S3_VECTOR_ACCESS(Foot_P, 1) = foot2Y1;
								 // - 0.105
			MAL_S3_VECTOR_ACCESS(Foot_P, 2) = -stepF.zc + stepF.rightfootHeight[count];

		InverseKinematics::ComputeInverseKinematics2ForLegs(Body_R, Body_P,Dt,Foot_R, Foot_P,q);

		for(int k=0;k<6;k++)
		{
			q_mem[k]=q[k];
		}

		//values for the left leg:

		MAL_S3_VECTOR_ACCESS(Dt,0) = 0.0;
		MAL_S3_VECTOR_ACCESS(Dt,1) = +0.06;
		MAL_S3_VECTOR_ACCESS(Dt,2) = 0.0;

			// ankle rotation
			MAL_S3x3_MATRIX_CLEAR(Foot_R);
			MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 0, 0) =  cos(foot1Theta1);
			MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 1, 0) =  sin(foot1Theta1);
			MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 0, 1) =  -sin(foot1Theta1);
			MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 1, 1) =  cos(foot1Theta1);
			MAL_S3x3_MATRIX_ACCESS_I_J(Foot_R, 2, 2) =  1.0;

			// ankle translation
			MAL_S3_VECTOR_ACCESS(Foot_P, 0) = foot1X1;
			MAL_S3_VECTOR_ACCESS(Foot_P, 1) = foot1Y1;
			MAL_S3_VECTOR_ACCESS(Foot_P, 2) = -stepF.zc + stepF.leftfootHeight[count];

		InverseKinematics::ComputeInverseKinematics2ForLegs(Body_R, Body_P, Dt, Foot_R, Foot_P, q);

		jointsRadValues.clear();
		jointsRadValues.resize(42);

		jointsRadValues[0]= q_mem[0];
		jointsRadValues[1]= q_mem[1];
		jointsRadValues[2]= q_mem[2];
		jointsRadValues[3]= q_mem[3];
		jointsRadValues[4]= q_mem[4];
		jointsRadValues[5]= q_mem[5];

		jointsRadValues[6]= q[0];
		jointsRadValues[7]= q[1];
		jointsRadValues[8]= q[2];
		jointsRadValues[9]= q[3];
		jointsRadValues[10]= q[4];
		jointsRadValues[11]= q[5];

		jointsRadValues[12]=0.0;
		jointsRadValues[13]=0.0;

		jointsRadValues[14]=0.0;
		jointsRadValues[15]=0.0;

		jointsRadValues[16]=0.2583087;
		jointsRadValues[17]=-0.174533;
		jointsRadValues[18]=0.000000;
		jointsRadValues[19]=-0.523599;
		jointsRadValues[20]=0.000000;
		jointsRadValues[21]=0.000000;
		jointsRadValues[22]=0.174533;
		jointsRadValues[23]=0.000000;

		jointsRadValues[24]=0.2583087;
		jointsRadValues[25]=0.174533;
		jointsRadValues[26]=0.000000;
		jointsRadValues[27]=-0.523599;
		jointsRadValues[28]=0.000000;
		jointsRadValues[29]=0.000000;
		jointsRadValues[30]=0.174533;
		jointsRadValues[31]=0.000000;
	
		jointsRadValues[32]=0.0;
		jointsRadValues[33]=0.0;
		jointsRadValues[34]=0.0;
		jointsRadValues[35]=0.0;
		jointsRadValues[36]=0.0;
		jointsRadValues[37]=0.0;
		jointsRadValues[38]=0.0;
		jointsRadValues[39]=0.0;
		jointsRadValues[40]=0.0;
		jointsRadValues[41]=0.0;

		
		if(withSelfCollision){
		//======================================================================================

  		MAL_VECTOR_DIM(totall,double,48);	

		MAL_VECTOR(,double) freefly;
    		freefly.resize(6);
    		for(int i=0;i<6;i++) {
      		  freefly[i]=0;
    		}
    		concatvectors(freefly,jointsRadValues,totall);


		Vect3 aP2;
  		Mat3 aR2;
  
		VclipPose aX;
		  
  		// Take the posture from the multibody configuration.
  		vector<CjrlJoint *> aVecOfJoints;
  		CjrlJoint *aJoint; 
  		aVecOfJoints = HDR->jointVector(); 
  		for(unsigned int i=0; i < 28 ;i++) //vectOfBodies.size();i++)
    		{

      			matrix4d aCurrentM;
      			aJoint = aVecOfJoints[i+1];//mp_vectOfBodies[i]];//aDMB->GetJointFromVRMLID(i);
			      
			aJoint->updateTransformation(totall);
      			aCurrentM = aJoint->currentTransformation();
			cout << aJoint->rankInConfiguration() << " ";  
      			aP2[0] = MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,0,3);
      			aP2[1] = MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,1,3);
      			aP2[2] = MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,2,3);
	      
			aR2[0][0] = MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,0,0); 
      			aR2[0][1] = MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,0,1); 
      			aR2[0][2] = MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,0,2);
      			aR2[1][0] = MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,1,0); 
      			aR2[1][1] = MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,1,1); 
      			aR2[1][2] = MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,1,2);
      			aR2[2][0] = MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,2,0);
      			aR2[2][1] = MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,2,1);
      			aR2[2][2] = MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,2,2);
      
	      		aX.set(aR2,aP2);
      			aHRCD->SetBodyPose(i+1,aX);
    		}
  
 		double r;
  
  		// Update the body posture.
  		aX = VclipPose::ID;
  		aHRCD->SetBodyPose(0,aX);
  
		// Check the possible collision.
  		r = aHRCD->ComputeSelfCollision(); cout << endl << r << endl;

  		if (r<rmin) rmin=r;

		//======================================================================================
		}

		fb << stepF.incrTime * count << " ";
		for(int j = 0; j < 42; j++)
		{
			fb << jointsRadValues[j] << " ";
		}
		fb << endl;

		fbZMP << stepF.incrTime * count << " ";
		fbZMP << (stepF.zmpTrajX[count]-stepF.comTrajX[count])*cos(-stepF.waistOrient[count]*PI/180)
			-(stepF.zmpTrajY[count]-stepF.comTrajY[count])*sin(-stepF.waistOrient[count]*PI/180) << " ";
		fbZMP << (stepF.zmpTrajX[count]-stepF.comTrajX[count])*sin(-stepF.waistOrient[count]*PI/180)
			+(stepF.zmpTrajY[count]-stepF.comTrajY[count])*cos(-stepF.waistOrient[count]*PI/180) << " ";
		fbZMP << -stepF.zc-0.105 << " ";
		fbZMP << endl;

	}	

	return rmin;

}





void CnewPGstepStudy::addStepFeaturesWithSlide(
	StepFeatures & stepF1,
	StepFeatures & stepF2,	
	double negativeSlideTime
	) 
{
	
	//PHASE 1: modify stepF2 according to the change of origin
	double lastwaistX = stepF1.comTrajX[stepF1.size - 1];
	double lastwaistY = stepF1.comTrajY[stepF1.size - 1]; 
	double radlastwaistOrient = stepF1.waistOrient[stepF1.size - 1]*PI/180;

	for(unsigned int count = 0 ; count < stepF2.size ; count++) {

	double newcomX = (stepF2.comTrajX[count]+lastwaistX)*cos(radlastwaistOrient)
			-(stepF2.comTrajY[count]+lastwaistY)*sin(radlastwaistOrient);
	double newcomY = (stepF2.comTrajX[count]+lastwaistX)*sin(radlastwaistOrient)
			+(stepF2.comTrajY[count]+lastwaistY)*cos(radlastwaistOrient);
	double newzmpX = (stepF2.zmpTrajX[count]+lastwaistX)*cos(radlastwaistOrient)
			-(stepF2.zmpTrajY[count]+lastwaistY)*sin(radlastwaistOrient);
	double newzmpY = (stepF2.zmpTrajX[count]+lastwaistX)*sin(radlastwaistOrient)
			+(stepF2.zmpTrajY[count]+lastwaistY)*cos(radlastwaistOrient);
	double newlfX = (stepF2.leftfootXtraj[count]+lastwaistX)*cos(radlastwaistOrient)
			-(stepF2.leftfootYtraj[count]+lastwaistY)*sin(radlastwaistOrient);	
	double newlfY = (stepF2.leftfootXtraj[count]+lastwaistX)*sin(radlastwaistOrient)
			+(stepF2.leftfootYtraj[count]+lastwaistY)*cos(radlastwaistOrient);	
	double newrfX = (stepF2.rightfootXtraj[count]+lastwaistX)*cos(radlastwaistOrient)
			-(stepF2.rightfootYtraj[count]+lastwaistY)*sin(radlastwaistOrient);
	double newrfY = (stepF2.rightfootXtraj[count]+lastwaistX)*sin(radlastwaistOrient)   
			+(stepF2.rightfootYtraj[count]+lastwaistY)*cos(radlastwaistOrient);	

	stepF2.comTrajX[count] = newcomX;
	stepF2.zmpTrajX[count] = newzmpX;

	stepF2.comTrajY[count] = newcomY;
	stepF2.zmpTrajY[count] = newzmpY;

	stepF2.leftfootXtraj[count] = newlfX;
	stepF2.leftfootYtraj[count] = newlfY;

	stepF2.leftfootOrient[count] += stepF1.waistOrient[stepF1.size - 1];

	stepF2.rightfootXtraj[count] = newrfX;
	stepF2.rightfootYtraj[count] = newrfY;

	stepF2.rightfootOrient[count] += stepF1.waistOrient[stepF1.size - 1];

	stepF2.waistOrient[count] += stepF1.waistOrient[stepF1.size - 1];	

	}

	int delayInt = (int) (abs(negativeSlideTime)/stepF2.incrTime);

	//PHASE 2: add the new stepF2 to stepF1
	for(unsigned int count = 0 ; count < stepF2.size ; count++) {

	if(count < delayInt) {

	stepF1.comTrajX[stepF1.size - delayInt + count] =
		(stepF1.comTrajX[stepF1.size - delayInt + count] + stepF2.comTrajX[count])
		-stepF1.comTrajX[stepF1.size - 1];
	stepF1.zmpTrajX[stepF1.size - delayInt + count] =
		(stepF1.zmpTrajX[stepF1.size - delayInt + count] + stepF2.zmpTrajX[count])
		-stepF1.zmpTrajX[stepF1.size - 1];

	stepF1.comTrajY[stepF1.size - delayInt + count] =
		( stepF1.comTrajY[stepF1.size - delayInt + count] + stepF2.comTrajY[count])
		-stepF1.comTrajY[stepF1.size - 1];
	stepF1.zmpTrajY[stepF1.size - delayInt + count] =
		( stepF1.zmpTrajY[stepF1.size - delayInt + count] + stepF2.zmpTrajY[count])
		-stepF1.zmpTrajY[stepF1.size - 1];

	stepF1.leftfootXtraj[stepF1.size - delayInt + count] =
		( stepF1.leftfootXtraj[stepF1.size - delayInt + count] + stepF2.leftfootXtraj[count])
		-stepF1.leftfootXtraj[stepF1.size - 1];
	stepF1.leftfootYtraj[stepF1.size - delayInt + count] =
		( stepF1.leftfootYtraj[stepF1.size - delayInt + count] + stepF2.leftfootYtraj[count])
		-stepF1.leftfootYtraj[stepF1.size - 1];

	stepF1.leftfootOrient[stepF1.size - delayInt + count] =
		( stepF1.leftfootOrient[stepF1.size - delayInt + count] + stepF2.leftfootOrient[count])
		-stepF1.leftfootOrient[stepF1.size - 1];
	stepF1.leftfootHeight[stepF1.size - delayInt + count] =
		(  stepF1.leftfootHeight[stepF1.size - delayInt + count] + stepF2.leftfootHeight[count])
		-stepF1.leftfootHeight[stepF1.size - 1];

	stepF1.rightfootXtraj[stepF1.size - delayInt + count] =
		( stepF1.rightfootXtraj[stepF1.size - delayInt + count] + stepF2.rightfootXtraj[count])
		-stepF1.rightfootXtraj[stepF1.size - 1];
	stepF1.rightfootYtraj[stepF1.size - delayInt + count] =
		( stepF1.rightfootYtraj[stepF1.size - delayInt + count] + stepF2.rightfootYtraj[count])
		-stepF1.rightfootYtraj[stepF1.size - 1];

	stepF1.rightfootOrient[stepF1.size - delayInt + count] =
		( stepF1.rightfootOrient[stepF1.size - delayInt + count] + stepF2.rightfootOrient[count])
		-stepF1.rightfootOrient[stepF1.size - 1];
	stepF1.rightfootHeight[stepF1.size - delayInt + count] =
		( stepF1.rightfootHeight[stepF1.size - delayInt + count] + stepF2.rightfootHeight[count])
		-stepF1.rightfootHeight[stepF1.size - 1];

	stepF1.waistOrient[stepF1.size - delayInt + count] =
		( stepF1.waistOrient[stepF1.size - delayInt + count] + stepF2.waistOrient[count])
		-stepF1.waistOrient[stepF1.size - 1];

	} else {

	stepF1.comTrajX.push_back(stepF2.comTrajX[count]);
	stepF1.zmpTrajX.push_back(stepF2.zmpTrajX[count]);

	stepF1.comTrajY.push_back(stepF2.comTrajY[count]);
	stepF1.zmpTrajY.push_back(stepF2.zmpTrajY[count]);

	stepF1.leftfootXtraj.push_back(stepF2.leftfootXtraj[count]);
	stepF1.leftfootYtraj.push_back(stepF2.leftfootYtraj[count]);

	stepF1.leftfootOrient.push_back(stepF2.leftfootOrient[count]);
	stepF1.leftfootHeight.push_back(stepF2.leftfootHeight[count]);

	stepF1.rightfootXtraj.push_back(stepF2.rightfootXtraj[count]);
	stepF1.rightfootYtraj.push_back(stepF2.rightfootYtraj[count]);

	stepF1.rightfootOrient.push_back(stepF2.rightfootOrient[count]);
	stepF1.rightfootHeight.push_back(stepF2.rightfootHeight[count]);

	stepF1.waistOrient.push_back(stepF2.waistOrient[count]);	

	}

	}

	stepF1.size = stepF1.size + stepF2.size - delayInt;

}



void CnewPGstepStudy::produceOneStep(vector<vector<double> > & fb, vector<vector<double> > & fbZMP, double incrTime, double zc, double g, double stepHeight, double t1, double t2, double t3, double t4, double t5, vector<double> vectStep_input, char leftOrRightFootStable)
{

	fb.clear();
	fbZMP.clear();	

	vector<double> comTrajX;
	vector<double> zmpTrajX;
	genCOMZMPtrajectory(comTrajX, zmpTrajX, incrTime, zc, g, 0, vectStep_input[0], vectStep_input[6]/2, t1, t2, t3, t4, t5);

	vector<double> comTrajY;
	vector<double> zmpTrajY;
	genCOMZMPtrajectory(comTrajY, zmpTrajY, incrTime, zc, g, 0, vectStep_input[1], vectStep_input[7]/2, t1, t2, t3, t4, t5);

	vector<double> footXtraj;
	vector<double> footYtraj;
	genFOOTposition(footXtraj, footYtraj, incrTime, vectStep_input[3], vectStep_input[4], vectStep_input[0]+vectStep_input[6], vectStep_input[1]+vectStep_input[7], 0.02, t1, t2, t3, t4, t5);

	vector<double> footHeight;
	genFOOTheight(footHeight, incrTime, stepHeight, t1, t2, t3, t4, t5);

	vector<double> footOrient;
	genFOOTorientation(footOrient, incrTime, vectStep_input[5], vectStep_input[8], 0.02, t1, t2, t3, t4, t5);

	vector<double> waistOrient;
	genWAISTorientation(waistOrient, incrTime, 0, vectStep_input[8], 0.02, t1, t2, t3, t4, t5);


	int count = -1;

	for(double i = 0.0 ; i < t5 ; i += incrTime)
	{
		count++;
		
		MAL_VECTOR(,double) jointsRadValues;
		genFullBodyConfig(count, jointsRadValues, comTrajX, comTrajY, waistOrient, footXtraj, footYtraj, footOrient, footHeight, vectStep_input[0], vectStep_input[1], leftOrRightFootStable, zc);

		/*
		fb << i << " ";
		for(int j = 0; j < 42; j++)
		{
			fb << jointsRadValues[j] << " " ;
		}
		fb << endl;

		fbZMP << i << " " << (zmpTrajX[count]-comTrajX[count])*cos(-waistOrient[count]*PI/180)
			-(zmpTrajY[count]-comTrajY[count])*sin(-waistOrient[count]*PI/180) << " " <<
			(zmpTrajX[count]-comTrajX[count])*sin(-waistOrient[count]*PI/180)
			+(zmpTrajY[count]-comTrajY[count])*cos(-waistOrient[count]*PI/180) << " " << -zc-0.105 << endl;
		*/

		vector<double> tmpvectCOM;
		vector<double> tmpvectZMP;		
		tmpvectCOM.push_back(i);  		
		for(int j = 0; j < 42; j++)
		{
			tmpvectCOM.push_back(jointsRadValues[j]);
		}
		fb.push_back(tmpvectCOM);

		tmpvectZMP.push_back(i);
		tmpvectZMP.push_back( (zmpTrajX[count]-comTrajX[count])*cos(-waistOrient[count]*PI/180)
			-(zmpTrajY[count]-comTrajY[count])*sin(-waistOrient[count]*PI/180)  );
		tmpvectZMP.push_back( (zmpTrajX[count]-comTrajX[count])*sin(-waistOrient[count]*PI/180)
			+(zmpTrajY[count]-comTrajY[count])*cos(-waistOrient[count]*PI/180)  );
		tmpvectZMP.push_back(  -zc-0.105   );
		fbZMP.push_back(tmpvectZMP);	

	}

}

void CnewPGstepStudy::produceOneStep(ofstream & fb, ofstream & fbZMP, double incrTime, double zc, double g, double stepHeight, double t1, double t2, double t3, double t4, double t5, vector<double> vectStep_input, char leftOrRightFootStable)
{
	vector<vector<double> > fbVect;
	vector<vector<double> > fbZMPVect;

	produceOneStep(fbVect, fbZMPVect, incrTime, zc, g, stepHeight, t1, t2, t3, t4, t5, vectStep_input, leftOrRightFootStable);

	int count = -1;
	for(double i = 0.0 ; i < t5 ; i += incrTime)
	{
		count++;

		fb << fbVect[count][0] << " ";
		for(int j = 0; j < 42; j++)
		{
			fb << fbVect[count][j+1] << " " ;
		}
		fb << endl;

		fbZMP << fbZMPVect[count][0] << " " << fbZMPVect[count][1] << " " <<
			fbZMPVect[count][2] << " " << fbZMPVect[count][3] << endl;
		
	}

}


void CnewPGstepStudy::produceOneUPHalfStep(vector<vector<double> > & fb, vector<vector<double> > & fbZMP, double incrTime, double zc, double g, double t1, double t2, double t3, vector<double> vectUPHalfStep_input, char leftOrRightFootStable)
{

	fb.clear();
	fbZMP.clear();	

	vector<double> comTrajX;
	vector<double> zmpTrajX;
	genCOMZMPtrajectory(comTrajX, zmpTrajX, incrTime, zc, g, 0, 0, vectUPHalfStep_input[0], t1/2, t1*3/4, t1, t2, t3);

	vector<double> comTrajY;
	vector<double> zmpTrajY;
	genCOMZMPtrajectory(comTrajY, zmpTrajY, incrTime, zc, g, 0, 0, vectUPHalfStep_input[1], t1/2, t1*3/4, t1, t2, t3);

	vector<double> footXtraj;
	vector<double> footYtraj;
	int leftRightCoef = 0;	
	if(leftOrRightFootStable == 'L') leftRightCoef = -1; else leftRightCoef = 1;
	genFOOTposition(footXtraj, footYtraj, incrTime, vectUPHalfStep_input[3], vectUPHalfStep_input[4], vectUPHalfStep_input[0], vectUPHalfStep_input[1]+leftRightCoef*vectUPHalfStep_input[6], 0.02, t1, t2, t3, t3, t3);

	vector<double> footHeight;
	genFOOTdownUPheight(footHeight, incrTime, vectUPHalfStep_input[7], t1, t2, t3);

	vector<double> footOrient;
	genFOOTorientation(footOrient, incrTime, vectUPHalfStep_input[5], 0, 0.02, t1, t2, t3, t3, t3);

	vector<double> waistOrient;
	genWAISTorientation(waistOrient, incrTime, 0, 0, 0.02, t1, t2, t3, t3, t3);


	int count = -1;

	for(double i = 0.0 ; i < t3 ; i += incrTime)
	{
		count++;
		
		MAL_VECTOR(,double) jointsRadValues;
		genFullBodyConfig(count, jointsRadValues, comTrajX, comTrajY, waistOrient, footXtraj, footYtraj, footOrient, footHeight, vectUPHalfStep_input[0], vectUPHalfStep_input[1], leftOrRightFootStable, zc);

		/*
		fb << i << " ";
		for(int j = 0; j < 42; j++)
		{
			fb << jointsRadValues[j] << " " ;
		}
		fb << endl;

		fbZMP << i << " " << (zmpTrajX[count]-comTrajX[count])*cos(-waistOrient[count]*PI/180)
			-(zmpTrajY[count]-comTrajY[count])*sin(-waistOrient[count]*PI/180) << " " <<
			(zmpTrajX[count]-comTrajX[count])*sin(-waistOrient[count]*PI/180)
			+(zmpTrajY[count]-comTrajY[count])*cos(-waistOrient[count]*PI/180) << " " << -zc-0.105 << endl;
		*/

		vector<double> tmpvectCOM;
		vector<double> tmpvectZMP;		
		tmpvectCOM.push_back(i);  		
		for(int j = 0; j < 42; j++)
		{
			tmpvectCOM.push_back(jointsRadValues[j]);
		}
		fb.push_back(tmpvectCOM);

		tmpvectZMP.push_back(i);
		tmpvectZMP.push_back( (zmpTrajX[count]-comTrajX[count])*cos(-waistOrient[count]*PI/180)
			-(zmpTrajY[count]-comTrajY[count])*sin(-waistOrient[count]*PI/180)  );
		tmpvectZMP.push_back( (zmpTrajX[count]-comTrajX[count])*sin(-waistOrient[count]*PI/180)
			+(zmpTrajY[count]-comTrajY[count])*cos(-waistOrient[count]*PI/180)  );
		tmpvectZMP.push_back(  -zc-0.105   );
		fbZMP.push_back(tmpvectZMP);	

	}
}

void CnewPGstepStudy::produceOneUPHalfStep(ofstream & fb, ofstream & fbZMP, double incrTime, double zc, double g, double t1, double t2, double t3, vector<double> vectUPHalfStep_input, char leftOrRightFootStable)
{
	vector<vector<double> > fbVect;
	vector<vector<double> > fbZMPVect;

	produceOneUPHalfStep(fbVect, fbZMPVect, incrTime, zc, g, t1, t2, t3, vectUPHalfStep_input, leftOrRightFootStable);
	
	int count = -1;
	for(double i = 0.0 ; i < t3 ; i += incrTime)
	{
		count++;

		fb << fbVect[count][0] << " ";
		for(int j = 0; j < 42; j++)
		{
			fb << fbVect[count][j+1] << " " ;
		}
		fb << endl;

		fbZMP << fbZMPVect[count][0] << " " << fbZMPVect[count][1] << " " <<
			fbZMPVect[count][2] << " " << fbZMPVect[count][3] << endl;
		
	}

}

void CnewPGstepStudy::produceOneUPHalfStepFeatures(StepFeatures & stepF, double incrTime, double zc, double g, double t1, double t2, double t3, vector<double> vectUPHalfStep_input, char leftOrRightFootStable)
{

	vector<double> comTrajX;
	vector<double> zmpTrajX;
	genCOMZMPtrajectory(comTrajX, zmpTrajX, incrTime, zc, g, 0, 0, vectUPHalfStep_input[0], t1/2, t1*3/4, t1, t2, t3);

	vector<double> comTrajY;
	vector<double> zmpTrajY;
	genCOMZMPtrajectory(comTrajY, zmpTrajY, incrTime, zc, g, 0, 0, vectUPHalfStep_input[1], t1/2, t1*3/4, t1, t2, t3);

	vector<double> footXtraj;
	vector<double> footYtraj;
	int leftRightCoef = 0;	
	if(leftOrRightFootStable == 'L') leftRightCoef = -1; else leftRightCoef = 1;
	genFOOTposition(footXtraj, footYtraj, incrTime, vectUPHalfStep_input[3], vectUPHalfStep_input[4], vectUPHalfStep_input[0], vectUPHalfStep_input[1]+leftRightCoef*vectUPHalfStep_input[6], 0.02, t1, t2, t3, t3, t3);

	vector<double> footHeight;
	genFOOTdownUPheight(footHeight, incrTime, vectUPHalfStep_input[7], t1, t2, t3);

	vector<double> footOrient;
	genFOOTorientation(footOrient, incrTime, vectUPHalfStep_input[5], 0, 0.02, t1, t2, t3, t3, t3);

	vector<double> stablefootXtraj;
	vector<double> stablefootYtraj;
	vector<double> stablefootHeight;
	vector<double> stablefootOrient;

	int count = -1;

	for(double i = 0.0 ; i < t3 ; i += incrTime)
	{
		count++;
		stablefootXtraj.push_back(vectUPHalfStep_input[0]);
		stablefootYtraj.push_back(vectUPHalfStep_input[1]);
		stablefootHeight.push_back(0);
		stablefootOrient.push_back(0);
	}

	vector<double> waistOrient;
	genWAISTorientation(waistOrient, incrTime, 0, 0, 0.02, t1, t2, t3, t3, t3);


	stepF.comTrajX = comTrajX;
	stepF.zmpTrajX = zmpTrajX;
	stepF.comTrajY = comTrajY;
	stepF.zmpTrajY = zmpTrajY;

	if(leftOrRightFootStable == 'L') {
	stepF.leftfootXtraj = stablefootXtraj;
	stepF.leftfootYtraj = stablefootYtraj;
	stepF.leftfootHeight = stablefootHeight;
	stepF.leftfootOrient = stablefootOrient;
	stepF.rightfootXtraj = footXtraj;
	stepF.rightfootYtraj = footYtraj;
	stepF.rightfootHeight = footHeight;
	stepF.rightfootOrient = footOrient;
	} else {
	stepF.leftfootXtraj = footXtraj;
	stepF.leftfootYtraj = footYtraj;
	stepF.leftfootHeight = footHeight;
	stepF.leftfootOrient = footOrient;
	stepF.rightfootXtraj = stablefootXtraj;
	stepF.rightfootYtraj = stablefootYtraj;
	stepF.rightfootHeight = stablefootHeight;
	stepF.rightfootOrient = stablefootOrient;
	}

	stepF.waistOrient =  waistOrient;	
	stepF.incrTime = incrTime;
	stepF.zc = zc;
	stepF.size = waistOrient.size();

}



void CnewPGstepStudy::produceOneDOWNHalfStep(vector<vector<double> > & fb, vector<vector<double> > & fbZMP, double incrTime, double zc, double g, double t1, double t2, double t3, vector<double> vectDOWNHalfStep_input, char leftOrRightFootStable)
{

	fb.clear();
	fbZMP.clear();	

	vector<double> comTrajX;
	vector<double> zmpTrajX;
	genCOMZMPtrajectory(comTrajX, zmpTrajX, incrTime, zc, g, 0, 0, vectDOWNHalfStep_input[2]/2, t1/2, t1*3/4, t1, t2, t3);

	vector<double> comTrajY;
	vector<double> zmpTrajY;
	genCOMZMPtrajectory(comTrajY, zmpTrajY, incrTime, zc, g, 0, 0, vectDOWNHalfStep_input[3]/2, t1/2, t1*3/4, t1, t2, t3);

	vector<double> footXtraj;
	vector<double> footYtraj;
	int leftRightCoef = 0;	
	if(leftOrRightFootStable == 'L') leftRightCoef = -1; else leftRightCoef = 1;
	genFOOTposition(footXtraj, footYtraj, incrTime, 0, leftRightCoef*vectDOWNHalfStep_input[0], vectDOWNHalfStep_input[2], vectDOWNHalfStep_input[3], 0.02, 0, 0, t1, t2, t3);

	vector<double> footHeight;
	genFOOTupDOWNheight(footHeight, incrTime, vectDOWNHalfStep_input[1], t1, t2, t3);

	vector<double> footOrient;
	genFOOTorientation(footOrient, incrTime, 0, vectDOWNHalfStep_input[4], 0.02, 0, 0, t1, t2, t3);

	vector<double> waistOrient;
	genWAISTorientation(waistOrient, incrTime, 0, vectDOWNHalfStep_input[4], 0.02, 0, 0, t1, t2, t3);


	int count = -1;

	for(double i = 0.0 ; i < t3 ; i += incrTime)
	{
		count++;
		
		MAL_VECTOR(,double) jointsRadValues;
		genFullBodyConfig(count, jointsRadValues, comTrajX, comTrajY, waistOrient, footXtraj, footYtraj, footOrient, footHeight, 0, 0, leftOrRightFootStable, zc);

		/*
		fb << i << " ";
		for(int j = 0; j < 42; j++)
		{
			fb << jointsRadValues[j] << " " ;
		}
		fb << endl;

		fbZMP << i << " " << (zmpTrajX[count]-comTrajX[count])*cos(-waistOrient[count]*PI/180)
			-(zmpTrajY[count]-comTrajY[count])*sin(-waistOrient[count]*PI/180) << " " <<
			(zmpTrajX[count]-comTrajX[count])*sin(-waistOrient[count]*PI/180)
			+(zmpTrajY[count]-comTrajY[count])*cos(-waistOrient[count]*PI/180) << " " << -zc-0.105 << endl;
		*/

		vector<double> tmpvectCOM;
		vector<double> tmpvectZMP;		
		tmpvectCOM.push_back(i);  		
		for(int j = 0; j < 42; j++)
		{
			tmpvectCOM.push_back(jointsRadValues[j]);
		}
		fb.push_back(tmpvectCOM);

		tmpvectZMP.push_back(i);
		tmpvectZMP.push_back( (zmpTrajX[count]-comTrajX[count])*cos(-waistOrient[count]*PI/180)
			-(zmpTrajY[count]-comTrajY[count])*sin(-waistOrient[count]*PI/180)  );
		tmpvectZMP.push_back( (zmpTrajX[count]-comTrajX[count])*sin(-waistOrient[count]*PI/180)
			+(zmpTrajY[count]-comTrajY[count])*cos(-waistOrient[count]*PI/180)  );
		tmpvectZMP.push_back(  -zc-0.105   );
		fbZMP.push_back(tmpvectZMP);	

	}
}

void CnewPGstepStudy::produceOneDOWNHalfStep(ofstream & fb, ofstream & fbZMP, double incrTime, double zc, double g, double t1, double t2, double t3, vector<double> vectDOWNHalfStep_input, char leftOrRightFootStable)
{
	vector<vector<double> > fbVect;
	vector<vector<double> > fbZMPVect;

	produceOneDOWNHalfStep(fbVect, fbZMPVect, incrTime, zc, g, t1, t2, t3, vectDOWNHalfStep_input, leftOrRightFootStable);
	
	int count = -1;
	for(double i = 0.0 ; i < t3 ; i += incrTime)
	{
		count++;

		fb << fbVect[count][0] << " ";
		for(int j = 0; j < 42; j++)
		{
			fb << fbVect[count][j+1] << " " ;
		}
		fb << endl;

		fbZMP << fbZMPVect[count][0] << " " << fbZMPVect[count][1] << " " <<
			fbZMPVect[count][2] << " " << fbZMPVect[count][3] << endl;
		
	}

}

void CnewPGstepStudy::produceOneDOWNHalfStepFeatures(StepFeatures & stepF, double incrTime, double zc, double g, double t1, double t2, double t3, vector<double> vectDOWNHalfStep_input, char leftOrRightFootStable)
{

	vector<double> comTrajX;
	vector<double> zmpTrajX;
	genCOMZMPtrajectory(comTrajX, zmpTrajX, incrTime, zc, g, 0, 0, vectDOWNHalfStep_input[2]/2, t1/2, t1*3/4, t1, t2, t3);

	vector<double> comTrajY;
	vector<double> zmpTrajY;
	genCOMZMPtrajectory(comTrajY, zmpTrajY, incrTime, zc, g, 0, 0, vectDOWNHalfStep_input[3]/2, t1/2, t1*3/4, t1, t2, t3);

	vector<double> footXtraj;
	vector<double> footYtraj;
	int leftRightCoef = 0;	
	if(leftOrRightFootStable == 'L') leftRightCoef = -1; else leftRightCoef = 1;
	genFOOTposition(footXtraj, footYtraj, incrTime, 0, leftRightCoef*vectDOWNHalfStep_input[0], vectDOWNHalfStep_input[2], vectDOWNHalfStep_input[3], 0.02, 0, 0, t1, t2, t3);

	vector<double> footHeight;
	genFOOTupDOWNheight(footHeight, incrTime, vectDOWNHalfStep_input[1], t1, t2, t3);

	vector<double> footOrient;
	genFOOTorientation(footOrient, incrTime, 0, vectDOWNHalfStep_input[4], 0.02, 0, 0, t1, t2, t3);

	vector<double> waistOrient;
	genWAISTorientation(waistOrient, incrTime, 0, vectDOWNHalfStep_input[4], 0.02, 0, 0, t1, t2, t3);

	vector<double> stablefootXtraj;
	vector<double> stablefootYtraj;
	vector<double> stablefootHeight;
	vector<double> stablefootOrient;

	for(double i = 0.0 ; i < t3 ; i += incrTime)
	{
		stablefootXtraj.push_back(0);
		stablefootYtraj.push_back(0);
		stablefootHeight.push_back(0);
		stablefootOrient.push_back(0);
	}	

	stepF.comTrajX = comTrajX;
	stepF.zmpTrajX = zmpTrajX;
	stepF.comTrajY = comTrajY;
	stepF.zmpTrajY = zmpTrajY;

	if(leftOrRightFootStable == 'L') {
	stepF.leftfootXtraj = stablefootXtraj;
	stepF.leftfootYtraj = stablefootYtraj;
	stepF.leftfootHeight = stablefootHeight;
	stepF.leftfootOrient = stablefootOrient;
	stepF.rightfootXtraj = footXtraj;
	stepF.rightfootYtraj = footYtraj;
	stepF.rightfootHeight = footHeight;
	stepF.rightfootOrient = footOrient;
	} else {
	stepF.leftfootXtraj = footXtraj;
	stepF.leftfootYtraj = footYtraj;
	stepF.leftfootHeight = footHeight;
	stepF.leftfootOrient = footOrient;
	stepF.rightfootXtraj = stablefootXtraj;
	stepF.rightfootYtraj = stablefootYtraj;
	stepF.rightfootHeight = stablefootHeight;
	stepF.rightfootOrient = stablefootOrient;
	}

	stepF.waistOrient =  waistOrient;	
	stepF.incrTime = incrTime;
	stepF.zc = zc;
	stepF.size = waistOrient.size();

}

void CnewPGstepStudy::produceSeqSteps(vector<vector<double> > & fb, vector<vector<double> > & fbZMP, double incrTime, double zc, double g, double stepHeight, double t1, double t2, double t3, double t4, double t5, vector<double> vectSteps_input, char leftOrRightFootStable)
{	

	char alternate = leftOrRightFootStable;
	int bigCount=-1;

	fb.clear();
	fbZMP.clear();

	vector<double> tmpNine;	
	tmpNine.resize(9);
	
	tmpNine[6] = (vectSteps_input[0]*2)*cos(-vectSteps_input[5]*PI/180)-(vectSteps_input[1]*2)*sin(-vectSteps_input[5]*PI/180);
	tmpNine[7] = (vectSteps_input[0]*2)*sin(-vectSteps_input[5]*PI/180)+(vectSteps_input[1]*2)*cos(-vectSteps_input[5]*PI/180) ;
	tmpNine[8] = -vectSteps_input[5];

	for(unsigned int i = 1; i <= vectSteps_input.size()/3 - 2; i++) {

		tmpNine[0] = (tmpNine[6]/2)*cos(-tmpNine[8]*PI/180) - (tmpNine[7]/2)*sin(-tmpNine[8]*PI/180);
		tmpNine[1] = (tmpNine[6]/2)*sin(-tmpNine[8]*PI/180) + (tmpNine[7]/2)*cos(-tmpNine[8]*PI/180);
		tmpNine[2] = 0;
		tmpNine[3] = -tmpNine[0];
		tmpNine[4] = -tmpNine[1];
		tmpNine[5] = -tmpNine[8];
		tmpNine[6] = vectSteps_input[3*i+3];
		tmpNine[7] = vectSteps_input[3*i+4];
		tmpNine[8] = vectSteps_input[3*i+5];

		vector<vector<double> > tmp_fb;
		vector<vector<double> > tmp_fbZMP;	

		produceOneStep(tmp_fb, tmp_fbZMP, incrTime, zc, g, stepHeight, t1, t2, t3, t4, t5, tmpNine, alternate);		

		int count = -1;
		for(double u = 0.0 ; u < t5 ; u += incrTime)
			{
			count++;
			bigCount++;

			fb.push_back(tmp_fb[count]);
			fb[bigCount][0] = bigCount * incrTime;
	
			fbZMP.push_back(tmp_fbZMP[count]);
			fbZMP[bigCount][0] = bigCount * incrTime;			

			}		

		if(alternate == 'L') alternate = 'R'; else alternate = 'L';

	}

}

void CnewPGstepStudy::produceSeqSteps(ofstream & fb, ofstream & fbZMP, double incrTime, double zc, double g, double stepHeight, double t1, double t2, double t3, double t4, double t5, vector<double> vectSteps_input, char leftOrRightFootStable)
{
	vector<vector<double> > fbVect;
	vector<vector<double> > fbZMPVect;

	produceSeqSteps(fbVect, fbZMPVect, incrTime, zc, g, stepHeight, t1, t2, t3, t4, t5, vectSteps_input, leftOrRightFootStable);

	for(unsigned int count = 0 ; count < fbVect.size() ; count++)
	{
		fb << fbVect[count][0] << " ";
		for(int j = 0; j < 42; j++)
		{			
			fb << fbVect[count][j+1] << " " ;
		}
		fb << endl;

		fbZMP << fbZMPVect[count][0] << " " << fbZMPVect[count][1] << " " <<
			fbZMPVect[count][2] << " " << fbZMPVect[count][3] << endl;
		
	}

}


void CnewPGstepStudy::produceSeqHalfSteps(vector<vector<double> > & fb, vector<vector<double> > & fbZMP, double incrTime, double zc, double g, double t1, double t2, double t3, vector<double> vectSteps_input, char leftOrRightFootStable)
{	

	char alternate = leftOrRightFootStable;
	int bigCount1=-1;
	int bigCount2=-1;

	fb.clear();
	fbZMP.clear();

	vector<double> tmpEleven;	
	tmpEleven.resize(11);

	vector<double> tmpPart1;
	tmpPart1.resize(8);

	vector<double> tmpPart2;
	tmpPart2.resize(5);	
	
	tmpEleven[8] = (vectSteps_input[0]*2)*cos(-vectSteps_input[5]*PI/180)-(vectSteps_input[1]*2)*sin(-vectSteps_input[5]*PI/180);
	tmpEleven[9] = (vectSteps_input[0]*2)*sin(-vectSteps_input[5]*PI/180)+(vectSteps_input[1]*2)*cos(-vectSteps_input[5]*PI/180) ;
	tmpEleven[10] = -vectSteps_input[5];

	for(unsigned int i = 1; i <= (vectSteps_input.size()-6)/5; i++) {		

		tmpEleven[0] = (tmpEleven[8]/2)*cos(-tmpEleven[10]*PI/180) - (tmpEleven[9]/2)*sin(-tmpEleven[10]*PI/180);
		tmpEleven[1] = (tmpEleven[8]/2)*sin(-tmpEleven[10]*PI/180) + (tmpEleven[9]/2)*cos(-tmpEleven[10]*PI/180);
		tmpEleven[2] = 0;
		tmpEleven[3] = -tmpEleven[0];
		tmpEleven[4] = -tmpEleven[1];
		tmpEleven[5] = -tmpEleven[10];

		tmpEleven[6] = vectSteps_input[5*i+1];
		tmpEleven[7] = vectSteps_input[5*i+2];

		tmpEleven[8] = vectSteps_input[5*i+3];
		tmpEleven[9] = vectSteps_input[5*i+4];
		tmpEleven[10] = vectSteps_input[5*i+5];

		vector<vector<double> > tmp_fb;
		vector<vector<double> > tmp_fbZMP;	

		tmpPart1[0] = tmpEleven[0];
		tmpPart1[1] = tmpEleven[1];
		tmpPart1[2] = tmpEleven[2];
		tmpPart1[3] = tmpEleven[3];
		tmpPart1[4] = tmpEleven[4];
		tmpPart1[5] = tmpEleven[5];
		tmpPart1[6] = tmpEleven[6];
		tmpPart1[7] = tmpEleven[7];	

		tmpPart2[0] = tmpEleven[6];
		tmpPart2[1] = tmpEleven[7];
		tmpPart2[2] = tmpEleven[8];
		tmpPart2[3] = tmpEleven[9];
		tmpPart2[4] = tmpEleven[10];	

		produceOneUPHalfStep(tmp_fb, tmp_fbZMP, incrTime, zc, g, t1, t2, t3, tmpPart1, alternate);

		int count = -1;
		for(double u = 0.0 ; u < t3 ; u += incrTime)
			{
			count++;
			bigCount1++;

			fb.push_back(tmp_fb[count]);
			fb[bigCount1][0] = bigCount1 * incrTime;
	
			fbZMP.push_back(tmp_fbZMP[count]);
			fbZMP[bigCount1][0] = bigCount1 * incrTime;			

			}	

		produceOneDOWNHalfStep(tmp_fb, tmp_fbZMP, incrTime, zc, g, t1, t2, t3, tmpPart2, alternate);

		count = -1;
		for(double u = 0.0 ; u < t3 ; u += incrTime)
			{
			count++;
			bigCount2++;

			fb.push_back(tmp_fb[count]);
			fb[bigCount2][0] = bigCount2 * incrTime;
	
			fbZMP.push_back(tmp_fbZMP[count]);
			fbZMP[bigCount2][0] = bigCount2 * incrTime;			

			}	

		if(alternate == 'L') alternate = 'R'; else alternate = 'L';

	}

}

void CnewPGstepStudy::produceSeqHalfStepsWithStepFeatures(ofstream & fb, ofstream & fbZMP, double incrTime, double zc, double g, double t1, double t2, double t3, vector<double> vectSteps_input, char leftOrRightFootStable)
{	

	char alternate = leftOrRightFootStable;

	vector<StepFeatures> vectSFeat;

	vector<double> tmpEleven;	
	tmpEleven.resize(11);

	vector<double> tmpPart1;
	tmpPart1.resize(8);

	vector<double> tmpPart2;
	tmpPart2.resize(5);	
	
	tmpEleven[8] = (vectSteps_input[0]*2)*cos(-vectSteps_input[5]*PI/180)-(vectSteps_input[1]*2)*sin(-vectSteps_input[5]*PI/180);
	tmpEleven[9] = (vectSteps_input[0]*2)*sin(-vectSteps_input[5]*PI/180)+(vectSteps_input[1]*2)*cos(-vectSteps_input[5]*PI/180) ;
	tmpEleven[10] = -vectSteps_input[5];

	for(unsigned int i = 1; i <= (vectSteps_input.size()-6)/5; i++) {		

		tmpEleven[0] = (tmpEleven[8]/2)*cos(-tmpEleven[10]*PI/180) - (tmpEleven[9]/2)*sin(-tmpEleven[10]*PI/180);
		tmpEleven[1] = (tmpEleven[8]/2)*sin(-tmpEleven[10]*PI/180) + (tmpEleven[9]/2)*cos(-tmpEleven[10]*PI/180);
		tmpEleven[2] = 0;
		tmpEleven[3] = -tmpEleven[0];
		tmpEleven[4] = -tmpEleven[1];
		tmpEleven[5] = -tmpEleven[10];

		tmpEleven[6] = vectSteps_input[5*i+1];
		tmpEleven[7] = vectSteps_input[5*i+2];

		tmpEleven[8] = vectSteps_input[5*i+3];
		tmpEleven[9] = vectSteps_input[5*i+4];
		tmpEleven[10] = vectSteps_input[5*i+5];

		StepFeatures tmp_hstepUp;			
		StepFeatures tmp_hstepDown;

		tmpPart1[0] = tmpEleven[0];
		tmpPart1[1] = tmpEleven[1];
		tmpPart1[2] = tmpEleven[2];
		tmpPart1[3] = tmpEleven[3];
		tmpPart1[4] = tmpEleven[4];
		tmpPart1[5] = tmpEleven[5];
		tmpPart1[6] = tmpEleven[6];
		tmpPart1[7] = tmpEleven[7];	

		tmpPart2[0] = tmpEleven[6];
		tmpPart2[1] = tmpEleven[7];
		tmpPart2[2] = tmpEleven[8];
		tmpPart2[3] = tmpEleven[9];
		tmpPart2[4] = tmpEleven[10];	

		produceOneUPHalfStepFeatures(tmp_hstepUp, incrTime, zc, g, t1, t2, t3, tmpPart1, alternate);

		vectSFeat.push_back(tmp_hstepUp);	

		produceOneDOWNHalfStepFeatures(tmp_hstepDown, incrTime, zc, g, t1, t2, t3, tmpPart2, alternate);

		vectSFeat.push_back(tmp_hstepDown);

		if(alternate == 'L') alternate = 'R'; else alternate = 'L';

	}

	for(unsigned int i = 1; i < vectSFeat.size(); i++) {
		addStepFeaturesWithSlide(vectSFeat[0],vectSFeat[i],0);
	}

	vector<int> none;
	genFullBodyTrajectoryFromStepFeatures(fb, fbZMP, false, NULL, NULL, none, vectSFeat[0]);

}

void CnewPGstepStudy::produceSeqSlidedHalfSteps(ofstream & fb, ofstream & fbZMP, double incrTime, double zc, double g, double t1, double t2, double t3, vector<double> vectSteps_input, char leftOrRightFootStable)
{	

	char alternate = leftOrRightFootStable;

	vector<StepFeatures> vectSFeat;
	vector<double> slideProfile;

	vector<double> tmpEleven;	
	tmpEleven.resize(11);

	vector<double> tmpPart1;
	tmpPart1.resize(8);

	vector<double> tmpPart2;
	tmpPart2.resize(5);	
	
	tmpEleven[8] = (vectSteps_input[0]*2)*cos(-vectSteps_input[5]*PI/180)-(vectSteps_input[1]*2)*sin(-vectSteps_input[5]*PI/180);
	tmpEleven[9] = (vectSteps_input[0]*2)*sin(-vectSteps_input[5]*PI/180)+(vectSteps_input[1]*2)*cos(-vectSteps_input[5]*PI/180) ;
	tmpEleven[10] = -vectSteps_input[5];

	for(unsigned int i = 1; i <= (vectSteps_input.size()-6)/7; i++) {		

		tmpEleven[0] = (tmpEleven[8]/2)*cos(-tmpEleven[10]*PI/180) - (tmpEleven[9]/2)*sin(-tmpEleven[10]*PI/180);
		tmpEleven[1] = (tmpEleven[8]/2)*sin(-tmpEleven[10]*PI/180) + (tmpEleven[9]/2)*cos(-tmpEleven[10]*PI/180);
		tmpEleven[2] = 0;
		tmpEleven[3] = -tmpEleven[0];
		tmpEleven[4] = -tmpEleven[1];
		tmpEleven[5] = -tmpEleven[10];

		slideProfile.push_back(vectSteps_input[7*i-1]);
		tmpEleven[6] = vectSteps_input[7*i];
		tmpEleven[7] = vectSteps_input[7*i+1];

		slideProfile.push_back(vectSteps_input[7*i+2]);
		tmpEleven[8] = vectSteps_input[7*i+3];
		tmpEleven[9] = vectSteps_input[7*i+4];
		tmpEleven[10] = vectSteps_input[7*i+5];

		StepFeatures tmp_hstepUp;			
		StepFeatures tmp_hstepDown;

		tmpPart1[0] = tmpEleven[0];
		tmpPart1[1] = tmpEleven[1];
		tmpPart1[2] = tmpEleven[2];
		tmpPart1[3] = tmpEleven[3];
		tmpPart1[4] = tmpEleven[4];
		tmpPart1[5] = tmpEleven[5];
		tmpPart1[6] = tmpEleven[6];
		tmpPart1[7] = tmpEleven[7];	

		tmpPart2[0] = tmpEleven[6];
		tmpPart2[1] = tmpEleven[7];
		tmpPart2[2] = tmpEleven[8];
		tmpPart2[3] = tmpEleven[9];
		tmpPart2[4] = tmpEleven[10];	

		produceOneUPHalfStepFeatures(tmp_hstepUp, incrTime, zc, g, t1, t2, t3, tmpPart1, alternate);

		vectSFeat.push_back(tmp_hstepUp);	

		produceOneDOWNHalfStepFeatures(tmp_hstepDown, incrTime, zc, g, t1, t2, t3, tmpPart2, alternate);

		vectSFeat.push_back(tmp_hstepDown);

		if(alternate == 'L') alternate = 'R'; else alternate = 'L';

	}

	for(unsigned int i = 1; i < vectSFeat.size(); i++) {
		addStepFeaturesWithSlide(vectSFeat[0],vectSFeat[i],slideProfile[i]);
	}

	vector<int> none;
	genFullBodyTrajectoryFromStepFeatures(fb, fbZMP, false, NULL, NULL, none, vectSFeat[0]);

}


void CnewPGstepStudy::produceSeqHalfSteps(ofstream & fb, ofstream & fbZMP, double incrTime, double zc, double g, double t1, double t2, double t3, vector<double> vectSteps_input, char leftOrRightFootStable)
{
	vector<vector<double> > fbVect;
	vector<vector<double> > fbZMPVect;

	produceSeqHalfSteps(fbVect, fbZMPVect, incrTime, zc, g, t1, t2, t3, vectSteps_input, leftOrRightFootStable);

	for(unsigned int count = 0 ; count < fbVect.size() ; count++)
	{
		fb << fbVect[count][0] << " ";
		for(int j = 0; j < 42; j++)
		{			
			fb << fbVect[count][j+1] << " " ;
		}
		fb << endl;

		fbZMP << fbZMPVect[count][0] << " " << fbZMPVect[count][1] << " " <<
			fbZMPVect[count][2] << " " << fbZMPVect[count][3] << endl;
		
	}

}


void CnewPGstepStudy::produceGlobalLinkedCOMZMP(vector<double> & gCOMx, vector<double> & gCOMy, vector<double> & gZMPx, vector<double> & gZMPy, double incrTime, double zc, double g, double t1, double t2, double t3, double t4, double t5, vector<double> vectSteps_input, char leftOrRightFootStable)
{

	char alternate = leftOrRightFootStable;
	int bigCount=-1;

	double angleCorrection = 0;


	vector<vector<double> > allCOMsX;
	vector<vector<double> > allCOMsY;
	vector<vector<double> > allZMPsX;
	vector<vector<double> > allZMPsY;

	vector<double> tmpNine;	
	tmpNine.resize(9);
	
	tmpNine[6] = (vectSteps_input[0]*2)*cos(-vectSteps_input[5]*PI/180)-(vectSteps_input[1]*2)*sin(-vectSteps_input[5]*PI/180);
	tmpNine[7] = (vectSteps_input[0]*2)*sin(-vectSteps_input[5]*PI/180)+(vectSteps_input[1]*2)*cos(-vectSteps_input[5]*PI/180) ;
	tmpNine[8] = -vectSteps_input[5];

	angleCorrection = tmpNine[8];

	for(unsigned int i = 1; i <= vectSteps_input.size()/3 - 2; i++) {

		tmpNine[0] = (tmpNine[6]/2)*cos(-tmpNine[8]*PI/180) - (tmpNine[7]/2)*sin(-tmpNine[8]*PI/180);
		tmpNine[1] = (tmpNine[6]/2)*sin(-tmpNine[8]*PI/180) + (tmpNine[7]/2)*cos(-tmpNine[8]*PI/180);
		tmpNine[2] = 0;
		tmpNine[3] = -tmpNine[0];
		tmpNine[4] = -tmpNine[1];
		tmpNine[5] = -tmpNine[8];

		angleCorrection -= tmpNine[8];	
	
		tmpNine[6] = vectSteps_input[3*i+3];
		tmpNine[7] = vectSteps_input[3*i+4];
		tmpNine[8] = vectSteps_input[3*i+5];		

		vector<double> comTrajX;
		vector<double> zmpTrajX;

		genCOMZMPtrajectory(comTrajX, zmpTrajX, incrTime, zc, g, 0, 
			tmpNine[0]*cos(-angleCorrection*PI/180) - tmpNine[1]*sin(-angleCorrection*PI/180),
			tmpNine[6]/2*cos(-angleCorrection*PI/180) - tmpNine[7]/2*sin(-angleCorrection*PI/180), 
			t1, t2, t3, t4, t5);

		vector<double> comTrajY;
		vector<double> zmpTrajY;
		genCOMZMPtrajectory(comTrajY, zmpTrajY, incrTime, zc, g, 0, 
			tmpNine[0]*sin(-angleCorrection*PI/180) + tmpNine[1]*cos(-angleCorrection*PI/180), 
			tmpNine[6]/2*sin(-angleCorrection*PI/180) + tmpNine[7]/2*cos(-angleCorrection*PI/180), 
			t1, t2, t3, t4, t5);
		
		allCOMsX.push_back(comTrajX);
		allCOMsY.push_back(comTrajY);
		allZMPsX.push_back(zmpTrajX);
		allZMPsY.push_back(zmpTrajY);

		if(alternate == 'L') alternate = 'R'; else alternate = 'L';

	}

	int numberOfSteps = (vectSteps_input.size()/3 - 2);
	int sizeOneLongStep = allCOMsX[0].size();
	int sizeGap = ((int) ((t1+t5-t3) / incrTime));

	vector<double> longCOMX (numberOfSteps*sizeOneLongStep-(numberOfSteps-1)*sizeGap,0);
	vector<double> longCOMY (numberOfSteps*sizeOneLongStep-(numberOfSteps-1)*sizeGap,0);
	vector<double> longZMPX (numberOfSteps*sizeOneLongStep-(numberOfSteps-1)*sizeGap,0);
	vector<double> longZMPY (numberOfSteps*sizeOneLongStep-(numberOfSteps-1)*sizeGap,0);

	for(unsigned int i = 1; i <= vectSteps_input.size()/3 - 2; i++) {
	
		for(int j = 0 ; j < sizeOneLongStep ; j++)
		{

		longCOMX[(i-1)*sizeOneLongStep+j-(i-1)*sizeGap] += allCOMsX[i-1][j];
		longCOMY[(i-1)*sizeOneLongStep+j-(i-1)*sizeGap] += allCOMsY[i-1][j];
		longZMPX[(i-1)*sizeOneLongStep+j-(i-1)*sizeGap] += allZMPsX[i-1][j];
		longZMPY[(i-1)*sizeOneLongStep+j-(i-1)*sizeGap] += allZMPsY[i-1][j];

		}
	}

	for(unsigned int i = 1; i <= vectSteps_input.size()/3 - 2; i++) {

		for(unsigned int j = i*(sizeOneLongStep - sizeGap)+sizeGap; j<longCOMX.size(); j++) {

		longCOMX[j] += allCOMsX[i-1][sizeOneLongStep-1];
		longCOMY[j] += allCOMsY[i-1][sizeOneLongStep-1];
		longZMPX[j] += allZMPsX[i-1][sizeOneLongStep-1];
		longZMPY[j] += allZMPsY[i-1][sizeOneLongStep-1];

		}

	}

	gCOMx = longCOMX;
	gCOMy = longCOMY;
	gZMPx = longZMPX;
	gZMPy = longZMPY;

}

void CnewPGstepStudy::produceGlobalSeparateCOMZMP(vector<double> & gCOMx, vector<double> & gCOMy, vector<double> & gZMPx, vector<double> & gZMPy, double incrTime, double zc, double g, double t1, double t2, double t3, double t4, double t5, vector<double> vectSteps_input, char leftOrRightFootStable)
{

	char alternate = leftOrRightFootStable;
	int bigCount=-1;

	double angleCorrection = 0;


	vector<vector<double> > allCOMsX;
	vector<vector<double> > allCOMsY;
	vector<vector<double> > allZMPsX;
	vector<vector<double> > allZMPsY;

	vector<double> tmpNine;	
	tmpNine.resize(9);
	
	tmpNine[6] = (vectSteps_input[0]*2)*cos(-vectSteps_input[5]*PI/180)-(vectSteps_input[1]*2)*sin(-vectSteps_input[5]*PI/180);
	tmpNine[7] = (vectSteps_input[0]*2)*sin(-vectSteps_input[5]*PI/180)+(vectSteps_input[1]*2)*cos(-vectSteps_input[5]*PI/180) ;
	tmpNine[8] = -vectSteps_input[5];

	angleCorrection = tmpNine[8];

	for(unsigned int i = 1; i <= vectSteps_input.size()/3 - 2; i++) {

		tmpNine[0] = (tmpNine[6]/2)*cos(-tmpNine[8]*PI/180) - (tmpNine[7]/2)*sin(-tmpNine[8]*PI/180);
		tmpNine[1] = (tmpNine[6]/2)*sin(-tmpNine[8]*PI/180) + (tmpNine[7]/2)*cos(-tmpNine[8]*PI/180);
		tmpNine[2] = 0;
		tmpNine[3] = -tmpNine[0];
		tmpNine[4] = -tmpNine[1];
		tmpNine[5] = -tmpNine[8];

		angleCorrection -= tmpNine[8];	
	
		tmpNine[6] = vectSteps_input[3*i+3];
		tmpNine[7] = vectSteps_input[3*i+4];
		tmpNine[8] = vectSteps_input[3*i+5];		

		vector<double> comTrajX;
		vector<double> zmpTrajX;

		genCOMZMPtrajectory(comTrajX, zmpTrajX, incrTime, zc, g, 0, 
			tmpNine[0]*cos(-angleCorrection*PI/180) - tmpNine[1]*sin(-angleCorrection*PI/180),
			tmpNine[6]/2*cos(-angleCorrection*PI/180) - tmpNine[7]/2*sin(-angleCorrection*PI/180), 
			t1, t2, t3, t4, t5);

		vector<double> comTrajY;
		vector<double> zmpTrajY;
		genCOMZMPtrajectory(comTrajY, zmpTrajY, incrTime, zc, g, 0, 
			tmpNine[0]*sin(-angleCorrection*PI/180) + tmpNine[1]*cos(-angleCorrection*PI/180), 
			tmpNine[6]/2*sin(-angleCorrection*PI/180) + tmpNine[7]/2*cos(-angleCorrection*PI/180), 
			t1, t2, t3, t4, t5);
		
		allCOMsX.push_back(comTrajX);
		allCOMsY.push_back(comTrajY);
		allZMPsX.push_back(zmpTrajX);
		allZMPsY.push_back(zmpTrajY);

		if(alternate == 'L') alternate = 'R'; else alternate = 'L';

	}

	int numberOfSteps = (vectSteps_input.size()/3 - 2);
	int sizeOneLongStep = allCOMsX[0].size();

	vector<double> longCOMX (numberOfSteps*sizeOneLongStep,0);
	vector<double> longCOMY (numberOfSteps*sizeOneLongStep,0);
	vector<double> longZMPX (numberOfSteps*sizeOneLongStep,0);
	vector<double> longZMPY (numberOfSteps*sizeOneLongStep,0);

	int bigcounter=-1;
	for(unsigned int i = 1; i <= vectSteps_input.size()/3 - 2; i++) {
	
		for(int j = 0 ; j < sizeOneLongStep ; j++)
		{

		longCOMX[(i-1)*sizeOneLongStep+j] += allCOMsX[i-1][j];
		longCOMY[(i-1)*sizeOneLongStep+j] += allCOMsY[i-1][j];
		longZMPX[(i-1)*sizeOneLongStep+j] += allZMPsX[i-1][j];
		longZMPY[(i-1)*sizeOneLongStep+j] += allZMPsY[i-1][j];

		}

	}

	for(unsigned int i = 1; i <= vectSteps_input.size()/3 - 2; i++) {

		for(unsigned int j = i*sizeOneLongStep; j<longCOMX.size() ;j++) {

		longCOMX[j] += allCOMsX[i-1][sizeOneLongStep-1];
		longCOMY[j] += allCOMsY[i-1][sizeOneLongStep-1];
		longZMPX[j] += allZMPsX[i-1][sizeOneLongStep-1];
		longZMPY[j] += allZMPsY[i-1][sizeOneLongStep-1];

		}

	}

	gCOMx = longCOMX;
	gCOMy = longCOMY;
	gZMPx = longZMPX;
	gZMPy = longZMPY;

}

void CnewPGstepStudy::plotGlobalLinkedCOMZMP(ofstream & fb, double incrTime, double zc, double g, double t1, double t2, double t3, double t4, double t5, vector<double> vectSteps_input, char leftOrRightFootStable)
{
	vector<double> gCOMx;
	vector<double> gCOMy;
	vector<double> gZMPx;
	vector<double> gZMPy;
		
	produceGlobalLinkedCOMZMP(gCOMx, gCOMy, gZMPx, gZMPy, incrTime, zc, g, t1, t2, t3, t4, t5, vectSteps_input, leftOrRightFootStable);

	for(unsigned int i = 0; i < gCOMx.size(); i++) {

		fb << gCOMx[i] << " " << gCOMy[i] << endl;
	
	}

	fb << endl;
 	fb << endl;

	for(unsigned int i = 0; i < gZMPx.size(); i++) {

		fb << gZMPx[i] << " " << gZMPy[i] << endl;
	
	}
	
}

void CnewPGstepStudy::plotGlobalSeparateCOMZMP(ofstream & fb, double incrTime, double zc, double g, double t1, double t2, double t3, double t4, double t5, vector<double> vectSteps_input, char leftOrRightFootStable)
{
	vector<double> gCOMx;
	vector<double> gCOMy;
	vector<double> gZMPx;
	vector<double> gZMPy;
		
	produceGlobalSeparateCOMZMP(gCOMx, gCOMy, gZMPx, gZMPy, incrTime, zc, g, t1, t2, t3, t4, t5, vectSteps_input, leftOrRightFootStable);

	for(unsigned int i = 0; i < gCOMx.size(); i++) {

		fb << gCOMx[i] << " " << gCOMy[i] << endl;
	
	}

	fb << endl;
 	fb << endl;

	for(unsigned int i = 0; i < gZMPx.size(); i++) {

		fb << gZMPx[i] << " " << gZMPy[i] << endl;
	
	}
	
}




