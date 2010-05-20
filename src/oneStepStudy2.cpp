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

#include <walkGenJrl/PatternGeneratorInterface.h>

#include "CollisionDetection/HumanoidRobotCollisionDetection.h"

#include "oneStepStudy.h"

#include "InverseKinematics.h"

#define DEBUGJOINTS(x)

using namespace std;
using namespace PatternGeneratorJRL;

//be careful with the vector's sizes:
void concatvectors(MAL_VECTOR(,double) & v1,MAL_VECTOR(,double) & v2,MAL_VECTOR(,double) & v3)
{

	for(unsigned int i=0;i<v1.size();i++)
	{
		v3[i]=v1[i];
	}
	for(unsigned int j=v1.size();j<v1.size()+v2.size();j++)
	{
		v3[j]=v2[j-v1.size()];
	}

}


ConeStepStudy::ConeStepStudy()
{
	mp_aPGI = NULL;
	mp_aHRCD = NULL;
}


ConeStepStudy::~ConeStepStudy()
{
	if(mp_aPGI != NULL)
	{
		delete mp_aPGI;
	}
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
void ConeStepStudy::activateModeAndPropertiesChanges()
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
void ConeStepStudy::activateModeAndPropertiesChanges(ModeAndPropertiesEnum Selected)
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


void ConeStepStudy::setSteps(MAL_VECTOR(,double) vect_input)
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


void ConeStepStudy::loadFiles(
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


void ConeStepStudy::buildNecessaryInternalStructures()
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

	mp_aPGI = patternGeneratorInterfaceFactory(mp_HDR);

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


void ConeStepStudy::initAndGenOpenHRPFiles()
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

	if (mp_aPGI != NULL) delete mp_aPGI;
	//  mp_aPGI = new PatternGeneratorInterface(strm);
	mp_aPGI = 0;
	if (mp_HDR!=0)
		mp_aPGI = patternGeneratorInterfaceFactory(mp_HDR);

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
		replace_value_vector3(mp_openHRPxml.c_str(),"WAIST.translation", 0.0, 0.0, mp_bodyHeight + 0.105);
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

	//string search2 =	"#MARKPOINT2\n  walk.sendMsg(\":stepseq";

	MAL_VECTOR_DIM(sseq,double,mp_stepsVector.size()-3);
	sseq[0] = foot_stay_x;
	sseq[1] = foot_stay_y;
	sseq[2] = foot_stay_theta*180/PI;

	for(unsigned int i=3; i<mp_stepsVector.size()-3; i++)
	{
		sseq[i] = mp_stepsVector[i+3];
	}
	/*
	if(mp_generate) {

	  replace_value_vectorN(sseq.size(), mp_pythonScript.c_str(), search2.c_str(), sseq, "");

	}*/

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

		mp_aPGI->SetCurrentJointValues(mp_jointsRadValues);

		//________________________

	}

	mp_NbOfIt=0;

	//set the goal for the PG:

	commandToDo(*mp_aPGI, sseq);

	if(mp_generate)
	{

		while(
			mp_aPGI->RunOneStepOfTheControlLoop(
			mp_currentConfiguration,
			mp_currentVelocity,
			mp_currentAcceleration,
			mp_ZMPTarget
			)
			&& mp_NbOfIt<2
			)
		{

			mp_NbOfIt++;
			mp_previousConfiguration = mp_currentConfiguration;
			mp_previousVelocity = mp_currentVelocity;
			mp_previousAcceleration = mp_currentAcceleration;

			if (mp_NbOfIt == 1)
			{

				for(int i=0; i<40; i++)
				{
					joints[i]=mp_currentConfiguration[i+6]*180/PI;
				}
				for(int i=0; i<7; i++)
				{
					qArmr[i]=mp_currentConfiguration[i+16+6];
					qArml[i]=mp_currentConfiguration[i+23+6];
				}

				//replace_value_vectorN(40 , mp_pythonScript.c_str() , search.c_str() , joints , "");

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
	}

}


void ConeStepStudy::runPGAndEvalTrajectory()
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
	double distZMP = 0.0;
	double distZMPx = 0.0;
	double distZMPy = 0.0;
	double distZMPmax = 0.0;

	double distLimits=10000.0;

	double rmin=10000.0;

	FootAbsolutePosition LeftFootPosition;
	FootAbsolutePosition RightFootPosition;
	COMPosition_s COMPosition;

	//ofstream logRF("logRightFoot.dat");
	//logRF << setprecision (5) << " time ----" << " x ----" << " y ----" << " z ----" << " theta" << endl;
	//ofstream logLF("logLeftFoot.dat");
	//logLF << setprecision (5) << " time ----" << " x ----" << " y ----" << " z ----" << " theta" << endl;

	int trigger = 3;
	int support_type = 0;		 //0 for double support, 1 for simple support
	int support_type_past = 0;

	filebuf fpbuf;
	fpbuf.open ("oneStep.pos",ios::out);
	ostream fp(&fpbuf);

	filebuf fp2buf;
	fp2buf.open ("oneStep.zmp",ios::out);
	ostream fp2(&fp2buf);

	while(
		mp_aPGI->RunOneStepOfTheControlLoop(
		mp_currentConfiguration,
		mp_currentVelocity,
		mp_currentAcceleration,
		mp_ZMPTarget,
		COMPosition,
		LeftFootPosition,
		RightFootPosition
		)
		)
	{

		//---------+++++++++++++++++++++++++++++++++++++++++++++++++++++
		mp_NbOfIt++;
		mp_previousConfiguration = mp_currentConfiguration;
		mp_previousVelocity = mp_currentVelocity;
		mp_previousAcceleration = mp_currentAcceleration;

		if (mp_NbOfIt % 20 == 6 && mp_NbOfIt > 20 )
		{

			mp_HDR->currentConfiguration(mp_currentConfiguration);
			mp_HDR->currentVelocity(mp_currentVelocity);
			mp_HDR->currentAcceleration(mp_currentAcceleration);
			mp_HDR->computeForwardKinematics();

		}

		if (mp_NbOfIt % 20 == 7 && mp_NbOfIt > 20 )
		{

			mp_HDR->currentConfiguration(mp_currentConfiguration);
			mp_HDR->currentVelocity(mp_currentVelocity);
			mp_HDR->currentAcceleration(mp_currentAcceleration);
			mp_HDR->computeForwardKinematics();

			//mp_aPGI->SetCurrentJointValues(mp_jointsRadValues);

			//---------++++++++++++++++++++++++++++++++++++++++++++++++
			//Joints limits...(not the arms)

			double angdiff=0.0;
			double valueNormal;
			for(int i=0; i<12; i++)
			{
				valueNormal = mp_currentConfiguration[i+6]*180/PI;
				angdiff = min(max(valueNormal-minJointsLimits[i],(double) 0), max(maxJointsLimits[i]-valueNormal,(double) 0));
				if(angdiff < distLimits)
				{
					distLimits=angdiff;

				}

				DEBUGJOINTS(     if(angdiff <= 0.001) {cout << "!!! Joint no. " << i << " beyond limits !!!" << endl;}
				)

			}

			//---------++++++++++++++++++++++++++++++++++++++++++++++++
			//ZMP...

			//MAL_S3_VECTOR(valCOM,double);
			//valCOM = (mp_aDMB)->getPositionCoM();
			//cout << valCOM[0] << " ; " << valCOM[1] << endl;

			MAL_S3_VECTOR(valZ,double);
			valZ = mp_HDR->zeroMomentumPoint();
			//cout << valZ[0] << " " << valZ[1] << endl;

			fp << 3 + mp_NbOfIt * 0.005 << " ";
			for(int i = 6; i < 46; i++)
			{
				if(abs(mp_currentConfiguration[i]) < 0.000000001) fp << 0 << " " ;
				else fp << mp_currentConfiguration[i] << " " ;

			}
			fp << "0 " << "0 " << endl;

			fp2 << 3 + mp_NbOfIt * 0.005 << " ";
			for(int i = 0; i < 3; i++)
			{
				if(abs(valZ[i]) < 0.000000001) fp2 << -0.645 << " " ;
				else fp2 << valZ[i] << " " ;
			}
			fp2 << endl;

			matrix4d aCurrentM;
			matrix4d aCurrentM2;
			double dist, distx, disty;

			//we get the absolute position of the left foot;
			aJoint = mp_HDR->leftAnkle();
			aCurrentM = aJoint->currentTransformation();
			//for(int i=0;i<3;i++) {
			//  cout << MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,i,3) << " ";
			//  }
			//cout << endl;

			//we get the absolute position of the right foot;
			aJoint = mp_HDR->rightAnkle();
			aCurrentM2 = aJoint->currentTransformation();
			//for(int i=0;i<3;i++) {
			//  cout << MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM2,i,3) << " ";
			// }
			//cout << endl;

			//first case: both feet on the ground:
			if (MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,2,3) <= 0.106 && MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM2,2,3) <= 0.106)
			{
				support_type = 0;
				//we just check the distance between the zmp and the segment
				//between the two feet

				//small calculation for the distance between a point and a segment============
				double A = valZ[0] - MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,0,3);
				double B = valZ[1] - MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,1,3);
				double C = MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM2,0,3) - MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,0,3);
				double D = MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM2,1,3) - MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,1,3);

				double dot = A * C + B * D;
				double len_sq = C * C + D * D;
				double param = dot / len_sq;

				double xx,yy;

				if(param < 0)
				{
					xx = MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,0,3);
					yy = MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,1,3);
				}
				else if(param > 1)
				{
					xx = MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,0,3);
					yy = MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,1,3);
				}
				else
				{
					xx = MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,0,3) + param * C;
					yy = MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,1,3) + param * D;
				}

				dist = sqrt ((valZ[0]-xx)*(valZ[0]-xx) + (valZ[1]-yy)*(valZ[1]-yy));
				distx = valZ[0]-xx;
				disty = valZ[1]-yy;

				//============================================================================

			}
			else
			{
				support_type = 1;//simple support

				//cout << valZ[1] << " ; " << MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,1,3) << " # " << MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM2,1,3) << endl;

				if (MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,2,3) > 0.1055)
				{
					//left foot up
					dist =  sqrt((valZ[0] - MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM2,0,3))*(valZ[0] - MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM2,0,3))  +   (valZ[1] - MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM2,1,3))*(valZ[1] - MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM2,1,3)));

					distx = abs(valZ[0] - MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM2,0,3));
					disty = abs(valZ[1] - MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM2,1,3));
				}
				else
				{
					//right foot up
					dist =  sqrt((valZ[0] - MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,0,3))*(valZ[0] - MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,0,3))  +   (valZ[1] - MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,1,3))*(valZ[1] - MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,1,3)));
					distx = abs(valZ[0] - MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,0,3));
					disty = abs(valZ[1] - MAL_S4x4_MATRIX_ACCESS_I_J(aCurrentM,1,3));
				}
			}

			trigger++;

								 //either one foot just
			if(support_type != support_type_past)
			{
				//rose, or one foot
				//just landed
				trigger = 0;
			}

			support_type_past = support_type;

			if(trigger > 1)
			{
				counter++;
				distZMP += dist*dist;
				distZMPx += distx*distx;
				distZMPy += disty*disty;
				if (dist > distZMPmax)
				{
					distZMPmax=dist;
				}
			}

			//---------++++++++++++++++++++++++++++++++++++++++++++++++
			//Collisions...

			//cout << ".\n";

			for(unsigned int i=0;i<mp_vectOfBodies.size();i++)
			{

				aJoint = aVecOfJoints[mp_vectOfBodies[i]];

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
				mp_aHRCD->SetBodyPose(mp_vectOfBodies[i]+1,aX);

			}

			aX = VclipPose::ID;
			mp_aHRCD->SetBodyPose(0,aX);

			// Check the possible collision.
			r = mp_aHRCD->ComputeSelfCollision();

			if (r<rmin) rmin=r;

		}

		//---------+++++++++++++++++++++++++++++++++++++++++++++++++++++

		//logRF << fixed << setprecision (5) << RightFootPosition.time << " " << RightFootPosition.x << " " << RightFootPosition.y << " " << RightFootPosition.z << " " << RightFootPosition.theta << endl;
		//logLF << fixed << setprecision (5) << LeftFootPosition.time << " " << LeftFootPosition.x << " " << LeftFootPosition.y << " " << LeftFootPosition.z << " " << LeftFootPosition.theta << endl;

	}

	//------------------------------------------------------

	double InitDistCol, DistMinJointsLim, DistMinSelfCol, VarZMPdev, VarZMPdevX, VarZMPdevY, ZMPdevMax;
	int PGit;

	InitDistCol = rinit*100;

	DistMinJointsLim = distLimits;
	if(DistMinJointsLim > 999) DistMinJointsLim = 0;

	DistMinSelfCol = rmin*100;
	if(DistMinSelfCol > 999*100) DistMinSelfCol = 0;

	VarZMPdev = distZMP/(counter-1.0)*100*100;

	VarZMPdevX = distZMPx/(counter-1.0)*100*100;

	VarZMPdevY = distZMPy/(counter-1.0)*100*100;

	ZMPdevMax = distZMPmax*100;

	PGit = mp_NbOfIt;

	m_lastRunInfo.possibleError = 0;

	//N.B.   (nan != nan) is evaluated to true !
	if(InitDistCol == InitDistCol && !std::isinf(InitDistCol))
	{
		m_lastRunInfo.initDistCol = InitDistCol;
	}
	else { m_lastRunInfo.initDistCol = -999; m_lastRunInfo.possibleError = 1; }

	if(DistMinJointsLim == DistMinJointsLim && !std::isinf(DistMinJointsLim))
	{
		m_lastRunInfo.distMinJointsLim = DistMinJointsLim;
	}
	else { m_lastRunInfo.distMinJointsLim = -999; m_lastRunInfo.possibleError = 1; }

	if(DistMinSelfCol == DistMinSelfCol && !std::isinf(DistMinSelfCol))
	{
		m_lastRunInfo.distMinSelfCol = DistMinSelfCol;
	}
	else { m_lastRunInfo.distMinSelfCol = -999; m_lastRunInfo.possibleError = 1; }

	if(VarZMPdev == VarZMPdev && !std::isinf(VarZMPdev))
	{
		m_lastRunInfo.varZMPdev = VarZMPdev;
	}
	else { m_lastRunInfo.varZMPdev = 999; m_lastRunInfo.possibleError = 1; }

	if(VarZMPdevX == VarZMPdevX && !std::isinf(VarZMPdevX))
	{
		m_lastRunInfo.varZMPdevX = VarZMPdevX;
	}
	else { m_lastRunInfo.varZMPdevX = 999; m_lastRunInfo.possibleError = 1; }

	if(VarZMPdevY == VarZMPdevY && !std::isinf(VarZMPdevY))
	{
		m_lastRunInfo.varZMPdevY = VarZMPdevY;
	}
	else { m_lastRunInfo.varZMPdevY = 999; m_lastRunInfo.possibleError = 1; }

	if( ZMPdevMax == ZMPdevMax && !std::isinf(ZMPdevMax))
	{
		m_lastRunInfo.ZMPdevMax = ZMPdevMax;
	}
	else { m_lastRunInfo.ZMPdevMax = 999; m_lastRunInfo.possibleError = 1; }

	m_lastRunInfo.PGit = PGit;
	m_lastRunInfo.lastConfig = mp_currentConfiguration;

}


void ConeStepStudy::printResults()
{
	cout << "Initial distance to self-collision: " << m_lastRunInfo.initDistCol << " cm" << endl;
	cout << "Distance min. to joints limits for legs: " << m_lastRunInfo.distMinJointsLim << " degrees" << endl;
	cout << "Distance min. to self-collision: " <<  m_lastRunInfo.distMinSelfCol << " cm" << endl;
	cout << "Variance of ZMP's deviation: " << m_lastRunInfo.varZMPdev << " cm^2" << endl;
	cout << "Variance of ZMP's x-deviation during one-foot-up phase: " << m_lastRunInfo.varZMPdevX << " cm^2" << endl;
	cout << "Variance of ZMP's y-deviation during one-foot-up phase: " << m_lastRunInfo.varZMPdevY << " cm^2" << endl;
	cout << "ZMP's max. deviation: " << m_lastRunInfo.ZMPdevMax << " cm" << endl;
	cout << "Number of PG iterations: " <<  m_lastRunInfo.PGit << endl;
}


void ConeStepStudy::commandToDo(
PatternGeneratorInterface & aPGI_,
MAL_VECTOR(,double) input_vector
)
{

	const char lBuffer[13][256] =
	{
		":samplingperiod 0.005",
		":previewcontroltime 1.6",
		":comheight 0.8078",
		":omega 0.0",
		":stepheight 0.07",
		":singlesupporttime 0.78",
		":doublesupporttime 0.02",
		":armparameters 0.5",
		":LimitsFeasibility 0.0",
		":ZMPShiftParameters 0.015 0.015 0.015 0.015",
		":TimeDistributionParameters 2.0 3.7 1.7 3.0",
		":UpperBodyMotionParameters -0.1 -1.0 0.0",
		":SetAlgoForZmpTrajectory Kajita"
	};

	// ":samplingperiod 0.005",
	//      ":previewcontroltime 1.6",
	//      ":comheight 0.8078",
	//      ":walkmode 0",
	//      ":omega 0.0",
	//      ":stepheight 0.07",
	//      ":singlesupporttime 0.78",
	//      ":doublesupporttime 0.02",
	//      ":armparameters 0.5",

	for(int i=0;i<10;i++)
	{
		std::istringstream strm(lBuffer[i]);
		aPGI_.ParseCmd(strm);
	}

	ostringstream command;
	command << ":stepseq";
	for(unsigned int i=0 ; i<input_vector.size() ; i++)
	{
		command << " " << input_vector[i];
	}

	//   istringstream strm2(command.str());
	//   aPGI_.ParseCmd(strm2);

	istringstream strm2(":stepseq 0.0 -0.105 0.0 \
                     0.2 0.21 0.0  \
                     0.2 -0.21 0.0 \
                     0.2 0.21 0.0  \
                     0.2 -0.21 0.0 \
                     0.2 0.21 0.0  \
                     0.2 -0.21 0.0 \
                     0.2 0.21 0.0  \
                     0.2 -0.21 0.0 \
                     0.2 0.21 0.0  \
                     0.2 -0.21 0.0 \
                     0.2 0.21 0.0  \
                     0.2 -0.21 0.0 \
                     0.2 0.21 0.0  \
                     0.2 -0.21 0.0 \
                     0.0 0.21 0.0");
	aPGI_.ParseCmd(strm2);

}


void ConeStepStudy::drawSteps(ofstream & fb)
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
