/*
 *  Copyright AIST-CNRS Joint Robotics Laboratory
 *  Authors: Nicolas Perrin, Olivier Stasse
 */

#include "oneStepStudy/oneStepStudy.h"

#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>

int main (int argc, char *argv[])
{

	filebuf fb;
	fb.open (argv[1],ios::in);
	istream is(&fb);

	ConeStepStudy * OSS;

	OSS = new ConeStepStudy;

	double body_height;
	char left_or_right;

	//get step(s) values in meters and degrees
	is >> body_height;
	is >> left_or_right;
	vector<double> footfalls;
	cout << "body height " << body_height << endl;

	double z;
	char zc;
	while (1)
	{
		is >> zc;
		if (zc == ')') break;
		is >> z;
		footfalls.push_back(z);
	}
	MAL_VECTOR_DIM(footfallz,double,footfalls.size());
	for(unsigned int i = 0; i<footfalls.size(); i++)
	{
		footfallz[i] = footfalls[i];
	}

	int i0=0;
	int i1=0;
	(OSS->m_modeAndProperties.pairsOfBodiesToCheck).clear();
	while (1)
	{
		PaireOfBodies paire0;
		is >> i0;
		if (i0 == -999) break;
		is >> i1;
		paire0.body1 = i0;
		paire0.body2 = i1;
		(OSS->m_modeAndProperties.pairsOfBodiesToCheck).push_back(paire0);
	}

	OSS->m_modeAndProperties.coordsConversion = CartesianINPUT;

	OSS->m_modeAndProperties.bodyHeight = body_height;

	OSS->m_modeAndProperties.leftOrRightFirstStable = left_or_right;

	OSS->m_modeAndProperties.generate = 1;

	OSS->activateModeAndPropertiesChanges();

	OSS->setSteps(footfallz);
	cout << footfallz << endl;

	//get important files
	string f1, f2, f3, f4, f5, f6, f7;

	is >> f1 >> f2 >> f3 >> f4 >> f5 >> f6 >> f7;

	OSS->loadFiles(f1,f2,f3,f4,f5,f6);

	OSS->buildNecessaryInternalStructures();

	//initialization and generation of the
	OSS->initAndGenOpenHRPFiles();
	//OpenHRP files in order to see the step
	//with OpenHRP

        //execution (through the PG) and evaluation of the step
	//OSS->runPGAndEvalTrajectory();

	//OSS->printResults();

	//ofstream ofst ("gnup.dat");

	//OSS->drawSteps(ofst);

	//delete OSS;

	//unsigned int TestProfil=PROFIL_STRAIGHT_WALKING;
	//  unsigned int TestProfil=PROFIL_ANALYTICAL_ONLINE_WALKING;

	// 	string VRMLPath = "/home/perrin/src/OpenHRP/OpenHRP-3.0.5/Controller/IOserver/robot/HRP2JRL/model/";
	// 	string VRMLFileName = "HRP2JRLmain.wrl";
	// 	string SpecificitiesFileName = "/home/perrin/src/OpenHRP/OpenHRP-3.0.5/Controller/IOserver/robot/HRP2JRL/etc/HRP2Specificities.xml";
	// 	string LinkJointRank = "/home/perrin/src/OpenHRP/OpenHRP-3.0.5/Controller/IOserver/robot/HRP2JRL/etc/HRP2LinkJointRank.xml";

	string VRMLPath = f2;
	string VRMLFileName = f3;
	string SpecificitiesFileName = f4;
	string LinkJointRank = f5;

	// 	if (argc!=5)
	// 	{
	// 		const char *openhrphome="OPENHRPHOME";
	// 		char *value = 0;
	// 		value = getenv(openhrphome);
	// 		if (value==0)
	// 		{
	// 			cerr << " This program takes 4 arguments: " << endl;
	// 			cerr << "./TestFootPrintPGInterface \ 
	//                          PATH_TO_VRML_FILE	   \ 
	//                          VRML_FILE_NAME		   \ 
	//                          PATH_TO_SPECIFICITIES_XML \ 
	//                          LINK_JOINT_RANK" << endl;
	// 			exit(-1);
	// 		}
	// 		else
	// 		{
	// 			VRMLPath=value;
	// 			VRMLPath+="/Controller/IOserver/robot/HRP2JRL/model/";
	// 			VRMLFileName="HRP2JRLmain.wrl";
	// 			SpecificitiesFileName = value;
	// 			SpecificitiesFileName +="/Controller/IOserver/robot/HRP2JRL/etc/";
	// 			SpecificitiesFileName += "HRP2Specificities.xml";
	// 			LinkJointRank = value;
	// 			LinkJointRank += "/Controller/IOserver/robot/HRP2JRL/etc/";
	// 			LinkJointRank += "HRP2LinkJointRank.xml";
	//
	// 			if (argc==2)
	// 			{
	// 				TestProfil=atoi(argv[1]);
	// 				cout << "Profil: " << ProfilesNames[TestProfil] << endl;
	// 			}
	//
	// 		}
	// 	}
	// 	else
	// 	{
	// 		VRMLPath=argv[1];
	// 		VRMLFileName=argv[2];
	// 		SpecificitiesFileName = argv[3];
	// 		LinkJointRank = argv[4];
	// 	}

	// Creating the humanoid robot.
	CjrlHumanoidDynamicRobot * aHDR = 0;
	dynamicsJRLJapan::ObjectFactory aRobotDynamicsObjectConstructor;

	/*#ifndef WITH_HRP2DYNAMICS
	aHDR = aRobotDynamicsObjectConstructor.createHumanoidDynamicRobot();
	#else*/
	Chrp2OptHumanoidDynamicRobot *aHRP2HDR= new Chrp2OptHumanoidDynamicRobot(&aRobotDynamicsObjectConstructor);
	aHDR = aHRP2HDR;
	//#endif

	// Parsing the file.
	string RobotFileName = VRMLPath + VRMLFileName;
	dynamicsJRLJapan::parseOpenHRPVRMLFile(*aHDR,RobotFileName,
		LinkJointRank,
		SpecificitiesFileName);

	// Create Pattern Generator Interface
	PatternGeneratorInterface * aPGI;
	aPGI = patternGeneratorInterfaceFactory(aHDR);

	bool conversiontoradneeded=true;

	//  double * dInitPos = InitialPoses[INTERACTION_2008];
	//double  *dInitPos = InitialPoses[HALF_SITTING_2008];
// 	double  dInitPos[40] = { 
// 	  0.0, 0.0, -26.0, 50.0, -24.0, 0.0, 
// 	  0.0, 0.0, -26.0, 50.0, -24.0, 0.0,  // legs
// 	  
// 	  0.0, 0.0, 0.0, 0.0, // chest and head
// 	  
// 	  15.0, -10.0, 0.0, -30.0, 0.0, 0.0,  10.0, // right arm
// 	  15.0,  10.0, 0.0, -30.0, 0.0, 0.0,  10.0, // left arm 
// 
// 	  -10.0, 10.0, -10.0, 10.0, -10.0,  // right hand
// 	  -10.0, 10.0, -10.0, 10.0, -10.0  // left hand
// 	};

	double dInitPos[40];	
	MAL_VECTOR_DIM(dIPos,double,40);
	dIPos = OSS->getRadJointValues();
	for(int i = 0; i<40 ; i++) { dInitPos[i]=dIPos[i]*180/M_PI;}
     	
	// This is a vector corresponding to the DOFs actuated of the robot.
	MAL_VECTOR_DIM(InitialPosition,double,30);
	//MAL_VECTOR_DIM(CurrentPosition,double,40);
	if (conversiontoradneeded)
		for(unsigned int i=0;i<MAL_VECTOR_SIZE(InitialPosition);i++)
		  {InitialPosition(i) = dInitPos[i]*M_PI/180.0; cout << InitialPosition[i] << endl; }
	else
		for(unsigned int i=0;i<MAL_VECTOR_SIZE(InitialPosition);i++)
			InitialPosition(i) = dInitPos[i];

	aPGI->SetCurrentJointValues(InitialPosition);

	// Specify the walking mode: here the default one.
	istringstream strm2(":walkmode 0");
	aPGI->ParseCmd(strm2);

	// This is a vector corresponding to ALL the DOFS of the robot:
	// free flyer + actuated DOFS.
	MAL_VECTOR_DIM(CurrentConfiguration,double,36);
	MAL_VECTOR_DIM(CurrentVelocity,double,36);
	MAL_VECTOR_DIM(CurrentAcceleration,double,36);
	MAL_VECTOR_DIM(PreviousConfiguration,double,36) ;
	MAL_VECTOR_DIM(PreviousVelocity,double,36);
	MAL_VECTOR_DIM(PreviousAcceleration,double,36);
	for(int i=0;i<6;i++)
	{
		PreviousConfiguration[i] =
			PreviousVelocity[i] =
			PreviousAcceleration[i] = 0.0;
	}

	for(int i=6;i<36;i++)
	{
		PreviousConfiguration[i] = InitialPosition[i-6];
		PreviousVelocity[i] =
			PreviousAcceleration[i] = 0.0;
	}

	MAL_VECTOR_DIM(ZMPTarget,double,3);

	//COMPosition CurrentWaistPosition;
	struct timeval begin,end,startingtime;
	unsigned long int NbOfIt=0, NbOfItToCompute=0;

	COMPosition finalCOMPosition;
	FootAbsolutePosition LeftFootPosition;
	FootAbsolutePosition RightFootPosition;

	unsigned int PGIInterface = 0;

	double TimeProfile[200*620];
	bool bTimeProfile=true;
	double TimeProfileTS[200*620];
	unsigned int TimeProfileIndex = 0;
	unsigned int TimeProfileUpperLimit=200*620;

	double totaltime=0,maxtime=0;
	double totaltimemodif=0, timemodif = 0;
	double totaltimeinplanning=0;
	unsigned long int nbofmodifs=0;

	gettimeofday(&startingtime,0);

	gettimeofday(&begin,0);

	const char lBuffer[12][256] =
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
		":UpperBodyMotionParameters -0.1 -1.0 0.0"
	};

	for(int i=0;i<9;i++)
	{
		std::istringstream strm(lBuffer[i]);
		aPGI->ParseCmd(strm);
	}
	// Evaluate current state of the robot in the PG.
	COMPosition   lStartingCOMPosition;
	MAL_S3_VECTOR(,double)  lStartingZMPPosition;
	MAL_VECTOR(,double)  lStartingWaistPose;
	FootAbsolutePosition  InitLeftFootAbsPos;
	FootAbsolutePosition  InitRightFootAbsPos;

	aPGI->EvaluateStartingState(lStartingCOMPosition,
		lStartingZMPPosition,
		lStartingWaistPose,
		InitLeftFootAbsPos,
		InitRightFootAbsPos);

	cout << "InitialPosition : " << InitialPosition * 180/M_PI<< endl;
	cout << "COMinit: " << lStartingCOMPosition.x[0] << " " 
	     << lStartingCOMPosition.y[0] << " " 
	     << lStartingCOMPosition.z[0] << endl;
	cout << "WaistStartingWaistPose: " << lStartingWaistPose(0) << " " 
	     << lStartingWaistPose(1) << " " 
	     << lStartingWaistPose(2) << endl;
	
	istringstream strm3(":SetAlgoForZmpTrajectory Kajita");
	aPGI->ParseCmd(strm3);


// 	istringstream strm4(":stepseq 0.0 -0.105 \
//                      0.0 0.2  0.21 0.0 \
//                      0.2 -0.21 0.0 \
//                      0.2  0.21 0.0 \
//                      0.2 -0.21 0.0 \
//                      0.2  0.21 0.0 \
//                      0.0 -0.21 0.0");
// 	aPGI->ParseCmd(strm4);

	MAL_VECTOR(,double) seqToPlay = OSS->getSteps();
  	ostringstream command;
  	command << ":stepseq";
  	for(unsigned int i=0 ; i<seqToPlay.size() ; i++) {
    		command << " " << seqToPlay[i];	
  	}    

	istringstream strm4(command.str());
	aPGI->ParseCmd(strm4);



	gettimeofday(&end,0);
	double ltime = end.tv_sec-begin.tv_sec + 0.000001 * (end.tv_usec - begin.tv_usec);
	totaltimeinplanning+=ltime;

	filebuf fpbuf;
	fpbuf.open ("oneStep.pos",ios::out);
	ostream fp(&fpbuf);

	filebuf fp2buf;
	fp2buf.open ("oneStep.zmp",ios::out);
	ostream fp2(&fp2buf);

	bool ok = true;
	while(ok)
	{

		gettimeofday(&begin,0);

		if (PGIInterface==0)
		{
			ok = aPGI->RunOneStepOfTheControlLoop(CurrentConfiguration,
				CurrentVelocity,
				CurrentAcceleration,
				ZMPTarget,
				finalCOMPosition,
				LeftFootPosition,
				RightFootPosition);
		}
		else if (PGIInterface==1)
		{
			ok = aPGI->RunOneStepOfTheControlLoop(CurrentConfiguration,
				CurrentVelocity,
				CurrentAcceleration,
				ZMPTarget);
		}

		gettimeofday(&end,0);
		double ltime = end.tv_sec-begin.tv_sec + 0.000001 * (end.tv_usec - begin.tv_usec);
		if (maxtime<ltime)
			maxtime = ltime;
		NbOfIt++;

		if (ltime>0.000300)
		{
			totaltime += ltime;
			NbOfItToCompute++;
		}

		PreviousConfiguration = CurrentConfiguration;
		PreviousVelocity = CurrentVelocity;
		PreviousAcceleration = CurrentAcceleration;

		//MAL_S3_VECTOR(valZ,double);
		//valZ = aHDR->zeroMomentumPoint();
		//cout << valZ[0] << " " << valZ[1] << endl;

		fp << 1 + NbOfIt * 0.005 << " ";
		for(int i = 6; i < 36; i++)
		{
		  if(abs(PreviousConfiguration[i]) < 0.000000001) fp << 0 << " " ;
		  else fp << PreviousConfiguration[i] << " " ;
		  
		  if ((i==23)  || (i==29))
		    {
		      fp << 0.0 << " ";
		    }
		  
		}
		fp << "0 " << "0 "<< "0 "<< "0 "<< "0 "<< "0 "<< "0 "<< "0 "<< "0 "<< "0 ";
		fp << endl;

		fp2 << 1 + NbOfIt * 0.005 << " ";
		for(int i = 0; i < 2; i++)
		{
			if(abs(ZMPTarget[i]) < 0.000000001) fp2 << 0 << " " ;
			else fp2 << ZMPTarget[i] << " " ;
		}
		fp2 << -0.645 << endl;

		timemodif =0;

		TimeProfile[TimeProfileIndex] = ltime + timemodif;
		TimeProfileTS[TimeProfileIndex] = begin.tv_sec + 0.000001 * begin.tv_usec;
		TimeProfileIndex++;
		if (TimeProfileIndex>TimeProfileUpperLimit)
			TimeProfileIndex = 0;

	}

	if (bTimeProfile)
	{
		ofstream lProfileOutput("TimeProfile.dat",ofstream::out);
		double dST = startingtime.tv_sec + 0.000001 * startingtime.tv_usec;
		for(unsigned int i=0;i<TimeProfileIndex;i++)
			lProfileOutput << " " <<  TimeProfileTS[i] - dST
				<< " " << TimeProfile[i] << std::endl;

		lProfileOutput.close();
	}

	delete aPGI;
	delete OSS;

	cout << "Number of iterations " << NbOfIt << " " << NbOfItToCompute << endl;
	cout << "Time consumption: " << (double)totaltime/(double)NbOfItToCompute
		<< " max time: " << maxtime <<endl;
	cout << "Time on ZMP ref planning (Kajita policy): "
		<< totaltimeinplanning<< " "
		<< totaltimeinplanning*4/(double)NbOfIt<< endl;

}
