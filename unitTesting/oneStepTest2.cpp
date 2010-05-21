/*
 *  Copyright AIST-CNRS Joint Robotics Laboratory
 *  Author: Nicolas Perrin
 */



#include "oneStepStudy/newPGstepStudy.h"

#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>


int main (int argc, char *argv[]) {

  filebuf fb;
  fb.open (argv[1],ios::in);
  istream is(&fb);

  CnewPGstepStudy * NPSS;
 
  NPSS = new CnewPGstepStudy; 

  double body_height;
  char left_or_right;

  //get step(s) values in meters and degrees
  is >> body_height; 
  is >> left_or_right;
  vector<double> footfalls;
  cout << "body height " << body_height << endl;

  double z;
  char zc;
  while (1) { 
    is >> zc;    
    if (zc == ')') break;
    is >> z;
    footfalls.push_back(z);
  }  
  MAL_VECTOR_DIM(footfallz,double,footfalls.size());
  for(unsigned int i = 0; i<footfalls.size(); i++) {
    footfallz[i] = footfalls[i];
  }

  int i0=0;
  int i1=0;
  (NPSS->m_modeAndProperties.pairsOfBodiesToCheck).clear();
  while (1) {
    PaireOfBodies paire0;
    is >> i0;    
    if (i0 == -999) break;
    is >> i1;
    paire0.body1 = i0;
    paire0.body2 = i1;         
    (NPSS->m_modeAndProperties.pairsOfBodiesToCheck).push_back(paire0);
  }   


  NPSS->m_modeAndProperties.coordsConversion = CartesianINPUT;  

  NPSS->m_modeAndProperties.bodyHeight = body_height;

  NPSS->m_modeAndProperties.leftOrRightFirstStable = left_or_right;

  NPSS->m_modeAndProperties.generate = 1;

  NPSS->activateModeAndPropertiesChanges();

  NPSS->setSteps(footfallz);
  cout << footfallz << endl;
  
  //get important files
  string f1, f2, f3, f4, f5, f6, f7;
  
  is >> f1 >> f2 >> f3 >> f4 >> f5 >> f6 >> f7;
  
  NPSS->loadFiles(f1,f2,f3,f4,f5,f6); 
  
  NPSS->buildNecessaryInternalStructures();   


  NPSS->initAndGenOpenHRPFiles(); //initialization and generation of the
			      //OpenHRP files in order to see the step
			      //with OpenHRP

  NPSS->runPGAndEvalTrajectory(); //execution (through the PG) and evaluation of the step

  ofstream ofst ("oneStep.pos");
  ofstream ofst2 ("oneStep.zmp");

  ofstream ofst3 ("comzmp.dat");

/*
  NPSS->produceSeqSteps(
	ofst,
        ofst2, 
	0.005, 
	body_height, 
	9.81, 
	0.15, //stepHeight
	1.3,  //t1,
	1.32, //t2,
	1.68, //t3,
	1.70, //t4,
	2.9,  //t5,
	footfalls,
	left_or_right);

  NPSS->produceSeqHalfSteps(
	ofst,
        ofst2, 
	0.005, 
	body_height, 
	9.81, 
	1.2,  //t1,
	1.22, //t2,
	1.60, //t3,
	footfalls,
	left_or_right);

  NPSS->plotGlobalLinkedCOMZMP(
	ofst3,
	0.005,
	body_height,
	9.81,
	1.3,
	1.32,
	1.68,
	1.70,
	2.9,
	footfalls,
	left_or_right);

  StepFeatures stepF;
  NPSS->produceOneUPHalfStepFeatures(
	stepF, 
	0.005, 
	body_height, 
	9.81, 
	1.2, 
	1.22, 
	1.60, 
	footfalls, 
	left_or_right);

  NPSS->genFullBodyTrajectoryFromStepFeatures(
	ofst,
	ofst2,
	stepF);

*/

  NPSS->produceSeqLinkedHalfSteps(
	ofst,
	ofst2, 
	0.005, 
	body_height, 
	9.81, 
	0.79, 
	0.81, 
	1.60, 
	footfalls, 
	left_or_right,
	-0.0);  


/*
  vector<double> footfalls2;
  footfalls2.resize(5);
  for(unsigned int i = 0; i<5; i++) {
    footfalls2[i] = footfalls[i+6];
  }

  StepFeatures stepF1;

  StepFeatures stepF2;

 NPSS->produceOneUPHalfStepFeatures(
	stepF1, 
	0.005, 
	body_height, 
	9.81, 
	1.2, 
	1.22, 
	1.60, 
	footfalls, 
	left_or_right);

 NPSS->produceOneDOWNHalfStepFeatures(
	stepF2, 
	0.005, 
	body_height, 
	9.81, 
	1.2, 
	1.22, 
	1.60, 
	footfalls2, 
	left_or_right);

 NPSS->addStepFeaturesWithSlide(stepF1, stepF2, 0); 

 NPSS->genFullBodyTrajectoryFromStepFeatures(
	ofst,
	ofst2,
	stepF1);

*/


  delete NPSS;

}
