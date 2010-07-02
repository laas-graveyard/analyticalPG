/*
 *  Copyright AIST-CNRS Joint Robotics Laboratory
 *  Author: Nicolas Perrin
 */

#include "analyticalPG/newPGstepStudy.h"

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
	
  string bufstring;

  CnewPGstepStudy * NPSS;
 
  NPSS = new CnewPGstepStudy(); 

  double body_height;
  char left_or_right;
  int option;
  double t1, t2, t3, t4, t5;
  double incrTime, gravity;
  string gnuplot_steps, gnuplot_comzmpx, gnuplot_comzmpy;

  is >> bufstring >> body_height; 
  is >> bufstring >> left_or_right;
  is >> bufstring >> option;
  is >> bufstring >> incrTime;
  is >> bufstring >> gravity;
  is >> bufstring >> t1;
  is >> bufstring >> t2;
  is >> bufstring >> t3;
  is >> bufstring >> t4;
  is >> bufstring >> t5;
  is >> bufstring >> gnuplot_steps;
  is >> bufstring >> gnuplot_comzmpx;
  is >> bufstring >> gnuplot_comzmpy;	
  is >> bufstring;

  vector<double> stepsVect;

  double z;
  char zc;
  while (1) { 
    is >> zc;    
    if (zc == ')') break;
    is >> z;
    stepsVect.push_back(z);
    cout << z << endl;
  }

  ofstream ofst_steps (gnuplot_steps.c_str());
  ofstream ofst_comzmpx (gnuplot_comzmpx.c_str());
  ofstream ofst_comzmpy (gnuplot_comzmpy.c_str());

  if(option==1) {
	
	NPSS->drawSeqStepFeatures(ofst_steps, incrTime, body_height, gravity, t1, t2, t3, t4, t5, stepsVect, left_or_right, 0.5, -0.5, 1.5, -1.5, 0.5);
// 
	NPSS->plotOneDimensionCOMZMPSeqStep(ofst_comzmpx, 'x', incrTime, body_height, gravity, t1, t2, t3, t4, t5, stepsVect, left_or_right);  

	NPSS->plotOneDimensionCOMZMPSeqStep(ofst_comzmpy, 'y', incrTime, body_height, gravity, t1, t2, t3, t4, t5, stepsVect, left_or_right);  


} else if(option==2) {

	NPSS->drawSeqHalfStepFeatures(ofst_steps, incrTime, body_height, gravity, t1, t2, t3, stepsVect, left_or_right, 0.5, -0.5, 1.5, -1.5, 0.5);
// 
	NPSS->plotOneDimensionCOMZMPSeqHalfStep(ofst_comzmpx, 'x', incrTime, body_height, gravity, t1, t2, t3, stepsVect, left_or_right);  

	NPSS->plotOneDimensionCOMZMPSeqHalfStep(ofst_comzmpy, 'y', incrTime, body_height, gravity, t1, t2, t3, stepsVect, left_or_right); 

} else if(option==3) {

	NPSS->drawSeqSlidedHalfStepFeatures(ofst_steps, incrTime, body_height, gravity, t1, t2, t3, stepsVect, left_or_right, 0.5, -0.5, 1.5, -1.5, 0.5);
// 
	NPSS->plotOneDimensionCOMZMPSeqSlidedHalfStep(ofst_comzmpx, 'x', incrTime, body_height, gravity, t1, t2, t3, stepsVect, left_or_right);  

	NPSS->plotOneDimensionCOMZMPSeqSlidedHalfStep(ofst_comzmpy, 'y', incrTime, body_height, gravity, t1, t2, t3, stepsVect, left_or_right); 

}

   delete NPSS;

}
