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

  CnewPGstepStudy * NPSS;
 
  NPSS = new CnewPGstepStudy(); 

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
    cout << z << endl;
  }

  ofstream ofst ("gnuSteps.dat");

  NPSS->drawSteps(ofst, footfalls, left_or_right, false, false);

  delete NPSS;

}
