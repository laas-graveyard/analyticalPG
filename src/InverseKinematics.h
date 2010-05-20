/*! \file Object to handle HRP-2's specific Inverse Kinematics. */

#ifndef _INVERSE_KINEMATICS_H_
#define _INVERSE_KINEMATICS_H_
#include <jrlMathTools/jrlConstants.h>

#include <MatrixAbstractLayer/MatrixAbstractLayer.h>

#include <robotDynamics/jrlHumanoidDynamicRobot.h>

namespace InverseKinematics 
{
  double ComputeXmax(double & Z) ;
  
  int ComputeInverseKinematicsForArms(double X,double Z,
				      double &Alpha,
				      double &Beta) ;
  
  void ComputeUpperBodyHeuristicForNormalWalking(    MAL_VECTOR(,double) & qArmr,
						     MAL_VECTOR(,double) & qArml,
						     MAL_VECTOR(,double) & aCOMPosition,
						     MAL_VECTOR(,double) & RFP,
						     MAL_VECTOR(,double) & LFP);

  int ComputeInverseKinematics2ForLegs(MAL_S3x3_MATRIX(,double) &Body_R,
							MAL_S3_VECTOR( ,double) &Body_P,
							MAL_S3_VECTOR( ,double) &Dt,
							MAL_S3x3_MATRIX( ,double) &Foot_R,
							MAL_S3_VECTOR( ,double) &Foot_P,
							MAL_VECTOR( ,double)&q);

    /*! Call the inverse kinematics for HRP-2's 2 legs  */
  void CallComputeInverseKinematics2ForLegs(MAL_S3x3_MATRIX(,double) &Body_R, 
					    MAL_S3_VECTOR(,double) &Body_P, 
					    MAL_S3x3_MATRIX(,double) &Foot_R, 
					    MAL_S3_VECTOR(,double) &Foot_P,
					    MAL_VECTOR(,double) &legsq,
					    CjrlHumanoidDynamicRobot *mp_HDR) ;
    


};




#endif /* _INVERSE_KINEMATICS_H_ */
