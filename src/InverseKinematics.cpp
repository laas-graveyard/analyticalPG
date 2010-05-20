#include <math.h>
#include "InverseKinematics.h"

#define ODEBUG4(x,y)


void InverseKinematics::ComputeUpperBodyHeuristicForNormalWalking(    MAL_VECTOR(,double) & qArmr,
MAL_VECTOR(,double) & qArml,
MAL_VECTOR(,double) & aCOMPosition,
MAL_VECTOR(,double) & RFP,
MAL_VECTOR(,double) & LFP)
/*
COMPosition aCOMPosition,
FootAbsolutePosition RFP,
FootAbsolutePosition LFP )*/
{
	double m_ZARM = -1.0;
	double m_GainFactor = 0.5;

	double m_Xmax = ComputeXmax(m_ZARM);

	double GainX = m_GainFactor * m_Xmax/0.2;

	// Arms Motion : Heuristic based.
	double Alpha,Beta;
	//Temporary variables
	double TempXL,TempXR,TempCos,TempSin,
		TempARight,TempALeft;

	// Compute the position of the hand according to the
	// leg.
	TempCos = cos(aCOMPosition(5)*M_PI/180.0);
	TempSin = sin(aCOMPosition(5)*M_PI/180.0);

	TempXR = TempCos * (RFP(0)  - aCOMPosition(0) ) +
		TempSin * (RFP(1) - aCOMPosition(1) );
	TempXL = TempCos * (LFP(0)   - aCOMPosition(0) ) +
		TempSin * (LFP(1)  - aCOMPosition(1) );

	TempARight = TempXR*-1.0;
	TempALeft = TempXL*-1.0;

	// Compute angles using inverse kinematics and the computed hand position.
	ComputeInverseKinematicsForArms(TempALeft * GainX,
		m_ZARM,
		Alpha,
		Beta);

	qArml(0)= Alpha;
	qArml(1)= 10.0*M_PI/180.0;
	qArml(2)= 0.0;
	qArml(3)= Beta;
	qArml(4)= 0.0;
	qArml(5)= 0.0;
	qArml(6)= 10.0*M_PI/180.0;

	ODEBUG4( "IK Left arm p:" << qArml(0)<< " " <<  qArml(1)  << " " << qArml(2)
		<< " " << qArml(3) << "  " << qArml(4) << " " << qArml(5), "DebugDataIKArms.txt" );

	ComputeInverseKinematicsForArms(TempARight * GainX,
		m_ZARM,
		Alpha,
		Beta);
	qArmr(0)= Alpha;
	qArmr(1)= -10.0*M_PI/180.0;
	qArmr(2)= 0.0;
	qArmr(3)= Beta;
	qArmr(4)= 0.0;
	qArmr(5)= 0.0;
	qArmr(6)= 10.0*M_PI/180.0;;

	ODEBUG4( "IK Right arm p:" << qArmr(0)<< " " <<  qArmr(1)  << " " << qArmr(2)
		<< " " << qArmr(3) << "  " << qArmr(4) << " " << qArmr(5), "DebugDataIKArms.txt" );

	ODEBUG4( qArml(0)<< " " <<  qArml(1)  << " " << qArml(2) << " "
		<< qArml(3) << "  " << qArml(4) << " " << qArml(5) << " "
		<< qArmr(0)<< " " <<  qArmr(1)  << " " << qArmr(2) << " "
		<< qArmr(3) << "  " << qArmr(4) << " " << qArmr(5), "DebugDataqArmsHeuristic.txt");

}


double InverseKinematics::ComputeXmax(double & Z)
{
	double A=0.25,
		B=0.25;
	double Xmax;
	if (Z<0.0)
		Z = 2*A*cos(15*M_PI/180.0);
	Xmax = sqrt(A*A - (Z - B)*(Z-B));
	return Xmax;
}


int InverseKinematics::ComputeInverseKinematicsForArms(double X,double Z,
double &Alpha,
double &Beta)

{
	double A=0.25,
		B=0.25;

	double C=0.0,Gamma=0.0,Theta=0.0;
	C = sqrt(X*X+Z*Z);

	Beta = acos((A*A+B*B-C*C)/(2*A*B))- M_PI;
	Gamma = asin((B*sin(M_PI+Beta))/C);
	Theta = atan2(X,Z);
	Alpha = Gamma - Theta;
	return 0;

}


int InverseKinematics::ComputeInverseKinematics2ForLegs(MAL_S3x3_MATRIX(,double) &Body_R,
							MAL_S3_VECTOR( ,double) &Body_P,
							MAL_S3_VECTOR( ,double) &Dt,
							MAL_S3x3_MATRIX( ,double) &Foot_R,
							MAL_S3_VECTOR( ,double) &Foot_P,
							MAL_VECTOR( ,double)&q)
{

  double m_KneeAngleBound=0.0*M_PI/180.0;
  double m_KneeAngleBoundCos=cos(m_KneeAngleBound);
  double m_KneeAngleBound1=30.0*M_PI/180.0; //sets a minimum angle for the knee and protects for overstretch
  double m_KneeAngleBoundCos1=cos(m_KneeAngleBound1);  //used during inverse kin calculations
  
  double m_FemurLength=0.30;
  double m_TibiaLength=0.30;
  
  //m_KneeAngleBound=15.0*M_PI/180.0; //sets a minimum angle for the knee and protects for overstretch
  double m_KneeAngleBoundCos2= 1.336;

  double A=m_FemurLength,B=m_TibiaLength,C=0.0,c5=0.0,q6a=0.0;
  MAL_S3_VECTOR( r,double);
  MAL_S3x3_MATRIX( rT,double);
  MAL_S3x3_MATRIX( Foot_Rt,double);
  double NormofDt=0.0;

  // New part for the inverse kinematics specific to the HRP-2
  // robot. The computation of rx, ry and rz is different.

  // We compute the position of the body inside the reference
  // frame of the foot.
  MAL_S3_VECTOR( v,double);
  double theta=0.0, psi=0.0, Cp=0.0;
  float OppSignOfDtY = Dt(1) < 0.0 ? 1.0 : -1.0;


  Foot_Rt = MAL_S3x3_RET_TRANSPOSE(Foot_R);
  v = Body_P - Foot_P;
  v = MAL_S3x3_RET_A_by_B(Foot_Rt , v);
  //  cout << "v : "<< v <<endl;
  r(0) = v(0);
  NormofDt = sqrt(Dt(0)*Dt(0) + Dt(1)*Dt(1) + Dt(2)*Dt(2));
  //  cout << "Norm of Dt: " << NormofDt << endl;
  Cp = sqrt(v(1)*v(1)+v(2)*v(2) - NormofDt * NormofDt);
  psi = OppSignOfDtY * atan2(NormofDt,Cp);

  
  //  cout << "vz: " << v(2,0) << " vy :" << v(1,0) << endl;
  theta = atan2(v(2),v(1));
  
  r(1) = cos(psi+theta)*Cp;

  r(2) = sin(psi+theta)*Cp;

  //  r = rT * (Body_P +  Body_R * Dt - Foot_P);
  C = sqrt(r(0)*r(0)+
	   r(1)*r(1)+
	   r(2)*r(2));
  //C2 =sqrt(C1*C1-D*D);
  c5 = (C*C-A*A-B*B)/(2.0*A*B);

  if (c5>=m_KneeAngleBoundCos)
    {

      q(3)=m_KneeAngleBound;

    }
  else if (c5<=-1.0)
    {
      q(3)= M_PI;
    }
  else 
    {
      q(3)= acos(c5);
    }
  q6a = asin((A/C)*sin(M_PI- q(3)));


  float c,s,cz,sz;

  q(5) = atan2(r(1),r(2));
  if (q(5)>M_PI/2.0)
    {
      q(5) = q(5)-M_PI;
    }
  else if (q(5)<-M_PI/2.0)
    {
      q(5)+= M_PI;
    }

  q(4) = -atan2(r(0), (r(2)<0? -1.0:1.0)*sqrt(r(1)*r(1)+r(2)*r(2) )) - q6a;

  MAL_S3x3_MATRIX(R,double);
  MAL_S3x3_MATRIX(BRt,double);

  BRt = MAL_S3x3_RET_TRANSPOSE(Body_R);
  
  MAL_S3x3_MATRIX( Rroll,double);
  c = cos(-q(5));
  s = sin(-q(5));
  
  Rroll(0,0) = 1.0;   Rroll(0,1) = 0.0;   Rroll(0,2) = 0.0; 
  Rroll(1,0) = 0.0;   Rroll(1,1) = c;   Rroll(1,2) = -s; 
  Rroll(2,0) = 0.0;   Rroll(2,1) = s;   Rroll(2,2) = c; 
  
  
  MAL_S3x3_MATRIX( Rpitch,double);
  c = cos(-q(4)-q(3));
  s = sin(-q(4)-q(3));
  
  Rpitch(0,0) = c;     Rpitch(0,1) = 0;   Rpitch(0,2) = s; 
  Rpitch(1,0) = 0.0;   Rpitch(1,1) = 1;   Rpitch(1,2) = 0; 
  Rpitch(2,0) = -s;    Rpitch(2,1) = 0;   Rpitch(2,2) = c; 
  
  //  cout << " BRt"  << BRt << endl;
  R = MAL_S3x3_RET_A_by_B(BRt, Foot_R );
  MAL_S3x3_MATRIX(Rtmp,double);
  Rtmp = MAL_S3x3_RET_A_by_B(Rroll,Rpitch);
  R = MAL_S3x3_RET_A_by_B(R,Rtmp);

  q(0) = atan2(-R(0,1),R(1,1));
  
  cz = cos(q(0)); sz = sin(q(0));
  q(1) = atan2(R(2,1), -R(0,1)*sz + R(1,1) *cz);
  q(2) = atan2( -R(2,0), R(2,2));
  
  //  exit(0);
  return 0;
}


void InverseKinematics::CallComputeInverseKinematics2ForLegs(

matrix3d &Body_R,
vector3d &Body_P,
matrix3d &Foot_R,
vector3d &Foot_P,
vectorN &legsq,
CjrlHumanoidDynamicRobot *mp_HDR)
{
	CjrlJoint *waist = mp_HDR->waist();
	CjrlJoint *lankle = mp_HDR->leftAnkle();

	matrix4d waistpose;
	matrix4d leftanklepose;
	MAL_S4x4_MATRIX_SET_IDENTITY(waistpose);
	MAL_S4x4_MATRIX_SET_IDENTITY(leftanklepose);

	for(unsigned int i=0;i<3;i++)
	{
		for(unsigned int j=0;j<3;j++)
		{
			waistpose(i,j) = Body_R(i,j);
			leftanklepose(i,j) = Foot_R(i,j);
		}
	}

	mp_HDR->getSpecializedInverseKinematics(*waist,*lankle,waistpose,leftanklepose,legsq);
}
