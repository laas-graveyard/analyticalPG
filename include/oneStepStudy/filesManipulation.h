/*
 *  Copyright AIST-CNRS Joint Robotics Laboratory
 *  Author: Nicolas Perrin
 */

#ifndef filesManipulation_H
#define filesManipulation_H

#include <MatrixAbstractLayer/MatrixAbstractLayer.h>

#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <vector>
#include <set>


#define REPLACE_END "\" value=" 
//in the below code, the functions replace_... will try to localize a
//string corresponding to a tag in a file, and then access the value
// just after the string. 
// If there is some useless syntax between the tag and the data, one
//should put it in the macro REPLACE_END. For instance, in:
// ...RLEG_JOINT0.angle" value= 0.22.., we can use the string
// "RLEG_JOINT0.angle", and choose REPLACE_END := "\" value=",
//so that the accessed value is correct (warning: be sure REPLACE_END
//will stop at least one character before the value to access)

//REPLACE_END is the value by default, but it can be precised

//The functions replace_ have an ugly code, which is not efficient,
// but does what it has to do. It is really a ad hoc collection of functions.


double replace_or_get_value_scalar(
				   const char *filename, 
				   const char *look_for_it_input, 
				   double value, 
				   bool choice, 
				   string str
				   );

MAL_VECTOR(,double) replace_or_get_value_vectorN(
						 int N, 
						 const char *filename, 
						 const char *look_for_it_input, 
						 MAL_VECTOR(,double) & vect, 
						 bool choice, 
						 string str
						 );

MAL_S3_VECTOR(,double) replace_or_get_value_vector3(
						    const char *filename, 
						    const char *look_for_it_input, 
						    double value1, 
						    double value2, 
						    double value3, 
						    bool choice, 
						    string str
						    );


double replace_value_scalar(
			    const char *, 
			    const char *, 
			    double, 
			    string=REPLACE_END
			    );

double get_value_scalar(
			const char *, 
			const char *, 
			string=REPLACE_END
			);

MAL_S3_VECTOR(,double) replace_value_vector3(
					     const char *, 
					     const char *, 
					     double, 
					     double, 
					     double, 
					     string=REPLACE_END
					     );

MAL_S3_VECTOR(,double) get_value_vector3(
					 const char *, 
					 const char *, 
					 string=REPLACE_END
					 );

MAL_VECTOR(,double) replace_value_vectorN(
					  int, 
					  const char *, 
					  const char *, 
					  MAL_VECTOR(,double) &, 
					  string=REPLACE_END
					  );

MAL_VECTOR(,double) get_value_vectorN(
				      int, 
				      const char *, 
				      const char *, 
				      string=REPLACE_END
				      );


#endif
