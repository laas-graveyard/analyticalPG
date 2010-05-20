/*
 *  Copyright AIST-CNRS Joint Robotics Laboratory
 *  Author: Nicolas Perrin
 */

#include <MatrixAbstractLayer/MatrixAbstractLayer.h>

#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <vector>
#include <set>

//see header to understand the use of macro REPLACE_END

#define PI 3.14159265359

using namespace std;


//Very IMPORTANT remark: with these functions replace_or_get_value.., we look
//into the file for the FIRST OCCURENCE of the string in input, and
//we replace (or get) the corresponding value. If you want to replace values
//for further occurences, you have to slightly modify the function(s)
//replace_value_
//***Or, a good trick can be to mark a place in the file with a
//comment, and then use this in the input string to be sure that the
//accessed position is the good one.

//WARNING these functions are not optimized and therefore not to use
//in critical codes.
//More efficient functions could be designed with boost spirit, or with scripts using sed.

//PLEASE create backups of the files on which you want to apply these functions



double replace_or_get_value_scalar(
				   const char *filename, 
				   const char *look_for_it_input, 
				   double value, 
				   bool choice, 
				   string str
				   )
{ 
  //if choice==1 then replace else get
  
  int i = 0;
  char c;
  bool Cont=false; //useless variable

  FILE *fp, *ftmp;

  fp = fopen((char *)filename,"r");

  const string s= str;
  string tmpstring = look_for_it_input;
  tmpstring += s;
  const char *look_for_it = tmpstring.c_str();
 
  ftmp = fopen((char *)"ftmp.xml","w");

  do
    {
      fscanf(fp, "%c", &c);
      fputc(c, ftmp);
      
      if (!Cont)
  	if (c == look_for_it[i] && i < (int)(strlen(look_for_it)))
  	  {i++;}
  	else
  	  if (i == (int)strlen(look_for_it))
  	    break;
  	  else
  	    i = 0;
    }
  while (!feof(fp));

  double ret;
  fscanf(fp, "%lf", &ret);

  fprintf(ftmp,"%f",value);

  do
    {
      fscanf(fp, "%c", &c);
      fputc(c, ftmp);
    }
  while (!feof(fp));

  fclose (fp);
  fclose (ftmp);

  if (choice) {
    int er=remove(filename); //HERE, you have
                             //to have the rights. The
                             //best is to be root.
    if(er!=0) {perror("Error writing file (you might lack of rights)");}
    rename("ftmp.xml",filename);
  }
  return ret;

}

//N>=1 only
MAL_VECTOR(,double) replace_or_get_value_vectorN(
						 int N, 
						 const char *filename, 
						 const char *look_for_it_input, 
						 MAL_VECTOR(,double) & vect, 
						 bool choice, 
						 string str
						 )
{
  int i = 0;
  char c;
  bool Cont=false; //useless variable
  vector<double> ret;    

  FILE *ftmp;
  filebuf fpbuf;
  fpbuf.open (filename,ios::in);
  istream fp(&fpbuf);

  const string s = str;
  string tmpstring = look_for_it_input;
  tmpstring += s;
  const char *look_for_it = tmpstring.c_str();


  ftmp = fopen((char *)"ftmp.xml","w");
  
  do
    {
      fp >> noskipws >> c;    
      fputc(c, ftmp);
      
      if (!Cont)
  	if (c == look_for_it[i] && i < (int)(strlen(look_for_it)))
  	  {i++;
	  }
  	else
  	  if (i == (int)strlen(look_for_it))
  	    break;
  	  else
  	    i = 0;

    }
  while (!fp.eof());

  double d;

  while(fp.good()) {
    fp >> skipws >> d;     
    ret.push_back(d);
  }

  fp.clear();
 
  
  fprintf(ftmp,"%f",vect[0]);
  for(i=1;i<N;i++) {
    fprintf(ftmp,"%c",' ');
    fprintf(ftmp,"%f",vect[i]);   
  }    
  
  const char *end_string = "</grxui>";
  do
    {
      fp >> noskipws >> c;    
      fputc(c, ftmp);
      
      if (!Cont)
  	if (c == end_string[i] && i < (int)(strlen(end_string)))
  	  {i++;
	  }
  	else
  	  if (i == (int)strlen(end_string))
  	    break;
  	  else
  	    i = 0;
      
    }
  while (!fp.eof());


  fclose (ftmp);
  fpbuf.close();   

  if (choice) {
    int er=remove(filename); //HERE, you have
    //to have the rights. The
    //best is to be root.
    if(er!=0) {perror("Error writing file (you might lack of rights)");}
    rename("ftmp.xml",filename);
  }
  else remove("ftmp.xml");

  MAL_VECTOR_DIM(ret2,double,ret.size());
  for(int i=0; i<ret.size(); i++) ret2[i] = ret[i];

  return ret2;

}


MAL_S3_VECTOR(,double) replace_or_get_value_vector3(
						    const char *filename, 
						    const char *look_for_it_input, 
						    double value1, 
						    double value2, 
						    double value3, 
						    bool choice, 
						    string str
						    )
{
  int i = 0;
  char c;
  bool Cont=false; //useless variable   
  MAL_S3_VECTOR(ret,double);

  FILE *fp, *ftmp;

  fp = fopen((char *)filename,"r");

  const string s= str;
  string tmpstring = look_for_it_input;
  tmpstring += s;
  const char *look_for_it = tmpstring.c_str();


  ftmp = fopen((char *)"ftmp.xml","w");

  do
    {
      fscanf(fp, "%c", &c);
      fputc(c, ftmp);
      
      if (!Cont)
  	if (c == look_for_it[i] && i < (int)(strlen(look_for_it)))
  	  {i++;}
  	else
  	  if (i == (int)strlen(look_for_it))
  	    break;
  	  else
  	    i = 0;
    }
  while (!feof(fp));

  double d;
  fscanf(fp, "%lf", &d);
  MAL_S3_VECTOR_ACCESS(ret,0) = d;
  fscanf(fp, "%lf", &d);
  MAL_S3_VECTOR_ACCESS(ret,1) = d;
  fscanf(fp, "%lf", &d);
  MAL_S3_VECTOR_ACCESS(ret,2) = d;
  
  fprintf(ftmp,"%f",value1);
  fprintf(ftmp,"%c",' ');
  fprintf(ftmp,"%f",value2);
  fprintf(ftmp,"%c",' ');
  fprintf(ftmp,"%f",value3);
  

  do
    {
      fscanf(fp, "%c", &c);
      fputc(c, ftmp);
    }
  while (!feof(fp));

  fclose (fp);
  fclose (ftmp);
   
  if (choice) {
    int er=remove(filename); //HERE, you have
    //to the right. The
    //best is to be root.
    if(er!=0) {perror("Error writing file (you might lack of rights)");}
    rename("ftmp.xml",filename);
  }
					   
  return ret;

}


double replace_value_scalar(
			    const char *filename, 
			    const char *look_for_it_input, 
			    double value, 
			    string str
			    )
{
  double ret;
  ret = replace_or_get_value_scalar(filename, look_for_it_input, value, 1, str);
  return ret;
}

double get_value_scalar(
			const char *filename, 
			const char *look_for_it_input, 
			string str
			)
{
  double ret;
  ret = replace_or_get_value_scalar(filename, look_for_it_input, 0.0, 0, str);
  return ret;
}

MAL_S3_VECTOR(,double) replace_value_vector3(
					     const char *filename, 
					     const char *look_for_it_input, 
					     double value1, 
					     double value2, 
					     double value3, 
					     string str
					     )
{
  MAL_S3_VECTOR(ret,double);
  ret = replace_or_get_value_vector3(filename, look_for_it_input, value1, value2, value3, 1, str);
  return ret;
}

MAL_S3_VECTOR(,double) get_value_vector3(
					 const char *filename, 
					 const char *look_for_it_input, 
					 string str
					 )
{
  MAL_S3_VECTOR(ret,double);
  ret = replace_or_get_value_vector3(filename, look_for_it_input, 0.0, 0.0, 0.0, 0, str);
  return ret;
}

MAL_VECTOR(,double) replace_value_vectorN(
					  int N, 
					  const char *filename, 
					  const char *look_for_it_input, 
					  MAL_VECTOR(,double) & vect, 
					  string str
					  )
{

  MAL_VECTOR_DIM(ret,double,N);
  ret = replace_or_get_value_vectorN(N, filename, look_for_it_input, vect, 1, str);
  return ret;
}

MAL_VECTOR(,double) get_value_vectorN(
				      int N, 
				      const char *filename, 
				      const char *look_for_it_input, 
				      string str
				      )
{
  MAL_VECTOR_DIM(ret,double,N);
  MAL_VECTOR_DIM(vecinput,double,N);
  ret = replace_or_get_value_vectorN(N, filename, look_for_it_input, vecinput, 0, str);
  return ret;
}
