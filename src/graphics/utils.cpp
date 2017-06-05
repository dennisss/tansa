#include "utils.h"

#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <ctype.h>

bool parse_int(const char *str, int *val){

	char *endptr;

	long int v = strtol(str, &endptr, 10);

	if(endptr == str || *endptr != '\0')
		return false;


	if(v > INT_MAX || v < INT_MIN )
		return false;

	*val = (int) v;

	return true;
}

bool parse_double(const char *str, double *val){
	char *endptr;

	double v = strtod(str, &endptr);

	if(endptr == str || *endptr != '\0')
		return false;

	*val = (double) v;

	return true;
}
bool parse_float(const char *str, float *val){
	char *endptr;

	double v = strtof(str, &endptr);

	if(endptr == str || *endptr != '\0')
		return false;

	*val = (float) v;

	return true;
}


// http://stackoverflow.com/questions/5309471/getting-file-extension-in-c
const char *get_filename_ext(const char *filename) {
    const char *dot = strrchr(filename, '.');
    if(!dot || dot == filename) return "";
    return dot + 1;
}



static char *laststr = NULL;
char *spacetok(char *str){
	char c, *tok;


	if(str == NULL)
		str = laststr;


	// Skip initial whitespace
	c = *str;
	while(c != '\0' && isspace(c)){
		str++;
		c = *str;
	}


	if(c == '\0')
		return NULL;


	tok = str;

	while(c != '\0' && !isspace(c)){
		str++;
		c = *str;
	}

	if(c == '\0'){ // It is the actual end of the string
		laststr = str;
	}
	else{
		*str = '\0';
		laststr = str + 1;
	}


	return tok;
}
