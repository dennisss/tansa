#ifndef TANSA_GRAPHICS_UTILS_H_
#define TANSA_GRAPHICS_UTILS_H_

#include <limits.h>

// Error checked integer parsing
bool parse_int(const char *str, int *val);

// Error checked double/float parsing
bool parse_double(const char *str, double *val);
bool parse_float(const char *str, float *val);

// Gets a files extension from its name/path
const char *get_filename_ext(const char *filename);

// Tokenizes a string by spaces; like strtok initally pass in the start string and then pass NULL
char *spacetok(char *str);

#endif
