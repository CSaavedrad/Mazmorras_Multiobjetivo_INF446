
#pragma once
#include <random>

extern int   NumberOfVariables;
extern int   NumberOfObjectives;
extern int   NumberOfFuncEvals;

extern char  strCrossOverType[1024];
extern char  strTestInstance[1024];


extern long  rnd_uni_init;  
extern int   rnd_uni_seed;

extern int      N_rooms;
extern double   LinCoef;

extern std::mt19937 rng;