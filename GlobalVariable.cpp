#include "GlobalVariable.h"


char  strTestInstance[1024];
char  strCrossOverType[1024];

long  rnd_uni_init;      
int   rnd_uni_seed;

std::mt19937 rng(123);

int   NumberOfVariables;
int   NumberOfObjectives;
int   NumberOfFuncEvals;

int      N_rooms = 1000;
double   LinCoef = 1.5;