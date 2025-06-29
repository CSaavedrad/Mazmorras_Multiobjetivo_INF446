#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>


#include "GlobalVariable.h"
#include "IndividualBase.h"
#include "SubProblemBase.h"



class CALG_EMO_MOEAD
{
public:
	CALG_EMO_MOEAD(void);
	~CALG_EMO_MOEAD(void);

	void Execute(int run_id);                  

	void InitializeNeighborhood();
	void InitializePopulation();
	void InitializeParameter();



	void UpdateReference(vector<double> &obj_vect);
    void UpdateProblem(CIndividualBase &child, unsigned sp_id);

	void FindNadirPoint();
	void NormalizeIndividual(CIndividualBase &ind);
	void NormalizeWeight();

    void SelectMatingPool(vector<unsigned> &pool, unsigned sp_id, unsigned selected_size);
	void EvolvePopulation();
	bool IsTerminated();


	void SaveObjSpace(char saveFilename[1024]);
	void SaveVarSpace(char saveFilename[1024]);
	void SavePopulation(int run_id);

	void mov_delhoja(Nodo& nodo, int& total_hijos, int& total_no_hojas, 
                   set<pair<int, int>>& used_coords);

	void mov_addhoja(Nodo& nodo, int& total_hijos, int& total_no_hojas, 
                   set<pair<int, int>>& used_coords);
	
	void mutate_tree(CIndividualBase &ind);


public:

    vector <CSubProblemBase> m_PopulationSOP; 
	vector <double>          v_IdealPoint;       
	vector <double>          v_NadirPoint; 

	unsigned int     s_PopulationSize;                
    unsigned int	 s_NeighborhoodSize;              
//	unsigned int     s_ReplacementLimit;            
//	double           s_LocalMatingRatio;         
	int              s_Fevals_Count;

	int              s_PBI_type; 

};

