#include "ALG_EMO_MOEAD.h"
#include <string.h>

CALG_EMO_MOEAD::CALG_EMO_MOEAD(void)
{
	s_PBI_type = 3;
}


CALG_EMO_MOEAD::~CALG_EMO_MOEAD(void)
{

}


void CALG_EMO_MOEAD::Execute(int run_id)
{
    this->InitializeParameter();
	this->InitializePopulation();
	this->InitializeNeighborhood();


	int gen = 1;
	
	for(;;)
	{

		gen++;

		this->EvolvePopulation();
		
		if(IsTerminated()){
			break;
		}

		if((gen%25)==0)
		{ 
			//printf("Instance: RUN:  %d  GEN = %d\n", run_id, gen);
			this->SavePopulation(run_id);
		}


	}
	
	this->SavePopulation(run_id);

	m_PopulationSOP.clear();
	v_IdealPoint.clear();

}



void CALG_EMO_MOEAD::InitializeNeighborhood()
{
	vector<double> v_dist   = vector<double>(s_PopulationSize, 0);
	vector<int>    v_indx   = vector<int>(s_PopulationSize, 0);

	unsigned int i, j, k;
	for(i=0; i<s_PopulationSize; i++)
	{
		for(j=0; j<s_PopulationSize; j++)
		{
			v_dist[j] = UtilityToolBox.DistanceVectorNorm2(m_PopulationSOP[i].v_Weight_Vector,
                                              				m_PopulationSOP[j].v_Weight_Vector);
			v_indx[j] = j;
		}

		UtilityToolBox.Minfastsort(v_dist, 
			                       v_indx, 
								   s_PopulationSize, 
								   s_NeighborhoodSize);  


		for(k=0; k<s_NeighborhoodSize; k++)
		{
			m_PopulationSOP[i].v_Neighbor_Index.push_back(v_indx[k]);  // save the indexes into neighborhood
		}
	}
	v_dist.clear();
	v_indx.clear();
}


void CALG_EMO_MOEAD::InitializeParameter()
{
	char filename[1024];
    
	sprintf(filename,"SETTINGS/algorithms/MOEAD.txt");

	char str_temp[1024];
	std::ifstream readf(filename);
	//Pop_Size     NeighborhoodSize
	readf>>s_PopulationSize;
	readf>>s_NeighborhoodSize;
	readf.close();
}



void CALG_EMO_MOEAD::UpdateReference(vector<double> &obj_vect)
{
	for(unsigned n=0; n<NumberOfObjectives; n++)
	{
		if(obj_vect[n]<v_IdealPoint[n])
		{
			v_IdealPoint[n] = obj_vect[n];
		}
	}
}


void CALG_EMO_MOEAD::InitializePopulation()
{
	unsigned i, j;

	s_Fevals_Count = 0;

	v_IdealPoint = vector<double>(NumberOfObjectives, 1.0e+30);   


	char filename1[1024];
	sprintf(filename1,"SETTINGS/weightvectors/W%d-P%d.dat", NumberOfObjectives, s_PopulationSize);
	std::ifstream readf(filename1);

	for(i=0; i<s_PopulationSize; i++)
	{
		CSubProblemBase SP;
		SP.m_BestIndividual.Randomize();
		SP.m_BestIndividual.Evaluate();
		//std::cout << "total_hijos: " << SP.m_BestIndividual.total_hijos << " - total_no_hojas: " << SP.m_BestIndividual.total_no_hojas << endl;
		//std::cout << "total_hijos: " << SP.m_BestIndividual.f_obj[0] << " - total_no_hojas: " << SP.m_BestIndividual.f_obj[1] << endl;
		//SP.m_BestIndividual.Show(0);

		s_Fevals_Count++;

		UpdateReference(SP.m_BestIndividual.f_obj);    // update reference point

		for(j=0; j<NumberOfObjectives; j++)
		{
			readf>>SP.v_Weight_Vector[j];
		}

		//SP.Show_Weight_Vector();  	getchar();

		m_PopulationSOP.push_back(SP);
	}

	readf.close();

	//this->FindNadirPoint();

	//if(s_PBI_type==3) 	NormalizeWeight();
}




void CALG_EMO_MOEAD::FindNadirPoint()
{

	v_NadirPoint = vector<double>(NumberOfObjectives, -1.0e30);

	for(int j=0; j<NumberOfObjectives; j++)
	{
		//*
		vector<double> weight_tch = vector<double>(NumberOfObjectives, 10e-6);
		weight_tch[j] = 1;

		double asf_min = 1.0e+30;
		int    asf_id  = 0;
		for(int s=0; s<s_PopulationSize; s++)
		{
			double tch_max = -1.0e+30;
			for(int k=0; k<NumberOfObjectives; k++)
			{
				double temp = m_PopulationSOP[s].m_BestIndividual.f_obj[k]/weight_tch[k];
				if(temp>tch_max)
				{
				    tch_max = temp;
				}
			}
			if(tch_max<asf_min)
			{
				asf_min = tch_max;
				asf_id  = s;
			}
		}
		v_NadirPoint[j] = m_PopulationSOP[asf_id].m_BestIndividual.f_obj[j];				
	}


	/*
	for(int s=0; s<s_PopulationSize; s++)
	{
		this->NormalizeIndividual(m_PopulationSOP[s].m_BestIndividual);
	}
	*/
}


void CALG_EMO_MOEAD::NormalizeWeight()
{

	for(int s=0; s<s_PopulationSize; s++)
	{
		for(int j=0; j<NumberOfObjectives; j++)
		{
			m_PopulationSOP[s].v_Weight_Vector2[j] = m_PopulationSOP[s].v_Weight_Vector[j]*(v_NadirPoint[j] - v_IdealPoint[j]);			
		}
		UtilityToolBox.NormalizeVector(m_PopulationSOP[s].v_Weight_Vector2);
	}
}


void CALG_EMO_MOEAD::NormalizeIndividual(CIndividualBase &ind)
{
	//ind.Show(1);
	for(int i=0; i<NumberOfObjectives; i++)
	{
		if(abs(v_NadirPoint[i] - v_IdealPoint[i])<1.0e-6)
			ind.f_normal[i] = (ind.f_obj[i] - v_IdealPoint[i])/1.0e-6;		
		else
		    ind.f_normal[i] = (ind.f_obj[i] - v_IdealPoint[i])/(v_NadirPoint[i] - v_IdealPoint[i] + 1.0e-6);		
		//printf("%f ", ind.f_normal[i]);
	}
	//getchar();
}


void CALG_EMO_MOEAD::UpdateProblem(CIndividualBase &child, 
								   unsigned sp_id)
{

	double f1, f2;

	int    id1 = sp_id, id2;

	vector<double> referencepoint = vector<double>(NumberOfObjectives, 0);

	for(int i=0; i<s_NeighborhoodSize; i++)
	{

		id2 = m_PopulationSOP[id1].v_Neighbor_Index[i];


		//*
		f1 = UtilityToolBox.ScalarizingFunction(m_PopulationSOP[id2].m_BestIndividual.f_obj,
              			                        m_PopulationSOP[id2].v_Weight_Vector,
												v_IdealPoint,
												1);  // 1 - TCH  3 - PBI


		f2 = UtilityToolBox.ScalarizingFunction(child.f_obj,
                                    			m_PopulationSOP[id2].v_Weight_Vector,
												v_IdealPoint,
												1);
		/*

		if(s_PBI_type==1)
		{

			f1 = UtilityToolBox.ScalarizingFunction(m_PopulationSOP[id2].m_BestIndividual.f_obj,
				m_PopulationSOP[id2].v_Weight_Vector,
				v_IdealPoint,
				3);  // 1 - TCH  3 - PBI


			f2 = UtilityToolBox.ScalarizingFunction(child.f_obj,
				m_PopulationSOP[id2].v_Weight_Vector,
				v_IdealPoint,
				3);
		}


		if(s_PBI_type==2)
		{
			vector<double> referencepoint = vector<double>(NumberOfObjectives, 0);
			this->NormalizeIndividual(m_PopulationSOP[id2].m_BestIndividual);
			f1 = UtilityToolBox.ScalarizingFunction(m_PopulationSOP[id2].m_BestIndividual.f_normal,
				m_PopulationSOP[id2].v_Weight_Vector,
				referencepoint,
				3);  // 1 - TCH  3 - PBI


			this->NormalizeIndividual(child);
			f2 = UtilityToolBox.ScalarizingFunction(child.f_normal,
				m_PopulationSOP[id2].v_Weight_Vector,
				referencepoint,
				3);
		}

		if(s_PBI_type==3)
		{

			f1 = UtilityToolBox.ScalarizingFunction(m_PopulationSOP[id2].m_BestIndividual.f_obj,
				m_PopulationSOP[id2].v_Weight_Vector2,
				v_IdealPoint,
				3);  // 1 - TCH  3 - PBI


			f2 = UtilityToolBox.ScalarizingFunction(child.f_obj,
				m_PopulationSOP[id2].v_Weight_Vector2,
				v_IdealPoint,
				3);
		}
		//*/

		if(f2<f1)
		{
			m_PopulationSOP[id2].m_BestIndividual = child;
		}
	}
}




void CALG_EMO_MOEAD::SelectMatingPool(vector<unsigned> &pool, 
									  unsigned sp_id, 
									  unsigned selected_size)
{
	unsigned  id, p;
	while(pool.size()<selected_size)
	{
		id      = int(s_NeighborhoodSize*UtilityToolBox.Get_Random_Number());
		p       = m_PopulationSOP[sp_id].v_Neighbor_Index[id];

		bool flag = true;
		for(unsigned i=0; i<pool.size(); i++)
		{
			if(pool[i]==p) // parent is in the list
			{
				flag = false;
				break;
			}
		}

		if(flag){
			pool.push_back(p);
		}
	}
}

// CROSSOVER

// Recolectar todos los nodos del árbol
void recolectar_nodos(shared_ptr<Nodo> nodo, vector<shared_ptr<Nodo>>& nodos) {
    if (!nodo) return;
    nodos.push_back(nodo);
    recolectar_nodos(nodo->left, nodos);
    recolectar_nodos(nodo->right, nodos);
    recolectar_nodos(nodo->down, nodos);
}

// Buscar el padre de un nodo dado
void buscar_y_reemplazar(shared_ptr<Nodo> nodo, shared_ptr<Nodo> objetivo, shared_ptr<Nodo> nuevo_subarbol) {
    if (!nodo) return;

    if (nodo->left == objetivo) {
        nodo->left = nuevo_subarbol;
        return;
    }
    if (nodo->right == objetivo) {
        nodo->right = nuevo_subarbol;
        return;
    }
    if (nodo->down == objetivo) {
        nodo->down = nuevo_subarbol;
        return;
    }

    buscar_y_reemplazar(nodo->left, objetivo, nuevo_subarbol);
    buscar_y_reemplazar(nodo->right, objetivo, nuevo_subarbol);
    buscar_y_reemplazar(nodo->down, objetivo, nuevo_subarbol);
}

// Función principal de crossover
void crossover_arboles(shared_ptr<Nodo> arbol1, shared_ptr<Nodo> arbol2, shared_ptr<Nodo>& arbol3) {
    // Copiar arbol1 en arbol3
    arbol3 = arbol1->copiar_nodo();

    // Recolectar nodos de arbol1 y arbol2
    vector<shared_ptr<Nodo>> nodos_arbol1, nodos_arbol2;
    recolectar_nodos(arbol3, nodos_arbol1); // copiar de arbol1
    recolectar_nodos(arbol2, nodos_arbol2);

    uniform_int_distribution<> dist1(1, nodos_arbol1.size() - 1);
    uniform_int_distribution<> dist2(1, nodos_arbol2.size() - 1);

    // Intentamos encontrar un par de nodos que no sean ambos hojas
    shared_ptr<Nodo> nodo_p1 = nodos_arbol1[dist1(rng)];
    shared_ptr<Nodo> nodo_p2 = nodos_arbol2[dist2(rng)];
    int intentos = 0;

    while(nodo_p1->nhijos == 0 && nodo_p2->nhijos == 0 && intentos < 100){
        shared_ptr<Nodo> nodo_p1 = nodos_arbol1[dist1(rng)];
        shared_ptr<Nodo> nodo_p2 = nodos_arbol2[dist2(rng)];
        intentos++;
    }

    shared_ptr<Nodo> subarbol_a_insertar = nodo_p2->copiar_nodo();

    // Reemplazar el subárbol en arbol3
    buscar_y_reemplazar(arbol3, nodo_p1, subarbol_a_insertar);
}

void reparar_coordenadas(shared_ptr<Nodo>& nodo, 
                         set<pair<int, int>>& used_coords) {
    if (!nodo) return;

    // Intentar insertar coordenadas actuales
    auto res = used_coords.insert(nodo->coordenadas);
    if (!res.second) {
        // Coordenada ya usada, eliminar este subárbol
        nodo = nullptr;
        return;
    }

    // Reparar cada hijo recursivamente
    vector<pair<string, shared_ptr<Nodo>*>> hijos = {
        {"left", &nodo->left},
        {"right", &nodo->right},
        {"down", &nodo->down}
    };

    for (auto& [tipo, ptr_hijo] : hijos) {
        if (*ptr_hijo) {
            // Recalcular coordenadas y dirección
            string nueva_direccion = nodo->calcular_direccion_hijo(tipo);
            pair<int, int> nueva_coord = nodo->coord_hijo(nueva_direccion);

            (*ptr_hijo)->direccion = nueva_direccion;
            (*ptr_hijo)->coordenadas = nueva_coord;

            reparar_coordenadas(*ptr_hijo, used_coords);

            // Si se eliminó el hijo por coordenadas duplicadas, ajustar nhijos
            if (!(*ptr_hijo)) {
                nodo->nhijos--;
            }
        }
    }
}

void contar_nodos(const shared_ptr<Nodo>& nodo, int& total_hijos, int& total_no_hojas) {
    if (!nodo) return;

    bool es_no_hoja = false;

    if (nodo->left) {
        contar_nodos(nodo->left, total_hijos, total_no_hojas);
        es_no_hoja = true;
    }
    if (nodo->right) {
        contar_nodos(nodo->right, total_hijos, total_no_hojas);
        es_no_hoja = true;
    }
    if (nodo->down) {
        contar_nodos(nodo->down, total_hijos, total_no_hojas);
        es_no_hoja = true;
    }

    if (es_no_hoja) {
        total_no_hojas++;
    }

    // Contamos este nodo (si quieres excluir la raíz, puedes condicionar esto)
    total_hijos++;
}

void CALG_EMO_MOEAD::EvolvePopulation()
{

	if(IsTerminated()) return;

	std::vector<int> order(std::vector<int>(s_PopulationSize,0));
	UtilityToolBox.RandomPermutation(order, 0);


	
	int p1, p2;
	vector<unsigned> mating_pool;

	for(unsigned int s=0; s<s_PopulationSize; s++)
	{
		CIndividualBase child;
		child.arbol = nullptr;
		child.used_coords.clear();

		unsigned int id_c = order[s];

		SelectMatingPool(mating_pool, id_c, 2);
		p1 = mating_pool[0]; p2 = mating_pool[1]; mating_pool.clear();

		//UtilityToolBox.SimulatedBinaryCrossover(m_PopulationSOP[p1].m_BestIndividual.x_var,
		//	                                    m_PopulationSOP[p2].m_BestIndividual.x_var,
		//										child.x_var);

		//UtilityToolBox.PolynomialMutation(child.x_var, 1.0/NumberOfVariables);

		crossover_arboles(m_PopulationSOP[p1].m_BestIndividual.arbol,
			            	m_PopulationSOP[p2].m_BestIndividual.arbol,
							child.arbol);

		if (!child.arbol) {
			cerr << "Error: el árbol hijo es nulo tras el crossover." << endl;
			return;
		}

		reparar_coordenadas(child.arbol, child.used_coords);

		child.total_hijos = -1;
		child.total_no_hojas = 0;

		contar_nodos(child.arbol, child.total_hijos, child.total_no_hojas);

		mutate_tree(child);

		//cout << "child.total_hijos: " << child.total_hijos << " - child.total_no_hojas: " << child.total_no_hojas << endl;
				
		child.Evaluate();   s_Fevals_Count++;

		//this->NormalizeIndividual(child);

		//child.Show(0); 
		//getchar();

		UpdateReference(child.f_obj);
		
		UpdateProblem(child, id_c);

		if(IsTerminated()) break;
	}

	//this->FindNadirPoint();

	//if(s_PBI_type==3)  this->NormalizeWeight();
}

void CALG_EMO_MOEAD::mov_delhoja(Nodo& nodo, int& total_hijos, int& total_no_hojas, 
                   set<pair<int, int>>& used_coords){

    vector<string> direcciones = {"left", "right", "down"};

    // Usamos el generador global 'rng' en lugar de crear uno nuevo
    shuffle(direcciones.begin(), direcciones.end(), rng);

    for (const auto& dir : direcciones) {
        if (nodo.left && dir == "left") {
            if (nodo.left->nhijos == 0){
                nodo.eliminar_hijo("left", total_hijos, total_no_hojas, used_coords);
                return;
            }
            mov_delhoja(*nodo.left, total_hijos, total_no_hojas, used_coords);
            break;
        } else if (nodo.right && dir == "right") {
            if (nodo.right->nhijos == 0){
                nodo.eliminar_hijo("right", total_hijos, total_no_hojas, used_coords);
                return;
            }
            mov_delhoja(*nodo.right, total_hijos, total_no_hojas, used_coords);
            break;
        } else if (nodo.down && dir == "down") {
            if (nodo.down->nhijos == 0){
                nodo.eliminar_hijo("down", total_hijos, total_no_hojas, used_coords);
                return;
            }
            mov_delhoja(*nodo.down, total_hijos, total_no_hojas, used_coords);
            break;
        }
    }  
}

void CALG_EMO_MOEAD::mov_addhoja(Nodo& nodo, int& total_hijos, int& total_no_hojas, 
                   set<pair<int, int>>& used_coords){

    vector<string> direcciones = {"left", "right", "down"};

    // Se utiliza el generador global 'rng'
    shuffle(direcciones.begin(), direcciones.end(), rng);

    for (const auto& dir : direcciones) {
        if (dir == "left") {
            if (nodo.left) {
                mov_addhoja(*nodo.left, total_hijos, total_no_hojas, used_coords);
                return;
            }
            nodo.agregar_hijo("left", total_hijos, total_no_hojas, used_coords);
            if (nodo.left) return;
        } else if (dir == "right") {
            if (nodo.right) {
                mov_addhoja(*nodo.right, total_hijos, total_no_hojas, used_coords);
                return;
            }
            nodo.agregar_hijo("right", total_hijos, total_no_hojas, used_coords);
            if (nodo.right) return;
        } else if (dir == "down") {
            if (nodo.down) {
                mov_addhoja(*nodo.down, total_hijos, total_no_hojas, used_coords);
                return;
            }
            nodo.agregar_hijo("down", total_hijos, total_no_hojas, used_coords);
            if (nodo.down) return;
        }
    }  
}

void CALG_EMO_MOEAD::mutate_tree(CIndividualBase &ind){
                    
	int n = N_rooms/3;

	

    // Realizar movimiento
    uniform_int_distribution<> distrib(0, n);

    int t_del = distrib(rng);
    int t_add = distrib(rng);

    if(t_del >= ind.total_hijos) t_del = 0;

    for (int i = 0; i < t_del; ++i) {
        mov_delhoja(*ind.arbol, ind.total_hijos, ind.total_no_hojas, ind.used_coords);
    }

    for (int i = 0; i < t_add; ++i) {
        mov_addhoja(*ind.arbol, ind.total_hijos, ind.total_no_hojas, ind.used_coords);
    }     

}

bool CALG_EMO_MOEAD::IsTerminated()
{
	if(s_Fevals_Count>=NumberOfFuncEvals){
		return true;
	}
	else{
		return false;
	}

}


void CALG_EMO_MOEAD::SaveObjSpace(char saveFilename[1024])
{
	std::fstream fout;
	fout.open(saveFilename,std::ios::out);
	for(unsigned n=0; n<s_PopulationSize; n++)
	{
		int total_hijos = m_PopulationSOP[n].m_BestIndividual.total_hijos;
		int total_no_hojas = m_PopulationSOP[n].m_BestIndividual.total_no_hojas;
		double lc_actual = static_cast<double>(total_hijos)/total_no_hojas;
		fout << total_hijos+1 << ";" << lc_actual;
		//if((total_hijos+1)==100 & lc_actual == 1.5)
			//m_PopulationSOP[n].m_BestIndividual.arbol->exportar_arbol_json("optimum_tree.json");
		fout<<"\n";
	}
	fout.close();
}



void CALG_EMO_MOEAD::SaveVarSpace(char saveFilename[1024])
{
	std::fstream fout;
	fout.open(saveFilename,std::ios::out);
	for(unsigned n=0; n<s_PopulationSize; n++)
	{
		for(unsigned k=0; k<NumberOfVariables; k++)
		{
			
		}
		fout<<"\n";
	}
	fout.close();
}

void CALG_EMO_MOEAD::SavePopulation(int run_id)
{
	char filename[1024];
	sprintf(filename,"SAVING/%d_%d_%.1f_%d.csv",rnd_uni_seed, N_rooms, LinCoef, NumberOfFuncEvals);
	SaveObjSpace(filename);//��ǰ��Ⱥ���

	//sprintf_s(filename,"Saving/MOEAD/POS_%s_RUN%d.dat",strTestInstance, run_id);
	//SaveVarSpace(filename);
}
