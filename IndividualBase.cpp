// Individual.cpp: implementation of the CIndividualBase class.
//
//////////////////////////////////////////////////////////////////////
#include <iostream>
#include "IndividualBase.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CIndividualBase::CIndividualBase()
{

	//x_var    = vector<double>(NumberOfVariables, 0);
	shared_ptr<Nodo> arbol;
	set<pair<int, int>> used_coords;
	int total_hijos;
    int total_no_hojas;
    f_obj    = vector<double>(NumberOfObjectives, 0);
	f_normal = vector<double>(NumberOfObjectives, 0);
}

CIndividualBase::~CIndividualBase()
{
	
}

void calcular_probabilidades(double h, double& p1, double& p2, double& p3) {
    if (fabs(h - 1.0) < 1e-9) { // h == 1
        p1 = 0.7;
        p2 = 0.2;
        p3 = 0.1;
    }
    else if (fabs(h - 2.0) < 1e-9) { // h == 2
        p1 = 0.2;
        p2 = 0.6;
        p3 = 0.2;
    }
    else if (fabs(h - 3.0) < 1e-9) { // h == 3
        p1 = 0.1;
        p2 = 0.2;
        p3 = 0.7;
    }
    else if (h < 2) { // 1 < h < 2
        p1 = (1-(h-1)) - 0.1;
        p2 = (h-1) - 0.1;
        p3 = 0.2;
    }
    else if (h < 3) { // 2 < h < 3
        p1 = 0.2;
        p2 = (1-(h-2)) - 0.1;
        p3 = (h-1) - 0.1;
    }
}

int generar_numero_hijos(double p1, double p2, double p3) {
    
    uniform_real_distribution<> dist(0, 1);
    double random_value = dist(rng);

    if (random_value < p1) return 1;
    else if (random_value < p1 + p2) return 2;
    else return 3;
}

void generar_arbol(Nodo& raiz, double h, int num_nodos, 
                   int& total_hijos, int& total_no_hojas, 
                   set<pair<int, int>>& used_coords) {

    double p1, p2, p3;
    calcular_probabilidades(h, p1, p2, p3); // Se calcula con que probabilidad tendrá cierta cantidad de hijos.

    // Cola de prioridad para procesar los nodos en orden aleatorio.
    priority_queue<pair<int, Nodo*>> nodos_por_explorar;

    // Lambda para generar una prioridad usando el generador global 'rng'
    auto generar_prioridad = []() -> int {
        uniform_int_distribution<int> distribution(1, 100);
        return distribution(rng);
    };

    nodos_por_explorar.push({generar_prioridad(), &raiz});

    while (!nodos_por_explorar.empty()) {

        Nodo* nodo = nodos_por_explorar.top().second;
        nodos_por_explorar.pop();

        vector<string> direcciones = {"left", "right", "down"};

        // Se utiliza el generador global 'rng' para mezclar las direcciones.
        shuffle(direcciones.begin(), direcciones.end(), rng);

        int hijos = generar_numero_hijos(p1, p2, p3); // Se calcula la cantidad de hijos.

        for (const auto& dir : direcciones) {
            if (hijos <= 0) break;
            if (!nodo->left && dir == "left") {
                nodo->agregar_hijo("left", total_hijos, total_no_hojas, used_coords);
                if(nodo->left){
                    nodos_por_explorar.push({generar_prioridad(), nodo->left.get()});
                    hijos--;
                }
            } else if (!nodo->right && dir == "right") {
                nodo->agregar_hijo("right", total_hijos, total_no_hojas, used_coords);
                if(nodo->right){
                    nodos_por_explorar.push({generar_prioridad(), nodo->right.get()});
                    hijos--;
                }
            } else if (!nodo->down && dir == "down") {
                nodo->agregar_hijo("down", total_hijos, total_no_hojas, used_coords);
                if(nodo->down){
                    nodos_por_explorar.push({generar_prioridad(), nodo->down.get()});
                    hijos--;
                }
            }
        }
        if (total_hijos+1 >= num_nodos) {
            //std::cout << "total_hijos: " << total_hijos << " - total_no_hojas: " << total_no_hojas << endl;
            break;
        }
    }
}

void CIndividualBase::Randomize()
{
// Inicializar raíz
	arbol = make_shared<Nodo>("S", make_pair(0, 0));
	used_coords = {{0, 0}};
	total_hijos = 0;
	total_no_hojas = 0;

	// Generar árbol aleatorio
	generar_arbol(*arbol, LinCoef, N_rooms, total_hijos, total_no_hojas, used_coords);
    //std::cout << "total_hijos: " << total_hijos << " - total_no_hojas: " << total_no_hojas << endl;
}

void CIndividualBase::Evaluate()
{

	//if(!strcmp("ZDT1", strTestInstance))   TestInstance.ZDT1(x_var, f_obj, x_var.size());
	//TestInstance.fdvrp(x_var, f_obj, x_var.size());
	f_obj[0] = abs(N_rooms-(total_hijos+1));
	double lc_actual = static_cast<double>(total_hijos)/total_no_hojas;
    //std::cout << "total_hijos: " << total_hijos << " - total_no_hojas: " << total_no_hojas << endl;
    
	f_obj[1] = abs(LinCoef-lc_actual);
    //std::cout << "total_hijos: " << total_hijos << " - f0: " << f_obj[0] << endl;
    //std::cout << "lc_actual: " << lc_actual << " - f1: " << f_obj[1] << endl;
}


void CIndividualBase::Show(int type)
{

	unsigned int n;
	if(type==0)
	{
		double lc_actual = static_cast<double>(total_hijos)/total_no_hojas;
		std::cout << "Rooms: " << total_hijos+1 << " Lin_Coeff: " << lc_actual << endl;
	}
}


void CIndividualBase::operator=(const CIndividualBase &ind2)
{
    //x_var = ind2.x_var;
    arbol = ind2.arbol->copiar_nodo();
    used_coords = ind2.used_coords;
    total_hijos = ind2.total_hijos;
    total_no_hojas = ind2.total_no_hojas;

	f_obj = ind2.f_obj;
	f_normal = ind2.f_normal;
	rank  = ind2.rank;
	type  = ind2.type;
	count = ind2.count;
	density = ind2.density;
}

bool CIndividualBase::operator<(const CIndividualBase &ind2)
{
	bool dominated = true;
	unsigned int n;
    for(n=0; n<NumberOfObjectives; n++)
	{
		if(ind2.f_obj[n]<f_obj[n]) return false;
	}
	if(ind2.f_obj==f_obj) return false;
	return dominated;
}


bool CIndividualBase::operator<<(const CIndividualBase &ind2)
{
	bool dominated = true;
	unsigned int n;
    for(n=0; n<NumberOfObjectives; n++)
	{
		if(ind2.f_obj[n]<f_obj[n]  - 0.0001) return false;
	}
	if(ind2.f_obj==f_obj) return false;
	return dominated;
}

bool CIndividualBase::operator==(const CIndividualBase &ind2)
{
	if(ind2.f_obj==f_obj) return true;
	else return false;
}


