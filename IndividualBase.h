// Individual.h: interface for the CIndividualBase class.
//
//////////////////////////////////////////////////////////////////////
#pragma once

#include "GlobalVariable.h"
#include "GlobalObject.h"
#include <vector>
#include <string.h>
using namespace std;

#include <iostream>
#include <string>
#include <unordered_map>
#include <memory>
#include <fstream>
#include <utility>
#include <queue>
#include <random>
#include <unordered_set>
#include <set>
#include <cmath>
#include <time.h>
#include <chrono>
#include <cstdlib> 

#include "json.hpp"
using json = nlohmann::json;

class Nodo {
public:
    string direccion; // direccion es para calcular las coordenadas.
    pair<int, int> coordenadas;
    shared_ptr<Nodo> left;
    shared_ptr<Nodo> right;
    shared_ptr<Nodo> down;
    int nhijos = 0;
    int bar = 0;
    int key = 0;

    Nodo(const string& direccion, pair<int, int> coordenadas, Nodo* parent = nullptr)
        : direccion(direccion), coordenadas(coordenadas), left(nullptr), right(nullptr), down(nullptr) {}

    void agregar_hijo(const string& tipo_hijo, 
                   int& total_hijos, int& total_no_hojas, 
                   set<pair<int, int>>& used_coords) {
        string direccion_hijo = calcular_direccion_hijo(tipo_hijo);

        pair<int, int> coordenadas_hijo = coord_hijo(direccion_hijo);
        auto resultado = used_coords.insert(coordenadas_hijo);
        if (!resultado.second)
            return;

        auto nuevo_hijo = make_shared<Nodo>(direccion_hijo, coordenadas_hijo, this);

        if (tipo_hijo == "left") {
            left = move(nuevo_hijo);
        } else if (tipo_hijo == "right") {
            right = move(nuevo_hijo);
        } else if (tipo_hijo == "down") {
            down = move(nuevo_hijo);
        }
        if (nhijos == 0)
        {
            total_no_hojas++;
        }
        nhijos++;
        
        total_hijos++;
    }

    string calcular_direccion_hijo(const string& tipo_hijo) {
        if (tipo_hijo == "down") {
            return direccion;
        } else if (tipo_hijo == "left") {
            unordered_map<string, string> direcciones = {{"N", "W"}, {"E", "N"}, {"S", "E"}, {"W", "S"}}; //(Se considera que el padre siempre es el Norte.)
            return direcciones[direccion];
        } else if (tipo_hijo == "right") {
            unordered_map<string, string> direcciones = {{"N", "E"}, {"E", "S"}, {"S", "W"}, {"W", "N"}};
            return direcciones[direccion];
        }
        return direccion;
    }

    pair<int, int> coord_hijo(const string& direccion_hijo) {
        if (direccion_hijo == "S") {
            return {coordenadas.first, coordenadas.second - 1};
        } else if (direccion_hijo == "N") {
            return {coordenadas.first, coordenadas.second + 1};
        } else if (direccion_hijo == "W") {
            return {coordenadas.first - 1, coordenadas.second};
        } else if (direccion_hijo == "E") {
            return {coordenadas.first + 1, coordenadas.second};
        }
        return coordenadas;
    }

    void eliminar_hijo(const string& tipo_hijo, 
                   int& total_hijos, int& total_no_hojas, 
                   set<pair<int, int>>& used_coords) {

        pair<int, int> coordenadas_hijo;

        if (tipo_hijo == "left") {
            string direccion_hijo = calcular_direccion_hijo(tipo_hijo);
            coordenadas_hijo = coord_hijo(direccion_hijo);
            left.reset();
        } else if (tipo_hijo == "right") {
            string direccion_hijo = calcular_direccion_hijo(tipo_hijo);
            coordenadas_hijo = coord_hijo(direccion_hijo);
            right.reset();
        } else if (tipo_hijo == "down") {
            string direccion_hijo = calcular_direccion_hijo(tipo_hijo);
            coordenadas_hijo = coord_hijo(direccion_hijo);
            down.reset();
        }

        used_coords.erase(coordenadas_hijo);

        total_hijos--;
        nhijos--;
        if (nhijos == 0)
        {
            total_no_hojas--;
        }
    }

    //Copiar

    // Constructor de copia para Nodo
    Nodo(const Nodo& otro)
        : direccion(otro.direccion),
          coordenadas(otro.coordenadas),
          nhijos(otro.nhijos),
          bar(otro.bar),
          key(otro.key) {
        // Copiar hijos recursivamente si existen
        if (otro.left) {
            left = std::make_shared<Nodo>(*otro.left);
        }
        if (otro.right) {
            right = std::make_shared<Nodo>(*otro.right);
        }
        if (otro.down) {
            down = std::make_shared<Nodo>(*otro.down);
        }
    }

    // Método copiar
    std::shared_ptr<Nodo> copiar_nodo() const {
        return std::make_shared<Nodo>(*this);
    }

    // Guardar los datos de árbol en un json:
    
    void exportar_arbol_json(const string& filename) {
        json tree_data;
        exportar_nodo_json(tree_data, this); // Exportar desde la raíz

        std::ofstream file(filename);
        file << tree_data.dump(4);

    }

    void exportar_nodo_json(json& tree_data, Nodo* nodo) {
        // Usa las coordenadas como clave en el árbol.
        string coord_key = "(" + to_string(nodo->coordenadas.first) + ", " + to_string(nodo->coordenadas.second) + ")";

        // Crea una lista para almacenar las conexiones del nodo
        json conexiones = json::array();

        if (nodo->left) {
            conexiones.push_back({
                {"direccion", "left"},
                {"coordenadas", {nodo->left->coordenadas.first, nodo->left->coordenadas.second}},
                {"llave", nodo->left->key},
                {"barrera", nodo->left->bar}
            });
            exportar_nodo_json(tree_data, nodo->left.get());
        }
        if (nodo->right) {
            conexiones.push_back({
                {"direccion", "right"},
                {"coordenadas", {nodo->right->coordenadas.first, nodo->right->coordenadas.second}},
                {"llave", nodo->right->key},
                {"barrera", nodo->right->bar}
            });
            exportar_nodo_json(tree_data, nodo->right.get());
        }
        if (nodo->down) {
            conexiones.push_back({
                {"direccion", "down"},
                {"coordenadas", {nodo->down->coordenadas.first, nodo->down->coordenadas.second}},
                {"llave", nodo->down->key},
                {"barrera", nodo->down->bar}
            });
            exportar_nodo_json(tree_data, nodo->down.get());
        }

        // Guarda las conexiones en el nodo actual en el JSON
        tree_data[coord_key] = conexiones;
    }

};

class CIndividualBase  
{
public:
	CIndividualBase();
	virtual ~CIndividualBase();

	shared_ptr<Nodo> arbol;
	set<pair<int, int>> used_coords;
	int total_hijos;
    int total_no_hojas;

	//vector <double> x_var;
	vector <double> f_obj;
	vector <double> f_normal;


	unsigned int    rank;
	unsigned int    count;
	unsigned int    type;
	double          density;

	void   Randomize();
	void   Evaluate();
	void   Show(int type);

    bool   operator<(const CIndividualBase &ind2);
	bool   operator<<(const CIndividualBase &ind2);
    bool   operator==(const CIndividualBase &ind2);
    void   operator=(const CIndividualBase &ind2);

};


