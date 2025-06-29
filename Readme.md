Para compilar:
make

Para correr
./MOEAD seed

run_seeds.sh corre las semillas del 1011 a 1020 automáticamente.

En ALG_EMO_MAIN.cpp línea 28 esta el número de funciones de evaluación que considera el criterio de término.
En GlobalVariable.cpp líneas 16 y 17 están los datos de la instancia, número de habitaciones y coeficiente lineal.
Los resultados se guardan en la carpeta SAVING, con el formato seed_rooms_lc_funceval.csv
Donde seed es la semilla usada, rooms y lc son los datos de la instancia y funceval es la cantidad de funciones de evaluacion que tiene el criterio de término.
Los resultados del informe están ordenados por instancia en la carpeta SAVING