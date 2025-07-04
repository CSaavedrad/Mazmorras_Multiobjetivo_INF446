#OBJS = Node.o Timeslot.o ProblemInstance.o Reader.o Truck.o GlobalObject.o GlobalVariable.o IndividualBase.o SubProblemBase.o TestInstance.o UtilityToolBox.o ALG_EMO_MAIN.o ALG_EMO_MOEAD_DE.o ALG_EMO_MOEAD.o
OBJS = Timeslot.o GlobalObject.o GlobalVariable.o IndividualBase.o SubProblemBase.o TestInstance.o UtilityToolBox.o ALG_EMO_MAIN.o ALG_EMO_MOEAD_DE.o ALG_EMO_MOEAD.o

CFLAGS = -std=c++1y -O3 -g -ggdb -fprofile-use

instance=b-Instancia14_cap2_relacion7UnoUnoUnoTodosDistintos.dat
seed=123
ARGS = $(seed)

all: MOEAD

MOEAD: $(OBJS)
	g++ -o MOEAD $(OBJS) $(CFLAGS)

%.o: %.cpp
	g++ -c $(CFLAGS) $< -o $@

clean:
	rm -f MOEAD *.o
	rm core

exe:
	./MOEAD ${ARGS}

