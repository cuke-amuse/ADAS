CC = g++
SRC = $(wildcard *.cpp)
SRC += $(wildcard ./env/*.cpp)
SRC += $(wildcard ./action/*.cpp)
SRC += $(wildcard ./communication/*.cpp)
SRC += $(wildcard ./plan/*.cpp)
SRC += $(wildcard ./utils/*.cpp)
SRC += $(wildcard ./math/*.cpp)
SRC += $(wildcard ./role/*.cpp)
SRC += $(wildcard ./os/*.cpp)
SRC += $(wildcard ./net/*.cpp)
SRC += $(wildcard ./compile/*.cpp)
BIN = $(SRC:%.cpp=%)
 
#all : ${BIN}
 
#${BIN}:%:%.cpp
	#${CC} $^ -o $@ -I./env -I./action -I./communication -I./plan -I./utils
 
OBJ = $(patsubst %.c, %.o, $(SRC))
 
ALL:RoboCup
 
RoboCup:$(OBJ)
	${CC} -g -Wall -o  $@ -I ./include -I./env -I./action -I./communication -I./plan -I./utils -I./math  -I./role -I./net  -I./os -I./compile $(OBJ) -lpthread
	
COPY: mv ./RoboCup ../

%.o:%.c
	${CC}  -c $< -I ./include -o $@
 
test:
	echo $(SRC)
	echo $(OBJ)
 
.PHONY:clean all
 
clean:
	-@rm -f *.o app ./src/*.o