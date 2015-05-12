################################################################################
# Robot Simulator Makefile, written by SIDNEY RDC, 2014
# Last Change: 2015 Abr 04 18:02:50
################################################################################
# Compiler definitions --------------------------------------------------------#
CC = g++
CFLAGS = -Wall -Wextra -Wfatal-errors -pedantic -ggdb3 -std=c++0x -O0
LDFLAGS = -L../lib -Wl,-rpath=../lib
LIBS = -lgurobi_g++4.1 -lgurobi56 -larmadillo -lQuadProgpp -lhdf5 -lhdf5_hl -lros_interface
INCLUDES = -I./

# Main directories ------------------------------------------------------------#
OBJDIR = ../obj
OUTDIR = ../bin

# Main files ------------------------------------------------------------------#
SOURCES = $(wildcard *.cpp)
OBJECTS = $(addprefix $(OBJDIR)/, $(SOURCES:.cpp=.o))
EXECUTABLE = $(OUTDIR)/main

# Rules -----------------------------------------------------------------------#
# $@ = term before :
# $^ = all terms after :
# $< = first term after :
all: $(SOURCES) $(EXECUTABLE)
	@echo All simulator sources are compiled!

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(LDFLAGS) $^ -o $@ $(LIBS) $(INCLUDES)

$(OBJDIR)/%.o: %.cpp
	$(CC) -c $< -o $@ $(CFLAGS)

clean:
	$(RM) $(OBJECTS) $(EXECUTABLE)

