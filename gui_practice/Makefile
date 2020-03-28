EXE := gui
CC = g++

SRC_DIR := src
OBJ_DIR := obj

SRC := $(wildcard $(SRC_DIR)/*.cpp)
OBJ := $(SRC:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/%.o)

CPPFLAGS := -Iinc `wx-config --cxxflags`
CFLAGS   := -Wall
LDFLAGS  :=
LDLIBS   := -lGL -lGLU -lglut `wx-config --libs std,gl`

all: $(EXE)

$(EXE): $(OBJ)
	$(CC) $(LDFLAGS) $^ -o $@ $(LDLIBS)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp | $(OBJ_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

$(OBJ_DIR):
	mkdir $@

clean:
	$(RM) -r $(OBJ_DIR)
	$(RM) -r ./inc/*.h.gch