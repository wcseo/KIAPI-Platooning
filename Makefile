APP_NAME ?= obu-keti-platooning
LIB_NAME ?= lib

CC ?= gcc
CXX ?= g++
AR ?= ar

BASE_TARGET ?= 
BDIR ?= ./build$(BASE_TARGET)
SDIR ?= ./src
INCDIR = ./include 

$(shell mkdir -p $(BDIR))

BUILD_DATE = $(shell date '+%y%m%d.%H%M%S') 
SW_VERSION = 0.0.0.1
DEF = -D BUILD_DATE='"$(BUILD_DATE)"' -D SW_VERSION='"$(SW_VERSION)"'

DEBUG = -O0 -g -Werror=return-type

OBJ ?=  main.o obu_handler.o platooning_service.o utils.o
OBJS = $(patsubst %,$(BDIR)/%,$(OBJ))             

INCLUDE += -I $(INCDIR) -I ../../cpp-framework/include/ -I ../libnr-v2x/include/ -I ../../dgd-sock/include/
LIB ?= -L ../../cpp-framework/$(BDIR)/ -L ../libnr-v2x/$(BDIR)/ -L ../../dgd-sock/$(BDIR)/ -ldgd-sock -lnr-v2x -lcpp-framework -pthread
    
all : app  
 
app: $(OBJS)
	$(CXX) -o $(BDIR)/$(APP_NAME) $(OBJS) $(LDFLAGS) $(LIB)

lib: $(OBJS)
	$(AR) rc $(BDIR)/$(LIB_NAME) $^

$(BDIR)/%.o: $(SDIR)/%.cpp
	$(CXX) $< -o $@ -c $(CXXFLAGS) $(INCLUDE) $(DEF) $(DEBUG) 
	
$(BDIR)/%.o: $(SDIR)/%.cc
	$(CXX) $< -o $@ -c $(CXXFLAGS) $(INCLUDE) $(DEF) $(DEBUG)

clean: 
	rm -r ./build* 