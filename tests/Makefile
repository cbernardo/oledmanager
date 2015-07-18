CXXFLAGS = -Wall
CPPFLAGS := -I . -I ../core

VPATH := $(CPPFLAGS)

HDRS := commif.h comport.h oled.h
SRC := testoled.cpp

.PHONY : all
all : objs test

OBJS := oled.o comport.o
.PHONY : objs
objs : $(OBJS)

.PHONY : test
test : testoled testtouch

testoled : testoled.cpp objs $(HDRS) test.img
	g++ $(CXXFLAGS)  -pthread -lm $(CPPFLAGS) $(OBJS) $< -o $@

testtouch : testtouch.cpp objs $(HDRS)
	g++ $(CXXFLAGS)  -pthread -lm $(CPPFLAGS) $(OBJS) $< -o $@

oled.o : oled.cpp $(HDRS)
	g++ $(CXXFLAGS) $(CPPFLAGS) -c $< -o $@

comport.o : comport.cpp commif.h comport.h
	g++ $(CXXFLAGS) $(CPPFLAGS) -c $< -o $@

.PHONY : clean
clean :
	-rm *.o testoled testtouch
