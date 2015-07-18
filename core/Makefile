CXXFLAGS = -Wall
CPPFLAGS := -I .

.PHONY : all
all : objs

OBJS := oled.o comport.o
.PHONY : objs
objs : $(OBJS)

oled.o : oled.cpp oled.h commif.h comport.h
	g++ $(CXXFLAGS) $(CPPFLAGS) -c $< -o $@

comport.o : comport.cpp commif.h comport.h
	g++ $(CXXFLAGS) $(CPPFLAGS) -c $< -o $@

.PHONY : clean
clean :
	-rm *.o
