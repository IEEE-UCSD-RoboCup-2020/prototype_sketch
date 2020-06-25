
default: udp_speak_to_grsim.exe
#multithreading_strand.exe

boostlib = -lboost_system -lboost_thread -lpthread 

std = -std=c++17

cppflags = $(std) $(boostlib)
 
%.o: %.cpp
	g++ -c $< 

%.exe: %.o
	g++ -o $@ $< $(cppflags) 
	@rm *.o
	@echo compilation completed

clean:
	@rm *.exe