all: a.exe clean
a.exe: begin.o
	g++ begin.o -lfreenect -o a.exe
begin.o:
	g++ -c begin.cpp
clean: 
	rm *.o 
