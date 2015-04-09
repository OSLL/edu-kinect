all: a.exe clean
a.exe: begin.o
	g++ -std=c++11 write_files.o -lfreenect -o a.exe
write_files.o:
	g++ -c write_files.cpp
clean: 
	rm *.o 
