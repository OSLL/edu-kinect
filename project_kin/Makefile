all: write_files clean
write_files: write_files.o
	g++ -std=c++11 write_files.o -lfreenect -o write_files
write_files.o:
	g++ -c write_files.cpp
clean: 
	rm *.o 
