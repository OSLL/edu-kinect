all: write_files clean
write_files: write_files.o
	g++ -L$(LD_LIBRARY_PATH) -g3 -std=c++11 write_files.o -lfreenect -o write_files
write_files.o: write_files.cpp
	g++ -I$(LIBFREENECT_HEADER) -g3 -std=c++11 -c write_files.cpp
clean: 
	rm *.o 
