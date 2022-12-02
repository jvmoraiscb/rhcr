all: compiler linker
	@rm main.o
	@sudo ./main.out

compiler:
	@g++ -I/usr/local/include -c src/main.cpp

linker:
	@g++ -L/usr/local/bin -o main.out main.o -lnifalcon -lnifalcon_cli_base -lnifalcon_device_thread
	@export LD_LIBRARY_PATH=/usr/local/lib

clean:
	@rm main.out