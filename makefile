all: compiler linker
	@rm main.o
	@sudo ./main.out

compiler:
	@g++ -I/usr/local/include -c src/main.cpp

linker:
	@g++ -L/usr/local/lib -o main.out main.o -lnifalcon -lnifalcon_cli_base -lnifalcon_device_thread

clean:
	@rm main.out