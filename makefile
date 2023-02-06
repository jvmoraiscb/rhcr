all: compiler linker build

run:
	@sudo ./build/main.out

build:
	@mkdir build
	@mv *.o build
	@mv main.out build

compiler:
	@g++ -I/usr/local/include -c src/*.cpp

linker:
	@g++ -L/usr/local/lib -o main.out *.o -lnifalcon -lnifalcon_cli_base -lnifalcon_device_thread

clean:
	@rm -r build