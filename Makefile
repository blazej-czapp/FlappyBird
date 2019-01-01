all:
	@g++ -o flappy flappy.cpp arm.cpp bird.cpp camera.cpp world.cpp -L/usr/lib/x86_64-linux-gnu/ -L/usr/local/lib -lk8055 -lusb -lpthread `pkg-config --libs opencv`
	@sudo ./flappy