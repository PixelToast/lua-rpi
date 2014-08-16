none:
	gcc -std=c99 -g -Wall -fPIC --shared -o rpi.so rpi.c -I/usr/include -I/usr/include/lua5.1
install:
	cp rpi.so /usr/local/lib/lua/5.1/
