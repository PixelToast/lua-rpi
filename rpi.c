// raspberry pi library for Lua by PixelToast
// public domain

// TODO:
//  pi model detection
//  gpio pin restrictions
//  onChange pin function
//  PWM
//  I2C


#define UARTfile "/dev/ttyAMA0"


#include "lua.h"
#include "lauxlib.h"
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <poll.h>
#include <string.h>
#include <sys/select.h>

void settable(lua_State *L,char *name,void *func) {
	lua_pushstring(L,name);
	lua_pushcfunction(L,func);
	lua_settable(L,-3);
}

//////////////////////////////////////////// GPIO pins ////////////////////////////////////////////

volatile int *gpiomem=0;
int mem=0;

static int lrpi_openpins(lua_State *L) {
	if (mem>0) {
		close(mem);
	}
	mem=open("/dev/mem",2,0);
	if (mem<0) {
		luaL_error(L,"must be run as root to use GPIO pins");
	}
	if (gpiomem>0) {
		munmap((void*)gpiomem,4096);
	}
	gpiomem=mmap(NULL,4096,3,1,mem,0x20200000);
	if (gpiomem==MAP_FAILED) {
		gpiomem=0;
		luaL_error(L,"mmap failed (%i)",strerror(errno));
	}
	return 0;
}

// -1 is in 1 is out
int dirs[32]={0};
void pin_setdir(int pin, int dir) {
	if (dirs[pin]!=dir) {
		dirs[pin]=dir;
		dir=dir==1?1:0;
		int idx=pin%10;
		gpiomem[(pin-idx)/10]=((gpiomem[(pin-idx)/10])&((7<<(idx*3))))|(dir<<(idx*3));
	}
}

void pin_isvalid(lua_State *L,int pin) {
	if (gpiomem==0) {
		lrpi_openpins(L);
	}
}

static int lrpi_pin_newindex(lua_State *L) {
	int pin=luaL_checkinteger(L,2);
	pin_isvalid(L,pin);
	int val=luaL_checkinteger(L,3);
	if ((val!=0)&(val!=1)) {
		luaL_error(L,"value must be 1 or 0");
	}
	pin_setdir(pin,1);
	gpiomem[10-val*3]=1<<pin;
	return 0;
}

static int lrpi_pin_index(lua_State *L) {
	int pin=luaL_checkinteger(L,2);
	pin_isvalid(L,pin);
	pin_setdir(pin,-1);
	lua_pushinteger(L,(gpiomem[13]>>pin)&1);
	return 1;
}

void pin_load(lua_State *L) {
	settable(L,"openpins",lrpi_openpins);
	lua_pushstring(L,"pin");
	lua_newtable(L);
	lua_newtable(L);
	settable(L,"__newindex",lrpi_pin_newindex);
	settable(L,"__index",lrpi_pin_index);
	lua_setmetatable(L,-2);
	lua_settable(L,-3);
}

//////////////////////////////////////////// UART ////////////////////////////////////////////

int uartfs=-1;
struct termios uartfs_opt;

int validbaud[][2]={
	{1200,B1200},
	{2400,B2400},
	{4800,B4800},
	{9600,B9600},
	{19200,B19200},
	{38400,B38400},
	{57600,B57600},
	{115200,B115200},
	{230400,B230400},
	{460800,B460800},
	{500000,B500000},
	{576000,B576000},
	{921600,B921600},
	{1000000,B1000000},
	{0,0}
};

static int lrpi_uart_open(lua_State *L) {
	if (uartfs>0) {
		close(uartfs);
	}
	uartfs=open(UARTfile,O_RDWR|O_NOCTTY|O_NDELAY);
	if (uartfs<0) {
		luaL_error(L,"error opening %s (%s)",UARTfile,strerror(errno));
	}
	tcgetattr(uartfs,&uartfs_opt);
	int baud=0;
	if (lua_gettop(L)>0) {
		int nbaud=luaL_checkinteger(L,1);
		for (int i=0;validbaud[i][0]!=0;i++) {
			if (validbaud[i][0]==nbaud) {
				baud=validbaud[i][1];
				break;
			}
		}
		if (baud==0) {
			luaL_error(L,"invalid baud rate");
		}
	} else {
		baud=B115200;
	}
	uartfs_opt.c_cflag=baud|CS8|CLOCAL|CREAD;
	uartfs_opt.c_iflag=IGNPAR;
	uartfs_opt.c_oflag=0;
	uartfs_opt.c_lflag=0;
	tcflush(uartfs,TCIFLUSH);
	tcsetattr(uartfs,TCSANOW,&uartfs_opt);
	return 0;
}

static int lrpi_uart_send(lua_State *L) {
	if (uartfs<0) {
		luaL_error(L,"UART not opened");
	}
	const char *data=luaL_checkstring(L,1);
	if (write(uartfs,data,lua_objlen(L,1))<0) {
		if (errno==EAGAIN) {
			lua_pushboolean(L,0);
			return 1;
		}
		luaL_error(L,"error sending (%s)",strerror(errno));
	}
	lua_pushboolean(L,1);
	return 1;
}

static int lrpi_uart_areceive(lua_State *L) {
	if (uartfs<0) {
		luaL_error(L,"UART not opened");
	}
	lua_pushstring(L,"");
	char buffer[256];
	while (1) {
		int len=read(uartfs,buffer,255);
		if (len<1) {
			break;
		}
		lua_pushlstring(L,buffer,len);
		lua_concat(L,2);
	}
	return 1;
}

static int lrpi_uart_receive(lua_State *L) {
	fd_set rfds;
	FD_ZERO(&rfds);
	FD_SET(uartfs,&rfds);
	if (lua_gettop(L)>0) {
		double timeout=luaL_checkinteger(L,1);
		struct timeval tv;
		tv.tv_sec=timeout;
		tv.tv_usec=(int)(timeout*1000)%10000;
		select(uartfs+1,&rfds,NULL,NULL,&tv);
	} else {
		select(uartfs+1,&rfds,NULL,NULL,NULL);
	}
	return lrpi_uart_areceive(L);
}

static int lrpi_uart_getfd(lua_State *L) {
	lua_pushinteger(L,uartfs);
	return 1;
}

void uart_load(lua_State *L) {
	lua_pushstring(L,"uart");
	lua_newtable(L);
	settable(L,"open",lrpi_uart_open);
	settable(L,"send",lrpi_uart_send);
	settable(L,"receive",lrpi_uart_receive);
	settable(L,"areceive",lrpi_uart_areceive);
	settable(L,"getfd",lrpi_uart_getfd);
	lua_settable(L,-3);
}

////////////////////////////////////////////////////////////////////////////////////////

static int lrpi_sleep(lua_State *L) {
	poll(NULL,0,luaL_checknumber(L,1)*1000);
	return 0;
}

extern int luaopen_rpi(lua_State *L) {
	lua_newtable(L);
	settable(L,"wait",lrpi_sleep);
	settable(L,"sleep",lrpi_sleep);
	pin_load(L);
	uart_load(L);
	return 1;
}

