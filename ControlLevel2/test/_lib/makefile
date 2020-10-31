# Makefile to generate OpenGL Graphic Program
.SUFFIXES: .cpp .o

SRCS = $(shell ls *.cpp)
SRCS_HDRS = $(shell ls *.h)
OBJS = $(SRCS:.cpp=.o)

CC := g++
CFLAGS = -O2 -I/usr/include/opengl

all: $(OBJS)

.cpp.o:
	$(CC) -c $(CFLAGS) $<

basic_calc.o:      basic_calc.cpp basic_calc.h
	$(CC) -c $(CFLAGS) $<

basic_matrix.o:    basic_matrix.cpp basic_matrix.h
	$(CC) -c $(CFLAGS) $<

basic_figure.o:    basic_figure.cpp basic_figure.h colors.h
	$(CC) -c -O2 -I/usr/include/opengl $<

basic_opengl.o:    basic_opengl.cpp basic_opengl.h colors.h
	$(CC) -c -O2 -I/usr/include/opengl $<

bmp_output.o:      bmp_output.cpp bmp_output.h
	$(CC) -c -O2 -I/usr/include/opengl $<

clean:
	@rm -rf *.o

