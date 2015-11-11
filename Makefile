TARGET=esp8266start

CXXFLAGS=-std=c++11

OBJS=\
	main.o \
	serial.o

$(TARGET): $(OBJS)
	g++ $^ -o $@


clean:
	-rm $(TARGET)
	-rm $(OBJS)

