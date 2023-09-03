TARGET = send_file

CXX = g++
CXXFLAGS = -g -Wall -std=c++17 -O0
HEADERS = socket_utils.h 
OBJS = send_file.o socket_utils.o

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS)

%.o: %.cc $(HEADERS)
	$(CXX) $(CXXFLAGS) -c $<

clean:
	@rm -f $(OBJS) $(TARGET)

.PHONY: all clean