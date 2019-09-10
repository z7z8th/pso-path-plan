CC=g++
CFLAGS=-g -O2 -Wall -Wextra -Wconversion -Wtraditional -Wtraditional-conversion -Wunsafe-loop-optimizations 
CXXFLAGS=-g -O2 -Wall -Wextra
#-Wconversion -Wunsafe-loop-optimizations 
#obj_dir=.objs
s_objs=main.o
m_objs=pso_plan.o
OBJS=$(s_objs) $(m_objs)
LIBS=-lpthread -lQtGui -lQtCore
TARGET=pso_planer
EXTRAFLAGS=-Wp,-D_FORTIFY_SOURCE=2 -fstack-protector --param=ssp-buffer-size=4 -m32 -march=i686 -fasynchronous-unwind-tables -D_REENTRANT -DQT_NO_DEBUG -DQT_GUI_LIB -DQT_CORE_LIB -I/usr/lib/qt4/mkspecs/linux-g++ -I. -I/usr/include/QtCore -I/usr/include/QtGui -I/usr/include -I.

all: $(TARGET)

.PHONEY: all clean

$(TARGET): $(OBJS)
	$(CC) $(CXXFLAGS) -o $@ $(OBJS) $(LIBS)

$(s_objs) : %.o : %.cpp
	$(CC)  $(EXTRAFLAGS) $(CXXFLAGS) -c $< -o $@

$(m_objs): %.o : %.cpp %.h
	$(CC)  $(EXTRAFLAGS) $(CXXFLAGS) -c $< -o $@

clean:
	-rm -rf *.o $(TARGET) tags *.png
