CXX=g++
#CFLAGS=-g -O2 -Wall -Wextra -Wconversion -Wtraditional -Wtraditional-conversion -Wunsafe-loop-optimizations 
CXXFLAGS=-g -O0 -Wall -Wextra
#-Wconversion -Wunsafe-loop-optimizations 
#obj_dir=.objs
s_objs=main.o
m_objs=pso_plan.o
OBJS=$(s_objs) $(m_objs)
LIBS=-lpthread -lQtGui -lQtCore
TARGET=pso_path_plan
#EXTRAFLAGS=-Wp,-D_FORTIFY_SOURCE=2 -fstack-protector --param=ssp-buffer-size=4 -fasynchronous-unwind-tables -D_REENTRANT -DQT_NO_DEBUG -DQT_GUI_LIB -DQT_CORE_LIB -I/usr/lib/qt4/mkspecs/linux-g++ -I. -I/usr/include/qt4/QtCore -I/usr/include/qt4/QtGui -I/usr/include/qt4 -I/usr/include -I.
EXTRAFLAGS=-I/usr/lib/qt4/mkspecs/linux-g++ -I/usr/include/qt4/QtCore -I/usr/include/qt4/QtGui -I/usr/include/qt4 -I/usr/include

all: $(TARGET)

.PHONEY: all clean

$(TARGET): $(OBJS) Makefile
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS) $(LIBS)

$(s_objs) : %.o : %.cpp
	$(CXX)  $(EXTRAFLAGS) $(CXXFLAGS) -c $< -o $@

$(m_objs): %.o : %.cpp %.h
	$(CXX)  $(EXTRAFLAGS) $(CXXFLAGS) -c $< -o $@

clean:
	-rm -rf *.o $(TARGET) tags *.png

rebuild: clean
	make all