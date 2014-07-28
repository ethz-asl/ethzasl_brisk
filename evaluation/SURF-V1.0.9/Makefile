
# ------------------ Compilation options ------------------------

# Loads math library.  
LIBS = -lm
GET = get
CXXFLAGS = -Wall -O3 -DNO_DEBUG -march=i686 -mfpmath=sse -mmmx -msse -msse2 -msse3 -ansi -m32
DEPEND= makedepend $(CFLAGS)

CC = g++
CXX = g++

# --------------------- Code modules ----------------------------

# Source files
SRCS = main.cpp imload.cpp os_mapping.cpp
# Object files
OBJ = main.o imload.o os_mapping.o
# Definitions
DEFS = image.h fasthessian.h ipoint.h surf.h imload.h

# ------------------------ Rules --------------------------------
#$(SRCS):
#        $(GET) $@

# Link against static library
surf.ln: ${OBJ} libSurf.so
	${CC} -o $@ ${CXXFLAGS} main.o imload.o os_mapping.o -static libSurf.a ${LIBS}

# Small matching demo application

match.ln: match.cpp imload.o 
	${CC} -o $@ ${CXXFLAGS} imload.o match.cpp -static libSurf.a -lm

# To link against a shared library, use
#surf.ln: ${OBJ} libSurf.so
#	${CC} -o $@ ${CFLAGS} main.o imload.o -L. -lSurf ${LIBS}
# Note to set LD_LIBRARY_PATH environment variable before running surf.ln

clean:
	-rm *.o surf.ln match.ln

#depend: $(SRCS)
#        $(DEPEND) $(SRCS)
