CXX=g++
CXXFLAGS=-std=gnu++17  -Wall -I/usr/include/eigen3
BINS=eigen_00_basics_test.bin eigen_01_point_loading_test.bin \
eigen_02_isometry_test.bin eigen_03_covariance_test.bin \
eigen_04_kdtree_test.bin
.phony:	clean all

all:	$(BINS)

%.bin: 	%.cpp
	$(CXX) $(CXXFLAGS) -o $@ $^

clean:
	rm -rf *~ *.o $(BINS)
