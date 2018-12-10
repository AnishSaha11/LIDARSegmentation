IncDirectories = /Users/anishsaha/Documents/SegmentationAndRendering/src/segmentation/Eigen
CXX = g++
output: main.o SceneManager.o SceneModel.o
	$(CXX) main.o SceneManager.o SceneModel.o -o output

main.o: main.cpp
	$(CXX) -c main.cpp

SceneManager: SceneManager.cpp
	$(CXX) -c SceneManager.cpp

SceneModel: SceneModel.cpp
	$(CXX) -I $(IncDirectories) -c SceneModel.cpp

clean:
	rm *.o output
