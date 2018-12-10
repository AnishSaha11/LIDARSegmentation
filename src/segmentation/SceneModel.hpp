//
//  SceneModel.hpp
//  TrialGraphics
//
//  Created by Anish Saha on 10/21/18.
//  Copyright © 2018 Anish Saha. All rights reserved.
//

#ifndef SceneModel_hpp
#define SceneModel_hpp
#include <map>
#include <stdio.h>
#include <string>
#include <vector>
#include <ctime>
struct Vertex{
    float x,y,z;
    int label;
};
struct Feature{
    //Geometric:
    float area;
    float edgeRatio;
    float maxEdge;
    std::vector<float> Covariance;
    //Orientation and location:
    float height;
    std::vector<float> normal;
    //3D features:
    float density;
    int label;
    bool train;
};
class SceneModel{
private:
    std::string sceneId;
    std::string vertexType;
    std::vector<float> GPSxy;
    std::vector<Vertex> vertexList;
    std::map<std::string,std::vector<Vertex*> > blockMap; //blocks of specified side length for ROAD segmentation
    std::map<std::string,std::vector<Vertex*> > blockMapBuild;
    std::map<std::string,std::vector<Vertex*> > segmentMap; //segments made up of vertices from blocks
    std::map<int,Feature> featureMap;
    float sceneMaxX,sceneMaxY,sceneMaxZ;
    float sceneMinX,sceneMinY,sceneMinZ;
    long int vertexCount;
    std::map<std::string,float> buildingScore;
    std::vector<std::string> bScoreValidKeys;
    std::vector<std::vector<Vertex*> > miscObjects;
    std::map<int,std::vector<Vertex*> > superVoxelMap;
    time_t kmeansRuntime,superVoxelRuntime;
    
    
public:
    SceneModel(std::string,std::string,float,float);
    void populateVertices(std::string,bool train=false);
    void printHeadVertexList();
    std::string getSceneId();
    std::vector<float> getGPSxy();
    long int getVertexCount();
    std::vector<Vertex*> getBlock(std::string);
    std::map<std::string,std::vector<Vertex*> > getBlockMap();
    std::map<std::string,std::vector<Vertex*> > getSegment();
    void blockify(float sideLength);
    void blockifyForBuild(float sideLength);
    std::string keyGenerator(Vertex*,float*);
    void fitRansacRoadPlane();
    void sortBlockVertices();
    void getPlane(Vertex*,Vertex*,Vertex*,float*);
    void getVoteAndStd(float*,float*);
    void writeSegmentToFile(std::string,std::string);
    void displaySceneStats();
    int getNumIterForRANSAC(int);
    int factorial(int);
    void getBuildingScore();
    void getBuildingShapeFeatures();
    void cluster(std::string);
    int getClusterIndex(std::vector<Vertex>,float,float,float);
    void writeClustersToFile(std::map<int,std::vector<Vertex*> >,std::string);
    float euclidDistance(float,float,float,float,float,float);
    void makeSuperVoxels(std::map<int,std::vector<Vertex*> >,std::vector<Vertex>,std::string);
    int getEigenAngle(std::vector<float>,std::vector<float>);
    int isEigenSimilar(std::vector<Vertex*>,std::vector<Vertex*>);
    std::vector<float> getEigenVector(std::vector<Vertex*>);
    void writeSuperVoxelsToFile(std::string);
    void computeSceneFeatures();
    Feature getFeatures(std::vector<Vertex*>);
    void buildMapForTrain();
    time_t getKMeansRuntime();
    time_t getSuperVoxelsRuntime();
};
#endif /* SceneModel_hpp */
