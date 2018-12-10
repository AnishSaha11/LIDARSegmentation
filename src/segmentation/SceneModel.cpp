//
//  SceneModel.cpp
//  TrialGraphics
//
//  Created by Anish Saha on 10/21/18.
//  Copyright © 2018 Anish Saha. All rights reserved.
//
#define SIDE 10
#define MAXVAL 65536
#define MINVAL -65536
#define THRESH1 0.8
#define THRESH2 0.3 //0.26 works
#define BSCORETHRESH 0.8
#define SHAPETHRESH 0.25  //0.25 work
#define CLUSTDISTTHRESH 10
#define EIGENANGLETHRESH 15
#define NUMCLUSTERS 100
#include <list>
#include <math.h>
#include <fstream>
#include "SceneModel.hpp"
#include <iostream>
#include <sstream>
#include <vector>
#include <iomanip>
#include "Eigen/Dense"
#include <queue>

using namespace Eigen;
SceneModel::SceneModel(std::string sceneId,std::string vertexType, float GPSx,float GPSy){
    this->sceneId=sceneId;
    this->vertexType=vertexType;
    GPSxy.push_back(GPSy);
    GPSxy.push_back(GPSx);
    this->vertexCount=0;
    
}
void SceneModel::populateVertices(std::string filename,bool isTrain){
    std::ifstream infile;
    std::cout<<"Starting vertex population from file: "<<filename<<std::endl;
    float x;
    long int count=0;
    infile.open(filename);
    if(infile.is_open()){
        for(std::string line;getline(infile,line);){
            std::stringstream ss(line);
            getline(ss,line,',');
            x=std::stof(line);
            struct Vertex v;
            v.x=x;
            getline(ss,line,',');
            x=std::stof(line);
            v.y=x;
            getline(ss,line,',');
            x=std::stof(line);
            v.z=x;
            if(isTrain)
            {
                getline(ss,line,',');
                x=std::stof(line);
                v.label=x;
                
                int miscLabels[]={1202,1203,1204,1205,1206,1400,1401,1402,1403,1405,1406,1411};
                if(std::find(std::begin(miscLabels),std::end(miscLabels),x)!=std::end(miscLabels)==true){
                    continue;
                }
            }
            vertexList.push_back(v);
            count++;
        }
        infile.close();
        
    }
    else{
        std::cout<<"Unable to open file";
    }
    vertexCount+=count;
    std::cout<<"Vertex count stands at: "<<vertexCount<<std::endl;
}
void SceneModel::printHeadVertexList(){
    int count=0;
    for(std::vector<Vertex>::iterator it=vertexList.begin();it!=vertexList.end();++it,++count){
        std::cout<<"x: "<<it->x<<" y: "<<it->y<<" z: "<<it->z<<std::endl;
    if(count==5)
        return;
    }
}
std::string SceneModel::getSceneId(){
    return sceneId;
}
std::vector<float> SceneModel::getGPSxy(){
    return GPSxy;
}
long int SceneModel::getVertexCount(){
    return vertexCount;
}
void SceneModel::blockifyForBuild(float sideLength){
    float minMax[]={MAXVAL,MINVAL,MAXVAL,MINVAL,MAXVAL,MINVAL};
    
    std::vector<Vertex*> RoadCompList = segmentMap["ROADComp"];
    
    printHeadVertexList();
    for(int i=0;i<RoadCompList.size();i++){
        if(RoadCompList[i]->x<minMax[0])
            minMax[0]=RoadCompList[i]->x;
        if(RoadCompList[i]->x>minMax[1])
            minMax[1]=RoadCompList[i]->x;
        if(RoadCompList[i]->y<minMax[2])
            minMax[2]=RoadCompList[i]->y;
        if(RoadCompList[i]->y>minMax[3])
            minMax[3]=RoadCompList[i]->y;
        if(RoadCompList[i]->z<minMax[4])
            minMax[4]=RoadCompList[i]->z;
        if(RoadCompList[i]->z>minMax[5])
            minMax[5]=RoadCompList[i]->z;
    }
    std::cout<<"X min is: "<<minMax[0]<<std::endl;sceneMinX=minMax[0];
    std::cout<<"X max is: "<<minMax[1]<<std::endl;sceneMaxX=minMax[1];
    std::cout<<"Y min is: "<<minMax[2]<<std::endl;sceneMinY=minMax[2];
    std::cout<<"Y max is: "<<minMax[3]<<std::endl;sceneMaxY=minMax[3];
    std::cout<<"Z min is: "<<minMax[4]<<std::endl;sceneMinZ=minMax[4];
    std::cout<<"Z max is: "<<minMax[5]<<std::endl;sceneMaxZ=minMax[5];
    float xRange=(minMax[1]-minMax[0]);
    float yRange=(minMax[3]-minMax[2]);
    float zRange=(minMax[5]-minMax[4]);
    std::cout<<"X range is: "<<xRange<<std::endl;
    std::cout<<"Y range is: "<<yRange<<std::endl;
    std::cout<<"Z range is: "<<zRange<<std::endl;
    float safeRange[4];
    safeRange[0]=((minMax[0]+minMax[1])/2)-(round((xRange+SIDE)/SIDE)*SIDE)/2;
    safeRange[1]=((minMax[0]+minMax[1])/2)+(round((xRange+SIDE)/SIDE)*SIDE)/2;
    safeRange[2]=((minMax[2]+minMax[3])/2)-(round((yRange+SIDE)/SIDE)*SIDE)/2;
    safeRange[3]=((minMax[2]+minMax[3])/2)+(round((yRange+SIDE)/SIDE)*SIDE)/2;
    
    std::cout<<"X min rounded is: "<<safeRange[0]<<std::endl;
    std::cout<<"X max rounded is: "<<safeRange[1]<<std::endl;
    std::cout<<"Y min rounded is: "<<safeRange[2]<<std::endl;
    std::cout<<"Y max rounded is: "<<safeRange[3]<<std::endl;
    
    //divide all points in x-y buckets of side length SIZE
    
    int count=0;
    for(int i=0;i<RoadCompList.size();i++){
        std::string key=keyGenerator(RoadCompList[i],safeRange);
        if(blockMapBuild.count(key)==0){
            blockMap[key];
            std::cout<<"Block created with key: "<<key<<std::endl;
            count++;
        }
        blockMapBuild[key].push_back(RoadCompList[i]);
    }
    std::cout<<"Total blocks created: "<<count<<std::endl;
    
}

void SceneModel::blockify(float sideLength){
    float minMax[]={MAXVAL,MINVAL,MAXVAL,MINVAL,MAXVAL,MINVAL};
    printHeadVertexList();
    for(int i=0;i<vertexList.size();i++){
        if(vertexList[i].x<minMax[0])
            minMax[0]=vertexList[i].x;
        if(vertexList[i].x>minMax[1])
            minMax[1]=vertexList[i].x;
        if(vertexList[i].y<minMax[2])
            minMax[2]=vertexList[i].y;
        if(vertexList[i].y>minMax[3])
            minMax[3]=vertexList[i].y;
        if(vertexList[i].z<minMax[4])
            minMax[4]=vertexList[i].z;
        if(vertexList[i].z>minMax[5])
            minMax[5]=vertexList[i].z;
    }
    std::cout<<"X min is: "<<minMax[0]<<std::endl;sceneMinX=minMax[0];
    std::cout<<"X max is: "<<minMax[1]<<std::endl;sceneMaxX=minMax[1];
    std::cout<<"Y min is: "<<minMax[2]<<std::endl;sceneMinY=minMax[2];
    std::cout<<"Y max is: "<<minMax[3]<<std::endl;sceneMaxY=minMax[3];
    std::cout<<"Z min is: "<<minMax[4]<<std::endl;sceneMinZ=minMax[4];
    std::cout<<"Z max is: "<<minMax[5]<<std::endl;sceneMaxZ=minMax[5];
    float xRange=(minMax[1]-minMax[0]);
    float yRange=(minMax[3]-minMax[2]);
    float zRange=(minMax[5]-minMax[4]);
    std::cout<<"X range is: "<<xRange<<std::endl;
    std::cout<<"Y range is: "<<yRange<<std::endl;
    std::cout<<"Z range is: "<<zRange<<std::endl;
    float safeRange[4];
    safeRange[0]=((minMax[0]+minMax[1])/2)-(round((xRange+SIDE)/SIDE)*SIDE)/2;
    safeRange[1]=((minMax[0]+minMax[1])/2)+(round((xRange+SIDE)/SIDE)*SIDE)/2;
    safeRange[2]=((minMax[2]+minMax[3])/2)-(round((yRange+SIDE)/SIDE)*SIDE)/2;
    safeRange[3]=((minMax[2]+minMax[3])/2)+(round((yRange+SIDE)/SIDE)*SIDE)/2;
   
    std::cout<<"X min rounded is: "<<safeRange[0]<<std::endl;
    std::cout<<"X max rounded is: "<<safeRange[1]<<std::endl;
    std::cout<<"Y min rounded is: "<<safeRange[2]<<std::endl;
    std::cout<<"Y max rounded is: "<<safeRange[3]<<std::endl;
   
    //divide all points in x-y buckets of side length SIZE
    int count=0;
    for(int i=0;i<vertexList.size();i++){
        std::string key=keyGenerator(&vertexList[i],safeRange);
        if(blockMap.count(key)==0){
            blockMap[key];
            std::cout<<"Block created with key: "<<key<<std::endl;
            count++;
        }
        blockMap[key].push_back(&vertexList[i]);
    }
    std::cout<<"Total blocks created: "<<count<<std::endl;
    
}
std::string SceneModel::keyGenerator(Vertex *v, float *arr){
    int x=(v->x-arr[0])/SIDE;
    int y=(v->y-arr[2])/SIDE;
    //int z=(v->z-arr[4])/SIDE;
    std::string key=std::to_string(x)+":"+std::to_string(y);
    return key;
}
void SceneModel::fitRansacRoadPlane(){
    segmentMap["ROAD"];
    segmentMap["ROADComp"];
    sortBlockVertices();
    //Iterate overall blocks that have more than 3 points
    for(std::map<std::string,std::vector<Vertex*> >:: iterator it=blockMap.begin();it!=blockMap.end();++it){
        //for all blocks that contain more than 3 points,proceed to clean before plane fitting.
        if(it->second.size()>3){   //make sure min is 3
            //create new list of z values for pre clean.
            std::list<Vertex*> vList;
            for(int i=0;i<it->second.size();i++){
                vList.push_back(it->second[i]);
            }
            int c=1;
            float sum=0;
            long int check=(vList.size()>10)?vList.size()-10:0;
            for(std::list<Vertex*>::iterator iter=vList.begin();iter!=vList.end();++iter){
                if(check==0){
                    sum+=(*iter)->z;
                    c++;
                }
                else{
                check--;
                }
            }
            //get average of the minimum 10/c z-values
            float avg=sum/c;
            std::cout<<"Computed average is: "<<avg<<std::endl;
            //remove all vertices in vlist with z more than threshold distance from min-avg-z value
            for(auto iter=vList.begin();iter!=vList.end();){
                float absDiff=abs(((*iter)->z)-avg);
                std::cout<<"Checking point: "<<(*iter)->x<<","<<(*iter)->y<<","<<(*iter)->z<<std::endl;
                std::cout<<"Absolute diff is: "<<absDiff<<std::endl;
                if(absDiff>THRESH1){
                    std::cout<<"Removing point: "<<(*iter)->x<<","<<(*iter)->y<<","<<(*iter)->z<<std::endl;
                    iter=vList.erase(iter++);
                }
                else{
                    std::cout<<"Accepting point: "<<(*iter)->x<<","<<(*iter)->y<<","<<(*iter)->z<<std::endl;
                    ++iter;
                }
            }
            std::cout<<"Final vList size is: "<<vList.size()<<std::endl;
            std::cout<<"Accepted vertices are:"<<std::endl;
            for(auto iter=vList.begin();iter!=vList.end();++iter){
                std::cout<<(*iter)->x<<","<<(*iter)->y<<","<<(*iter)->z<<std::endl;
                
            }
            //create vector representation of list for fast ransac access.
            std::vector<Vertex*> vVec;
            for(auto iter=vList.begin();iter!=vList.end();++iter){
                vVec.push_back(*iter);
            }
            //free up vList
            vList.clear();
            //now try plane fitting with remaining vertices.
            if(vVec.size()>=3){
                
                 int bestSupport=0;
                 float bestPlane[4];
                //float bestStdDev=MAXVAL;
                 int counter=0;
//int numIter=(vVec.size()>11?45:factorial(vVec.size()));
                 int numIter=15;
                 std::cout<<"Iterating for "<<numIter<<" iterations";
                 float curPlane[4];
                 srand(time(NULL));
                 while(counter<numIter){
                     int randomIndex1=rand()%vVec.size();
                     std::cout<<"Random index 1: "<<randomIndex1<<std::endl;
                     Vertex *p1=vVec[randomIndex1];
                     int randomIndex2=rand()%vVec.size();
                     std::cout<<"Random index 2: "<<randomIndex2<<std::endl;
                     Vertex *p2=vVec[randomIndex2];
                     int randomIndex3=rand()%vVec.size();
                     std::cout<<"Random index 3: "<<randomIndex3<<std::endl;
                     Vertex *p3=vVec[randomIndex3];
                     if(p1==p2 || p1==p3 || p2==p3)
                         continue;
                     getPlane(p1,p2,p3,curPlane);
                     if(curPlane[0]==0.0 && curPlane[1]==0.0 && curPlane[2]==0.0 && curPlane[3]==0.0)
                         continue;
                     //calculate dist for all points within THRESH1
                     int votes=0;
                     
                     for(int i=0;i<vVec.size();i++){
                         
                         float distance=fabs(vVec[i]->x*curPlane[0]+vVec[i]->y*curPlane[1]+vVec[i]->z*curPlane[2]+curPlane[3])/(sqrt(curPlane[0]*curPlane[0]+curPlane[1]*curPlane[1]+curPlane[2]*curPlane[2]));
                         if(distance<=THRESH1){
                             std::cout<<"Point: "<<vVec[i]->x<<","<<vVec[i]->y<<","<<vVec[i]->z<<" voted as a fit in iteration: "<<counter+1<<" with distance: "<<distance<<std::endl;
                             votes++;
                         }
                     }
                     ///update params
                     if(votes>bestSupport){
                         std::cout<<"Votes received: "<<votes<<std::endl;
                         bestSupport=votes;
                         bestPlane[0]=curPlane[0];
                         bestPlane[1]=curPlane[1];
                         bestPlane[2]=curPlane[2];
                         bestPlane[3]=curPlane[3];
                     }
                     counter++;
                 }
                 //add points to road list that are THRESH2 within beat plane
                for(int i=0;i<it->second.size();i++){
                    
                    float distance=fabs(it->second[i]->x*bestPlane[0]+it->second[i]->y*bestPlane[1]+it->second[i]->z*bestPlane[2]+bestPlane[3])/sqrt(bestPlane[0]*bestPlane[0]+bestPlane[1]*bestPlane[1]+bestPlane[2]*bestPlane[2]);
                    if(distance<=THRESH2){
                        std::cout<<"Point: "<<it->second[i]->x<<","<<it->second[i]->y<<","<<it->second[i]->z<<" Added to final block road list with distance: "<<distance<<std::endl;
                        segmentMap["ROAD"].push_back(it->second[i]);
                    }
                    else{
                        segmentMap["ROADComp"].push_back(it->second[i]);
                    }
                }
            }
            
        }
    }
}
void SceneModel::sortBlockVertices(){
    for(std::map<std::string,std::vector<Vertex*> >:: iterator it=blockMap.begin();it!=blockMap.end();++it){
        long int vlistSize=it->second.size();
        if(vlistSize!=0){
            for(int i=0;i<(vlistSize-1)&&i<=10;i++){
                for(int j=0;j<(vlistSize-i-1);j++){
                    if((it->second[j])->z<(it->second[j+1])->z){
                        std::swap((it->second[j]),(it->second[j+1]));
                    }
                }
            }
            
        }
    }
}
void SceneModel::getPlane(Vertex *p1,Vertex *p2,Vertex *p3,float *arr){
    std::cout<<"Computing Plane for points:"<<std::endl;
    std::cout<<p1->x<<","<<p1->y<<","<<p1->z<<std::endl;
    std::cout<<p2->x<<","<<p2->y<<","<<p2->z<<std::endl;
    std::cout<<p3->x<<","<<p3->y<<","<<p3->z<<std::endl;
    float a1 = p2->x - p1->x;
    float b1 = p2->y - p1->y;
    float c1 = p2->z - p1->z;
    float a2 = p3->x - p1->x;
    float b2 = p3->y - p1->y;
    float c2 = p3->z - p1->z;
    float a = b1 * c2 - b2 * c1;
    float b = a2 * c1 - a1 * c2;
    float c = a1 * b2 - b1 * a2;
    float d = (- a * p1->x - b * p1->y - c * p1->z);
    std::cout << std::fixed;
    //std::cout << std::setprecision(2);
    std::cout << "equation of plane is "<<a<<"x + "<<b<<"y + "<<c<<"z + "<< d <<" = 0."<<std::endl;
    arr[0]=a;
    arr[1]=b;
    arr[2]=c;
    arr[3]=d;
}
void SceneModel::writeSegmentToFile(std::string category,std::string dir){
    std::ofstream newFile;
    newFile.open(dir+"/"+getSceneId()+"_"+category);
    for(int i=0;i<segmentMap[category].size();i++){
        newFile<<(segmentMap[category])[i]->x<<","<<(segmentMap[category])[i]->y<<","<<(segmentMap[category])[i]->z<<std::endl;
    }
    newFile.close();
}
void SceneModel::displaySceneStats(){
    long int maxPts=0;
    int minPts=100000;
    for(auto iter=blockMap.begin();iter!=blockMap.end();++iter){
        long int blockPts=iter->second.size();
        if(blockPts>maxPts){
            maxPts=blockPts;
        }
        if(blockPts<minPts){
            minPts=blockPts;
        }
    }
    std::cout<<"********************* SCENE: "<<sceneId<<" STATS *************************"<<std::endl;
    std::cout<<"X-Max is: "<<sceneMaxX<<std::endl;
    std::cout<<"X-Min is: "<<sceneMinX<<std::endl;
    std::cout<<"Y-Max is: "<<sceneMaxY<<std::endl;
    std::cout<<"Y-Min is: "<<sceneMinY<<std::endl;
    std::cout<<"Z-Max is: "<<sceneMaxZ<<std::endl;
    std::cout<<"Z-Min is: "<<sceneMinZ<<std::endl;
    std::cout<<"Number of blocks is: "<<blockMap.size()<<std::endl;
    std::cout<<"Avg Point count per blocks is: "<<(100000/blockMap.size())<<std::endl;
    std::cout<<"Sparsest block points count : "<<minPts<<std::endl;
    std::cout<<"Densest block points count : "<<maxPts<<std::endl;
    std::cout<<"***********************************************************************"<<std::endl;
}
int SceneModel::getNumIterForRANSAC(int numOfPts){
    int nfact = factorial(numOfPts);
    int nC2 = nfact/(3*factorial(numOfPts-3));
    return nC2;
}
int SceneModel::factorial(int n){
    int fact=1;
    for(int i=1;i<=n;i++)
        fact=fact*i;
    return fact;
}
void SceneModel::getBuildingScore(){
    segmentMap["RoadBuildComp"];
    segmentMap["Building"];
    float maxHeight=MINVAL;
    float minHeight=MAXVAL;
    float maxBlockRange=0;
    float heightRange=0;;
    float curBlockRange=0;
    float maxDensity=0;
    //find max range from all blocks
    for(auto iter = blockMapBuild.begin();iter!= blockMapBuild.end();++iter){
        if(iter->second.size()>maxDensity)
            maxDensity=iter->second.size();
        for(int i=0;i<iter->second.size();i++){
            if(iter->second[i]->z>maxHeight)
                maxHeight=iter->second[i]->z;
            if(iter->second[i]->z<minHeight)
                minHeight=iter->second[i]->z;
        }
        curBlockRange=maxHeight-minHeight;
        if(curBlockRange>maxBlockRange)
            maxBlockRange=curBlockRange;
    }
    //float normalisedMaxHeight=maxHeight;
    //float normalisedMinHeight=minHeight;
    //if(minHeight<0){
     //   normalisedMaxHeight=maxHeight+abs(minHeight);
      //  normalisedMinHeight=minHeight+abs(minHeight);
    //}
    std::cout<<"Max Range amongst all blocks: "<<maxBlockRange<<std::endl;
    std::cout<<"Max Density amongst all blocks: "<<maxDensity<<std::endl;
    //compute cuilding scores for blocks
    float densityScore=0;
    float heightScore = 0;
    //calculate bScore for each block
    for(auto iter = blockMapBuild.begin();iter!= blockMapBuild.end();++iter){
        
        densityScore=iter->second.size()/maxDensity;
        std::cout<<"Density score for block is: "<<densityScore<<std::endl;
        float curMaxHeight=MINVAL;
        float curMinHeight=MAXVAL;
        for(int i=0;i<iter->second.size();i++){
            if(iter->second[i]->z>curMaxHeight)
                curMaxHeight=iter->second[i]->z;
            if(iter->second[i]->z<curMinHeight)
                curMinHeight=iter->second[i]->z;
        }
        heightScore=(curMaxHeight-curMinHeight)/maxBlockRange;
        std::cout<<"Height score for block is: "<<heightScore<<std::endl;
        float bScore = densityScore+heightScore;
        //buildingScore[iter->first]=bScore;
        std::cout<<"Building score for block is: "<<bScore<<std::endl;
        if(bScore>BSCORETHRESH){
            for(int i=0;i<iter->second.size();i++)
            {
                //segmentMap["Building"].push_back(iter->second[i]);
                buildingScore[iter->first]=bScore;
                
            }
            bScoreValidKeys.push_back(iter->first);
            std::cout<<"Key pushed: "<<iter->first<<std::endl;
        }
        else{
            for(int i=0;i<iter->second.size();i++)
            {
                segmentMap["RoadBuildComp"].push_back(iter->second[i]);
            }
        }
    }
    std::cout<<"Number of blocks in after score computation is: "<<buildingScore.size()<<std::endl;
}
void SceneModel::getBuildingShapeFeatures(){
    for(int i=0;i<=bScoreValidKeys.size();i++){
        std::vector<Vertex*> blockVertices = blockMapBuild[bScoreValidKeys[i]];
        std::cout<<"Vertexes fetched from block: "<<bScoreValidKeys[i]<<std::endl;
        int vertexCount = blockVertices.size();
        if(vertexCount==0)
            continue;
        std::cout<<"Number of vertices in block: "<<bScoreValidKeys[i]<<", is: "<<vertexCount<<std::endl;
        //define matrix
        MatrixXf pointsSet(vertexCount,2);
        float xmean =0;
        float ymean =0;
        for(int j=0;j<vertexCount;j++){
            pointsSet(j,0)=blockVertices[j]->x;
            xmean+=blockVertices[j]->x;
            pointsSet(j,1)=blockVertices[j]->y;
            ymean+=blockVertices[j]->y;
        }
        xmean=xmean/vertexCount;
        ymean=ymean/vertexCount;
        VectorXf mean(2);
        mean<<xmean,ymean;
        pointsSet.rowwise()-=mean.transpose();
        Matrix2f cov_matrix=pointsSet.transpose()*(pointsSet);
        //std::cout<<"Cov matrix: "<<cov_matrix<<std::endl;
        SelfAdjointEigenSolver<Matrix2f> eigensolver(cov_matrix);
        if (eigensolver.info() != Success) continue;
        Matrix2f eigenVector =eigensolver.eigenvectors();
        //std::cout << "Column as eigen vectors: "<<eigenVector<<std::endl;
        MatrixXf rotated = pointsSet*eigenVector;
        //std::cout<<"Max of X is: "<<(rotated.col(0)).maxCoeff()<<std::endl;
        //std::cout<<"Min of X is: "<<(rotated.col(0)).minCoeff()<<std::endl;
        //std::cout<<"Max of Y is: "<<(rotated.col(1)).maxCoeff()<<std::endl;
        //std::cout<<"Min of Y is: "<<(rotated.col(1)).minCoeff()<<std::endl;
        float axis1 =((rotated.col(0)).maxCoeff()-(rotated.col(0)).minCoeff())/2;
        float axis2 =((rotated.col(1)).maxCoeff()-(rotated.col(1)).minCoeff())/2;
        std::cout<<"Axis1 length: "<<axis1<<std::endl;
        std::cout<<"Axis2 length: "<<axis2<<std::endl;
        float area=22/7*axis1*axis2;
        float diameter =std::max(axis1,axis2);
        //std::cout<<"Area: "<<area<<std::endl;
        //std::cout<<"Diameter: "<<diameter<<std::endl;
        float shapeCompactness = 22/7*(diameter*diameter)/(4*area);
        std::cout<<"Shape score: "<<shapeCompactness<<std::endl;
        if(shapeCompactness>SHAPETHRESH){
            for(int i =0;i<vertexCount;i++){
                segmentMap["Building"].push_back(blockVertices[i]);
            }
        }else{
            for(int i =0;i<vertexCount;i++){
                segmentMap["RoadBuildComp"].push_back(blockVertices[i]);
            }
        }
    }
}
void SceneModel::cluster(std::string dataDir){
    std::vector<Vertex*> miscVertices =segmentMap["RoadBuildComp"];
    long vertexCount=miscVertices.size();
    std::cout<<"Vertices count for miscelllaneos: "<<vertexCount<<std::endl;
    float percentage = (float)vertexCount/(float)vertexList.size()*100;
    std::cout<<"Percentage of total points is : "<<percentage<<std::endl;
    int num_clusters=NUMCLUSTERS;
    int resetCounter=vertexCount/num_clusters;
    std::vector<Vertex> centers;
    int k=0;
    Vertex mean;
    mean.x=0;
    mean.y=0;
    mean.z=0;
    int counter=0;
    float max_x=MINVAL;
    float max_y=MINVAL;
    float max_z=MINVAL;
    float min_x=MAXVAL;
    float min_y=MAXVAL;
    float min_z=MAXVAL;
    for(int i=1;i<=vertexCount;i++){
        
        if(miscVertices[i-1]->x>max_x)
            max_x=miscVertices[i-1]->x;
        if(miscVertices[i-1]->y>max_y)
            max_y=miscVertices[i-1]->y;
        if(miscVertices[i-1]->z>max_z)
            max_z=miscVertices[i-1]->z;
        if(miscVertices[i-1]->x<min_x)
            min_x=miscVertices[i-1]->x;
        if(miscVertices[i-1]->y<min_y)
            min_y=miscVertices[i-1]->y;
        if(miscVertices[i-1]->z<min_z)
            min_z=miscVertices[i-1]->z;
    }
    std::cout<<"Max x is: "<<max_x<<std::endl;
    std::cout<<"Max y is: "<<max_y<<std::endl;
    std::cout<<"Max z is: "<<max_z<<std::endl;
    std::cout<<"Min x is: "<<min_x<<std::endl;
    std::cout<<"Min y is: "<<min_y<<std::endl;
    std::cout<<"Min z is: "<<min_z<<std::endl;
    
    time_t kmeansStartTime=time(NULL);
    
    for(int i=0;i<num_clusters;i++){
        Vertex c;
        c.x = rand()%int(max_x-min_x)+min_x;
        c.y = rand()%int(max_y-min_y)+min_y;
        c.z = rand()%int(max_z-min_z)+min_z;
        centers.push_back(c);
        std::cout<<"Computed centers are: "<<c.x<<","<<c.y<<","<<c.z<<std::endl;
    }
   
    int numIter=500;
    std::map<int,std::vector<Vertex*> > clusterMap;
    for(int i=0;i<numIter;i++){
        //assign points to new cluster centers
        clusterMap.clear();
        for(int j=0;j<vertexCount;j++){
            int fetchedCenter=getClusterIndex(centers,miscVertices[j]->x,miscVertices[j]->y,miscVertices[j]->z);
           // std::cout<<"Fetched center is: "<<fetchedCenter<<std::endl;
            clusterMap[fetchedCenter].push_back(miscVertices[j]);
        }
        //recompute centres
        int k=0;
        for(auto iter=clusterMap.begin();iter!=clusterMap.end();++iter){
            std::vector<Vertex*> list = iter->second;
            Vertex mean;
            mean.x=0,mean.y=0,mean.z=0;
            for(int m =0;m<list.size();m++){
                mean.x+=list[m]->x;
                mean.y+=list[m]->y;
                mean.z+=list[m]->z;
            }
            mean.x=mean.x/list.size();
            mean.y=mean.y/list.size();
            mean.z=mean.z/list.size();
            centers[k]=mean;
            k++;
        }
        /*for(int m=0;m<20;m++){
            clusterMap[20].push_back(&centers[m]);
        }*/
        if((i+1)%10==0)
            std::cout<<"Finished iteration: "<<i+1<<std::endl;
    }
    time_t kmeansEndTime=time(NULL);
    kmeansRuntime=kmeansEndTime-kmeansStartTime;
    writeClustersToFile(clusterMap,dataDir);
 /*   for(int i=0 ;i<centers.size();i++){
        for(int j=i+1 ;j<centers.size();j++){
            if(i==j)
                continue;
            std::cout<<"Cluster distance between "<<(i+1)<<" and "<<(j+1)<<" centres are: "<<euclidDistance(centers[i].x, centers[i].y, centers[i].z, centers[j].x, centers[j].y, centers[j].z)<<std::endl;
        }
    }*/
    makeSuperVoxels(clusterMap,centers,dataDir);
}
int SceneModel::getClusterIndex(std::vector<Vertex> c,float x,float y,float z){
    int closestIndex=-1;
    //float euclidDist = 0;
    float minDist=MAXVAL;
    for(int i=0;i<c.size();i++){
        //std::cout<<"Checking point: "<<x<<","<<y<<","<<z<<" with cluster center: "<<c[i].x<<","<<c[i].y<<","
        //<<c[i].z<<std::endl;
        float curDist=euclidDistance(c[i].x,c[i].y,c[i].z,x,y,z);
        //std::cout<<"Current Distance is: "<<curDist<<std::endl;
        if(curDist<minDist){
            minDist=curDist;
            closestIndex=i;
            //std::cout<<"Min Distance is: "<<minDist<<std::endl;
        }
    }
    return closestIndex;
}
void SceneModel::writeClustersToFile(std::map<int,std::vector<Vertex*> > cMap,std::string dir){
    std::ofstream newFile;
    std::cout<<"Creating "<<cMap.size()<<" cluster files in: "<<dir+"/"+getSceneId()<<std::endl;
    int counter=0;
    for(auto iter=cMap.begin();iter!=cMap.end();++iter){
        std::vector<Vertex*> list=iter->second;
        if(list.size()==0)
            continue;
        newFile.open(dir+"/"+getSceneId()+"_CLUSTERS/CLUSTER"+std::to_string(counter+1));
        for(int j=0;j<list.size();j++){
            newFile<<(iter->second)[j]->x<<","<<(iter->second)[j]->y<<","<<(iter->second)[j]->z<<","<<std::endl;
        }
        std::cout<<"File: "<<dir+"/"+getSceneId()+"_CLUSTERS//CLUSTER"+std::to_string(counter+1)<<" created"<<std::endl;
        newFile.close();
        counter++;
    }
}
float SceneModel::euclidDistance(float x1,float y1, float z1,float x2,float y2, float z2){
    float a = (x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)+(z1-z2)*(z1-z2);
    return sqrt(a);
}
void SceneModel::makeSuperVoxels(std::map<int,std::vector<Vertex*> > cMap,std::vector<Vertex> centers,std::string dir){
    time_t superVoxelStartTime=time(NULL);
    int counter=0;
    std::vector<std::vector<int> > connectList;
    for(int i=0 ;i<(centers.size()-1);i++){
        
        for(int j=i+1 ;j<centers.size();j++){
            float clusterDistance = euclidDistance(centers[i].x, centers[i].y, centers[i].z, centers[j].x, centers[j].y, centers[j].z);
            if(clusterDistance<=CLUSTDISTTHRESH){
                std::cout<<"Cluster distance between "<<(i+1)<<" and "<<(j+1)<<" centres are: "<<clusterDistance<<std::endl;
                std::cout<<"Checking for cluster merge possibility"<<std::endl;
                int isMatch=isEigenSimilar(cMap[i],cMap[j]);
                //ADD TO SUPERVOXEL MAP
                if(isMatch){
                    std::vector<int> edge;
                    edge.push_back(i);
                    edge.push_back(j);
                    connectList.push_back(edge);
                }
            }
        }
            
    }
    for(int i=0;i<connectList.size();i++){
        for(int j=0;j<connectList[i].size();j++){
            std::cout<<(connectList[i])[j]<<",";
        }
        std::cout<<std::endl;
    }
    std::vector<int> visited;
    
    for(int i=0;i<centers.size();i++){
        if(std::find(visited.begin(),visited.end(),i)!=visited.end()==false){
            std::queue<int> travQ;
            travQ.push(i);
            std::vector<Vertex*> nodesList;
            while(travQ.empty()==false){
                int cur = travQ.front();
                travQ.pop();
                std::cout<<"Node: "<<cur<<",";
                std::vector<Vertex*> curList=cMap[cur];
                
                for(int m=0;m<curList.size();m++)
                    nodesList.push_back(curList[m]);
                
                visited.push_back(cur);
                for(int j=0;j<connectList.size();j++){
                    if((connectList[j])[0]==cur)
                        if((std::find(visited.begin(),visited.end(),(connectList[j])[1])!=visited.end())==false)
                            travQ.push((connectList[j])[1]);
                    if((connectList[j])[1]==cur)
                        if((std::find(visited.begin(),visited.end(),(connectList[j])[0])!=visited.end())==false)
                            travQ.push((connectList[j])[0]);
                }
            }
            std::cout<<std::endl;
            if(nodesList.size()!=0){
                superVoxelMap[counter]=nodesList;
                std::cout<<nodesList.size()<<" vertices added in supervoxel: "<<counter+1<<std::endl;
                counter++;
                std::cout<<"Above is a cluster"<<std::endl;
            }
            
        }
        
    }
    time_t superVoxelEndTime=time(NULL);
    superVoxelRuntime=superVoxelEndTime-superVoxelStartTime;
    std::string superVoxelDir = dir+"/"+getSceneId()+"_SUPERVOXELS";
    writeSuperVoxelsToFile(superVoxelDir);
    
}
int SceneModel::getEigenAngle(std::vector<float> vec1,std::vector<float> vec2){
    float dotProd = vec1[0]*vec2[0]+vec1[1]*vec2[1]+vec1[2]*vec2[2];
  //  std::cout<<"Dot product is: "<<dotProd<<std::endl;
    float angle = acos(dotProd)*180/3.1412;
  //  std::cout<<"Computed angle is: "<<angle<<std::endl;
    return angle;
}
int SceneModel::isEigenSimilar(std::vector<Vertex*> shape1,std::vector<Vertex*> shape2){
    
  //  std::cout<<"Shape 1 size: "<<shape1.size()<<std::endl;
 //  std::cout<<"Shape 2 size: "<<shape2.size()<<std::endl;
    std::vector<float> eigen1 = getEigenVector(shape1);
    std::vector<float> eigen2 = getEigenVector(shape2);
  //  std::cout<<"Vector 1 is: "<<eigen1[0]<<"i + "<<eigen1[1]<<"j + "<<eigen1[2]<<"k"<<std::endl;
  //  std::cout<<"Vector 2 is: "<<eigen2[0]<<"i + "<<eigen2[1]<<"j + "<<eigen2[2]<<"k"<<std::endl;
   
    if(getEigenAngle(eigen1,eigen2)<=EIGENANGLETHRESH){
        return 1;
    }
    else{
        return 0;
    }
}
std::vector<float> SceneModel::getEigenVector(std::vector<Vertex*> points){
    int vertexCount = points.size();
    MatrixXf pointsSet(vertexCount,3);
    float xmean = 0;
    float ymean = 0;
    float zmean = 0;
    for(int j=0;j<vertexCount;j++){
        pointsSet(j,0)=points[j]->x;
        xmean+=points[j]->x;
        pointsSet(j,1)=points[j]->y;
        ymean+=points[j]->y;
        pointsSet(j,2)=points[j]->z;
        zmean+=points[j]->z;
    }
    xmean=xmean/vertexCount;
    ymean=ymean/vertexCount;
    zmean=zmean/vertexCount;
    VectorXf mean(3);
    mean<<xmean,ymean,zmean;
    pointsSet.rowwise()-=mean.transpose();
    Matrix3f cov_matrix=pointsSet.transpose()*(pointsSet);
    //std::cout<<"Cov matrix: "<<cov_matrix<<std::endl;
    SelfAdjointEigenSolver<Matrix3f> eigensolver(cov_matrix);
    if (eigensolver.info() != Success) std::cout<<"Matrix not self adjoint. Normal computation fail"<<std::endl;
    Matrix3f eigenVector =eigensolver.eigenvectors();
    std::vector<float> maxEigen;
    maxEigen.push_back(eigenVector(0,0));
    maxEigen.push_back(eigenVector(1,0));
    maxEigen.push_back(eigenVector(2,0));
    return maxEigen;
}
void SceneModel::writeSuperVoxelsToFile(std::string dir){
    std::ofstream newFile;
    std::cout<<"Creating "<<superVoxelMap.size()<<" supervoxel cluster files in: "<<dir<<std::endl;
    int counter=0;
    for(auto iter=superVoxelMap.begin();iter!=superVoxelMap.end();++iter){
        std::vector<Vertex*> list=iter->second;
        if(list.size()==0)
            continue;
        newFile.open(dir+"/CLUSTER"+std::to_string(counter+1));
        for(int j=0;j<list.size();j++){
            newFile<<(iter->second)[j]->x<<","<<(iter->second)[j]->y<<","<<(iter->second)[j]->z<<","<<std::endl;
        }
        std::cout<<"File: "<<dir+"/CLUSTER"+std::to_string(counter+1)<<" created"<<std::endl;
        newFile.close();
        counter++;
    }
}
void SceneModel::computeSceneFeatures(){
    
    for(auto iter=superVoxelMap.begin();iter!=superVoxelMap.end();++iter){
        Feature f = getFeatures(iter->second);
        featureMap[iter->first]=f;
    }
}
Feature SceneModel::getFeatures(std::vector<Vertex *> vertices){
    int vertexCount = vertices.size();
    std::cout<<"Computing features for "<<vertexCount<<" vertices"<<std::endl;
    MatrixXf pointsSet(vertexCount,3);
    float xmean = 0;
    float ymean = 0;
    float zmean = 0;
    float max_z=MINVAL;
    float min_z=MAXVAL;
    float maxHeightRange=MINVAL;
    for(int j=0;j<vertexCount;j++){
        pointsSet(j,0)=vertices[j]->x;
        xmean+=vertices[j]->x;
        pointsSet(j,1)=vertices[j]->y;
        ymean+=vertices[j]->y;
        pointsSet(j,2)=vertices[j]->z;
        zmean+=vertices[j]->z;
        if(vertices[j]->z>max_z)
            max_z=vertices[j]->z;
        if(vertices[j]->z<min_z)
            min_z=vertices[j]->z;
        float heightRange=max_z-min_z;
        if(heightRange>maxHeightRange)
            maxHeightRange=heightRange;
    }
    xmean=xmean/vertexCount;
    ymean=ymean/vertexCount;
    zmean=zmean/vertexCount;
    VectorXf mean(3);
    mean<<xmean,ymean,zmean;
    pointsSet.rowwise()-=mean.transpose();
    Matrix3f cov_matrix=pointsSet.transpose()*(pointsSet);
    SelfAdjointEigenSolver<Matrix3f> eigensolver(cov_matrix);
    if (eigensolver.info() != Success) std::cout<<"Matrix not self adjoint. Normal computation fail"<<std::endl;
    Matrix3f eigenVector =eigensolver.eigenvectors();
    MatrixXf rotated = pointsSet*eigenVector;
    float axis1 =((rotated.col(0)).maxCoeff()-(rotated.col(0)).minCoeff())/2;
    float axis2 =((rotated.col(1)).maxCoeff()-(rotated.col(1)).minCoeff())/2;
    float axis3 =((rotated.col(2)).maxCoeff()-(rotated.col(2)).minCoeff())/2;
    float area = 3.1412*axis1*axis2;
    float edgeRatio = axis1/axis2;
    std::vector<float> cov;
    cov.push_back(cov_matrix(0,0));
    cov.push_back(cov_matrix(1,1));
    cov.push_back(cov_matrix(2,2));
    std::vector<float> norm;
    norm.push_back(eigenVector(0,0));
    norm.push_back(-eigenVector(1,0));
    norm.push_back(eigenVector(2,0));
    float density = vertexCount/area;
    float max = ( axis1 < axis2 ) ? axis2 : axis1;
    float maxEdge=( max < axis3 ) ? axis3 : max;
    
    //Assign feature values
    Feature feature;
    feature.area=area;
    feature.density=density;
    feature.edgeRatio=edgeRatio;
    feature.height=maxHeightRange;
    feature.maxEdge=maxEdge;
    feature.Covariance=cov;
    feature.normal=norm;
    feature.train=false;
    feature.label=-1;
    
    return feature;
}
void SceneModel::buildMapForTrain(){
    segmentMap["RoadBuildComp"];
    for(int i=0;i<vertexList.size();i++){
        segmentMap["RoadBuildComp"].push_back(&vertexList[i]);
    }
}
time_t SceneModel::getKMeansRuntime(){
    return kmeansRuntime;
}
time_t SceneModel::getSuperVoxelsRuntime(){
    return superVoxelRuntime;
}
