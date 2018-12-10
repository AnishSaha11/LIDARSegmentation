//
//  SceneManager.cpp
//  TrialGraphics
//
//  Created by Anish Saha on 10/22/18.
//  Copyright © 2018 Anish Saha. All rights reserved.
//
#define SIDE 0.82
#include <iostream>
#include "SceneManager.hpp"
// #define WINDOWS  /* uncomment this line to use it for windows.*/
#ifdef WINDOWS
#include "direct.h"
#endif
#include <dirent.h>
#include <string>
#include <ctime>
#define LIDAR "LidarData"
#define ROAD "ROAD"
#define ROADCOMP "ROADComp"
#define ROADBUILDCOMP "RoadBuildComp"
#define BUILD "Building"
SceneManager::SceneManager(std::string dir){
    this->datasetDir=dir;
    sceneCount=0;
    std::cout<<"Scene Manager started at directory location: "<<dir<<std::endl;
    DIR* dirp = opendir(dir.c_str());
    struct dirent *dp;
    while ((dp = readdir(dirp)) != NULL) {
        fileList.push_back(dp->d_name);
    }
    closedir(dirp);
    //delete the . and .. files
    fileList.erase(fileList.begin());
    fileList.erase(fileList.begin());
    std::cout<<fileList.size()<<" files available for loading"<<std::endl;
    std::string trainDir=dir+"/../LabelledData";
    dirp = opendir(trainDir.c_str());
    while ((dp = readdir(dirp)) != NULL) {
        trainList.push_back(dp->d_name);
    }
    closedir(dirp);
    //delete the . and .. files
    trainList.erase(trainList.begin());
    trainList.erase(trainList.begin());
    std::cout<<trainList.size()<<" files available for Trainning"<<std::endl;
}
void SceneManager::createScene(bool isTraining){
    if(isTraining==false){
        std::cout<<"******************** CHOOSE FILE TO LOAD INTO SCENE ***************"<<std::endl;
        int count=1;
        for(std::vector<std::string>::iterator it=fileList.begin();it!=fileList.end();++it){
            std::cout<<"Loadable file: "<<count<<". "<<*it<<std::endl;
            count++;
        }
        std::string choice;
        std::cin>>choice;
        int c=stoi(choice);
        std::cout<<"Chosen file: "<<c<<std::endl;
        SceneModel sm("SCN"+std::to_string(sceneCount+1),LIDAR,0,0);
        sceneCount+=1;
        std::cout<<"Scene created with SceneID: "<<sm.getSceneId()<<std::endl;
        std::cout<<"Populating scene vertices from file: "<<fileList.at(c-1)<<std::endl;
        sm.populateVertices(datasetDir+"/"+fileList.at(c-1));
        sceneList.push_back(sm);
    }else{
        std::cout<<"******************** CHOOSE FILE FOR TRAINING ***************"<<std::endl;
        int count=1;
        for(std::vector<std::string>::iterator it=trainList.begin();it!=trainList.end();++it){
            std::cout<<"Loadable file: "<<count<<". "<<*it<<std::endl;
            count++;
        }
        std::string choice;
        std::cin>>choice;
        int c=stoi(choice);
        std::cout<<"Chosen file: "<<c<<std::endl;
        SceneModel sm("SCN"+std::to_string(sceneCount+1)+"_TRAIN",LIDAR,0,0);
        sceneCount+=1;
        std::cout<<"Scene created with SceneID: "<<sm.getSceneId()<<std::endl;
        std::cout<<"Populating scene vertices from file: "<<trainList.at(c-1)<<std::endl;
        sm.populateVertices(datasetDir+"/../LabelledData/"+trainList.at(c-1),true);
        sceneList.push_back(sm);
        sm.buildMapForTrain();
        sm.cluster(datasetDir);
    }
    
}
void SceneManager::duplicateScene(){
    
}
void SceneManager::showScenesList(){
    if(sceneList.empty()){
        std::cout<<"There are no loaded scenes."<<std::endl;
        return;
    }
 //   std::cout<<"******************** WELCOME TO THE LIDAR SCENE MANAGER ***************"<<std::endl;
    std::cout<<"******************** BELOW IS THE LIST OF LOADED SCENES ***************"<<std::endl;
    for(std::vector<SceneModel>::iterator it=sceneList.begin();it!=sceneList.end();++it){
        std::cout<<"Scene id: "<<it->getSceneId()<<std::endl;
    }
}
void SceneManager::showLoadableScenes(){
    std::cout<<"******************** BELOW IS THE LIST OF LOADABLE SCENES ***************"<<std::endl;
    for(std::vector<std::string>::iterator it=fileList.begin();it!=fileList.end();++it){
        std::cout<<"Loadable file: "<<*it<<std::endl;
    }
    
}
void SceneManager::segmentScene(){
    std::cout<<"*********** Below is the list of available scenes to segment *************"<<std::endl;
    for(int i=0;i<sceneList.size();i++){
        std::cout<<sceneList[i].getSceneId()<<std::endl;
    }
    std::string c;
    std::cin>>c;
    SceneModel sc=getScene(c);
    std::cout<<"Fetched scene with scene id: "<<sc.getSceneId()<<std::endl;
    
    //Create variable for performance timing
    time_t roadSegStartTime,roadSegEndTime,roadSegElapsedTime,buildSegStartTime,buildSegEndTime,buildSegElapsedTime,clusterStartTime,clusterEndTime,clusterElapsedTime;
    
    roadSegStartTime=time(NULL);
    //create x-y blocks of side length SIZE
    sc.blockify(SIDE);
    sc.fitRansacRoadPlane();
    roadSegEndTime=time(NULL);
    roadSegElapsedTime=roadSegEndTime-roadSegStartTime;
    sc.writeSegmentToFile(ROAD,datasetDir+"/"+sc.getSceneId()+"_ROAD");
    sc.writeSegmentToFile(ROADCOMP, datasetDir+"/"+sc.getSceneId()+"_ROAD");
    std::cout<<"***********************************************************************"<<std::endl;
    std::cout<<"Road Segmentation completed in: "<<roadSegElapsedTime<<" secs."<<std::endl;
    std::cout<<"***********************************************************************"<<std::endl;
    std::cout<<" \nPress anything to continue"<<std::endl;
    std::cin>>c;
    
    buildSegStartTime=time(NULL);
    sc.blockifyForBuild(1);
    sc.getBuildingScore();
    sc.getBuildingShapeFeatures();
    buildSegEndTime=time(NULL);
    buildSegElapsedTime=buildSegEndTime-buildSegStartTime;
    sc.writeSegmentToFile(BUILD, datasetDir+"/"+sc.getSceneId()+"_ROADBUILD");
    sc.writeSegmentToFile(ROADBUILDCOMP, datasetDir+"/"+sc.getSceneId()+"_ROADBUILD");
    sc.writeSegmentToFile(ROAD,datasetDir+"/"+sc.getSceneId()+"_ROADBUILD");
    std::cout<<"***********************************************************************"<<std::endl;
    std::cout<<"Building Segmentation completed in: "<<buildSegElapsedTime<<" secs."<<std::endl;
    std::cout<<"***********************************************************************"<<std::endl;
    std::cout<<" \nPress anything to continue"<<std::endl;
    std::cin>>c;
    
    clusterStartTime=time(NULL);
    sc.cluster(datasetDir);
    clusterEndTime=time(NULL);
    clusterElapsedTime=clusterEndTime-clusterStartTime;
    std::cout<<"***********************************************************************"<<std::endl;
    std::cout<<"Total clustering completed in: "<<clusterElapsedTime<<" secs."<<std::endl;
    std::cout<<"***********************************************************************"<<std::endl;
    std::cout<<" \nPress anything to continue"<<std::endl;
    publishPerformanceReport(roadSegElapsedTime, buildSegElapsedTime, clusterElapsedTime,sc.getKMeansRuntime(),sc.getSuperVoxelsRuntime());
}
void SceneManager::showSceneStats(){
    std::cout<<"******************** BELOW IS THE LIST OF LOADED SCENES ***************"<<std::endl;
    for(int i=0;i<sceneList.size();i++){
        std::cout<<sceneList[i].getSceneId()<<std::endl;
    }
    std::string c;
    std::cin>>c;
    SceneModel sc=getScene(c);
    std::cout<<"Fetched scene with scene id: "<<sc.getSceneId()<<std::endl;
    sc.blockify(SIDE);
    sc.displaySceneStats();
}
SceneModel& SceneManager::getScene(std::string sceneid){
        SceneModel *ptr=NULL;
        for(int i=0;i<sceneList.size();i++){
            if(sceneList[i].getSceneId().compare(sceneid)==0){
                ptr=&sceneList[i];
            }
        }
        return *ptr;
}

void SceneManager::publishPerformanceReport(time_t roadTime, time_t buildTime, time_t clusterTime,time_t kmeansTime,time_t supervoxelTime){
    std::cout<<"***********************************************************************"<<std::endl;
    std::cout<<"************************* PERFORMANCE REPORT **************************"<<std::endl;
    std::cout<<"***********************************************************************"<<std::endl;
    std::cout<<std::endl;
    time_t totalTime=roadTime+buildTime+clusterTime;
    float roadSegPercent,buildSegPercent,totalClusterPercent,kmeansPercent,supervoxelPercent;
    roadSegPercent=(float)roadTime/totalTime*100;
    buildSegPercent=(float)buildTime/totalTime*100;
    totalClusterPercent=(float)clusterTime/totalTime*100;
    kmeansPercent=(float)kmeansTime/totalTime*100;
    supervoxelPercent=(float)supervoxelTime/totalTime*100;
    std::cout<<"|       MODULE          |     TIME ELAPSED    |   PERCENT OF TOTAL TIME    |"<<std::endl;
    std::cout<<"----------------------------------------------------------------------------"<<std::endl;
    std::cout<<"  ROAD SEGMENTATION            "<<roadTime<<" secs              "<<roadSegPercent<<" %"<<std::endl;
    std::cout<<"  BUILDING SEGMENTATION        "<<buildTime<<" secs               "<<buildSegPercent<<" %"<<std::endl;
    std::cout<<"  TOTAL CLUSTERING             "<<clusterTime<<" secs               "<<totalClusterPercent<<" %"<<std::endl;
    std::cout<<" -- KMEANS CLUSTERING          "<<kmeansTime<<" secs               "<<kmeansPercent<<" %"<<std::endl;
    std::cout<<" -- SUPERVOXEL CREATION        "<<supervoxelTime<<" secs               "<<supervoxelPercent<<" %"<<std::endl;
    std::cout<<"****************************************************************************"<<std::endl;
    std::cout<<"TOTAL TIME ELAPSED: "<<totalTime<<std::endl;
    std::cout<<std::endl;
    std::cout<<std::endl;
}
