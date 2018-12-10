//
//  SceneManager.hpp
//  TrialGraphics
//
//  Created by Anish Saha on 10/22/18.
//  Copyright © 2018 Anish Saha. All rights reserved.
//

#ifndef SceneManager_hpp
#define SceneManager_hpp
#include <vector>
#include <stdio.h>
#include "SceneModel.hpp"
class SceneManager{
private:
    std::vector<SceneModel> sceneList;
    std::string datasetDir;
    int sceneCount;
    std::vector<std::string> fileList;
    std::vector<std::string> trainList;
    
public:
    SceneManager(std::string);
    void createScene(bool isTrain=false);
    void duplicateScene();
    void showScenesList();
    void showLoadableScenes();
    void segmentScene();
    void showSceneStats();
    SceneModel& getScene(std::string);
    void publishPerformanceReport(time_t,time_t,time_t,time_t,time_t);
    
};
#endif /* SceneManager_hpp */
