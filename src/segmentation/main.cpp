//
//  main.cpp
//  TrialGraphics
//
//  Created by Anish Saha on 10/20/18.
//  Copyright © 2018 Anish Saha. All rights reserved.
//
#include <stdio.h>  /* defines FILENAME_MAX */
// #define WINDOWS  /* uncomment this line to use it for windows.*/
#ifdef WINDOWS
#include <direct.h>
#define GetCurrentDir _getcwd
#else
#include <unistd.h>
#define GetCurrentDir getcwd
#endif
#include <iostream>
#include "SceneModel.hpp"
#include "SceneManager.hpp"
#include <queue>
#include <ctime>
int main(int argc, const char * argv[]) {
    
    /*char buffer[FILENAME_MAX];
    char *answer = GetCurrentDir(buffer, sizeof(buffer));
    std::string s_cwd;
    if (answer)
    {
        s_cwd = answer;
    }
    std::cout<<s_cwd<<std::endl;*/
    time_t startTime,endTime;
    startTime=time(NULL);

    char cCurrentPath[FILENAME_MAX];
    
    if (!GetCurrentDir(cCurrentPath, sizeof(cCurrentPath)))
    {
        return errno;
    }
    cCurrentPath[sizeof(cCurrentPath) - 1] = '\0'; /* not really required */
    printf ("The current working directory is %s", cCurrentPath);
    std::string datadir=std::string(cCurrentPath)+"/../../dataset/UnlabelledData";
    //SceneManager sm("/Users/anishsaha/Documents/GitHub/GraphicsFall2018/datasets/UnlabelledData");
    SceneManager sm(datadir);
    char choice;
    std::cout<<"***********************************************************************"<<std::endl;
    std::cout<<"******************** WELCOME TO THE LIDAR SCENE MANAGER ***************"<<std::endl;
    std::cout<<"***********************************************************************"<<std::endl;
    std::cout<<std::endl;
    int controller=1;
    while(controller){
        std::cout<<"Choose from the options below:"<<std::endl;
        std::cout<<"1. Show loaded Scenes"<<std::endl;
        std::cout<<"2. Show scenes available for loading"<<std::endl;
        std::cout<<"3. Load Scene"<<std::endl;
        std::cout<<"4. Segment Scene"<<std::endl;
        std::cout<<"5. Show Scene Stats"<<std::endl;
        std::cout<<"6. Train and Save Learner model"<<std::endl;
        std::cout<<"7. Load Learner Model"<<std::endl;
        std::cout<<"8. Exit"<<std::endl;
        std::cin>>choice;
        switch(choice){
            case '1':
                sm.showScenesList();
                break;
            case '2':
                sm.showLoadableScenes();
                break;
            case '3':
                sm.createScene();
                break;
            case '4':
                sm.segmentScene();
                break;
            case '5':
                sm.showSceneStats();
                break;
            case '6':
                sm.createScene(true);
                break;
            case '7':
                //sm.showSceneStats();
                break;
            case '8':
                controller=0;
                std::cout<<"******************** EXITING LIDAR SCENE MANAGER ***************"<<std::endl;
                break;
            default:
                std::cout<<"Incorrect choice"<<std::endl;
        };
    }
    endTime=time(NULL);
    std::cout<<"Process completed in: "<<endTime-startTime<<" seconds."<<std::endl;
    
    return 0;
}

