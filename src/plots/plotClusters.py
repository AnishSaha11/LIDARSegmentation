
# coding: utf-8

# In[3]:

from os import listdir
from os import path
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
import sys
import pandas.io.common


# In[7]:


def plotScene(dir):
    #read data from csv
    if os.path.exists(dir+"/.DS_Store"):
        os.remove(dir+"/.DS_Store")
    fig=plt.figure(figsize=(15,15))
    ax=fig.add_subplot(111,projection='3d')
    colorList = ['red','blue','green','yellow','black','magenta','cyan']
    clusterFiles =listdir(dir)
    print clusterFiles
   # print clusterFiles
    for i,x in enumerate(clusterFiles):
        print x
        if x == "." or x=="..":
            continue
        try:    
            df = pd.read_csv(dir+"/"+x,header=None)
        except pd.io.common.EmptyDataError:
            print(dir+"/"+x, " is empty and has been skipped.")
            continue
        x1list = df[0].tolist()
        y1list= df[1].tolist()
        z1list = df[2].tolist()
        
        ax.scatter(x1list,y1list,z1list,linewidths=0.02,color=colorList[i%7])
    plt.show()
        
    
if __name__=='__main__':
    #arg = sys.argv
    #plotScene(arg[1])

    datadir='../../dataset/UnlabelledData'
    filelist=listdir(datadir)
    count = 1
    for x in filelist:
        
        print count,'-',x
        count=count+1
    choice1 = input("Choose folder for clusters: ")
 
    plotScene(datadir+"/"+filelist[choice1-1])


