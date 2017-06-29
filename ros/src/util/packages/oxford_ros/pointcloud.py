from sdk import Dataset
import numpy as np
from tf import transformations



def BuildPointCloud (dataset_dir):
    
    dataset = Dataset (dataset_dir)
    poseList = dataset.getIns()
    lidarTs = dataset.getTimestamp ('ldmrs', raw=True)
    scans = dataset.getMainLidar ()
    
    poses = {}
    # Build list of poses 
    for ts in lidarTs :
        pass



if __name__ == '__main__' :
    pass