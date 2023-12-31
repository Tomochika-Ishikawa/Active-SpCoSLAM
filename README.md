# Active-SpCoSLAM
Implementation of Active-SpCoSLAM that is the active semantic mapping method.<br>
This repository includes the source codes used for the experiments in our paper on SMC 2023.<br>

# Abstract of Active-SpCoSLAM
We proposed Active-SpCoSLAM, which simultaneously learns spatial concepts and maps based on information gain (IG).
IG is derived from the following graphical model: IG is calculated separately for IG related to simultaneous localization and mapping (SLAM), IG related to location concepts, and finally a weighted sum.
The mapping between language and location is then done using an image captioning model called ClipCap.
![graphical_model](./images/graphical_model.png)

# Execution environment
- Ubuntu 20.04
- Python 3.8.10  
- ROS noetic
- Image captioning system: [ClipCap](https://github.com/rmokady/CLIP_prefix_caption)

# Preparation
- Please refer to the file `external_packages.txt` to download packages of gmapping, A* algorithm and simulation environment
- Please refer to the file `requirements.txt` to install requirements
- Please refer to the file `/Active-SpCoSLAM/models/model_for_clipcap/download_weights.txt` to download weights for ClipCap
- Please refer to the file `/Active-SpCoSLAM/models/model_for_places/download_weights.txt` to download weights for PlacesCNN
- Please refer to the file `code_replace_procedure.txt` to modify gmapping and A* algorithm package

# Execution procedure
- Launch the file `slam_gmapping/gmapping/launch/slam_gmapping_pr2.launch`
- Launch the file `Active_SpCoSLAM/scripts/pub_global_pose.py`
- Launch the file `Astar/launch/astar.launch`
- Execute the python file `/Active-SpCoSLAM/scripts/main.py`
