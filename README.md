# TerrainTraversabilityMapping
Traversability is calculated from normals and roughness.

![](TerrainTraversabilityMapping/image/traversability.gif)

### submodule branches
- approach_1
    - TerrainTraversabilityMapping : approach_1
    - Azure_Kinect_ROS_Driver : rough-terrain-perception-setup
    - normal_and_roughness_estimator : approach_1
- approach_2
    - TerrainTraversabilityMapping : approach_2
    - Azure_Kinect_ROS_Driver : rough-terrain-perception-setup
    - normal_and_roughness_estimator : approach_2

### launch
- downsample and normal estimation and roughness estimation
    ``` bash:traversability_evaluator
    roslaunch TerrainTraversabilityMapping traversability_evaluator.launch
    ```

### Dataset
[CrossProgressDataset](https://drive.google.com/drive/folders/1R3Wtpu1K4gtzwmeaHm4RCCFxt8rx4zGU?usp=sharing)