# Interactive Segmentation_and_Tracking ROS
Fozilov Khusniddin, Yutaro Yamada

[Paper](https://www.mdpi.com/1424-8220/23/24/9865)

## Installation
We need to install ROS with Anaconda to subscribe and process the images from ROS publishers. as reference check: https://robostack.github.io/GettingStarted.html

```
conda install mamba -c conda-forge
mamba create -n ros_env
mamba activate ros_env
```

This adds the conda-forge channel to the new created environment configuration 
```
conda config --env --add channels conda-forge
```

and the robostack channel

```
conda config --env --add channels robostack-staging
```

Remove the defaults channel just in case, this might return an error if it is not in the list which is ok
```
conda config --env --remove channels defaults
```

Install ros-noetic into the environment (ROS1) only works with ROS1 for now.
```
mamba install ros-noetic-desktop
```

Check if roscore runs in new environment.

```
mamba activate ros_env
roscore
```


ROS independent requirements:
- Python 3.8+
- PyTorch 1.11+ (See PyTorch for installation instructions)
- torchvision corresponding to the PyTorch version
- OpenCV (try pip install opencv-python)

Segment Anything Model refer to installation steps from : https://github.com/facebookresearch/segment-anything
```
pip install git+https://github.com/facebookresearch/segment-anything.git
```

Xmem refer to installation stesps from : https://github.com/hkchengrex/XMem/blob/main/docs/GETTING_STARTED.md

I think in the end we need to combine and create a final requirements.txt and run pip install -r requirements.txt

download the model for xmem: 
```
wget -P ./saves/ https://github.com/hkchengrex/XMem/releases/download/v1.0/XMem.pth
wget -P ./saves/ https://github.com/hkchengrex/XMem/releases/download/v1.0/XMem-s012.pth
```
## TODO
check and create the final requirements.txt and test the build and running.
Create a Installation.md, RunWithROS.md, RunWithoutROS.md and add instructions for each.

## Getting Started
This project includes code for offline video masking and online publishing of masks and target positions.

### Offline Segmentation
To perform offline video masking, run the following command:
```
python test_script.py --input_path ./inputs/input_video.mp4 --output_path ./outputs/output_video.mp4
```
First, tkinter pop up. Please select the targets.

1.  Select the targets by both points and boxes
    * Draw box by mouse drag 
    * Point out object by right click
1. Push "Generate masks" button, then masked image is displayed
1. Select "Reselect" or "Apply Selection"

After pushing "Apply Selection", masking process starts.

The execution result will be saved in mp4 format at the specified output path.

### Online Segmentation and Tracking with ROS
#### Publishing
First, run the following command to start the ROS node for online masking and tracking:
```
python test_script_ros.py --input_topic /ximea_cam/image_raw
```
A blank Tkinter window will pop up.

Next, start target selection in another terminal:
```
rosservice call /init_segmentation "{}"
```
After the target selection, start processing:
```
rosservice call /run_inference "{}"
```
Then, masks and positions are being published.

#### Visualizing
We provide two way to visualize: maked image and images with circles pointing out the positions of the tracked targets.

Run the following command to start the ROS node that subscribes to the results from "test_script_ros.py" and visualizes them:
```
python visualizer.py
```
You can visualize the images in two ways by specifying command-line arguments:
1. Masked image (default)
* This is the default visualization mode. To disable it, add the "--disable_mask_mode" argument when running "visualizer.py".
2. Image with circles
* You can visualize the positions of the tracked target by adding the "--circle_mode" argument when running "visualizer.py". This mode displays circles at the tracked target positions.
For example, to run the visualizer with circle mode enabled, use the following command:
```
python visualizer.py --circle_mode
```
To view the visualized image, run the following command in another terminal:
```
rosrun image_view image_view image:=visualized_image
```
We can visualize image in 2 ways:
1. Masked image (default)
    * Set as default. To disable, put command "--disable_mask_mode" after "python visualizer.py"
1. Image with circles
    * We can visualize the positions of the tracked target by commend "--circle_mode"

## Demo
### Target selection through Graphic User Interface



https://github.com/Yutaro88/interactive_segmentation/assets/165972021/1207495b-76c9-4e37-88a6-72d9a72bf1a6



### Offline target masking

Pick and Place:

https://github.com/Yutaro88/interactive_segmentation/assets/165972021/f195f3a7-0011-4c72-8527-cd5d12a89af4


### Online target masking
![Online tool masking](assets/online_tool_masking.mp4)

## Citation
```
@Article{s23249865,
AUTHOR = {Fozilov, Khusniddin and Colan, Jacinto and Davila, Ana and Misawa, Kazunari and Qiu, Jie and Hayashi, Yuichiro and Mori, Kensaku and Hasegawa, Yasuhisa},
TITLE = {Endoscope Automation Framework with Hierarchical Control and Interactive Perception for Multi-Tool Tracking in Minimally Invasive Surgery},
JOURNAL = {Sensors},
VOLUME = {23},
YEAR = {2023},
NUMBER = {24},
ARTICLE-NUMBER = {9865},
URL = {https://www.mdpi.com/1424-8220/23/24/9865},
PubMedID = {38139711},
ISSN = {1424-8220},
DOI = {10.3390/s23249865}
}

@inproceedings{cheng2022xmem,
  title={{XMem}: Long-Term Video Object Segmentation with an Atkinson-Shiffrin Memory Model},
  author={Cheng, Ho Kei and Alexander G. Schwing},
  booktitle={ECCV},
  year={2022}
}

@article{kirillov2023segany,
  title={Segment Anything},
  author={Kirillov, Alexander and Mintun, Eric and Ravi, Nikhila and Mao, Hanzi and Rolland, Chloe and Gustafson, Laura and Xiao, Tete and Whitehead, Spencer and Berg, Alexander C. and Lo, Wan-Yen and Doll{\'a}r, Piotr and Girshick, Ross},
  journal={arXiv:2304.02643},
  year={2023}
}

```
# interactive_segmentation_and_tracking_ros
#TODO
