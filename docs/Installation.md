# Installation
We need to install ROS with Anaconda to subscribe and process the images from ROS publishers. as reference check: https://robostack.github.io/GettingStarted.html


## ROS Setup
It is recommended use Mamba with a fresh miniforge installation. Please refer to the Mamba website for detailed instructions: https://mamba.readthedocs.io/en/latest/installation/mamba-installation.html

Installing Mamba to an existing conda environment is not recommended. However, you can install it from the conda-forge channel. In this case, you need to install it in the base environment:
```
conda install mamba -c conda-forge
```
### Installation of Robostack
This is needed to run ROS inside the Anaconda environment.
Create new virtual environment. Please select a suitable version of Python (3.9+):
```
mamba create -n ros_env python=3.9
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
## Setup the SAM and Xmem (VOS model)
Requirements:
- Python 3.9+
- PyTorch 1.11+ (See PyTorch for installation instructions)
- torchvision corresponding to the PyTorch version

Install the required packages:
```
pip install pandas tqdm
```
Install the Segment Anything Model:
```
pip install git+https://github.com/facebookresearch/segment-anything.git
```

Environment setting is completed. Install IST_ROS:
```
git clone https://github.com/husikl/ist_ros.git
cd ist_ros
mkdir -p data/weights
```

Install the weights for SAM and XMem
1. SAM:
```
wget -P ./data/weights/ https://dl.fbaipublicfiles.com/segment_anything/sam_vit_h_4b8939.pth
```

2. XMem: 
```
wget -P ./data/weights/ https://github.com/hkchengrex/XMem/releases/download/v1.0/XMem.pth
wget -P ./data/weights/ https://github.com/hkchengrex/XMem/releases/download/v1.0/XMem-s012.pth
```
