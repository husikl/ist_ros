# Installation
We need to install ROS with Anaconda to subscribe and process the images from ROS publishers. as reference check: https://robostack.github.io/GettingStarted.html

## Installation mamba
The installation of Mamba is recommended to be started with miniforge installation in fresh environment. Please refer to mamba website: https://mamba.readthedocs.io/en/latest/installation/mamba-installation.html

The instllation of Mamba to existing conda is not recommended. However, you can install it from the conda-forge channel. In this case, you need to install this to base enviroment:
```
conda install mamba -c conda-forge
```
--> it takes more than 5 hours, but Not installed


It seems there is a conflict between mamba and spyder
```
conda uninstall spyder
conda install mamba -c conda-forge (again)
```

Works


!!!!!!!!!!!!!!!The later one introduces error to conda environment. I run conda command such as "conda list", and then show the following error everytime.


Error while loading conda entry point: anaconda-cloud-auth (cannot import name 'ChannelAuthBase' from 'conda.plugins.types' (/home/yutaroyamada/anaconda3/lib/python3.9/site-packages/conda/plugins/types.py))
Error while loading conda entry point: anaconda-cloud-auth (cannot import name 'ChannelAuthBase' from 'conda.plugins.types' (/home/yutaroyamada/anaconda3/lib/python3.9/site-packages/conda/plugins/types.py))
## Instllation robostack
Create new virtual environment
```
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
!!!!!!!!!!!!!ERROR I think this causes from my previous experiment that connect 2 PCs


RLException: roscore cannot run as another roscore/master is already running. 
Please kill other roscore/master processes before relaunching.
The ROS_MASTER_URI is
The traceback for the exception was written to the log file
I got this error even in ros env as well

--> sudo killall -9 rosmaster

then

roscore

Works
## Installation remaining packages
conda install pytorch==1.11.0 torchvision==0.12.0 torchaudio==0.11.0 cudatoolkit=11.4 -c pytorch
conda install pytorch==1.11.0 torchvision==0.12.0 torchaudio==0.11.0 cudatoolkit=11.3 -c pytorch

--> Failed: There is no torch
Confilicts to python. Python for this version should be less then 3.10. Default python version for robostack is 3.11.

conda install pytorch==2.1.0 torchvision==0.16.0 torchaudio==2.1.0 -c pytorch -c nvidia

Works (wo gpu)
/home/yutaroyamada/anaconda3/envs/ros_env/lib/python3.11/site-packages/torch/cuda/__init__.py:138: UserWarning: CUDA initialization: The NVIDIA driver on your system is too old (found version 11040). Please update your GPU driver by downloading and installing a new version from the URL: http://www.nvidia.com/Download/index.aspx Alternatively, go to: https://pytorch.org to install a PyTorch version that has been compiled with your version of the CUDA driver. (Triggered internally at /opt/conda/conda-bld/pytorch_1695392035891/work/c10/cuda/CUDAFunctions.cpp:108.)
  return torch._C._cuda_getDeviceCount() > 0
False

I think this is because our cuda is 11.3, next

conda install opencv
conda install pandas
pip install git+https://github.com/facebookresearch/segment-anything.git
conda install tqdm
wget -P ./weights/ https://github.com/hkchengrex/XMem/releases/download/v1.0/XMem.pth

--> Error CUDA is not available. 

## PART2
mamba create -n ros_env_py38 python=3.8
mamba activate ros_env_py38
conda config --env --add channels conda-forge
conda config --env --add channels robostack-staging
conda config --env --remove channels defaults
(conda config --show channels)
mamba install ros-noetic-desktop
--> failure, cannot run roscore
Could not solve for environment specs
The following packages are incompatible
└─ ros-noetic-desktop is installable with the potential options
   ├─ ros-noetic-desktop 1.5.0 would require
   │  ├─ ros-distro-mutex 0.4.* , which can be installed;
   │  ├─ ros-noetic-joint-state-publisher-gui with the potential options
   │  │  ├─ ros-noetic-joint-state-publisher-gui 1.15.1 would require
   │  │  │  └─ ros-noetic-python-qt-binding with the potential options
   │  │  │     ├─ ros-noetic-python-qt-binding 0.4.4 would require
   │  │  │     │  └─ qt-main >=5.15.8,<5.16.0a0  with the potential options
   │  │  │     │     ├─ qt-main 5.15.8 would require
   │  │  │     │     │  └─ openssl >=3.1.0,<4.0a0 , which can be installed;
   │  │  │     │     ├─ qt-main 5.15.8 would require
   │  │  │     │     │  └─ openssl >=3.2.1,<4.0a0 , which can be installed;
   │  │  │     │     ├─ qt-main 5.15.8 would require
   │  │  │     │     │  └─ openssl >=3.2.0,<4.0a0 , which can be installed;
   │  │  │     │     ├─ qt-main 5.15.8 would require
   │  │  │     │     │  └─ openssl >=3.0.8,<4.0a0 , which can be installed;
   │  │  │     │     ├─ qt-main 5.15.8 would require
   │  │  │     │     │  └─ openssl >=3.1.1,<4.0a0 , which can be installed;
   │  │  │     │     ├─ qt-main 5.15.8 would require
   │  │  │     │     │  └─ openssl >=3.1.3,<4.0a0 , which can be installed;
   │  │  │     │     └─ qt-main 5.15.8 would require
   │  │  │     │        └─ openssl >=3.1.2,<4.0a0 , which can be installed;
   │  │  │     └─ ros-noetic-python-qt-binding 0.4.4 would require
   │  │  │        └─ ros-distro-mutex 0.5.* , which conflicts with any installable versions previously reported;
   │  │  └─ ros-noetic-joint-state-publisher-gui 1.15.1 would require
   │  │     └─ ros-distro-mutex 0.5.* , which conflicts with any installable versions previously reported;
   │  └─ ros-noetic-viz, which requires
   │     ├─ ros-distro-mutex [0.4.* |0.5.* ] with the potential options
   │     │  ├─ ros-distro-mutex 0.5.0 conflicts with any installable versions previously reported;
   │     │  └─ ros-distro-mutex 0.4.0, which can be installed;
   │     └─ ros-noetic-rqt-common-plugins with the potential options
   │        ├─ ros-noetic-rqt-common-plugins 0.4.9 would require
   │        │  ├─ ros-distro-mutex 0.4.* , which can be installed;
   │        │  └─ ros-noetic-rqt-image-view with the potential options
   │        │     ├─ ros-noetic-rqt-image-view 0.4.16 would require
   │        │     │  └─ ros-noetic-rqt-gui-cpp with the potential options
   │        │     │     ├─ ros-noetic-rqt-gui-cpp 0.5.3 would require
   │        │     │     │  └─ ros-noetic-qt-gui-cpp with the potential options
   │        │     │     │     ├─ ros-noetic-qt-gui-cpp 0.4.2 would require
   │        │     │     │     │  ├─ python with the potential options
   │        │     │     │     │  │  ├─ python [1.0.1|1.2|...|3.8.5], which can be installed;
   │        │     │     │     │  │  ├─ python [3.11.0|3.11.1|...|3.11.9], which can be installed;
   │        │     │     │     │  │  ├─ python [3.8.0|3.8.1] would require
   │        │     │     │     │  │  │  └─ openssl >=1.1.1a,<1.1.2a , which conflicts with any installable versions previously reported;
   │        │     │     │     │  │  ├─ python 3.8.10 would require
   │        │     │     │     │  │  │  └─ openssl >=1.1.1k,<1.1.2a , which conflicts with any installable versions previously reported;
   │        │     │     │     │  │  ├─ python [3.8.10|3.8.12] would require
   │        │     │     │     │  │  │  └─ openssl >=1.1.1l,<1.1.2a , which conflicts with any installable versions previously reported;
   │        │     │     │     │  │  ├─ python 3.8.13 would require
   │        │     │     │     │  │  │  └─ openssl >=1.1.1n,<1.1.2a , which conflicts with any installable versions previously reported;
   │        │     │     │     │  │  ├─ python 3.8.2 would require
   │        │     │     │     │  │  │  └─ openssl >=1.1.1e,<1.1.2a , which conflicts with any installable versions previously reported;
   │        │     │     │     │  │  ├─ python 3.8.2 would require
   │        │     │     │     │  │  │  └─ openssl >=1.1.1f,<1.1.2a , which conflicts with any installable versions previously reported;
   │        │     │     │     │  │  ├─ python [3.8.2|3.8.3|3.8.4|3.8.5] would require
   │        │     │     │     │  │  │  └─ openssl >=1.1.1g,<1.1.2a , which conflicts with any installable versions previously reported;
   │        │     │     │     │  │  ├─ python 3.8.2 would require
   │        │     │     │     │  │  │  └─ openssl >=1.1.1d,<1.1.2a , which conflicts with any installable versions previously reported;
   │        │     │     │     │  │  ├─ python [3.8.5|3.8.6] would require
   │        │     │     │     │  │  │  └─ openssl >=1.1.1h,<1.1.2a , which conflicts with any installable versions previously reported;
   │        │     │     │     │  │  ├─ python 3.8.6 would require
   │        │     │     │     │  │  │  └─ openssl >=1.1.1i,<1.1.2a , which conflicts with any installable versions previously reported;
   │        │     │     │     │  │  ├─ python 3.8.8 would require
   │        │     │     │     │  │  │  └─ openssl >=1.1.1j,<1.1.2a , which conflicts with any installable versions previously reported;
   │        │     │     │     │  │  ├─ python [3.9.0|3.9.1|...|3.9.9], which can be installed;
   │        │     │     │     │  │  ├─ python 3.12.0rc3 would require
   │        │     │     │     │  │  │  └─ _python_rc, which does not exist (perhaps a missing channel);
   │        │     │     │     │  │  └─ python [3.8.14|3.8.15] would require
   │        │     │     │     │  │     └─ openssl >=1.1.1s,<1.1.2a , which conflicts with any installable versions previously reported;
   │        │     │     │     │  └─ python_abi 3.9.* *_cp39 with the potential options
   │        │     │     │     │     ├─ python_abi 3.9 would require
   │        │     │     │     │     │  └─ python 3.9.* , which can be installed;
   │        │     │     │     │     └─ python_abi 3.9 would require
   │        │     │     │     │        └─ python 3.9.* *_cpython, which conflicts with any installable versions previously reported;
   │        │     │     │     └─ ros-noetic-qt-gui-cpp 0.4.2 would require
   │        │     │     │        └─ ros-distro-mutex 0.5.* , which conflicts with any installable versions previously reported;
   │        │     │     └─ ros-noetic-rqt-gui-cpp 0.5.3 would require
   │        │     │        └─ ros-distro-mutex 0.5.* , which conflicts with any installable versions previously reported;
   │        │     └─ ros-noetic-rqt-image-view 0.4.17 would require
   │        │        └─ ros-distro-mutex 0.5.* , which conflicts with any installable versions previously reported;
   │        └─ ros-noetic-rqt-common-plugins 0.4.9 would require
   │           ├─ ros-distro-mutex 0.5.* , which conflicts with any installable versions previously reported;
   │           └─ ros-noetic-rqt-bag with the potential options
   │              ├─ ros-noetic-rqt-bag 0.5.1 would require
   │              │  └─ ros-noetic-rosbag with the potential options
   │              │     ├─ ros-noetic-rosbag 1.16.0 would require
   │              │     │  └─ ros-noetic-rosbag-storage with the potential options
   │              │     │     ├─ ros-noetic-rosbag-storage 1.16.0 would require
   │              │     │     │  ├─ openssl >=3.2.1,<4.0a0 , which can be installed;
   │              │     │     │  └─ ros-noetic-roslz4 with the potential options
   │              │     │     │     ├─ ros-noetic-roslz4 1.16.0 would require
   │              │     │     │     │  ├─ python with the potential options
   │              │     │     │     │  │  ├─ python [1.0.1|1.2|...|3.8.5], which can be installed;
   │              │     │     │     │  │  ├─ python [3.11.0|3.11.1|...|3.11.9], which can be installed;
   │              │     │     │     │  │  ├─ python [3.8.0|3.8.1], which cannot be installed (as previously explained);
   │              │     │     │     │  │  ├─ python 3.8.10, which cannot be installed (as previously explained);
   │              │     │     │     │  │  ├─ python [3.8.10|3.8.12], which cannot be installed (as previously explained);
   │              │     │     │     │  │  ├─ python 3.8.13, which cannot be installed (as previously explained);
   │              │     │     │     │  │  ├─ python 3.8.2, which cannot be installed (as previously explained);
   │              │     │     │     │  │  ├─ python 3.8.2, which cannot be installed (as previously explained);
   │              │     │     │     │  │  ├─ python [3.8.2|3.8.3|3.8.4|3.8.5], which cannot be installed (as previously explained);
   │              │     │     │     │  │  ├─ python 3.8.2, which cannot be installed (as previously explained);
   │              │     │     │     │  │  ├─ python [3.8.5|3.8.6], which cannot be installed (as previously explained);
   │              │     │     │     │  │  ├─ python 3.8.6, which cannot be installed (as previously explained);
   │              │     │     │     │  │  ├─ python 3.8.8, which cannot be installed (as previously explained);
   │              │     │     │     │  │  ├─ python [3.9.0|3.9.1|...|3.9.9], which can be installed;
   │              │     │     │     │  │  ├─ python 3.12.0rc3, which cannot be installed (as previously explained);
   │              │     │     │     │  │  └─ python [3.8.14|3.8.15], which cannot be installed (as previously explained);
   │              │     │     │     │  └─ python_abi 3.11.* *_cp311 with the potential options
   │              │     │     │     │     ├─ python_abi 3.11 would require
   │              │     │     │     │     │  └─ python 3.11.* , which can be installed;
   │              │     │     │     │     └─ python_abi 3.11 would require
   │              │     │     │     │        └─ python 3.11.* *_cpython, which conflicts with any installable versions previously reported;
   │              │     │     │     └─ ros-noetic-roslz4 1.15.15 would require
   │              │     │     │        └─ ros-distro-mutex 0.4.* , which can be installed;
   │              │     │     └─ ros-noetic-rosbag-storage 1.16.0 would require
   │              │     │        └─ ros-distro-mutex 0.4.* , which can be installed;
   │              │     └─ ros-noetic-rosbag [1.15.15|1.16.0] would require
   │              │        └─ ros-distro-mutex 0.4.* , which can be installed;
   │              └─ ros-noetic-rqt-bag 0.5.1 would require
   │                 └─ ros-distro-mutex 0.4.* , which can be installed;
   └─ ros-noetic-desktop 1.5.0 would require
      ├─ ros-distro-mutex 0.5.* , which conflicts with any installable versions previously reported;
      └─ ros-noetic-viz, which can be installed (as previously explained).

Python 3.8 is not appropriate for robostack
## PART3
The version of python for robostack should be 3.9+.
mamba create -n ros_env_py39 python=3.9
mamba activate ros_env_py39
conda config --env --add channels conda-forge
conda config --env --add channels robostack-staging
conda config --env --remove channels defaults
(conda config --show channels)
mamba install ros-noetic-desktop

(mamba activate ros_env_py39)
(roscore)

mamba install pytorch==1.11.0 torchvision==0.12.0 cudatoolkit=11.3 -c pytorch
->FAIL

conda install pytorch==1.12.0 torchvision==0.13.0 torchaudio==0.12.0 cudatoolkit=11.3 -c pytorch
->FAIL
mamba install pytorch==1.13.0 torchvision==0.14.0 -c pytorch -c nvidia
->FAIL
mamba install pytorch==2.0.0 torchvision==0.15.0 -c pytorch -c nvidia
->FAIL
I cannot use conda to install pytorch cause I cannot import it after installation:
>>> import torch
Traceback (most recent call last):
  File "<stdin>", line 1, in <module>
  File "/home/yutaroyamada/anaconda3/envs/ros_env_py39/lib/python3.9/site-packages/torch/__init__.py", line 202, in <module>
    from torch._C import *  # noqa: F403
ImportError: /home/yutaroyamada/anaconda3/envs/ros_env_py39/lib/python3.9/site-packages/torch/lib/libtorch_cpu.so: undefined symbol: iJIT_NotifyEvent

pip install torch==1.11.0+cu113 torchvision==0.12.0+cu113 torchaudio==0.11.0 --extra-index-url https://download.pytorch.org/whl/cu113
Success

(pip install opencv-python)
pip install pandas
pip install tqdm
install weights for sam from website 
pip install git+https://github.com/facebookresearch/segment-anything.git
wget -P ./weights/ https://github.com/hkchengrex/XMem/releases/download/v1.0/XMem.pth
--> python segment_and_track.py works