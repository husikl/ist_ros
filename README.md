```
# ROS-IST: ROS-based Interactive Segmentation and Tracking

ROS-IST is a software solution that addresses the challenges of object detection and tracking in robotics and computer science applications. By combining the interactivity of Segment Anything Model with semi-supervised video object segmentation methods, ROS-IST enables flexible and efficient object segmentation and tracking in ROS-based systems.

ROS-IST provides a user-friendly graphical interface that allows users to interactively select and segment objects of interest using various prompts, such as bounding boxes and point-based annotations. The integrated tracking ensures robust performance under occlusions and object interactions. ROS-IST's modular design and ROS compatibility make it a valuable tool for rapid prototyping, data collection, and developing object-based perception algorithms.

## Installation

For detailed installation instructions, please refer to the [INSTALLATION.md](INSTALLATION.md) file.

## Usage

ROS-IST can be used with or without ROS. For examples on how to run ROS-IST, please refer to the following files:

- [RunWithROS.md](RunWithROS.md): Instructions for running ROS-IST with ROS.
- [RunWithoutROS.md](RunWithoutROS.md): Instructions for running ROS-IST without ROS.

### Interactive Target Selection through GUI

![Target selection](assets/target_selection.gif)

### Target Segmentation and Tracking

<p float="left">
  <img alt="pick_and_place_gui" src="assets/pick_and_place_gui.gif?raw=true" width="55%" style="vertical-align:middle;" />
  <img alt="pp" src="assets/pick_and_place_combined.gif?raw=true" width="43%" style="vertical-align:middle;" />
  <img alt="micromanipulation_gui" src="assets/micromanipulation_gui.gif?raw=true" width="55%" style="vertical-align:middle;" />
  <img alt="micromanipulation" src="assets/micromanipulation_combined.gif?raw=true" width="43%" style="vertical-align:middle;" />
  <img alt="boxing_gui" src="assets/boxing_gui.gif?raw=true" width="55%" style="vertical-align:middle;" />
  <img alt="boxing" src="assets/boxing_combined.gif?raw=true" width="43%" style="vertical-align:middle;" />
  <img alt="sheep_gui" src="assets/sheep_gui.gif?raw=true" width="55%" style="vertical-align:middle;" />
  <img alt="sheep" src="assets/sheep_combined.gif?raw=true" width="43%" style="vertical-align:middle;" />
</p>

### Multi-mode Tracking Visualization

![Target selection](assets/pick_and_place_visualization.gif)

## Related publications

```bibtex
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
  author={Kirillov, Alexander and Mintun, Eric and Ravi, Nikhila and Mao, Hanzi and Rolland, Chloe and Gustafson, Laura and Xiao, Tete and Whitehead, Spencer and Berg, Alexander C. and Lo, Wan-Yen and Doll{\\'a}r, Piotr and Girshick, Ross},
  journal={arXiv:2304.02643},
  year={2023}
}
```
