# IST-ROS: A Flexible Object Segmentation and Tracking Framework for ROS

IST-ROS is a package based on integration of Segment Anything Model with semi-supervised video object segmentation method, for flexible object segmentation and tracking in ROS-based systems.

## Installation

For detailed installation instructions, please refer to the [Installation.md](Installation.md) file.

## Usage

IST-ROS can be used with or without ROS. For examples on how to run IST-ROS, please refer to the following files:

- [GUI.md](GUI.md): Instruction for using Graphical User Interface .
- [RunWithROS.md](RunWithROS.md): Instructions for running IST-ROS with ROS.
- [RunWithoutROS.md](RunWithoutROS.md): Instructions for running IST-ROS without ROS.

### Interactive Target Selection through GUI

![Target selection](assets/target_selection.gif)

### Target Segmentation and Tracking

<p float="left">
  <img alt="pick_and_place_gui" src="assets/pick_and_place_gui.gif?raw=true" width="55%" style="vertical-align:middle;" />
  <img alt="pp" src="assets/pick_and_place_combined.gif?raw=true" width="43%" style="vertical-align:middle;" />
  <img alt="suturing_gui" src="assets/suturing_gui.gif?raw=true" width="55%" style="vertical-align:middle;" />
  <img alt="suturing" src="assets/suturing_combined.gif?raw=true" width="43%" style="vertical-align:middle;" />
  <img alt="wallet_gui" src="assets/wallet_gui.gif?raw=true" width="55%" style="vertical-align:middle;" />
  <img alt="wallet" src="assets/wallet_combined.gif?raw=true" width="43%" style="vertical-align:middle;" />
  <img alt="open_gui" src="assets/open_gui.gif?raw=true" width="55%" style="vertical-align:middle;" />
  <img alt="open" src="assets/open_combined.gif?raw=true" width="43%" style="vertical-align:middle;" />
  <img alt="fingers_gui" src="assets/fingers_gui.gif?raw=true" width="55%" style="vertical-align:middle;" />
  <img alt="fingers" src="assets/fingers_combined.gif?raw=true" width="43%" style="vertical-align:middle;" />  
  <img alt="hsr_gui" src="assets/hsr_gui.gif?raw=true" width="55%" style="vertical-align:middle;" />
  <img alt="hsr" src="assets/hsr_combined.gif?raw=true" width="43%" style="vertical-align:middle;" />
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
@Article{fozilov2025istros,
title = {IST-ROS: A flexible object segmentation and tracking framework for robotics applications},
journal = {SoftwareX},
volume = {29},
pages = {101979},
year = {2025},
issn = {2352-7110},
doi = {https://doi.org/10.1016/j.softx.2024.101979},
url = {https://www.sciencedirect.com/science/article/pii/S2352711024003492},
author = {Khusniddin Fozilov and Yutaro Yamada and Jacinto Colan and Yaonan Zhu and Yasuhisa Hasegawa},
keywords = {Real-time object detection, Video object segmentation, Robot operating system (ROS)},
abstract = {Object detection and tracking are crucial components in the development of various applications and research endeavors within the computer science and robotics community. However, the diverse shapes and appearances of real-world objects, as well as dynamic nature of the scenes, may pose significant challenges for these tasks. Existing object detection and tracking methods often require extensive data annotation and model re-training when applied to new objects or environments, diverting valuable time and resources from the primary research objectives. In this paper, we present IST-ROS, Interactive Segmentation and Tracking for ROS, a software solution that leverages the capabilities of the Segment Anything Model (SAM) and semi-supervised video object segmentation methods to enable flexible and efficient object segmentation and tracking. Its graphical interface allows interactive object selection and segmentation using various prompts, while integrated tracking ensures robust performance even under occlusions and object interactions. By providing a flexible solution for object segmentation and tracking, IST-ROS aims to facilitate rapid prototyping and advancement of robotics applications.}
}

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
