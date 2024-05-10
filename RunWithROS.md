# Online Segmentation and Tracking with ROS
This document provides a detailed guide on how to perform segmentation and tracking using ROS.
First, run the following command to start the ROS master:
```
roscore
```
## Publishing
Next, create a new terminal and run the following command to start the ROS node for online masking and tracking:
```
python segment_and_track_ros.py --input_topic /image_topic
```
Please specify the ROS topic of the image in `--input_topic`. Additionally, you can use `--camera_param` to input a resized or cropped image to the model to speed up the process.

A blank Tkinter window will pop up.

Next, start target selection in another terminal:
```
rosservice call /init_segmentation "{}"
```
The target selection GUI will appear. Please select the targets by following this workflow. For detailed instructions on using the GUI, please refer to `GUI.md`.

1. Select one target at a time using the following operations:
    * Draw a bounding box around the object by clicking and dragging the mouse.
    * Right-click on the object to specify it as foreground or background. You can change the mode using button.
1. Click the "Generate Masks" button to display and register the masked image. 
    * If you are not satisfied with the results, use the "Reselect" or "Clear All" button to remove them.
1. Repeat steps 1-2 for each object you want to track.
1. After selecting all targets, click the "Start Tracking" button to initiate the tracking process.

Once tracking starts, masks and their positions will be published.
## Visualizing
We provide two ways to visualize the results: masked images and images with points indicating the positions of the tracked targets.

Run the following command to start the ROS node that subscribes to the results from `segment_and_tracking_ros.py` and visualizes them:
```
python visualizer.py
```
You can visualize the images in two ways by specifying command-line arguments:
1. Masked image (default)
* This is the default visualization mode. To disable it, add the `--disable_mask_mode` argument when running `visualizer.py`.
2. Image with points
* You can visualize the positions of the tracked targets by adding the `--point_mode` argument when running `visualizer.py`. This mode displays points at the tracked target positions.

For example, to run the visualizer with point mode enabled in addition to mask mode, use the following command:
```
python visualizer.py --point_mode
```
To view the visualized image, run the following command in another terminal:
```
rosrun rqt_image_view rqt_image_view 
```