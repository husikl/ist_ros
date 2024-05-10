# Offline Segmentation and Tracking
This document provides a detailed guide on how to perform the offline segmentation and tracking.
## Command Line Usage
To perform offline video processing, run the following command:
```
python segment_and_track.py --input_path ./inputs/input_video.mp4 --output_path ./outputs/output_video.mp4
```
## Command Line Arguments
* `--input_path`: Path to the input video.
* `--output_path`: Path to the output video.
* `--point_mode`: Track the targets using points.
* `--disable_mask_mode`: Disable the mask mode for visualization.
* `--save_all_mode`: Save all visualized frames and/or point positions. The saved data depends on the selected visualizing mode.
* `--save_all_path`: Path to save data when `--save_all_mode` is enabled.

## Workflow
1. Run the scrpt with the appropriate command line arguments.
1. A tkinter GUI window will appear.
1. Select one target at a time using the following operations:
    * Draw a bounding box around the object by clicking and dragging the mouse.
    * Right-click on the object to specify it as foreground or background. You can change the mode using bottun.
1. Click the "Generate Masks" button to display and register the masked image. 
    * If you are not satisfied with the results, use the "Reselect" or "Clear All" button to remove them.
1. Repeat steps 3-4 for each object you want to track.
1. After selecting all targets, click the "Start Tracking" button to initiate the tracking process.

For detailed instructions on using the GUI, please refer to `GUI.md`.
## Outputs
The script will process the input video and save the output video with the tracked objects at the specified output path (`--output_path`).
If `--save_all_mode` is enabled, the script will also save all visualized frames and/or mask positions at the specified path (`--save_all_path`).
