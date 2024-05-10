# Target selection through Graphic User Interface

![Target selection](assets/target_selection.gif)

## Workflow
1. Select one target at a time using the mouse. See the next section for detailed instructions.
1. Click the "Generate Masks" button to display and register the masked image. 
    * If you are not satisfied with the results, use the "Reselect" or "Clear All" button to remove them.
1. Repeat steps 1-2 for each object you want to track.
1. After selecting all targets, click the "Start Tracking" button to initiate the tracking process.

## How to Select Targets
You can select targets using any combination of prompts:
* Bounding box: Draw a bounding box around the object by clicking and dragging the mouse while holding the left mouse button.
* Point(Foreground): Add blue dots to the target by right-clicking on the object. The pointed area will be included in the mask.
* Point(Background): Add red crosses to the background by right-clicking on the area you want to exclude. The pointed area will not be included in the mask.

You can switch between the foreground and background modes using the "Foreground/Background" button.
## Buttons
* Foreground/Background: Switches the mode for point selection between foreground and background.
* Generate mask: After selecting a target, click this button to generate and register a mask.
* Start Tracking: After selecting all targets you want to track, click this button to start the tracking process.
* Reselect: Reverts to the previous state, allowing you to select the object again.
* Clear All: Clears all selected targets and masks, allowing you to start the selection process from the beginning.