import cv2
import time
import threading
from queue import Queue
import tkinter as tk
from utils.core_utils import build_control, init_interactive_segmentation, inference_masks
import numpy as np
from tqdm import tqdm
import os
import pandas as pd
from PIL import Image
from configs.get_args import parse_args
from utils.utils import add_masks_to_image, get_color
class ResourceHandler:
    def __init__(
        self,
        command_queue,
        initial_image,
        res_manager,
        interact_control,
        tk_root,
        mask_threshold=220,
        mask_mode=True,
        point_mode=False,
        center_radius=5,
        save_all_mode=False,
        save_all_path=None
    ):
        self.command_queue = command_queue
        self.initial_image = initial_image
        self.res_manager = res_manager
        self.interact_control = interact_control

        self.lock = threading.Lock()
        self.image_to_process = None
        self.tk_root = tk_root
        self.mask_threshold = mask_threshold
        self.mask_mode = mask_mode
        self.point_mode = point_mode
        self.center_radius = center_radius
        self.save_all_mode = save_all_mode
        self.save_all_path = save_all_path
       
    def process_commands(self):
        while True:
            if not self.command_queue.empty():
                command, image = self.command_queue.get()
                if command == "init_segmentation":
                    self.init_segmentation(image)
            time.sleep(1.0/65.0)

    def init_segmentation(self, image):
        with self.lock:
            interact_control.image_queue.put(image)
            init_interactive_segmentation(
                image, res_manager, interact_control, self.tk_root
            )
    
    def process_video(self, input_video_path, output_video_path):
        cap = cv2.VideoCapture(input_video_path)

        orig_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        orig_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        orig_fps = cap.get(cv2.CAP_PROP_FPS)
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        out = cv2.VideoWriter(
            output_video_path, fourcc, orig_fps, (orig_width, orig_height)
        )  # Adjust frame size
        if self.save_all_mode:
            os.makedirs(self.save_all_path,exist_ok=True)
        total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        print("Video processing start")
        pbar = tqdm(total=total_frames, unit='frames')        
        frame_id = 1
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            if frame_id > 10:
                break
            masks, center_points = inference_masks(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB), res_manager,save_mode=self.save_all_mode)
            
            # Save all information
            if self.save_all_mode:
                if frame_id==1: # Prepare save files
                    for target_id in range(1, center_points.shape[0]+1):
                        if self.mask_mode:
                            os.makedirs(f"{self.save_all_path}/mask_target{target_id}", exist_ok=True)
                    point_df = pd.DataFrame() # for point mode 
                    if self.mask_mode:
                        os.makedirs(f"{self.save_all_path}/mask_all",exist_ok=True)
                if self.mask_mode:
                    Image.fromarray(masks[0]).save(f"{self.save_all_path}/mask_all/mask_all_{frame_id:06}.png")
                frame_df = pd.DataFrame() # for point mode
                frame_df["frame_id"] = [frame_id]
                for target_id, (mask,center) in enumerate(zip(masks[1:], center_points),start=1):
                    if self.mask_mode:
                        Image.fromarray(mask).save(f"{self.save_all_path}/mask_target{target_id}/mask_target{target_id}_{frame_id:06}.png")
                    if self.point_mode:
                        frame_df[f"center_target{target_id}"] = [center[:2]]
                        frame_df[f"detect_target{target_id}"] = [center[2]]
                point_df = pd.concat([point_df,frame_df])
                masks = masks[1:] # Remove a first image that contains all masks
                        
            # Visualization by video file
            if self.mask_mode:
                frame = add_masks_to_image(frame, masks, threshold=self.mask_threshold)

            if self.point_mode:
                for ii, center in enumerate(center_points):
                    if bool(center[2]) is True: # Check if target is detected
                        if self.mask_mode:
                            color = [0,0,255]
                        else:
                            color = get_color(ii,bgr=True)
                        cv2.circle(
                            frame,
                            (int(center[0]), int(center[1])),
                            int(self.center_radius),
                            color,
                            thickness=-1,
                        )   
            # Write frame with masks to output video
            out.write(frame)
            pbar.update(1)
            time.sleep(1.0/100.0)
            frame_id += 1
        pbar.close()

        # Release resources
        cap.release()
        out.release()
        cv2.destroyAllWindows()
        point_df.to_pickle(f"{self.save_all_path}/points_dataframe.pkl")
        point_df.to_csv(f"{self.save_all_path}/points_dataframe.csv")
        print("Video processing finished")
        self.tk_root.destroy()


if __name__ == "__main__":
    args = parse_args()
    res_manager, interact_control = build_control(args)

    input_video_path =  args.input_path
    output_video_path = args.output_path
    output_dir = os.path.dirname(output_video_path)
    os.makedirs(output_dir, exist_ok=True)       
    cap = cv2.VideoCapture(input_video_path)

    ret, frame = cap.read()
    cap.release()
    initial_image = frame
    initial_image = cv2.cvtColor(initial_image, cv2.COLOR_BGR2RGB)
    tk_root = tk.Tk()
    command_queue = Queue()
    # Initialize ResourceHandler with res_manager and interact_control
    resource_handler = ResourceHandler(
        command_queue,
        initial_image,
        res_manager,
        interact_control,
        tk_root,
        mask_threshold=args.mask_threshold, mask_mode=args.mask_mode, point_mode=args.point_mode, center_radius=args.center_radius, save_all_mode=args.save_all_mode, save_all_path=args.save_all_path
    )

    resource_handler.init_segmentation(initial_image)
    resource_handler.process_video(input_video_path, output_video_path)
    tk_root.mainloop()
