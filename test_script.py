import cv2
import time
import threading
from queue import Queue
import tkinter as tk
from main import build_control, init_interactive_segmentation, inference_masks
import numpy as np
from tqdm import tqdm
from configs.get_args import parse_args
from utils import add_masks_to_image
class ResourceHandler:
    def __init__(
        self,
        command_queue,
        initial_image,
        res_manager,
        interact_control,
        tk_root
    ):
        self.command_queue = command_queue
        self.initial_image = initial_image
        self.res_manager = res_manager
        self.interact_control = interact_control

        self.lock = threading.Lock()
        self.image_to_process = None
        self.tk_root = tk_root
       
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
        colors = [(255,0,0), (0,255,0),(0,0,255),(255,255,0),(0,255,255)]
        cap = cv2.VideoCapture(input_video_path)

        orig_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        orig_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        orig_fps = cap.get(cv2.CAP_PROP_FPS)
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        out = cv2.VideoWriter(
            output_video_path, fourcc, orig_fps, (orig_width, orig_height)
        )  # Adjust frame size

        total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        print("Video processing start")
        pbar = tqdm(total=total_frames, unit='frames')        
        ii = 0
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            if ii > 10:
                break
            ii+=1
            masks = inference_masks(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB), res_manager,ros=False)
            masked_image = add_masks_to_image(frame, masks)
            # Write frame with masks to output video
            out.write(masked_image)
            pbar.update(1)
            time.sleep(1.0/100.0)
        pbar.close()

        # Release resources
        cap.release()
        out.release()
        cv2.destroyAllWindows()
        print("Video processing finished")
        self.tk_root.destroy()


if __name__ == "__main__":
    args = parse_args()
    res_manager, interact_control = build_control(args)

    input_video_path =  args.input_path
    output_video_path = args.output_path  
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
        tk_root
    )

    resource_handler.init_segmentation(initial_image)
    resource_handler.process_video(input_video_path, output_video_path)
    tk_root.mainloop()
