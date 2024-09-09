import os
import cv2
from PIL import Image
from tqdm import tqdm
from utils.core_utils import inference_masks
import threading
from utils.core_utils import build_control, init_interactive_segmentation, inference_masks
from configs.get_args import parse_args
import tkinter as tk
from queue import Queue
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
        save_all_mode=False,
        save_all_path=None
    ):
        self.command_queue = command_queue
        self.initial_image = initial_image
        self.res_manager = res_manager
        self.interact_control = interact_control
        self.tk_root = tk_root
        self.mask_threshold = mask_threshold
        self.mask_mode = mask_mode
        self.save_all_mode = save_all_mode
        self.save_all_path = save_all_path

        self.lock = threading.Lock()
        print(f"Mask mode: {mask_mode}, Save all mode: {save_all_mode}")

    def init_segmentation(self, image):
        with self.lock:
            self.interact_control.image_queue.put(image)
            init_interactive_segmentation(
                image, self.res_manager, self.interact_control, self.tk_root
            )
    
    def process_images(self, input_folder, output_folder):
        image_files = sorted([f for f in os.listdir(input_folder) if f.endswith('.jpg') or f.endswith('.png')])
        total_images = len(image_files)

        # Create output folder if it doesn't exist
        if not os.path.exists(output_folder):
            os.makedirs(output_folder)

        print("Image processing start")
        pbar = tqdm(total=total_images, unit='images')

        for frame_id, image_file in enumerate(image_files, start=1):
            image_path = os.path.join(input_folder, image_file)
            image = cv2.imread(image_path)
            if image is None:
                print(f"Error loading image: {image_path}")
                continue
            
            # Segment the current image
            masks, _ = inference_masks(cv2.cvtColor(image, cv2.COLOR_BGR2RGB), self.res_manager, save_mode=self.save_all_mode)
            
            # Save masks
            # if self.save_all_mode:
            # Create directories for masks if they don't exist
            mask_all_dir = os.path.join(output_folder, "mask_all")
            if not os.path.exists(mask_all_dir):
                os.makedirs(mask_all_dir)

            if frame_id == 1:  # Prepare save directories for each target if needed
                if self.mask_mode:
                    os.makedirs(os.path.join(output_folder, "mask_target"), exist_ok=True)

            if self.mask_mode:
                # Save all masks in DAVIS format: mask_all_{frame_id:06}.png
                Image.fromarray(masks[0]).save(os.path.join(mask_all_dir, f"mask_all_{frame_id:06}.png"))

            for target_id, mask in enumerate(masks[1:], start=1):
                if self.mask_mode:
                    # Save each target mask in a separate directory
                    target_mask_dir = os.path.join(output_folder, f"mask_target{target_id}")
                    if not os.path.exists(target_mask_dir):
                        os.makedirs(target_mask_dir)
                    Image.fromarray(mask).save(os.path.join(target_mask_dir, f"mask_target{target_id}_{frame_id:06}.png"))

            pbar.update(1)

        pbar.close()

        print("Image processing finished")
        self.tk_root.destroy()

if __name__ == "__main__":
    # Assuming args is being parsed and appropriate paths are set
    args = parse_args()
    res_manager, interact_control = build_control(args)

    # Update these paths
    input_folder = "/home/safetyu-desktop1/benchmarks_related/davis-2017/DAVIS/JPEGImages/480p/bear"
    # output_folder = args.output_path  # Ensure this is set appropriately in your arguments
    output_folder = "benchmark"

    os.makedirs(output_folder, exist_ok=True)

    # Initialize an example image for segmentation initialization
    example_image_path = sorted([f for f in os.listdir(input_folder) if f.endswith('.jpg') or f.endswith('.png')])[0]
    example_image = cv2.imread(os.path.join(input_folder, example_image_path))
    example_image = cv2.cvtColor(example_image, cv2.COLOR_BGR2RGB)
    
    tk_root = tk.Tk()
    command_queue = Queue()
    
    resource_handler = ResourceHandler(
        command_queue,
        example_image,
        res_manager,
        interact_control,
        tk_root,
        mask_threshold=args.mask_threshold,
        mask_mode=args.mask_mode,
        save_all_mode=args.save_all_mode,
        save_all_path=args.save_all_path
    )

    resource_handler.init_segmentation(example_image)
    resource_handler.process_images(input_folder, output_folder)
    tk_root.mainloop()
