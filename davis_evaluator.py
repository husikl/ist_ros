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
from PIL import Image
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image, ImageShow, ImageDraw
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
        save_all_path=None,
        display_annotation=False,
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
        self.display_annotation = display_annotation

        self.lock = threading.Lock()
        print(f"Mask mode: {mask_mode}, Save all mode: {save_all_mode}")

    def init_segmentation(self, image, annotation_path):
        if not os.path.exists(annotation_path):
            raise FileNotFoundError(f"Annotation file not found: {annotation_path}")
        annotation = Image.open(annotation_path)
        if self.display_annotation:
            self.separate_objects(annotation)        
        # get palette for evaluation
        self.palette = annotation.getpalette()
        
        with self.lock:
            self.interact_control.image_queue.put(image)
            init_interactive_segmentation(
                image, self.res_manager, self.interact_control, self.tk_root
            )

    def separate_objects(self, palette_image):
        # Check object number
        palette_array = np.array(palette_image)
        unique_ids = np.unique(palette_array)
        object_ids = unique_ids[unique_ids != 0]
        
        width, height = palette_image.size
        new_width, new_height = width // 4, height // 4
        
        combined_image = Image.new('RGB', (new_width * len(object_ids), new_height))
        
        for i, obj_id in enumerate(object_ids):
            mask = Image.fromarray((palette_array == obj_id).astype(np.uint8) * 255)

            mask_rgb = mask.convert('RGB').resize((new_width, new_height), Image.LANCZOS)
            
            draw = ImageDraw.Draw(mask_rgb)
            font_size = max(10, new_height // 20)
            draw.text((5, 5), f"Object {obj_id}", fill=(255, 0, 0), font_size=font_size)
            
            combined_image.paste(mask_rgb, (i * new_width, 0))
        
        ImageShow.show(combined_image)

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

            # save image
            out_img = Image.fromarray(masks)
            if self.palette is not None:
                out_img.putpalette(self.palette)
            out_img.save(os.path.join(output_folder, f'{frame_id:05}.png'))

            pbar.update(1)

        pbar.close()

        print("Image processing finished")
        self.tk_root.destroy()

if __name__ == "__main__":
    # python davis_evaluator.py --davis_year 2016 --davis_task blackswan     
    # Assuming args is being parsed and appropriate paths are set
    args = parse_args()
    res_manager, interact_control = build_control(args)

    # Update these paths
    data_root = "/media/medical/Data/yamada/davis_evaluation"
    input_folder = f"{data_root}/DAVIS_{args.davis_year}/JPEGImages/480p/{args.davis_task}"
    # output_folder = args.output_path  # Ensure this is set appropriately in your arguments
    output_folder = f"{data_root}/DAVIS_{args.davis_year}/results/{args.davis_task}"
    annotation_path = f"{data_root}/DAVIS_{args.davis_year}/Annotations/480p/{args.davis_task}/00000.png"
    os.makedirs(output_folder, exist_ok=True)
    display_annotation = False if args.davis_year == 2016 else True
    args.save_all_mode = True
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
        save_all_path=args.save_all_path,
        display_annotation=display_annotation
    )

    resource_handler.init_segmentation(example_image,annotation_path)
    resource_handler.process_images(input_folder, output_folder)
    tk_root.mainloop()
