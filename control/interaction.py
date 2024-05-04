import tkinter as tk
from PIL import Image, ImageTk, ImageDraw
import torch
import numpy as np
import cv2
from queue import Queue
import rospy
from std_srvs.srv import Empty
from utils.utils import add_masks_to_image
class interaction_control:
    def __init__(self, sam=None, image=None):
        self.sam = sam
        self.image = (
            image if image is not None else np.zeros((512, 512, 3), dtype=np.uint8)
        )
        self.boxes = []
        self.masks = None
        self.tk_root = None
        self.image_queue = Queue()
        self.anchors = []
        self.rect = None

    def on_click(self, event=None):
        self.init_x = event.x
        self.init_y = event.y
        self.rect = None

    def on_release(self, event=None):
        self.end_x = event.x
        self.end_y = event.y
        self.push_box()

    def on_move_press(self, event):
        if self.rect:
            if event.x < 0:
                end_x = 0
            else:
                end_x =  min(self.photo_width, event.x)
            if event.y < 0:
                end_y = 0
            else:
                end_y = min(self.photo_height, event.y)
            self.canvas.coords(self.rect, self.init_x, self.init_y, end_x, end_y)
        else:
            self.rect = self.canvas.create_rectangle(self.init_x, self.init_y, self.init_x+1, self.init_x+1, outline='white')            

    def push_box(self):
        ori_init_x, ori_init_y = self.convert_coodinates(self.init_x, self.init_y)
        ori_end_x, ori_end_y = self.convert_coodinates(self.end_x, self.end_y)
        self.boxes.append([ori_init_x, ori_init_y, ori_end_x, ori_end_y])

    def on_right_click(self, event=None):
        if event:
            self.right_click_x = event.x
            self.right_click_y = event.y
            ori_right_x, ori_right_y = self.convert_coodinates(self.right_click_x,self.right_click_y)
            self.anchors.append(np.array([[ori_right_x, ori_right_y]]))
            
            circle_radius = 5
            self.canvas.create_oval(
                self.right_click_x - circle_radius,
                self.right_click_y - circle_radius,
                self.right_click_x + circle_radius,
                self.right_click_y + circle_radius,
                fill='red'
            )

    def predict_by_boxes(self):
        image_np = np.array(self.image)
        self.sam.set_image(image_np)
        masks_array = []  # Initialize an empty list to store the mask images
        if len(self.boxes) == 1:
            trans_boxes = np.array(self.boxes[0])
            masks, _, _ = self.sam.predict(
                point_coords=None,
                point_labels=None,
                box=trans_boxes[None, :],
                multimask_output=False,
            )
            mask_image = (masks[0] * 255).astype(np.uint8)
            masks_array.append(mask_image)  # Append the mask image to masks_array
            masks = torch.from_numpy(masks).unsqueeze(0)
            Image.fromarray(mask_image).save(f"outputs/images/mask_to_track0.png")           
            print(f"{len(self.boxes)} mask generated")
        elif len(self.boxes) > 1:
            maskss = []
            if len(self.anchors) == len(self.boxes):
                for i, i_box in enumerate(self.boxes)                                     :
                    self.sam.set_image(image_np)
                    trans_boxes = np.array(i_box)
                    masks, _, _ = self.sam.predict(
                        point_coords=self.anchors[i],
                        point_labels=np.array([1]),
                        box=trans_boxes[None, :],
                        multimask_output=True,
                    )
                    maskss.append(torch.from_numpy(masks[0]).unsqueeze(0))
                    mask_image = (masks[0] * 255).astype(np.uint8)
                    masks_array.append(mask_image)  # Append the mask image to masks_array
                    Image.fromarray(mask_image).save(f"outputs/images/mask_to_track{i}.png")
                print(f"{len(self.boxes)} masks generated")

            else:
                raise ValueError("Error: The number of boxes and anchors are unmatched.")    
            masks = torch.stack(maskss, dim=0)
        else:
            raise ValueError("No boxes drawn")
        return_masks = masks.to(self.sam.device)
        return return_masks, masks_array

    def predict_masks(self):
        self.masks, mask_images = self.predict_by_boxes()
        # Display the masked image
        if len(mask_images) > 0:
            original_image = np.array(self.image)
            masked_image = add_masks_to_image(original_image, mask_images, bgr=False)            
            masked_image_pil = Image.fromarray(masked_image)
            self.mask_photo = ImageTk.PhotoImage(self.resize_image(masked_image_pil))

            self.canvas.itemconfig(self.image_id, image=self.mask_photo)
            self.canvas.update()
    
    def reset(self):
        self.canvas.delete("all")
        self.rect = None        
        self.boxes = []
        self.anchors = []
        self.masks = None
        self.mask_photo = None
        self.image_id = self.canvas.create_image(0, 0, anchor=tk.NW, image=self.photo)
        self.canvas.update()
        
    def tracking(self):
        self.track_pressed = 1

    def resize_image(self, image, max_width=1300, max_height=800):
        width, height = image.size
        
        if width > max_width or height > max_height:
            ratio = min(max_width / width, max_height / height)
            self.photo_width = int(width * ratio)
            self.photo_height = int(height * ratio)
            self.ratio_x = width / self.photo_width
            self.ratio_y = height / self.photo_height
            image = image.resize((self.photo_width, self.photo_height))
        else:
            self.photo_width, self.photo_height = width, height
        return image

    def convert_coodinates(self,photo_x,photo_y):
        if hasattr(self, 'ratio_x') and hasattr(self, 'ratio_y'):
            original_x = photo_x*self.ratio_x
            original_y = photo_y*self.ratio_y
            return original_x, original_y
        else:
            return photo_x, photo_y

    def do_interact(self, image=None, tk_root=None, ros=False):
        if tk_root:  # If a new Tk root is provided
            self.tk_root = tk_root  # Update the stored Tk root
        if not self.tk_root:
            raise ValueError("Tk root must be initialized.")
        self.tk_root.title("Target Selection")
        if image is not None:
            self.image = image
        elif not self.image_queue.empty():
            image_array = self.image_queue.get()
            self.image = Image.fromarray(image_array)

        if self.image is not None:
            self.photo = ImageTk.PhotoImage(self.resize_image(self.image))
            if not hasattr(self, "canvas"):  # Create canvas if not already exists
                self.canvas = tk.Canvas(self.tk_root, width=self.photo_width, height=self.photo_height)
                self.canvas.pack()
                self.canvas.bind("<Button 1>", self.on_click)
                self.canvas.bind("<B1-Motion>",self.on_move_press)
                self.canvas.bind("<ButtonRelease 1>", self.on_release)
                self.canvas.bind("<Button 3>", self.on_right_click)

            self.image_id = self.canvas.create_image(0, 0, anchor=tk.NW, image=self.photo)  # Display original image
        button_frame = tk.Frame(self.tk_root)
        button_frame.pack(pady=10)
        button_bg_color = "#616161"
        button_fg_color = "#FFFFFF"
        button_active_bg_color = "#424242"
        button_active_fg_color = "#FFFFFF"

        mask_button = tk.Button(button_frame, text="Generate Masks", command=self.predict_masks,
                                width=15, height=2, font=("Helvetica", 12), bg=button_bg_color, fg=button_fg_color,
                                activebackground=button_active_bg_color, activeforeground=button_active_fg_color)
        mask_button.pack(side=tk.LEFT, padx=10, pady=5)

        reset_button = tk.Button(button_frame, text="Reselect", command=self.reset,
                                width=15, height=2, font=("Helvetica", 12), bg=button_bg_color, fg=button_fg_color,
                                activebackground=button_active_bg_color, activeforeground=button_active_fg_color)
        reset_button.pack(side=tk.LEFT, padx=10, pady=5)

        self.track_pressed = 0
        track_button = tk.Button(button_frame, text="Start Tracking", command=self.tracking,
                                width=15, height=2, font=("Helvetica", 12), bg="green", fg="white",
                                activebackground="darkgreen", activeforeground="white")
        track_button.pack(side=tk.LEFT, padx=10, pady=5)
        while not self.track_pressed:
            self.tk_root.update()
            self.tk_root.after(10)
        #Selection completed
        if ros:
            rospy.wait_for_service('run_inference')
            try:
                start_inference = rospy.ServiceProxy('run_inference', Empty)
                start_inference()
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

        print("Start tracking")
        return self.masks
