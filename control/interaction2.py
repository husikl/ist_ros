import tkinter as tk
from PIL import Image, ImageTk
import torch
import numpy as np
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
        self.tk_root = None
        self.rect = None
        self.image_queue = Queue()
        # Inputs
        self.boxes = None        
        self.anchors = []
        self.anchors_label = []
        self.current_label = 1

        # Sam
        self.num_masks = 1
        self.masks_array = []
        self.masks = []

    def on_click(self, event=None):
        self.init_x = event.x
        self.init_y = event.y
        if self.rect is not None:
            self.canvas.delete(self.rect)
            self.rect = None
            self.boxes = None

    def on_release(self, event=None):
        self.end_x = event.x
        self.end_y = event.y
        self.push_box()

    def on_move_press(self, event):
        if self.rect is not None:
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
        self.boxes = np.array([ori_init_x, ori_init_y, ori_end_x, ori_end_y])

    def on_right_click(self, event=None):
        if event:
            self.right_click_x = event.x
            self.right_click_y = event.y
            ori_right_x, ori_right_y = self.convert_coodinates(self.right_click_x,self.right_click_y)
            self.anchors.append([ori_right_x, ori_right_y])
            self.anchors_label.append(self.current_label)
            
            if self.current_label == 1:
                circle_radius = 5
                self.canvas.create_oval(
                    self.right_click_x - circle_radius,
                    self.right_click_y - circle_radius,
                    self.right_click_x + circle_radius,
                    self.right_click_y + circle_radius,
                    fill='blue'
                )
            else:
                cross_size = 5
                width = 2
                self.canvas.create_line(
                    self.right_click_x - cross_size,
                    self.right_click_y - cross_size,
                    self.right_click_x + cross_size,
                    self.right_click_y + cross_size,
                    fill='red',
                    width=width
                )
                self.canvas.create_line(
                    self.right_click_x + cross_size,
                    self.right_click_y - cross_size,
                    self.right_click_x - cross_size,
                    self.right_click_y + cross_size,
                    fill='red',
                    width=width
                )

    def toggle_label(self):
        self.current_label = 1 - self.current_label  # Switch between 0 and 1
        if self.current_label == 1:
            self.label_button.config(text="Foreground", bg="blue", fg="white",
                                    activebackground="darkblue", activeforeground="white")
        else:
            self.label_button.config(text="Background", bg="red", fg="white",
                                    activebackground="darkred", activeforeground="white")

    def predict_by_boxes(self):
        image_np = np.array(self.image)
        self.sam.set_image(image_np)
        anchors_np = self.safe_np_array(self.anchors)
        anchors_label_np = self.safe_np_array(self.anchors_label)
        print(anchors_np, anchors_label_np, self.boxes)
        mask, _, _ = self.sam.predict(
            point_coords=anchors_np,
            point_labels=anchors_label_np,
            box=self.boxes,
            multimask_output=False,
        )
        mask_image = (mask[0] * 255).astype(np.uint8)
        mask = torch.from_numpy(mask[0]).unsqueeze(0)
        print(f"Mask{self.num_masks} generated")
        return mask, mask_image

    def safe_np_array(self, x):
        if x == []:
            x_np = None
        else:
            x_np = np.array(x)
        return x_np
    
    def predict_mask(self):
        mask, mask_image = self.predict_by_boxes()
        self.masks.append(mask)
        self.masks_array.append(mask_image)
        # Display the masked image
        current_image_np = np.array(self.image)
        masked_image = add_masks_to_image(current_image_np, self.masks_array,bgr=False)        
        masked_image_pil = Image.fromarray(masked_image)
        self.mask_photo = ImageTk.PhotoImage(self.resize_image(masked_image_pil))
        self.canvas.itemconfig(self.image_id, image=self.mask_photo)
        self.canvas.update()
        # Clear inputs and remove drawn objects
        self.boxes = None
        self.anchors = []
        self.anchors_label = []
        if self.rect is not None:
            self.canvas.delete(self.rect)
            self.rect = None

        # Remove all drawn circles and crosses
        drawn_objects = self.canvas.find_all()
        for obj in drawn_objects:
            if obj != self.image_id:
                self.canvas.delete(obj)

        self.canvas.update()
        self.num_masks += 1
    
    def reset(self):
        self.canvas.delete("all")
        # Reset Inputs
        self.rect = None        
        self.boxes = None
        self.anchors = []
        self.anchors_label = []

        # Reset sam
        self.num_masks = 0
        self.masks_array = []
        self.masks = []
        self.mask_photo = None

        # Reset Image
        self.image_id = self.canvas.create_image(0, 0, anchor=tk.NW, image=self.photo)
        self.canvas.update()
        print("All Cleared")

    def reselect(self):
        # Check if selection is done.
        if self.boxes is not None or self.anchors!=[] or self.anchors_label!=[]:
            if self.rect is not None:
                self.canvas.delete(self.rect)
                self.rect = None

            # Remove all drawn circles and crosses
            drawn_objects = self.canvas.find_all()
            for obj in drawn_objects:
                if obj != self.image_id:
                    self.canvas.delete(obj)            
            # Reset Inputs
            self.boxes = None
            self.anchors = []
            self.anchors_label = []
        else:
            # Remove latest output
            print(self.num_masks, len(self.masks_array), len(self.masks))
            # Display previous version
            if len(self.masks_array) > 0:
                self.num_masks -= 1
                self.masks_array.pop(-1)
                self.masks.pop(-1)
                if len(self.masks_array) == 0:
                    self.mask_photo = self.photo
                else:                
                    current_image_np = np.array(self.image)
                    masked_image = add_masks_to_image(current_image_np, self.masks_array,bgr=False)        
                    masked_image_pil = Image.fromarray(masked_image)
                    self.mask_photo = ImageTk.PhotoImage(self.resize_image(masked_image_pil))
            else:
                # All mask cleared
                self.mask_photo = self.photo
            self.image_id = self.canvas.create_image(0, 0, anchor=tk.NW, image=self.mask_photo)        
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
        button_frame_top = tk.Frame(self.tk_root)
        button_frame_top.pack(pady=5)

        button_frame_bottom = tk.Frame(self.tk_root)
        button_frame_bottom.pack(pady=5)

        button_bg_color = "#616161"
        button_fg_color = "#FFFFFF"
        button_active_bg_color = "#424242"
        button_active_fg_color = "#FFFFFF"

        label_button = tk.Button(button_frame_top, text="Foreground", command=self.toggle_label,
                                width=15, height=2, font=("Helvetica", 12), bg="blue", fg="white",
                                activebackground="darkblue", activeforeground="white")
        label_button.pack(side=tk.LEFT, padx=10, pady=5)
        self.label_button = label_button

        mask_button = tk.Button(button_frame_top, text="Generate Mask", command=self.predict_mask,
                                width=15, height=2, font=("Helvetica", 12), bg=button_bg_color, fg=button_fg_color,
                                activebackground=button_active_bg_color, activeforeground=button_active_fg_color)
        mask_button.pack(side=tk.LEFT, padx=10, pady=5)
        self.mask_button = mask_button

        self.track_pressed = 0
        track_button = tk.Button(button_frame_top, text="Start Tracking", command=self.tracking,
                                width=15, height=2, font=("Helvetica", 12), bg="green", fg="white",
                                activebackground="darkgreen", activeforeground="white")
        track_button.pack(side=tk.LEFT, padx=10, pady=5)
        self.track_button = track_button

        reselect_button = tk.Button(button_frame_bottom, text="Reselect", command=self.reselect,
                                    width=15, height=2, font=("Helvetica", 12), bg=button_bg_color, fg=button_fg_color,
                                    activebackground=button_active_bg_color, activeforeground=button_active_fg_color)
        reselect_button.pack(side=tk.LEFT, padx=10, pady=5)
        self.reselect_button = reselect_button

        reset_button = tk.Button(button_frame_bottom, text="Clear All", command=self.reset,
                                width=15, height=2, font=("Helvetica", 12), bg=button_bg_color, fg=button_fg_color,
                                activebackground=button_active_bg_color, activeforeground=button_active_fg_color)
        reset_button.pack(side=tk.LEFT, padx=10, pady=5)
        self.reset_button = reset_button
        
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
        for i, maskk in enumerate(self.masks):
            print(i, maskk.size())

        return_masks = torch.stack(self.masks, dim=0)
        return_masks = return_masks.to(self.sam.device)
        print(return_masks.size())
        print("Start tracking")
        return return_masks
