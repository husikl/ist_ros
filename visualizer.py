import rospy
import yaml
import cv2
import threading
import numpy as np
from collections import defaultdict, deque
from scipy.optimize import minimize
from sensor_msgs.msg import Image as ImageMsg
from geometry_msgs.msg import PoseArray, Point
from cv_bridge import CvBridge, CvBridgeError
import copy
from math import sqrt
from std_msgs.msg import Float32MultiArray
from functools import partial
from configs.get_args import parse_args
from utils import get_color, add_masks_to_image
class ToolTip:
    def __init__(self, x, y, r, id, resize_scale_x, resize_scale_y, resize_scale_r, x0, y0, gamma=1.0, flag_resize_image=False, flag_crop_image=False):
        if flag_resize_image or flag_crop_image:
            x, y, r = self.map_coordinates_to_resized(x,y,r, resize_scale_x, resize_scale_y, resize_scale_r, x0, y0)
        self.x, self.y, self.r, self.gamma = x, y, r, gamma
        self.id = id

    def map_coordinates_to_resized(self, x_ori, y_ori, r_ori, resize_scale_x, resize_scale_y, resize_scale_r, x0, y0):
        x_resized = (x_ori-x0) / resize_scale_x
        y_resized = (y_ori-y0) / resize_scale_y
        r_resized = r_ori / resize_scale_r
        return x_resized, y_resized, r_resized
    
class MaskProcessor:
    def __init__(self, input_topic, resize_scale, x0_crop=None, x1_crop=None, y0_crop=None, y1_crop=None, mask_mode=True, circle_mode=False):
        self.mask_mode = mask_mode
        self.circle_mode = circle_mode
        self.bridge = CvBridge()
        self.init = False
        self.masks = None
        self.latest_image = None
        if resize_scale!=1:
            self.flag_resize_image = True
            self.resize_scale_x = None
            self.resize_scale_y = None            
        else:
            self.flag_resize_image = False
            self.resize_scale_x = 1
            self.resize_scale_y = 1
        self.resize_scale_r = 3.0
        self.resize_scale = resize_scale
        crop_coordinates = [x0_crop,x1_crop,y0_crop,y1_crop]
        if all([a is not None for a in crop_coordinates]):
            self.x0 = x0_crop
            self.x1 = x1_crop
            self.y0 = y0_crop
            self.y1 = y1_crop
            self.flag_crop_image = True
        elif any([a is not None for a in crop_coordinates]):
            raise Exception("Please specify all values when cropping")
        else:
            self.flag_crop_image = False
            self.x0 = 0
            self.x1 = 0
            self.y0 = 0
            self.y1 = 0
        self.tooltip = None                
        self.image_sub = rospy.Subscriber(
            input_topic, ImageMsg, self.image_callback
        )
        self.image_pub = rospy.Publisher("/visualized_image", ImageMsg, queue_size=1)
        if mask_mode:
            self.masks_sub = rospy.Subscriber(
                "masks", Float32MultiArray, self.masks_callback
            )
        if circle_mode:
            self.tips = []            
            self.tools_sub = rospy.Subscriber(
                "tracked_tools", PoseArray, self.tools_callback
            )
            # self.multi_tips = defaultdict(deque)
        print(f"Mask mode: {mask_mode}, Circle mode: {circle_mode}")
        threading.Thread(target=self.visualize_image).start()
        rospy.sleep(1.0)
        rospy.loginfo("Visualizer initialized")

    def tools_callback(self, msg):
        if self.init:
            tips = [None] * len(msg.poses)  # Pre-allocate list with None
            for i, p in enumerate(msg.poses):
                if p.orientation.x > 0:  # Tool is detected
                    tip = self.tooltip(p.position.x, p.position.y, p.position.z, i)
                    tips[i] = tip  # Place the tip at the corresponding index

                    # can I remove this process?
                    # if i not in self.multi_tips:
                    #     self.multi_tips[i] = deque(maxlen=10)
                    # self.multi_tips[i].append(tip)
                    # # Create a temporary variable to store x, y positions for averaging
                    # temp_tips_positions = [np.array([t.x, t.y]) for t in self.multi_tips[i]]

                    # # Compute the mean of the positions
                    # mean_tip_position = np.mean(temp_tips_positions, axis=0)

                    # # Update the SurgTip object's x and y with the mean values
                    # # tip.x, tip.y = mean_tip_position[0], mean_tip_position[1]

                    # # Place the tip at the corresponding index
                    # tips[i] = tip  
                else :
                    tips[i] = None
                    # print("not detected i = ", i)

            self.tips = tips  # Directly assign, keeping None for undetected tools

    def visualize_image(self):
        while True:
            if self.init:
                vis_image = self.latest_image.copy()
                try:
                    if self.mask_mode and self.masks is not None:
                        try:
                            vis_image = add_masks_to_image(vis_image, self.masks)
                        except:
                            raise ValueError("Set the same way to preprocess as the image processor when masking")                            
                    if self.circle_mode:
                        for i, tip in enumerate(self.tips):
                            if tip is not None:  # Check if the tip is detected
                                color = get_color(i,bgr=True)
                                cv2.circle(
                                    vis_image,
                                    (int(tip.x), int(tip.y)),
                                    int(tip.r),
                                    color,
                                    1,
                                )                        
                    self.image_pub.publish(self.bridge.cv2_to_imgmsg(vis_image, "bgr8"))
                    rospy.sleep(1.0 / 30.0)
                except CvBridgeError as e:
                    print(e)          
            rospy.sleep(1.0 / 1000.0)

    # def image_callback(self, msg):
    #     try:
    #         current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    #         if self.flag_resize_image:
    #             # Resize the extracted image to speed up detections
    #             new_dims = (current_image.shape[1] // self.resize_scale, current_image.shape[0] // self.resize_scale)
    #             current_image = cv2.resize(current_image, new_dims)
    #         self.latest_image = current_image
    #     except CvBridgeError as e:
    #         print(e)

    def image_callback(self, msg):
        try:
            original_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            if self.flag_crop_image:
                original_image = original_image[self.y0:self.y1,self.x0:self.x1]
            if self.flag_resize_image:
                # Resize the extracted image to speed up detections
                new_dims = (original_image.shape[1] // self.resize_scale, original_image.shape[0] // self.resize_scale)
                self.latest_image = cv2.resize(original_image, new_dims)
            else:
                self.latest_image = original_image

            if self.init == False:
                # Initialization for circle mode
                if self.flag_resize_image:
                    self.resize_scale_x = original_image.shape[1] / new_dims[0]
                    self.resize_scale_y = original_image.shape[0] / new_dims[1]
                self.tooltip = partial(ToolTip, resize_scale_x=self.resize_scale_x, resize_scale_y=self.resize_scale_y, resize_scale_r=self.resize_scale_r,
                x0=self.x0, y0=self.y0, gamma=1.0,
                flag_resize_image=self.flag_resize_image,flag_crop_image=self.flag_crop_image)                    
                self.init = True

        except CvBridgeError as e:
            print(e)

    def masks_callback(self, msg):
        dims = tuple(map(lambda x: x.size, msg.layout.dim))
        self.masks = np.array(msg.data, dtype=float).reshape(dims).astype(np.float32)

if __name__ == "__main__":
    args = parse_args()
    with open(args.camera_param, 'r') as yml:
        camera = yaml.safe_load(yml)    
    rospy.init_node("mask_processor_node", anonymous=False)
    # Initialize MaskProcessor
    mask_processor = MaskProcessor(args.input_topic, camera["resize_scale"], camera["x0_crop"], camera["x1_crop"], camera["y0_crop"], camera["y1_crop"], mask_mode=args.mask_mode, circle_mode=args.circle_mode)

    rospy.spin()