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
class TargetPosition:
    def __init__(self, x, y, id, resize_scale_x, resize_scale_y, x0, y0, flag_resize_image=False, flag_crop_image=False):
        if flag_resize_image or flag_crop_image:
            x, y = self.map_coordinates_to_resized(x,y,resize_scale_x, resize_scale_y, x0, y0)
        self.x, self.y = x, y
        self.id = id

    def map_coordinates_to_resized(self, x_ori, y_ori, resize_scale_x, resize_scale_y, x0, y0):
        x_resized = (x_ori-x0) / resize_scale_x
        y_resized = (y_ori-y0) / resize_scale_y
        return x_resized, y_resized
    
class MaskProcessor:
    def __init__(self, input_topic, resize_scale, x0_crop=None, x1_crop=None, y0_crop=None, y1_crop=None, mask_threshold=220, mask_mode=True, point_mode=False):
        self.mask_mode = mask_mode
        self.point_mode = point_mode
        self.mask_threshold = mask_threshold
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
        self.targetpositon = None                
        self.image_sub = rospy.Subscriber(
            input_topic, ImageMsg, self.image_callback
        )
        self.image_pub = rospy.Publisher("/visualized_image", ImageMsg, queue_size=1)
        if mask_mode:
            self.masks_sub = rospy.Subscriber(
                "masks", Float32MultiArray, self.masks_callback
            )
        if point_mode:
            self.targets = []            
            self.tools_sub = rospy.Subscriber(
                "tracked_targets", Float32MultiArray, self.targets_callback
            )
            # self.multi_tips = defaultdict(deque)
        print(f"Mask mode: {mask_mode}, Point mode: {point_mode}")
        threading.Thread(target=self.visualize_image).start()
        rospy.sleep(1.0)
        rospy.loginfo("Visualizer initialized")

    def targets_callback(self, msg):
        if self.init:
            dims = tuple(map(lambda x: x.size, msg.layout.dim))
            msg_np = np.array(msg.data, dtype=float).reshape(dims).astype(np.float32)
            targets = [None] * msg_np.shape[0]  # Pre-allocate list with None
            for i, p in enumerate(msg_np):
                if bool(p[2]):
                    targets[i] = self.targetpositon(p[0], p[1], i)
                else:
                    targets[i] = None
            # for i, p in enumerate(msg.poses):
            #     if p.orientation.x > 0:  # Tool is detected
            #         target = self.targetpositon(p.position.x, p.position.y, i)
            #         targets[i] = target  # Place the target at the corresponding index

            #     else :
            #         targets[i] = None
            #         # print("not detected i = ", i)
            self.targets = targets  # Directly assign, keeping None for undetected tools

    def visualize_image(self):
        while True:
            if self.init:
                vis_image = self.latest_image.copy()
                try:
                    if self.mask_mode and self.masks is not None:
                        try:
                            vis_image = add_masks_to_image(vis_image, self.masks, threshold=self.mask_threshold)
                        except:
                            raise ValueError("Set the same way to preprocess as the image processor when masking")                            
                    if self.point_mode:
                        for i, target in enumerate(self.targets):
                            if target is not None:  # Check if the tip is detected
                                color = get_color(i,bgr=True)
                                cv2.circle(
                                    vis_image,
                                    (int(target.x), int(target.y)),
                                    int(10),
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
                self.targetpositon = partial(TargetPosition, resize_scale_x=self.resize_scale_x, resize_scale_y=self.resize_scale_y,
                x0=self.x0, y0=self.y0,
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
    mask_processor = MaskProcessor(args.input_topic, camera["resize_scale"], camera["x0_crop"], camera["x1_crop"], camera["y0_crop"], camera["y1_crop"],mask_threshold=args.mask_threshold, mask_mode=args.mask_mode, point_mode=args.point_mode)

    rospy.spin()