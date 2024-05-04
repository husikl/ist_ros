import rospy
import yaml
from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import MultiArrayDimension, Float32MultiArray
from std_msgs.msg import Header
import cv2
import time
import threading
from queue import Queue
from std_srvs.srv import Empty
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseArray, Pose
import tkinter as tk
import numpy as np
from configs.get_args import parse_args
from main import build_control, init_interactive_segmentation, inference_masks

class ResourceHandler:
    def __init__(
        self,
        command_queue,
        initial_image,
        res_manager,
        interact_control,
        tk_root,
        masks_queue,
        centers_queue
    ):
        self.command_queue = command_queue
        self.initial_image = initial_image
        self.res_manager = res_manager
        self.interact_control = interact_control

        self.lock = threading.Lock()
        self.image_to_process = None
        self.masks_result = None
        self.center_points = None
        self.tk_root = tk_root
        self.masks_queue = masks_queue
        self.centers_queue = centers_queue
       
    def process_commands(self):
        while True:
            if not self.command_queue.empty():
                command, image = self.command_queue.get()
                if command == "init_segmentation":
                    self.init_segmentation(image)
                elif command == "infer_masks":
                    self.infer_masks(image)
            rospy.sleep(1.0/100.0) # camera fps

    def init_segmentation(self, image):
        with self.lock:
            interact_control.image_queue.put(image)
            init_interactive_segmentation(
                image, res_manager, interact_control, self.tk_root
            )

    def infer_masks(self, image):
        with self.lock:
            self.masks_result, self.center_points = inference_masks(image, res_manager,ros=True) 
            self.masks_queue.put(self.masks_result)
            self.centers_queue.put(self.center_points)

class ImageProcessor:
    def __init__(self, command_queue, masks_queue, centers_queue, input_topic, resize_scale=1.0, x0_crop=None, x1_crop=None, y0_crop=None, y1_crop=None):
        self.command_queue = command_queue
        self.bridge = CvBridge()
        self.latest_image = None
        self.masks_queue = masks_queue
        self.centers_queue = centers_queue
        self.processing_enabled = False
        self.init_segmentation_done = False
        self.np_map_coordinates_to_original = np.frompyfunc(self.map_coordinates_to_original, 3, 3)
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
        self.initial_image_collected = threading.Event()
        self.image_sub = rospy.Subscriber(
            input_topic, ImageMsg, self.image_callback
        )
        self.init_service = rospy.Service(
            "init_segmentation", Empty, self.init_segmentation_service
        )
        self.inference_service = rospy.Service(
            "run_inference", Empty, self.run_inference_service
        )
        # self.detected_targets_pub = rospy.Publisher(
        #     "tracked_targets", PoseArray, queue_size=1
        # )
        self.detected_targets_pub = rospy.Publisher(
            "tracked_targets", Float32MultiArray, queue_size=1
        )        
        self.masks_pub = rospy.Publisher(
            "masks", Float32MultiArray, queue_size=1
        )                  

        self.lock = threading.Lock()  # Initialize the lock here
        self.processing_thread = threading.Thread(target=self.process_images)
        self.processing_thread.start()
        self.scale = 3.0

        rospy.loginfo("init completed...")

    def process_images(self):
        centers_msg = PoseArray()
        header = Header()
        
        while True:
            with self.lock:
                if (self.processing_enabled and self.init_segmentation_done and self.latest_image is not None):
                    self.command_queue.put(("infer_masks", self.latest_image))

                    # Wait for the result to be put into the queue
                    # Publish tool positions
                    # Sort keys to maintain a consistent order
                    center_points = self.centers_queue.get()
                    center_points = self.np_map_coordinates_to_original(center_points[:,0],center_points[:,1],center_points[:,2])
                    center_points = np.array(center_points).T
                    self.detected_targets_pub.publish(self.numpy_converter_centers(center_points))
                    
                    # centers_msg.poses.clear()
                    # for key, value in center_points.items():
                    #     p = Pose()
                    #     if value[2] is True:  # Check if the object was detected
                    #         x, y = value[0], value[1]
                    #         if self.flag_resize_image or self.flag_crop_image:
                    #             x, y = self.map_coordinates_to_original((x, y))
                    #         p.position.x = x
                    #         p.position.y = y
                    #         # p.position.z = value[2]*self.scale
                    #         #  send the detected object id
                    #         p.orientation.x = key
                    #     else:
                    #         # do not need this? 
                    #         p.orientation.x = -key
                    #     centers_msg.poses.append(p)
                    # centers_msg.header.stamp = rospy.Time.from_sec(time.time())
                    # self.detected_targets_pub.publish(centers_msg)

                    # Publish masks
                    masks_result = self.masks_queue.get()
                    self.masks_pub.publish(self.numpy_converter_masks(masks_result))
                    
                    self.latest_image = None  # Clear the latest image

            rospy.sleep(1.0 / 65.0) # camera fps
    
    def map_coordinates_to_original(self, x, y, detected):
        x_original = x * self.resize_scale_x + self.x0
        y_original = y * self.resize_scale_y + self.y0
        return x_original, y_original, detected
        
    def numpy_converter_masks(self, np_masks):
        num_masks, h, w = np_masks.shape
        mask_msg = Float32MultiArray()
        mask_msg.layout.dim = [
            MultiArrayDimension(label="num_masks", size=num_masks, stride=h*w),
            MultiArrayDimension(label="height", size=h, stride=w),
            MultiArrayDimension(label="width", size=w, stride=1)
        ]
        mask_msg.data = np_masks.flatten().tolist()
        return mask_msg

    def numpy_converter_centers(self, np_centers):
        num_obj, c = np_centers.shape
        center_msg = Float32MultiArray()
        center_msg.layout.dim = [
            MultiArrayDimension(label="num_obj", size=num_obj, stride=c),
            MultiArrayDimension(label="center", size=c, stride=1)
        ]
        center_msg.data = np_centers.flatten().tolist()
        return center_msg
    
    def image_callback(self, msg):
        try:
            original_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            if self.flag_crop_image:
                original_image = original_image[self.y0:self.y1,self.x0:self.x1]
            if self.flag_resize_image:
                # Resize the extracted image to speed up detections
                new_dims = (original_image.shape[1] // self.resize_scale, original_image.shape[0] // self.resize_scale)
                self.latest_image = cv2.resize(original_image, new_dims)
            else:
                self.latest_image = original_image

            if not self.initial_image_collected.is_set():
                if self.flag_resize_image:
                    self.resize_scale_x = original_image.shape[1] / new_dims[0]
                    self.resize_scale_y = original_image.shape[0] / new_dims[1]
                self.initial_image = self.latest_image
                self.initial_image_collected.set()
        except CvBridgeError as e:
            print(e) 

    def init_segmentation_service(self, req):
        self.command_queue.put(("init_segmentation", self.latest_image))
        self.init_segmentation_done = True
        return []

    def run_inference_service(self, req):
        if not self.init_segmentation_done:
            print("Initialization not done. Ignoring start request.")
            return []
        else:
            print("Tool tracking start")
        self.processing_enabled = not self.processing_enabled  # Toggle processing
        return []


if __name__ == "__main__":
    args = parse_args()
    with open(args.camera_param, 'r') as yml:
        camera = yaml.safe_load(yml)
    res_manager, interact_control = build_control(args)     
    rospy.init_node("image_processor_node", anonymous=False)

    command_queue = Queue()
    masks_queue = Queue()
    centers_queue = Queue()

    # Initialize ImageProcessor
    image_processor = ImageProcessor(command_queue, masks_queue, centers_queue, args.input_topic, camera["resize_scale"], camera["x0_crop"], camera["x1_crop"], camera["y0_crop"], camera["y1_crop"])

    # Wait until an initial image is collected
    image_processor.initial_image_collected.wait()

    tk_root = tk.Tk()

    # Initialize ResourceHandler with res_manager and interact_control
    resource_handler = ResourceHandler(
        command_queue,
        image_processor.initial_image,
        res_manager,
        interact_control,
        tk_root,
        masks_queue,
        centers_queue
    )

    resource_thread = threading.Thread(target=resource_handler.process_commands)
    resource_thread.start()
    tk_root.mainloop()

    rospy.spin()
