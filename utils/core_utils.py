import sys
import torch
from PIL import Image
import numpy as np
import cv2

from models.XMem.definition import XMem
from models.XMem.inference.inference_core import InferenceCore
from segment_anything import SamPredictor, sam_model_registry

from configs.get_args import get_group_args

# ! load interaction control
from control.interaction2 import interaction_control

# ! load resource manager
from control.res_manager import ResManager as resource_manager
import math

def get_mask_center(mask):
    n_obj = mask.shape[0]
    # create a dictionory to store the 2d pixel position of the mask center and detected true/false variable
    mask_cal_idv = []
    for idx in range(n_obj):
        single_mask = mask[idx]
        
        _, thresh = cv2.threshold(single_mask, 0.75, 1, cv2.THRESH_BINARY)

        # Convert to a compatible type if necessary
        if thresh.dtype != np.uint8:
            thresh = (thresh * 255).astype(np.uint8)

        # Apply morphological operations
        kernel = np.ones((3,3), np.uint8)  # Kernel size can be adjusted
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)  # Erosion followed by dilation (removes noise)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)  # Dilation followed by erosion (closes small holes)

        # Calculate mass center
        M = cv2.moments(thresh)
        m00 = M["m00"]
        
        if m00 != 0:
            m10, m01 = M["m10"], M["m01"]
            cX, cY = int(m10 / m00), int(m01 / m00)
            # assign value to the dictionary the pixel position and detected = True
            mask_cal_idv.append([cX, cY, True])
        else:
            # assign value to the dictionary the pixel position and detected = False
            mask_cal_idv.append([0, 0, False])
    mask_cal_idv = np.array(mask_cal_idv,dtype=np.float32)
    return mask_cal_idv

def build_control(args):
    XMem_config = get_group_args(args,'xmem')
    network = XMem(XMem_config, XMem_config["checkpoint"]).cuda().eval()
    infer_control = InferenceCore(network=network, config=XMem_config)
    res_manager = resource_manager(infer_control)
    sam_config = get_group_args(args,'sam')
    sam = SamPredictor(sam_model_registry[sam_config['encoder_type']](sam_config['checkpoint']).cuda())
    interact_control = interaction_control(sam)
    return res_manager, interact_control


def init_interactive_segmentation(image, res_manager, interact_control, tk_root=None, ros=False, debug=False):
    if debug:
        assert isinstance(image, np.ndarray)
        assert image.shape[2] == 3
        assert len(image.shape) == 3

    masks = interact_control.do_interact(Image.fromarray(image), tk_root,ros=ros)
    if debug:
        print("got masks from sam...")

    res_manager.set_image(image)
    res_manager.set_mask(masks)
    masks = res_manager.step(mask=True)
    
    return masks

def inference_masks(image, res_manager, debug=False, save_mode=False):
    if debug:
        assert isinstance(image, np.ndarray)
        assert image.shape[2] == 3
        assert len(image.shape) == 3
    
    res_manager.set_image(image)
    masks = res_manager.step(mask=None)

    # Probability mask -> index mask
    out_mask = torch.max(masks, dim=0).indices
    out_mask = (out_mask.detach().cpu().numpy()).astype(np.uint8)
    return out_mask, None