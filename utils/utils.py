import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib

color_order = [0,1,3,4,6,7,8,9,2,5]
def get_color(index, bgr=False, cmap_name='tab10', color_order=None):
    """
    To change color map or color order, set cmap_name and color_order
    """
    cmap = plt.get_cmap(cmap_name)
    colors = cmap.colors
    
    if color_order is not None:
        colors = [colors[i] for i in color_order]
        cmap = matplotlib.colors.ListedColormap(colors)
    
    color = cmap(index % cmap.N)
    color = [int(c * 255) for c in color[:3]]
    if bgr:
        color.reverse()
    return color

def add_masks_to_image(image, masks, bgr=True, threshold=100):
    """
    Add all masks to image
    """
    colored_mask = np.zeros_like(image)
    for i, mask_image in enumerate(masks):
        color = np.array(get_color(i,bgr=bgr), dtype=np.uint8)
        colored_mask = np.where(mask_image[..., None]>threshold, color, colored_mask)
    masked_image = cv2.addWeighted(image, 0.8, colored_mask, 0.5, 0)
    return masked_image

def add_masks_to_image_resize(image, masks, x0, x1, y0, y1, resize_scale, flag_crop_image, flag_resize_image, bgr=True, threshold=100):
    """
    Add all masks to image while cropping and resizing
    """
    colored_mask = np.zeros_like(image)
    for i, mask_image in enumerate(masks):
        if flag_crop_image or flag_resize_image:
            mask_image = crop_resize(mask_image, x0=x0, x1=x1, y0=y0, y1=y1,resize_scale=resize_scale, flag_crop_image=flag_crop_image, flag_resize_image=flag_resize_image)
        color = np.array(get_color(i,bgr=bgr), dtype=np.uint8)
        colored_mask = np.where(mask_image[..., None]>threshold, color, colored_mask)
    masked_image = cv2.addWeighted(image, 0.8, colored_mask, 0.5, 0)
    return masked_image

def crop_resize(original_image, x0, x1, y0, y1, resize_scale, flag_crop_image, flag_resize_image):
    """
    Crop and Resize image
    """
    if flag_crop_image:
        original_image = original_image[y0:y1,x0:x1]
    if flag_resize_image:
        # Resize the extracted image to speed up detections
        new_dims = (original_image.shape[1] // resize_scale, original_image.shape[0] // resize_scale)
        original_image = cv2.resize(original_image, new_dims)
    return original_image