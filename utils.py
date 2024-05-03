import cv2
import numpy as np
import matplotlib.pyplot as plt

def get_color(index, bgr=False):
    cmap = plt.get_cmap('tab10')
    color = cmap(index % cmap.N)
    color = [int(c * 255) for c in color[:3]]
    if bgr:
        color.reverse()
    return color

def add_masks_to_image(image, masks, bgr=True, threshold=100):
    colored_mask = np.zeros_like(image)
    for i, mask_image in enumerate(masks):
        color = np.array(get_color(i,bgr=bgr), dtype=np.uint8)
        colored_mask = np.where(mask_image[..., None]>threshold, color, colored_mask)
    masked_image = cv2.addWeighted(image, 0.8, colored_mask, 0.5, 0)
    return masked_image