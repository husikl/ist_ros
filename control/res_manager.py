from PIL import Image
import numpy as np
import torch


# ! funcs for preprocessing image
import torchvision.transforms as transforms

im_mean = (124, 116, 104)

im_normalization = transforms.Normalize(
                    mean=[0.485, 0.456, 0.406],
                    std=[0.229, 0.224, 0.225]
                )

inv_im_trans = transforms.Normalize(
                mean=[-0.485/0.229, -0.456/0.224, -0.406/0.225],
                std=[1/0.229, 1/0.224, 1/0.225])
        
class ResManager:
    def __init__(self, processor, debug=False):
        self.current_image = None
        self.current_mask = None
        self.processor = processor
        self.num_obj = None
        self.debug = debug

    def image_to_torch(self, frame: np.ndarray, device='cuda'):
        frame = frame.transpose(2, 0, 1)
        frame = torch.from_numpy(frame).float().to(device)/255
        frame_norm = im_normalization(frame)
        return frame_norm, frame
    
    def set_image(self, image):
        self.current_image_torch, self.current_image_torch_no_norm = self.image_to_torch(image)
        
    def set_mask(self, mask):
        self.current_prob = mask[:,0].float()
        if self.num_obj is None:
            self.num_obj = mask.shape[0]
        else:
            if self.debug:
                assert self.num_obj == mask.shape[0], 'new obj?'
        self.processor.set_all_labels(list(range(1, self.num_obj+1)))
    
    @torch.no_grad()
    def step(self, mask=None):
        if not mask is None:
            self.current_prob = self.processor.step(self.current_image_torch, self.current_prob)
        else:
            self.current_prob = self.processor.step(self.current_image_torch)
        return self.current_prob
        
    