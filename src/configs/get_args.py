import argparse

def parse_args():
    parser = argparse.ArgumentParser()

    # version without ROS
    parser.add_argument('--input_path', type=str, default='./data/inputs/test_video2.mp4',
                        help='Path to the input video')
    parser.add_argument('--output_path', type=str, default='./data/outputs/videos/out_test_video2.mp4',
                        help='Path to the output video file')
    parser.add_argument('--mask_threshold', type=int, default=220,
                        help='Threshold value for applying the mask (0-255). Pixels with values greater than this threshold will be masked.')
    # version with ROS
    parser.add_argument('--input_topic', type=str, default='/ximea_cam/image_raw',
                        help='Topic name to subscribe to')
    parser.add_argument('--camera_param', type=str, default='./src/configs/camera_env.yaml',
                        help='Path to yaml file to specify resize scale and crop parameters')

    sam_group = parser.add_argument_group('SAM Model Arguments')
    sam_group.add_argument('--sam.encoder_type', type=str, default='vit_h',
                           choices=['vit_h', 'vit_l', 'vit_b'],
                           help='Encoder type for SAM model')
    sam_group.add_argument('--sam.checkpoint', type=str, default='./data/weights/sam_vit_h_4b8939.pth',
                           help='Path to the checkpoint file for SAM model')

    xmem_group = parser.add_argument_group('XMem Model Arguments')
    xmem_group.add_argument('--xmem.checkpoint', type=str, default='./data/weights/XMem.pth',
                            help='Path to the checkpoint file for XMem model')
    xmem_group.add_argument('--xmem.buffer_size', type=int, default=50,
                            help='Correlate with CPU memory consumption')
    xmem_group.add_argument('--xmem.num_objects', type=int, default=1,
                            help='Number of objects')
    xmem_group.add_argument('--xmem.max_mid_term_frames', type=int, default=10,
                            help='T_max in paper, decrease to save memory')
    xmem_group.add_argument('--xmem.min_mid_term_frames', type=int, default=5,
                            help='T_min in paper, decrease to save memory')
    xmem_group.add_argument('--xmem.max_long_term_elements', type=int, default=100000,
                            help='LT_max in paper, increase if objects disappear for a long time')
    xmem_group.add_argument('--xmem.num_prototypes', type=int, default=128,
                            help='P in paper')
    xmem_group.add_argument('--xmem.top_k', type=int, default=30)
    xmem_group.add_argument('--xmem.mem_every', type=int, default=10)
    xmem_group.add_argument('--xmem.deep_update_every', type=int, default=-1,
                            help='Leave -1 normally to synchronize with mem_every')
    xmem_group.add_argument('--xmem.no_amp', action='store_true',
                            help='Turn off AMP')
    xmem_group.add_argument('--xmem.size', type=int, default=-1,
                            help='Resize the shorter side to this size. -1 to use original resolution. 480 for default')
    xmem_group.add_argument('--xmem.diable_long_term', action='store_false',dest='xmem.enable_long_term')
    xmem_group.add_argument('--xmem.diable_long_term_count_usage', action='store_false',dest='xmem.enable_long_term_count_usage')
    visualization_group = parser.add_argument_group('Visualization')
    visualization_group.add_argument('--point_mode', action='store_true')
    visualization_group.add_argument('--disable_mask_mode', action='store_false',
                                  dest='mask_mode')
    visualization_group.add_argument('--center_radius',type=int,default=5,
                                  help='Radius of circle to attach to targets')
    visualization_group.add_argument('--save_all_mode', action='store_true')
    visualization_group.add_argument('--save_all_path', type=str, default='./data/outputs/data/out_test_video2')
    return parser.parse_args()

def get_group_args(args, group_name):
    args = vars(args)
    group_args = dict()
    for key, value in args.items():
        if key.startswith(f'{group_name}.'):
            new_key = key.replace(f'{group_name}.','')
            group_args[new_key] = value
    return group_args
