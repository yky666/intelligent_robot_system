"""Color threshold shared by pickup, holding checks and image guard."""
def threshold(image, config):
    import cv2
    import numpy as np
    space = config.get('color_space', 'LAB').lower()
    pairs = [(config['lower_' + space], config['upper_' + space])]
    if space == 'hsv':
        pairs += [(item['lower'], item['upper']) for item in config.get('extra_hsv_ranges', [])]
    mask = np.zeros(image.shape[:2], dtype=np.uint8)
    for lower, upper in pairs:
        if len(lower) != 3 or len(upper) != 3 or any(a > b for a, b in zip(lower, upper)):
            raise ValueError('Invalid color interval')
        mask |= cv2.inRange(image, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8))
    return mask
