from .detector import Detector
from PIL import Image, ImageDraw, ImageFont

# detector = Detector()

    
# img = Image.open(img_path)
# results = model.predict(img)

# predictions = results["predictions"]
# original_image = results["original_image"]
    
# out = model.draw_box(original_image, predictions, draw_all=True)



img_path = f"{os.path.dirname(__file__)}/../../media/red.jpg" 
# my fun constants
color_thresh = {'red': {'lower': np.array([2, 120, 100], dtype=np.uint8), 'upper': np.array([10,255,255], dtype=np.uint8)},
                'yellow': {'lower': np.array([2, 120, 100], dtype=np.uint8), 'upper': np.array([10,255,255], dtype=np.uint8)},
                'green': {'lower': np.array([2, 120, 100], dtype=np.uint8), 'upper': np.array([10,255,255], dtype=np.uint8)},
                }
color_idx = {0: 'red', 1: 'yellow', 2: 'green'}
# we get predictions out
# go through them and get the traffic light one

# if traffic light, then we do color segmentation by the three different ranges

color_area = [-1,-1,-1]

hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
for i, color in enumerate(['red', 'yellow', 'green']):
    mask = cv2.inRange(hsv_img, color_thresh[color]['lower'], color_thresh[color]['upper'])
    mask.save
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    #bounding_box = ((0, 0), (0, 0))  # Default if no object found
    largest_area = 0
    best_contour = None

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > largest_area:
            largest_area = area
            best_contour = cnt

    if best_contour is not None:
        color_area[i]=largest_area

signal = color_idx[color_area.index(max(color_area))]
print(signal)

save_path = f"{os.path.dirname(__file__)}/trafficlight_output.png"
print(f"Saved demo to {save_path}!")




# we pick the one with the biggest blob and call that the signal
# red+yellow means stop
# green means go 

