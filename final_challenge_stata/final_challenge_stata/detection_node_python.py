
from zed_source import zed_source
import cv2
from detector import Detector
from traffic_light import color_detect
import numpy as np
import time
class DetectorNode:
    def __init__(self):
        self.detector = Detector()
        self.zed = zed_source(set_fps=30, resize_dim=(640, 360)) 
        self.prev_time = 0
    def detect_loop(self):
        # Process image with CV Bridge
        signal='?'
        while True:
            cur_time = time.time()
            img = self.zed.get_frame()
            results = self.detector.predict(img)

            predictions = results["predictions"]

            for prediction in predictions:
                if prediction[1]=='banana':
                    #do shrink ray gun sequence
                    print('banananana')
                
                elif prediction[1]=='traffic light':
                    # get bounding box
                    x1, y1, x2, y2 = prediction[0] 
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                    cropped_img = img[y1:y2+1, x1:x2+1]
                    signal = color_detect(cropped_img)
            out=np.array(self.detector.draw_box(img, predictions, draw_all=True))
            fps = 1/(cur_time-self.prev_time)
            out=cv2.cvtColor(out, cv2.COLOR_RGB2BGR)
            cv2.putText(out, "FPS: " + str(round(fps, 2))+signal, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow('frame', out)
            # print(fps)
            self.prev_time=cur_time
            if cv2.waitKey(1) == ord('q'):
                break


my_detector = DetectorNode()
my_detector.threshold=0.2
my_detector.detect_loop()