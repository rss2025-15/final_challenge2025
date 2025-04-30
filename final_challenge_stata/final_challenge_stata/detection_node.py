import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from zed_source import zed_source
from sensor_msgs.msg import Image
from detector import Detector
from traffic_light import color_detect
class DetectorNode(Node):
    def __init__(self):
        super().__init__("detector")
        self.detector = Detector()

        self.banana_publisher = None #TODO
        self.traffic_light_publisher = None # Todo
        self.zed = zed_source(set_fps=30, resize_dim=(640, 360)) 

        self.get_logger().info("Detector Initialized")
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
            
            self.prev_time=cur_time



def main(args=None):
    rclpy.init(args=args)
    detector = DetectorNode()
    detector.detect_loop()
    rclpy.spin(detector)
    rclpy.shutdown()

if __name__=="__main__":
    main()
