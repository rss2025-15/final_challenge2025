import cv2
import pyzed.sl as sl

import time
class zed_source:

    def __init__(self, set_fps, resize_dim=None, video_path=None, verbose=False):
        """
        Creates a wrapper around the zed camera to initialize, grab frame, and shutdown
        
        set_fps: int 15, 30, or 60 ONLY
        resize_dim: tuple (width, height). Maybe (640, 360) to match existing code?? None if you don't want to resize
        video_path: string for where to save video. No video saved if None
        verbose: bool to control if we print fps/latency
        """
        # Create a Camera object
        self.zed = sl.Camera()

        # Create a InitParameters object and set configuration parameters
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.VGA # Use HD720 if you want really high res, slows everything down though
        init_params.depth_mode = sl.DEPTH_MODE.NONE
        init_params.camera_fps = set_fps # Set fps at 15, 30, or 60 ONLY
        
        # Open the camera
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            print("Camera Open : "+repr(err)+". Exit program.")
            exit()

        self.last_ts = 0
        self.image = sl.Mat()
        self.runtime_parameters = sl.RuntimeParameters()

        self.verbose = verbose
        self.resize_dim = resize_dim
        if self.resize_dim is not None:
            video_dim = self.resize_dim
        else:
            video_dim = (672, 376)
        if video_path is not None:
            self.video=cv2.VideoWriter(video_path,cv2.VideoWriter_fourcc(*'X264'), set_fps , video_dim) # (672, 376) corresponds to VGA mode
        else:
            self.video=None

    def get_frame(self):
        success=False
        while not success:
            # Grab an image, a RuntimeParameters object must be given to grab()
            if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                # A new image is available if grab() returns SUCCESS
                self.zed.retrieve_image(self.image, sl.VIEW.LEFT)  #TODO can change left/right as we desire??? I don't really know what's going on now
                timestamp = self.zed.get_timestamp(sl.TIME_REFERENCE.CURRENT).get_milliseconds()/1000  # Get the timestamp at the time the image was captured
                frame=self.image.get_data()[:, :, :3]
                if self.resize_dim is not None:
                    frame = cv2.resize(frame, self.resize_dim)
                    # print(frame.shape)
                
                if self.video is not None:
                    self.video.write(frame)

                
                # display video if you really wanna?
                # cv2.imshow("frame", image.get_data())
                # if cv2.waitKey(30) >= 0 :
                #     break
                success=True

                if self.verbose:
                    # print("Image resolution: {0} x {1} || Image timestamp: {2}\n".format(image.get_width(), image.get_height(),
                    #     timestamp.get_milliseconds()))
                    print('fps: ', 1/(timestamp-self.last_ts))
                    print('latency (seconds): ', time.time()-timestamp)
                self.last_ts=timestamp
        return frame
    
    def shutdown_zed(self):
        self.video.release()
        self.zed.close()
        cv2.destroyAllWindows()
        print('zed shutdown and video saved')

if __name__ == "__main__":
    pass