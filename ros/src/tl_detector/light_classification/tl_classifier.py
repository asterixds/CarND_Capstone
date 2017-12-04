from styx_msgs.msg import TrafficLight
from keras.models import load_model
import numpy as np
import cv2
import tensorflow as tf
from utils import non_max_suppression_fast

class TLClassifier(object):
    def __init__(self):
        self.cascade = cv2.CascadeClassifier('./cascade2.xml')
        self.classifier = load_model('./classifier_model_sim_n_real.h5')

    def get_classification(self, image,  img_width=64, img_height = 64):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        lights = self.cascade.detectMultiScale(image, 1.4)
        lights = non_max_suppression_fast(lights, 0.2)
        state = TrafficLight.UNKNOWN
        for (x, y, w, h) in lights:
            border = int(round(h * 0.1))
            slice = image[(y + border):(y + h - border), int(round(x + w / 2)), :]
            #are pixels in neighbourhood similar
            if np.std(slice) > 32:
                tl_img = image[y:(y + h), x:(x + w)]
                tl_img = cv2.resize(tl_img, (img_width, img_height))
                tl_img_data = np.expand_dims(tl_img, axis=0)
                with tf.get_default_graph().as_default():
                    state = self.classifier.predict_classes(tl_img_data, verbose=False)
                if int(state) == TrafficLight.RED:
                    break
                else:
                    continue
        return int(state)

