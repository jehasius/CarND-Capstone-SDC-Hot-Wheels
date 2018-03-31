from styx_msgs.msg import TrafficLight
import cv2
from keras.models import load_model
from numpy import newaxis
import rospkg
import numpy as np
import tensorflow as tf 


class TLClassifier(object):
    def __init__(self):

        r = rospkg.RosPack()
        path = r.get_path('tl_detector')
        
        self.model = load_model(path + '/light_classifier_model.h5')

        self.model._make_predict_function()
        self.graph = tf.get_default_graph()
        
        # print(model)
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        imrs = cv2.resize(image, (64, 64)) 
        imrs = imrs.astype(float)
        imrs = imrs / 255.0
        
        imrs = imrs[newaxis, :, :, :]

        with self.graph.as_default():
            preds = self.model.predict(imrs)
        
        predicted_class = np.argmax(preds, axis=1)

        choices = {0: TrafficLight.RED,
                   1: TrafficLight.YELLOW,
                   2: TrafficLight.GREEN}
        return choices.get(predicted_class[0], TrafficLight.GREEN)
