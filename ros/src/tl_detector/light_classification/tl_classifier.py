from styx_msgs.msg import TrafficLight
import numpy as np
import tensorflow as tf
import os
import rospy
import cv2
from PIL import Image

class TLClassifier(object):
    def __init__(self, subsample_rate, confidence_level):
        #TODO load classifier
        
        """Subsampling for inference delay"""
        self.skip_frames = subsample_rate
        self.frames_count = 1
        self.save_box_count = 1
        
        """Loads a frozen inference graph"""
        self.confidence_cutoff = confidence_level
        graph_file = os.path.join(os.getcwd(),'light_classification/model/frozen_inference_graph_mn.pb')
        self.tl_graph = tf.Graph()
        
        with self.tl_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(graph_file, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

            config = tf.ConfigProto()
            config.gpu_options.allow_growth = True
            self.tl_sess = tf.Session(graph=self.tl_graph, config=config)
        
            self.image_tensor = self.tl_graph.get_tensor_by_name('image_tensor:0')
            # Each box represents a part of the image where a particular object was detected.
            self.detection_boxes = self.tl_graph.get_tensor_by_name('detection_boxes:0')
            # Each score represent how level of confidence for each of the objects.
            # Score is shown on the result image, together with the class label.
            self.detection_scores = self.tl_graph.get_tensor_by_name('detection_scores:0')
            # The classification of the object (integer id).
            self.detection_classes = self.tl_graph.get_tensor_by_name('detection_classes:0')


    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        #Subsampling for inference delay
        if self.frames_count % self.skip_frames != 1:
            self.frames_count += 1
            return TrafficLight.SKIP
        else:
            self.frames_count += 1 

        
        #Inference for image to classify traffic lights
        #image = Image.fromarray(image)
        cv2_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        with self.tl_graph.as_default():   
            image_np = np.expand_dims(np.asarray(cv2_image, dtype=np.uint8), 0)
            
            # Actual detection.
            (boxes, scores, classes) = self.tl_sess.run(
                                       [self.detection_boxes, self.detection_scores, self.detection_classes], 
                                       feed_dict={self.image_tensor: image_np})
            
            # Remove unnecessary dimensions
            boxes, scores, classes = np.squeeze(boxes), np.squeeze(scores), np.squeeze(classes)

            # Filter boxes with a confidence score less than `confidence_cutoff`
            boxes, scores, classes = self.filter_boxes(self.confidence_cutoff, boxes, scores, classes)
            
            #width, height = image.size
            width, height = image.shape[1], image.shape[0]
            
            # boxes are normalized coordinates(0-1). Convert to image coordinates
            box_coords = self.to_image_coords(boxes, height, width)
            
            # mobilenet coco classify a traffic light box into 10. pull the most likely.
            tl_boxes = []
            zipped_items = sorted(zip(classes,scores,box_coords), key=lambda x:x[1], reverse=True)

            for c, s, b in zipped_items:
                if s > self.confidence_cutoff and c == 10.:
                    #rospy.loginfo("[TL Detector] Detected TL | score: " + str(scores))

                    # skip if box is too small or inproportional
                    box_height, box_width = (b[2] - b[0]), (b[3] - b[1])
                    if(box_height < 15) or (box_width < 15) or (box_height/box_width < 1.5):
                        continue
 
                    tl_boxes.append(b)
                    break # Return just one highest confidence box
            
            # Crop light color by looking at only the top portion of box (red light)
            cropped_tls = []
            #image_as_np_array = np.asarray(image, dtype="uint8")
            for box in tl_boxes:
                #rospy.loginfo("[TL Detector] Detected box: " +str(box))
                cropped_tls.append(
                        # Do not use PIL for crop. use cv2 functions tha takes np.ndarray type as input
                        cv2.resize(cv2_image[int(box[0]+10) : int(box[0]+(box[2]-box[0])/2.5), 
                                             int(box[1]+5 ) : int(box[3]-5)], (32, 32)))
                
            for cropped_tl in cropped_tls:
                
                #cv2.imwrite("./images/"+str(self.save_box_count)+'.jpeg', cropped_tl)
                self.save_box_count += 1

                # Detect number of red pixels
                cropped_image_hsv = cv2.cvtColor(cropped_tl, cv2.COLOR_RGB2HSV)

                v_channel = cropped_image_hsv[:, :, 2]
                brightness = np.sum(v_channel)
                #rospy.loginfo("[TL Detector] Red TL brightness: " + str(brightness))
                
                # in simulator, RED light is usually greater than 50000
                if int(brightness) > 50000: 
                    rospy.loginfo("[TL Detector] Detected RED light"+ str(brightness))
                    return TrafficLight.RED
                else:
                  rospy.loginfo("[TL Detector] Detected GREEN light"+ str(brightness))
                  return TrafficLight.GREEN

            return TrafficLight.UNKNOWN
        
        
        
    def filter_boxes(self, min_score, boxes, scores, classes):
        """Return boxes with a confidence >= `min_score`"""
        n = len(classes)
        idxs = []
        for i in range(n):
            if scores[i] >= min_score:
                idxs.append(i)

        filtered_boxes = boxes[idxs, ...]
        filtered_scores = scores[idxs, ...]
        filtered_classes = classes[idxs, ...]
        return filtered_boxes, filtered_scores, filtered_classes
    
    
    def to_image_coords(self, boxes, height, width):
        """
        The original box coordinate output is normalized, i.e [0, 1].

        This converts it back to the original coordinate based on the image
        size.
        """
        box_coords = np.zeros_like(boxes)
        box_coords[:, 0] = boxes[:, 0] * height
        box_coords[:, 1] = boxes[:, 1] * width
        box_coords[:, 2] = boxes[:, 2] * height
        box_coords[:, 3] = boxes[:, 3] * width 
        
        return box_coords
    
     