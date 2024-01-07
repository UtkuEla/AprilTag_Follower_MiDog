import yaml
import numpy as np
from ApriltagDetector import ApriltagDetector

def load_camera_params():
    params = {}
    with open(r'calibrationResults.yaml') as file:
        documents = yaml.safe_load(file)
        for item, doc in documents.items():
            params[item] = doc
        
    return params


if __name__ == "__main__":
    params = load_camera_params()
    K, distortion = np.array(params["camMatrix"]),np.array(params["dist"])
    apriltag_detector = ApriltagDetector(K)
    apriltag_detector.distance_to_camera()

