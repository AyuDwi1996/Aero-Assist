# Aero-Assist

This project focuses on implementing path planning and obstacle avoidance system to help people with limited mobility. It involves data annotation, object detection using YOLO, and Dijkstra's path planning algorithm implementation.

## Features

- **Data Annotation**: Annotation of data for use in training object detection models using Roboflow.
- **Object Detection**: Implementation of object detection using the YOLO algorithm is done in Google Colab.
- **Path Planning**: Implementation of Dijkstra's algorithm for optimal path planning.
- **Obstacle Avoidance**: Integration of object detection with path planning for real-time obstacle avoidance in python script.

## Data

- Data files can be found in this location [Roboflow](https://universe.roboflow.com/custom-object-detection-vvwt6/accessibility-object-detection/).

## Distribution of Data Classes
<img align="right" alt="labels" src="https://github.com/AyuDwi1996/Aero-Assist/blob/main/results%20of%20object%20detction/labels.jpg" width="700" height="500" />

## Results after training Yolo
<img align="right" alt="confusion matrix" src="https://github.com/AyuDwi1996/Aero-Assist/blob/main/results%20of%20object%20detction/confusion_matrix_normalized.png" width="700" height="500" />
<img align="right" alt="f1 curve" src="https://github.com/AyuDwi1996/Aero-Assist/blob/main/results%20of%20object%20detction/F1_curve.png" width="700" height="500" />
<img align="right" alt="results" src="https://github.com/AyuDwi1996/Aero-Assist/blob/main/results%20of%20object%20detction/results.png" width="700" height="500" />

## Testing model on validation data
<img align="right" alt="results" src="https://github.com/AyuDwi1996/Aero-Assist/blob/main/results%20of%20object%20detction/val_batch2_pred.jpg" width="700" height="500" />

## Video Demo of execution

- To see path planning and object detection in action [Watch the Youtube Video](https://youtu.be/UlIQnAkgNVc).

## Copyright and Usage

This project is the intellectual property of [California State University, East Bay]. Unauthorized copying, distribution, or use of the code, data, or related materials without explicit permission is prohibited.

Please contact me at [adwivedi@horizon.csueastbay.edu] for any requests or permissions related to this project.

## Acknowledgments

- We would like to thank the developers of YOLO-Ultralytics for providing an excellent object detection algorithm.
- Thanks to the open-source community for their contributions and resources.

