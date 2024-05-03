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
![labels](results of object detction/labels.jpg)

## Results after training Yolo
![confusion matrix](results of object detction/confusion_matrix_normalized.png)
![confusion matrix](results of object detction/F1_curve.png)
![confusion matrix](results of object detction/results.png)

## Testing model on validation data
![test results](results of object detction/val_batch2_pred.jpg)

## Video Demo of execution

- To see path planning and object detection in action [Watch the Youtube Video](https://youtu.be/UlIQnAkgNVc).

## Copyright and Usage

This project is the intellectual property of [California State University, East Bay]. Unauthorized copying, distribution, or use of the code, data, or related materials without explicit permission is prohibited.

Please contact me at [adwivedi@horizon.csueastbay.edu] for any requests or permissions related to this project.

## Acknowledgments

- We would like to thank the developers of YOLO-Ultralytics for providing an excellent object detection algorithm.
- Thanks to the open-source community for their contributions and resources.

