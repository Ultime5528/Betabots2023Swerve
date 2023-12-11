import robotpy_apriltag
from cscore import CameraServer
import cv2
import numpy as np
from robotpy_apriltag import AprilTagDetector, AprilTagPoseEstimator

from utils.property import autoproperty
from ntcore import NetworkTable, NetworkTableInstance


def get_apriltag_detector_and_estimator(frame_size):
    detector = AprilTagDetector()
    detector.addFamily("tag36h11", 0)
    estimator = AprilTagPoseEstimator(
        AprilTagPoseEstimator.Config(
            0.2, 500, 500, frame_size[1] / 2.0, frame_size[0] / 2.0
        )
    )
    return detector, estimator


def draw_tag(frame, result, color: tuple[int, int, int]):
    assert frame is not None
    assert result is not None
    tag_id, pose, center = result
    cv2.circle(frame, (int(center.x), int(center.y)), 50, color, 3)
    msg = f"Tag ID: {tag_id} Pose: {pose}"
    cv2.putText(
        frame, msg, (100, 50 * 1), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2
    )
    return frame


def detect_and_process_apriltag(frame, detector, estimator):
    assert frame is not None
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Detect apriltag
    tag_info = detector.detect(gray)
    DETECTION_MARGIN_THRESHOLD = 100
    filter_tags = [
        tag for tag in tag_info if tag.getDecisionMargin() > DETECTION_MARGIN_THRESHOLD
    ]
    results = [process_apriltag(estimator, tag) for tag in filter_tags]
    # Note that results will be empty if no apriltag is detected
    for result in results:
        frame = draw_tag(frame, result)
    return frame


def process_apriltag(estimator: AprilTagPoseEstimator, tag: robotpy_apriltag.AprilTag):
    tag_id = tag.getId()
    center = tag.getCenter()
    hamming = tag.getHamming()
    decision_margin = tag.getDecisionMargin()

    est = estimator.estimateOrthogonalIteration(tag, 50)
    return tag_id, est.pose1, center


def main():
    instance = NetworkTableInstance.getDefault()
    vision_table = instance.getTable("vision")
    tag_x = vision_table.getDoubleTopic("tag_x").publish()
    tag_y = vision_table.getDoubleTopic("tag_y").publish()
    tag_rot = vision_table.getDoubleTopic("tag_rot").publish()

    min_detection_threshold = autoproperty(100)
    CameraServer.enableLogging()

    # Capture from the first USB Camera on the system
    resolution = (1080, 1920)
    camera = CameraServer.startAutomaticCapture()
    camera.setResolution(*resolution)

        
    # Get a CvSink. This will capture images from the camera
    cv_sink = CameraServer.getVideo()
    output_stream = CameraServer.putVideo("Video", *resolution)

    detector, estimator = get_apriltag_detector_and_estimator(resolution)

    # Allocating new images is very expensive, always try to preallocate
    img = np.zeros(shape=(*resolution, 3), dtype=np.uint8)

    while True:
        # Tell the CvSink to grab a frame from the camera and put it
        # in the source image.  If there is an error notify the output.
        time, img = cv_sink.grabFrame(img)
        if time == 0:
            # Send the output the error.
            output_stream.notifyError(cv_sink.getError())
            # skip the rest of the current iteration
            continue

        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        tag_info = detector.detect(gray_img)
        filtered_tags = [
            tag for tag in tag_info if tag.getDecisionMargin() > min_detection_threshold
        ]
        results = [process_apriltag(estimator, tag) for tag in filtered_tags]

        if results:
            most_centered = min(results, key=lambda x: x[2].x)

            img = draw_tag(img, most_centered, (0, 255, 0))
            for result in results:
                img = draw_tag(img, result, (0,0,255))

            tag_x.set(most_centered[2].x)
            tag_y.set(most_centered[2].y)
            tag_rot.set(most_centered[1].rotation())
        output_stream.putFrame(img)

if __name__ == '__main__':
    main()
