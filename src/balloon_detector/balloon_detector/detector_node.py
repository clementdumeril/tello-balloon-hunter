
#!/usr/bin/env python3
"""ROS 2 balloon detector: subscribes to a camera image topic, runs YOLO on
each frame, and publishes the detections.
 
Subscribes:
    /image_raw            (sensor_msgs/Image)          camera stream (usb_cam)
 
Publishes:
    /detections           (vision_msgs/Detection2DArray)  boxes in pixel coords
    /detections_image     (sensor_msgs/Image)             frame with boxes drawn
                                                          (only computed while
                                                          something subscribes)
 
Parameters:
    model_path         (str,   'yolo11n.pt')  path to YOLO weights (.pt)
    conf               (float, 0.4)           confidence threshold
    imgsz              (int,   320)           inference size; 320 matches a
                                              320x240 camera stream and is
                                              faster on CPU
    device             (str,   '')            '' = auto, 'cpu', or '0' for GPU
    classes            (int[], [-1])          class IDs to keep; [-1] = all.
                                              With the stock COCO model,
                                              [32] keeps only "sports ball"
    image_topic        (str,   '/image_raw')
    publish_annotated  (bool,  True)
 
Run (ROS sourced, yolo_env site-packages on PYTHONPATH):
    python3 detector_node.py --ros-args -p model_path:=/path/to/best.pt
"""
 
import time
 
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from ultralytics import YOLO
from vision_msgs.msg import (Detection2D, Detection2DArray,
                             ObjectHypothesisWithPose)
 
 
def bgr_to_imgmsg(frame, header):
    """Build a sensor_msgs/Image (bgr8) directly from a numpy array.
 
    Used instead of CvBridge.cv2_to_imgmsg, which raised KeyError: 16 here
    (a mismatch between cv_bridge and the OpenCV build that Python imported).
    """
    frame = np.ascontiguousarray(frame, dtype=np.uint8)
    msg = Image()
    msg.header = header
    msg.height, msg.width = frame.shape[:2]
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = frame.tobytes()
    return msg
 
 
class BalloonDetector(Node):
    def __init__(self):
        super().__init__('balloon_detector')
 
        # ---- parameters ----
        self.declare_parameter('model_path', 'tools/yolo11n.pt')
        self.declare_parameter('conf', 0.4)
        self.declare_parameter('imgsz', 320)
        self.declare_parameter('device', '')
        self.declare_parameter('classes', [-1])
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('publish_annotated', True)
 
        model_path = self.get_parameter('model_path').value
        self.conf = float(self.get_parameter('conf').value)
        self.imgsz = int(self.get_parameter('imgsz').value)
        self.device = self.get_parameter('device').value
        classes = list(self.get_parameter('classes').value)
        self.classes = [c for c in classes if c >= 0] or None
        image_topic = self.get_parameter('image_topic').value
        self.publish_annotated = bool(self.get_parameter('publish_annotated').value)
 
        # ---- model ----
        self.get_logger().info(f'Loading model: {model_path}')
        self.model = YOLO(model_path)
        self.get_logger().info(f'Model classes: {self.model.names}')
        self._predict(np.zeros((240, 320, 3), dtype=np.uint8))  # warm-up
 
        # ---- ROS interfaces ----
        self.bridge = CvBridge()
 
        # Best effort + depth 1: always work on the newest frame and never
        # build up a backlog if inference is slower than the camera.
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.sub = self.create_subscription(Image, image_topic, self.on_image, qos)
        self.det_pub = self.create_publisher(Detection2DArray, 'detections', 10)
        self.img_pub = self.create_publisher(Image, 'detections_image', 1)
 
        self.get_logger().info(f'Subscribed to {image_topic}; waiting for frames...')
 
    def _predict(self, frame):
        kwargs = dict(conf=self.conf, imgsz=self.imgsz, verbose=False)
        if self.classes:
            kwargs['classes'] = self.classes
        if self.device:
            kwargs['device'] = self.device
        return self.model.predict(frame, **kwargs)[0]
 
    def on_image(self, msg: Image):
        t0 = time.perf_counter()
 
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # unsupported encoding, corrupt frame, etc.
            self.get_logger().error(f'Image conversion failed: {exc}',
                                    throttle_duration_sec=5.0)
            return
 
        result = self._predict(frame)
 
        # ---- build the detection message ----
        out = Detection2DArray()
        out.header = msg.header
        for box in result.boxes:
            cx, cy, w, h = box.xywh[0].tolist()
            cls_id = int(box.cls[0])
 
            det = Detection2D()
            det.header = msg.header
            det.bbox.center.position.x = float(cx)
            det.bbox.center.position.y = float(cy)
            det.bbox.size_x = float(w)
            det.bbox.size_y = float(h)
 
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(result.names[cls_id])
            hyp.hypothesis.score = float(box.conf[0])
            det.results.append(hyp)
            out.detections.append(det)
        self.det_pub.publish(out)
 
        # ---- annotated image (only if someone is listening) ----
        if self.publish_annotated and self.img_pub.get_subscription_count() > 0:
            self.img_pub.publish(bgr_to_imgmsg(result.plot(), msg.header))
 
        dt_ms = (time.perf_counter() - t0) * 1000.0
        self.get_logger().info(
            f'{len(out.detections)} detection(s), {dt_ms:.0f} ms per frame',
            throttle_duration_sec=2.0)
 
 
def main(args=None):
    rclpy.init(args=args)
    node = BalloonDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()
 