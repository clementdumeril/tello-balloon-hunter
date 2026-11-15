import cv2
from ultralytics import YOLO

model = YOLO("tools/yolo11n.pt")          # replace with your balloon weights later
cap = cv2.VideoCapture("/home/miles/tello-balloon-hunter/tools/cv_testing/balloon_sample.mp4")   # a video file for now

while True:
    ok, frame = cap.read()
    if not ok:
        break
    annotated = model(frame, conf=0.4, verbose=False)[0].plot()
    cv2.imshow("detections", annotated)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
cap.release()
cv2.destroyAllWindows()