import cv2
import zmq
import numpy as np
from ultralytics import YOLO
from helpers.get_bounding_box import get_bounding_boxes
from helpers.get_diagonal import get_diagonal

HOST = "192.168.137.214"
PORT = 1807
FRAME_WIDTH = 224
FRAME_HEIGHT = 224
CHANNELS = 3


def main():
    model = YOLO('runs/detect/train3/weights/best.pt')

    # ZMQ subscriber
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.setsockopt(zmq.SUBSCRIBE, b"")
    socket.connect(f"tcp://{HOST}:{PORT}")

    print("Connected to ZMQ stream, press 'q' to quit")

    while True:
        # Receive frame
        buf = socket.recv()
        frame = np.frombuffer(buf, dtype=np.uint8).reshape(
            (FRAME_HEIGHT, FRAME_WIDTH, CHANNELS)
        )

        # Run YOLO
        results = model(frame, conf=0.5)
        result = results[0]

        # Get bboxes info (list of dicts)
        boxes = get_bounding_boxes(result)

        annotated = frame.copy()

        for b in boxes:
            # Box corners
            x1 = int(b["left_bottom_x"])
            y1 = int(b["left_bottom_y"])
            x2 = int(b["right_top_x"])
            y2 = int(b["right_top_y"])
            conf = b["conf"]
            cls_name = b["class_name"]

            # Draw bbox (top-left, bottom-right)
            cv2.rectangle(annotated, (x1, y2), (x2, y1), (0, 255, 0), 2)

            # Line and its info
            slope, length, angle = get_diagonal(x1, y1, x2, y2)
            cv2.line(annotated, (x1, y1), (x2, y2), (255, 0, 0), 2)

            # Text for bbox
            text_box = f"{cls_name} {conf:.2f}"
            cv2.putText(
                annotated,
                text_box,
                (x1, max(0, y2 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )

            # Text for line (center of line)
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            if slope is None:
                slope_text = "inf"
            else:
                slope_text = f"{slope:.2f}"
            text_line = f"m={slope_text}, L={length:.1f}, a={angle:.1f}"
            cv2.putText(
                annotated,
                text_line,
                (cx, cy),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255, 0, 0),
                1,
                cv2.LINE_AA,
            )

        cv2.imshow("Line detection", annotated)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
