import cv2

cap = cv2.VideoCapture(2, cv2.CAP_AVFOUNDATION)  # if this grabs your built-in webcam, try 1, 2, etc.

# Ask for the full stereo resolution
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

if not cap.isOpened():
    print("Camera did not open")
    exit()

while True:
    ok, frame = cap.read()
    if not ok:
        print("Frame read failed")
        break
    print(f"Got frame: {frame.shape}")  # should print (720, 2560, 3)
    cv2.imshow("stereo", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()