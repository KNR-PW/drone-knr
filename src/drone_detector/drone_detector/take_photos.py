import cv2
import time

photos_path = "/home/raspberrypi/Drone/drone_photos/"
series_counter = 0
i = 0
video_capture = cv2.VideoCapture(0)
while (video_capture.isOpened() == False):
    print('Waiting for camera video cpture to open...')

ret, frame = video_capture.read()

while True:
    time.sleep(1)
    ret, frame = video_capture.read()
    if not ret:
        print("error")
    else:
        i += 1
        print(photos_path + "drone_photo" + str(series_counter) + str(i) + '.jpg')
        cv2.imwrite(photos_path + "drone_photo" + str(series_counter) + str(i) + '.jpg', frame)