import cv2
import time

cv2.namedWindow("Video")

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
cap.set(cv2.CAP_PROP_FPS, 2)
#cap.set(cv2.CAP_PROP_AUTOFOCUS, False)

while True:
	status, img = cap.read()

	print(status)

	if status:
		cv2.imshow("Video", img)

	k = 0xFF & cv2.waitKey(1)

	print(img.shape)

	if k == ord('q'):
		break
	elif k == ord('s'):
		left = img[:, 0:(1920-1), :]
		right = img[:, 1920:, :]
		cv2.imwrite("full.jpg", img)
		cv2.imwrite("left.jpg", left)
		cv2.imwrite("right.jpg", right)
		print("Saved!")
