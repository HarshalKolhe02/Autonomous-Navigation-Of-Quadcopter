# Bonus of Quesiton 1 including 1
# run on vs code
# press R to turn on the pixel art mode

import cv2
import numpy as np

img = cv2.imread(r"C:\Users\harsh\Downloads\test (2).jpeg")

pixel_art_mode = False
pixel_size = 10

def pixelate_and_blur(x, y):
    global img_copy
    x_start = max(0, (x // pixel_size) * pixel_size)
    y_start = max(0, (y // pixel_size) * pixel_size)
    x_end = min(img_copy.shape[1], x_start + pixel_size)
    y_end = min(img_copy.shape[0], y_start + pixel_size)


    region = img_copy[y_start:y_end, x_start:x_end]


    if region.size > 0:

        blurred = cv2.blur(region, (max(1, pixel_size // 2), max(1, pixel_size // 2)))


        img_copy[y_start:y_end, x_start:x_end] = blurred

def mouse_callback(event, x, y, flags, param):
    global img_copy, pixel_art_mode

    if pixel_art_mode:
        if event == cv2.EVENT_LBUTTONDOWN or (event == cv2.EVENT_MOUSEMOVE and flags & cv2.EVENT_FLAG_LBUTTON):
            pixelate_and_blur(x, y)
    else:
        img_copy = img.copy()
        if 0 <= y < img.shape[0] and 0 <= x < img.shape[1]:
            bgr = img[y, x]
            bgr_array = np.uint8([[bgr]])
            hsv_array = cv2.cvtColor(bgr_array, cv2.COLOR_BGR2HSV)
            hsv = hsv_array[0, 0]

            hsv_text = f"HSV: {hsv}"
            bgr_text = f"BGR: {bgr}"

            text_x = x + 10
            text_y_hsv = y + 40
            text_y_bgr = y + 20

            if text_x + 160 > img.shape[1]:
                text_x = x - 160

            if text_y_hsv + 10 > img.shape[0]:
                text_y_hsv = y - 10

            if text_y_bgr + 30 > img.shape[0]:
                text_y_bgr = y - 30

            cv2.putText(img_copy, hsv_text, (text_x, text_y_hsv), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(img_copy, bgr_text, (text_x, text_y_bgr), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

cv2.namedWindow("Image Viewer")
cv2.setMouseCallback("Image Viewer", mouse_callback)

img_copy = img.copy()

while True:
    cv2.imshow("Image Viewer", img_copy)
    key = cv2.waitKey(1)

    if key == 27:  # ESC key
        break
    elif key == ord('r') or key == ord('R'):
        pixel_art_mode = not pixel_art_mode
        if pixel_art_mode:
            img_copy = img.copy()
        print("Pixel Art Mode:", "On" if pixel_art_mode else "Off")
    elif key == ord('+') or key == ord('='):
        pixel_size = min(pixel_size + 1, 100)
        print("Pixel Size:", pixel_size)
    elif key == ord('-') or key == ord('_'):
        pixel_size = max(pixel_size - 1, 1)
        print("Pixel Size:", pixel_size)

cv2.destroyAllWindows()