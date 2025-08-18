import cv2

# Read the image
image = cv2.imread("test.jpg")

# Convert BGR to YUV
yuv_image = cv2.cvtColor(image, cv2.COLOR_BGR2YUV)

# Save or display the YUV image (note: saving requires converting back to BGR or RGB)
cv2.imwrite("yuv_image.jpg", yuv_image)
