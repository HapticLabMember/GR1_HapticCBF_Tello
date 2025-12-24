import numpy as np
import cv2
img = np.random.randint(0, 255, (720, 960), dtype=np.uint8)
print("Displaying test image...")
cv2.imshow("Test Window", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
print("Done.")

