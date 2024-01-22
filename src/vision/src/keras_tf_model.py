import matplotlib.pyplot as plt
import numpy as np
import tensorflow as tf
from tensorflow import keras
import cv2

# Load your model
model = keras.models.load_model("/home/alexii/robotics/onebot_ws/src/vision/Model")

def array2dir(array):
    if array[0][0] > array[0][1] and array[0][0] > array[0][2]:
        print("Right")
    elif array[0][1] > array[0][0] and array[0][1] > array[0][2]:
        print("Left")
    elif array[0][2] > array[0][1] and array[0][2] > array[0][0]:
        print("Up")
    else:
        print("HATA!")

# Initialize the camera
cam = cv2.VideoCapture(0)  # 0 -> index of camera

print("Realtime Start")
while True:
    s, img = cam.read()
    if s:  # Frame captured without any errors
        cv2.imshow('Camera', img)

        img = cv2.resize(img, (224, 224))

        img = np.asarray(img)
        plt.imshow(img)
        img = np.expand_dims(img, axis=0)
        output = model.predict(img)
        # print(output)
        array2dir(output)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cam.release()
cv2.destroyAllWindows()
