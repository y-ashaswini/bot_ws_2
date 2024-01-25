import cv2
import numpy as np

def convex_hull_pointing_up(ch):
    points_above_center, points_below_center = [], []
    
    x, y, w, h = cv2.boundingRect(ch) 
    aspect_ratio = w / h 

    if aspect_ratio < 0.8:
        
        vertical_center = y + h / 2

        for point in ch:
            if point[0][1] < vertical_center: 
                points_above_center.append(point)
            elif point[0][1] >= vertical_center:
                points_below_center.append(point)

        left_x = points_below_center[0][0][0]
        right_x = points_below_center[0][0][0]
        for point in points_below_center:
            if point[0][0] < left_x:
                left_x = point[0][0]
            if point[0][0] > right_x:
                right_x = point[0][0]

        for point in points_above_center:
            if (point[0][0] < left_x) or (point[0][0] > right_x):
                return False
    else:
        return False
        
    return True

# Use the camera (0 for default camera, adjust if needed)
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    
    if not ret:
        break
    
    img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    img_HSV = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2HSV)
    
    img_thresh_low = cv2.inRange(img_HSV, np.array([0, 135, 135]), np.array([15, 255, 255]))
    img_thresh_high = cv2.inRange(img_HSV, np.array([159, 135, 135]), np.array([179, 255, 255]))
    img_thresh = cv2.bitwise_or(img_thresh_low, img_thresh_high)
    
    kernel = np.ones((5, 5))
    img_thresh_opened = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel)
    img_thresh_blurred = cv2.medianBlur(img_thresh_opened, 5)
    img_edges = cv2.Canny(img_thresh_blurred, 80, 160)
    
    contours, _ = cv2.findContours(np.array(img_edges), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    approx_contours = []
    for c in contours:
        approx = cv2.approxPolyDP(c, 10, closed=True)
        approx_contours.append(approx)
    
    all_convex_hulls = []
    for ac in approx_contours:
        all_convex_hulls.append(cv2.convexHull(ac))
    
    convex_hulls_3to10 = []
    for ch in all_convex_hulls:
        if 3 <= len(ch) <= 10:
            convex_hulls_3to10.append(cv2.convexHull(ch))
    
    cones = []
    bounding_rects = []
    for ch in convex_hulls_3to10:
        if convex_hull_pointing_up(ch):
            cones.append(ch)
            rect = cv2.boundingRect(ch)
            bounding_rects.append(rect)
    
    img_res = frame.copy()
    cv2.drawContours(img_res, cones, -1, (255, 255, 255), 2)
    
    for rect in bounding_rects:
        cv2.rectangle(img_res, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), (0, 255, 0), 3)
    
    cv2.imshow('Cone Detection', img_res)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
import cv2
import numpy as np
from matplotlib import pyplot as plt

def convex_hull_pointing_up(ch):

    points_above_center, points_below_center = [], []
    
    x, y, w, h = cv2.boundingRect(ch) # координаты левого верхнего угла описывающего прямоугольника, ширина и высота
    aspect_ratio = w / h # отношение ширины прямоугольника к высоте

    # если прямоугольник узкий, продолжаем определение. Если нет, то контур не подходит
    if aspect_ratio < 0.8:
        # каждую точку контура классифицируем как лежащую выше или ниже центра
        vertical_center = y + h / 2

        for point in ch:
            if point[0][1] < vertical_center: # если координата y точки выше центра, то добавляем эту точку в список точек выше центра
                points_above_center.append(point)
            elif point[0][1] >= vertical_center:
                points_below_center.append(point)

        # определяем координаты x крайних точек, лежащих ниже центра
        left_x = points_below_center[0][0][0]
        right_x = points_below_center[0][0][0]
        for point in points_below_center:
            if point[0][0] < left_x:
                left_x = point[0][0]
            if point[0][0] > right_x:
                right_x = point[0][0]

        # проверяем, лежат ли верхние точки контура вне "основания". Если да, то контур не подходит
        for point in points_above_center:
            if (point[0][0] < left_x) or (point[0][0] > right_x):
                return False
    else:
        return False
        
    return True

# Use the camera (0 for default camera, adjust if needed)
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    
    if not ret:
        break
    
    img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    img_HSV = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2HSV)
    
    img_thresh_low = cv2.inRange(img_HSV, np.array([0, 135, 135]), np.array([15, 255, 255]))
    img_thresh_high = cv2.inRange(img_HSV, np.array([159, 135, 135]), np.array([179, 255, 255]))
    img_thresh = cv2.bitwise_or(img_thresh_low, img_thresh_high)
    
    kernel = np.ones((5, 5))
    img_thresh_opened = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel)
    img_thresh_blurred = cv2.medianBlur(img_thresh_opened, 5)
    img_edges = cv2.Canny(img_thresh_blurred, 80, 160)
    
    contours, _ = cv2.findContours(np.array(img_edges), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    approx_contours = []
    for c in contours:
        approx = cv2.approxPolyDP(c, 10, closed=True)
        approx_contours.append(approx)
    
    all_convex_hulls = []
    for ac in approx_contours:
        all_convex_hulls.append(cv2.convexHull(ac))
    
    convex_hulls_3to10 = []
    for ch in all_convex_hulls:
        if 3 <= len(ch) <= 10:
            convex_hulls_3to10.append(cv2.convexHull(ch))
    
    cones = []
    bounding_rects = []
    for ch in convex_hulls_3to10:
        if convex_hull_pointing_up(ch):
            cones.append(ch)
            rect = cv2.boundingRect(ch)
            bounding_rects.append(rect)
    
    img_res = frame.copy()
    cv2.drawContours(img_res, cones, -1, (255, 255, 255), 2)
    
    for rect in bounding_rects:
        cv2.rectangle(img_res, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), (0, 255, 0), 3)
    
    cv2.imshow('Cone Detection', img_res)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

