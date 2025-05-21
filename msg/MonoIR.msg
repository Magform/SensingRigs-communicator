# Date and time of capture
string<=19  timestamp
# Name of the detected object
string<=10  label
# How accurate is the prediction
float32     confidence
# Array of coordinates for detection box (x1, x2, y1, y2)
float32[4]  box
