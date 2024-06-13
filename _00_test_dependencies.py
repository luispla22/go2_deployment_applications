try:
    from scipy.spatial.transform import Rotation
except Exception as e:
    print(e)
    print("Need to install scipy!")

try:
    import cv2
except Exception as e:
    print(e)
    print("Need to install opencv-python!")

try:
    import PIL
except Exception as e:
    print(e)
    print("Need to install pillow!")