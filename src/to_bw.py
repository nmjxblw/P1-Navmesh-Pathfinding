import sys
import os
from PIL import Image


def to_bw(file_path, file_name):
    # load image
    image = Image.open(file_path + "/" + file_name)

    # convert to grayscale image
    gray_image = image.convert("L")

    # set threshold
    threshold = 185

    # Thresholding to generate pure black and white image
    bw_image = gray_image.point(lambda x: 255 if x > threshold else 0, "1")

    # save image
    bw_image.save(file_path + "/test_image.png")


if __name__ == "__main__":
    global file_name, file_path

    if len(sys.argv) == 3:
        file_path = os.path.dirname(sys.argv[1])
        file_name = os.path.basename(sys.argv[2])
    elif len(sys.argv) == 2:
        file_path = os.path.dirname(sys.argv[1])
        file_name = os.path.basename(sys.argv[1])
    elif len(sys.argv) == 1:
        file_path = "../input"
        file_name = "test.png"
    else:
        print("usage: %s path file_name" % sys.argv[0])
        sys.exit(-1)

    print(f"file path:{file_path} \tfile name:{file_name}")

    to_bw(file_path, file_name)

    print(f"{file_path}/test_image.png has been generated.")

    sys.exit()
