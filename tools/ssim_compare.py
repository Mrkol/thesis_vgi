import os.path

from skimage import io, img_as_float
from skimage.metrics import structural_similarity as ssim


def compare(path1, path2):
    img1 = img_as_float(io.imread(path1))
    img2 = img_as_float(io.imread(path2))
    err = ssim(img1, img2, data_range=img1.max() - img1.min(), multichannel=True)
    print(f"{os.path.basename(path1)} - {os.path.basename(path2)}: {err}")


compare("../text/images/high1.png", "../text/images/high1.png")
compare("../text/images/high1.png", "../text/images/ours1.png")
compare("../text/images/high1.png", "../text/images/lod1.png")

compare("../text/images/high2.png", "../text/images/high2.png")
compare("../text/images/high2.png", "../text/images/ours2.png")
compare("../text/images/high2.png", "../text/images/lod2.png")

compare("../text/images/high3.png", "../text/images/high3.png")
compare("../text/images/high3.png", "../text/images/ours3.png")
compare("../text/images/high3.png", "../text/images/lod3.png")
