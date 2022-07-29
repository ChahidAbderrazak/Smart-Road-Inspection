import os
import numpy as np
import matplotlib.pyplot as plt

from lib import utils

img_path='/media/abdo2020/DATA1/Datasets/images-dataset/raw-data/road-conditions-google/good-roads/road2.jpeg'
image = utils.load_image(img_path)
img=image


# from skimage import data
# from skimage import color
color.rgb2gray(img)
# from skimage import morphology
# from skimage import segmentation

# # Input data
# # img = data.immunohistochemistry()

# # Compute a mask
# lum = color.rgb2gray(img)
# mask = morphology.remove_small_holes(
#     morphology.remove_small_objects(
#         lum < 0.7, 500),
#     500)

# mask = morphology.opening(mask, morphology.disk(3))

# # SLIC result
# slic = segmentation.slic(img, n_segments=200, start_label=1)

# # maskSLIC result
# m_slic = segmentation.slic(img, n_segments=100, mask=mask, start_label=1)

# # Display result
# fig, ax_arr = plt.subplots(2, 2, sharex=True, sharey=True, figsize=(10, 10))
# ax1, ax2, ax3, ax4 = ax_arr.ravel()

# ax1.imshow(img)
# ax1.set_title('Original image')

# ax2.imshow(mask, cmap='gray')
# ax2.set_title('Mask')

# ax3.imshow(segmentation.mark_boundaries(img, slic))
# ax3.contour(mask, colors='red', linewidths=1)
# ax3.set_title('SLIC')

# ax4.imshow(segmentation.mark_boundaries(img, m_slic))
# ax4.contour(mask, colors='red', linewidths=1)
# ax4.set_title('maskSLIC')

# for ax in ax_arr.ravel():
#     ax.set_axis_off()

# plt.tight_layout()
# plt.show()