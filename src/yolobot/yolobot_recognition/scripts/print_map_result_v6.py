from kalmanfilter import KalmanFilter
import cv2
from filterpy.kalman import kalman_filter
import time
# Kalman Filter
kf = KalmanFilter()

# map_file_name = "origin_map_v2.png"
map_file_name = "origin_map_v3.png"

if map_file_name == "origin_map_v2.png":
    # ! map is my_own_map_v2.pgm
    origin_position_x = -5.18
    origin_position_y = -5.53

elif map_file_name == "origin_map_v3.png":
    # ! map is my_own_map_v3.pgm
    origin_position_x = -8.8
    origin_position_y = -5.24

img = cv2.imread(map_file_name)
img2 = cv2.imread(map_file_name)
img_combine = cv2.imread(map_file_name)
origin_height, origin_width, origin_channel = img.shape
map_resolution = 0.05
print("origin_height: ", origin_height)
print("origin_width: ", origin_width)
print("origin_channel: ", origin_channel)
print("origin_position_x: ", origin_position_x)
print("origin_position_y: ", origin_position_y)
fix_map_center_pixel_x = origin_position_x / map_resolution
fix_map_center_pixel_y = origin_position_y / map_resolution
print("fix_map_center_pixel_x: ", fix_map_center_pixel_x)
print("fix_map_center_pixel_y: ", fix_map_center_pixel_y)
map_center_pixel_x = (origin_width - (origin_width + fix_map_center_pixel_x))
map_center_pixel_y = origin_height + fix_map_center_pixel_y
print("map_center_pixel_x: ", map_center_pixel_x)
print("map_center_pixel_y: ", map_center_pixel_y)


target_positions = [
    (108, 200),
    (107, 199),
    (108, 199),
    (109, 199),
    (108, 198),
    (109, 197),
    (109, 197),
    (108, 197),
    (108, 196),
    (109, 195),
    (109, 193),
    (110, 191),
    (110, 190),
    (110, 190),
    (111, 189),
    (111, 188),
    (111, 187),
    (111, 186),
    (111, 186),
    (111, 185),
    (110, 184),
    (110, 183),
    (110, 182),
    (110, 182),
    (110, 181),
    (109, 180),
    (109, 179),
    (110, 178),
    (110, 176),
    (111, 175),
    (111, 173),
    (111, 172),
    (110, 171),
    (110, 170),
    (110, 169),
    (109, 168),
    (109, 166),
    (110, 165),
    (110, 165),
    (110, 163),
    (111, 161),
    (111, 160),
    (111, 159),
    (110, 158),
    (110, 157),
    (109, 156),
    (109, 154),
    (109, 153),
    (109, 152),
    (110, 151),
    (110, 149),
    (110, 148),
    (111, 145),
    (110, 144),
    (110, 143),
    (109, 142),
    (109, 140),
    (109, 139),
    (109, 138),
    (110, 136),
    (110, 133),
    (111, 132),
    (110, 131),
    (110, 129),
    (110, 71),
    (110, 69),
    (110, 67),
    (112, 65),
    (112, 64),
    (112, 62),
    (112, 60),
    (112, 57),
    (112, 56),
    (112, 55),
    (112, 52),
    (112, 48),
    (113, 46),
    (112, 44),
    (112, 43),
    (111, 43),
    (111, 42),
    (110, 41),
    (110, 41),
    (110, 40),
    (110, 40),
    (110, 40),
    (110, 39),
    (110, 39),
    (110, 37),
    (110, 37),
    (111, 36),
    (111, 35),
    (111, 35),
    (111, 35),
    (110, 34),
    (110, 34),
    (110, 34),
    (110, 33),
    (109, 33),
    (109, 32),
    (109, 32),
    (109, 32),
    (109, 31),
    (109, 31),
    (109, 31),
    (109, 31),
    (109, 31),
    (109, 31),
    (109, 31),
    (109, 31),
    (110, 31),
    (110, 31),
    (109, 31),
    (109, 31),
    (109, 31),
    (109, 31),
    (110, 31),
    (109, 31),
    (109, 31),
    (110, 31),
    (110, 31)
]

predicted_positions = [
    (109, 202),
    (109, 190),
    (109, 200),
    (108, 197),
    (108, 197),
    (108, 196),
    (108, 196),
    (108, 195),
    (109, 193),
    (109, 192),
    (109, 191),
    (110, 190),
    (110, 188),
    (110, 187),
    (111, 185),
    (111, 184),
    (111, 183),
    (111, 183),
    (110, 182),
    (110, 181),
    (110, 180),
    (110, 179),
    (109, 179),
    (109, 178),
    (109, 177),
    (109, 176),
    (110, 174),
    (110, 173),
    (110, 172),
    (110, 170),
    (110, 169),
    (110, 168),
    (109, 166),
    (109, 165),
    (109, 164),
    (109, 163),
    (109, 162),
    (110, 160),
    (110, 159),
    (110, 157),
    (110, 156),
    (110, 155),
    (110, 154),
    (109, 153),
    (109, 152),
    (109, 150),
    (109, 149),
    (109, 148),
    (109, 147),
    (110, 145),
    (110, 143),
    (110, 142),
    (109, 140),
    (109, 139),
    (109, 137),
    (109, 136),
    (109, 134),
    (109, 132),
    (110, 131),
    (110, 129),
    (110, 127),
    (108, 69),
    (109, 67),
    (109, 66),
    (108, 65),
    (109, 64),
    (109, 62),
    (109, 60),
    (111, 58),
    (110, 55),
    (110, 53),
    (110, 52),
    (111, 50),
    (110, 48),
    (110, 44),
    (110, 42),
    (110, 43),
    (109, 40),
    (109, 37),
    (109, 36),
    (109, 35),
    (109, 34),
    (110, 32),
    (110, 32),
    (111, 33),
    (110, 34),
    (110, 34),
    (110, 36),
    (109, 36),
    (109, 36),
    (109, 35),
    (109, 35),
    (110, 34),
    (110, 34),
    (110, 33),
    (110, 33),
    (110, 33),
    (110, 32),
    (110, 32),
    (110, 32),
    (109, 32),
    (109, 31),
    (109, 31),
    (109, 31),
    (108, 30),
    (108, 30),
    (108, 30),
    (108, 30),
    (108, 30),
    (108, 30),
    (108, 30),
    (108, 30),
    (108, 30),
    (109, 30),
    (109, 30),
    (109, 30),
    (109, 30),
    (109, 30),
    (109, 30),
    (109, 31),
    (109, 31),
    (109, 31),
    (109, 31)

]


for pt in target_positions:
    cv2.circle(img, pt, 1, color=(0, 0, 255), thickness=1)
    cv2.circle(img_combine, pt, 1, color=(0, 0, 255), thickness=1)

# for i in range(len(target_positions) - 1):
#     cv2.line(img, target_positions[i], target_positions[i+1],
#              color=(255, 255, 0), thickness=1)

for pt2 in predicted_positions:
    cv2.circle(img2, pt2, 1, color=(255, 0, 0), thickness=1)
    cv2.circle(img_combine, pt2, 1, color=(255, 0, 0), thickness=1)

# for i in range(len(predicted_positions) - 1):
#     cv2.line(
#         img, predicted_positions[i], predicted_positions[i+1], color=(255, 0, 0), thickness=1)

dsize = (int(2.0 * origin_width),
         int(2.0 * origin_height))
img = cv2.resize(img, dsize)
img2 = cv2.resize(img2, dsize)
img_combine = cv2.resize(img_combine, dsize)

while(True):
    cv2.imshow("target_positions", img)
    cv2.imshow("predicted_positions", img2)
    cv2.imshow("img_combine", img_combine)
    key = cv2.waitKey(1)

    if key & 0xFF == ord('q'):
        break
    elif key == ord('s'):
        print("save")
        cv2.imwrite(filename="阻擋預測2_target_positions.png",
                    img=img)
        cv2.imwrite(filename="阻擋預測2_predicted_positions.png",
                    img=img2)
        cv2.imwrite(filename="阻擋預測2_img_combine.png",
                    img=img_combine)

cv2.destroyAllWindows()
