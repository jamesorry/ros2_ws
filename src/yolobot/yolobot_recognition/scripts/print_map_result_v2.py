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
    (94, 200),
    (94, 200),
    (94, 199),
    (95, 198),
    (95, 198),
    (95, 197),
    (95, 196),
    (95, 196),
    (95, 195),
    (95, 195),
    (95, 194),
    (96, 192),
    (96, 192),
    (96, 190),
    (96, 190),
    (96, 188),
    (96, 187),
    (96, 187),
    (96, 186),
    (95, 184),
    (95, 183),
    (95, 182),
    (95, 181),
    (95, 180),
    (95, 179),
    (95, 178),
    (96, 177),
    (96, 176),
    (96, 174),
    (96, 173),
    (95, 172),
    (95, 171),
    (95, 170),
    (95, 167),
    (95, 166),
    (95, 165),
    (95, 164),
    (96, 163),
    (96, 160),
    (96, 159),
    (95, 158),
    (95, 157),
    (95, 156),
    (95, 154),
    (95, 153),
    (95, 150),
    (95, 150),
    (96, 148),
    (96, 147),
    (96, 145),
    (95, 144),
    (95, 143),
    (95, 141),
    (95, 139),
    (95, 138),
    (95, 137),
    (95, 136),
    (95, 135),
    (96, 132),
    (96, 131),
    (95, 130),
    (95, 129),
    (95, 128),
    (95, 125),
    (95, 124),
    (95, 123),
    (95, 121),
    (96, 121),
    (96, 118),
    (96, 117),
    (95, 116)
]

predicted_positions = [
    (95, 202),
    (93, 199),
    (94, 200),
    (94, 198),
    (94, 199),
    (94, 197),
    (94, 197),
    (95, 196),
    (94, 196),
    (95, 195),
    (95, 194),
    (95, 193),
    (95, 192),
    (95, 190),
    (96, 189),
    (96, 188),
    (96, 187),
    (96, 186),
    (96, 185),
    (95, 183),
    (95, 182),
    (95, 181),
    (95, 180),
    (95, 179),
    (95, 178),
    (94, 177),
    (95, 176),
    (95, 175),
    (95, 173),
    (95, 172),
    (95, 171),
    (95, 170),
    (95, 169),
    (95, 167),
    (95, 165),
    (95, 164),
    (95, 162),
    (95, 161),
    (95, 159),
    (95, 158),
    (95, 156),
    (95, 155),
    (95, 154),
    (95, 152),
    (95, 151),
    (95, 149),
    (95, 148),
    (95, 146),
    (95, 145),
    (95, 144),
    (95, 142),
    (95, 141),
    (95, 139),
    (95, 138),
    (95, 136),
    (95, 135),
    (95, 134),
    (95, 133),
    (95, 131),
    (95, 129),
    (95, 128),
    (95, 127),
    (95, 126),
    (95, 124),
    (95, 123),
    (95, 121),
    (95, 120),
    (95, 119),
    (95, 117),
    (95, 115),
    (95, 114)
]


# for pt in target_positions:
#     cv2.circle(img, pt, 1, color=(0, 0, 255), thickness=1)

# for i in range(len(target_positions) - 1):
#     cv2.line(img, target_positions[i], target_positions[i+1],
#              color=(255, 255, 0), thickness=1)

for pt in predicted_positions:
    cv2.circle(img, pt, 1, color=(255, 0, 0), thickness=1)

# for i in range(len(predicted_positions) - 1):
#     cv2.line(
#         img, predicted_positions[i], predicted_positions[i+1], color=(255, 0, 0), thickness=1)

dsize = (int(2.0 * origin_width),
         int(2.0 * origin_height))
img = cv2.resize(img, dsize)

while(True):
    cv2.imshow("Img", img)
    key = cv2.waitKey(1)

    if key & 0xFF == ord('q'):
        break
    elif key == ord('s'):
        print("save")
        # cv2.imwrite(filename="origin_map_v3_result2_target.png", img=img)  # 紅色點
        cv2.imwrite(filename="origin_map_v3_result2_predict.png", img=img) #藍色點

cv2.destroyAllWindows()
