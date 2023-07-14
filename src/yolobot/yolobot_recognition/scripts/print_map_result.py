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
    (184, 230),
    (182, 230),
    (182, 230),
    (182, 230),
    (179, 230),
    (177, 230),
    (174, 230),
    (172, 230),
    (168, 231),
    (166, 231),
    (164, 231),
    (161, 230),
    (158, 230),
    (155, 230),
    (152, 230),
    (150, 230),
    (148, 230),
    (146, 230),
    (144, 230),
    (140, 230),
    (138, 230),
    (136, 231),
    (134, 230),
    (133, 230),
    (131, 230),
    (128, 229),
    (127, 229),
    (126, 229),
    (124, 230),
    (122, 230),
    (121, 230),
    (120, 230),
    (119, 230),
    (118, 230),
    (117, 230),
    (116, 230),
    (116, 229),
    (115, 229),
    (114, 229),
    (114, 229),
    (113, 229),
    (113, 230),
    (111, 230),
    (109, 230),
    (108, 230),
    (107, 230),
    (105, 230),
    (104, 229),
    (103, 229),
    (101, 229),
    (100, 229),
    (99, 229),
    (98, 230),
    (97, 230),
    (96, 230),
    (95, 230),
    (95, 229),
    (95, 229),
    (95, 230),
    (96, 230),
    (96, 230),
    (96, 230),
    (96, 229),
    (96, 230),
    (97, 229),
    (97, 229),
    (97, 229),
    (97, 229),
    (97, 230),
    (97, 230),
    (97, 230)
]

predicted_positions = [
    (188, 230),
    (184, 229),
    (184, 230),
    (182, 229),
    (182, 230),
    (179, 229),
    (177, 230),
    (175, 229),
    (172, 230),
    (169, 230),
    (166, 230),
    (162, 230),
    (159, 230),
    (155, 230),
    (151, 230),
    (148, 230),
    (145, 230),
    (143, 230),
    (141, 230),
    (138, 230),
    (135, 230),
    (133, 230),
    (131, 230),
    (129, 230),
    (127, 230),
    (125, 229),
    (124, 229),
    (122, 229),
    (121, 229),
    (119, 229),
    (118, 229),
    (117, 229),
    (115, 229),
    (115, 229),
    (114, 230),
    (113, 229),
    (113, 229),
    (112, 229),
    (112, 229),
    (111, 229),
    (111, 229),
    (110, 229),
    (110, 229),
    (109, 229),
    (107, 229),
    (106, 229),
    (105, 230),
    (103, 229),
    (102, 229),
    (101, 229),
    (99, 229),
    (98, 229),
    (96, 229),
    (95, 229),
    (94, 229),
    (93, 229),
    (93, 229),
    (92, 229),
    (92, 229),
    (92, 229),
    (93, 229),
    (93, 229),
    (94, 229),
    (94, 229),
    (95, 229),
    (95, 229),
    (96, 229),
    (96, 229),
    (96, 229),
    (97, 229),
    (97, 229)
]


for pt in target_positions:
    cv2.circle(img, pt, 1, color=(0, 0, 255), thickness=1)

# for i in range(len(target_positions) - 1):
#     cv2.line(img, target_positions[i], target_positions[i+1],
#              color=(255, 255, 0), thickness=1)

# for pt in predicted_positions:
#     cv2.circle(img, pt, 1, color=(255, 0, 0), thickness=1)

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
        cv2.imwrite(filename="origin_map_v3_result_target.png", img=img) #紅色點
        # cv2.imwrite(filename="origin_map_v3_result_predict.png", img=img) #藍色點

cv2.destroyAllWindows()
