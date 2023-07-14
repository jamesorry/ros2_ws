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
    (198, 230),
    (197, 230),
    (197, 230),
    (197, 230),
    (197, 230),
    (196, 230),
    (196, 230),
    (196, 230),
    (196, 230),
    (196, 230),
    (196, 230),
    (195, 230),
    (195, 230),
    (195, 231),
    (194, 231),
    (193, 231),
    (192, 231),
    (191, 230),
    (191, 230),
    (190, 230),
    (190, 230),
    (189, 230),
    (188, 230),
    (187, 230),
    (186, 230),
    (186, 229),
    (185, 229),
    (184, 229),
    (184, 229),
    (183, 229),
    (183, 230),
    (182, 230),
    (180, 230),
    (180, 231),
    (179, 231),
    (178, 231),
    (177, 231),
    (177, 230),
    (176, 230),
    (175, 230),
    (174, 230),
    (173, 230),
    (172, 230),
    (171, 229),
    (170, 229),
    (168, 229),
    (168, 230),
    (167, 230),
    (166, 230),
    (165, 230),
    (164, 231),
    (163, 231),
    (162, 230),
    (162, 230),
    (161, 230),
    (159, 229),
    (158, 229),
    (157, 229),
    (156, 229),
    (155, 229),
    (153, 230),
    (152, 230),
    (151, 230),
    (150, 231),
    (149, 231),
    (148, 230),
    (148, 230),
    (146, 230),
    (145, 229),
    (143, 229),
    (142, 229),
    (141, 229),
    (141, 230),
    (139, 230),
    (138, 230),
    (137, 231),
    (136, 231),
    (135, 231),
    (134, 230),
    # (93, 231),
    # (93, 230),
    # (91, 227),
    # (88, 227),
    # (87, 226),
    # (85, 226),
    # (85, 227),
    # (84, 228),
    # (83, 229),
    # (81, 229),
    # (80, 229),
    (79, 230),
    (79, 230),
    (78, 230),
    (77, 230),
    (77, 230),
    (76, 230),
    (75, 230),
    (74, 230),
    (73, 230),
    (72, 231),
    (72, 231),
    (71, 231),
    (70, 231),
    (70, 231),
    (69, 232),
    (69, 232),
    (68, 232),
    (67, 231),
    (67, 231),
    (66, 231),
    (66, 231),
    (65, 231),
    (65, 231),
    (64, 230),
    (63, 230),
    (63, 230),
    (62, 230),
    (62, 230),
    (61, 230),
    (61, 230),
    (60, 230),
    (60, 230),
    (60, 230),
    (59, 230),
    (59, 230),
    (59, 231),
    (58, 231),
    (58, 231),
    (58, 231),
    (58, 230),
    (57, 231),
    (57, 231),
    (57, 231),
    (57, 231),
    (56, 231),
    (55, 231),
    (54, 231),
    (53, 230),
    (52, 230),
    (51, 230),
    (51, 230),
    (52, 230),
    (52, 230),
    (52, 230),
    (53, 230),
    (53, 230),
    (54, 230),
    (55, 229),
    (56, 229),
    (56, 229),
    (57, 229),
    (57, 229),
    (57, 229),
    (57, 229),
    (57, 229),
    (57, 229),
    (57, 229),
    (57, 229),
    (57, 229),
    (57, 230),
    (57, 230)
]

predicted_positions = [
    # (10, 6),
    # (131, 149),
    # (271, 254),
    # (213, 230),
    # (184, 224),
    (199, 231),
    (195, 229),
    (196, 230),
    (195, 229),
    (196, 230),
    (195, 229),
    (196, 230),
    (195, 229),
    (195, 230),
    (194, 230),
    (194, 230),
    (193, 230),
    (192, 230),
    (191, 230),
    (190, 230),
    (189, 230),
    (188, 230),
    (187, 230),
    (186, 230),
    (185, 230),
    (185, 229),
    (184, 229),
    (183, 229),
    (182, 228),
    (182, 228),
    (181, 229),
    (181, 229),
    (180, 229),
    (179, 230),
    (178, 230),
    (177, 230),
    (176, 230),
    (175, 230),
    (175, 230),
    (174, 230),
    (173, 230),
    (172, 230),
    (171, 230),
    (170, 229),
    (169, 229),
    (168, 229),
    (167, 229),
    (166, 229),
    (165, 229),
    (164, 229),
    (163, 230),
    (162, 230),
    (161, 230),
    (160, 230),
    (159, 230),
    (158, 229),
    (157, 229),
    (156, 229),
    (155, 229),
    (154, 229),
    (152, 229),
    (151, 229),
    (150, 229),
    (149, 230),
    (148, 230),
    (146, 230),
    (146, 230),
    (145, 230),
    (144, 229),
    (142, 229),
    (141, 229),
    (140, 229),
    (139, 229),
    (138, 229),
    (137, 229),
    (136, 230),
    (135, 230),
    (134, 230),
    # (132, 230),
    # (117, 230),
    # (106, 230),
    # (97, 229),
    # (89, 228),
    # (83, 227),
    (79, 230),
    (75, 230),
    (73, 231),
    (72, 230),
    (71, 230),
    (70, 230),
    (70, 231),
    (70, 230),
    (71, 230),
    (71, 230),
    (71, 230),
    (72, 230),
    (72, 230),
    (72, 230),
    (72, 230),
    (71, 230),
    (70, 231),
    (70, 231),
    (69, 231),
    (68, 231),
    (68, 231),
    (67, 231),
    (66, 232),
    (66, 231),
    (65, 231),
    (65, 231),
    (64, 231),
    (64, 231),
    (63, 231),
    (63, 230),
    (62, 230),
    (62, 230),
    (61, 230),
    (61, 229),
    (60, 229),
    (60, 229),
    (59, 229),
    (59, 229),
    (58, 229),
    (58, 229),
    (58, 229),
    (57, 230),
    (57, 230),
    (57, 230),
    (57, 230),
    (57, 230),
    (56, 230),
    (56, 231),
    (56, 231),
    (56, 231),
    (55, 231),
    (55, 231),
    (54, 231),
    (53, 230),
    (52, 230),
    (51, 230),
    (50, 230),
    (50, 230),
    (50, 229),
    (50, 229),
    (50, 229),
    (51, 229),
    (52, 229),
    (53, 229),
    (53, 229),
    (54, 229),
    (55, 228),
    (56, 228),
    (57, 228),
    (57, 228),
    (57, 228),
    (57, 228),
    (57, 228),
    (57, 228),
    (57, 228),
    (57, 229),
    (57, 229)

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
        # cv2.imwrite(filename="origin_map_v3_result2_target.png", img=img)  # 紅色點
        cv2.imwrite(filename="阻擋預測1_target_positions.png",
                    img=img)  # 藍色點
        cv2.imwrite(filename="阻擋預測1_predicted_positions.png",
                    img=img2)  # 藍色點
        cv2.imwrite(filename="阻擋預測1_img_combine.png",
                    img=img_combine)  # 藍色點

cv2.destroyAllWindows()
