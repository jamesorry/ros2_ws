#!/usr/bin/env python3
import cv2
import time
from pathlib import Path

def get_files_num(dir_path):
    count = 0
    for path in Path(dir_path).iterdir():
        if path.is_file():
            count += 1
    return count

Video_Path = Path().absolute() / 'videos' / 'output.mp4'
Image_Path = Path().absolute() / 'images'
image_count = get_files_num(Image_Path)

interval = 10  # 保存时的帧数间隔
frame_count = image_count  # 保存帧的索引
frame_index = 0  # 原视频的帧索引，与 interval*frame_count = frame_index

cap = cv2.VideoCapture(str(Video_Path))

if cap.isOpened():
    print("video from: ", Video_Path)
    success = True
else:
    success = False
    print("影片讀取失敗!")

while(success):
    success, frame = cap.read()
    if success is False:
        # print("---> 第%d幀讀取失敗:" % frame_index)
        print("convert image to: ", Image_Path)
        break

    # print("---> 正在讀取第%d幀:" % frame_index, success)

    if frame_index % interval == 0:
        frame_count += 1
        Img_File_Name = str(frame_count) + '.jpg'
        cv2.imwrite(str(Image_Path / Img_File_Name), frame)
        print("SAVE IMAGE: ", Img_File_Name)
    frame_index += 1

cap.release()
