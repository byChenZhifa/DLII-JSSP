# Python Standard Library Imports
import os
import time
from pathlib import Path as PathlibPath
from os.path import isfile, join

# Third-Party Imports
import numpy as np
from PIL import Image
import imageio

#! Note: This script is copied to '... /gif_cache_png/' for usage.
IMAGE_DIR_NAME = "8_3_4_565"

# method = "frenet"
method = "JSSP"
BASE_DIR = PathlibPath(__file__).parent
PNGS_DIR = BASE_DIR / method / IMAGE_DIR_NAME
GIF_FILE_PATH = BASE_DIR / f"{method}-{IMAGE_DIR_NAME}.gif"
IMAGE_TEMPLATE = f"{{}}.png"


def generate_gif_from_images():
    tic = time.time()
    image_list = []

    # Gets a list of PNG image files in the directory
    if not os.path.exists(PNGS_DIR):
        print(f"路径 {PNGS_DIR} 不存在!")

    files_in_directory = [file for file in PNGS_DIR.iterdir() if file.is_file() and file.name.endswith(".png")]

    # Find the maximum number present in the folder
    max_image_index = -1
    for file in files_in_directory:
        try:
            index = int(file.stem.split("_")[-1])
            max_image_index = max(max_image_index, index)
        except ValueError:
            pass

    expected_dimensions = None
    expected_mode = None

    for i in range(max_image_index + 1):
        image_path = PNGS_DIR / IMAGE_TEMPLATE.format(i)
        if image_path.exists():
            img = Image.open(image_path)
            if expected_dimensions is None:
                expected_dimensions = img.size
                expected_mode = img.mode
            elif img.size != expected_dimensions or img.mode != expected_mode:
                print(f"Warning: Image {file.name} has different dimensions or color mode. Adjusting...")
                # Adjust image to match the expected dimensions and color mode
                img = img.resize(expected_dimensions)
            image_list.append(img)

    if image_list:
        if len(image_list) > 100:
            step = int(len(image_list) / 100) + 1
            image_list = image_list[::step]
        image_list[0].save(
            GIF_FILE_PATH,
            save_all=True,
            append_images=image_list[1:],
            duration=100,
            loop=0,
        )

        print(f"GIF文件保存在 {GIF_FILE_PATH}")

    print(f"###log### GIF文件保存在 {GIF_FILE_PATH}")
    toc = time.time()
    print(f"###log### GIF generated in {toc - tic:.2f} seconds.")


if __name__ == "__main__":
    generate_gif_from_images()
