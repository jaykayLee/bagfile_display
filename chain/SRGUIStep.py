import sys
import numpy as np
import datetime
import time

def print_data_step_hello():
    print(f'--gui_step_python')


def set_pointcloud_color_by_zindex(zmin, zmax, color_table, len_pc, pc_xyz, pc_color, out_color):
    color_range = zmax+abs(zmin)
    color_index = 0
    for i in range(len_pc):
        if pc_xyz[i][2] < zmin or pc_xyz[i][2] > zmax :
            pc_color[i] = out_color
        else:
            color_index = int((pc_xyz[i][2]/color_range)*10)
            pc_color[i] = color_table[color_index]
    