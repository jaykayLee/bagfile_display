import sys
import datetime
import time
import numpy as pnp
cimport numpy as np
cimport cython
#@cython.boundscheck(False)
#@cython.wraparound(False)

cdef print_data_step_hello():
    print(f'--gui_step_Cython')

cdef test_value(np.ndarray[np.float64_t, ndim=1] out_color):
    print(f'test')

cdef int mult(int a, int b):
    return a * b


cdef int multloop(int a, int b):
    cdef int result = 1
    for i in range(b):
        result = result * b
    return result

@cython.boundscheck(False)  # Deactivate bounds checking
@cython.wraparound(False)   # Deactivate negative indexing.
cdef int set_pc_color_by_zindex(float zmin, float zmax, double * color_table, int len_pc, double * pc_xyz, double * pc_color, double * out_color):
    cdef double color_range = zmax+abs(zmin)
    cdef double pos_z = 0, ridx_f = 0
    cdef int cidx = 0, ridx = 0
    cdef int color_item_size = 4
    cdef int xyz_item_size = 3
    cdef int pc_xyz_idx = 0
    cdef int ci_idx = 0
    cdef int rc_idx = 0
    cdef int i = 0
    
    with nogil:
        for i in range(len_pc):
            #pos_z = pc_xyz[i, 2]
            #if pos_z < zmin or pos_z > zmax :
            #    pc_color[i] = out_color
            #else:
            #    pc_color[i] = color_table[int((pos_z/color_range)*10)]
            pc_xyz_idx = xyz_item_size*i + 2
            pos_z = pc_xyz[pc_xyz_idx]
            if pos_z < zmin or pos_z > zmax :
                ci_idx = i*color_item_size
                pc_color[ci_idx + 0] = out_color[0]
                pc_color[ci_idx + 1] = out_color[1]
                pc_color[ci_idx + 2] = out_color[2]
                pc_color[ci_idx + 3] = out_color[3]
            else:
                ridx_f = (pos_z/color_range)*10
                ridx = int(ridx_f)
                ci_idx = i*color_item_size
                rc_idx = ridx*color_item_size
                pc_color[ ci_idx + 0] = color_table[rc_idx + 0]
                pc_color[ ci_idx + 1] = color_table[rc_idx + 1]
                pc_color[ ci_idx + 2] = color_table[rc_idx + 2]
                pc_color[ ci_idx + 3] = color_table[rc_idx + 3]
    return 0

# cdef set_pointcloud_color_by_zindex(float zmin, float zmax, np.ndarray[np.float64_t, ndim=2] color_table, int len_pc, np.ndarray[np.float64_t, ndim=2] pc_xyz, np.ndarray[np.float64_t, ndim=2] pc_color, np.ndarray[np.float64_t, ndim=1] out_color):
#     cdef float color_range = zmax+abs(zmin)
#     cdef float pos_z
#     for i in range(len_pc):
#         pos_z = pc_xyz[i, 2]
#         if pos_z < zmin or pos_z > zmax :
#             pc_color[i] = out_color
#         else:
#             pc_color[i] = color_table[int((pos_z/color_range)*10)]


@cython.boundscheck(False)  # Deactivate bounds checking
@cython.wraparound(False)   # Deactivate negative indexing.
cdef set_pointcloud_color_by_zindex(float zmin, float zmax, np.ndarray[np.float64_t, ndim=2] color_table, int len_pc, np.ndarray[np.float64_t, ndim=2] pc_xyz, np.ndarray[np.float64_t, ndim=2] pc_color, np.ndarray[np.float64_t, ndim=1] out_color):
    #cdef double color_range = zmax+abs(zmin)
    #cdef double pos_z
    #cdef int cidx

    cdef np.ndarray[np.float64_t, ndim=2, mode = 'c'] ctbl_buff = pnp.ascontiguousarray(color_table, dtype="float64")
    cdef double * ctbl_cbuff = <np.float64_t*> ctbl_buff.data
    #cdef int color_item_size = pnp.shape(color_table)[1]
    cdef np.ndarray[np.float64_t, ndim=2, mode = 'c'] xyz_buff = pnp.ascontiguousarray(pc_xyz, dtype="float64")
    cdef double * xyz_cbuff = <np.float64_t*> xyz_buff.data
    #cdef int xyz_item_size = pnp.shape(pc_xyz)[1]
    cdef np.ndarray[np.float64_t, ndim=2, mode = 'c'] color_buff = pnp.ascontiguousarray(pc_color, dtype="float64")
    cdef double * color_cbuff = <np.float64_t*> color_buff.data
    cdef np.ndarray[np.float64_t, ndim=2, mode = 'c'] outc_buff = pnp.ascontiguousarray(out_color, dtype="float64")
    cdef double * outc_cbuff = <np.float64_t*> outc_buff.data
    
    set_pc_color_by_zindex(zmin, zmax, ctbl_cbuff, len_pc, xyz_cbuff, color_cbuff, outc_cbuff)

    # for i in range(len_pc):
    #     #pos_z = pc_xyz[i, 2]
    #     #if pos_z < zmin or pos_z > zmax :
    #     #    pc_color[i] = out_color
    #     #else:
    #     #    pc_color[i] = color_table[int((pos_z/color_range)*10)]
    #     pos_z = xyz_cbuff[xyz_item_size*i + 2]
    #     if pos_z < zmin or pos_z > zmax :
    #         color_cbuff[i*color_item_size + 0] = outc_cbuff[0]
    #         color_cbuff[i*color_item_size + 1] = outc_cbuff[1]
    #         color_cbuff[i*color_item_size + 2] = outc_cbuff[2]
    #         color_cbuff[i*color_item_size + 3] = outc_cbuff[3]
    #     else:
    #         cidx = (int)(pos_z/color_range)*10
    #         color_cbuff[ i*color_item_size + 0] = ctbl_cbuff[cidx*color_item_size + 0]
    #         color_cbuff[ i*color_item_size + 1] = ctbl_cbuff[cidx*color_item_size + 1]
    #         color_cbuff[ i*color_item_size + 2] = ctbl_cbuff[cidx*color_item_size + 2]
    #         color_cbuff[ i*color_item_size + 3] = ctbl_cbuff[cidx*color_item_size + 3]

