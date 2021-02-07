//Contains functions used to help in no-fly zone avoidance.

typedef coords float[2];

int intersect_two_lines(float *x_i, float *y_i, float ax0, float ay0, float ax1, float ay1, float bx0, float by0, float bx1, float by1);

int path_intersect_nfz(int num_verts, coords *verts);
