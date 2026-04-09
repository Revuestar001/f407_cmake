#pragma once

#include "arm_math.h"

#include <stdbool.h>

typedef arm_matrix_instance_f32 mathMatrix_t;

#define mathMatrixInit arm_mat_init_f32
#define mathMatrixScale arm_mat_scale_f32
#define mathMatrixAdd arm_mat_add_f32
#define mathMatrixSub arm_mat_sub_f32
#define mathMatrixMult arm_mat_mult_f32
#define mathMatrixInverse arm_mat_inverse_f32

bool mathMatrixSetZero(mathMatrix_t *mat);
bool mathMatrixSetIdentity(mathMatrix_t *mat);
bool mathMatrixSetSymmetricInPlace(mathMatrix_t *mat);
bool mathMatrixTranspose(const mathMatrix_t *mat, mathMatrix_t *mat_out);
bool mathMatrixSetDiagScalar(mathMatrix_t *mat, float value);
bool mathMatrixSetDiag(mathMatrix_t *mat, const float *diag, uint16_t diag_len);
// 行列row_start col_start从0开始,block_rows block_cols表示要操作的block的总行数和总列数,可以为0，均为0时表示空矩阵，请注意不是结束位置行列，而是尺寸
bool mathMatrixSetBlock(mathMatrix_t *mat,
                        uint16_t row_start,
                        uint16_t col_start,
                        uint16_t block_rows,
                        uint16_t block_cols,
                        const float *block_data);
bool mathMatrixGetBlockMatrix(const mathMatrix_t *mat,
                              uint16_t row_start,
                              uint16_t col_start,
                              mathMatrix_t *block_out);
bool mathMatrixSetBlockIdentity(mathMatrix_t *mat,
                                uint16_t row_start,
                                uint16_t col_start,
                                uint16_t block_rows,
                                uint16_t block_cols);
bool mathMatrixSetBlockDiagScalar(mathMatrix_t *mat,
                                  uint16_t row_start,
                                  uint16_t col_start,
                                  uint16_t block_rows,
                                  uint16_t block_cols,
                                  float value);
bool mathMatrixSetBlockByMatrix(mathMatrix_t *mat,
                                uint16_t row_start,
                                uint16_t col_start,
                                const mathMatrix_t *block_mat);
bool mathMatrixGetBlock(const mathMatrix_t *mat,
                        uint16_t row_start,
                        uint16_t col_start,
                        uint16_t block_rows,
                        uint16_t block_cols,
                        float *block_data);
