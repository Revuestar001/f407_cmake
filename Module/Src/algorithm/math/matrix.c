#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "matrix.h"

bool mathMatrixSetZero(mathMatrix_t *mat)
{
    if (mat == NULL || mat->pData == NULL) {
        return false;
    }

    memset(mat->pData, 0, sizeof(float) * mat->numRows * mat->numCols);
    return true;
}

bool mathMatrixSetIdentity(mathMatrix_t *mat)
{
    if (mat == NULL || mat->pData == NULL) {
        return false;
    }

    if (mat->numRows != mat->numCols) {
        return false;
    }

    uint16_t diag_len;

    if (mathMatrixSetZero(mat) == false) {
        return false;
    }

    diag_len = mat->numRows;
    for (uint16_t i = 0; i < diag_len; i++) {
        mat->pData[mat->numCols * i + i] = 1.0f;
    }

    return true;
}

bool mathMatrixSetSymmetricInPlace(mathMatrix_t *mat)
{
    if (mat == NULL || mat->pData == NULL) {
        return false;
    }

    if (mat->numRows != mat->numCols) {
        return false;
    }

    for (uint16_t r = 0; r < mat->numRows; r++) {
        for (uint16_t c = r + 1U; c < mat->numCols; c++) {
            float sym_val = 0.5f * (mat->pData[r * mat->numCols + c] +
                                    mat->pData[c * mat->numCols + r]);
            mat->pData[r * mat->numCols + c] = sym_val;
            mat->pData[c * mat->numCols + r] = sym_val;
        }
    }

    return true;
}

bool mathMatrixTranspose(const mathMatrix_t *mat, mathMatrix_t *mat_out)
{
    if (mat == NULL || mat->pData == NULL || mat_out == NULL || mat_out->pData == NULL) {
        return false;
    }

    if (mat_out->numRows != mat->numCols || mat_out->numCols != mat->numRows) {
        return false;
    }

    for (uint16_t r = 0; r < mat->numRows; r++) {
        for (uint16_t c = 0; c < mat->numCols; c++) {
            mat_out->pData[c * mat_out->numCols + r] = mat->pData[r * mat->numCols + c];
        }
    }

    return true;
}

bool mathMatrixSetDiagScalar(mathMatrix_t *mat, float value)
{
    if (mat == NULL || mat->pData == NULL) {
        return false;
    }

    if (mat->numRows != mat->numCols) {
        return false;
    }

    uint16_t diag_len;

    if (mathMatrixSetZero(mat) == false) {
        return false;
    }

    diag_len = mat->numRows;
    for (uint16_t i = 0; i < diag_len; i++) {
        mat->pData[mat->numCols * i + i] = value;
    }

    return true;
}

bool mathMatrixSetDiag(mathMatrix_t *mat, const float *diag, uint16_t diag_len)
{
    if (mat == NULL || mat->pData == NULL || diag == NULL) {
        return false;
    }

    if (mat->numRows != mat->numCols) {
        return false;
    }

    uint16_t mat_diag_len;

    mat_diag_len = mat->numRows;
    if (diag_len != mat_diag_len) {
        return false;
    }

    if (mathMatrixSetZero(mat) == false) {
        return false;
    }

    for (uint16_t i = 0; i < diag_len; i++) {
        mat->pData[mat->numCols * i + i] = diag[i];
    }

    return true;
}

// 行列row_start col_start从0开始,block_rows block_cols表示要操作的block的总行数和总列数,可以为0，均为0时表示空矩阵
bool mathMatrixSetBlock(mathMatrix_t *mat, uint16_t row_start, uint16_t col_start, uint16_t block_rows, uint16_t block_cols, const float *block_data)
{
    if (mat == NULL || mat->pData == NULL || block_data == NULL) {
        return false;
    }

    if (row_start >= mat->numRows || col_start >= mat->numCols) {
        return false;
    }

    if (block_rows == 0U || block_cols == 0U) {
        return true;
    }

    if ((uint32_t)row_start + (uint32_t)block_rows > mat->numRows ||
        (uint32_t)col_start + (uint32_t)block_cols > mat->numCols) {
        return false;
    }

    for (uint16_t r = 0; r < block_rows; r++) {
        for (uint16_t c = 0; c < block_cols; c++) {
            mat->pData[(row_start + r) * mat->numCols + (col_start + c)] =
                block_data[r * block_cols + c];
        }
    }

    return true;
}

bool mathMatrixGetBlockMatrix(const mathMatrix_t *mat, uint16_t row_start, uint16_t col_start, mathMatrix_t *block_out)
{
    if (mat == NULL || mat->pData == NULL || block_out == NULL || block_out->pData == NULL) {
        return false;
    }

    if (row_start >= mat->numRows || col_start >= mat->numCols) {
        return false;
    }

    uint16_t block_rows = block_out->numRows;
    uint16_t block_cols = block_out->numCols;

    if (block_rows == 0U || block_cols == 0U) {
        return true;
    }

    if ((uint32_t)row_start + (uint32_t)block_rows > mat->numRows ||
        (uint32_t)col_start + (uint32_t)block_cols > mat->numCols) {
        return false;
    }

    for (uint16_t r = 0; r < block_rows; r++) {
        for (uint16_t c = 0; c < block_cols; c++) {
            block_out->pData[r * block_cols + c] =
                mat->pData[(row_start + r) * mat->numCols + (col_start + c)];
        }
    }

    return true;
}

bool mathMatrixSetBlockIdentity(mathMatrix_t *mat, uint16_t row_start, uint16_t col_start, uint16_t block_rows, uint16_t block_cols)
{
    if (mat == NULL || mat->pData == NULL) {
        return false;
    }

    if (block_rows != block_cols) {
        return false;
    }

    if (row_start >= mat->numRows || col_start >= mat->numCols) {
        return false;
    }

    if (block_rows == 0U) {
        return true;
    }

    if ((uint32_t)row_start + (uint32_t)block_rows > mat->numRows ||
        (uint32_t)col_start + (uint32_t)block_cols > mat->numCols) {
        return false;
    }

    for (uint16_t r = 0; r < block_rows; r++) {
        for (uint16_t c = 0; c < block_cols; c++) {
            mat->pData[(row_start + r) * mat->numCols + (col_start + c)] = (r == c) ? 1.0f : 0.0f;
        }
    }

    return true;
}

bool mathMatrixSetBlockDiagScalar(mathMatrix_t *mat, uint16_t row_start, uint16_t col_start, uint16_t block_rows, uint16_t block_cols, float value)
{
    if (mat == NULL || mat->pData == NULL) {
        return false;
    }

    if (block_rows != block_cols) {
        return false;
    }

    if (row_start >= mat->numRows || col_start >= mat->numCols) {
        return false;
    }

    if (block_rows == 0U) {
        return true;
    }

    if ((uint32_t)row_start + (uint32_t)block_rows > mat->numRows ||
        (uint32_t)col_start + (uint32_t)block_cols > mat->numCols) {
        return false;
    }

    for (uint16_t r = 0; r < block_rows; r++) {
        for (uint16_t c = 0; c < block_cols; c++) {
            mat->pData[(row_start + r) * mat->numCols + (col_start + c)] = (r == c) ? value : 0.0f;
        }
    }

    return true;
}

bool mathMatrixSetBlockByMatrix(mathMatrix_t *mat, uint16_t row_start, uint16_t col_start, const mathMatrix_t *block_mat)
{
    if (mat == NULL || mat->pData == NULL || block_mat == NULL || block_mat->pData == NULL) {
        return false;
    }

    return mathMatrixSetBlock(mat,
                              row_start,
                              col_start,
                              block_mat->numRows,
                              block_mat->numCols,
                              block_mat->pData);
}

bool mathMatrixGetBlock(const mathMatrix_t *mat, uint16_t row_start, uint16_t col_start, uint16_t block_rows, uint16_t block_cols, float *block_data)
{
    if (mat == NULL || mat->pData == NULL || block_data == NULL) {
        return false;
    }

    if (row_start >= mat->numRows || col_start >= mat->numCols) {
        return false;
    }

    if (block_rows == 0U || block_cols == 0U) {
        return true;
    }

    if ((uint32_t)row_start + (uint32_t)block_rows > mat->numRows ||
        (uint32_t)col_start + (uint32_t)block_cols > mat->numCols) {
        return false;
    }

    for (uint16_t r = 0; r < block_rows; r++) {
        for (uint16_t c = 0; c < block_cols; c++) {
            block_data[r * block_cols + c] =
                mat->pData[(row_start + r) * mat->numCols + (col_start + c)];
        }
    }

    return true;
}
