/**
 * @file
 * @author David Barina <ibarina@fit.vutbr.cz>
 * @brief Fast wavelet transform implemented via lifting scheme.
 */
#ifndef LIBDWT_H
#define LIBDWT_H

#include <stdint.h> // int64_t
#include <stdio.h> // FILE
#include <stdarg.h> // va_list

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup c_dwt C interface
 * @{
 **/

/**
 * @brief Lifting implementation of one level of fast wavelet transform using CDF 9/7 wavelet.
 *
 * This function works with double precision floating point numbers (i.e. double data type).
 *
 * @deprecated Use @ref dwt_cdf97_f_ex_d instead.
 */
void dwt_cdf97_f_d(
	const double *src,	///< input signal of the length @e N
	double *dst,		///< output signal of the length @e N, i.e. one level of discrete wavelet transform, the L and H channels are concatenated
	double *tmp,		///< temporary memory space of the length @e N
	int N			///< length of the signal, odd or even length
);

/**
 * @brief Lifting implementation of one level of fast wavelet transform using CDF 5/3 wavelet.
 *
 * This function works with double precision floating point numbers (i.e. double data type).
 *
 * @deprecated Use @ref dwt_cdf53_f_ex_d instead.
 */
void dwt_cdf53_f_d(
	const double *src,	///< input signal of the length @e N
	double *dst,		///< output signal of the length @e N, i.e. one level of discrete wavelet transform, the L and H channels are concatenated
	double *tmp,		///< temporary memory space of the length @e N
	int N			///< length of the signal, odd or even length
);

/**
 * @brief Lifting implementation of one level of fast wavelet transform using CDF 9/7 wavelet.
 *
 * This function works with single precision floating point numbers (i.e. float data type).
 *
 * @deprecated Use @ref dwt_cdf97_f_ex_s instead.
 */
void dwt_cdf97_f_s(
	const float *src,	///< input signal of the length @e N
	float *dst,		///< output signal of the length @e N, i.e. one level of discrete wavelet transform, the L and H channels are concatenated
	float *tmp,		///< temporary memory space of the length @e N
	int N			///< length of the signal, odd or even length
);

/**
 * @brief Lifting implementation of one level of fast wavelet transform using CDF 5/3 wavelet.
 *
 * This function works with single precision floating point numbers (i.e. float data type).
 *
 * @deprecated Use @ref dwt_cdf53_f_ex_s instead.
 */
void dwt_cdf53_f_s(
	const float *src,	///< input signal of the length @e N
	float *dst,		///< output signal of the length @e N, i.e. one level of discrete wavelet transform, the L and H channels are concatenated
	float *tmp,		///< temporary memory space of the length @e N
	int N			///< length of the signal, odd or even length
);

/**
 * @brief Lifting implementation of one level of fast inverse wavelet transform using CDF 9/7 wavelet.
 *
 * This function works with double precision floating point numbers (i.e. double data type).
 *
 * @deprecated Use @ref dwt_cdf97_i_ex_d instead.
 */
void dwt_cdf97_i_d(
	const double *src,	///< input signal of the length @e N, i.e. the discrete wavelet transform
	double *dst,		///< output signal of the length @e N, i.e. the reconstructed signal
	double *tmp,		///< temporary memory space of the length @e N
	int N			///< length of the signal, odd or even length
);

/**
 * @brief Lifting implementation of one level of fast inverse wavelet transform using CDF 5/3 wavelet.
 *
 * This function works with double precision floating point numbers (i.e. double data type).
 *
 * @deprecated Use @ref dwt_cdf53_i_ex_d instead.
 */
void dwt_cdf53_i_d(
	const double *src,	///< input signal of the length @e N, i.e. the discrete wavelet transform
	double *dst,		///< output signal of the length @e N, i.e. the reconstructed signal
	double *tmp,		///< temporary memory space of the length @e N
	int N			///< length of the signal, odd or even length
);

/**
 * @brief Lifting implementation of one level of fast inverse wavelet transform using CDF 9/7 wavelet.
 *
 * This function works with single precision floating point numbers (i.e. float data type).
 *
 * @deprecated Use @ref dwt_cdf97_i_ex_s instead.
 */
void dwt_cdf97_i_s(
	const float *src,	///< input signal of the length @e N, i.e. the discrete wavelet transform
	float *dst,		///< output signal of the length @e N, i.e. the reconstructed signal
	float *tmp,		///< temporary memory space of the length @e N
	int N			///< length of the signal, odd or even length
);

/**
 * @brief Lifting implementation of one level of fast inverse wavelet transform using CDF 5/3 wavelet.
 *
 * This function works with single precision floating point numbers (i.e. float data type).
 *
 * @deprecated Use @ref dwt_cdf53_i_ex_s instead.
 */
void dwt_cdf53_i_s(
	const float *src,	///< input signal of the length @e N, i.e. the discrete wavelet transform
	float *dst,		///< output signal of the length @e N, i.e. the reconstructed signal
	float *tmp,		///< temporary memory space of the length @e N
	int N			///< length of the signal, odd or even length
);

/**
 * @brief Lifting implementation of one level of fast wavelet transform using CDF 9/7 wavelet.
 *
 * This function works with double precision floating point numbers (i.e. double data type).
 */
void dwt_cdf97_f_ex_d(
	const double *src,	///< input signal of the length @e N
	double *dst_l,		///< output L (low pass) channel of length @f$ \lfloor N/2 \rfloor @f$ for even @e N or @f$ \lfloor N/2 \rfloor + 1 @f$ for odd @e N
	double *dst_h,		///< output H (high pass) channel of length @f$ \lfloor N/2 \rfloor @f$
	double *tmp,		///< temporary memory space of the length @e N
	int N			///< length of the input signal, odd or even length
);

/**
 * @brief Lifting implementation of one level of fast wavelet transform using CDF 5/3 wavelet.
 *
 * This function works with double precision floating point numbers (i.e. double data type).
 */
void dwt_cdf53_f_ex_d(
	const double *src,	///< input signal of the length @e N
	double *dst_l,		///< output L (low pass) channel of length @f$ \lfloor N/2 \rfloor @f$ for even @e N or @f$ \lfloor N/2 \rfloor + 1 @f$ for odd @e N
	double *dst_h,		///< output H (high pass) channel of length @f$ \lfloor N/2 \rfloor @f$
	double *tmp,		///< temporary memory space of the length @e N
	int N			///< length of the input signal, odd or even length
);

/**
 * @brief Lifting implementation of one level of fast wavelet transform using CDF 9/7 wavelet.
 *
 * This function works with single precision floating point numbers (i.e. float data type).
 */
void dwt_cdf97_f_ex_s(
	const float *src,	///< input signal of the length @e N
	float *dst_l,		///< output L (low pass) channel of length @f$ \lfloor N/2 \rfloor @f$ for even @e N or @f$ \lfloor N/2 \rfloor + 1 @f$ for odd @e N
	float *dst_h,		///< output H (high pass) channel of length @f$ \lfloor N/2 \rfloor @f$
	float *tmp,		///< temporary memory space of the length @e N
	int N			///< length of the input signal, odd or even length
);

/**
 * @brief Lifting implementation of one level of fast wavelet transform using CDF 5/3 wavelet.
 *
 * This function works with single precision floating point numbers (i.e. float data type).
 */
void dwt_cdf53_f_ex_s(
	const float *src,	///< input signal of the length @e N
	float *dst_l,		///< output L (low pass) channel of length @f$ \lfloor N/2 \rfloor @f$ for even @e N or @f$ \lfloor N/2 \rfloor + 1 @f$ for odd @e N
	float *dst_h,		///< output H (high pass) channel of length @f$ \lfloor N/2 \rfloor @f$
	float *tmp,		///< temporary memory space of the length @e N
	int N			///< length of the input signal, odd or even length
);

/**
 * @brief Lifting implementation of one level of fast wavelet transform using CDF 9/7 wavelet performed on column.
 *
 * This function works with double precision floating point numbers (i.e. double data type).
 */
void dwt_cdf97_f_ex_stride_d(
	const double *src,	///< input signal of the length @e N
	double *dst_l,		///< output L (low pass) channel of length @f$ \lfloor N/2 \rfloor @f$ for even @e N or @f$ \lfloor N/2 \rfloor + 1 @f$ for odd @e N
	double *dst_h,		///< output H (high pass) channel of length @f$ \lfloor N/2 \rfloor @f$
	double *tmp,		///< temporary memory space of the length @e N
	int N,			///< length of the input signal, odd or even length
	int stride		///< image stride, i.e. the number of bytes between two neighboring pixels, no matter if in the row or column, common for @e src, @e dst_l and @e dst_h
);

/**
 * @brief Lifting implementation of one level of fast wavelet transform using CDF 5/3 wavelet performed on column.
 *
 * This function works with double precision floating point numbers (i.e. double data type).
 */
void dwt_cdf53_f_ex_stride_d(
	const double *src,	///< input signal of the length @e N
	double *dst_l,		///< output L (low pass) channel of length @f$ \lfloor N/2 \rfloor @f$ for even @e N or @f$ \lfloor N/2 \rfloor + 1 @f$ for odd @e N
	double *dst_h,		///< output H (high pass) channel of length @f$ \lfloor N/2 \rfloor @f$
	double *tmp,		///< temporary memory space of the length @e N
	int N,			///< length of the input signal, odd or even length
	int stride		///< image stride, i.e. the number of bytes between two neighboring pixels, no matter if in the row or column, common for @e src, @e dst_l and @e dst_h
);

/**
 * @brief Lifting implementation of one level of fast wavelet transform using CDF 9/7 wavelet performed on column.
 *
 * This function works with single precision floating point numbers (i.e. float data type).
 */
void dwt_cdf97_f_ex_stride_s(
	const float *src,	///< input signal of the length @e N
	float *dst_l,		///< output L (low pass) channel of length @f$ \lfloor N/2 \rfloor @f$ for even @e N or @f$ \lfloor N/2 \rfloor + 1 @f$ for odd @e N
	float *dst_h,		///< output H (high pass) channel of length @f$ \lfloor N/2 \rfloor @f$
	float *tmp,		///< temporary memory space of the length @e N
	int N,			///< length of the input signal, odd or even length
	int stride		///< image stride, i.e. the number of bytes between two neighboring pixels, no matter if in the row or column, common for @e src, @e dst_l and @e dst_h
);

/**
 * @brief Lifting implementation of one level of fast wavelet transform using CDF 5/3 wavelet performed on column.
 *
 * This function works with single precision floating point numbers (i.e. float data type).
 */
void dwt_cdf53_f_ex_stride_s(
	const float *src,	///< input signal of the length @e N
	float *dst_l,		///< output L (low pass) channel of length @f$ \lfloor N/2 \rfloor @f$ for even @e N or @f$ \lfloor N/2 \rfloor + 1 @f$ for odd @e N
	float *dst_h,		///< output H (high pass) channel of length @f$ \lfloor N/2 \rfloor @f$
	float *tmp,		///< temporary memory space of the length @e N
	int N,			///< length of the input signal, odd or even length
	int stride		///< image stride, i.e. the number of bytes between two neighboring pixels, no matter if in the row or column, common for @e src, @e dst_l and @e dst_h
);

/**
 * @brief Lifting implementation of one level of fast inverse wavelet transform using CDF 9/7 wavelet.
 *
 * This function works with double precision floating point numbers (i.e. double data type).
 */
void dwt_cdf97_i_ex_d(
	const double *src_l,	///< input L (low pass) channel of length @f$ \lfloor N/2 \rfloor @f$ for even @e N or @f$ \lfloor N/2 \rfloor + 1 @f$ for odd @e N
	const double *src_h,	///< input H (high pass) channel of length @f$ \lfloor N/2 \rfloor @f$
	double *dst,		///< reconstructed (output) signal
	double *tmp,		///< temporary memory space of the length @e N
	int N			///< length of the reconstructed (output) signal, odd or even length
);

/**
 * @brief Lifting implementation of one level of fast inverse wavelet transform using CDF 5/3 wavelet.
 *
 * This function works with double precision floating point numbers (i.e. double data type).
 */
void dwt_cdf53_i_ex_d(
	const double *src_l,	///< input L (low pass) channel of length @f$ \lfloor N/2 \rfloor @f$ for even @e N or @f$ \lfloor N/2 \rfloor + 1 @f$ for odd @e N
	const double *src_h,	///< input H (high pass) channel of length @f$ \lfloor N/2 \rfloor @f$
	double *dst,		///< reconstructed (output) signal
	double *tmp,		///< temporary memory space of the length @e N
	int N			///< length of the reconstructed (output) signal, odd or even length
);

/**
 * @brief Lifting implementation of one level of fast inverse wavelet transform using CDF 9/7 wavelet.
 *
 * This function works with single precision floating point numbers (i.e. float data type).
 */
void dwt_cdf97_i_ex_s(
	const float *src_l,	///< input L (low pass) channel of length @f$ \lfloor N/2 \rfloor @f$ for even @e N or @f$ \lfloor N/2 \rfloor + 1 @f$ for odd @e N
	const float *src_h,	///< input H (high pass) channel of length @f$ \lfloor N/2 \rfloor @f$
	float *dst,		///< reconstructed (output) signal
	float *tmp,		///< temporary memory space of the length @e N
	int N			///< length of the reconstructed (output) signal, odd or even length
);

/**
 * @brief Lifting implementation of one level of fast inverse wavelet transform using CDF 5/3 wavelet.
 *
 * This function works with single precision floating point numbers (i.e. float data type).
 */
void dwt_cdf53_i_ex_s(
	const float *src_l,	///< input L (low pass) channel of length @f$ \lfloor N/2 \rfloor @f$ for even @e N or @f$ \lfloor N/2 \rfloor + 1 @f$ for odd @e N
	const float *src_h,	///< input H (high pass) channel of length @f$ \lfloor N/2 \rfloor @f$
	float *dst,		///< reconstructed (output) signal
	float *tmp,		///< temporary memory space of the length @e N
	int N			///< length of the reconstructed (output) signal, odd or even length
);

/**
 * @brief Lifting implementation of one level of fast inverse wavelet transform using CDF 9/7 wavelet.
 *
 * This function works with double precision floating point numbers (i.e. double data type).
 */
void dwt_cdf97_i_ex_stride_d(
	const double *src_l,	///< input L (low pass) channel of length @f$ \lfloor N/2 \rfloor @f$ for even @e N or @f$ \lfloor N/2 \rfloor + 1 @f$ for odd @e N
	const double *src_h,	///< input H (high pass) channel of length @f$ \lfloor N/2 \rfloor @f$
	double *dst,		///< reconstructed (output) signal
	double *tmp,		///< temporary memory space of the length @e N
	int N,			///< length of the reconstructed (output) signal, odd or even length
	int stride		///< image stride, i.e. the number of bytes from one row of pixels to the next row of pixels, common for @e dst, @e src_l and @e src_h
);

/**
 * @brief Lifting implementation of one level of fast inverse wavelet transform using CDF 5/3 wavelet.
 *
 * This function works with double precision floating point numbers (i.e. double data type).
 */
void dwt_cdf53_i_ex_stride_d(
	const double *src_l,	///< input L (low pass) channel of length @f$ \lfloor N/2 \rfloor @f$ for even @e N or @f$ \lfloor N/2 \rfloor + 1 @f$ for odd @e N
	const double *src_h,	///< input H (high pass) channel of length @f$ \lfloor N/2 \rfloor @f$
	double *dst,		///< reconstructed (output) signal
	double *tmp,		///< temporary memory space of the length @e N
	int N,			///< length of the reconstructed (output) signal, odd or even length
	int stride		///< image stride, i.e. the number of bytes from one row of pixels to the next row of pixels, common for @e dst, @e src_l and @e src_h
);

/**
 * @brief Lifting implementation of one level of fast inverse wavelet transform using CDF 9/7 wavelet.
 *
 * This function works with single precision floating point numbers (i.e. float data type).
 */
void dwt_cdf97_i_ex_stride_s(
	const float *src_l,	///< input L (low pass) channel of length @f$ \lfloor N/2 \rfloor @f$ for even @e N or @f$ \lfloor N/2 \rfloor + 1 @f$ for odd @e N
	const float *src_h,	///< input H (high pass) channel of length @f$ \lfloor N/2 \rfloor @f$
	float *dst,		///< reconstructed (output) signal
	float *tmp,		///< temporary memory space of the length @e N
	int N,			///< length of the reconstructed (output) signal, odd or even length
	int stride		///< image stride, i.e. the number of bytes from one row of pixels to the next row of pixels, common for @e dst, @e src_l and @e src_h
);

/**
 * @brief Lifting implementation of one level of fast inverse wavelet transform using CDF 5/3 wavelet.
 *
 * This function works with single precision floating point numbers (i.e. float data type).
 */
void dwt_cdf53_i_ex_stride_s(
	const float *src_l,	///< input L (low pass) channel of length @f$ \lfloor N/2 \rfloor @f$ for even @e N or @f$ \lfloor N/2 \rfloor + 1 @f$ for odd @e N
	const float *src_h,	///< input H (high pass) channel of length @f$ \lfloor N/2 \rfloor @f$
	float *dst,		///< reconstructed (output) signal
	float *tmp,		///< temporary memory space of the length @e N
	int N,			///< length of the reconstructed (output) signal, odd or even length
	int stride		///< image stride, i.e. the number of bytes from one row of pixels to the next row of pixels, common for @e dst, @e src_l and @e src_h
);

/**
 * @brief Fill padding in L and H signals with zeros. Useful for nice looking output. Suitable for decomposition.
 *
 * This function works with double precision floating point numbers (i.e. double data type).
 */
void dwt_zero_padding_f_d(
	double *dst_l,		///< L (low pass) channel which will be padded with zeros
	double *dst_h,		///< H (high pass) channel which will be padded with zeros
	int N,			///< length of a signal used for decomposition into the L and H channels, odd or even length
	int N_dst_L,		///< length of the L channel
	int N_dst_H		///< length of the H channel
);

/**
 * @brief Fill padding in L and H signals with zeros. Useful for nice looking output. Suitable for decomposition.
 *
 * This function works with single precision floating point numbers (i.e. float data type).
 */
void dwt_zero_padding_f_s(
	float *dst_l,		///< L (low pass) channel which will be padded with zeros
	float *dst_h,		///< H (high pass) channel which will be padded with zeros
	int N,			///< length of a signal used for decomposition into the L and H channels, odd or even length
	int N_dst_L,		///< length of the L channel
	int N_dst_H		///< length of the H channel
);

/**
 * @brief Fill padding in L and H signals with zeros. Useful for nice looking output. Suitable for decomposition.
 *
 * This function works with double precision floating point numbers (i.e. double data type).
 */
void dwt_zero_padding_f_stride_d(
	double *dst_l,		///< L (low pass) channel which will be padded with zeros
	double *dst_h,		///< H (high pass) channel which will be padded with zeros
	int N,			///< length of a signal used for decomposition into the L and H channels, odd or even length
	int N_dst_L,		///< length of the L channel
	int N_dst_H,		///< length of the H channel
	int stride		///< image stride, i.e. the number of bytes from one row of pixels to the next row of pixels, common for @e dst_l and @e dst_h
);

/**
 * @brief Fill padding in L and H signals with zeros. Useful for nice looking output. Suitable for decomposition.
 *
 * This function works with single precision floating point numbers (i.e. float data type).
 */
void dwt_zero_padding_f_stride_s(
	float *dst_l,		///< L (low pass) channel which will be padded with zeros
	float *dst_h,		///< H (high pass) channel which will be padded with zeros
	int N,			///< length of a signal used for decomposition into the L and H channels, odd or even length
	int N_dst_L,		///< length of the L channel
	int N_dst_H,		///< length of the H channel
	int stride		///< image stride, i.e. the number of bytes from one row of pixels to the next row of pixels, common for @e dst_l and @e dst_h
);

/**
 * @brief Fill padding in L signal with zeros. Useful for nice looking output. Suitable for reconstruction.
 *
 * This function works with double precision floating point numbers (i.e. double data type).
 */
void dwt_zero_padding_i_d(
	double *dst_l,		///< L (low pass) channel which will be padded with zeros
	int N,			///< length of composed L (actual usage)
	int N_dst		///< total length of the L channel
);

/**
 * @brief Fill padding in L signal with zeros. Useful for nice looking output. Suitable for reconstruction.
 *
 * This function works with single precision floating point numbers (i.e. float data type).
 */
void dwt_zero_padding_i_s(
	float *dst_l,		///< L (low pass) channel which will be padded with zeros
	int N,			///< length of composed L (actual usage)
	int N_dst		///< total length of the L channel
);

/**
 * @brief Fill padding in L signal with zeros. Useful for nice looking output. Suitable for reconstruction.
 *
 * This function works with double precision floating point numbers (i.e. double data type).
 */
void dwt_zero_padding_i_stride_d(
	double *dst_l,		///< L (low pass) channel which will be padded with zeros
	int N,			///< length of composed L (actual usage)
	int N_dst,		///< total length of the L channel
	int stride		///< image stride, i.e. the number of bytes from one row of pixels to the next row of pixels
);

/**
 * @brief Fill padding in L signal with zeros. Useful for nice looking output. Suitable for reconstruction.
 *
 * This function works with single precision floating point numbers (i.e. float data type).
 */
void dwt_zero_padding_i_stride_s(
	float *dst_l,		///< L (low pass) channel which will be padded with zeros
	int N,			///< length of composed L (actual usage)
	int N_dst,		///< total length of the L channel
	int stride		///< image stride, i.e. the number of bytes from one row of pixels to the next row of pixels
);

/**
 * @brief Forward image fast wavelet transform using CDF 9/7 wavelet and lifting scheme, in-place version.
 *
 * This function works with double precision floating point numbers (i.e. double data type).
 */
void dwt_cdf97_2f_d(
	void *ptr,		///< pointer to beginning of image data
	int stride_x,		///< difference between rows (in bytes)
	int stride_y,		///< difference between columns (in bytes)
	int size_o_big_x,	///< width of outer image frame (in elements)
	int size_o_big_y,	///< height of outer image frame (in elements)
	int size_i_big_x,	///< width of nested image (in elements)
	int size_i_big_y,	///< height of nested image (in elements)
	int *j_max_ptr,		///< pointer to the number of intended decomposition levels (scales), the number of achieved decomposition levels will be stored also here
	int decompose_one,	///< should be row or column of size one pixel decomposed? zero value if not
	int zero_padding	///< fill padding in channels with zeros? zero value if not, should be non zero only for sparse decomposition
);

/**
 * @brief Forward image fast wavelet transform using CDF 5/3 wavelet and lifting scheme, in-place version.
 *
 * This function works with double precision floating point numbers (i.e. double data type).
 */
void dwt_cdf53_2f_d(
	void *ptr,		///< pointer to beginning of image data
	int stride_x,		///< difference between rows (in bytes)
	int stride_y,		///< difference between columns (in bytes)
	int size_o_big_x,	///< width of outer image frame (in elements)
	int size_o_big_y,	///< height of outer image frame (in elements)
	int size_i_big_x,	///< width of nested image (in elements)
	int size_i_big_y,	///< height of nested image (in elements)
	int *j_max_ptr,		///< pointer to the number of intended decomposition levels (scales), the number of achieved decomposition levels will be stored also here
	int decompose_one,	///< should be row or column of size one pixel decomposed? zero value if not
	int zero_padding	///< fill padding in channels with zeros? zero value if not, should be non zero only for sparse decomposition
);

/**
 * @brief Forward image fast wavelet transform using CDF 9/7 wavelet and lifting scheme, in-place version.
 *
 * This function works with single precision floating point numbers (i.e. float data type).
 */
void dwt_cdf97_2f_s(
	void *ptr,		///< pointer to beginning of image data
	int stride_x,		///< difference between rows (in bytes)
	int stride_y,		///< difference between columns (in bytes)
	int size_o_big_x,	///< width of outer image frame (in elements)
	int size_o_big_y,	///< height of outer image frame (in elements)
	int size_i_big_x,	///< width of nested image (in elements)
	int size_i_big_y,	///< height of nested image (in elements)
	int *j_max_ptr,		///< pointer to the number of intended decomposition levels (scales), the number of achieved decomposition levels will be stored also here
	int decompose_one,	///< should be row or column of size one pixel decomposed? zero value if not
	int zero_padding	///< fill padding in channels with zeros? zero value if not, should be non zero only for sparse decomposition
);

/**
 * @brief Forward image fast wavelet transform using CDF 5/3 wavelet and lifting scheme, in-place version.
 *
 * This function works with single precision floating point numbers (i.e. float data type).
 */
void dwt_cdf53_2f_s(
	void *ptr,		///< pointer to beginning of image data
	int stride_x,		///< difference between rows (in bytes)
	int stride_y,		///< difference between columns (in bytes)
	int size_o_big_x,	///< width of outer image frame (in elements)
	int size_o_big_y,	///< height of outer image frame (in elements)
	int size_i_big_x,	///< width of nested image (in elements)
	int size_i_big_y,	///< height of nested image (in elements)
	int *j_max_ptr,		///< pointer to the number of intended decomposition levels (scales), the number of achieved decomposition levels will be stored also here
	int decompose_one,	///< should be row or column of size one pixel decomposed? zero value if not
	int zero_padding	///< fill padding in channels with zeros? zero value if not, should be non zero only for sparse decomposition
);

/**
 * @brief Inverse image fast wavelet transform using CDF 9/7 wavelet and lifting scheme, in-place version.
 *
 * This function works with double precision floating point numbers (i.e. double data type).
 */
void dwt_cdf97_2i_d(
	void *ptr,		///< pointer to beginning of image data
	int stride_x,		///< difference between rows (in bytes)
	int stride_y,		///< difference between columns (in bytes)
	int size_o_big_x,	///< width of outer image frame (in elements)
	int size_o_big_y,	///< height of outer image frame (in elements)
	int size_i_big_x,	///< width of nested image (in elements)
	int size_i_big_y,	///< height of nested image (in elements)
	int j_max,		///< pointer to the number of achieved decomposition levels (scales)
	int decompose_one,	///< should be row or column of size one pixel decomposed? zero value if not
	int zero_padding	///< fill padding in channels with zeros? zero value if not, should be non zero only for sparse decomposition
);

/**
 * @brief Inverse image fast wavelet transform using CDF 5/3 wavelet and lifting scheme, in-place version.
 *
 * This function works with double precision floating point numbers (i.e. double data type).
 */
void dwt_cdf53_2i_d(
	void *ptr,		///< pointer to beginning of image data
	int stride_x,		///< difference between rows (in bytes)
	int stride_y,		///< difference between columns (in bytes)
	int size_o_big_x,	///< width of outer image frame (in elements)
	int size_o_big_y,	///< height of outer image frame (in elements)
	int size_i_big_x,	///< width of nested image (in elements)
	int size_i_big_y,	///< height of nested image (in elements)
	int j_max,		///< pointer to the number of achieved decomposition levels (scales)
	int decompose_one,	///< should be row or column of size one pixel decomposed? zero value if not
	int zero_padding	///< fill padding in channels with zeros? zero value if not, should be non zero only for sparse decomposition
);

/**
 * @brief Inverse image fast wavelet transform using CDF 9/7 wavelet and lifting scheme, in-place version.
 *
 * This function works with single precision floating point numbers (i.e. float data type).
 */
void dwt_cdf97_2i_s(
	void *ptr,		///< pointer to beginning of image data
	int stride_x,		///< difference between rows (in bytes)
	int stride_y,		///< difference between columns (in bytes)
	int size_o_big_x,	///< width of outer image frame (in elements)
	int size_o_big_y,	///< height of outer image frame (in elements)
	int size_i_big_x,	///< width of nested image (in elements)
	int size_i_big_y,	///< height of nested image (in elements)
	int j_max,		///< pointer to the number of achieved decomposition levels (scales)
	int decompose_one,	///< should be row or column of size one pixel decomposed? zero value if not
	int zero_padding	///< fill padding in channels with zeros? zero value if not, should be non zero only for sparse decomposition
);

/**
 * @brief Inverse image fast wavelet transform using CDF 5/3 wavelet and lifting scheme, in-place version.
 *
 * This function works with single precision floating point numbers (i.e. float data type).
 */
void dwt_cdf53_2i_s(
	void *ptr,		///< pointer to beginning of image data
	int stride_x,		///< difference between rows (in bytes)
	int stride_y,		///< difference between columns (in bytes)
	int size_o_big_x,	///< width of outer image frame (in elements)
	int size_o_big_y,	///< height of outer image frame (in elements)
	int size_i_big_x,	///< width of nested image (in elements)
	int size_i_big_y,	///< height of nested image (in elements)
	int j_max,		///< pointer to the number of achieved decomposition levels (scales)
	int decompose_one,	///< should be row or column of size one pixel decomposed? zero value if not
	int zero_padding	///< fill padding in channels with zeros? zero value if not, should be non zero only for sparse decomposition
);

/**
 * @}
 */

/**
 * @defgroup c_dwt_util C utility interface
 * @{
 **/

/**
 * @brief Smallest integer not less than the base 2 logarithm of x, i.e. @f$ \lceil log_2(x) \rceil @f$.
 */
int dwt_util_ceil_log2(
	int x			///< input value
);

/**
 * @brief Power of two using greater or equal to x, i.e. @f$ 2^{\lceil log_2(x) \rceil} @f$.
 */
int dwt_util_pow2_ceil_log2(
	int x			///< input value
);

/**
 * @brief Fill test image.
 *
 * This function works with double precision floating point numbers (i.e. double data type).
 */
void dwt_util_test_image_fill_d(
	void *ptr,		///< pointer to beginning of image data
	int stride_x,		///< difference between rows (in bytes)
	int stride_y,		///< difference between columns (in bytes)
	int size_i_big_x,	///< width of nested image (in elements)
	int size_i_big_y,	///< height of nested image (in elements)
	int rand		///< random seed
);

/**
 * @brief Fill test image.
 *
 * This function works with single precision floating point numbers (i.e. float data type).
 */
void dwt_util_test_image_fill_s(
	void *ptr,		///< pointer to beginning of image data
	int stride_x,		///< difference between rows (in bytes)
	int stride_y,		///< difference between columns (in bytes)
	int size_i_big_x,	///< width of nested image (in elements)
	int size_i_big_y,	///< height of nested image (in elements)
	int rand		///< random seed
);

void dwt_util_test_image_zero_s(
	void *ptr,		///< pointer to beginning of image data
	int stride_x,		///< difference between rows (in bytes)
	int stride_y,		///< difference between columns (in bytes)
	int size_i_big_x,	///< width of nested image (in elements)
	int size_i_big_y	///< height of nested image (in elements)
);

/**
 * @brief Allocate image.
 *
 * Allocates memory for image of given sizes.
 */
void dwt_util_alloc_image(
	void **pptr,		///< place pointer to newly allocated data here
	int stride_x,		///< difference between rows (in bytes)
	int stride_y,		///< difference between columns (in bytes)
	int size_o_big_x,	///< width of outer image frame (in elements)
	int size_o_big_y	///< height of outer image frame (in elements)
);

/**
 * @brief Free image.
 *
 * Frees memory allocated by @ref dwt_util_alloc_image.
 */
void dwt_util_free_image(
	void **pptr		///< pointer to data that will be released
);

/**
 * @brief Compare two images.
 *
 * This function compares two images and returns zero value if they equal.
 * This function works with double precision floating point numbers (i.e. double data type).
 */
int dwt_util_compare_d(
	void *ptr1,		///< pointer to data of first image
	void *ptr2,		///< pointer to data of second image
	int stride_x,		///< difference between rows (in bytes)
	int stride_y,		///< difference between columns (in bytes)
	int size_i_big_x,	///< width of nested image (in elements)
	int size_i_big_y	///< height of nested image (in elements)
);

/**
 * @brief Compare two images.
 *
 * This function compares two images and returns zero value if they equal.
 * This function works with single precision floating point numbers (i.e. float data type).
 */
int dwt_util_compare_s(
	void *ptr1,		///< pointer to data of first image
	void *ptr2,		///< pointer to data of second image
	int stride_x,		///< difference between rows (in bytes)
	int stride_y,		///< difference between columns (in bytes)
	int size_i_big_x,	///< width of nested image (in elements)
	int size_i_big_y	///< height of nested image (in elements)
);

/**
 * @brief Returns library version string.
 */
const char *dwt_util_version();

/**
 * @brief Architecture we are running on.
 */
const char *dwt_util_arch();

/**
 * @brief Timer types.
 *
 * Timer sources used in @ref dwt_util_clock_available, @ref dwt_util_get_frequency and @ref dwt_util_get_clock functions.
 */
enum dwt_timer_types
{
	DWT_TIME_CLOCK_GETTIME,				///< use clock_gettime() function from <time.h>; defined by POSIX; with appropriate argument
	DWT_TIME_CLOCK_GETTIME_REALTIME,		///< use clock_gettime() function from <time.h>; defined by POSIX; with argument CLOCK_REALTIME
	DWT_TIME_CLOCK_GETTIME_MONOTONIC,		///< use clock_gettime() function from <time.h>; defined by POSIX; with argument CLOCK_MONOTONIC
	DWT_TIME_CLOCK_GETTIME_MONOTONIC_RAW,		///< use clock_gettime() function from <time.h>; defined by POSIX; with argument CLOCK_MONOTONIC_RAW
	DWT_TIME_CLOCK_GETTIME_PROCESS_CPUTIME_ID,	///< use clock_gettime() function from <time.h>; defined by POSIX; with argument CLOCK_PROCESS_CPUTIME_ID
	DWT_TIME_CLOCK_GETTIME_THREAD_CPUTIME_ID,	///< use clock_gettime() function from <time.h>; defined by POSIX; with argument CLOCK_THREAD_CPUTIME_ID
	DWT_TIME_CLOCK,					///< use clock() function from <time.h>; defined since C89 and by POSIX
	DWT_TIME_TIMES,					///< use times() function from <sys/times.h>; defined by POSIX; only user time is considered
	DWT_TIME_GETRUSAGE,				///< use getrusage() function from <sys/resource.h>; defined by POSIX; with appropriate argument
	DWT_TIME_GETRUSAGE_SELF,			///< use getrusage() function from <sys/resource.h>; defined by POSIX; with argument RUSAGE_SELF
	DWT_TIME_GETRUSAGE_CHILDREN,			///< use getrusage() function from <sys/resource.h>; defined by POSIX; with argument RUSAGE_CHILDREN
	DWT_TIME_GETRUSAGE_THREAD,			///< use getrusage() function from <sys/resource.h>; defined by POSIX; with argument RUSAGE_THREAD
	DWT_TIME_GETTIMEOFDAY,				///< use gettimeofday() function from <sys/time.h>; defined by POSIX
	DWT_TIME_IOCTL_RTC,				///< use ioctl() function from <sys/ioctl.h>; with argument RTC_RD_TIME; available on Linux except EdkDSP platform
	DWT_TIME_AUTOSELECT				///< autoselect appropriate timer
};

/**
 * @brief Indicate if the given clock type is available.
 *
 * @returns Return 0 if the clock @p type is available, or -1 if is not.
 */
int dwt_util_clock_available(
	int type		///< timer type, see @ref dwt_timer_types
);

/**
 * @brief Integer type for storing time or number of clock ticks.
 *
 * This type is used by @ref dwt_util_get_frequency and @ref dwt_util_get_clock functions.
 */
typedef int64_t dwt_clock_t;

/**
 * @brief Autoselect appropriate timer.
 *
 * Try to select appropriate timer according to system dispositions.
 */
int dwt_util_clock_autoselect();

/**
 * @brief Number of ticks per second.
 *
 * This function returns a number of clock ticks per second according to indicated clock source type.
 */
dwt_clock_t dwt_util_get_frequency(
	int type		///< timer type, see @ref dwt_timer_types
);

/**
 * @brief Number of ticks from certain event.
 *
 * This function returns a number of clock ticks from some event (e.g. system boot). Used clock source is indicated by @p type argument.
 *
 * Example usage:
 * @code
 * int type = dwt_util_clock_autoselect();
 *
 * dwt_clock_t start = dwt_util_get_clock(type);
 * // some computation...
 * dwt_clock_t stop = dwt_util_get_clock(type);
 *
 * double elapsed_time_in_seconds = (stop - start)/(double)dwt_util_get_frequency(type);
 * @endcode
 */
dwt_clock_t dwt_util_get_clock(
	int type		///< timer type, see @ref dwt_timer_types
);

/**
 * @brief Wrapper to @p omp_get_max_threads function.
 *
 * Returns the maximum number of threads what can be used in parallel region.
 *
 * @warning experimental
 */
int dwt_util_get_max_threads();

/**
 * @brief Get the maximum number of workers available.
 */
int dwt_util_get_max_workers();

/**
 * @brief Wrapper to @p omp_set_num_threads function.
 *
 * Sets the number of threads that will be used in parallel region.
 *
 * @warning experimental
 */
void dwt_util_set_num_threads(
	int num_threads		///< the number of threads
);

/**
 * @brief Set the number of active workers.
 */
void dwt_util_set_num_workers(
	int num_workers		///< the number of workers
);

/**
 * @brief Set algorithm for acceleration of lifting scheme.
 *
 * On all platforms, select from one several loop algorithms.
 * On UTIA ASVP/EdkDSP platform, enable block-acceleration using workers.
 * On x86 architecture, enable SIMD acceleration using SSE instruction set.
 *
 * @param[in] accel_type Means
 *   @li 0 for CPU multi-loop implementation,
 *   @li 1 for BCE implementation if available (ASVP platform),
 *   @li 2 for empty implementation (for performance measurement),
 *   @li 3 for test BCE implementation on whole data vector (ASVP platform),
 *   @li 4 for CPU double-loop algorithm,
 *   @li 5 for CPU shifted double-loop SIMD algorithm (reference implementation),
 *   @li 6 for CPU shifted double-loop SIMD algorithm (2 iterations merged),
 *   @li 7 for CPU shifted double-loop SIMD algorithm (6 iterations merged),
 *   @li 8 for CPU shifted double-loop SIMD algorithm (2 iterations merged, SSE implementation, x86 platform),
 *   @li 9 for CPU shifted double-loop SIMD algorithm (6 iterations merged, SSE implementation, x86 platform).
 *
 * @note This function currently affects only single precision floating point number functions.
 * @warning experimental
 */
void dwt_util_set_accel(
	int accel_type);

/**
 * @brief Initialize workers in UTIA ASVP platform.
 */
void dwt_util_init();

/**
 * @brief Release all resources allocated in @ref dwt_util_init function.
 */
void dwt_util_finish();

/**
 * @brief Save grayscale image into PGM file.
 *
 * See <a href="http://netpbm.sourceforge.net/">the home page for Netpbm</a>.
 * This function works with single precision floating point numbers (i.e. float data type).
 *
 * @note Use @ref dwt_util_conv_show_s function before this function call to save a transform.
 */
int dwt_util_save_to_pgm_s(
	const char *filename,	///< target file name, e.g. "output.pgm"
	float max_value, 	///< maximum value of pixel, e.g. 1.0 if image values lie inside an interval [0.0; 1.0]
	const void *ptr,	///< pointer to beginning of image data
	int stride_x,		///< difference between rows (in bytes)
	int stride_y,		///< difference between columns (in bytes)
	int size_i_big_x,	///< width of nested image (in elements)
	int size_i_big_y	///< height of nested image (in elements)
);

/**
 * @brief Save grayscale image into PGM file.
 *
 * See <a href="http://netpbm.sourceforge.net/">the home page for Netpbm</a>.
 * This function works with double precision floating point numbers (i.e. double data type).
 *
 * @note Use @ref dwt_util_conv_show_d function before this function call to save a transform.
 */
int dwt_util_save_to_pgm_d(
	const char *filename,	///< target file name, e.g. "output.pgm"
	double max_value, 	///< maximum value of pixel, e.g. 1.0 if image values lie inside an interval [0.0; 1.0]
	const void *ptr,	///< pointer to beginning of image data
	int stride_x,		///< difference between rows (in bytes)
	int stride_y,		///< difference between columns (in bytes)
	int size_i_big_x,	///< width of nested image (in elements)
	int size_i_big_y	///< height of nested image (in elements)
);

/**
 * @brief Wrapper to @p omp_get_thread_num function.
 *
 * @warning experimental
 */
int dwt_util_get_thread_num();

/**
 * @brief Wrapper to @p omp_get_num_threads function.
 *
 * @warning experimental
 */
int dwt_util_get_num_threads();

/**
 * @brief Get number of active workers.
 */
int dwt_util_get_num_workers();

/**
 * @brief Identifier of PicoBlaze operation.
 *
 * @warning experimental
 */
enum dwt_op
{
	DWT_OP_NONE,		///< undefined operation
	DWT_OP_LIFT4SA,		///< CDF 9/7 wavelet, forward transform
	DWT_OP_LIFT4SB		///< CDF 9/7 wavelet, inverse transform
};

/**
 * @brief Set PicoBlaze operation.
 *
 * Function changes active PicoBlaze firmware. This makes sense only on UTIA
 * EdkDSP platform.
 *
 * @warning experimental
 */
void dwt_util_switch_op(
	enum dwt_op op		///< identifier of PicoBlaze operation
);

/**
 * @brief Check the correct function of ASVP (EdkDSP) platform.
 */
void dwt_util_test();

/**
 * @brief Terminate the program.
 */
void dwt_util_abort();

/**
 * @brief Actively waits the specified number of milliseconds.
 */
void dwt_util_wait(
	int ms	///< the number of milliseconds
);

/**
 * @brief Allocate vector of @e size floats.
 *
 * Allocate vector of given size that have to be even. Allocated memory has alignment on 64-bits boundary.
 *
 * @return Pointer to allocated memory that is 64-bits aligned.
 */
float *dwt_util_allocate_vec_s(
	int size	///< the number of elements (floats) to allocate, must be even
);

/**
 * @brief Fill vector of floats with simple sequence. Useful for testing.
 *
 * @return Return non-zero value if an error occurred.
 */
int dwt_util_generate_vec_s(
	float *addr,	///< pointer to the vector
	int size	///< number of vector elements
);

/**
 * @brief Fill vector with zeros.
 *
 * @return Return non-zero value if an error occurred.
 */
int dwt_util_zero_vec_s(
	float *addr,	///< pointer to the vector
	int size	///< number of vector elements
);

/**
 * @brief Copy vector of given size and check if values was transferred correctly.
 *
 * @return Return non-zero value if an error occurred.
 */
int dwt_util_copy_vec_s(
	const float *src,	///< source vector
	float *dst,		///< destination vector
	int size		///< number of vector elements
);

/**
 * @brief Compare two vectors.
 *
 * @return Return non-zero value in case of the two vectors are different.
 */
int dwt_util_cmp_vec_s(
	const float *a,		///< first vector
	const float *b,		///< second vector
	int size		///< number of vectors' elements
);

/**
 * @brief Replacement for @p vfprintf.
 */
int dwt_util_vfprintf(
	FILE *stream,		///< output stream
	const char *format,	///< format with same meaning like in @p printf function
	va_list ap		///< @p va_list encapsulating variable number of arguments
);

/**
 * @brief Replacement for @p vprintf.
 */
int dwt_util_vprintf(
	const char *format,	///< format with same meaning like in @p printf function
	va_list ap		///< @p va_list encapsulating variable number of arguments
);

/**
 * @brief Replacement for @p fprintf.
 */
int dwt_util_fprintf(
	FILE *stream,		///< output stream
	const char *format,	///< format with same meaning like in @p printf function
	...			///< variable number of arguments
);

/**
 * @brief Replacement for @p printf.
 */
int dwt_util_printf(
	const char *format,	///< format with same meaning like in @p printf function
	...			///< variable number of arguments
);

/**
 * Log levels for @ref dwt_util_log function.
 */
enum dwt_util_loglevel {
	LOG_NONE = 0,	///< messages without prefix
	LOG_DBG,	///< debug messages
	LOG_INFO,	///< informational messages
	LOG_WARN,	///< warnings
	LOG_ERR,	///< errors
	LOG_TEST,	///< tests
};

/**
 * @brief Formatted output. Same syntax like @p printf function.
 */
int dwt_util_log(
	enum dwt_util_loglevel level,	///< log level
	const char *format,		///< format string that specifies how subsequent arguments re converted for output
	...				///< the subsequent arguments
);

/**
 * @brief Check if memory is aligned to 64 bits.
 * 
 * @returns Returns 0 when not aligned or 1 when aligned.
 */
int dwt_util_is_aligned_8(
	const void *ptr		///< pointer to the memory
);

/**
 * @brief Get node (machine) name which we are running on.
 * 
 * @returns Returns pointer to string. Do not pass this pointer to @p free. This string can be changed by next @ref dwt_util_node function call.
 */
const char *dwt_util_node();

/**
 * @brief Get program name.
 * 
 * @returns Returns pointer to null-terminated string. Do not pass this pointer to @p free. This string can be changed by next @ref dwt_util_appname function call.
 */
const char *dwt_util_appname();

/**
 * Gets optimal data stride according to cache usage.
 * 
 * @return Returns optimal stride in bytes.
 */
int dwt_util_get_opt_stride(
	int min_stride		///< minimum required stride (in bytes)
);

/**
 * @brief Determine if a number is probable prime.
 * 
 * Currently uses variant of Fermat primality test for base-2.
 * 
 * @return Non-zero value for probable prime, zero otherwise.
 */
int dwt_util_is_prime(
	int N	///< the number to test
);

/**
 * @brief Find smallest probable prime number not less than @e N.
 *
 * @return Return the probable prime found.
 */
int dwt_util_next_prime(
	int N	///< the number
);

enum dwt_subbands {
	DWT_LL,		///< subband filtered by LP filter horizontally and vertically
	DWT_HL,		///< subband filtered by HP horizontally and LP vertically
	DWT_LH,		///< subband filtered by HP vertically and LP horizontally
	DWT_HH		///< subband filtered by HP filter horizontally and vertically
};

/**
 * @brief Gets pointer to and sizes of the selected subband (LL, HL, LH or HH).
 */
void dwt_util_subband_s(
	void *ptr,		///< pointer to beginning of image data
	int stride_x,		///< difference between rows (in bytes)
	int stride_y,		///< difference between columns (in bytes)
	int size_o_big_x,	///< width of outer image frame (in elements)
	int size_o_big_y,	///< height of outer image frame (in elements)
	int size_i_big_x,	///< width of nested image (in elements)
	int size_i_big_y,	///< height of nested image (in elements)
	int j_max,		///< pointer to the decomposition level of interest
	enum dwt_subbands band,	///< subband of interest (LL, HL, LH, HH)
	void **dst_ptr,		///< here will be stored pointer to beginning of subband data
	int *dst_size_x,	///< here will be stored width of subband
	int *dst_size_y		///< here will be stored height of subband
);

/**
 * @brief Gets pointer to and sizes of the selected subband (LL, HL, LH or HH).
 */
void dwt_util_subband_d(
	void *ptr,		///< pointer to beginning of image data
	int stride_x,		///< difference between rows (in bytes)
	int stride_y,		///< difference between columns (in bytes)
	int size_o_big_x,	///< width of outer image frame (in elements)
	int size_o_big_y,	///< height of outer image frame (in elements)
	int size_i_big_x,	///< width of nested image (in elements)
	int size_i_big_y,	///< height of nested image (in elements)
	int j_max,		///< pointer to the decomposition level of interest
	enum dwt_subbands band,	///< subband of interest (LL, HL, LH, HH)
	void **dst_ptr,		///< here will be stored pointer to beginning of subband data
	int *dst_size_x,	///< here will be stored width of subband
	int *dst_size_y		///< here will be stored height of subband
);

/**
 * @brief Gets pointer to and sizes of the selected subband (LL, HL, LH or HH).
 */
void dwt_util_subband(
	void *ptr,		///< pointer to beginning of image data
	int stride_x,		///< difference between rows (in bytes)
	int stride_y,		///< difference between columns (in bytes)
	int size_o_big_x,	///< width of outer image frame (in elements)
	int size_o_big_y,	///< height of outer image frame (in elements)
	int size_i_big_x,	///< width of nested image (in elements)
	int size_i_big_y,	///< height of nested image (in elements)
	int j_max,		///< pointer to the decomposition level of interest
	enum dwt_subbands band,	///< subband of interest (LL, HL, LH, HH)
	void **dst_ptr,		///< here will be stored pointer to beginning of subband data
	int *dst_size_x,	///< here will be stored width of subband
	int *dst_size_y		///< here will be stored height of subband
);

/**
 * @brief Compute address of given transform coefficient or image pixel.
 * 
 * @warning This function is slow; faster way is calculate the address directly.
 */
float *dwt_util_addr_coeff_s(
	void *ptr,		///< pointer to beginning of image data
	int y,			///< x-coordinate
	int x,			///< y-coordinate
	int stride_x,		///< difference between rows (in bytes)
	int stride_y		///< difference between columns (in bytes)
);

/**
 * @brief Compute address of given transform coefficient or image pixel.
 * 
 * @warning This function is slow; faster way is calculate the address directly.
 */
double *dwt_util_addr_coeff_d(
	void *ptr,		///< pointer to beginning of image data
	int y,			///< x-coordinate
	int x,			///< y-coordinate
	int stride_x,		///< difference between rows (in bytes)
	int stride_y		///< difference between columns (in bytes)
);

/**
 * @brief Compute address of given transform coefficient or image pixel.
 * 
 * @warning This function is slow; faster way is calculate the address directly.
 */
void *dwt_util_addr_coeff(
	void *ptr,		///< pointer to beginning of image data
	int y,			///< x-coordinate
	int x,			///< y-coordinate
	int stride_x,		///< difference between rows (in bytes)
	int stride_y		///< difference between columns (in bytes)
);

/**
 * @brief Convert transform to viewable format.
 */
void dwt_util_conv_show_s(
	const void *src,	///< transform
	void *dst,		///< viewable image of transform
	int stride_x,		///< difference between rows (in bytes)
	int stride_y,		///< difference between columns (in bytes)
	int size_i_big_x,	///< width of nested image (in elements)
	int size_i_big_y	///< height of nested image (in elements)
);

/**
 * @brief Convert transform to viewable format.
 */
void dwt_util_conv_show_d(
	const void *src,	///< transform
	void *dst,		///< viewable image of transform
	int stride_x,		///< difference between rows (in bytes)
	int stride_y,		///< difference between columns (in bytes)
	int size_i_big_x,	///< width of nested image (in elements)
	int size_i_big_y	///< height of nested image (in elements)
);

/**
 * @brief Performance test of 2-D DWT with CDF 9/7 wavelet.
 *
 * @warning experimental
 */
void dwt_util_perf_cdf97_2_s(
	int stride_x,		///< difference between rows (in bytes)
	int stride_y,		///< difference between columns (in bytes)
	int size_o_big_x,	///< width of outer image frame (in elements)
	int size_o_big_y,	///< height of outer image frame (in elements)
	int size_i_big_x,	///< width of nested image (in elements)
	int size_i_big_y,	///< height of nested image (in elements)
	int j_max,		///< the number of intended decomposition levels (scales)
	int decompose_one,	///< should be row or column of size one pixel decomposed? zero value if not
	int zero_padding,	///< fill padding in channels with zeros? zero value if not, should be non zero only for sparse decomposition
	int M,			///< one test loop consists of transform of M images
	int N,			///< number of test loops performed
	int clock_type,		///< timer type
	float *fwd_secs,	///< store resulting time for forward transform here
	float *inv_secs		///< store resulting time for inverse transform here
);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
