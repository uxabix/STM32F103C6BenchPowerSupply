/**
 * @file ftoa.h
 * @brief A simple float-to-ASCII conversion function.
 * @author kiril
 * @date Jul 9, 2025
 */

#ifndef INC_FTOA_H_
#define INC_FTOA_H_

#include <math.h>

/**
 * @brief Converts a floating-point number to a null-terminated string.
 * @param n The float number to convert.
 * @param res Pointer to the output character array.
 * @param afterpoint The number of digits to display after the decimal point.
 */
void ftoa(float n, char* res, uint8_t afterpoint);

#endif /* INC_FTOA_H_ */
