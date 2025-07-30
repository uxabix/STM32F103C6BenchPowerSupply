/**
 * @file ftoa.c
 * @brief A simple float-to-ASCII conversion function.
 * @author kiril
 * @date Jul 9, 2025
 */

#include "ftoa.h"
#include <stdbool.h>

/**
 * @brief Reverses a string in place.
 * @param str The string to reverse.
 * @param len The length of the string.
 */
static void reverse(char* str, int len) {
    int i = 0, j = len - 1;
    while (i < j) {
        char temp = str[i];
        str[i++] = str[j];
        str[j--] = temp;
    }
}

/**
 * @brief Converts an integer to a string.
 * @param x The integer to convert.
 * @param str The output character buffer.
 * @param d The minimum number of digits to output (pads with leading zeros).
 * @return The number of characters written to the string.
 */
static int int_to_str(int x, char str[], int d) {
    int i = 0;
    bool is_negative = false;

    if (x == 0) {
        str[i++] = '0';
        while (i < d) {
            str[i++] = '0';
        }
        str[i] = '\0';
        return i;
    }

    if (x < 0) {
        is_negative = true;
        x = -x;
    }

    while (x > 0) {
        str[i++] = (x % 10) + '0';
        x /= 10;
    }

    // Pad with leading zeros if necessary
    while (i < d) {
        str[i++] = '0';
    }

    if (is_negative) {
        str[i++] = '-';
    }
    reverse(str, i);
    str[i] = '\0';
    return i;
}

/**
 * @brief Converts a floating-point number to a null-terminated string.
 * @details This is a simple implementation that handles negative numbers and
 *          a specified number of decimal places. It includes rounding for the
 *          fractional part.
 * @param n The float number to convert.
 * @param res Pointer to the output character array. The buffer must be large enough.
 * @param afterpoint The number of digits to display after the decimal point.
 */
void ftoa(float n, char* res, int afterpoint) {
    if (n < 0) {
        *res++ = '-';
        n = -n;
    }

    // Extract integer and fractional parts
    int ipart = (int)n;
    float fpart = n - (float)ipart;

    // Convert integer part to string
    int i = int_to_str(ipart, res, 0);

    // Append fractional part
    if (afterpoint > 0) {
        res[i] = '.';

        // Calculate the fractional part as an integer with rounding
        fpart = fpart * pow(10, afterpoint);
        int_to_str((int)(fpart + 0.5f), res + i + 1, afterpoint);
    }
}
