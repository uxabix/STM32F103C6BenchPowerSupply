/*
 * ftoa.c
 *
 *  Created on: Jul 9, 2025
 *      Author: kiril
 */


#include <stdio.h>
#include <stdbool.h>

void reverse(char* str, int len) {
    int i = 0, j = len - 1;
    while (i < j) {
        char temp = str[i];
        str[i++] = str[j];
        str[j--] = temp;
    }
}

// Преобразует int в строку
int int_to_str(int x, char str[], int d) {
    int i = 0;
    bool is_negative = false;

    if (x < 0) {
        is_negative = true;
        x = -x;
    }

    do {
        str[i++] = (x % 10) + '0';
        x /= 10;
    } while (x);

    if (is_negative) {
        str[i++] = '-';
    }

    // Добавление нулей после запятой
    while (i < d)
        str[i++] = '0';

    reverse(str, i);
    str[i] = '\0';
    return i;
}

// Преобразует float в строку (до 2 знаков после запятой)
void ftoa(float n, char* res, int afterpoint) {
    if (n < 0) {
        *res++ = '-';
        n = -n;
    }

    int ipart = (int)n;
    float fpart = n - (float)ipart;

    // Преобразуем целую часть
    int i = int_to_str(ipart, res, 0);

    // Добавим дробную часть
    if (afterpoint > 0) {
        res[i] = '.';

        fpart = fpart * pow(10, afterpoint);
        int_to_str((int)(fpart + 0.5f), res + i + 1, afterpoint);
    }
}
