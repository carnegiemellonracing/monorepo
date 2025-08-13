#include <stdio.h>
#include <stdlib.h>

int main() {
    int x = 5, y = 10;
    if (x < y) {
        printf("x is less than y\n");
    } else {
        printf("x is greater than or equal to y\n");
    }

    for (int i = 0; i < 5; i++) {
        printf("i = %d\n", i);
    }

    int* ptr = malloc(sizeof(int) * 10);
    if (ptr == NULL) {
        return 1;
    }

    for (int j = 0; j < 10; j++) {
        ptr[j] = j * 2;
    }

    int sum = 0;
    for (int k = 0; k < 10; k++) {
        sum += ptr[k];
    }

    printf("Sum: %d\n", sum);

    free(ptr);
    return 0;
}