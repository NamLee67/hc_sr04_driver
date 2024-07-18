#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <endian.h>

#define DEV_PATH "/dev/hc_sr04"

// Hàm đọc dữ liệu từ thiết bị
int read_device() {
    int fd = open(DEV_PATH, O_RDONLY);
    if (fd == -1) {
        perror("Could not open /dev/hc_sr04");
        return -1;
    }

    uint32_t data;
    ssize_t bytes_read = read(fd, &data, sizeof(data));
    if (bytes_read == sizeof(data)) {
        close(fd);
        return data;
    } else {
        if (bytes_read == -1) {
            perror("Read failed");
        } else {
            fprintf(stderr, "Read incomplete data\n");
        }
        close(fd);
        return -1;
    }
}

int main() {
    while (1) {
        int range_mm = read_device();
        if (range_mm != -1) {
            printf("Measured range in mm: %d\n", range_mm);
        }
        usleep(100 * 1000); // 100 milliseconds
    }

    return 0;
}

