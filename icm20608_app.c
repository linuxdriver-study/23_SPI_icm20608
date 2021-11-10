#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <linux/input.h>

int main(int argc, char *argv[])
{
        int ret = 0;
        int fd;
        char *filename = NULL;
        int buf[7] = {0};

        int gyro_x_adc, gyro_y_adc, gyro_z_adc;
	int accel_x_adc, accel_y_adc, accel_z_adc;
	int temp_adc;
	float gyro_x_act, gyro_y_act, gyro_z_act;
	float accel_x_act, accel_y_act, accel_z_act;
	float temp_act;

        if (argc != 2) {
                printf("Error Usage!\n"
                       "Usage: %s filename\n", argv[0]);
                ret = -1;
                goto error;
        }

        filename = argv[1];
        fd = open(filename, O_RDWR);
        if (fd == -1) {
                perror("open failed!\n");
                ret = -1;
                goto error;
        }

        while (1)
        {
                ret = read(fd, buf, sizeof(buf));
                if (ret < 0)
                {
                        perror("read error");
                        goto error;
                }
                gyro_x_adc = buf[0];
                gyro_y_adc = buf[1];
                gyro_z_adc = buf[2];
                accel_x_adc = buf[3];
                accel_y_adc = buf[4];
                accel_z_adc = buf[5];
                temp_adc = buf[6];

                /* 计算实际值 */
                gyro_x_act = (float)(gyro_x_adc) / 16.4;
                gyro_y_act = (float)(gyro_y_adc) / 16.4;
                gyro_z_act = (float)(gyro_z_adc) / 16.4;
                accel_x_act = (float)(accel_x_adc) / 2048;
                accel_y_act = (float)(accel_y_adc) / 2048;
                accel_z_act = (float)(accel_z_adc) / 2048;
                temp_act = ((float)(temp_adc)-25) / 326.8 + 25;

                printf("\r\n原始值:\r\n");
                printf("gx = %d, gy = %d, gz = %d\r\n", gyro_x_adc, gyro_y_adc, gyro_z_adc);
                printf("ax = %d, ay = %d, az = %d\r\n", accel_x_adc, accel_y_adc, accel_z_adc);
                printf("temp = %d\r\n", temp_adc);
                printf("实际值:");
                printf("act gx = %.2f°/S, act gy = %.2f°/S, act gz = %.2f°/S\r\n", gyro_x_act, gyro_y_act, gyro_z_act);
                printf("act ax = %.2fg, act ay = %.2fg, act az = %.2fg\r\n", accel_x_act, accel_y_act, accel_z_act);
                printf("act temp = %.2f°C\r\n", temp_act);
                memset(buf, 0, sizeof(buf));
                usleep(200000);
        }

error:
        close(fd);
        return ret;
}
