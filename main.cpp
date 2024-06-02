#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <sys/select.h>

#define CHUNK_SIZE 64

#define SERIAL_PORT            "/dev/ttyGS0"
#define REMOTEPROC_STATE       "/sys/class/remoteproc/remoteproc0/state"
#define REMOTEPROC_FIRMWARE    "/sys/class/remoteproc/remoteproc0/firmware"
#define FIRMWARE_CLASS         "/sys/module/firmware_class/parameters/path"
#define FIRMWARE_FILE_PATH     "/lib/firmware"
#define FIRMWARE_FILE_NAME     "arduino.elf"
#define OLD_FIRMWARE_FILE_NAME "arduino_old.elf"

#define START "start"
#define STOP  "stop"

#define NONE_CMD    ((unsigned char[]){0xAA, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00})
#define CONNECT_CMD ((unsigned char[]){0xAA, 0x55, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00})
#define STOP_CMD    ((unsigned char[]){0xAA, 0x55, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00})
#define START_CMD   ((unsigned char[]){0xAA, 0x55, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00})
#define UPDATE_CMD  ((unsigned char[]){0xAA, 0x55, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00})
#define DATAEND_CMD ((unsigned char[]){0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF})

int main(int argc, char *argv[]) {
  int serial_fd = 0;
  int state_fd = 0;
  int fd = 0;

  struct termios tio;
  int baudRate = B9600;  // 9600bps
  bool update = false;

  fd_set rfds;
  int ret = 0;

  int i;
  unsigned char buf[CHUNK_SIZE];  // Serial receive buffer
  uint32_t file_size = 0;         // Received file size
  FILE *file;

  serial_fd = open(SERIAL_PORT, O_RDWR);
  if (serial_fd < 0) {
    return -1;
  }

  tio.c_cflag += CREAD;
  tio.c_cflag += CLOCAL;
  tio.c_cflag += CS8;
  tio.c_cflag += 0;
  tio.c_cflag += 0;

  cfsetispeed(&tio, baudRate);
  cfsetospeed(&tio, baudRate);

  cfmakeraw(&tio);

  tcsetattr(serial_fd, TCSANOW, &tio);

  ioctl(serial_fd, TCSETS, &tio);

  state_fd = open(REMOTEPROC_STATE, O_RDWR);
  if (state_fd < 0) {
    return -1;
  }

  write(state_fd, STOP, sizeof(STOP));
  close(state_fd);

  fd = open(FIRMWARE_CLASS, O_RDWR);
  if (fd < 0) {
    return -1;
  }
  write(fd, FIRMWARE_FILE_PATH, sizeof(FIRMWARE_FILE_PATH));
  close(fd);

  fd = open(REMOTEPROC_FIRMWARE, O_RDWR);
  if (fd < 0) {
    return -1;
  }
  write(fd, FIRMWARE_FILE_NAME, sizeof(FIRMWARE_FILE_NAME));
  close(fd);

  state_fd = open(REMOTEPROC_STATE, O_RDWR);
  if (state_fd < 0) {
    return -1;
  }

  write(state_fd, START, sizeof(START));
  close(state_fd);

  while (true) {
    FD_ZERO(&rfds);
    FD_SET(serial_fd, &rfds);
    if (update == false) {
      ret = select(serial_fd + 1, &rfds, NULL, NULL, NULL);
    }
    if (ret == -1) {
      exit(0);
    }
    int len = read(serial_fd, buf, sizeof(buf));
    if (0 < len) {
      // NONE
      if (len == sizeof(NONE_CMD)) {
        for (i = 0; i < len; i++) {
          if (buf[i] != NONE_CMD[i]) {
            break;
          }
        }
        if (i == len) {
          write(serial_fd, buf, len);
          continue;
        }
      }

      // Connection
      if (len == sizeof(CONNECT_CMD)) {
        for (i = 0; i < len; i++) {
          if (buf[i] != CONNECT_CMD[i]) {
            break;
          }
        }
        if (i == len) {
          write(serial_fd, buf, len);
          continue;
        }
      }

      // STOP RrmoteProc
      if (len == sizeof(STOP_CMD)) {
        for (i = 0; i < len; i++) {
          if (buf[i] != STOP_CMD[i]) {
            break;
          }
        }
        if (i == len) {
          state_fd = open(REMOTEPROC_STATE, O_RDWR);
          if (state_fd < 0) {
            return -1;
          }

          write(state_fd, STOP, sizeof(STOP));
          write(serial_fd, buf, len);

          close(state_fd);
          continue;
        }
      }

      // START RrmoteProc
      if (len == sizeof(START_CMD)) {
        for (i = 0; i < len; i++) {
          if (buf[i] != START_CMD[i]) {
            break;
          }
        }
        if (i == len) {
          state_fd = open(REMOTEPROC_STATE, O_RDWR);
          if (state_fd < 0) {
            return -1;
          }

          write(state_fd, START, sizeof(START));
          close(state_fd);
          continue;
        }
      }

      // UPDATE_COMMAND
      if (len == sizeof(UPDATE_CMD)) {
        for (i = 0; i < 4; i++) {
          if (buf[i] != UPDATE_CMD[i]) {
            break;
          }
        }
        // printf("%d\n", file_size);
        if (i == 4) {
          file_size |= ((uint32_t)buf[4] << 0);
          file_size |= ((uint32_t)buf[5] << 8);
          file_size |= ((uint32_t)buf[6] << 16);
          file_size |= ((uint32_t)buf[7] << 24);

          file = fopen(FIRMWARE_FILE_PATH "/" FIRMWARE_FILE_NAME, "wb");
          update = true;
          continue;
        }
      }

      // End Command
      if (len == sizeof(DATAEND_CMD)) {
        for (i = 4; i < len; i++) {
          if (buf[i] != DATAEND_CMD[i]) {
            break;
          }
        }
        if (i == len) {
          fclose(file);
          continue;
        }
      }

      if (update == true) {
        fwrite(buf, len, 1, file);
        file_size -= len;
        if (file_size == 0) {
          update = false;
        }
        continue;
      }
    }
  }

  close(serial_fd);
  return 0;
}