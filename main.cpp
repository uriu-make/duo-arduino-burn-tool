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
#include <sys/stat.h>
#include <stdint.h>

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

bool rcv_none(int fd, uint8_t *buf, int len);
bool rcv_connect(int fd, uint8_t *buf, int len);
bool rcv_stop(int fd, uint8_t *buf, int len);
bool rcv_start(uint8_t *buf, int len);
bool rcv_update(uint8_t *buf, int len, uint32_t *file_size);
bool rcv_dataend(uint8_t *buf, int len, FILE *file);

bool remoteproc_init();
bool remoteproc_start(void);
bool remoteproc_stop(void);

int main(int argc, char *argv[]) {
  int serial_fd = 0;

  struct termios tio;
  int baudRate = B9600;  // 9600bps
  bool update = false;

  fd_set rfds;
  int ret = 0;

  uint8_t buf[CHUNK_SIZE];  // Serial receive buffer
  uint32_t file_size = 0;   // Received file size
  FILE *file = NULL;

  if (false == remoteproc_init()) {
    return -1;
  }

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
      if (true == rcv_none(serial_fd, buf, len)) {
        continue;
      }

      // Connection
      if (true == rcv_connect(serial_fd, buf, len)) {
        continue;
      }

      // STOP RrmoteProc
      if (true == rcv_stop(serial_fd, buf, len)) {
        continue;
      }

      // START RrmoteProc
      if (true == rcv_start(buf, len)) {
        continue;
      }

      // UPDATE_COMMAND
      if (true == rcv_update(buf, len, &file_size)) {
        update = true;
        file = fopen(FIRMWARE_FILE_PATH "/" FIRMWARE_FILE_NAME, "wb");
        continue;
      }

      // End Command
      if (true == rcv_dataend(buf, len, file)) {
        continue;
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

bool rcv_none(int fd, uint8_t *buf, int len) {
  int i = 0;
  if (len == sizeof(NONE_CMD)) {
    for (i = 0; i < len; i++) {
      if (buf[i] != NONE_CMD[i]) {
        return false;
      }
    }
    write(fd, buf, len);
    return true;
  }
  return false;
}

bool rcv_connect(int fd, uint8_t *buf, int len) {
  int i = 0;
  if (len == sizeof(CONNECT_CMD)) {
    for (i = 0; i < len; i++) {
      if (buf[i] != CONNECT_CMD[i]) {
        return false;
      }
    }
    write(fd, buf, len);
    return true;
  }
  return false;
}

bool rcv_stop(int fd, uint8_t *buf, int len) {
  int i = 0;
  int state_fd = 0;
  if (len == sizeof(STOP_CMD)) {
    for (i = 0; i < len; i++) {
      if (buf[i] != STOP_CMD[i]) {
        return false;
      }
    }
    remoteproc_stop();
    write(fd, buf, len);
    return true;
  }
  return false;
}

bool rcv_start(uint8_t *buf, int len) {
  int i = 0;
  int state_fd = 0;
  if (len == sizeof(START_CMD)) {
    for (i = 0; i < len; i++) {
      if (buf[i] != START_CMD[i]) {
        return false;
      }
    }
    if (i == len) {
      remoteproc_start();
      return true;
    }
  }
  return false;
}

bool rcv_update(uint8_t *buf, int len, uint32_t *file_size) {
  int i = 0;

  if (len == sizeof(UPDATE_CMD)) {
    for (i = 0; i < 4; i++) {
      if (buf[i] != UPDATE_CMD[i]) {
        return false;
      }
    }
    if (i == 4) {
      *file_size |= ((uint32_t)buf[4] << 0);
      *file_size |= ((uint32_t)buf[5] << 8);
      *file_size |= ((uint32_t)buf[6] << 16);
      *file_size |= ((uint32_t)buf[7] << 24);
      return true;
    }
  }
  return false;
}

bool rcv_dataend(uint8_t *buf, int len, FILE *file) {
  int i = 0;
  if (len == sizeof(DATAEND_CMD)) {
    for (i = 4; i < len; i++) {
      if (buf[i] != DATAEND_CMD[i]) {
        return false;
      }
    }
    if (i == len) {
      fclose(file);
      return true;
    }
  }
  return false;
}

bool remoteproc_init() {
  bool ret;
  int class_fd = open(FIRMWARE_CLASS, O_RDWR);
  if (class_fd < 0) {
    return false;
  }
  write(class_fd, FIRMWARE_FILE_PATH, sizeof(FIRMWARE_FILE_PATH));
  close(class_fd);

  int firmware_fd = open(REMOTEPROC_FIRMWARE, O_RDWR);
  if (firmware_fd < 0) {
    return false;
  }
  write(firmware_fd, FIRMWARE_FILE_NAME, sizeof(FIRMWARE_FILE_NAME));
  close(firmware_fd);

  ret = remoteproc_start();
  return ret;
}

bool remoteproc_start(void) {
  if (access(FIRMWARE_FILE_PATH "/" FIRMWARE_FILE_NAME, F_OK) == 0) {
    int fd = open(REMOTEPROC_STATE, O_RDWR);
    if (fd < 0) {
      return false;
    }
    write(fd, START, sizeof(START));
    close(fd);
    return true;
  } else {
    return false;
  }
}

bool remoteproc_stop(void) {
  int fd = open(REMOTEPROC_STATE, O_RDWR);
  if (fd < 0) {
    return false;
  }

  write(fd, STOP, sizeof(STOP));
  close(fd);
  return true;
}