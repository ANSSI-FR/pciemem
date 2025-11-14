#include <string.h>
#include <stdint.h>
#include <error.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/mman.h>

/* Read/write an USB 3380 EEPROM
 * © 2016-2017 ANSSI
 *
 * Yves-Alexis Perez <yves-alexis.perez@ssi.gouv.fr>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Documentation from USB 3380-AA/AB Data Book, v1.3 -- 7/10/12
 * Chapter  5 Serial EEPROM Controller
 * Chapter 14 PCI Configuration registers
 *         14.14.3 Device specific registers - Serial EEPROM (Offsets 260h - 26Ch)
 */

#define OFFSET_REG_EEPROM_CTL	0x260
#define OFFSET_REG_EEPROM_DATA  0x264
#define DEVICE_WAIT_TIME	10*1000

#define EEPROM_CMD 13
#define EEPROM_RDY 24
#define EEPROM_WEN 25

#define EEPROM_HEADER_SIZE 4 /* Table 5-1 Serial EEPROM Data Format */
#define EEPROM_ADDR_WIDTH 2 /* ours always returns 0b00 (undefined) but it
                               seems to use 2-bytes addressing */

#define SUBSYSTEM_VENDOR  "0x10b5"
#define SUBSYSTEM_DEVICE  "0x3380"

int check_subsystem(char* devpath)
{
  char subpath[] = "/sys/bus/pci/devices/0000:00:00.0/subsystem_vendor";
  int fd = -ENOENT;
  char info[] = SUBSYSTEM_VENDOR;

  /* check subsystem vendor id */
  sprintf(subpath, "%s/subsystem_vendor", devpath);
  fd = open(subpath, O_RDONLY);
	if (fd < 0)
		return -ENODEV;

  if (read(fd, info, 7) < 7)
		return -ENODEV;

  close(fd);

  if (strncmp(info, SUBSYSTEM_VENDOR, 6))
    return -ENODEV;

  /* check subsystem device id */
  sprintf(subpath, "%s/subsystem_device", devpath);
  fd = open(subpath, O_RDONLY);
	if (fd < 0)
		return -ENODEV;

  if (read(fd, info, 7) < 7)
		return -ENODEV;

  close(fd);

  if (strncmp(info, SUBSYSTEM_DEVICE, 6))
    return -ENODEV;

  return 0;
}

int open_config(char* devpath)
{
  char res[] = "/sys/bus/pci/devices/0000:00:00.0/config";

  sprintf(res, "%s/config", devpath);
  return open(res, O_RDWR);
}

int write_reg(const int config, const off_t reg, const uint32_t data)
{
  return pwrite(config, &data, 4, reg);
}

int read_reg(int config, off_t reg, uint32_t *data)
{
  return pread(config, data, 4, reg);
}

void eeprom_write_init(int config)
{
  uint32_t eectl = 0;

  read_reg(config, OFFSET_REG_EEPROM_CTL, &eectl);
  /* only keep bits 23-16, serial EEPROM status */
  eectl = (0x00ff0000 & eectl);
  /* set write latch enable, 0b110 */
  eectl |= 6 << EEPROM_CMD;
  write_reg(config, OFFSET_REG_EEPROM_CTL, eectl);
  usleep(DEVICE_WAIT_TIME);
}


void eeprom_write_exit(int config)
{
  uint32_t eectl = 0;

  read_reg(config, OFFSET_REG_EEPROM_CTL, &eectl);
  /* only keep bits 23-16, serial EEPROM status */
  eectl = (0x00ff0000 & eectl);
  /* reset write latch enable, 0b100 */
  eectl |= 4 << EEPROM_CMD;
  write_reg(config, OFFSET_REG_EEPROM_CTL, eectl);
  usleep(DEVICE_WAIT_TIME);
}


int eeprom_read_word(int config, off_t offset, uint32_t *data)
{
  uint32_t eectl=0;
  int ret=0;

  /* get current data of the EEPROM CTL register */
  ret = read_reg(config, OFFSET_REG_EEPROM_CTL, &eectl);

  /* only keep bits 23-16, serial EEPROM status */
  eectl = (0x00ff0000 & eectl);
  eectl |= offset >> EEPROM_ADDR_WIDTH;
  eectl |= 3 << EEPROM_CMD; /* read 4 bytes to EEPROM_DATA register */

  ret = write_reg(config, OFFSET_REG_EEPROM_CTL, eectl);
  usleep(DEVICE_WAIT_TIME);

  /* read data from the EEPROM data register */
  ret = read_reg(config, OFFSET_REG_EEPROM_DATA, data);

  return ret;
}
int eeprom_write_word(int config, off_t offset, uint32_t data)
{
  uint32_t eectl=0;
  int ret=-1;
  uint32_t check;

  eeprom_write_init(config);
  /* write data to the EEPROM data register */
  ret = write_reg(config, OFFSET_REG_EEPROM_DATA, data);

  /* get current data of the EEPROM CTL register */
  ret = read_reg(config, OFFSET_REG_EEPROM_CTL, &eectl);

  /* only keep bits 23-16, serial EEPROM status */
  eectl = (0x00ff0000 & eectl);
  eectl |= offset >> EEPROM_ADDR_WIDTH;
  eectl |= 2 << EEPROM_CMD; /* write 4 bytes from EEPROM_DATA buffer */
  eectl |= 1 << EEPROM_RDY;
  eectl |= 1 << EEPROM_WEN;

  write_reg(config, OFFSET_REG_EEPROM_CTL, eectl);
  usleep(DEVICE_WAIT_TIME);

  ret = eeprom_read_word(config, offset, &check);
  if (ret > 0 && check == data) return 0;

  error(0, 0, "ret=%d, offset=0x%.2lx, data=0x%.8x, check=0x%.8x", ret, offset, data, check);

  return ret;
}

int eeprom_write(int config, int file)
{
  uint32_t data=0;
  ssize_t count=0;
  off_t offset=0;

  do
  {
    count = pread(file, &data, 4, offset);
    switch(count)
    {
      case 1:
        if (errno == EINTR) continue;

        error(0, errno, "pread()");
        return -1;
        ;;
      case 0: /* EOF */
        break;
        ;;
      case 4:
        if (eeprom_write_word(config, offset, data) < 0) return -1;
        offset += count;
        continue;
        ;;
      default: /* something wrong happened */
        error(0, 0, "pread returned %ld\n", count);
        return -1;
        ;;
    }
  } while(count);

  return 0;
}

/* get EEPROM size, Table 5-1 and 5-2, pp 36-37 */
int eeprom_get_size(int config)
{
  uint32_t data=0;
  int eep_size = EEPROM_HEADER_SIZE;

  eeprom_read_word(config, 0, &data);

  /* check signature */
  if ((data & 0xff) != 0x5a)
    return -1;

  /* check format (0 for Configuration Register Load */
  if (data & 0xff00)
    return -1;

  eep_size += (data & 0xffff0000) >> 16;
  return eep_size;
}

int eeprom_read(int config, int dst, ssize_t size)
{
  int ret=-1;
  uint32_t data;

  for(off_t offset=0; offset < size; offset += 4)
  {
    eeprom_read_word(config, offset, &data);
    ret = pwrite(dst, &data, 4, offset);
		if(ret < 0) return ret;
  }

  return ret;
}

int main(int argc, char* argv[])
{
  int config;
  char* devpath;
  int ret=0;
  int eep_size;
  int file;

  if (argc < 4)
    error(EXIT_FAILURE, 0, "Usage: %s <sysfs device path> <flash|dump> <file>", argv[0]);

  devpath = strdup(argv[1]);
  printf("Using device file %s\n", devpath);

  /* safety check */
  if (check_subsystem(devpath))
    error(EXIT_FAILURE, 0, "Subsystem info don't match (should be %s:%s)",
        SUBSYSTEM_VENDOR, SUBSYSTEM_DEVICE);

  if (geteuid() > 0)
    error(EXIT_FAILURE, 0, "You must be root.\n");

  config = open_config(devpath);
	if (config == -1)
		error(EXIT_FAILURE, errno, "Can't open device %s PCI configuration header\n", devpath);

  if (strncmp(argv[2], "dump", 4) == 0)
  {
    eep_size =  eeprom_get_size(config);

    if (eep_size < EEPROM_HEADER_SIZE)
      error(EXIT_FAILURE, 0, "Invalid EEPROM size (%d)", eep_size);

    file = open(argv[3], O_WRONLY | O_CREAT | O_EXCL, S_IRUSR | S_IWUSR);
    if (file == -1)
      error(EXIT_FAILURE, errno, "Can't open file %s", argv[3]);

    return eeprom_read(config, file, eep_size);
  }

  if (strncmp(argv[2], "flash", 5) == 0)
  {
    eeprom_write_init(config);

    file = open(argv[3], O_RDONLY);
    if (file == -1)
      error(EXIT_FAILURE, errno, "Can't open file %s", argv[3]);

    return eeprom_write(config, file);
  }

  return ret;
}
