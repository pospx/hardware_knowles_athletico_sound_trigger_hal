/****************************************************************************
 ****************************************************************************
 ***
 ***   This header was automatically generated from a Linux kernel header
 ***   of the same name, to make information necessary for userspace to
 ***   call into the kernel available to libc.  It contains only constants,
 ***   structures, and macros generated from the original header, and thus,
 ***   contains no copyrightable information.
 ***
 ***   To edit the content of this header, modify the corresponding
 ***   source file (e.g. under external/kernel-headers/original/) then
 ***   run bionic/libc/kernel/tools/update_all.py
 ***
 ***   Any manual change here will be lost the next time this script will
 ***   be run. You've been warned!
 ***
 ****************************************************************************
 ****************************************************************************/
#ifndef __IAXXX_MODULE_H__
#define __IAXXX_MODULE_H__
struct iaxxx_sensor_info {
  uint32_t block_id;
  uint32_t inst_id;
};
struct iaxxx_sensor_param {
  uint32_t inst_id;
  uint32_t param_id;
  uint32_t param_val;
  uint8_t block_id;
};
#define IAXXX_IOCTL_MAGIC 'I'
#define MODULE_SENSOR_ENABLE _IO(IAXXX_IOCTL_MAGIC, 0x51)
#define MODULE_SENSOR_DISABLE _IO(IAXXX_IOCTL_MAGIC, 0x52)
#define MODULE_SENSOR_SET_PARAM _IO(IAXXX_IOCTL_MAGIC, 0x53)
#define MODULE_SENSOR_GET_PARAM _IO(IAXXX_IOCTL_MAGIC, 0x54)
#endif
