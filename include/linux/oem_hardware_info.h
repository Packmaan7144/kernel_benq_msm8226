#ifndef _OEM_HARDWARE_INFO_H_
#define _OEM_HARDWARE_INFO_H_
typedef enum
{
  EVB              = 0x0101,
  EVT0             = 0x0201,
  EVT1             = 0x0301,
  EVT1_2           = 0x0302,
  EVT2             = 0x0401,
  EVT2_2           = 0x0402,
  EVT3             = 0x0501,
  DVT              = 0x0601,
  PVT              = 0x0701,
  MP               = 0x0801,
  UNKNOWN_BOARD_ID = 0x7FFFFFFF
}board_id_type;

typedef enum
{
  EU_MTS             = 0x0000,
  CHT                = 0x0001,
  TWM                = 0x0002,
  FET                = 0x0003,
  APT                = 0x0004,
  VIBO               = 0x0005,
  MY1                = 0x0006,
  MY2                = 0x0007,
  UNKNOWN_VARIANT_ID = 0x7FFFFFFF
}variant_id_type;

typedef struct
{
  board_id_type board_id;
  variant_id_type variant_id;
  unsigned int ddr_vendor_id;
  unsigned int ddr_size;
  unsigned int cpu_feature_id;
  unsigned int cpu_hw_revision;
  unsigned int facable;
}oem_hw_info_type;

#endif
