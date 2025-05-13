#ifndef __PTAB__H__
#define __PTAB__H__




/* flash2 */
#undef  FLASH_TABLE_START_ADDR
#define FLASH_TABLE_START_ADDR                             (0x12000000)
#undef  FLASH_TABLE_SIZE
#define FLASH_TABLE_SIZE                                   (0x00008000)
#undef  FLASH_TABLE_OFFSET
#define FLASH_TABLE_OFFSET                                 (0x00000000)
#undef  HCPU_FLASH_CODE_START_ADDR
#define HCPU_FLASH_CODE_START_ADDR                         (0x12020000)
#undef  HCPU_FLASH_CODE_SIZE
#define HCPU_FLASH_CODE_SIZE                               (0x00200000)
#undef  HCPU_FLASH_CODE_OFFSET
#define HCPU_FLASH_CODE_OFFSET                             (0x00020000)
#define CODE_START_ADDR                                    (HCPU_FLASH_CODE_START_ADDR)
#define CODE_SIZE                                          (HCPU_FLASH_CODE_SIZE)
#undef  DFU_FLASH_CODE_START_ADDR
#define DFU_FLASH_CODE_START_ADDR                          (0x12220000)
#undef  DFU_FLASH_CODE_SIZE
#define DFU_FLASH_CODE_SIZE                                (0x00060000)
#undef  DFU_FLASH_CODE_OFFSET
#define DFU_FLASH_CODE_OFFSET                              (0x00220000)
#undef  DFU_DOWNLOAD_REGION_START_ADDR
#define DFU_DOWNLOAD_REGION_START_ADDR                     (0x12280000)
#undef  DFU_DOWNLOAD_REGION_SIZE
#define DFU_DOWNLOAD_REGION_SIZE                           (0x00120000)
#undef  DFU_DOWNLOAD_REGION_OFFSET
#define DFU_DOWNLOAD_REGION_OFFSET                         (0x00280000)
#undef  FS_REGION_START_ADDR
#define FS_REGION_START_ADDR                               (0x123A0000)
#undef  FS_REGION_SIZE
#define FS_REGION_SIZE                                     (0x00280000)
#undef  FS_REGION_OFFSET
#define FS_REGION_OFFSET                                   (0x003A0000)
#undef  KVDB_DFU_REGION_START_ADDR
#define KVDB_DFU_REGION_START_ADDR                         (0x12620000)
#undef  KVDB_DFU_REGION_SIZE
#define KVDB_DFU_REGION_SIZE                               (0x00004000)
#undef  KVDB_DFU_REGION_OFFSET
#define KVDB_DFU_REGION_OFFSET                             (0x00620000)
#undef  KVDB_BLE_REGION_START_ADDR
#define KVDB_BLE_REGION_START_ADDR                         (0x12624000)
#undef  KVDB_BLE_REGION_SIZE
#define KVDB_BLE_REGION_SIZE                               (0x00004000)
#undef  KVDB_BLE_REGION_OFFSET
#define KVDB_BLE_REGION_OFFSET                             (0x00624000)


/* psram1 */
#undef  PSRAM_DATA_START_ADDR
#define PSRAM_DATA_START_ADDR                              (0x60000000)
#undef  PSRAM_DATA_SIZE
#define PSRAM_DATA_SIZE                                    (0x00800000)
#undef  PSRAM_DATA_OFFSET
#define PSRAM_DATA_OFFSET                                  (0x00000000)


/* hpsys_ram */
#undef  HCPU_RAM_DATA_START_ADDR
#define HCPU_RAM_DATA_START_ADDR                           (0x20000000)
#undef  HCPU_RAM_DATA_SIZE
#define HCPU_RAM_DATA_SIZE                                 (0x0006BC00)
#undef  HCPU_RAM_DATA_OFFSET
#define HCPU_RAM_DATA_OFFSET                               (0x00000000)
#undef  FLASH_BOOT_LOADER_START_ADDR
#define FLASH_BOOT_LOADER_START_ADDR                       (0x20020000)
#undef  FLASH_BOOT_LOADER_SIZE
#define FLASH_BOOT_LOADER_SIZE                             (0x00020000)
#undef  FLASH_BOOT_LOADER_OFFSET
#define FLASH_BOOT_LOADER_OFFSET                           (0x00020000)
#undef  BOOTLOADER_RAM_DATA_START_ADDR
#define BOOTLOADER_RAM_DATA_START_ADDR                     (0x20040000)
#undef  BOOTLOADER_RAM_DATA_SIZE
#define BOOTLOADER_RAM_DATA_SIZE                           (0x00010000)
#undef  BOOTLOADER_RAM_DATA_OFFSET
#define BOOTLOADER_RAM_DATA_OFFSET                         (0x00040000)
#undef  HCPU_RO_DATA_START_ADDR
#define HCPU_RO_DATA_START_ADDR                            (0x2006BC00)
#undef  HCPU_RO_DATA_SIZE
#define HCPU_RO_DATA_SIZE                                  (0x00014000)
#undef  HCPU_RO_DATA_OFFSET
#define HCPU_RO_DATA_OFFSET                                (0x0006BC00)
#undef  HPSYS_MBOX_START_ADDR
#define HPSYS_MBOX_START_ADDR                              (0x2007FC00)
#undef  HPSYS_MBOX_SIZE
#define HPSYS_MBOX_SIZE                                    (0x00000400)
#undef  HPSYS_MBOX_OFFSET
#define HPSYS_MBOX_OFFSET                                  (0x0007FC00)
#undef  HCPU2LCPU_MB_CH2_BUF_START_ADDR
#define HCPU2LCPU_MB_CH2_BUF_START_ADDR                    (0x2007FC00)
#undef  HCPU2LCPU_MB_CH2_BUF_SIZE
#define HCPU2LCPU_MB_CH2_BUF_SIZE                          (0x00000200)
#undef  HCPU2LCPU_MB_CH2_BUF_OFFSET
#define HCPU2LCPU_MB_CH2_BUF_OFFSET                        (0x0007FC00)
#undef  HCPU2LCPU_MB_CH1_BUF_START_ADDR
#define HCPU2LCPU_MB_CH1_BUF_START_ADDR                    (0x2007FE00)
#undef  HCPU2LCPU_MB_CH1_BUF_SIZE
#define HCPU2LCPU_MB_CH1_BUF_SIZE                          (0x00000200)
#undef  HCPU2LCPU_MB_CH1_BUF_OFFSET
#define HCPU2LCPU_MB_CH1_BUF_OFFSET                        (0x0007FE00)


/* lpsys_ram */
#undef  LPSYS_RAM_START_ADDR
#define LPSYS_RAM_START_ADDR                               (0x20400000)
#undef  LPSYS_RAM_SIZE
#define LPSYS_RAM_SIZE                                     (0x00006000)
#undef  LPSYS_RAM_OFFSET
#define LPSYS_RAM_OFFSET                                   (0x00000000)



#endif
