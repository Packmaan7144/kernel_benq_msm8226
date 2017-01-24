/*******************************************************************************
                                                                                
                      Qisda Handset Project                                     
                                                                                
                     Copyright (c) 2011 Qisda Corpration                        
                                                                                
********************************************************************************/ 
#ifndef _OEM_SMEM_STRUCT_H_
#define _OEM_SMEM_STRUCT_H_

#include "qotp.h"
#include "oem_hardware_info.h"

#define OTP_IMEI_LENGTH     	15
#define OTP_SERIAL_NO_LENGTH    16
#define OTP_BT_LENGTH       	12
#define OTP_WIFI_LENGTH     	12
#define OTP_QLOCK_LENGTH    	128

/* This enum indicates OEM band type*/
typedef enum
{
	ATT                             =	0x11,

    #if defined(T_DETROIT_CDMA)
    CDMA_BC0                        =	0x01,
    CDMA_DUAL                       =	0x00,
    #endif

	BAND_CDMA                       =	0x01,
	BAND_UMTS,
	
	BAND_NOT_SUPPORTED              = 	0x7fffffff
}BAND_TYPE;

typedef enum
{
	DETROIT							=	0x0,
	DENVER							=	0x1,
	DETROIT_CDMA					=	0x2,
	SEATTLE							=	0x3,
	HOUSTON 						=	0x4,
	UNKNOWN_PROJECT_ID				= 	0x7fffffff
} PROJECT_ID_TYPE;
	
/* This is real DDR vendor number from mode register */
typedef enum
{
	SAMSUNG_DDR						=	0x1,
	ELPIDA_DDR						=	0x3,
	HYNIX_DDR						=	0x6,
	MICRON_DDR						= 	0xff,
	UNKNOWN_DDR_VENDOR				= 	0x7fffffff
} DDR_VENDOR_TYPE;
	

/* structure for EFS file saving buffer */
#define MAX_EFS_FILE_SIZE 35840
#define OFFSET_START_ADDRESS  0xFA00000 

/* max filename, path length */
#define MAX_EFS_NAME_LENGTH  50
#define MAX_EFS_PATH_LENGTH  50

/* max name list, each separated by '#' */
#define MAX_EFS_LIST_LEN 1000

/* EFS file content */
typedef struct {
	char 	        name[MAX_EFS_NAME_LENGTH];
	unsigned long   size;                     /* size in bytes */
	char            buffer[MAX_EFS_FILE_SIZE+1];
}LogBufferInfo;

/* EFS directory content */
typedef struct {
	char           path[MAX_EFS_PATH_LENGTH];
	unsigned long  fileNum; 
	char           fileList[MAX_EFS_LIST_LEN+1];	    
	unsigned long  dirNum; 
	char           dirList[MAX_EFS_LIST_LEN+1];	
}LogFolderInfo;

/* This is real reversion number */
typedef enum
{
	First_Tapeout 					= 	0x0,
	ES1								=	0x1,
	ES2								=	0x2,
	CS								=	0x4,
	UNKNOWN_CPU_VERSION				= 	0x7fffffff
} CPU_VERSION_TYPE;

typedef enum
{
	NORMAL_CABLE					=	0x0,
	FACTORY_CABLE					=	0x1,
	UNKNOWN_Q_CABLE					= 	0x7fffffff
} CABLE_TYPE;

typedef enum
{
	BIST_MODE						=	0x0,
	FT_MODE							=	0x1,
	NORMAL_MODE						= 	0x2,
	UNKNOWN_FACTORY_MODE			= 	0x7fffffff
} FACTORY_MODE_TYPE;

/* This is real eMMC's vendor ID */
typedef enum
{
	SANDISK_EMMC					=	0x45,
	KINGSTON_EMMC					=	0x70,
	SAMSUNG_EMMC        			= 	0x15,
	MICRON_EMMC 					= 	0xFE,
	UNKNOWN_EMMC_VENDOR				= 	0x7fffffff
} EMMC_VENDOR_TYPE;
		
typedef struct
{
	unsigned int 		card_size_in_sectors;
	unsigned int 		write_protect_group_size_in_sectors;
	EMMC_VENDOR_TYPE	vendor_id;
}EMMC_INFO_TYPE;

typedef struct
{
	unsigned char		rf_id_flag;           // rf chip version;
	unsigned char		rf_id_ver;           // rf chip version;
}RF_VERSION_TYPE;
		
typedef struct 
{
	BAND_TYPE 			band_type;
	PROJECT_ID_TYPE 	project_id;
	int 				hw_id_adc_value;
	unsigned int		lpddr2_size_in_MB;
	DDR_VENDOR_TYPE		ddr_vendor;
	CPU_VERSION_TYPE  	cpu_version;
	CABLE_TYPE			cable_type;
	FACTORY_MODE_TYPE	factory_mode;
	EMMC_INFO_TYPE		emmc_info;
	unsigned char		uart_over_uim1; // 1 means uart over uim1;
}HW_INFO;

typedef enum 
{
	DEFAULT_NV_RESTORE_STATUS_NOT_READY		= 0,
	DEFAULT_NV_RESTORE_STATUS_READY 		= 1,
	DEFAULT_NV_RESTORE_STATUS_MAX 			= 0x7fffffff,
} default_nv_restore_status_type;



/* Vendor ID 0 : data written by AMSS */
typedef struct
{
    unsigned long int 	modem_reboot_reason;
	RF_VERSION_TYPE     rf_id_info;
	int 				time_zone;
  LogBufferInfo  EFSfileBuf;
  LogFolderInfo  EFSfolderBuf;
} smem_vendor_id0_amss_data;
  
/* Vendor ID 1 : data written by APPS */
typedef struct
{
    unsigned char imei[OTP_IMEI_LENGTH];
    unsigned char serial_no[OTP_SERIAL_NO_LENGTH];
    unsigned char bt[OTP_BT_LENGTH];
    unsigned char wifi[OTP_WIFI_LENGTH];
    unsigned char qlock[OTP_QLOCK_LENGTH];
  unsigned char meid[OTP_MEID_LENGTH-1];
} smem_vendor_id1_apps_data;
  
/* Vendor ID 2 : data written by Boot loader */
typedef struct
{    
   //HW_INFO  hw_info;
   oem_hw_info_type hw_info;
   default_nv_restore_status_type default_nv_restore_status;
}smem_vendor_id2_bl_data;
  
#endif /* _OEM_SMEM_STRUCT_H_ */

