#ifndef __HWID_H_
#define __HWID_H_

#include <linux/oem_smem_struct.h>

#include <asm/system_info.h>



extern int msm_project_id;


extern unsigned QcableVar;
extern unsigned QfactoryVar;
#define QcableFACTORY  0x66616374
#define QfactoryBIST   0x62697374
#define QfactoryFT     0x20206674

#endif
