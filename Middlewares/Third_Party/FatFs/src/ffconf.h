/*---------------------------------------------------------------------------/
/  FatFs Functional Configuration (本项目裁剪版，基于 R0.15)
/---------------------------------------------------------------------------*/
#ifndef _FFCONF
#define _FFCONF 68300

/* 功能开关 */
#define FF_FS_READONLY      0      /* 允许写 */
#define FF_FS_MINIMIZE      0      /* 全功能 API */
#define FF_USE_STRFUNC      0
#define FF_USE_FIND         0
#define FF_USE_MKFS         0      /* 不建文件系统（卡预格式化为 FAT32） */
#define FF_USE_FASTSEEK     0
#define FF_USE_EXPAND       0
#define FF_USE_CHMOD        0
#define FF_USE_LABEL        0
#define FF_USE_LFN          0      /* 短文件名（省 RAM） */
#define FF_LFN_UNICODE      0
#define FF_FS_RPATH         0

/* 卷 */
#define FF_VOLUMES          1
#define FF_STR_VOLUME_ID    0
#define FF_MULTI_PARTITION  0
#define FF_MIN_SS           512
#define FF_MAX_SS           512
#define FF_USE_TRIM         0
#define FF_FS_NOFSINFO      0
#define FF_FS_TINY          0
#define FF_FS_EXFAT         0

/* 时间戳（无 RTC，固定值） */
#define FF_FS_NORTC         1
#define FF_NORTC_MON        1
#define FF_NORTC_DAY        1
#define FF_NORTC_YEAR       2026

/* 其它 */
#define FF_FS_LOCK          0
#define FF_FS_REENTRANT     0
#define FF_CODE_PAGE        437

#endif /* _FFCONF */
