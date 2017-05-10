#ifndef __EXTDISP_DRV_PLATFORM_H__
#define __EXTDISP_DRV_PLATFORM_H__

#include <linux/dma-mapping.h>
#include <linux/types.h>
/*#include <mach/mt_gpio.h>*/
#include "m4u.h"
/*#include <mach/mt_reg_base.h>*/
#ifdef CONFIG_MTK_CLKMGR
#include <mach/mt_clkmgr.h>
#endif
/*#include <mach/mt_irq.h>*/
/*#include <board-custom.h>*/
#include "disp_assert_layer.h"
#include "ddp_hal.h"


#define MAX_SESSION_COUNT		5

/* #define MTK_LCD_HW_3D_SUPPORT */
#define ALIGN_TO(x, n)  \
	(((x) + ((n) - 1)) & ~((n) - 1))

#define MTK_EXT_DISP_ALIGNMENT 32
#define MTK_EXT_DISP_START_DSI_ISR
#define MTK_EXT_DISP_OVERLAY_SUPPORT
#define MTK_EXT_DISP_SYNC_SUPPORT
#define MTK_EXT_DISP_ION_SUPPORT
#define MTK_EXT_DISP_RELEASE_FENCE_NEW
/* /#define EXTD_DBG_USE_INNER_BUF */
#ifndef HW_OVERLAY_COUNT
#define HW_OVERLAY_COUNT                 (4)
#endif

#endif				/* __DISP_DRV_PLATFORM_H__ */
