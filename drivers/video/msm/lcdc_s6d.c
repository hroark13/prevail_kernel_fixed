#if defined(CONFIG_MACH_ROOKIE)
#include "lcdc_s6d05a1_rookie.c"
#elif defined(CONFIG_MACH_ESCAPE)
#include "lcdc_s6d05a1_escape.c"
#elif defined(CONFIG_MACH_GIO)
#include "lcdc_s6d05a1_gio.c"
#elif defined(CONFIG_FB_MSM_LCDC_S6D04M0_QVGA)
#include "lcdc_s6d04m0.c"
#elif defined(CONFIG_FB_MSM_LCDC_S6D04D1_WQVGA)
#include "lcdc_s6d04d1.c"
#endif
