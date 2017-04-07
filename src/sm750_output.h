#ifndef __SM750_OUTPUT_H__
#define __SM750_OUTPUT_H__

#include "xorg-server.h"
#include "xf86.h"
#include "xf86Crtc.h"
#include <xorg/edid.h>

#define XINF(...) xf86DrvMsg(pScrn->scrnIndex, X_INFO,": "__VA_ARGS__)
#define XMSG(...) xf86DrvMsg(pScrn->scrnIndex, X_NOTICE,": "__VA_ARGS__)
#define XERR(...) xf86DrvMsg(pScrn->scrnIndex, X_ERROR,": "__VA_ARGS__)

DisplayModePtr DRV_OutputGetModes(xf86OutputPtr output);
xf86OutputStatus DRV_OutputDetect_PNL_CRT(xf86OutputPtr output);
void DRV_OutputCreateResources(xf86OutputPtr output);
Bool DRV_OutputModeFixup(xf86OutputPtr output, DisplayModePtr mode, DisplayModePtr adjusted_mode);
void DRV_OutputPrepare(xf86OutputPtr output);
void DRV_OutputCommit(xf86OutputPtr output);
void DRV_OutputDestroy(xf86OutputPtr output);
Bool DRV_CheckModeSize(ScrnInfoPtr scrp, int w, int x, int y);

#endif/*__SM750_OUTPUT_H__*/
