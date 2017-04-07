#include "sm750_output.h"

DisplayModePtr DRV_OutputGetModes(xf86OutputPtr output) {
    ScrnInfoPtr pScrn = output->scrn;
    xf86MonPtr pMon = NULL;
    I2CBusPtr bus;
    char edid[128];

    /*bus is null in the current randr driver, so canel the following*/
    //bus = (priv->path == PANEL_PATH ? pSmi->I2C_primary:pSmi->I2C_secondary);
    /* 	some thing weird: some times Xserver method to access EDID is okay while some times not
        but use hardware i2c to access DVI EDID should be always okay
    */
    XINF("try DDK method\n");
    /*
       because 750ddk.so is loaded by siliconmotion_drv.so
       so if one function of 750ddk.so is referenced as function pointer
       it will be failed .
       but calling the function is okay.
       I suggest combine 750ddk.so into siliconmotion_drv.so 
       which is a must for function pointer passing method

       below code pass xorg_I2CUDelay address into edidRead
       but X will exit with error when it be executed
       while just calling xorg_I2CUDelay is okay
       monk  @  10/20/2010
     */
    //pointer sm = xf86LoadDrvSubModule(pScrn->drv, "siliconmotion");
    //if (sm) {
      extern int32_t SM750_edidReadMonitorUtility(ScrnInfoPtr pScrn, char* edid, size_t size);
      int32_t r = SM750_edidReadMonitorUtility(pScrn, edid, sizeof(edid));
      //xf86UnloadSubModule(sm);
      
      if (r == 0) {
        pMon = xf86InterpretEDID(pScrn->scrnIndex,edid);
        if(pMon){
          xf86OutputSetEDID(output,pMon);
          XINF("Found monitor by DDK\n");
          /*Modify for changing the modeline used by manual*/
          //if(pSmi->MODELNE) {
          //LEAVE(xf86GetMonitorModes (output->scrn, output->conf_monitor));
          //} else {
          return xf86OutputGetEDIDModes(output);
        } else {
          XERR("DDK cannot get monitor\n");
        }
      } else {
        XERR("DDK cannot detect EDID Version\n");
      }
//}
    XMSG("DDK seems no mode found\n");
    return NULL;	
}

xf86OutputStatus
DRV_OutputDetect_PNL_CRT(xf86OutputPtr output)
{
  xf86DrvMsg(0, X_INFO,": CALL DRV_OutputDetect_PNL_CRT\n");
  return XF86OutputStatusConnected;
}

void
DRV_OutputCreateResources(xf86OutputPtr output) {
  xf86DrvMsg(0, X_INFO,": CALL DRV_OutputCreateResources\n");
}

Bool
DRV_OutputModeFixup(xf86OutputPtr output, DisplayModePtr mode, DisplayModePtr adjusted_mode) {
  xf86DrvMsg(0, X_INFO,": CALL DRV_OutputModeFixup\n");
  return TRUE;
}

void
DRV_OutputPrepare(xf86OutputPtr output) {
  xf86DrvMsg(0, X_INFO,": CALL DRV_OutputPrepare\n");
}

void
DRV_OutputCommit(xf86OutputPtr output) {
  xf86DrvMsg(0, X_INFO,": CALL DRV_OutputCommit\n");  
}

void
DRV_OutputDestroy(xf86OutputPtr output) {
  xf86DrvMsg(0, X_INFO,": CALL DRV_OutputDestroy\n");
  xfree(output);
}


Bool
DRV_CheckModeSize(ScrnInfoPtr scrp, int w, int x, int y)
{
    int bpp = scrp->fbFormat.bitsPerPixel, pad = scrp->fbFormat.scanlinePad;
    int lineWidth, lastWidth;

    if (scrp->depth == 4)
        pad *= 4;               /* 4 planes */

    /* Sanity check */
    if ((w < 0) || (x < 0) || (y <= 0))
        return FALSE;

    lineWidth = (((w * bpp) + pad - 1) / pad) * pad;
    lastWidth = x * bpp;

    /*
     * At this point, we need to compare
     *
     *  (lineWidth * (y - 1)) + lastWidth
     *
     * against
     *
     *  scrp->videoRam * (1024 * 8)
     *
     * These are bit quantities.  To avoid overflows, do the comparison in
     * terms of BITMAP_SCANLINE_PAD units.  This assumes BITMAP_SCANLINE_PAD
     * is a power of 2.  We currently use 32, which limits us to a video
     * memory size of 8GB.
     */

    lineWidth = (lineWidth + (BITMAP_SCANLINE_PAD - 1)) / BITMAP_SCANLINE_PAD;
    lastWidth = (lastWidth + (BITMAP_SCANLINE_PAD - 1)) / BITMAP_SCANLINE_PAD;

    if ((lineWidth * (y - 1) + lastWidth) >
        (scrp->videoRam * ((1024 * 8) / BITMAP_SCANLINE_PAD)))
        return FALSE;

    return TRUE;
}
