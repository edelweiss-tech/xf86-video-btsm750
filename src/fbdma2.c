#include <linux/fb.h>
#include <linux/kd.h>
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <memory.h>
#include <stdlib.h>
#include "time_prof.h"
#include <stdio.h>
#include <pci/pci.h>
#include "sm750_dma.h"


#define RESERVED_MEM_ADDR 0x07000000
#define RUN_TIMES 100UL

uint32_t pixel_color(uint8_t r, uint8_t g, uint8_t b, struct fb_var_screeninfo *vinfo)
{
	return (r<<vinfo->red.offset) | (g<<vinfo->green.offset) | (b<<vinfo->blue.offset);
}

void
pset(void *dest, uint32_t c, size_t s) {
  uint32_t *ptr = (uint32_t *)dest;
  int ctr = 0;
  int end = (s/4);
  end *=4;
  while (ctr<end) {
    *ptr = c;
    ptr++;
    ctr+=4;
  }
  if ((s-end)!=0) {
    memcpy(dest+end, &c, s-end);
  }
}

int main(int argc, char **argv)
{
	struct fb_fix_screeninfo finfo;
	struct fb_var_screeninfo vinfo;

	int fb_fd = open("/dev/fb0",O_RDWR);
  int run;
  int color = 0;
  uint8_t *colors[6];


	long screensize = 1024 * 1024;

  printf("finfo.smem_len: %i\n", screensize);

  int mem_fd = open("/dev/mem", O_RDWR);
  void* fbp = mmap(0, screensize, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, (off_t)RESERVED_MEM_ADDR);

  memset(fbp, 0x0c, screensize);

  printf("Finished allocating frames\n");
  fflush(stdout);
  struct pci_access *pacc;
  struct pci_dev *dev;
  char namebuf[1024], *name;
  int flags = PCI_LOOKUP_VENDOR | PCI_LOOKUP_DEVICE;

  printf("open\n");
  fflush(stdout);
  if (argc < 2) {
    printf ("fbdma <sysfs pcie resource path>\n");
    goto exit;
  }

  open_pci_config_space(argv[1]);
  setup_plls();

  printf("src_addr: 0x%08x\n", RESERVED_MEM_ADDR);


  dma_channel_pre(RESERVED_MEM_ADDR, 1, 0, 0, screensize);

  printf("Running copy operation %i times\n", RUN_TIMES);
  double start = get_real_time();

  for (run=0;run<RUN_TIMES;run++) {
    dma_channel_loop(screensize);
  }
    
  double end = get_real_time();
  dma_channel_post();
  double dt = end-start;
  double mb = screensize/(1024.0*1024.0);
  double throughput = RUN_TIMES*mb/dt;
  printf("CPU time used = %lf\n", dt );
  printf("Throughput: %lfMB/s\n", throughput );
  printf("Width: %i, height: %i, bpp: %i\n", vinfo.xres, vinfo.yres, vinfo.bits_per_pixel);
  printf("FPS: %lf\n", RUN_TIMES/dt);
  printf("munmap\n");
  fflush(stdout);
  printf("ioctl\n");
  fflush(stdout);

exit:

  close_pci_config_space();
exit_no_pci:
//  ioctl(tty_fd,KDSETMODE,KD_TEXT);
  printf("close\n");
  fflush(stdout);
  
  close(fb_fd);
  printf("close\n");
  fflush(stdout);

//  close(tty_fd);

	return 0;
}
