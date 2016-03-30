#include "sm750_dma.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <pci/pci.h>
#include <pci/types.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/time.h>
#include "sm750_pll.h"

/* Global variable to save all the SMI devices */
static struct pci_dev *g_pCurrentDevice = (struct pci_dev *)0;
static struct pci_access *g_pAccess;
static struct pci_filter g_filter;


#define MAX_DMA_SDRAM_LEN 0xFFFFF0
#define SWAP32(a) ((a>>24)|((a&0xff00)<<8)|((a&0xff0000)>>8)|((a&0xff)<<24))

const int i = 1;
#define is_bigendian() ( (*(char*)&i) == 0 )

#define PCI_CONFIG_SIZE (2*1024*1024)

int pci_config_fd;
void* pci_config_addr;

void open_pci_config_space (const char const* pci_config_res_file) {
  pci_config_fd = open(pci_config_res_file, O_RDWR | O_SYNC);
  if (pci_config_fd == -1)
    { perror("Open PCI config failed"); exit(-1); }
printf("mmap\n");
fflush(stdout);
  pci_config_addr =
    mmap(0, PCI_CONFIG_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, pci_config_fd, 0);
  if (pci_config_addr == MAP_FAILED)
    { perror("Mmap PCI config failed"); exit(-1); }
printf("read\n");
fflush(stdout);
}

void close_pci_config_space() {
  int res = munmap(pci_config_addr, PCI_CONFIG_SIZE);
  if (res == -1)
    { perror("Munmap PCI config failed"); exit(-1); }
  res = close(pci_config_fd);
  if (res == -1)
    { perror("Close PCI config failed"); exit(-1); }
}

void setup_plls () {
  uint32_t chip_freq = 336;
  uint32_t mem_freq = chip_freq;
  uint32_t master_freq = mem_freq/3;

  pll_value_t pll;
  pll.inputFreq = DEFAULT_INPUT_CLOCK;
  pll.clockType = MXCLK_PLL;
printf("Requesting chip clock: %i MHz\n", chip_freq);
  uint32_t actual_freq = calcPllValue(chip_freq*1000000UL, &pll);
printf("Actual chip clock: %i MHz\n", actual_freq);

  uint32_t pll_reg = (PLL_POWER_ON | (pll.POD<<PLL_POD_offt) | (pll.OD<<PLL_OD_offt) | (pll.N<<PLL_N_offt) | (pll.M<<PLL_M_offt));

printf("Setting MXCLK_PLL_CTRL: 0x%x\n", pll_reg);
  pci_write_mmio_uint32(MXCLK_PLL_CTRL, pll_reg);

  uint32_t divisor = (uint32_t) roundedDiv(actual_freq, mem_freq*1000000);
  uint32_t current_clock_status = pci_read_mmio_uint32(CLOCK_REG);
  switch (divisor) {
  default:
  case 1:
    current_clock_status &= ~((1<<12)|(1<<13));
    break;
  case 2:
    current_clock_status &= ~(1<<13);
    current_clock_status |= (1<<12);
    break;
  case 3:
    current_clock_status &= ~(1<<12);
    current_clock_status |= (1<<13);
    break;
  case 4:
    current_clock_status |= ((1<<12)|(1<<13));
    break;
  }
  uint32_t pm_control = pci_read_mmio_uint32(PM_CONTROL);
  uint32_t pm_reg = PM0_CLOCK_REG;
  if (pm_control & 1) {
    pm_reg = PM1_CLOCK_REG;
  }
printf("Setting m2xclk divisor: 0x%x\n", current_clock_status);
  pci_write_mmio_uint32(pm_reg, current_clock_status);

  uint32_t master_divisor = (uint32_t)roundedDiv(actual_freq, master_freq*1000000);
  switch (master_divisor) {
  default:
  case 3:
    current_clock_status &= ~((1<<14)|(1<<15));
    break;
  case 4:
    current_clock_status &= ~(1<<15);
    current_clock_status |= (1<<14);
    break;
  case 6:
    current_clock_status |= (1<<15);
    current_clock_status &= ~(1<<14);
    break;
  case 8:
    current_clock_status |= ((1<<14)|(1<<15));
    break;
  }
  printf("Setting mclk divisor: 0x%x\n", current_clock_status);
  pci_write_mmio_uint32(pm_reg, current_clock_status);

  printf("SM750 local mem reset\n");
  uint32_t misc_ctrl = pci_read_mmio_uint32(MISC_CTRL);
  misc_ctrl &= ~(1<<6);
  pci_write_mmio_uint32(MISC_CTRL, misc_ctrl);
  misc_ctrl |= (1<<6);
  pci_write_mmio_uint32(MISC_CTRL, misc_ctrl);
}

void pci_write_mmio_uint32(uint32_t offset, uint32_t val) {
  if (is_bigendian()) {
    val = SWAP32(val);
#ifdef PCIACCESS
    readPCIDword()
#else
    *((uint32_t *)(pci_config_addr + offset)) = val;
#endif
  } else {
    *((uint32_t *)(pci_config_addr + offset)) = val;
  }
}

uint32_t pci_read_mmio_uint32(uint32_t offset) {
  if (is_bigendian()) {
    uint32_t val = (*(uint32_t *)(pci_config_addr + offset));
    return SWAP32(val);
  }
  return (*(uint32_t *)(pci_config_addr + offset));
}

uint32_t pci_read_mmio_uint32_ex(void *p, uint32_t offset) {
  if (is_bigendian()) {
    uint32_t val = (*(uint32_t *)(p+offset));
    return SWAP32(val);
  }
  return (*(uint32_t *)(p+offset));
}

int dma_is_busy(void) {
  uint32_t value = pci_read_mmio_uint32(DMA1_SC);
  if (value & DMA_ACT) {
    return 1;
  }
  return 0;
}

void enable_dma(bool en) {
  if (en) {
    uint32_t value = pci_read_mmio_uint32(CLOCK_REG);
    pci_write_mmio_uint32(CLOCK_REG, value | DMA_EN);
    value = pci_read_mmio_uint32(PM0_CLOCK_REG);
    pci_write_mmio_uint32(PM0_CLOCK_REG, value | DMA_EN);
    value = pci_read_mmio_uint32(PM1_CLOCK_REG);
    pci_write_mmio_uint32(PM1_CLOCK_REG, value | DMA_EN);
  } else {
    uint32_t value = pci_read_mmio_uint32(CLOCK_REG);
    pci_write_mmio_uint32(CLOCK_REG, value & ~(DMA_EN));
    value = pci_read_mmio_uint32(PM0_CLOCK_REG);
    pci_write_mmio_uint32(PM0_CLOCK_REG, value);
    value = pci_read_mmio_uint32(PM1_CLOCK_REG);
    pci_write_mmio_uint32(PM1_CLOCK_REG, value);
  }
}

void enable_bus_master(bool en) {
  //printf("Enable bus master\n");
}

uint32_t set_pci_master_base_address(unsigned long physical_system_mem_address) {
	unsigned long pci_master_base_address, remaining_address;
  
  /* Set PCI Master Base Address */
//#define SM750_AA
/* #ifdef SM750_AA */
  /* pci_master_base_address = ((physical_system_mem_address & 0xFFF00000) >> 23) << 3; */
  /* pci_write_mmio_uint32(PCI_MASTER_BASE, pci_master_base_address); */
  
  /* /\* This errata only applies to System Memory. For local to local, no correction is */
  /*    needed. *\/ */
  /* remaining_address = ((physical_system_mem_address & 0x00700000) << 3) + (physical_system_mem_address & 0x000FFFFF); */
/* #else */
  /* pci_master_base_address = physical_system_mem_address >> 23; */
  /* pci_master_base_address = (pci_master_base_address > 0xFF) ? 0xFF : pci_master_base_address; */
  /* pci_write_mmio_uint32(PCI_MASTER_BASE, pci_master_base_address); */
  /* remaining_address = physical_system_mem_address - (pci_master_base_address << 23); */
/* #endif */

  uint32_t ph = physical_system_mem_address;
  remaining_address = ph & 0x7fffff;
  uint32_t pmb = (ph-remaining_address)>>23;
  pci_master_base_address = pmb;
  pci_write_mmio_uint32(PCI_MASTER_BASE, pci_master_base_address);
//ph-(pmb<<23);

  printf("pci_master_base_address: %x, sum: %x\n", pci_master_base_address, (remaining_address) | pmb<<23);
//  printf("pci_master_base_address: %x\n", pci_master_base_address);

  /* Send back the remaining address */
  return remaining_address;
}

long dma_channel_ex(unsigned long src_addr, unsigned char src_ext, unsigned long dest_addr, unsigned char dest_ext, unsigned long length) {
  unsigned long value, timeout;
  unsigned long old_system_control, pci_master_base_address;
  long return_value = 0;
    
  printf("src_addr %x\n", src_addr);
  //printf("dest_addr: %x\n", dest_addr);
    
  if (dma_is_busy() == 1)
  {
    printf("DMA is still busy.\n");
    return (-1);
  }

  /* Sanity check */
  if (length > MAX_DMA_SDRAM_LEN)
  {
    printf("Transfer Length exceeds 16 MB.\n");
    return (-2);
  }
    
  if ((src_addr & 0x0F) | (dest_addr & 0x0F) | (length & 0x0F))
  {
    printf("The source address, destination address, and length have to be 128-bit aligned.\n");
    return (-3);
  }
        
  if ((src_ext == 1) && (dest_ext == 1))
  {
    printf("System to System DMA transfer is not supported\n");
    return (-4);
  }

  value = src_addr; // & 0x3ffffff;//FIELD_VALUE(0, DMA_1_SOURCE, ADDRESS, src_addr);
  int remainder = 0;
  if (src_ext == 1)
  {
    value = set_pci_master_base_address(src_addr);
    remainder = value;
    //printf("Actual Source PCI_MASTER_BASE Address: %x\n", pci_read_mmio_uint32(PCI_MASTER_BASE));

    //value = (src_addr & 0x3ffffff) | EXT_MEM;
    value = remainder | EXT_MEM;
  }
  pci_write_mmio_uint32(DMA1_SRC, value);
  //printf("real source address: %x\n", src_addr);
  //printf("remainder: %x\n", remainder);
  //printf("write DMA_1_SOURCE Address: %x\n", value);
  printf("DMA_1_SOURCE Address: %x\n", pci_read_mmio_uint32(DMA1_SRC));

  value = dest_addr & 0x3ffffff;
  if (dest_ext == 1)
  {
    value = set_pci_master_base_address(dest_addr);
    remainder = value;
    //printf("Actual Destination PCI_MASTER_BASE Address: %x\n", pci_read_mmio_uint32(PCI_MASTER_BASE));

    value = remainder | EXT_MEM;
  }
  pci_write_mmio_uint32(DMA1_DST, value);
  printf("DMA_1_DESTINATION Address: %x\n", pci_read_mmio_uint32(DMA1_DST));

  value = length;
  pci_write_mmio_uint32(DMA1_SC, value);
  //printf("DMA_1_SIZE_CONTROL: %x\n", pci_read_mmio_uint32(DMA1_SC));

  value |= DMA_ACT;
  pci_write_mmio_uint32(DMA1_SC, value);

  return 0;
}

long dma_channel(unsigned long src_addr, unsigned char src_ext, unsigned long dest_addr, unsigned char dest_ext, unsigned long length) {
  unsigned long timeout, value;
  long return_value = 0;
    
  /* Enable DMA engine */    
  enable_dma(true);
    
  /* Abort previous DMA and enable it back */
  /* value = peekRegisterDWord(DMA_ABORT_INTERRUPT); */
  /* value = FIELD_SET(value, DMA_ABORT_INTERRUPT, ABORT_1, ABORT); */
  /* pokeRegisterDWord(DMA_ABORT_INTERRUPT, value); */
  /* value = FIELD_SET(value, DMA_ABORT_INTERRUPT, ABORT_1, ENABLE); */
  /* pokeRegisterDWord(DMA_ABORT_INTERRUPT, value); */
  value = pci_read_mmio_uint32(DMA1_INT);
  value = DMA_ABORT1;
  pci_write_mmio_uint32(DMA1_INT, value);
  value &= ~DMA_ABORT1;
  pci_write_mmio_uint32(DMA1_INT, value);
    
  /* Update the system control register if necessary. The PCI Master needs 
     to be started and enabled when accessing system memory. */
  
  if ((src_ext == 1) || (dest_ext == 1))
    enable_bus_master(1);
    
  return_value = dma_channel_ex(src_addr, src_ext, dest_addr, dest_ext, length);
    
  if (return_value == 0)
  {
    //printf("DMA channel returned 0\n");
    /* Wait until the DMA has finished. */
    timeout = 0x1000000; 
    do
    {
      timeout--;
    } while ((dma_is_busy() == 1) && (timeout));
    
    /* Timeout. Something wrong with the DMA engine */
    if (timeout == 0)
    {
      printf("Timeout waiting for DMA finish.\n");
      return_value = (-1);
    }
   
    /* Clear DMA Interrupt Status */
    /* value = peekRegisterDWord(DMA_ABORT_INTERRUPT); */
    /* value = FIELD_SET(value, DMA_ABORT_INTERRUPT, INT_1, CLEAR); */
    /* pokeRegisterDWord(DMA_ABORT_INTERRUPT, value); */
    value = pci_read_mmio_uint32(DMA1_INT);
    value &= ~(DMA_INT1);
    pci_write_mmio_uint32(DMA1_INT, value);
  } else {
    printf("dma_channel_ex returned %i\n", return_value);
  }
    
  if ((src_ext == 1) || (dest_ext == 1))
    enable_bus_master(0);

	/* Disable the DMA engine after finish. */
  enable_dma(false);
    
  return (return_value);
}

int
dma_channel_pre(unsigned long src_addr, unsigned char src_ext, unsigned long dest_addr, unsigned char dest_ext, uint32_t length) {
  unsigned long value, timeout;
  unsigned long old_system_control, pci_master_base_address;
  long return_value = 0;

  /* Enable DMA engine */    
  enable_dma(true);
    
  /* Abort previous DMA and enable it back */
  /* value = peekRegisterDWord(DMA_ABORT_INTERRUPT); */
  /* value = FIELD_SET(value, DMA_ABORT_INTERRUPT, ABORT_1, ABORT); */
  /* pokeRegisterDWord(DMA_ABORT_INTERRUPT, value); */
  /* value = FIELD_SET(value, DMA_ABORT_INTERRUPT, ABORT_1, ENABLE); */
  /* pokeRegisterDWord(DMA_ABORT_INTERRUPT, value); */

  value = pci_read_mmio_uint32(DMA1_INT);
  value = DMA_ABORT1;
  pci_write_mmio_uint32(DMA1_INT, value);
  value &= ~DMA_ABORT1;
  pci_write_mmio_uint32(DMA1_INT, value);
    
  /* Update the system control register if necessary. The PCI Master needs 
     to be started and enabled when accessing system memory. */
  
  if ((src_ext == 1) || (dest_ext == 1))
    enable_bus_master(1);
    
  //printf("src_addr %x\n", src_addr);
  //printf("dest_addr: %x\n", dest_addr);
    
  if (dma_is_busy() == 1)
  {
    printf("DMA is still busy.\n");
    return (-1);
  }

  /* Sanity check */
  if (length > MAX_DMA_SDRAM_LEN)
  {
    printf("Transfer Length exceeds 16 MB.\n");
    return (-2);
  }
    
  if ((src_addr & 0x0F) | (dest_addr & 0x0F) | (length & 0x0F))
  {
    printf("The source address, destination address, and length have to be 128-bit aligned.\n");
    return (-3);
  }
        
  if ((src_ext == 1) && (dest_ext == 1))
  {
    printf("System to System DMA transfer is not supported\n");
    return (-4);
  }

  value = src_addr; // & 0x3ffffff;//FIELD_VALUE(0, DMA_1_SOURCE, ADDRESS, src_addr);
  int remainder = 0;
  if (src_ext == 1)
  {
    value = set_pci_master_base_address(src_addr);
    remainder = value;
    //printf("Actual Source PCI_MASTER_BASE Address: %x\n", pci_read_mmio_uint32(PCI_MASTER_BASE));

    //value = (src_addr & 0x3ffffff) | EXT_MEM;
    value = remainder | EXT_MEM;
  }
  pci_write_mmio_uint32(DMA1_SRC, value);
  //printf("real source address: %x\n", src_addr);
  //printf("remainder: %x\n", remainder);
  //printf("write DMA_1_SOURCE Address: %x\n", value);
  printf("DMA_1_SOURCE Address: %x\n", pci_read_mmio_uint32(DMA1_SRC));

  value = dest_addr & 0x3ffffff;
  if (dest_ext == 1)
  {
    value = set_pci_master_base_address(dest_addr);
    remainder = value;
    //printf("Actual Destination PCI_MASTER_BASE Address: %x\n", pci_read_mmio_uint32(PCI_MASTER_BASE));

    value = remainder | EXT_MEM;
  }
  pci_write_mmio_uint32(DMA1_DST, value);
  printf("DMA_1_DESTINATION Address: %x\n", pci_read_mmio_uint32(DMA1_DST));

}

void
dma_channel_loop(int length) {
  uint32_t value;
  uint32_t timeout;

  //printf("DMA_1_SOURCE Address: %x\n", pci_read_mmio_uint32(DMA1_SRC));
  //printf("DMA_1_DESTINATION Address: %x\n", pci_read_mmio_uint32(DMA1_DST));
    
  value = length;
  pci_write_mmio_uint32(DMA1_SC, value);
  //printf("DMA_1_SIZE_CONTROL: %x\n", pci_read_mmio_uint32(DMA1_SC));

  value |= DMA_ACT;
  pci_write_mmio_uint32(DMA1_SC, value);

  timeout = 0x1000; // In milliseconds

  while (timeout--) {
    if (dma_is_busy() != 1) break;
    usleep(1000);
  }
    
    /* Timeout. Something wrong with the DMA engine */
    if (timeout == 0)
    {
      printf("Timeout waiting for DMA finish.\n");
    }
   
    /* Clear DMA Interrupt Status */
    /* value = peekRegisterDWord(DMA_ABORT_INTERRUPT); */
    /* value = FIELD_SET(value, DMA_ABORT_INTERRUPT, INT_1, CLEAR); */
    /* pokeRegisterDWord(DMA_ABORT_INTERRUPT, value); */
    value = pci_read_mmio_uint32(DMA1_INT);
    value &= ~(DMA_INT1);
    pci_write_mmio_uint32(DMA1_INT, value);

}

void
dma_channel_post(void) {
	/* Disable the DMA engine after finish. */
  enable_dma(false);
}
