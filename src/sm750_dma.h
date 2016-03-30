#ifndef __SM750_DMA_H__
#define __SM750_DMA_H__

#include <stdbool.h>
#include <stdint.h>

#define MMIO_BASE 0x0c000000
#define CLOCK_REG (0x000040)
#define PM0_CLOCK_REG (0x000044)
#define PM1_CLOCK_REG (0x000048)
#define PM_CONTROL (0x4c)
#define PCI_MASTER_BASE (0x000050)
#define FB0_ADDR (0x080044)
#define MISC_CTRL (0x04)

#define DMA1_SRC (0x0d0010)
#define DMA1_DST (0x0d0014)
#define DMA1_SC (0x0d0018)
#define DMA1_INT (0x0d0020)

#define EXT_MEM (1<<27)
#define DMA_ACT (1<<31)
#define DMA_EN (1)
#define DMA_ABORT1 (1<<5)
#define DMA_INT1 (1<<1)

#define PRIMARY_DISPLAY_CONTROL (0x080000)


void open_pci_config_space (const char const* pci_config_res_file);
void close_pci_config_space ();
void setup_plls();

int dma_is_busy(void);
void pci_write_mmio_uint32(uint32_t offset, uint32_t val);
uint32_t pci_read_mmio_uint32(uint32_t offset);
uint32_t pci_read_mmio_uint32_ex(void *p, uint32_t offset);
void enable_dma(bool en);
void enable_bus_master(bool en);
uint32_t set_pci_master_base_address(unsigned long physical_system_mem_address);
long dma_channel_ex(unsigned long src_addr, unsigned char src_ext, unsigned long dest_addr, unsigned char dest_ext, unsigned long length);
long dma_channel(unsigned long src_addr, unsigned char src_ext, unsigned long dest_addr, unsigned char dest_ext, unsigned long length);

int dma_channel_pre(unsigned long src_addr, unsigned char src_ext, unsigned long dest_addr, unsigned char dest_ext, uint32_t length);
void dma_channel_loop(int length);
void dma_channel_post(void);

#endif/*__SM750_DMA_H__*/
