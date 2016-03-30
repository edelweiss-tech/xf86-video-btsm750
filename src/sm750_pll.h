#ifndef __SM750_PLL_H__
#define __SM750_PLL_H__

#define DEFAULT_INPUT_CLOCK 14318181 /* Default reference clock */
#define MHz(x) (x*1000000) /* Don't use this macro if x is fraction number */

/* Maximum Master Clock is about 190MHz */
#define MAXIMUM_MASTER_CLOCK        MHz(190)

/* Maximum Chip Clock (MXCLK) is 1 GHz */
#define MAXIMUM_CHIP_CLOCK          MHz(1000)

#define PLL_POWER_ON (1<<17)
#define PLL_POD_offt 14
#define PLL_OD_offt 12
#define PLL_N_offt 8
#define PLL_M_offt 0

#define MXCLK_PLL_CTRL 0x70

typedef enum _clock_type_t
{
    MXCLK_PLL,      /* Programmable Master clock */
    PRIMARY_PLL,    /* Programmable Primary pixel clock */
    SECONDARY_PLL,  /* Programmable Secondary pixel clock */
    VGA0_PLL,
    VGA1_PLL,
}
clock_type_t;

typedef struct pll_value_t
{
    clock_type_t clockType;
    unsigned long inputFreq; /* Input clock frequency to the PLL */
    unsigned long M;
    unsigned long N;
    unsigned long OD;
    unsigned long POD;
}
pll_value_t;


unsigned long
roundedDiv(unsigned long num, unsigned long denom);
unsigned long
calcPllValue(unsigned long ulRequestClk, pll_value_t *pPLL);

#endif/*__SM750_PLL_H__*/
