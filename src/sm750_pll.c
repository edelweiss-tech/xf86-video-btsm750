#include "sm750_pll.h"
#include <stdio.h>

unsigned long
twoToPowerOfx(unsigned long x) {
    unsigned long i;
    unsigned long result = 1;

    for (i=1; i<=x; i++)
        result *= 2;

    return result;
}

/* Absolute differece between two numbers */
unsigned long
absDiff(unsigned long a, unsigned long b) {
    if ( a >= b )
        return(a - b);
    else
        return(b - a);
}

unsigned long
roundedDiv(unsigned long num, unsigned long denom) {
    /* n / d + 1 / 2 = (2n + d) / 2d */
    return (2 * num + denom) / (2 * denom);
}

unsigned long
calcPLL(pll_value_t *pPLL) {
  return (pPLL->inputFreq * pPLL->M / pPLL->N / twoToPowerOfx(pPLL->OD) / twoToPowerOfx(pPLL->POD));
}

unsigned long
calcPllValue(unsigned long ulRequestClk, pll_value_t *pPLL) {
  unsigned long M, N, OD, POD = 0, diff, pllClk, odPower, podPower, tempRequestClk;
  unsigned long bestDiff = 0xffffffff; /* biggest 32 bit unsigned number */

  /* Init PLL structure to know states */
  pPLL->M = 0;
  pPLL->N = 0;
  pPLL->OD = 0;
  pPLL->POD = 0;

  /* Convert everything in Khz range in order to avoid calculation overflow */
  pPLL->inputFreq /= 1000;
  tempRequestClk = ulRequestClk / 1000;
    
  /* If the requested clock is higher than 1 GHz, then set it to the maximum, which is
     1 GHz. */
  if (tempRequestClk > MHz(1000))
    tempRequestClk = MHz(1000);

  /* The maximum of post divider is 8. */
  for (POD=0; POD<=3; POD++)
  {
    /* MXCLK_PLL does not have post divider. */
    if ((POD > 0) && (pPLL->clockType == MXCLK_PLL))
      break;
    
    /* Work out 2 to the power of POD */
    podPower = twoToPowerOfx(POD);
        
    /* OD has only 2 bits [15:14] and its value must between 0 to 3 */
    for (OD=0; OD<=3; OD++)
    {
      /* Work out 2 to the power of OD */
      odPower = twoToPowerOfx(OD);            

      /* N has 4 bits [11:8] and its value must between 2 and 15. 
         The N == 1 will behave differently --> Result is not correct. */
      for (N=2; N<=15; N++)
      {
        /* The formula for PLL is ulRequestClk = inputFreq * M / N / (2^OD)
           In the following steps, we try to work out a best M value given the others are known.
           To avoid decimal calculation, we use 1000 as multiplier for up to 3 decimal places of accuracy.
        */
        M = tempRequestClk * N * odPower * 1000 / pPLL->inputFreq;
        M = roundedDiv(M, 1000);

        /* M field has only 8 bits, reject value bigger than 8 bits */
        if (M < 256)
        {
          /* Calculate the actual clock for a given M & N */        
          pllClk = pPLL->inputFreq * M / N / odPower / podPower;

          /* How much are we different from the requirement */
          diff = absDiff(pllClk, tempRequestClk);
        
          if (diff < bestDiff)
          {
            bestDiff = diff;

            /* Store M and N values */
            pPLL->M  = M;
            pPLL->N  = N;
            pPLL->OD = OD;
            pPLL->POD = POD;
          }
        }
      }
    }
  }
    
  /* Restore input frequency from Khz to hz unit */
  pPLL->inputFreq *= 1000;

  /* Output debug information */
  printf( "calcPllValue: Requested Frequency = %d\n", ulRequestClk);
  printf("calcPllValue: Input CLK = %dHz, M=%d, N=%d, OD=%d, POD=%d\n", pPLL->inputFreq, pPLL->M, pPLL->N, pPLL->OD, pPLL->POD);

  /* Return actual frequency that the PLL can set */
  return calcPLL(pPLL);
}
