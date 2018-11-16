#include <linux/kernel.h>
#include "port.h"
#include "common.h"

extern unsigned long _ENCODER_BUF_START,_DECODER_BUF_START;
extern unsigned int w55fa92_vde_v,w55fa92_vde_p;

/**
 ************************************************************************
 * \brief
 *    calculate Ceil(Log2(uiVal))
 ************************************************************************
 */
 /*
unsigned CeilLog2( unsigned uiVal)
{
  unsigned uiTmp = uiVal-1;
  unsigned uiRet = 0;

  while( uiTmp != 0 )
  {
    uiTmp >>= 1;
    uiRet++;
  }
  return uiRet;
}
*/
unsigned int phys_to_virt_92(unsigned int phy_addr)
{
    unsigned int addr_offset;
    
    addr_offset = phy_addr - _ENCODER_BUF_START;

    return (w55fa92_vde_v + addr_offset);
    
}





