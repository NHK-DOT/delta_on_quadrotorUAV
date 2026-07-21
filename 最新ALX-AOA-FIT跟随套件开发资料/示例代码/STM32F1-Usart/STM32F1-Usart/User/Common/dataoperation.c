/*
 *******************************************************************************
 *                           Includes
 *******************************************************************************
 */


#include  "dataoperation.h"

/*
 *******************************************************************************
 *
 *******************************************************************************
 */
void FillString(u8 *pString,u32 nStrLen,u8 nFillData)
{
    for(u32 i=0;i<nStrLen;i++)
    {
        *(pString+i) = nFillData;
    }
}

u16 U16HighLowByteSwap(u16 nByte)
{
  u16 high,low;
  
  high = nByte<<8;
  low = nByte>>8;
  
  return high+low;
}

u32 U32HighLowByteSwap(u32 nByte)
{
  u32 high;

  high = ((nByte&0x000000ff)<<24) + ((nByte&0xff000000)>>24)+ ((nByte&0x0000ff00)<<8)+ ((nByte&0x00ff0000)>>8);

  return high;
}

