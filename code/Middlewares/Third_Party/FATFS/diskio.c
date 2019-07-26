 #include "diskio.h"
 #include "uSD.h"


//#include "usart.h"
//#include "delay.h"
/*--------------------------------------------------------------------------
这里为FS文件系统的底层函数
移植FS文件系统总共需要写6个底层函数

---------------------------------------------------------------------------*/

extern SD_CardInfo SDCardInfo;


DSTATUS disk_initialize (
	BYTE drv				/* Physical drive nmuber (0..) */
)
{
 //SD_Error Status;
	int res;
 switch (drv) 
 {
	case 0 :	  
    //Status = SD_Init();
	res = uSD_Init();
    //if(Status == SD_OK)
	if (res >= 0)  return 0;
    else 
	  return STA_NOINIT;
	
	case 1 :	  
		return STA_NOINIT;
		  
	case 2 :
		return STA_NOINIT;
  }
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Return Disk Status                                                    */

DSTATUS disk_status (
	BYTE drv		/* Physical drive nmuber (0..) */
)
{
    switch (drv) 
	{
	  case 0 :		
	  /* translate the reslut code here	*/
	    return 0;
	  case 1 :
	  /* translate the reslut code here	*/
	    return 0;
	  case 2 :
	  /* translate the reslut code here	*/
	    return 0;
	  default:
        break;
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */

DRESULT disk_read (
	BYTE drv,		/* Physical drive nmuber (0..) */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address (LBA) */
	BYTE count		/* Number of sectors to read (1..255) */
)
{
  SD_Error Status;
  int res = 0;

  if ((drv != 0) && (count != 0)) return RES_ERROR;

  res = uSD_Read(sector, buff);
  if (res == 0) return RES_OK;
  else return RES_ERROR;
  /*
  if( !count )
  {    
    return RES_PARERR;
  }
  switch (drv)
  {
    case 0:
    if(count==1)
    {       
	  Status =  SD_ReadBlock( buff ,sector<< 9 , 512);
    }                                                
    else
    {   
      Status = SD_ReadMultiBlocks( buff ,sector<< 9 ,512,count);
    }                                                
    // Check if the Transfer is finished

printf("Read 1");


printf(" 's:%d ",sector);	
printf("c:%d' ",count);	

    Status = SD_WaitReadOperation();
printf("2,");
    while(SD_GetStatus() != SD_TRANSFER_OK);
printf("3.\r\n");
    if(Status == SD_OK)
    {
      return RES_OK;
    }
    else
    {
      return RES_ERROR;
    }   
	case 1:	
	  break;

    case 2:	
	  break;

    default:
      break;
  } 
  return RES_ERROR;*/
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */

#if _READONLY == 0
DRESULT disk_write_main (
	BYTE drv,			/* Physical drive nmuber (0..) */
	const BYTE *buff,	        /* Data to be written */
	DWORD sector,		/* Sector address (LBA) */
	BYTE count			/* Number of sectors to write (1..255) */
)
{
//DWORD L_sector=sector;
//BYTE L_count=count;

  SD_Error Status;

  int res = 0;

  if ((drv != 0) && (count != 0)) return RES_ERROR;

  res = uSD_Write(sector, buff);
  if (res == 0) return RES_OK;
  else return RES_ERROR;

  /*

printf("Write 1,");

printf(" 's:%d ",sector);	
printf("c:%d' ",count);	

//  delay_nms(100);

  if( !count )
  {    
    return RES_PARERR;
  }
  switch (drv)
  {
    case 0:
    if(count==1)
    {

       Status = SD_WriteBlock( (uint8_t *)(&buff[0]) ,sector << 9 , 512);
//       Status = SD_ReadBlock( (uint8_t *)(&buff[0]) ,sector << 9 , 512);

   
    }                                                
    else
    {

for (; L_count>0; L_count--,L_sector++){
printf("#1");
       Status = SD_WriteBlock( (uint8_t *)(&buff[((count-L_count)*512)]) ,L_sector << 9 , 512); 
//       Status = SD_ReadBlock( (uint8_t *)(&buff[((count-L_count)*512)]) ,L_sector << 9 , 512); 

    if(Status != SD_OK)printf("Fail_1!");
printf("#2");
    Status = SD_WaitReadOperation();
    if(Status != SD_OK)printf("Fail_2!");
printf("#3");
    while(SD_GetStatus() != SD_TRANSFER_OK);	 
printf("#4");
}

       Status = SD_WriteMultiBlocks( (uint8_t *)(&buff[0]) ,sector << 9 , 512,count);
    } 






//printf("sector: %d",sector);	
//printf("count: %d\r\n",count);	

    Status = SD_WaitReadOperation();
printf("2,");
    while(SD_GetStatus() != SD_TRANSFER_OK);	

printf("3.\r\n");
                                          
    if(Status == SD_OK)    {       return RES_OK;    }
    else    {       return RES_ERROR;    }

    case 2:
	   break;
    default :
       break;
  }


 return RES_ERROR;*/
}
#endif /* _READONLY */





DRESULT disk_write (
	BYTE drv,			/* Physical drive nmuber (0..) */
	const BYTE *buff,	        /* Data to be written */
	DWORD sector,		/* Sector address (LBA) */
	BYTE count			/* Number of sectors to write (1..255) */
)
{
DWORD L_sector=sector;
BYTE L_count=count;

//  SD_Error Status;

const BYTE * index_buff = buff;
BYTE L_buff[512];
unsigned int a;

DRESULT answer;

  if( !count )    return RES_PARERR;
  if( drv )    return RES_ERROR;

for (; L_count>0; L_count--){
for (a=0;a<512;a++) L_buff[a] = *index_buff++;
answer = disk_write_main (0, &L_buff[0], L_sector++, 1);
}

if (!L_count) return answer;
 return RES_ERROR;
}


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */

DRESULT disk_ioctl (
	BYTE drv,		/* Physical drive nmuber (0..) */
	BYTE ctrl,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
    if (drv)
    {    
        return RES_PARERR;  /* 仅支持单磁盘操作，否则返回参数错误 */
    }
	switch (ctrl) 
	{
	  case CTRL_SYNC :	      
		return RES_OK;
	  case GET_SECTOR_COUNT : 
	    *(DWORD*)buff = uSD_GetSize();//SDCardInfo.CardCapacity/SDCardInfo.CardBlockSize;
	    return RES_OK;
	  case GET_BLOCK_SIZE :
	    *(WORD*)buff = 512;//SDCardInfo.CardBlockSize;
	    return RES_OK;	
	  case CTRL_POWER :
		break;
	  case CTRL_LOCK :
		break;
	  case CTRL_EJECT :
		break;
      /* MMC/SDC command */
	  case MMC_GET_TYPE :
		break;
	  case MMC_GET_CSD :
		break;
	  case MMC_GET_CID :
		break;
	  case MMC_GET_OCR :
		break;
	  case MMC_GET_SDSTAT :
		break;	



	  case GET_SECTOR_SIZE :
		    *(DWORD*)buff = 512;
		break;



	}
	return RES_PARERR;   
}

/* 得到文件Calendar格式的建立日期,是DWORD get_fattime (void) 逆变换 */							
/*-----------------------------------------------------------------------*/
/* User defined function to give a current time to fatfs module          */
/* 31-25: Year(0-127 org.1980), 24-21: Month(1-12), 20-16: Day(1-31) */                                                                                                                                                                                                                                          
/* 15-11: Hour(0-23), 10-5: Minute(0-59), 4-0: Second(0-29 *2) */                                                                                                                                                                                                                                                
DWORD get_fattime (void)
{
   
    return 0;
}
