/* V1730 for libbabies + CAENLIB */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <CAENDigitizer.h>
#include <CAENDigitizerType.h>
#include "libbabies.h"
#include "libbbcaenvme.h"
#include "v1730.h"

#define DEBUG
#ifdef DEBUG
#define DB(x) x
#else
#define DB(x)
#endif

typedef enum{
  CH_ENABLE_MASK,
  CH_TRIG_MASK,
  GAIN_CONTROL,
  RECORD_LENGTH,
  MAX_BLT_EVENTS,

  TRIG_LOGIC,
  DC_OFFSET,
  PRE_TRIG,
  BASE_LINE,
  N_SAMPLE,
  TRIG_THRESHOLD,
  ZLE_THRESHOLD,
  POLARITY,
  NO_THRESHOLD,
  TEST_PULSE,
  FP_IO_CONTROL,
  LVDS_MODE,
  
  PARAM_NUM,
} PARAM_INDEX;

const char Config_Name[PARAM_NUM][20]=
  {
    "CH_ENABLE_MASK",
    "CH_TRIG_MASK",
    "GAIN_CONTROL",
    "RECORD_LENGTH",
    "MAX_BLT_EVENTS",

    "TRIG_LOGIC",
    "DC_OFFSET",
    "PRE_TRIG",
    "BASE_LINE",
    "N_SAMPLE",
    "TRIG_THRESHOLD",
    "ZLE_THRESHOLD",
    "POLARITY",
    "NO_THRESHOLD",
    "TEST_PULSE",
    "FP_IO_CONTROL",
    "LVDS_MODE",
    
  };


V1730_Link* ConnectionConfigFile(char *f_ini, int *N_FADC_BOARD, int *N_BOARD, int efn){
  FILE *fp;
  char cp_line[400];
  char cfg_c[100];
  int board=0;
  int iword,temp_para[3];
  int SyncEnable=0,SelfTrig=0;
  int babies_flag=0;
  V1730_Link* Link=NULL;

  fp=fopen(f_ini,"r");
  if(fp==NULL){
    printf("Error: can't open Link ConfigFile %s\n", f_ini);
    return NULL;
  }

  while(fgets(cp_line,400,fp)!=NULL){
    if(cp_line[0]=='#') continue;
    if(cp_line[0]=='\n') continue;
    iword=sscanf(cp_line,"%s %d %d %d",cfg_c,&temp_para[0],&temp_para[1],&temp_para[2]);
    if(strcmp(cfg_c,"N_FADC_BOARD")==0){
      *N_FADC_BOARD=temp_para[0];
      continue;
    }
    if(strcmp(cfg_c,"SYNC_ENABLE")==0){
      SyncEnable=temp_para[0];
      continue;
    }
    if(strcmp(cfg_c,"SELF_TRIG")==0){
      SelfTrig=temp_para[0];
      continue;
    }
    if(strcmp(cfg_c,"BABIES")==0){
      if(temp_para[0]==efn){
	*N_BOARD=temp_para[1];
	Link = (V1730_Link*)calloc(*N_BOARD, sizeof(V1730_Link));
	babies_flag=1;
      }
      else babies_flag=0;
      continue;
    }
    if(strcmp(cfg_c,"BOARD")==0 && babies_flag==1){
      if(iword!=4){
	printf("Error: Required 3 board parameters\n");
	return NULL;
      }
      
      Link[board].BoardIndex=temp_para[0];
      Link[board].LinkNum=temp_para[1];
      Link[board].ConetNode=temp_para[2];
      Link[board].SyncEnable=SyncEnable;
      Link[board].SelfTrig=SelfTrig;
      board++;
      if(board==*N_BOARD) break;	  
    }
  }
  
  if(Link==NULL){
    printf("Error: Not defined N_BOARD for babies:%d\n",efn);
    return NULL;
  }
  if(board!=*N_BOARD){
    printf("Error: Required N_FADC_BOARD=%d, defined=%d\n",*N_BOARD,board);
    return NULL;
  }

  fclose(fp);
  return Link;
}

int ParseConfigFile(char *f_ini, V1730_param *ConfigVar){
  FILE *fp;
  char cp_line[400];
  char cfg_c[100],para_c[100];
  int iword;
  int temp_para[3];
  int i,nline=0,index;
  int32_t mask,regval;
  fp=fopen(f_ini,"r");
  if(fp==NULL){
    printf("Error: can't open ConfigFile %s\n", f_ini);
    return -1;
  }
  else printf("   ConfigFile %s\n",f_ini);

  while(fgets(cp_line,400,fp)!=NULL){
    nline++;
    if(cp_line[0]=='#') continue;
    iword=sscanf(cp_line,"%s %d %d %d",cfg_c,&temp_para[0],&temp_para[1],&temp_para[2]);
    if(iword==0) continue;
    index=-1;
    for(i=0;i<PARAM_NUM;i++){
      if(strcmp(cfg_c,Config_Name[i])==0){
	index=i;
	break;
      }
    }
    switch(index){
    case CH_ENABLE_MASK:
      sscanf(cp_line,"%s %x",cfg_c,&mask);
      ConfigVar->ChEnableMask=mask;
      break;
    case CH_TRIG_MASK:
      sscanf(cp_line,"%s %x",cfg_c,&mask);
      ConfigVar->ChTrigMask=mask;
      break;
    case GAIN_CONTROL:
      sscanf(cp_line,"%s %s",cfg_c,para_c);
      if(strcmp(para_c,"HIGH")==0) ConfigVar->gain=0;
      else if(strcmp(para_c,"LOW")==0) ConfigVar->gain=1;
      else printf("GAIN_CONTROL must be ''HIGH'' or ''LOW'' (set ''HIGH'')\n");
      break;
    case RECORD_LENGTH:
      ConfigVar->RecordLength=temp_para[0];
      break;
    case MAX_BLT_EVENTS:
      ConfigVar->MaxBLTEvents=temp_para[0];
      break;
    case TRIG_LOGIC:
      break;
    case DC_OFFSET:
      ConfigVar->DCOffset=temp_para[0];
      break;
    case PRE_TRIG:
      ConfigVar->PreTrig=temp_para[0];
      break;
    case BASE_LINE:
      ConfigVar->BLine[0]=temp_para[0];
      ConfigVar->BLine[1]=temp_para[1];
      ConfigVar->BLine[2]=temp_para[2];
      break;
    case N_SAMPLE:
      ConfigVar->NSample[0]=temp_para[0];
      ConfigVar->NSample[1]=temp_para[1];
      break;
    case TRIG_THRESHOLD:
      ConfigVar->TrigThreshold=temp_para[0];
      break;
    case  ZLE_THRESHOLD:
      ConfigVar->ZLEThreshold=temp_para[0];
      break;
    case POLARITY:
      sscanf(cp_line,"%s %s",cfg_c,para_c);
      if(strcmp(para_c,"POSITIVE")==0) ConfigVar->Polarity=1;
      else if(strcmp(para_c,"NEGATIVE")==0) ConfigVar->Polarity=0;
      else printf("POLARITY must be ''POSITIVE'' or ''NEGATIVE'' (set ''NEGATIVE'')\n");
      break;
    case NO_THRESHOLD:
      sscanf(cp_line,"%s %s",cfg_c,para_c);
      if(strcmp(para_c,"YES")==0) ConfigVar->NoThreshold=1;
      else ConfigVar->NoThreshold=0;
      break;
    case TEST_PULSE:
      break;
    case FP_IO_CONTROL:
      sscanf(cp_line,"%s %x",cfg_c,&regval);
      ConfigVar->FrontIO=regval;
      break;
    case LVDS_MODE:
      sscanf(cp_line,"%s %x",cfg_c,&regval);
      ConfigVar->LVDSMode=regval;
      break;
    }
  }
  fclose(fp);
  
  return 0;
}

/* Channel calibration */
int v1730_calib(int handle){
  int ret=CAEN_DGTZ_Calibrate(handle);
  if(ret!=CAEN_DGTZ_Success){
    printf("\n\n Error calibration the digitizer\n");
    return 0;
  }
  printf("\n Channel calibration of the digitizer was done.\n");
  return 1;
}

/* Set the waveform test bit for debugging */
int v1730_test(int handle){
  int ret=CAEN_DGTZ_WriteRegister(handle, CAEN_DGTZ_BROAD_CH_CONFIGBIT_SET_ADD, 1<<3);
  if(ret!=CAEN_DGTZ_Success){
    printf("\n\n Error set test bit the digitizer\n");
    return 0;
  }
  printf("\n Set the waveform test bit for debugging.\n");
  return 1;
}

/**** for v1730 ****/
void v1730_clear(int handle){
  int ret;
  ret=CAEN_DGTZ_ClearData(handle);
  if(ret!=CAEN_DGTZ_Success) printf("clear ret=%d\n",ret);
}

void v1730_read_data(int handle, char *buffer, uint32_t *buffersize){
  CAEN_DGTZ_ReadData(handle,CAEN_DGTZ_SLAVE_TERMINATED_READOUT_MBLT,buffer,buffersize);
}

void v1730_get_event(int handle, char *buffer, uint32_t buffersize, void **event, uint32_t *numevents){
#ifdef USE_ZLE
  CAEN_DGTZ_GetZLEEvents(handle, buffer, buffersize/4,event,numevents);
#else
  CAEN_DGTZ_GetNumEvents(handle, buffer, buffersize, numevents);
#endif
}

void v1730_start_acquisition(int *handle, int N_FADC_BOARD, V1730_Link *Link){
  int board;
  for(board=0;board<N_FADC_BOARD;board++){
    if(Link[board].SyncEnable==0 || Link[board].BoardIndex==0){
      CAEN_DGTZ_SWStartAcquisition(handle[board]);
      printf("Board:%d Acquisition start\n",board);
    }
  }
}

int v1730_malloc_readout(int handle, char **buffer){
  uint32_t size;
  // Allocate memory for the readout buffer.
  if(CAEN_DGTZ_MallocReadoutBuffer(handle, buffer, &size)){
    printf("Error: Malloc Readout Buffer\n");
    return -1;
    
  }
  printf("Readout buffer malloc: %u byte\n",size);
  
  return 0;  
}

int v1730_malloc_event(int handle, int N_FADC_BOARD, void **event){
  uint32_t size;
  // Allocate memory for the events.
#ifdef USE_ZLE
  for(int board=0;board<N_FADC_BOARD;board++){
    if(CAEN_DGTZ_MallocZLEEvents(handle, &event[board], &size)){
      printf("Error: Malloc ZLE Event Buffer\n");
      return -1;    
    }
  }
#else
  for(int board=0;board<N_FADC_BOARD;board++){
    //if(CAEN_DGTZ_AllocateEvent(handle, &event[board])){
    if(CAEN_DGTZ_MallocDPPEvents(handle, event, &size)){
      printf("Error: Malloc DGTZ Event Buffer\n");
      return -1;
    }
  }
#endif
  printf("Event buffer malloc\n");
  
  return 0;  
}

void v1730_stop_acquisition(int *handle, int N_FADC_BOARD, V1730_Link *Link){
  int board;
  for(board=0;board<N_FADC_BOARD;board++){
    if(Link[board].SyncEnable==0 || Link[board].BoardIndex==0){
      CAEN_DGTZ_SWStopAcquisition(handle[board]);
      printf("Board:%d Acquisition stop\n",board);
    }
  }
}

void v1730_check_AC_status(int *handle, int N_FADC_BOARD, V1730_Link *Link){
  int board;
  uint32_t regval;
  for(board=0;board<N_FADC_BOARD;board++){
    CAEN_DGTZ_ReadRegister(handle[board],0x8104,&regval);
    if(!(regval>>2 & 0x01)){
      printf("\x1b[31m");
      printf("Error@board %d: acquisition is not running\n",Link[board].BoardIndex);
      printf("\x1b[39m");
    }
  }
}

int v1730_check_init(int handle){
  uint32_t regval;
  CAEN_DGTZ_ReadRegister(handle,0x811C,&regval);
  if(regval==0x00000000){
    printf("\x1b[31m");
    printf("Error: This digitizer is not initialized.\n");
    printf("\x1b[39m");
    return -1;
  }
  return 0;
}

void v1730_get_MaxM_index(int *handle, int N_BOARD, int *BLTIndex,
			  int *RLIndex, int *MSizeIndex, int *MSizeSum){
  uint32_t BLTn,MaxBLTn=0;
  uint32_t RL,MaxRL=0;
  uint32_t MSize,MaxMSize=0;
  for(int i=0;i<N_BOARD;i++){
    CAEN_DGTZ_GetMaxNumEventsBLT(handle[i], &BLTn);
    if (BLTn > MaxBLTn) { MaxBLTn = BLTn; *BLTIndex = i; }
    CAEN_DGTZ_GetRecordLength(handle[i], &RL);
    if (RL > MaxRL) { MaxRL = RL; *RLIndex = i; }
    if ((MSize = CheckMallocSize(handle[i])) > MaxMSize)
      { MaxMSize = MSize; *MSizeIndex = i; }
    *MSizeSum+=MSize;
  }
}

/* Open Digitizer and Get Board Info*/
int open_DGTZ(V1730_Link *Link, int *handle, CAEN_DGTZ_BoardInfo_t *BoardInfo){
  int ret,MajorNumber;
  int LinkNum=Link->LinkNum, ConetNode=Link->ConetNode;
  ret=CAEN_DGTZ_OpenDigitizer(CAEN_DGTZ_OpticalLink, LinkNum, ConetNode, 0, handle);
  if(ret!=CAEN_DGTZ_Success){
    printf("\n***** Error opening the digitizer ret=%d *****\n",ret);
    return -1;
  }
  ret=CAEN_DGTZ_GetInfo(*handle, BoardInfo);
  if(ret!=CAEN_DGTZ_Success){
    printf("\n***** Error getinfo the digitizer ret=%d *****\n",ret);
    return -1;
  }
  printf("Board:%d, S.Num:%d\n",Link->BoardIndex,BoardInfo->SerialNumber);
  printf("  ROC FPGA:%s\n",BoardInfo->ROC_FirmwareRel);
  printf("  AMC FPGA:%s\n",BoardInfo->AMC_FirmwareRel);
  MajorNumber=atoi(BoardInfo->AMC_FirmwareRel);
  DB(printf("  MajorNumber=%d\n",MajorNumber));
  switch(MajorNumber){
  case STANDARD_FW_CODE:
    printf("  waveform recording firmware\n");
    break;
  case V1730_DPP_PSD_CODE:
    printf("  V1730 DPP-PSD firmware\n");
    break;
  case V1730_DPP_PHA_CODE:
    printf("  V1730 DPP-PHA firmware\n");
    break;
  case V1730_DPP_ZLE_CODE:
    printf("  V1730 DPP-ZLE firmware\n");
    break;
  case V1730_DPP_DAW_CODE:
    printf("  V1730 DPP-DAW firmware\n");
    break;
  default:
    printf("  This digitizer is not V1730.");
  }
#ifdef USE_ZLE
  if(atoi(BoardInfo->AMC_FirmwareRel)!=140){
    printf("Error: This digitizer doesn't have ZLE firmware\n");
    return -1;
  }
#else
  /*
  if(atoi(BoardInfo->AMC_FirmwareRel)>=128){
    printf("Error: This digitizer has a DPP firmware\n");
    return -1;
  }
  */
#endif
  
  return 0;
}

int init_DGTZ(int handle, CAEN_DGTZ_BoardInfo_t BoardInfo){
  int ch;
  uint32_t mask=0b11;
  uint32_t val;
  CAEN_DGTZ_Reset(handle);
  //v1730_test(handle);
  CAEN_DGTZ_SetRecordLength(handle, 1024);
  CAEN_DGTZ_SetPostTriggerSize(handle, 50);
  CAEN_DGTZ_SetIOLevel(handle, 0);
  CAEN_DGTZ_SetMaxNumEventsBLT(handle, 1);
  CAEN_DGTZ_SetAcquisitionMode(handle, CAEN_DGTZ_SW_CONTROLLED);
  CAEN_DGTZ_SetExtTriggerInputMode(handle, CAEN_DGTZ_TRGMODE_ACQ_ONLY);
  CAEN_DGTZ_SetChannelEnableMask(handle, mask);

  val = (uint32_t)((float)(fabs((((float)10 - 0) / 1) - 100.))*(655.35));
  if (val > 65535) val = 65535;
  if (val < 0) val = 0;
  for(ch=0;ch<16;ch++){
    if(mask & (1<<ch)){
      CAEN_DGTZ_SetChannelDCOffset(handle, (uint32_t)ch, val);
      CAEN_DGTZ_SetChannelSelfTrigger(handle, CAEN_DGTZ_TRGMODE_DISABLED, (1<<ch));
      CAEN_DGTZ_SetChannelTriggerThreshold(handle, ch, 50);
      CAEN_DGTZ_SetTriggerPolarity(handle, ch, CAEN_DGTZ_PulsePolarityPositive);
    }
  }

  v1730_calib(handle);
  
  return 0;
}

int CheckBoardFailureStatus(int handle, CAEN_DGTZ_BoardInfo_t BoardInfo) {
  int ret = 0;
  uint32_t status = 0;
  ret = CAEN_DGTZ_ReadRegister(handle, 0x8104, &status);
  if (ret != 0) {
    printf("Error: Unable to read board failure status.\n");
    return -1;
  }
  
  usleep(200000);
  
  //read twice (first read clears the previous status)
  ret = CAEN_DGTZ_ReadRegister(handle, 0x8104, &status);
  if (ret != 0) {
    printf("Error: Unable to read board failure status.\n");
    return -1;
  }
  
  if(!(status & (1 << 7))) {
    printf("Board error detected: PLL not locked.\n");
    return -1;
  }
  
  return 0;
}

int CheckRegister(int handle){
  int ret=0,ch;
  uint32_t data,regval;
  //Readout Control
  ret|=CAEN_DGTZ_ReadRegister(handle, 0xEF00, &data);
  DB(printf("Reg Readout_Control=%u\n",data));
  //Acquisition Status
  ret|=CAEN_DGTZ_ReadRegister(handle, 0x8104, &data);
  DB(printf("Reg Acquisition_Status=%u\n",data));
  //Acquisition Control
  ret|=CAEN_DGTZ_ReadRegister(handle, 0x8100, &data);
  DB(printf("Reg Acquisition_Control=%u\n",data));
  //Channel n Status
  for(ch=0;ch<16;ch++){
    regval= 0x1088 | (ch<<8);
    ret|=CAEN_DGTZ_ReadRegister(handle, regval, &data);
    DB(printf("Reg Channel_%02d_status=%u\n",ch,data));
  }

  if(ret) printf("Error: Check Register\n");
  
  return ret;
}

/* Get Dgitizer Board Info */
int get_DGTZ_info(int handle, CAEN_DGTZ_BoardInfo_t *BoardInfo){
  if(CAEN_DGTZ_GetInfo(handle, BoardInfo)!=CAEN_DGTZ_Success){
    printf("\n\n Error getinfo the digitizer\n");
    return 0;
  }
  printf("SerialNum=%d\n",BoardInfo->SerialNumber);
  return 1;
}

int close_DGTZ(char **buffer, int handle, void **Event){
  // free readout buffer (if allocated)
  if(buffer!=NULL){
    CAEN_DGTZ_FreeReadoutBuffer(buffer);
  }
  // free events (if allocated)
  if (Event != NULL) {
#ifdef USE_ZLE
    CAEN_DGTZ_FreeZLEEvents(handle,(void**)Event);
#else
    //CAEN_DGTZ_FreeEvent(handle,(void**)Event);
    CAEN_DGTZ_FreeDPPEvents(handle,(void**)Event);
#endif
  }

  CAEN_DGTZ_CloseDigitizer(handle);
  return 0;
}

void DGTZ_enable_interrupt(int handle){
  int ret;
  ret=CAEN_DGTZ_SetInterruptConfig(handle, CAEN_DGTZ_ENABLE, 1, 0xAAAA, 1, CAEN_DGTZ_IRQ_MODE_RORA);
  DB(printf("\n Digitizer interrupt enable\n"));
  DB(printf("ena_int ret=%d\n",ret));  
}

void DGTZ_disable_interrupt(int handle){
  int ret;
  ret=CAEN_DGTZ_SetInterruptConfig(handle, CAEN_DGTZ_DISABLE, 1, 0xAAAA, 1, CAEN_DGTZ_IRQ_MODE_RORA);
  DB(printf("dis_int ret=%d\n",ret));
}

/** return 0=no interrupt, 1=is intterupt */
int DGTZ_check_interrupt(int handle){
  int ret=0;
  if(CAEN_DGTZ_IRQWait(handle,200)!=CAEN_DGTZ_Success){
  }
  else ret=1;
  
  return ret;
}
/**************************************************/

/* initialization for ZLE firmware*/
int init_ZLE(int handle, CAEN_DGTZ_BoardInfo_t BoardInfo, V1730_param *ConfigVar, V1730_Link *Link,
	     int N_FADC_BOARD){
  uint32_t regval;
  int ch;
  uint32_t ChannelEnableMask=ConfigVar->ChEnableMask;
  int SyncEnable=Link->SyncEnable;
  int board=Link->BoardIndex;
  int selftrig=Link->SelfTrig;

  CAEN_DGTZ_Reset(handle);

  DB(printf("**** initial value ****\n"));
  CheckRegister(handle);
  DB(printf("*******************************\n"));
  
  CAEN_DGTZ_SetIOLevel(handle, 0);
  CAEN_DGTZ_SetSWTriggerMode(handle, CAEN_DGTZ_TRGMODE_DISABLED);
  CAEN_DGTZ_SetExtTriggerInputMode(handle, CAEN_DGTZ_TRGMODE_ACQ_AND_EXTOUT);
  ZLE_SetTriggerMask(handle, ConfigVar->ChTrigMask);
  ZLE_SetTriggerOutMask(handle, ConfigVar->ChTrigMask);
  // Active channels
  CAEN_DGTZ_SetChannelEnableMask(handle, ChannelEnableMask);
  // gain control
  CAEN_DGTZ_WriteRegister(handle, 0x8028, ConfigVar->gain);
  // record length
  CAEN_DGTZ_SetRecordLength(handle, ConfigVar->RecordLength);
  // max BLT events
  CAEN_DGTZ_SetMaxNumEventsBLT(handle, ConfigVar->MaxBLTEvents);
  // sets whether the LVDS quartets are input or output (bits [5:2]):
  // 1st quartet is input, other outputs here
  // sets the LVDS "new" mode (bit 8)
  // TRG OUT is used to propagate signals (bits [17:16])
  // signal propagated through the trgout (bits [19:18]) is the busy signal
  // enable extended time-stamp (bits [22:21] = "10")
  // the other two quartets (not used) are also set to output
  CAEN_DGTZ_ReadRegister(handle, 0x811C, &regval);
  CAEN_DGTZ_WriteRegister(handle, 0x811C, ConfigVar->FrontIO);
  //CAEN_DGTZ_WriteRegister(handle, 0x811C, regval | 0x4d0138);
  //CAEN_DGTZ_WriteRegister(handle, 0x811C, regval | 0x4c0138);
  
  if(SyncEnable==1){
    // acquisition mode is sw-controlled for the first board, LVDS-controlled for the others
    CAEN_DGTZ_ReadRegister(handle, 0x8100, &regval);
    if(board==0){
      CAEN_DGTZ_WriteRegister(handle, 0x8100, regval | 0x00000100);
      printf("board=%d_0x8100status=%u\n",board,regval|0x00000100);
    }
    else{
      CAEN_DGTZ_WriteRegister(handle, 0x8100, regval | 0x00000107);
      printf("board=%d_0x8100status=%u\n",board,regval|0x00000107);
    }
      //CAEN_DGTZ_WriteRegister(handle, 0x8100, regval | (board == 0 ? 0x00000120 : 0x00000127));
    // register 0x816C: reduces the threshold at which the BUSY is raised
    // at 2^buffer organization-20 events
    CAEN_DGTZ_ReadRegister(handle, 0x800C, &regval);
    CAEN_DGTZ_WriteRegister(handle, 0x816C, (uint32_t)(pow(2., regval) - 20));
    // register 0x8170: timestamp offset
    CAEN_DGTZ_WriteRegister(handle, 0x8170, 3*(N_FADC_BOARD - board - 1) + (board == 0 ? -1 : 0));
    // register 0x81A0: select the lowest two quartet as "nBUSY/nVETO" type.
    // BEWARE: set ALL the quartet bits to 2
    CAEN_DGTZ_ReadRegister(handle, 0x81A0, &regval);
    //CAEN_DGTZ_WriteRegister(handle, 0x81A0, regval | 0x00002222);
    CAEN_DGTZ_WriteRegister(handle, 0x81A0, regval | ConfigVar->LVDSMode);
  }else {
    // enable extended timestamp (bits [22:21] = "10")
    CAEN_DGTZ_ReadRegister(handle, 0x811C, &regval);
    CAEN_DGTZ_WriteRegister(handle, 0x811C, regval | 0x400000);
    // set acquisition mode
    CAEN_DGTZ_SetAcquisitionMode(handle, CAEN_DGTZ_SW_CONTROLLED);
    // register 0x8100: set bit 2 to 1 if not in sw-controlled mode
    CAEN_DGTZ_ReadRegister(handle, 0x8100, &regval);
    CAEN_DGTZ_WriteRegister(handle, 0x00008100, regval | 0x000100);
  }

  if(selftrig==1){
    CAEN_DGTZ_ReadRegister(handle, 0x81A0, &regval);
    CAEN_DGTZ_WriteRegister(handle, 0x81A0, regval | 0x00010000);
  }
  
  // channel-specific settings
  for(ch=0;ch<16;ch++){
    //trigger generation mode from channel couples
    if ((ch % 2) == 0)
      ZLE_SetTrigCoupleLogic(handle, (int)(ch / 2), 3);
    if(ChannelEnableMask & (1<<ch)){
      // set DC offset
      CAEN_DGTZ_SetChannelDCOffset(handle, ch, (int)((ConfigVar->DCOffset + 50) * 65535 / 100));
      // pretrigger
      ZLE_SetPreTrigger(handle, ConfigVar->PreTrig, ch);
      //ZLE baseline register
      ZLE_SetBLineMode(handle, ConfigVar->BLine[0], ch);
      ZLE_SetBLineDefValue(handle, ConfigVar->BLine[1], ch);
      ZLE_SetBLineNoise(handle, ConfigVar->BLine[2], ch);
      //NSampBack 
      ZLE_SetPreSamples(handle, ConfigVar->NSample[0], ch);
      //NSampAhead
      ZLE_SetPostSamples(handle, ConfigVar->NSample[1], ch);
      //ZLE Threshold
      ZLE_SetDataThreshold(handle, ConfigVar->ZLEThreshold, ch);
      //ZLE Trigger Threshold
      ZLE_SetTriggerThreshold(handle, ConfigVar->TrigThreshold, ch);
      //ZLE signal logic register
      ZLE_SetPulsePolarity(handle, ConfigVar->Polarity, ch);
      // collect the whole event, independently from threshold or trigger polarity
      ZLE_NoThreshold(handle, ConfigVar->NoThreshold , ch);
      // test pulse emulator
      ZLE_SetTestPulse(handle, 0, 6, 3, 0, ch);
    }
  }
  
  // ADC calibration
  CAEN_DGTZ_WriteRegister(handle, 0x809c, 0x1);

  DB(printf("**** after initialize value ****\n"));
  CheckRegister(handle);
  DB(printf("*******************************\n"));

  printf("** initialized board:%d\n",Link->BoardIndex);
  
  return 1;
}

int ZLE_SetTriggerMask(int handle, uint32_t mask){
  int ret=0;
  uint32_t regvalue;
  if (mask < 0x100) {
    ret = CAEN_DGTZ_ReadRegister(handle, 0x810C, &regvalue);
    ret |= CAEN_DGTZ_WriteRegister(handle, 0x810C, (regvalue & ~(0x000000ff)) | (uint32_t)(mask));
  }
  else {
    printf("invalid value for channel couple trigger mask\n");
  }
  return ret;
}

int ZLE_SetTriggerOutMask(int handle, uint32_t mask) {
  int ret=0;
  uint32_t regvalue;
  if (mask < 0x100) {
    ret = CAEN_DGTZ_ReadRegister(handle, 0x8110, &regvalue);
    ret |= CAEN_DGTZ_WriteRegister(handle, 0x8110,(regvalue & ~(0x000000ff)) | (uint32_t)(mask));
  } else {
    printf("invalid value for channel couple trigger mask\n");
  }
  return ret;
}

int ZLE_SetTrigCoupleLogic(int handle, int couple, uint32_t logic) {
  int ret=0;
  uint32_t regvalue;
  if (logic < 4) {
    ret = CAEN_DGTZ_ReadRegister(handle, 0x1068, &regvalue);
    regvalue = (regvalue & (uint32_t)(~(0x00000003 << (2 * couple)))) | ((uint32_t)(logic << (2 * couple)));
    // replace only the two bits affecting the selected couple's logic.
    ret |= CAEN_DGTZ_WriteRegister(handle, 0x8068,regvalue);
  } else {
    printf("invalid value for trigger logic of couple %d\n", couple);
  }
  return ret;
}

int ZLE_SetPreTrigger(int handle, uint32_t samples, int channel) {
  int ret=0;
  if (samples < 256) {
    ret = CAEN_DGTZ_WriteRegister(handle, 0x1038 | (channel << 8), samples);
    DB(printf("pretrigger sample number of channel %d: %d\n", channel,samples));
  } else {
    printf("invalid value for pretrigger sample number of channel %d\n", channel);
  }
  return ret;
}

int ZLE_SetBLineMode(int handle,uint32_t mode,int channel) {
  int ret=0;
  uint32_t regvalue;
  ret = CAEN_DGTZ_ReadRegister(handle, 0x1034 | (channel << 8), &regvalue);
  if (mode) ret |= CAEN_DGTZ_WriteRegister(handle, 0x1034 | (channel << 8), regvalue | (uint32_t)(1 << 24));
  else ret |= CAEN_DGTZ_WriteRegister(handle, 0x1034 | (channel << 8), (regvalue & (~(uint32_t)(1 << 24))));
  return ret;
}

int ZLE_SetBLineDefValue(int handle, uint32_t bl, int channel) {
  int ret=0;
  uint32_t regvalue;
  if (bl < 16384) {
    ret = CAEN_DGTZ_ReadRegister(handle, 0x1034 | (channel << 8), &regvalue);
    regvalue = (regvalue & (uint32_t)(~(0x00003fff))) | (uint32_t)(bl & 0x3fff);
    // replace only the two bits affecting the selected couple's logic.
    ret |= CAEN_DGTZ_WriteRegister(handle, 0x1034 | (channel << 8), regvalue);
  } else {
    printf("invalid value for default baseline of channel %d\n", channel);
  }
  return ret;
}

int ZLE_SetBLineNoise(int handle, uint32_t noise, int channel) {
  int ret=0;
  uint32_t regvalue;
  if (noise < 256) {
    ret = CAEN_DGTZ_ReadRegister(handle, 0x1034 | (channel << 8), &regvalue);
    regvalue = (regvalue & (uint32_t)(~(0x00ff0000))) | (uint32_t)(noise << 16);
    // replace only the two bits affecting the selected couple's logic.
    ret |= CAEN_DGTZ_WriteRegister(handle, 0x1034 | (channel << 8), regvalue);
  } else {
    printf("invalid value for baseline noise of channel %d\n", channel);
  }
  return ret;
}

int ZLE_SetPreSamples(int handle,uint32_t samples, int channel) {
  int ret=0;
  if (samples < 256) {
    ret = CAEN_DGTZ_WriteRegister(handle, 0x1054 | (channel << 8),samples);
    DB(printf("pre-signal sample number of channel %d: %d\n", channel,samples));
  } else {
    printf("invalid value for pre-signal sample number of channel %d\n",channel);
  }
  return ret;
}

int ZLE_SetPostSamples(int handle, uint32_t samples, int channel) {
  int ret=0;
  if (samples < 256) {
    ret = CAEN_DGTZ_WriteRegister(handle, 0x1058 | (channel << 8),samples);
    DB(printf("post-signal sample number of channel %d: %d\n", channel,samples));
  } else {
    printf("invalid value for post-signal sample number of channel %d\n",channel);
  }
  return ret;
}

int ZLE_SetDataThreshold(int handle, uint16_t threshold, int channel) {
  int ret=0;
  if (threshold < 16384) {
    ret = CAEN_DGTZ_WriteRegister(handle, 0x105C | (channel << 8), (uint32_t)(threshold & 0x3FFF));
    DB(printf("data threshold of channel %d: %d\n", channel,threshold));
  } else {
    printf("invalid value for data threshold of channel %d\n", channel);
  }
  return ret;
}

int ZLE_SetTriggerThreshold(int handle, uint16_t threshold, int channel) {
  int ret=0;
  if (threshold < 16384) {
    ret = CAEN_DGTZ_WriteRegister(handle, 0x1060 | (channel << 8), (uint32_t)(threshold & 0x3FFF));
    DB(printf("trigger threshold of channel %d: %d\n",channel, threshold));
  } else {
    printf("invalid value for trigger threshold of channel %d\n", channel);
  }
  return ret;
}

int ZLE_SetPulsePolarity(int handle, uint32_t polarity, int channel) {
  int ret=0;
  uint32_t regvalue;
  ret = CAEN_DGTZ_ReadRegister(handle, 0x1064 | (channel << 8), &regvalue);
  (polarity) ? (regvalue = regvalue | 0x00000100) : (regvalue = regvalue & ~(0x00000100));
  ret = CAEN_DGTZ_WriteRegister(handle, 0x1064 | (channel << 8), regvalue);
  return ret;
}

int ZLE_NoThreshold(int handle, int enable, int channel) {
  int ret;
  uint32_t regvalue;
  ret = CAEN_DGTZ_ReadRegister(handle, 0x1064 | (channel << 8), &regvalue);
  if(enable==1)
    ret = CAEN_DGTZ_WriteRegister(handle, 0x1064 | (channel << 8), regvalue | (uint32_t)(0x80));
  else
    ret = CAEN_DGTZ_WriteRegister(handle, 0x1064 | (channel << 8), regvalue & 0xffffff7f);    
  return ret;
}

int ZLE_SetTestPulse(int handle, int enable, uint32_t rate, uint32_t scale, uint32_t polarity, int channel) {
  int ret=0;
  uint32_t regvalue;
  ret = CAEN_DGTZ_ReadRegister(handle, 0x1064 | (channel << 8), &regvalue);
  if(enable==1){
    ret = CAEN_DGTZ_WriteRegister(handle, 0x1064 | (channel << 8), regvalue | (uint32_t)(0x1));
    if (rate < 8) {
      ret |= CAEN_DGTZ_WriteRegister(handle, 0x1064 | (channel << 8), (regvalue & ~(0x0000000e)) | (uint32_t)(rate << 1));
    }
    else {
      printf("invalid value for test pulse rate channel %d\n", channel);
    }
    if (scale<4) {
      ret |= CAEN_DGTZ_WriteRegister(handle, 0x1064 | (channel << 8), (regvalue & ~(0x00000030)) | (uint32_t)(scale << 4));
    } else {
      printf("invalid value for test pulse scale channel %d\n", channel);
    }
    ret = CAEN_DGTZ_ReadRegister(handle, 0x1064 | (channel << 8), &regvalue);
    (polarity) ? (regvalue = regvalue | 0x00000040) : (regvalue = regvalue & ~(0x00000040));
    ret = CAEN_DGTZ_WriteRegister(handle, 0x1064 | (channel << 8),regvalue);
  }
  else
    ret = CAEN_DGTZ_WriteRegister(handle, 0x1064 | (channel << 8), regvalue & 0xfffffffe);    
  return ret;
}

uint32_t CheckMallocSize(int handle) {
  uint32_t size, lsize, maxEvents, nChEnabled = 0, u32;
  if (((CAEN_DGTZ_GetRecordLength(handle, &lsize)) != CAEN_DGTZ_Success) ||
      ((CAEN_DGTZ_GetChannelEnableMask(handle, &u32)) != CAEN_DGTZ_Success) ||
      ((CAEN_DGTZ_GetMaxNumEventsBLT(handle, &maxEvents)) != CAEN_DGTZ_Success)) return 0;
  for (; u32 > 0; u32 >>= 1) if ((u32 & 0x1) != 0) nChEnabled++;
  size = (4 * ((((lsize * 2) + 1)*nChEnabled) + 4)*maxEvents);
  return size;
}

#if 1
/* initialization for PHA firmware*/
int init_PHA(int handle, CAEN_DGTZ_BoardInfo_t BoardInfo, V1730_PHA_param *ConfigVar, V1730_Link *Link,
	     CAEN_DGTZ_DPP_PHA_Params_t DPPParams, int N_FADC_BOARD)
{
  int i, ret = 0;
  uint32_t regval;
  int SyncEnable=Link->SyncEnable;
  int board=Link->BoardIndex;
  int selftrig=Link->SelfTrig;
  
  /* Reset the digitizer */
  ret |= CAEN_DGTZ_Reset(handle);

  if (ret) {
    printf("ERROR: can't reset the digitizer.\n");
    return -1;
  }
  DB(printf("**** initial value ****\n"));
  CheckRegister(handle);
  DB(printf("*******************************\n"));

  // Board Configuration
  ret |= CAEN_DGTZ_WriteRegister(handle, 0x8000, 0x01000114);
  // Set the I/O level (CAEN_DGTZ_IOLevel_NIM or CAEN_DGTZ_IOLevel_TTL)
  ret |= CAEN_DGTZ_SetIOLevel(handle, CAEN_DGTZ_IOLevel_NIM);
  // Front Panel LVDS I/O
  CAEN_DGTZ_ReadRegister(handle, 0x81A0, &regval);
  CAEN_DGTZ_WriteRegister(handle, 0x81A0, regval | 0x0222);
  CAEN_DGTZ_ReadRegister(handle, 0x811C, &regval);
  CAEN_DGTZ_WriteRegister(handle, 0x811C, regval | 0x004d0138);

  ret |= CAEN_DGTZ_SetSWTriggerMode(handle, CAEN_DGTZ_TRGMODE_DISABLED);
  /* Set the digitizer's behaviour when an external trigger arrives:
     CAEN_DGTZ_TRGMODE_DISABLED: do nothing
     CAEN_DGTZ_TRGMODE_EXTOUT_ONLY: generate the Trigger Output signal
     CAEN_DGTZ_TRGMODE_ACQ_ONLY = generate acquisition trigger
     CAEN_DGTZ_TRGMODE_ACQ_AND_EXTOUT = generate both Trigger Output and acquisition trigger
     see CAENDigitizer user manual, chapter "Trigger configuration" for details */
  ret |= CAEN_DGTZ_SetExtTriggerInputMode(handle, CAEN_DGTZ_TRGMODE_ACQ_ONLY);
  // Set the enabled channels
  ret |= CAEN_DGTZ_SetChannelEnableMask(handle, ConfigVar->ChannelMask);
  // Set the number of samples for each waveform
  ret |= CAEN_DGTZ_SetRecordLength(handle, ConfigVar->RecordLength);
  /* Set the DPP acquisition mode
     This setting affects the modes Mixed and List (see CAEN_DGTZ_DPP_AcqMode_t definition for details)
     CAEN_DGTZ_DPP_SAVE_PARAM_EnergyOnly        Only energy (DPP-PHA) or charge (DPP-PSD/DPP-CI v2) is returned
     CAEN_DGTZ_DPP_SAVE_PARAM_TimeOnly        Only time is returned
     CAEN_DGTZ_DPP_SAVE_PARAM_EnergyAndTime    Both energy/charge and time are returned
     CAEN_DGTZ_DPP_SAVE_PARAM_None            No histogram data is returned */
  ret |= CAEN_DGTZ_SetDPPAcquisitionMode(handle, ConfigVar->DPPmode, CAEN_DGTZ_DPP_SAVE_PARAM_EnergyAndTime);
  // Set how many events to accumulate in the board memory before being available for readout
  ret |= CAEN_DGTZ_SetDPPEventAggregation(handle, ConfigVar->EventAggr, 0);
  // Set the DPP specific parameters for the channels in the given channelMask
  ret |= CAEN_DGTZ_SetDPPParameters(handle, ConfigVar->ChannelMask, &DPPParams);

  if(SyncEnable==1){
    // acquisition mode is sw-controlled for the first board, LVDS-controlled for the others
    CAEN_DGTZ_ReadRegister(handle, 0x8100, &regval);
    if(board==0) CAEN_DGTZ_WriteRegister(handle, 0x8100, regval | 0x00000100);
    else CAEN_DGTZ_WriteRegister(handle, 0x8100, regval | 0x00000107);
  }
  else{
    // set acquisition mode
    CAEN_DGTZ_SetAcquisitionMode(handle, CAEN_DGTZ_SW_CONTROLLED);
  }
        
  /* Set the mode used to syncronize the acquisition between different boards.
     In this example the sync is disabled */
  //ret |= CAEN_DGTZ_SetRunSynchronizationMode(handle, CAEN_DGTZ_RUN_SYNC_Disabled);    
    
  for(i=0; i<16; i++) {
    if (ConfigVar->ChannelMask & (1<<i)) {
      // Set a DC offset to the input signal to adapt it to digitizer's dynamic range
      ret |= CAEN_DGTZ_SetChannelDCOffset(handle, i, 0x8000);
            
      // Set the Pre-Trigger size (in samples)
      ret |= CAEN_DGTZ_SetDPPPreTriggerSize(handle, i, 1000);
            
      // Set the polarity for the given channel (CAEN_DGTZ_PulsePolarityPositive or CAEN_DGTZ_PulsePolarityNegative)
      ret |= CAEN_DGTZ_SetChannelPulsePolarity(handle, i, ConfigVar->Polarity);
    }
  }

  /* Set the virtual probes settings
     DPP-PHA can save:
     2 analog waveforms:
     the first and the second can be specified with the  ANALOG_TRACE 1 and 2 parameters
        
     2 digital waveforms:
     the first can be specified with the DIGITAL_TRACE_1 parameter
     the second  is always the trigger

     CAEN_DGTZ_DPP_VIRTUALPROBE_SINGLE	 -> Save only the ANALOG_TRACE_1 waveform
     CAEN_DGTZ_DPP_VIRTUALPROBE_DUAL      -> Save also the waveform specified in  ANALOG_TRACE_2

     Virtual Probes 1 types:
     CAEN_DGTZ_DPP_VIRTUALPROBE_Input
     CAEN_DGTZ_DPP_VIRTUALPROBE_Delta
     CAEN_DGTZ_DPP_VIRTUALPROBE_Delta2
     CAEN_DGTZ_DPP_VIRTUALPROBE_Trapezoid
    
     Virtual Probes 2 types:
     CAEN_DGTZ_DPP_VIRTUALPROBE_Input
     CAEN_DGTZ_DPP_VIRTUALPROBE_Threshold
     CAEN_DGTZ_DPP_VIRTUALPROBE_TrapezoidReduced
     CAEN_DGTZ_DPP_VIRTUALPROBE_Baseline
     CAEN_DGTZ_DPP_VIRTUALPROBE_None

     Digital Probes types:
     CAEN_DGTZ_DPP_DIGITALPROBE_TRGWin
     CAEN_DGTZ_DPP_DIGITALPROBE_Armed
     CAEN_DGTZ_DPP_DIGITALPROBE_PkRun
     CAEN_DGTZ_DPP_DIGITALPROBE_PileUp
     CAEN_DGTZ_DPP_DIGITALPROBE_Peaking
     CAEN_DGTZ_DPP_DIGITALPROBE_CoincWin
     CAEN_DGTZ_DPP_DIGITALPROBE_BLFreeze
     CAEN_DGTZ_DPP_DIGITALPROBE_TRGHoldoff
     CAEN_DGTZ_DPP_DIGITALPROBE_TRGVal
     CAEN_DGTZ_DPP_DIGITALPROBE_ACQVeto
     CAEN_DGTZ_DPP_DIGITALPROBE_BFMVeto
     CAEN_DGTZ_DPP_DIGITALPROBE_ExtTRG
     CAEN_DGTZ_DPP_DIGITALPROBE_Busy
     CAEN_DGTZ_DPP_DIGITALPROBE_PrgVeto*/

  ret |= CAEN_DGTZ_SetDPP_VirtualProbe(handle, ANALOG_TRACE_1, CAEN_DGTZ_DPP_VIRTUALPROBE_Delta2);
  ret |= CAEN_DGTZ_SetDPP_VirtualProbe(handle, ANALOG_TRACE_2, CAEN_DGTZ_DPP_VIRTUALPROBE_Input);
  ret |= CAEN_DGTZ_SetDPP_VirtualProbe(handle, DIGITAL_TRACE_1, CAEN_DGTZ_DPP_DIGITALPROBE_Peaking);

  if (ret) {
    printf("Warning: errors found during the programming of the digitizer.\nSome settings may not be executed\n");
    return ret;
  } else {
    return 0;
  }
}
#endif
