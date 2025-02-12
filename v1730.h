#ifndef _V1730_H
#define _V1730_H

#define USE_ZLE

typedef struct {
  int BoardIndex;
  int LinkNum;
  int ConetNode;
  int SyncEnable;
  int SelfTrig;
} V1730_Link;

typedef struct {
  uint32_t ChEnableMask;
  uint32_t ChTrigMask;
  int gain;
  int RecordLength;
  int MaxBLTEvents;

  int TrigCoupleLogic[8];
  int DCOffset;
  int PreTrig;
  int BLine[3]; // 0:mode, 1:DefValue, 2:Noise
  int NSample[2]; // 0:pre, 1:post trigger
  int TrigThreshold;
  int ZLEThreshold;
  int Polarity;
  int NoThreshold;
  int TestPulse[4];

  uint32_t FrontIO;
  uint32_t LVDSMode;
  
} V1730_param;

typedef struct{
  uint32_t ChannelMask;
  int RecordLength;
  CAEN_DGTZ_DPP_AcqMode_t DPPmode;
  int Polarity;
  int EventAggr;
} V1730_PHA_param;

V1730_Link* ConnectionConfigFile(char *f_ini, int *N_FADC_BOARD, int *N_BOARD, int efn);
int ParseConfigFile(char *f_ini, V1730_param *ConfigVar);

int v1730_calib(int handle);
int v1730_test(int handle);
void v1730_clear(int handle);
void v1730_read_data(int handle, char *buffer, uint32_t *buffersize);
void v1730_get_event(int handle, char *buffer, uint32_t buffersize, void **event, uint32_t *numeven);
void v1730_start_acquisition(int *handle, int N_FADC_BOARD, V1730_Link *Link);
void v1730_stop_acquisition(int *handle, int N_FADC_BOARD, V1730_Link *Link);
void v1730_check_AC_status(int *handle, int N_FADC_BOARD, V1730_Link *Link);
int v1730_malloc_readout(int handle, char **buffer);
int v1730_malloc_event(int handle, int N_FADC_BOARD, void **event);
int v1730_check_init(int handle);
void v1730_get_MaxM_index(int *handle, int N_BOARD, int *BLTIndex,
			  int *RLIndex, int *MSizeIndex, int *MSizeSum);

int open_DGTZ(V1730_Link *Link, int *handle, CAEN_DGTZ_BoardInfo_t *BoardInfo);
int close_DGTZ(char **buffer, int handle, void **Event);

int init_DGTZ(int handle, CAEN_DGTZ_BoardInfo_t BoardInfo);

int CheckBoardFailureStatus(int handle, CAEN_DGTZ_BoardInfo_t BoardInfo);
int CheckRegister(int handle);

int get_DGTZ_info(int handle, CAEN_DGTZ_BoardInfo_t *BoardInfo);
void DGTZ_enable_interrupt(int handle);
void DGTZ_disable_interrupt(int handle);
int DGTZ_check_interrupt(int handle);

int init_ZLE(int handle, CAEN_DGTZ_BoardInfo_t BoardInfo, V1730_param *ConfigVar, V1730_Link *Link,
	     int N_FADC_BOARD);
int ZLE_SetTriggerMask(int handle, uint32_t mask);
int ZLE_SetTriggerOutMask(int handle, uint32_t mask);
int ZLE_SetTrigCoupleLogic(int handle, int couple, uint32_t logic);
int ZLE_SetPreTrigger(int handle, uint32_t samples, int channel);
int ZLE_SetBLineMode(int handle,uint32_t mode,int channel);
int ZLE_SetBLineDefValue(int handle, uint32_t bl, int channel);
int ZLE_SetBLineNoise(int handle, uint32_t noise, int channel);
int ZLE_SetPreSamples(int handle,uint32_t samples, int channel);
int ZLE_SetPostSamples(int handle, uint32_t samples, int channel);
int ZLE_SetDataThreshold(int handle, uint16_t threshold, int channel);
int ZLE_SetTriggerThreshold(int handle, uint16_t threshold, int channel);
int ZLE_SetPulsePolarity(int handle, uint32_t polarity, int channel);
int ZLE_NoThreshold(int handle, int enable, int channel);
int ZLE_SetTestPulse(int handle, int enable, uint32_t rate, uint32_t scale,
		     uint32_t polarity, int channel);

uint32_t CheckMallocSize(int handle);

int init_PHA(int handle, CAEN_DGTZ_BoardInfo_t BoardInfo, V1730_PHA_param *ConfigVar, V1730_Link *Link,
	     CAEN_DGTZ_DPP_PHA_Params_t DPPParams, int N_FADC_BOARD);


#endif
