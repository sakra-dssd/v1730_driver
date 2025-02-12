#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <CAENDigitizer.h>
#include "v1730.h"

V1730_Link *Link=NULL;
V1730_param DefaConfigVar;
V1730_param ConfigVar;
int N_FADC_BOARD=-1;
int N_BOARD=-1;
int Handler=-1;
CAEN_DGTZ_BoardInfo_t BoardInfo;
int SelfTrigMode=0;

int main(int argc, char *argv[]){
  int board;
  uint32_t regval;
  char f_name[100];

  Link=ConnectionConfigFile("connection.cfg",&N_FADC_BOARD,&N_BOARD,2); 
  
  if(Link==NULL){
    printf("Error: can't set connection config\n");
    return 0;
  }
  printf("N_FADC=%d, N_BOARD=%d\n",N_FADC_BOARD,N_BOARD);
  for(int i=0;i<N_BOARD;++i){
    printf("board:%d index:%d link:%d conet:%d\n",i,
	   Link[i].BoardIndex,Link[i].LinkNum,Link[i].ConetNode);
  }
  
  /*
  printf("*** use %d FADC BOARD***\n",N_FADC_BOARD);
  
  ParseConfigFile("default.cfg",&DefaConfigVar);

  for(board=0;board<N_FADC_BOARD;board++){
    if(open_DGTZ(Link[board],&Handler,&BoardInfo)==-1) return 0;
    printf("Board %d: S.Num=%d AMC FPGA=%s\n",
	   board,BoardInfo.SerialNumber,BoardInfo.AMC_FirmwareRel);
    if(CheckBoardFailureStatus(Handler,BoardInfo)) return 0;

    ConfigVar=DefaConfigVar;
    sprintf(f_name,"v1730_board_%d.cfg",board);
    ParseConfigFile(f_name,&ConfigVar);
    if(atoi(BoardInfo.AMC_FirmwareRel)==140){
      printf("This digitizer has ZLE firmware\n");
      init_ZLE(Handler,BoardInfo,&ConfigVar,&Link[board],N_FADC_BOARD);
    }
    else if(!(atoi(BoardInfo.AMC_FirmwareRel)>=128)){
      printf("This digitizer has no DPP firmware\n");
    }

    CAEN_DGTZ_CloseDigitizer(Handler);
  }
  */
  
  return 0;
}
