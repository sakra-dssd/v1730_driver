/* 
 * example to use libbabies + CAENVMELib
 * H.B. RIKEN
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>

//*** for transfer time measurement ***
#include <time.h>
#include <sys/time.h>
//*************************************

#include "libbbcaenvme.h"
#include "libbabies.h"
#include "bbpid.h"
#include "segidlist.h"

#include "v792.h"
#include "v1290.h"
#include "v1730.h"

#define EB_EFBLOCK_SIZE 0x800000   ///< Usual max size of block data = 8MB

#define DEBUG
#ifdef DEBUG
#define DB(x) x
#else
#define DB(x)
#endif

//#define TTM_DEF 1
#ifdef TTM_DEF
#define TTM(x) x
#else
#define TTM(x)
#endif

//*** for transfer time measurement ***
struct timeval startTime[2], endTime[2], diffTime[2], tempTime[2];
struct timeval sT_div[2], eT_div[2], diffT_div[2], tempT_div[2];
struct timeval sT_daq[2], eT_daq[2], diffT_daq[2], tempT_daq[2];
long long diffsub[2];
//*************************************

int efn = 0;
unsigned int scrbuff[32] = {0};
//char pidpath[] = "/tmp/babies";
//to use babies
char pidpath[] = "/var/run/babies";

int maxbuff=8000;

// v1730 Handler
int N_FADC_BOARD=-1; //total num. of FADC board
int N_BOARD=-1; //num. of FADC board handled in this babies process
int *Handler=NULL;
CAEN_DGTZ_BoardInfo_t *BoardInfo=NULL;

#ifdef USE_ZLE
#define MODULE V1730ZLE
CAEN_DGTZ_730_ZLE_Event_t **Event=NULL;
#else
#define MODULE V1730
CAEN_DGTZ_UINT16_EVENT_t **Event=NULL;
#endif

uint32_t AllocatedSize, BufferSize, NumEvents;
char *buffer = NULL;
V1730_Link *Link=NULL;
int board;
uint32_t regval;
int *EventCounter=NULL;

void quit(void){
  for(board=0;board<N_BOARD;board++){
    close_DGTZ(&buffer,Handler[board],(void**)&Event[board]);
  }
  
  rmpid(pidpath);
  printf("Exit\n");
}

void start(void){
  for(board=0;board<N_BOARD;board++){
    EventCounter[board]=0;
    TTM(diffTime[board].tv_sec=0);
    TTM(diffTime[board].tv_usec=0);
    TTM(diffT_div[board].tv_sec=0);
    TTM(diffT_div[board].tv_usec=0);
    TTM(diffT_daq[board].tv_sec=0);
    TTM(diffT_daq[board].tv_usec=0);    
    TTM(gettimeofday(&sT_daq[board],NULL));
  }
  
  for(board=0;board<N_BOARD;board++){
    EventCounter[board]=0;
    v1730_clear(Handler[board]);
  }
  DGTZ_enable_interrupt(Handler[0]);
  v1730_start_acquisition(Handler,N_BOARD,Link);

  CAEN_DGTZ_ReadRegister(Handler[board], 0x8118, &regval);
  CAEN_DGTZ_WriteRegister(Handler[board], 0x8118, regval | 0xC000);
  CAEN_DGTZ_ReadRegister(Handler[board], 0x8118, &regval);
  CAEN_DGTZ_WriteRegister(Handler[board], 0x8118, regval & ~(1<<14));

  DB(printf("**** check register start *****\n"));
  CheckRegister(Handler[0]);
  DB(printf("*******************************\n"));

  /*
  CAEN_DGTZ_ReadRegister(Handler[0], 0x811C, &regval);
  CAEN_DGTZ_WriteRegister(Handler[0], 0x811C, regval | 0x4000);
  //usleep(1);
  
  CAEN_DGTZ_ReadRegister(Handler[0], 0x811C, &regval);
  CAEN_DGTZ_WriteRegister(Handler[0], 0x811C, regval & ~(1<<14));
  //usleep(1);
  */
  
  printf("Start\n");
}


void stop(void){
  DGTZ_disable_interrupt(Handler[0]);

  for(board=0;board<N_BOARD;board++) v1730_clear(Handler[board]);
  
  DB(printf("**** check register stop ******\n"));
  CheckRegister(Handler[0]);
  DB(printf("*******************************\n"));
  
  for(board=0;board<N_BOARD;board++){
    TTM(gettimeofday(&eT_daq[board],NULL));
    TTM(timersub(&eT_daq[board],&sT_daq[board],&tempT_daq[board]));
    TTM(timeradd(&diffT_daq[board],&tempT_daq[board],&diffT_daq[board]));
    TTM(printf("Transfer Time (sum): %lld(s) %lld(us)\n",
	       (long long)diffTime[board].tv_sec,(long long)diffTime[board].tv_usec));
    TTM(diffsub[board]=1e6*diffTime[board].tv_sec+diffTime[board].tv_usec);
    TTM(printf("Transfer Time: %lf(us)\n",(double)diffsub[board]/EventCounter[board]));
    TTM(printf("Divide Time (sum): %lld(s) %lld(us)\n",
	       (long long)diffT_div[board].tv_sec,(long long)diffT_div[board].tv_usec));
    TTM(diffsub[board]=1e6*diffT_div[board].tv_sec+diffT_div[board].tv_usec);
    TTM(printf("Divide Time: %lf(us)\n",(double)diffsub[board]/EventCounter[board]));
    TTM(printf("Daq Time (sum): %lld(s) %lld(us)\n",
	       (long long)diffT_daq[board].tv_sec,(long long)diffT_daq[board].tv_usec));
    TTM(diffsub[board]=1e6*diffT_daq[board].tv_sec+diffT_daq[board].tv_usec);
    TTM(printf("Daq Time: %lf(us)\n",(double)diffsub[board]/EventCounter[board]));
  } 
 
  printf("Stop\n");
}

void reload(void){
  printf("Reload\n");
}

void sca(void){
  //return;
  //int i;
  //printf("Sca\n");
  /*
  for(i=0;i<32;i++){
    scrbuff[i] = scrbuff[i] + 1;
  }
  babies_init_ncscaler(efn);
  babies_scrdata((char *)scrbuff, sizeof(scrbuff));
  babies_end_ncscaler32();
  */
}

void clear(void){
  // write clear function i.e. send end-of-busy pulse
  /*
  CAEN_DGTZ_ReadRegister(Handler[0], 0x811C, &regval);
  CAEN_DGTZ_WriteRegister(Handler[0], 0x811C, regval | 0x4000);
  //CAEN_DGTZ_ReadRegister(Handler[0], 0x811C, &regval);
  CAEN_DGTZ_WriteRegister(Handler[0], 0x811C, regval & ~(1<<14));
  */
  
  //printf("Clear\n");
}

void read_data(void){
  //clear();
  babies_init_event();
  
  //babies_init_segment(segid)
  //MKSEGID(device, focalplane, detector, module)
  //please see segidlist.h
  //module is important, e.g. C16 is 16bit data
  //device, focalplane, detector, can be defined as you want

  for(board=0;board<N_BOARD;board++){
    babies_init_segment(MKSEGID(0, 0, Link[board].BoardIndex, MODULE));
    TTM(gettimeofday(&startTime[board],NULL));
    v1730_read_data(Handler[board],buffer,&BufferSize);
    TTM(gettimeofday(&endTime[board],NULL));
    TTM(timersub(&endTime[board],&startTime[board],&tempTime[board]));
    TTM(timeradd(&diffTime[board],&tempTime[board],&diffTime[board]));
    if(BufferSize>0){
      v1730_get_event(Handler[board],buffer,BufferSize,(void**)&Event[board],&NumEvents);
      EventCounter[board]+=NumEvents;

      printf("\r numevent=%d  BufferSize=%d Evtn=%d  "
	     ,NumEvents,BufferSize,EventCounter[board]);

    }
    
    memcpy(babies_pt(),buffer,BufferSize);
    babies_segptfx(BufferSize);
    babies_end_segment();

    //v1730_clear(Handler[board]);
  }
  // End of event
  babies_end_event();
  
  // babies_chk_block(int maxbuff)
  // if block size is larger than maxbuff,
  //  data should be send to the event builder
  //  i.e., read scaler and flush
  // example : 8000 = 16kB

  if(babies_chk_block(160000)){
    sca();
    babies_flush();
  }
}

// thread
void evtloop(void){
  int status;
  
  while((status = babies_status()) != -1){
    switch(status){
    case STAT_RUN_IDLE:
      /* noop */
      usleep(1000);
      break;
    case STAT_RUN_START:
    case STAT_RUN_NSSTA:
      if(DGTZ_check_interrupt(Handler[0])){
	/* continue */
	//DB(printf("interrupt ok\n"));
      }else{
	printf("interrupt non\n");
	v1730_check_AC_status(Handler,N_BOARD,Link);
	usleep(100);
	break;
      }

      //printf("evtloop\n");
      read_data();
      
      clear();

      break;
    case STAT_RUN_WAITSTOP:
      v1730_stop_acquisition(Handler,N_BOARD,Link);
      CAEN_DGTZ_ReadRegister(Handler[board], 0x8118, &regval);
      CAEN_DGTZ_WriteRegister(Handler[board], 0x8118, regval & ~(1<<15));

      while(DGTZ_check_interrupt(Handler[0])){
	read_data();
      }

      for(board=0;board<N_BOARD;++board)
	printf("board:%d EventCounter=%d\n",board,EventCounter[board]);
      // for the last sequense of run
      sca();
      babies_flush();
      babies_last_block();
      break;
    default:
      break;
    }
  }

  // write codes to quit safely

}

int main(int argc, char *argv[]){
  int MaxBLTnHIndex,MaxRLHIndex,MaxMSizeHIndex;
  int MaxMSizeSum=0;
  V1730_param DefConf,*Config=NULL;
  char fnam[100];
  char bnam[100];
  if(argc != 2){
    printf("babies EFN\n");
    exit(0);
  }else{
    efn = strtol(argv[1], NULL, 0);
  }

  sprintf(bnam,"babies_%d",efn);
  
  babies_quit(quit);
  babies_start(start);
  babies_stop(stop);
  babies_reload(reload);
  babies_evtloop(evtloop);
  babies_name(bnam);

  babies_init(efn);
  babies_check();

  /* init vme */
#ifdef USE_ZLE
  printf("\n*** ZLE firmware mode ***\n");
#endif
  
  Link=ConnectionConfigFile("connection.cfg",&N_FADC_BOARD,&N_BOARD,efn);
  if(Link==NULL){
    printf("Error: can't set connection config\n");
    return 0;
  }
  printf("*** use %d FADC BOARD***\n",N_BOARD);

  // Allocate space for handles etc. according to the num of boards in the acquisition chain
  EventCounter=(int*)calloc(N_BOARD,sizeof(int));
  Config=(V1730_param*)calloc(N_BOARD,sizeof(V1730_param));
  Handler=(int*)calloc(N_BOARD, sizeof(int));
  BoardInfo=(CAEN_DGTZ_BoardInfo_t*)calloc(N_BOARD, sizeof(CAEN_DGTZ_BoardInfo_t));
#ifdef USE_ZLE
  Event=(CAEN_DGTZ_730_ZLE_Event_t**)calloc(N_BOARD,sizeof(CAEN_DGTZ_730_ZLE_Event_t*));
#else
  Event=(CAEN_DGTZ_UINT16_EVENT_t**)calloc(N_BOARD,sizeof(CAEN_DGTZ_UINT16_EVENT_t*));
#endif
  if(EventCounter==NULL || Handler==NULL || BoardInfo==NULL || Event==NULL){
    printf("Error: Allocate memory space for handler etc.\n");
    return 0;
  }

  ParseConfigFile("default.cfg", &DefConf);
  // Open the digitizer and read the board information
  printf("*** Connected to CAEN Digitizer ***\n");
  for(board=0;board<N_BOARD;board++){
    if(open_DGTZ(&Link[board],&Handler[board],&BoardInfo[board])==-1) return 0;
    if(CheckBoardFailureStatus(Handler[board],BoardInfo[board])) return 0;
    //if(v1730_check_init(Handler[board])==-1) return 0;

    // initialize Digitizer
    Config[board]=DefConf;
    sprintf(fnam,"v1730_board_%d.cfg",Link[board].BoardIndex);
    ParseConfigFile(fnam, &Config[board]);
#ifdef USE_ZLE
    init_ZLE(Handler[board],BoardInfo[board],&Config[board],&Link[board],N_FADC_BOARD);
#else
    init_DGTZ(Handler[board],BoardInfo[board]);
#endif
    if(CheckBoardFailureStatus(Handler[board],BoardInfo[board])) return 0; 
  }
  
  v1730_get_MaxM_index(Handler,N_BOARD,&MaxBLTnHIndex,&MaxRLHIndex,&MaxMSizeHIndex,&MaxMSizeSum);
  
  if(EB_EFBLOCK_SIZE<MaxMSizeSum){
    printf("Error: data buffer is smaller than required size\n");
    printf("Required buffer size=%d byte\n",MaxMSizeSum);
    return 0;
  }
  if(maxbuff*2<MaxMSizeSum){
    printf("Warning: maxbuff(data send threshold) is smaller than all-board event size\n");
    printf("all-board event size=%d byte\n",MaxMSizeSum);
    printf("maxbuff=%d\n",2*maxbuff);
  }  
 
  // WARNING: These mallocs must be done after the digitizer programming
  if(v1730_malloc_readout(Handler[MaxMSizeHIndex],&buffer)==-1) return 0;
  if(v1730_malloc_event(Handler[MaxBLTnHIndex],N_BOARD,(void**)Event)==-1) return 0;
  
  //mkpid to use babimo
  //default = /var/run/babies
  //in this example, use /tmp/babies
  mkpid(pidpath);

  /** use TRG OUT for V1730 BUSY clear **/
  /*
  CAEN_DGTZ_ReadRegister(Handler[0], 0x811C, &regval);
  CAEN_DGTZ_WriteRegister(Handler[0], 0x811C, regval | 0x8000);
  */
    
  babies_main();

  return 0;
}
