/* Memory-only test bridge. Bosch source remains byte-exact under BSD-3-Clause. */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "bmi2.h"
static unsigned forbidden_io=0;
static BMI2_INTF_RETURN_TYPE blocked_read(uint8_t reg,uint8_t *data,uint32_t len,void *ptr){(void)reg;(void)data;(void)len;(void)ptr;forbidden_io++;return -1;}
static BMI2_INTF_RETURN_TYPE blocked_write(uint8_t reg,const uint8_t *data,uint32_t len,void *ptr){(void)reg;(void)data;(void)len;(void)ptr;forbidden_io++;return -1;}
static void blocked_delay(uint32_t period,void *ptr){(void)period;(void)ptr;forbidden_io++;}
__declspec(dllexport) int fifo_probe(const uint8_t *bytes,uint32_t length,int32_t *out,uint32_t capacity){
  if(!bytes||!out||length>8192||capacity<9+6*(length+1))return -1;
  struct bmi2_dev dev;struct bmi2_fifo_frame fifo;memset(&dev,0,sizeof(dev));memset(&fifo,0,sizeof(fifo));forbidden_io=0;
  dev.read=blocked_read;dev.write=blocked_write;dev.delay_us=blocked_delay;dev.remap.x_axis=0;dev.remap.y_axis=1;dev.remap.z_axis=2;dev.remap.x_axis_sign=dev.remap.y_axis_sign=dev.remap.z_axis_sign=BMI2_POS_SIGN;
  fifo.data=(uint8_t*)bytes;fifo.length=(uint16_t)length;fifo.header_enable=BMI2_ENABLE;fifo.data_enable=BMI2_FIFO_ACC_EN|BMI2_FIFO_GYR_EN;fifo.acc_frm_len=6;fifo.gyr_frm_len=6;fifo.acc_gyr_frm_len=12;fifo.aux_frm_len=8;fifo.acc_aux_frm_len=14;fifo.aux_gyr_frm_len=14;fifo.all_frm_len=20;
  struct bmi2_sens_axes_data *acc=calloc(length+1,sizeof(*acc)),*gyr=calloc(length+1,sizeof(*gyr));if(!acc||!gyr){free(acc);free(gyr);return -2;}
  uint16_t na=(uint16_t)(length+1),ng=na;int ra=bmi2_extract_accel(acc,&na,&fifo,&dev);int rg=bmi2_extract_gyro(gyr,&ng,&fifo,&dev);
  out[0]=ra;out[1]=rg;out[2]=na;out[3]=ng;out[4]=(int32_t)fifo.sensor_time;out[5]=fifo.skipped_frame_count;out[6]=fifo.acc_byte_start_idx;out[7]=fifo.gyr_byte_start_idx;out[8]=(int32_t)forbidden_io;
  unsigned index=9;for(unsigned i=0;i<na;i++){out[index++]=acc[i].x;out[index++]=acc[i].y;out[index++]=acc[i].z;}for(unsigned i=0;i<ng;i++){out[index++]=gyr[i].x;out[index++]=gyr[i].y;out[index++]=gyr[i].z;}
  free(acc);free(gyr);return 0;
}
