/*
 * Copyright (c) 2023-2024 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
*/

#include "adrd2121_imu_ros2_common.hpp"

long long current_timestamp()
{
  struct timeval te;
  gettimeofday(&te, NULL);
  long long microseconds = te.tv_sec*1000000LL + te.tv_usec;
  return microseconds;
}

void print_time(std::string debug_msg)
{
  long long ts = current_timestamp();
  std::cout << debug_msg << " : " << ts << " usec \n";
  fflush(stdout);
}

uint32_t crc32_block(uint32_t crc, const uint16_t *data, int n)
{
  uint32_t c;
  int i;

  /* cycle through memory */
  for(i=0; i<n; i++)
  {
    /* Get lower byte */
    c = 0x000000FF & (uint32_t)data[i];
    /* Process with CRC */
    crc = ( (crc>>8) & 0x00FFFFFF) ^ crc32_tab[(crc^c) & 0xFF];
    /* Get upper byte */
    c = 0x000000FF & ((uint32_t)data[i]>>8);
    /* Process with CRC */
    crc = ( (crc>>8) & 0x00FFFFFF) ^ crc32_tab[(crc^c) & 0xFF];
  }

  return crc;
}

