#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
// #include "ylidar_protocol.hpp"

typedef uint8_t result_t;

#define DEBUG_ANGLE 1

#define BUFF_SIZE 14
#define RESULT_OK 1
#define RESULT_TIMEOUT 0

#define PH 0x55AA
#define PH1 0xAA
#define PH2 0x55 
#define PH3 0x66 
#define LIDAR_RESP_CHECKBIT       (0x1<<0)

//Package
#define TRI_PACKHEADSIZE 10
#define TRI_PACKMAXNODES 80

typedef enum {
  CT_Normal = 0,///< Normal package
  CT_RingStart  = 1,///< Starting package
  CT_Tail,
} CT;

struct stamp_package {
  uint8_t flag1; //包头标记1
  uint8_t flag2; //包头标记2
  uint8_t cs; //校验和
  uint32_t stamp; //时间戳
  uint8_t reserved; //保留字段
} __attribute__((packed));
#define SIZE_STAMPPACKAGE sizeof(stamp_package)


static int byte_offset = 0;

result_t parseResponseHeader(uint8_t packageBuffer[BUFF_SIZE], uint8_t globalRecvBuffer[BUFF_SIZE]) {
  int recvPos = 0;
  int package_Sample_Num = 0;
  uint16_t CheckSumCal = 0;
  uint8_t package_type = 0;
  result_t ans = RESULT_TIMEOUT;

  size_t remainSize = TRI_PACKHEADSIZE - recvPos;
  size_t recvSize = remainSize;

  bool has_package_error = false;

  uint8_t SampleNumlAndCTCal = 0;
  int FirstSampleAngle = 0;
  int LastSampleAngle = 0;
  uint16_t LastSampleAngleCal = 0;
  float IntervalSampleAngle = 0.0f;
  float IntervalSampleAngle_LastPackage = 0.0f;
  uint16_t CheckSum = 0;

  int scan_frequence = 0;

  for (size_t pos = 0; pos < recvSize; ++pos)
  {
    uint8_t currentByte = globalRecvBuffer[pos];
    switch (recvPos)
    {
    case 0:
      if (currentByte != PH1)
      {
        // checkBlockStatus(currentByte);
        continue;
      }
      break;

    case 1:
    {
      CheckSumCal = PH;
      if (currentByte == PH2)
      {
        // if (m_driverErrno == BlockError)
        // {
        //   setDriverError(NoError);
        // }
      }
      else if (currentByte == PH1) // 防止出现连续0xAA
      {
        continue;
      }
      else if (currentByte == PH3) // 时间戳标识
      {
        recvPos = 0;
        size_t lastPos = pos - 1;
        // 解析时间戳（共8个字节）
        int remainSize = SIZE_STAMPPACKAGE - (recvSize - pos + 1); // 计算剩余应读字节数
        if (remainSize > 0)
        {
          size_t lastSize = recvSize;
          // ans = waitForData(remainSize, timeout - waitTime, &recvSize);
          // if (!IS_OK(ans))
          //   return ans;
          if (recvSize > remainSize)
            recvSize = remainSize;
          // getData(&globalRecvBuffer[lastSize], recvSize);
          recvSize += lastSize;
          pos = TRI_PACKHEADSIZE;
        }
        else
        {
          pos += 6;
        }
        // 时间戳校验和检测
        uint8_t csc = 0; // 计算校验和
        uint8_t csr = 0; // 实际校验和
        for (int i = 0; i < SIZE_STAMPPACKAGE; ++i)
        {
          if (i == 2)
            csr = globalRecvBuffer[lastPos + i];
          else
            csc ^= globalRecvBuffer[lastPos + i];
        }
        if (csc != csr)
        {
          // error("Checksum error c[0x%02X] != r[0x%02X]", csc, csr);
        }
        else
        {
          // stamp_package sp;
          // memcpy(&sp, &globalRecvBuffer[lastPos], SIZE_STAMPPACKAGE);
          // stamp = uint64_t(sp.stamp) * 1000000; // 毫秒转纳秒需要×1000000
          // debug("stamp: 0x%"PRIx64" -> 0x%"PRIx64"", sp.stamp, stamp);
          // 测试扫描时长
          // static uint32_t s_scanTime = 0;
          // if (s_scanTime > 0)
          // {
          //     uint32_t dt = sp.stamp - s_scanTime;
          //     if (dt < 44 || dt > 57)
          //     {
                  // error("单帧时长[%u]ms超出标准[%u~%u]",
                  //     dt, 44, 57);
          //     }
          // }
          // s_scanTime = sp.stamp;
        }
        continue;
      }
      else
      {
        has_package_error = true;
        recvPos = 0;
        continue;
      }
      break;
    }
    case 2:
      SampleNumlAndCTCal = currentByte;
      package_type = currentByte & 0x01; // 是否是零位包标识
      if (package_type == CT_RingStart)
      {
        scan_frequence = (currentByte & 0xFE) >> 1;
      }
      break;

    case 3:
      SampleNumlAndCTCal += (currentByte * 0x100);
      package_Sample_Num = currentByte;
      printf("lsn: %d\n", package_Sample_Num);
      if (package_Sample_Num > TRI_PACKMAXNODES)
      {
          // warn("Current pack point count %d too big", package_Sample_Num);
          recvPos = 0;
          continue;
      }
      break;

    case 4:
      if (currentByte & LIDAR_RESP_CHECKBIT)
      {
        FirstSampleAngle = currentByte;
      }
      else
      {
        has_package_error = true;
        recvPos = 0;
        continue;
      }
      break;

    case 5:
      FirstSampleAngle += currentByte * 0x100;
      CheckSumCal ^= FirstSampleAngle;
      FirstSampleAngle = FirstSampleAngle >> 1;
      break;

    case 6:
      if (currentByte & LIDAR_RESP_CHECKBIT)
      {
        LastSampleAngle = currentByte;
      }
      else
      {
        has_package_error = true;
        recvPos = 0;
        continue;
      }

      break;

    case 7:
      printf("LSA => lsb: %02X, msb: %02X\n", (uint8_t) LastSampleAngle, currentByte);

      LastSampleAngle = currentByte * 0x100 + LastSampleAngle;
      LastSampleAngleCal = LastSampleAngle;
      LastSampleAngle = LastSampleAngle >> 1;

      printf("lsa: %d, fsa: %d\n", LastSampleAngle, FirstSampleAngle);


      if (package_Sample_Num == 1)
      {
        IntervalSampleAngle = 0;
      }
      else
      {
        if (LastSampleAngle < FirstSampleAngle)
        {
          if ((FirstSampleAngle > 270 * 64) && (LastSampleAngle < 90 * 64))
          {
            IntervalSampleAngle = (float)((360 * 64 + LastSampleAngle -
                                           FirstSampleAngle) /
                                          ((
                                               package_Sample_Num - 1) *
                                           1.0));
            IntervalSampleAngle_LastPackage = IntervalSampleAngle;
          }
          else
          {
            IntervalSampleAngle = IntervalSampleAngle_LastPackage;
          }
        }
        else
        {
          IntervalSampleAngle = (float)((LastSampleAngle - FirstSampleAngle) / ((
                                                                                    package_Sample_Num - 1) *
                                                                                1.0));
          IntervalSampleAngle_LastPackage = IntervalSampleAngle;
        }
        if (DEBUG_ANGLE) {
          printf("angle: %f\n", IntervalSampleAngle);
        }
      }
      break;

    case 8:
      CheckSum = currentByte;
      break;
    case 9:
      CheckSum += (currentByte * 0x100);
      break;
    }

    printf("curr byte offset: %d\n", recvPos);
    packageBuffer[recvPos++] = currentByte;
  }

  if (recvPos == TRI_PACKHEADSIZE)
  {
    ans = RESULT_OK;
    return ans;
    // break;
  }

  ans = RESULT_TIMEOUT;

  return ans;
}

int main(void) {
  uint8_t packet[BUFF_SIZE] = {
     0xaa , 0x55 , 0x00 , 0x02 , 0x33 , 0x69 , 0xc5 , 0x69 , 0xad , 0x57 , 0x8a , 0x04 , 0x7b, 0x04
  };
  uint8_t result[BUFF_SIZE];
  parseResponseHeader(result, packet);
  return 0;
}
