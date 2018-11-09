#include "bufferDecode.h"
#include <string.h>
#include <cmath>
#include <time.h>
#include <stdio.h>
#include <ros/ros.h>
#include <sys/time.h>
#include "ioapi.h"


static const int LINE_POINT_MINI_COUNT = 500; //一条扫描线最少点的个数
static const float ANGLE_CIRCLE_CONDITION = 270; //角度抖动处理值
static const float UINTCONVERT = 0.01;

static const size_t packet_size = sizeof(rfans_driver::RfansPacket().data);
static const size_t BLOCK_COUNT_MAX = sizeof(rfans_driver::RfansPacket().data)/138;
double m_lidarAngle[2][32];      //32个激光器的角度
double  m_mirrorVector[4][3];     //旋转矩阵
float m_anglePara[30]={0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0,45,-15,45,-15,0,0};
static float s_angle_duration = 359 ;
const double TIME_FLAG_SCALE = 0.0000001;
static int LINE_POINT_COUNT = 128*1024;
static std::vector<RFANS_XYZ_S> s_lineData;
static std::vector<RFANS_XYZ_S> s_rowData[32];
static int s_lineCount = 0 ;
static float s_lastAngle = 0 ;
static bool s_findStartZero = false;

static std::vector<float> s_vangles;
static std::vector<float> s_hangles;
static std::vector<float> s_reviseangles;

static DEVICE_TYPE_E s_device_type;
FILE *g_xyzFile = NULL;
inline unsigned int strToList(std::vector<float> &list, std::string str)
{
  unsigned int count = 0;
  string::size_type prev = 0;
  string::size_type pos = 0;
  string tmpValue ;
  list.clear();
  while((pos = str.find_first_of(',', pos))!= string::npos)
  {
    tmpValue =  str.substr(prev, pos - prev);
    list.push_back( atof(tmpValue.c_str()));
    pos++;
    prev = pos;
    count++;
  }
  return count;
}


 //计算坐标
inline int calcCFansCoor(float range ,float angle ,int index,int laserID, RFANS_XYZ_S &outXyz){
      int rtn = 1;
    // float tmpXYZ[3];
     double tmpMirrorVector[3];
     memcpy(tmpMirrorVector, m_mirrorVector[index], sizeof(m_mirrorVector[index]));

    /* if (laserID % 2 == 0) {
         //angle += 45 ;
         angle += m_anglePara[24];
     }else{
         // angle -= 15 ;
         angle += m_anglePara[25];
     }*/

     switch (laserID % 4) {
     case 0:
         angle  += m_anglePara[24];
         break;
     case 1:
         angle  += m_anglePara[25];
         break;
     case 2:
         angle  += m_anglePara[26];
         break;
     case 3:
         angle  += m_anglePara[27];
         break;
     }


     double tmpDrection[3];
     double angleV = m_lidarAngle[0][laserID] * M_PI / 180.0;
     double angleH = m_lidarAngle[1][laserID] * M_PI / 180.0;
     double direction[2][3]={ {abs(sin(angleH)*cos(angleV)), cos(angleH)*cos(angleV), sin(angleV) } ,{abs(sin(angleH)*cos(angleV)), -cos(angleH)*cos(angleV), sin(angleV) }};
     if (laserID % 2 == 0)
         memcpy(tmpDrection,direction[1],sizeof(direction[1])) ;
     else
         memcpy(tmpDrection,direction[0],sizeof(direction[0])) ;

     double stb = sin(angle*M_PI / 180.0);
     double	ctb = cos(angle*M_PI / 180.0);
     double tmptt = fabs(( (ctb*tmpMirrorVector[0] + stb*tmpMirrorVector[1])*tmpDrection[0] +(-stb*tmpMirrorVector[0] + ctb*tmpMirrorVector[1])*tmpDrection[1] +tmpMirrorVector[2]*tmpDrection[2]) );
     double tmpXyz0[3];
     tmpXyz0[0] = (tmpDrection[0] + 2 * (ctb*tmpMirrorVector[0] + stb*tmpMirrorVector[1])*tmptt)* range;
     tmpXyz0[1] = (tmpDrection[1] + 2 * (-stb*tmpMirrorVector[0] + ctb*tmpMirrorVector[1])*tmptt)* range;
     tmpXyz0[2] = (tmpDrection[2] + 2 * tmpMirrorVector[2] * tmptt)* range;



     double angleCorrX, angleCorrY, angleCorrZ;    
     int planBaseIndex;
     if ((laserID % 2) == 0){
         planBaseIndex = index* 3;
         tmpXyz0[1] += m_anglePara[28];
     }else{
          planBaseIndex = index * 3 + 12;
          tmpXyz0[1] += m_anglePara[29];
     }

   
     angleCorrX = m_anglePara[ planBaseIndex + 0];
     angleCorrY = m_anglePara[ planBaseIndex + 1];
     angleCorrZ = m_anglePara[ planBaseIndex + 2];

     outXyz.x = tmpXyz0[1] * (cos(angleCorrX)*sin(angleCorrZ) - cos(angleCorrZ)*sin(angleCorrX)*sin(angleCorrY)) - tmpXyz0[2] * (sin(angleCorrX)*sin(angleCorrZ) + cos(angleCorrX)*cos(angleCorrZ)*sin(angleCorrY)) + tmpXyz0[0] * cos(angleCorrY)*cos(angleCorrZ);
     outXyz.y= tmpXyz0[1] * (cos(angleCorrX)*cos(angleCorrZ) + sin(angleCorrX)*sin(angleCorrY)*sin(angleCorrZ)) - tmpXyz0[2] * (cos(angleCorrZ)*sin(angleCorrX) - cos(angleCorrX)*sin(angleCorrY)*sin(angleCorrZ)) - tmpXyz0[0] * cos(angleCorrY)*sin(angleCorrZ);
     outXyz.z = tmpXyz0[0] * sin(angleCorrY) + tmpXyz0[2] * cos(angleCorrX)*cos(angleCorrY) + tmpXyz0[1] * cos(angleCorrY)*sin(angleCorrX);

     return rtn;
     //return tmpXYZ;
 }



inline int calcXyz(unsigned char flag,float &mtRange, float &mtAngle, RFANS_XYZ_S &outXyz) {
  int rtn = 1;
  double tmptheta=0, ot = 0 ;

  switch (flag)
  {
  case RFANS_PRODUCT_MODEL_V6G_X32_0X33:
    mtAngle+= HANGLE_V6G_X32_0X33[outXyz.laserid];
    tmptheta = VANGLE_V6G_X32_0X33[outXyz.laserid] * M_PI / 180.0;
    break;
 case RFANS_PRODUCT_MODEL_V6P_X32_0X45:
  case RFANS_PRODUCT_MODEL_V6_X32_0X40:
    mtAngle+= HANGLE_V6_X32_0x40[outXyz.laserid];
    tmptheta = VANGLE_V6_X32_0X40[outXyz.laserid] * M_PI / 180.0;
    break;
 case RFANS_PRODUCT_MODEL_V6P_X16A_0X46:
  case RFANS_PRODUCT_MODEL_V6_X16A_0X41:
    outXyz.laserid = outXyz.laserid >= RFANS_LASER_COUNT ?
          outXyz.laserid - RFANS_LASER_COUNT : outXyz.laserid;

    mtAngle+= HANGLE_V6_X32_0x40[RFANS_PRODUCT_MODEL_V6_X16A_0X41_LASER_ID[outXyz.laserid]];
    tmptheta = VANGLE_V6_X32_0X40[RFANS_PRODUCT_MODEL_V6_X16A_0X41_LASER_ID[outXyz.laserid]] * M_PI / 180.0;
    break;

  case RFANS_PRODUCT_MODEL_V6P_X16B_0X47:
  case RFANS_PRODUCT_MODEL_V6_X16B_0X42:

    outXyz.laserid = outXyz.laserid >= RFANS_LASER_COUNT ?
          outXyz.laserid - RFANS_LASER_COUNT : outXyz.laserid;

    mtAngle+= HANGLE_V6_X32_0x40[RFANS_PRODUCT_MODEL_V6_X16A_0X42_LASER_ID[outXyz.laserid]];
    tmptheta = VANGLE_V6_X32_0X40[RFANS_PRODUCT_MODEL_V6_X16A_0X42_LASER_ID[outXyz.laserid]] * M_PI / 180.0;
    break;
 case RFANS_PRODUCT_MODEL_V6P_X16Even_0X48:
  case RFANS_PRODUCT_MODEL_V6_X16Even_0X43:

    outXyz.laserid = outXyz.laserid >= RFANS_LASER_COUNT ?
          outXyz.laserid - RFANS_LASER_COUNT : outXyz.laserid;
    mtAngle+= HANGLE_V6_X32_0x40[RFANS_PRODUCT_MODEL_V6_X16A_0X43_LASER_ID[outXyz.laserid]];
    tmptheta = VANGLE_V6_X32_0X40[RFANS_PRODUCT_MODEL_V6_X16A_0X43_LASER_ID[outXyz.laserid]] * M_PI / 180.0;
    break;

  case RFANS_PRODUCT_MODEL_V6P_X16Odd_0X49:
  case RFANS_PRODUCT_MODEL_V6G_X16_0X32:
  case RFANS_PRODUCT_MODEL_V6_X16Odd_0X44:
    outXyz.laserid = outXyz.laserid >= RFANS_LASER_COUNT ?
          outXyz.laserid - RFANS_LASER_COUNT : outXyz.laserid;
    tmptheta = VANGLE_V6_X32_0X40[RFANS_PRODUCT_MODEL_V6_X16A_0X44_LASER_ID[outXyz.laserid]] * M_PI / 180.0;
    mtAngle+= HANGLE_V6_X32_0x40[RFANS_PRODUCT_MODEL_V6_X16A_0X44_LASER_ID[outXyz.laserid]];
    break;

 case RFANS_PRODUCT_MODEL_V6A_X32_0X4A:
    mtAngle+= HANGLE_V6_X32_0x40[outXyz.laserid];
    tmptheta = VANGLE_V6A_X32[outXyz.laserid] * M_PI / 180.0;
    break;
  case RFANS_PRODUCT_MODEL_V6A_X16A_0X4B:
    outXyz.laserid = outXyz.laserid >= RFANS_LASER_COUNT ?
          outXyz.laserid - RFANS_LASER_COUNT : outXyz.laserid;

    mtAngle+= HANGLE_V6_X32_0x40[RFANS_PRODUCT_MODEL_V6_X16A_0X41_LASER_ID[outXyz.laserid]];
    tmptheta = VANGLE_V6A_X32[RFANS_PRODUCT_MODEL_V6_X16A_0X41_LASER_ID[outXyz.laserid]] * M_PI / 180.0;
     break;
  case RFANS_PRODUCT_MODEL_V6A_X16B_0X4C:
    outXyz.laserid = outXyz.laserid >= RFANS_LASER_COUNT ?
          outXyz.laserid - RFANS_LASER_COUNT : outXyz.laserid;

    mtAngle+= HANGLE_V6_X32_0x40[RFANS_PRODUCT_MODEL_V6_X16A_0X42_LASER_ID[outXyz.laserid]];
    tmptheta = VANGLE_V6A_X32[RFANS_PRODUCT_MODEL_V6_X16A_0X42_LASER_ID[outXyz.laserid]] * M_PI / 180.0;
     break;
  case RFANS_PRODUCT_MODEL_V6A_X16Even_0X4D:

    outXyz.laserid = outXyz.laserid >= RFANS_LASER_COUNT ?
          outXyz.laserid - RFANS_LASER_COUNT : outXyz.laserid;
    mtAngle+= HANGLE_V6_X32_0x40[RFANS_PRODUCT_MODEL_V6_X16A_0X43_LASER_ID[outXyz.laserid]];
    tmptheta = VANGLE_V6A_X32[RFANS_PRODUCT_MODEL_V6_X16A_0X43_LASER_ID[outXyz.laserid]] * M_PI / 180.0;
     break;
  case RFANS_PRODUCT_MODEL_V6A_X16Odd_0X4E:
    outXyz.laserid = outXyz.laserid >= RFANS_LASER_COUNT ?
          outXyz.laserid - RFANS_LASER_COUNT : outXyz.laserid;
    tmptheta = VANGLE_V6A_X32[RFANS_PRODUCT_MODEL_V6_X16A_0X44_LASER_ID[outXyz.laserid]] * M_PI / 180.0;
    mtAngle+= HANGLE_V6_X32_0x40[RFANS_PRODUCT_MODEL_V6_X16A_0X44_LASER_ID[outXyz.laserid]];
     break;
  case RFANS_PRODUCT_MODEL_V6A_E1_0X55:
      outXyz.laserid = outXyz.laserid >= RFANS_LASER_COUNT ?
            outXyz.laserid - RFANS_LASER_COUNT : outXyz.laserid;
      tmptheta = VAngle_16E1[outXyz.laserid] * M_PI / 180.0;
      mtAngle+= HANGLE_V6A_E1_0x55[outXyz.laserid];
       break;
  case RFANS_PRODUCT_MODEL_V6A_E2_0X56:
      outXyz.laserid = outXyz.laserid >= RFANS_LASER_COUNT ?
            outXyz.laserid - RFANS_LASER_COUNT : outXyz.laserid;
      tmptheta = VAngle_16E2[outXyz.laserid] * M_PI / 180.0;
      mtAngle+= HANGLE_V6A_E1_0x55[outXyz.laserid];
       break;
  case ID_RFANSBLOCKV32_16_31_SYNC:
    outXyz.laserid += RFANS_LASER_COUNT;
    tmptheta = VANGLE_V6_X32_0X40[outXyz.laserid] * M_PI / 180.0;
    break;
  case ID_RFANSBLOCKV6G_16_31_SYNC:
    outXyz.laserid += RFANS_LASER_COUNT;
    tmptheta = VANGLE_V6G_X32_0X33[outXyz.laserid] * M_PI / 180.0;
    break;
  case ID_RFANSBLOCKV32_0_15_SYNC:
    tmptheta = VANGLE_V6_X32_0X40[outXyz.laserid] * M_PI / 180.0;
    break;
  case ID_RFANSBLOCKV6G_0_15_SYNC:
    tmptheta = VANGLE_V6G_X32_0X33[outXyz.laserid] * M_PI / 180.0;
    break;
  case ID_RFANSBLOCKV2_SYNC:
    tmptheta = VANGLE_V5_X16[outXyz.laserid] * M_PI / 180.0;
    break;
  case RFANS_PRODUCT_MODEL_V6A_X16M_0X4F:
    outXyz.laserid = outXyz.laserid >= RFANS_LASER_COUNT ?
          outXyz.laserid - RFANS_LASER_COUNT : outXyz.laserid;
    mtAngle+= HANGLE_V5_X16[outXyz.laserid];
    tmptheta = VANGLE_V5_X16[outXyz.laserid] * M_PI / 180.0;
     break;
  case RFANS_PRODUCT_MODEL_V6B_X32_0X50:
      mtAngle += HANGLE_V6B_X32_0x40[outXyz.laserid];
      tmptheta = VANGLE_V6A_X32[outXyz.laserid] * M_PI / 180.0;
     break;
  case RFANS_PRODUCT_MODEL_V6BC_16G_0X57:
      outXyz.laserid = outXyz.laserid >= RFANS_LASER_COUNT ?
            outXyz.laserid - RFANS_LASER_COUNT : outXyz.laserid;
      mtAngle += HANGLE_V6BC_16G_0x57[outXyz.laserid];
      tmptheta = VAngle_V6B_16G[outXyz.laserid] * M_PI / 180.0;
     break;
  case RFANS_PRODUCT_MODEL_V6BC_16M_0X58:
      outXyz.laserid = outXyz.laserid >= RFANS_LASER_COUNT ?
            outXyz.laserid - RFANS_LASER_COUNT : outXyz.laserid;
      mtAngle += HANGLE_V6BC_16M_0x58[outXyz.laserid];
      tmptheta = VAngle_V6B_16M[outXyz.laserid] * M_PI / 180.0;
     break;      
   case   RFANS_PRODUCT_MODEL_V6C_Z_X32_0X59:
      outXyz.laserid = outXyz.laserid >= RFANS_LASER_COUNT ?
            outXyz.laserid - RFANS_LASER_COUNT : outXyz.laserid;
      mtAngle += HANGLE_V6B_X32_0x40[outXyz.laserid];
      tmptheta = VAngle_V6C_Z_32[outXyz.laserid] * M_PI / 180.0;
     break;

  }

  if(mtAngle>360) mtAngle -= 360;
  if(mtAngle<0) mtAngle +=360 ;

  ot = mtAngle*M_PI / 180.0;

  outXyz.x = mtRange*cos(tmptheta) *cos(-ot );
  outXyz.y = mtRange*cos(tmptheta) *sin(-ot );
  outXyz.z = mtRange*sin(tmptheta) ;
  outXyz.hangle = mtAngle;
  return rtn ;
}

inline bool sortByHangle(const RFANS_XYZ_S &v1, const RFANS_XYZ_S &v2) {
    return v1.hangle < v2.hangle;
}

inline int checkFrame_sum(float mtAngle,int mtlaserId,sensor_msgs::PointCloud2 &outCloud) {
  int rtn = 0 ;
  static float s_angleSum = 0;
  float angleDif = 0 ;
  if(mtlaserId!=0) return rtn ;

  int angleZero = (int)mtAngle;
  if (s_findStartZero == false) {
      if (angleZero == 0) {
          s_lineCount = 0 ;
          s_angleSum = 0 ;
          for (int i = 0; i < 32; ++i) {
            s_rowData[i].clear();
          }
          s_findStartZero = true;
      }
      return rtn;
  }


  if(mtAngle > s_lastAngle) {
    angleDif = mtAngle-s_lastAngle;
    s_angleSum += angleDif;
    s_lastAngle = mtAngle;
  } else {
    //1: 360-> 0
    if(s_lastAngle -mtAngle> 300)  {
      angleDif = 360 - s_lastAngle+mtAngle;
      s_angleSum += angleDif;
      s_lastAngle = mtAngle;
    } else {
      //nothing
    }

  }

  if(s_angleSum >= s_angle_duration) {

      timeval tval_begin, tval_end;
      gettimeofday(&tval_begin, NULL);
      int pointCnt = 0;
        for (int i = 0; i < 32; i++) {
            int lsrPointCnt = (s_rowData[i]).size();
            if (lsrPointCnt > 0) {
                // first sort
                //std::sort(s_rowData[i].begin(), s_rowData[i].end(), sortByHangle);
                for (int j = 0; j < lsrPointCnt; ) {
                    if ((s_rowData[i])[0].hangle > 345 && j == 0) {
                        RFANS_XYZ_S tmpxyz = (s_rowData[i])[0];
                        s_rowData[i].erase((s_rowData[i]).begin() + j);
                        s_rowData[i].push_back(tmpxyz);
                        j = 0;
                        continue;
                    }
                    j++;
                }

                // second save
                for (int j = 0; j < lsrPointCnt; j++) {
                    s_lineData[pointCnt++] = (s_rowData[i])[j];//!!!!
                }
            }
        }

 /*     for (int i = 0; i <s_lineCount; i++) {
          if(g_xyzFile) {
              //fprintf(g_xyzFile,"%d,%f\r\n",s_lineData[i].laserid,s_lineData[i].hangle);
              fprintf(g_xyzFile,"%d,%f,%f,%f,%f \r\n",
                                s_lineData[i].laserid, s_lineData[i].x, s_lineData[i].y, s_lineData[i].z, s_lineData[i].hangle);
              fflush(g_xyzFile);
          }
      }
      gettimeofday(&tval_end, NULL);
      int elapse = (tval_end.tv_sec - tval_begin.tv_sec)*1000000 + (tval_end.tv_usec - tval_begin.tv_usec);
      if (g_xyzFile) {
          fprintf(g_xyzFile,"time elapsed:%d us \r\n", elapse);
          fflush(g_xyzFile);
      }*/


    outCloud.header.stamp = ros::Time::now();
    outCloud.width = s_lineCount;
    outCloud.data.resize( outCloud.point_step*outCloud.width  );
    outCloud.row_step = outCloud.data.size();
    memcpy(&outCloud.data[0] , &s_lineData[0], outCloud.data.size() );
    rtn = 1;
    s_lineCount = 0 ;
    s_angleSum = 0 ;
    for (int i = 0; i < 32; ++i) {
        s_rowData[i].clear();
    }
  }
  return rtn ;
}

inline int checkFrame(float mtAngle,int mtlaserId,sensor_msgs::PointCloud2 &outCloud) {
  int rtn = 0 ;
  float angleDif = 0 ;
  if( mtlaserId !=0 ) return rtn ;

  if(mtAngle > s_lastAngle) {
    angleDif = mtAngle-s_lastAngle;
  } else {
    angleDif = 360 - s_lastAngle+mtAngle;
  }

  if(angleDif < 0 ) angleDif= 360+angleDif;

  if(angleDif >= s_angle_duration) {
    outCloud.width = s_lineCount;
    outCloud.data.resize( outCloud.point_step*outCloud.width  );
    outCloud.row_step = outCloud.data.size();
    memcpy(&outCloud.data[0] , &s_lineData[0], outCloud.data.size() );
    rtn = 1;
    s_lastAngle = mtAngle;
    s_lineCount = 0 ;

  }
  return rtn ;
}
inline int calCFansMirrorIndex(int angle_fix,int laserID,float &range){
    int mirrorIndex = 0;
    //修正零位角度 （奇偶路相差60°）    
    switch (laserID % 4) {
    case 0:
        angle_fix  += m_anglePara[24];
        break;
    case 1:
        angle_fix  += m_anglePara[25];
        break;
    case 2:
        angle_fix  += m_anglePara[26];
        break;
    case 3:
        angle_fix  += m_anglePara[27];
        break;
    }

    if (angle_fix< 0)
        angle_fix += 360;
    else if (angle_fix >360)
        angle_fix -= 360;

    //根据点的角度范围和激光器的奇偶路，确定镜面的编号
    if (laserID % 2 == 1) {
        //if (angle_fix >= 0 && angle_fix  < 90){
        if ((angle_fix >= 15 ) && (angle_fix < 90 ))  {
            mirrorIndex = 2;
        }
        //else if (angle_fix >= 90 && angle_fix  < 180) {
        else if ((angle_fix >= 105 ) && (angle_fix < 180 )) {
            mirrorIndex = 3;
        }
        // else if (angle_fix >= 180 && angle_fix  < 270) {
        else if ((angle_fix >= 195 ) && (angle_fix < 270 )) {
            mirrorIndex = 0;
        }
        //else if (angle_fix >= 270 && angle_fix  < 360){
        else if ((angle_fix >= 285 ) && (angle_fix < 360 )) {
            mirrorIndex = 1;
        }
        else {
            range = 0;
        }

    }
    else{
        //if ((angle_fix >= 0 && angle_fix  < 90)) {
        if ((angle_fix >= 0) && (angle_fix < 75 )) {
            mirrorIndex = 3;
        }
        //else if (angle_fix >= 90 && angle_fix  < 180) {
        else if ((angle_fix >= 90 ) && (angle_fix < 165 )) {
            mirrorIndex = 0;
        }
        //else if (angle_fix >= 180 && angle_fix  < 270) {
        else if ((angle_fix >= 180 ) && (angle_fix < 255 )) {
            mirrorIndex = 1;
        }
        //else if (angle_fix >= 270 && angle_fix  < 360) {
        else if ((angle_fix >= 270 ) && (angle_fix < 345 )) {
            mirrorIndex = 2;
        }
        else {
            range = 0;
        }
    }
    return mirrorIndex;
}
inline int processFrameV6G(RFans_UDP32FRAMEV6G_S *mtFrame, sensor_msgs::PointCloud2 &outCloud)
{
  int rtn = 0;
  bool tmp_isFull = false;
  float s_angleStep = 0;
  unsigned char mtSync;

  RFANS_XYZ_S tmpXyz ;
  int tmpDif = mtFrame->dataBlock[1].azimuthAngle - mtFrame->dataBlock[0].azimuthAngle;
  if ( tmpDif< -35000 ){
    tmpDif = tmpDif + 36000;
  }
  s_angleStep = (tmpDif) / 32.0;

  const float CONVERT_4mm_2_m =0.004f;

  unsigned int tmpDiftime = 0;
  float tmpAngle,tmpRange;

  float timeOffset = 0.0000015625; //us   640KHz
  int index;

  for (int j = 0; j < UDP32FRAMEV6G_COUNT; j++)//12Group
  {
    RFans_DataBlock_S *mtBlock = &mtFrame->dataBlock[j];
    mtSync = mtBlock->flag;

    for (int i = 0; i < 32; i++) {
        if (i % 2 == 0)            {
            index = (mtFrame->dataBlock[j].flag & 0xF000)>>12;
        }
        else{
            index = (mtFrame->dataBlock[j].flag & 0x0F00)>>8;
        }


      tmpXyz.timeflag = mtFrame->gpsTimestamp*0.000001 + timeOffset*i*j; // us->s//640Hz

      tmpXyz.intent  = mtBlock->laserBlock[i].intensity;

      tmpRange = mtBlock->laserBlock[i].range *CONVERT_4mm_2_m;
      tmpAngle = (mtBlock->azimuthAngle + (i*s_angleStep))*UINTCONVERT;

      tmpXyz.laserid = i;

      if(mtFrame->gmReservedA==RFANS_PRODUCT_MODEL_CFANS_X32_0X3780){
          calcCFansCoor(tmpRange,tmpAngle,index,i,tmpXyz) ;
      }else{
          calcXyz(mtFrame->gmReservedA, tmpRange, tmpAngle, tmpXyz);
      }

    /*  if (s_device_type == DEVICE_TYPE_CFANS) {
          //ROS_INFO("========parse point by cfans v6g========");
          calcCFansCoor(tmpRange,tmpAngle,index,i,tmpXyz) ;
      } else if (s_device_type == DEVICE_TYPE_RFANS){// rfans
          //ROS_INFO("========parse point by rfans v6g========");
          calcXyz(mtFrame->gmReservedA, tmpRange, tmpAngle, tmpXyz);
      }*/

//      if(g_xyzFile) {
//          fprintf(g_xyzFile,"%d,%f,%f,%f,%f,%lf,%f,%lf\r\n",
//                  tmpXyz.laserid, tmpXyz.x, tmpXyz.y, tmpXyz.z, tmpXyz.intent, tmpXyz.timeflag,tmpAngle,tmpRange);
//          fflush(g_xyzFile);
//      }
      if (tmpXyz.x == 0.0 && tmpXyz.y == 0.0 && tmpXyz.z == 0.0) {
          tmpXyz.x = tmpXyz.y = tmpXyz.z = NAN;
      }

      if ( checkFrame_sum(tmpAngle,tmpXyz.laserid ,outCloud) ) rtn =1 ;
      //s_lineData[s_lineCount] = tmpXyz;
      s_rowData[i].push_back(tmpXyz);//按激光器通道保存对应点
      ++s_lineCount;
      if(s_lineCount>=LINE_POINT_COUNT) s_lineCount = LINE_POINT_COUNT-1;
    }

  }
  //m_lastBloc
  return rtn;
}

const unsigned char SYNC_RELEASE_BLOCKV32_CFANS128_0_15 = 0x9D;
const unsigned char SYNC_RELEASE_BLOCKV32_CFANS128_16_31 = 0x9E;

inline int processFrameV5( RFans_UDPFRAMEV5_S *mtFrame, sensor_msgs::PointCloud2 &outCloud)
{
  int rtn = 0;
  RFANS_XYZ_S tmpXyz ;
  float angleDif = 0 ,T0_STEP_VALUE = 0;
  float tmpAngle,tmpRange;
  for (int i = 0; i < 10; i++)
  {
    SCDRFANS_BLOCK_S *mtBlock = &mtFrame->blockdata[i];
    unsigned char tmpCheck = checkSum(((unsigned char*)mtBlock) + 2, sizeof(SCDRFANS_BLOCK_S)-2);
    if (tmpCheck != mtBlock->chksum) {  //check sum
      continue;
    }

    for(int j = 0 ; j <RFANS_LASER_COUNT;j++ ) {
      tmpXyz.laserid = j;

      switch (mtBlock->dataID ) {
      case ID_RFANSBLOCKV32_16_31_SYNC:
      case ID_RFANSBLOCKV6G_16_31_SYNC:
      case ID_RFANSBLOCKV32_0_15_SYNC:
      case ID_RFANSBLOCKV6G_0_15_SYNC:
      case SYNC_RELEASE_BLOCKV32_CFANS128_0_15:
      case SYNC_RELEASE_BLOCKV32_CFANS128_16_31:
          T0_STEP_VALUE = 15.625;
          if(mtBlock->dataID==ID_RFANSBLOCKV32_16_31_SYNC || mtBlock->dataID==ID_RFANSBLOCKV6G_16_31_SYNC || mtBlock->dataID== SYNC_RELEASE_BLOCKV32_CFANS128_16_31)
              tmpXyz.laserid+=RFANS_LASER_COUNT;
          break;
      case ID_RFANSBLOCKV2_SYNC:
          T0_STEP_VALUE = 31.25;
          break;
      }

      tmpXyz.timeflag = mtBlock->t0stampH + TIME_FLAG_SCALE*(mtBlock->t0stampL + T0_STEP_VALUE*j);
      tmpAngle = mtBlock->laserData[j].angle *UINTCONVERT;
      tmpXyz.hangle = tmpAngle;
      tmpRange = mtBlock->laserData[j].rangeOne*UINTCONVERT;
      tmpXyz.intent  = mtBlock->laserData[j].intentTwo;
      if(mtBlock->dataID==SYNC_RELEASE_BLOCKV32_CFANS128_0_15 || mtBlock->dataID==SYNC_RELEASE_BLOCKV32_CFANS128_16_31){
          int index=calCFansMirrorIndex(tmpAngle,tmpXyz.laserid,tmpRange) ;
          calcCFansCoor(tmpRange,tmpAngle,index,tmpXyz.laserid,tmpXyz) ;
      }else{
         calcXyz(mtBlock->dataID,tmpRange, tmpAngle, tmpXyz);
      }


      if (tmpXyz.x == 0.0 && tmpXyz.y == 0.0 && tmpXyz.z == 0.0) {
          tmpXyz.x = tmpXyz.y = tmpXyz.z = NAN;
      }

      if ( checkFrame_sum(tmpAngle,tmpXyz.laserid ,outCloud) ) rtn =1 ;
      //s_lineData[s_lineCount] = tmpXyz;
      s_rowData[i].push_back(tmpXyz);//按激光器通道保存对应点
      ++s_lineCount;
      if(s_lineCount>=LINE_POINT_COUNT) s_lineCount = LINE_POINT_COUNT-1;

   

    }
  }
  return rtn;
}

SSBufferDec::SSBufferDec()
{
  reset();
}

SSBufferDec::~SSBufferDec()
{

}

 int SSBufferDec::initCFansPara(std::string reviseAngle) {
     //readAngleParaFile() ;
    // 32个激光器的角度
    double tmpAngle[2][32]= { {-13.6, -13.35,-11.82,-11.57,-10.04 ,-9.79,-8.26 ,-8.01,-6.48,-6.23,-4.7,-4.45,-2.92,-2.67,-1.14,-0.89,0.64,0.89,2.42,2.67,4.2,4.45,5.98,6.23,7.76,8.01,9.54,9.79,11.32,11.57,13.1,13.35},
                              { -14.95, 15.95, -17.05, 14.05, -14.95, 15.95, -17.05, 14.05, -14.95, 15.95, -17.05, 14.05, -14.95, 15.95, -17.05, 14.05, -14.95, 15.95, -17.05, 14.05, -14.95, 15.95, -17.05, 14.05, -14.95, 15.95, -17.05, 14.05, -14.95, 15.95, -17.05, 14.05}   };
    for (int i=0; i<2; i++)  {
        memcpy(m_lidarAngle[i], tmpAngle[i], sizeof(tmpAngle[i]));
    }
    //旋转矩阵
    double tmpPlaneNormal[4][3]= { { -1.0000, 0, -0.0066 }, { 0, -1.0000, 0 }, { 1.0000, 0, 0.0066 }, { 0 , 0.9999 ,-0.0133 }} ;
    for (int i=0; i<4; i++)    {
        memcpy(m_mirrorVector[i], tmpPlaneNormal[i], sizeof(tmpPlaneNormal[i]));
    }

    strToList(s_reviseangles, reviseAngle);
    for (int i = 0; i < s_reviseangles.size(); ++i) {
        m_anglePara[i] = s_reviseangles[i];
    }
//    char angle[256] = { 0 };
//    char cmd[512] = {0};
//    sprintf(angle, "%f,%f,%f,%f,%f,%f,%f,%f", m_anglePara[0], m_anglePara[1],m_anglePara[2],
//            m_anglePara[3],m_anglePara[4],m_anglePara[5],m_anglePara[6],m_anglePara[7]);
//    sprintf(cmd, "echo \"%s\" > aaa.txt", angle);
//    system(cmd);
    return 1;
}

int SSBufferDec::moveWriteIndex(int setpIndex)
{
  m_decBuf.bufSize += setpIndex;
  m_udpSize = setpIndex;
  m_udpCount++;
  m_decBuf.wrHead = (m_decBuf.wrHead+setpIndex)%DECODE_BUFFER_SIZE;
  return m_decBuf.bufSize ;
}

int SSBufferDec::moveReadIndex(int setpIndex)
{
  m_decBuf.bufSize -= setpIndex;
  m_decBuf.rdTail = (m_decBuf.rdTail+setpIndex)%DECODE_BUFFER_SIZE;
  return m_decBuf.bufSize ;
}

unsigned char *SSBufferDec::getWriteIndex()
{
  return m_decBuf.buffer+m_decBuf.wrHead ;
}

unsigned char *SSBufferDec::getReadIndex()
{
  return m_decBuf.buffer+m_decBuf.rdTail ;
}


int SSBufferDec::writeBuffer(unsigned char *data, int size)
{
  if(m_decBuf.bufSize+size >= DECODE_BUFFER_SIZE) return 0 ;

  memcpy(m_decBuf.buffer+m_decBuf.wrHead,data,size);
  m_decBuf.bufSize += size;
  m_decBuf.wrHead += size ;

  //  ROS_INFO_STREAM( "writeBuffer"
  //                  << " bufSize " << m_decBuf.bufSize
  //                  << " wrHead "<< m_decBuf.wrHead
  //                  << " rdTail " <<m_decBuf.rdTail);
  return size ;
}

int SSBufferDec::readPacket(rfans_driver::RfansPacket &pkt)
{
  int rtn = 0;
  if(m_decBuf.bufSize > 0 ) {
    pkt.data.resize(m_decBuf.bufSize);
    memcpy(&pkt.data[0], m_decBuf.buffer,  m_decBuf.bufSize);
    pkt.udpCount = m_udpCount;
    pkt.udpSize = m_udpSize;
    reset();
    rtn = 1;
  }
  return rtn ;
}

int SSBufferDec::size()
{
  return m_decBuf.bufSize;
}

int SSBufferDec::freeSize()
{
  return DECODE_BUFFER_SIZE-m_decBuf.bufSize;
}


static char s_tmpBuffer[DECODE_BUFFER_SIZE];

void SSBufferDec::bufferReverse()
{
  if (m_decBuf.bufSize > 0) {
    memcpy(s_tmpBuffer, m_decBuf.buffer + m_decBuf.rdTail, m_decBuf.bufSize);
    memcpy(m_decBuf.buffer, s_tmpBuffer, m_decBuf.bufSize);
    m_decBuf.rdTail = 0;
    m_decBuf.wrHead = m_decBuf.bufSize;
  } else {
    m_decBuf.wrHead = m_decBuf.bufSize = m_decBuf.rdTail = 0;
  }
  return;
}

void SSBufferDec::reset()
{
  //memset(&m_decBuf,0,sizeof(m_decBuf)) ;
  m_decBuf.bufSize = m_decBuf.wrHead = m_decBuf.rdTail = 0 ;
  memset(&m_packet,0,sizeof(m_packet)) ;
  m_status = eReady;
  s_preAngle =0 ;
  m_packetSize = 0;
  m_blockCout = 0 ;
  m_udpCount = 0 ;
}

int SSBufferDec::Depacket(rfans_driver::RfansPacket &inPack, sensor_msgs::PointCloud2 &outCloud , ros::Publisher &rosOut, DEVICE_TYPE_E deviceType)
{
  int rtn =0, updateflag = 0;
  s_device_type = deviceType;

  RFans_UDP32FRAMEV6G_S *tmpFrameV6;
  RFans_UDPFRAMEV5_S * tmpFrameV5;

  if( UDP_PACKET_SIZE_V5A == inPack.udpSize) {
    for( int i = 0 ; i < inPack.udpCount;i++) {
      tmpFrameV5 = (RFans_UDPFRAMEV5_S*)(&inPack.data[0] + i*inPack.udpSize);
      if( processFrameV5(tmpFrameV5,outCloud) ){
        rosOut.publish(outCloud);
        SSBufferDec::ResetPointCloud2(outCloud);
      }
    }
  }
  else if(UDP_PACKET_SIZE_V6G == inPack.udpSize) {
    for( int i = 0 ; i < inPack.udpCount;i++) {
      tmpFrameV6 = (RFans_UDP32FRAMEV6G_S*)(&inPack.data[0] + i*inPack.udpSize);
      if( processFrameV6G(tmpFrameV6,outCloud) ) {
        rosOut.publish(outCloud);
        SSBufferDec::ResetPointCloud2(outCloud);
      }
    }
  } else {
    ROS_INFO_STREAM(" inPack.udpSize " <<inPack.udpSize );
  }
  return rtn ;
}

void SSBufferDec::InitPointcloud2(sensor_msgs::PointCloud2 &initCloud) {
  static const size_t DataSize = sizeof(rfans_driver::RfansPacket().data) / sizeof(SCDRFANS_BLOCK_S ) * sizeof(RFANS_XYZ_S) *RFANS_LASER_COUNT;
  initCloud.data.clear();
  initCloud.data.resize( DataSize); //point data

  initCloud.is_bigendian = false ;//false;      //stream foramt
  initCloud.fields.resize(7);          //line format
  initCloud.is_dense = false;

  int tmpOffset = 0 ;
  for(int i=0; i < initCloud.fields.size() ;i++) {
    switch(i) { //value type
    case 0:
      initCloud.fields[i].name = "x" ;
      initCloud.fields[i].datatype = 7u;
      break;
    case 1:
      initCloud.fields[i].name = "y" ;
      initCloud.fields[i].datatype = 7u;
      tmpOffset += 4;
      break;
    case 2:
      initCloud.fields[i].name = "z" ;
      initCloud.fields[i].datatype = 7u;
      tmpOffset += 4;
      break;
    case 3:
      initCloud.fields[i].name = "intensity" ;
      initCloud.fields[i].datatype = 7u;//2u;
      tmpOffset += 4;
      break;
    case 4:
      initCloud.fields[i].name = "laserid" ;
      initCloud.fields[i].datatype = 2u;
      tmpOffset += 1;
	  break;
    case 5:
      initCloud.fields[i].name = "timeflag" ;
      initCloud.fields[i].datatype = 8u;
      tmpOffset += 8;
      break;
	case 6:
		initCloud.fields[i].name = "hangle" ;
        initCloud.fields[i].datatype = 7u;
        tmpOffset += 4;
        break;
		
    }
    initCloud.fields[i].offset = tmpOffset ;      //value offset
    initCloud.fields[i].count = 1 ;
  }
  initCloud.height = 1;
  initCloud.point_step = sizeof(RFANS_XYZ_S);
  initCloud.row_step = DataSize ;
  initCloud.width = 0 ;


  //node name
  std::string node_name = ros::this_node::getName();

  std::string frame_id_str = "/world";
  std::string frame_id_path = node_name + "/frame_id";
  ros::param::get(frame_id_path,frame_id_str);

  initCloud.header.frame_id = frame_id_str;

  s_lastAngle = 0 ;
  s_lineData.resize(LINE_POINT_COUNT);
  s_lineCount = 0 ;

  s_hangles.clear();
  s_vangles.clear();
  for (int i = 0; i < 32; ++i) {
    s_rowData[i].clear();
  }
  //logFile = fopen("/home/liyp/test.log","w+");
//  //device  ip name
//  std::string vangles_str = "\
//      -25,   -22,   -19,   -16,\
//      -13,   -11,    -9,    -7,\
//      -5.5,  -4.5,  -3.5,  -2.9,\
//      -2.45,  -2.1, -1.75,  -1.4,\
//      -1.05,  -0.7, -0.35,     0,\
//      0.35,   0.7,  1.05,   1.4,\
//      2.5,   3.5,   4.5,     6,\
//      8,    10,    12,    15, ";

//      std::string vangle_path = node_name + "/laser_vangle";
//  ros::param::get(vangle_path,vangles_str);
//  strToList(s_vangles,vangles_str);

//  std::string hangles_str = "\
//      0,  0,  0,  0,\
//      0,  0,  0,  0,\
//      0,  0,  0,  0,\
//      0,  0,  0,  0,\
//      0,  0,  0,  0,\
//      0,  0,  0,  0,\
//      0,  0,  0,  0,\
//      0,  0,  0,  0,\
//      0,  0,  0,  0, ";

//  std::string hangle_path = node_name + "/laser_hangle";
//  ros::param::get(hangle_path, hangles_str);
//  ROS_INFO("hable %s\n",hangles_str.c_str() );
//  strToList(s_hangles, hangles_str);
}

void SSBufferDec::ResetPointCloud2(sensor_msgs::PointCloud2 &initCloud) {
  initCloud.width = 0;
}

void SSBufferDec::SetAngleDuration(float value)
{
  if(value <10  || value > 360)
    return ;
  s_angle_duration = value;
}

void SSBufferDec::setSaveXYZ(bool save) {

    if (true == save && !g_xyzFile) {
        time_t now; struct tm *timeNow;
        time(&now);
        timeNow = localtime(&now);
        int yy = timeNow->tm_year % 100;
        int mm = (timeNow->tm_mon + 1) % 12;
        int dd = timeNow->tm_mday;
        int hh = timeNow->tm_hour;
        int mn = timeNow->tm_min;
        int ss = timeNow->tm_sec;
        char filename[FILENAME_MAX] = { '\0' };
        sprintf(filename, "xyz-%02d%02d%02d-%02d%02d%02d.txt", yy, mm, dd, hh, mn, ss);

        g_xyzFile = fopen(filename, "w+");
        fprintf(g_xyzFile,"id, X, Y, Z, Intent, Timeflag \r\n");
        fflush(g_xyzFile);
    }
}

