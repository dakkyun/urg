/*!
  \example get_distance.c 距離データを取得する
  \author Satofumi KAMIMURA

  $Id: get_distance.c,v c5747add6615 2015/05/07 03:18:34 alexandr $
  */

#include "urg_sensor.h"
#include "urg_utils.h"
#include "open_urg_sensor.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <time.h>

#define PI 3.141592653589793

#define BAUDRATE B9600
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

clock_t start,end;

FILE *fp;

int serial_arduinowrite(char * , char * ,int);
int serial_arduinoread(char *,char *);
int fd,error[10];

double x,y;	//UAV's position
double slope;	//UAV's slope
double add = 0;

char rec[255] = {0};

static void print_data(urg_t *urg, long data[], int data_n, long time_stamp, int rad_s, int rad_e)
{
#if 1

  int a = 0,i,j,k,l,m,n;	//Variable of if
  int flag;	//Whether a linear approximation
  int step[1081] = {0};
  int step_max,step_min,step_s;
  int Farther_value[10] = {0},Farther_flag;
  int counter;

  double X = 8.0,Y = 4.1,sl = 0;	//Tunnel size
  double x_t[1081] = {0},y_t[1081] = {0};	//Tunnel coordinates
  double deg,rad;	//Angle from corner to corner
  double stddeg,cmpdeg;	//Reference angle, Comparison angle
  double x_s,y_s;	//sum
  double x_a,y_a;	//average
  double part_1,part_2;	//Calculating element of approximate straight line
  double a_1[1081] = {0},b_1[1081] = {0};	//Slope and Intercept of wall line
  double a_2,b_2;	//Slope and Intercept of bottom line
  double x_c,y_c,x_cc,y_cc;	//Corner coordinates
  double cldis_1,cldis_2;	//Distance to corner
  double diff_1,diff_2,difference;	//Detection of corner step number
  double step_1,step_2;	//Number of steps in the corner
  double x_0,cldis_0;	//Coordinates and distance of 180th step
  double max,min;
  double x_t_a[1081] = {0},y_t_a[1081] = {0};
  double x_t_s,y_t_s;
  double angle,angle_jdg;

  (void)data_n;

  for(i = rad_s;i <= rad_e;i++){
    //printf("%d    :   %ld [mm], (%ld [msec])\n", i, data[i], time_stamp);
    if(i >= 180 && i <= 900){
      //tunnel coodinate
      rad = (i - 180) * 0.25 * (PI / 180.0);	
      x_t[i] = (data[i] / 1000.0) * cos(-rad);
      y_t[i] = (data[i] / 1000.0) * sin(-rad);	
    }
  }
  //display
  //for(i = 180;i <= 900;i++)
  //printf("%d  x : %f   y : %f\n",i,x_t[i],y_t[i]);

  //Linear approximation
  for(i = 189;i <= 900;i++){
    for(j = 0;j < 10;j++){
      if(j == 0){
        max = x_t[i];
        min = x_t[i];
        step_max = i;
        step_min = i;
      }
      else if(max < x_t[i - j]){
        max = x_t[i - j];
        step_max = i - j;
      }
      else if(min > x_t[i - j]){
        min = x_t[i - j];
        step_min = i - j;
      }
    }
    for(j = 0;j < 10;j++){
      if(i - j != step_max && i - j != step_min){
        x_t_a[i] += x_t[i - j];
      }
    }
    x_t_a[i] /= 8.0;
  }
  for(i = 189;i <= 900;i++){
    for(j = 0;j < 10;j++){
      if(j == 0){
        max = y_t[i];
        min = y_t[i];
        step_max = i;
        step_min = i;
      }
      else if(max < y_t[i - j]){
        max = y_t[i - j];
        step_max = i - j;
      }
      else if(min > y_t[i - j]){
        min = y_t[i - j];
        step_min = i - j;
      }
    }
    for(j = 0;j < 10;j++){
      if(i - j != step_max && i - j != step_min)
        y_t_a[i] += y_t[i - j];

      y_t_a[i] /= 8.0;
    }

    //for(i = 180;i <= 900;i++)
    //printf("%d  x_t_a : %f   y_t_a : %f\n",i,x_t_a[i],y_t_a[i]);

    //bottom line
    i = 410;
    k = 0;
    while(1){
      step[k] = i;
      k++;
      if(fabs(y_t[i] - y_t[i + 10]) > 0.1){
        if(fabs(y_t[i + 10] - y_t[i + 20]) > 0.1){
          if(fabs(y_t[i + 20] - y_t[i + 30]) > 0.1)
            break;
        }
      }
      if(i > 870)
        break;

      i += 10;
    }

    //for(i = 0;i < k;i++)
    //printf("%d %f %f\n",step[i],x_t_a[step[i]],y_t_a[step[i]]);

    y_t_s = 0;
    for(i = 0;i < k;i++){
      y_t_s += y_t_a[step[i]];
    }
    y_t_s /= (float)k;

    j = 0;
    for(i = 0;i < k;i++){
      if(fabs(y_t_a[step[i]] - y_t_s) > 1.0){
        Farther_value[j] = step[i];
        j++;
      }
    }

    //sum
    x_s = 0;
    y_s = 0;
    m = 0;
    for(i = 0;i < k;i++){
      Farther_flag = 0;
      for(l = 0;l < j;l++){
        if(step[i] == Farther_value[l]){
          Farther_flag = 1;
          m++;
        }
      }
      if(Farther_flag != 1){
        x_s += x_t_a[ step[i] ];
        y_s += y_t_a[ step[i] ];
      }
    }

    //average
    x_a = x_s / (float)(k - m);
    y_a = y_s / (float)(k - m);

    //Linear equations
    part_1 = 0;
    part_2 = 0;
    for(i = 0;i < k;i++){
      Farther_flag = 0;
      for(l = 0;l < j;l++){
        if(step[i] == Farther_value[l])
          Farther_flag = 1;
      }
      if(Farther_flag != 1){
        part_1 += x_t_a[ step[i] ] * y_t_a[ step[i] ];
        part_2 += pow( x_t_a[ step[i] ] , 2.0 );
      }
    }
    a_2 = ( part_1 - ((float)(k - m) * x_a * y_a) ) / ( part_2 - ( (float)(k - m) * pow(x_a , 2.0) ) );
    b_2 = y_a - (a_2 * x_a);

    //side line
    if(data[180] < data[900]){
      flag = 1;
    }

    for(i = 189;i < 400;i++){
      j = i;
      k = 0;
      while(1){
        step[k] = j;
        k++;
        if(fabs(x_t[j] - x_t[j + 10]) > 0.08){
          if(fabs(x_t[j + 10] - x_t[j + 20]) > 0.08){
            if(fabs(x_t[j + 20] - x_t[j + 30]) > 0.08){
              break;
            }
          }
        }
        if(j > 870)
          break;
        j += 10;
      }
      if(k > 4){
        x_t_s = 0;
        for(n = 0;n < k;n++)
          x_t_s += x_t_a[step[n]];
        x_t_s /= (float)k;
        j = 0;
        for(n = 0;n < k;n++){
          if(fabs(x_t_a[step[n]] - x_t_s) > 5.0){
            Farther_value[j] = step[n];
            j++;
          }
        }

        //sum
        x_s = 0;
        y_s = 0;
        m = 0;
        for(n = 0;n < k;n++){
          Farther_flag = 0;
          for(l = 0;l < j;l++){
            if(step[n] == Farther_value[l]){
              Farther_flag = 1;
              m++;
            }
          }
          if(Farther_flag != 1){
            x_s += x_t_a[ step[n] ];
            y_s += y_t_a[ step[n] ];
          }
        }

        //average
        x_a = x_s / (float)(k - m);
        y_a = y_s / (float)(k - m);

        //Linear equations
        part_1 = 0;
        part_2 = 0;
        for(n = 0;n < k;n++){
          Farther_flag = 0;
          for(l = 0;l < j;l++){
            if(step[n] == Farther_value[l])
              Farther_flag = 1;
          }
          if(Farther_flag != 1){
            part_1 += x_t_a[ step[n] ] * y_t_a[ step[n] ];
            part_2 += pow( x_t_a[ step[n] ] , 2.0 );
          }
        }

        a_1[i] = ( part_1 - ((float)(k - m) * x_a * y_a) ) / ( part_2 - ( (float)(k - m) * pow(x_a , 2.0) ) );
        b_1[i] = y_a - (a_1[i] * x_a);
      }
    }

    counter = 0;
    for(i = 189;i < 400;i++){
      if(a_1[i] != 0){
        counter++;
        angle = acos( fabs(a_1[i]*a_2 + b_1[i]*b_2) / sqrt( pow(a_1[i] , 2) + pow(b_1[i] , 2) ) * sqrt( pow(a_2 , 2) + pow(b_2 , 2) ) );

        if(counter == 1){
          angle_jdg = angle * (180 / PI);
          step_s = i;
        }
        else if( fabs(90.0 - angle_jdg) > fabs(90.0 - angle * (180 / PI)) ){
          angle_jdg = angle * (180 / PI);
          step_s = i;
        }
      }
    }


    //next side line
    for(i = 900;i > 680;i--){
      if(flag == 1)
        break;

      j = i;
      k = 0;
      while(1){
        step[k] = j;
        k++;
        if(fabs(x_t[i] - x_t[i - 10]) > 0.08){
          if(fabs(x_t[i + 10] - x_t[i - 20]) > 0.08){
            if(fabs(x_t[i + 20] - x_t[i - 30]) > 0.08){
              break;
            }
          }
        }
        if(j < 219)
          break;
        j -= 10;
      }
      if(k > 4){
        x_t_s = 0;
        for(n = 0;n < k;n++)
          x_t_s += x_t_a[step[n]];
        x_t_s /= (float)k;
        j = 0;
        for(n = 0;n < k;n++){
          if(fabs(x_t_a[step[n]] - x_t_s) > 5.0){
            Farther_value[j] = step[n];
            j++;
          }
        }

        //sum
        x_s = 0;
        y_s = 0;
        m = 0;
        for(n = 0;n < k;n++){
          Farther_flag = 0;
          for(l = 0;l < j;l++){
            if(step[n] == Farther_value[l]){
              Farther_flag = 1;
              m++;
            }
          }
          if(Farther_flag != 1){
            x_s += x_t_a[ step[n] ];
            y_s += y_t_a[ step[n] ];
          }
        }

        //average
        x_a = x_s / (float)(k - m);
        y_a = y_s / (float)(k - m);

        //Linear equations
        part_1 = 0;
        part_2 = 0;
        for(n = 0;n < k;n++){
          Farther_flag = 0;
          for(l = 0;l < j;l++){
            if(step[n] == Farther_value[l])
              Farther_flag = 1;
          }
          if(Farther_flag != 1){
            part_1 += x_t_a[ step[n] ] * y_t_a[ step[n] ];
            part_2 += pow( x_t_a[ step[n] ] , 2.0 );
          }
        }

        a_1[i] = ( part_1 - ((float)(k - m) * x_a * y_a) ) / ( part_2 - ( (float)(k - m) * pow(x_a , 2.0) ) );
        b_1[i] = y_a - (a_1[i] * x_a);
      }
    }

    counter = 0;
    for(i = 900;i > 680;i--){
      if(flag == 1)
        break;

      if(a_1[i] != 0){
        counter++;
        angle = acos( fabs(a_1[i]*a_2 + b_1[i]*b_2) / sqrt( pow(a_1[i] , 2) + pow(b_1[i] , 2) ) * sqrt( pow(a_2 , 2) + pow(b_2 , 2) ) );

        if(counter == 1){
          angle_jdg = angle * (180 / PI);
          step_s = i;
        }
        else if( fabs(90.0 - angle_jdg) > fabs(90.0 - angle * (180 / PI)) ){
          angle_jdg = angle * (180 / PI);
          step_s = i;
        }	
      }
    }

    //corner
    x_c = (b_2 - b_1[step_s]) / (a_1[step_s] - a_2);
    y_c = (a_1[step_s] * x_c) + b_1[step_s];

    //reverse corner
    if(flag == 1){
      x_cc = -sqrt( pow(X , 2.0) / ( 1 + pow(a_2 , 2.0) ) ) + x_c;
      y_cc = a_2 * (x_cc - x_c) + y_c;
    }
    else{
      x_cc = sqrt( pow(X , 2.0) / ( 1 + pow(a_2 , 2.0) ) ) + x_c;
      y_cc = a_2 * (x_cc - x_c) + y_c;
    }

    //distance to corner
    cldis_1 = sqrt( pow(x_c , 2.0) + pow(y_c , 2.0) );
    cldis_2 = sqrt( pow(x_cc , 2.0) + pow(y_cc , 2.0) );

    //step of corner
    for(i = 180;i <= 900;i++){
      diff_1 = x_c - x_t_a[i];
      diff_2 = y_c - y_t_a[i];
      diff_1 = fabs(diff_1);
      diff_2 = fabs(diff_2);
      if(i == 180){
        difference = diff_1 + diff_2;
        step_1 = i;
      }
      else if(difference > diff_1 + diff_2){
        difference = diff_1 + diff_2;
        step_1 = i;
      }
    }

    for(i = 180;i <= 900;i++){
      diff_1 = x_cc - x_t_a[i];
      diff_2 = y_cc - y_t_a[i];
      diff_1 = fabs(diff_1);
      diff_2 = fabs(diff_2);
      if(i == 180){
        difference = diff_1 + diff_2;
        step_2 = i;
      }
      else if(difference > diff_1 + diff_2){
        difference = diff_1 + diff_2;
        step_2 = i;
      }
    }

    //coordinate
    if(flag == 1)
      deg = (step_2 - step_1) * 0.25;
    else
      deg = (step_1 - step_2) * 0.25;
    rad = deg * PI / 180.0;

    y = ( cldis_1 * cldis_2 * sin(rad) ) / X;
    if(flag == 1)
      x = (X / 2.0) - sqrt( pow(cldis_1 , 2.0) - pow(y , 2.0) );
    else
      x = (X / 2.0) - sqrt( pow(cldis_2 , 2.0) - pow(y , 2.0) );

    //slope
    x_0 = sqrt( pow((X / 2.0) , 2.0) - pow((y - sl) , 2.0) );
    cldis_0 = x_0 - x;
    if(flag == 1)
      stddeg = acos( ( (X / 2.0) - x) / cldis_1 ) * (180.0 / PI);
    else
      stddeg = acos( ( (X / 2.0) - x) / cldis_2 ) * (180.0 / PI);
    if(flag == 1)
      cmpdeg = (step_1 - 180) * 0.25;
    else
      cmpdeg = (step_2 - 180) * 0.25;

    slope = (cmpdeg - stddeg);

    x += 4.0;

    //fprintf(fp,"x : %f  y : %f  deg : %f\n",x,y,slope);
    //printf("x : %f  y : %f  deg : %f\n",x,y,slope);
  }

#else
  (void)time_stamp;

  int i;
  long min_distance;
  long max_distance;

  // 全てのデータの X-Y の位置を表示
  urg_distance_min_max(urg, &min_distance, &max_distance);
  for (i = 0; i < data_n; ++i) {
    long l = data[i];
    double radian;
    long x;
    long y;

    if ((l <= min_distance) || (l >= max_distance)) {
      continue;
    }
    radian = urg_index2rad(urg, i);
    x = (long)(l * cos(radian));
    y = (long)(l * sin(radian));
    printf("(%ld, %ld), ", x, y);
  }
  printf("\n");

#endif
}

int main(int argc, char *argv[])
{
  char name[255],devicename[] = "/dev/ttyACM0";
  struct termios oldtio,newtio;

  enum {
    CAPTURE_TIMES = 1,
  };
  urg_t urg;
  long *data = NULL;
  long time_stamp;
  int n;
  int i = 0,count = 0;
  if (open_urg_sensor(&urg, argc, argv) < 0) {
    return 1;
  }

  data = (long *)malloc(urg_max_data_size(&urg) * sizeof(data[0]));
  if (!data) {
    perror("urg_max_index()");
    return 1;
  }

  // データ取得
#if 0
  // データの取得範囲を変更する場合
  urg_set_scanning_parameter(&urg,
      urg_deg2step(&urg, -90),
      urg_deg2step(&urg, +90), 0);
#endif

  urg_start_measurement(&urg, URG_DISTANCE, URG_SCAN_INFINITY, 0);

  /////////////////////////////////////
  fd = open(devicename,O_RDWR|O_NONBLOCK);
  if(fd<0) 
  {
    printf("ERROR on device open.\n");
    exit(1);
  }
  ioctl(fd,TCGETS,&oldtio);
  newtio = oldtio;
  newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
  ioctl(fd,TCSETS,&newtio);
  ///////////////////////////////////////
  fp = fopen("data2.txt","w");
  start = 0;
  end = 0;
  while (1) {
    n = urg_get_distance(&urg, data, &time_stamp);

    if (n <= 0) {
      printf("urg_get_distance: %s\n", urg_error(&urg));
      free(data);
      urg_close(&urg);
      return 1;
    }
    if(end - start >= 500){
      print_data(&urg, data, n, time_stamp, 0, 1080); 
      //serial_arduinoread(devicename,name);
      serial_arduinowrite(devicename,(char *)"whatyourname",count);
      printf("\n\n");
      start = clock();
    }
    count++;
    if(count == 10)
      count = 0;

    end = clock();
  }
  //////////////////////////////////////////
  ioctl(fd,TCSETS,&oldtio);

  close(fd);
  //////////////////////////////////////////
  fclose(fp);
  // 切断
  free(data);
  urg_close(&urg);

#if defined(URG_MSC)
  getchar();
#endif
  return 0;
}
int serial_arduinowrite(char *devicename,char *messege,int i)
{
  struct termios oldtio,newtio;
  int a,b,j;
  int sum = 0;
  char temp,mark[255];
  char buf[255];

  a = x * 1000.0;
  ///////////////////////////////
  if(a < 0 || 8000 < a){
    for(j = 0;j < 10;j++)
      sum += error[j];
    a = sum / 10;
  }
  error[i] = a;
  ///////////////////////////////

  //printf("a : %d\n",a);
  fprintf(fp,"x : %f  y : %f  deg : %f\n",a,y,slope);

  strcpy(buf,"");
  mark[0] = 126;
  //printf("mark[0] : %c\n",mark[0]);
  write(fd,mark,1);

  temp = a;
  buf[0] = a>>8;
  //printf("buf[0] : %c\n",buf[0]);
  write(fd,buf,1);

  buf[0] = temp;
  //printf("buf[0] : %c\n",buf[0]);
  write(fd,buf,1);

  return 0;
}

int serial_arduinoread(char *devicename,char *messege)
{
  char mes[255];
  char CR[2],LF[2];
  int flg,len;

  flg = 1;
  strcpy(mes,"");
  while(flg)
  {
    strcpy(rec,"");
    len = read(fd,rec,1);

    if(len == 0)
    {
      printf("continue\n");
      continue;
    }
    else 
    {
      strncat(mes,rec,len);
      if(strstr(mes,CR) != NULL || strstr(mes,LF) != NULL)
      {
        flg = 0;
      }
      break;
    }
  }

  return 0;
}
