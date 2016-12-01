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

#define PI 3.141592653589793

struct curve{
		double element[3];
		int flag;
	};
struct straight{
		int step[3];
		int flag;
	};


static void print_data(urg_t *urg, long data[], int data_n, long time_stamp, int rad_s, int rad_e)
{
#if 1

	int front_index;
	int i,j,k,max,min;
	int clcon[1081] = {0};
	int convex[1081] = {0};
	int decision[1081] = {0};
	int step[10] = {0};
	long cldis1[1081] = {0};
	
	double X = 8.0,Y = 5.0,sp = 0;
	double x,y,x0,x_t[1081] = {0},y_t[1081] = {0};
	double deg,rad,stddeg,cmpdeg,slope,radius;
	double cldis0;
	double decision_1,decision_2;
	double x_s,y_s,x_a,y_a,part_1,part_2,a,b;
	
	struct curve q[1081] = {0};
	struct straight p[1081] = {0};
	
	(void)data_n;

	// 前方のデータのみを表示
	for(i = rad_s;i <= rad_e;i++){
		printf("%d    :   %ld [mm], (%ld [msec])\n", i, data[i], time_stamp);
		if(i >= 180 && i <= 900){
			//tunnel coodinate
			rad = (i - 180) * 0.25 * (PI / 180.0);	
			x_t[i] = (data[i] / 1000.0) * cos(-rad);
			y_t[i] = (data[i] / 1000.0) * sin(-rad);	
		}
	}
	//Linear approximation
	for(i = 180;i <= 900;i++){
		for(j = 0;j < 10;j++){
			for(;;){
				if(data[i] != 65533)
					break;
				i++;
			}
			step[j] = i;
		}

		//max and min
		max = step[0];
		min = step[0];
		for(j = 1;j < 10;j++){
			if(data[max] < data[ step[j] ])
				max = step[j];
			if(data[min] > data[ step[j] ])
				min = step[j];
		}
		//sum
		x_s = 0;
		for(j = 0;j < 10;j++){
			if(step[j] != max || step[j] != min)
				x_s += x_t[ step[j] ];
		}
		y_s = 0;
		for(j = 0;j < 10;j++){
			if(step[j] != max || step[j] != min)
				y_s += y_t[ step[j] ];
		}
		//average
		x_a = x_s / 8;
		y_a = y_s / 8;
		//Linear equations
		part_1 = 0;
		part_2 = 0;
		for(j = 0;j < 10;j++){
			if(step[j] != max || step[j] != min){
				part_1 += x_t[ step[j] ] * y_t[ step[j] ];
				part_2 += pow( x_t[ step[j] ] , 2.0 );
			}
		}
		a = ( part_1 - (8 * x_a * y_a) ) / ( part_2 - ( 8 * pow(x_a , 2.0) ) );
		b = y_a - (a * x_a);
	}
	//display
	for(i = 180;i <= 900;i++)
		printf("a : %f     b : %f\n",a,b);
	/*
	//Curve equation
	for(i = 180;i <= 900;i++){
		if(i == 180 || i == 850){
			for(;;){
				if(data[i] != 65533)
					break;
				i++;
			}
			j = 10;
			for(;;){
				if(data[i+j] != 65533)
					break;
				j++;
			}
			k = j + 10;
			for(;;){
				if(data[i+k] != 65533)
					break;
				k++;
			}
		
			q[i].element[0] = ( (y_t[i] - y_t[i+j]) * (x_t[i+k] - x_t[i+j]) - (x_t[i] - x_t[i+j]) * (y_t[i+k] - y_t[i+j]) ) / ( ( pow(x_t[i] , 2.0) - pow(x_t[i+j] , 2.0) ) * (x_t[i+k] - x_t[i+j]) - (x_t[i] - x_t[i+j]) * ( pow(x_t[i+k] , 2.0) - pow(x_t[i+j] , 2.0) ) );
			q[i].element[1] = ( (y_t[i] - y_t[i+j]) / (x_t[i] - x_t[i+j]) ) - ( ( ( pow(x_t[i] , 2.0) - pow(x_t[i+j] , 2.0) ) * q[i].element[0] ) / (x_t[i] - x_t[i+j]) );
			q[i].element[2] = y_t[i] - ( q[i].element[0] * pow(x_t[i] , 2.0) ) + (q[i].element[1] * x_t[i]);
			q[i].flag = 1;
		}
	}

	//display
	for(i = 180;i <= 900;i++){
		if(q[i].flag == 1)
			printf("Curve equation : y = %fx^2 + %fx + %f\n",q[i]);
	}
	//curvature radius
	for(i = 180;i <= 900;i++){
		if(q[i].flag == 1){
			radius = ( pow( 1 + pow( 2 * q[i].element[0] * x_t[i] + q[i].element[1] , 2.0 ) , 1.5 ) ) / ( 2 * q[i].element[0] );
			printf("R = %f\n",radius);
		}
	}*/
	//bottom line
	/*for(i = 180;i <= 900;i++){
		for(j = 0;j < 10;j++){
			if




		if(-0.01 < y_t[i+j] - y_t[i] && y_t[i+j] - y_t[i] < 0.01)
			decision[i] = 1;

		decision_1 = (x_t[i+j] - x_t[i]) / (y_t[i+j] - y_t[i]);
		p[i].step[0] = i;
		p[i].step[1] = i+j;
		
		for(;;){
			j++;
			if(data[i+j] != 65533)
				break;

		}
		decision_2 = (x_t[i+j] - x_t[i]) / (y_t[i+j] - y_t[i]);
		p[i].step[2] = i+j;
			
		if(-0.05 < decision_2 - decision_1 && decision_2 - decision_1 < 0.05)
			p[i].flag = 1;

		else if(decision[i] == 1 && -0.01 < y_t[i+j] - y_t[i] && y_t[i+j] - y_t[i] < 0.01)
			p[i].flag = 1;

		printf("%d  %f\n",i,decision_2 - decision_1);
	}
	
	for(i = 180;i <= 900;i++)
		printf("tunnel coodinate : %d  %ld[mm]  (%lf , %lf)\n",i,data[i],x_t[i],y_t[i]);

	for(i = 180;i<= 900;i++){
		printf("step : %d  %d  %d  flag : %d\n",p[i].step[0],p[i].step[1],p[i].step[2],p[i].flag);
	}*/

	/*j = 0;
	for(i = rad_s;i <= rad_e;i++){
		if(convex[i] == 1){
			printf("Convex : %d Distance : %ld [mm]\n",i,data[i]);
			cldis1[j] = data[i];
			clcon[j] = i;
			j++;
			convex[i] = 0;
		}
		
	}
	
	//coordinate
	
	deg = (clcon[1] - clcon[0]) * 0.25;
	rad = deg * PI / 180.0;

	y = ( (cldis1[0] / 1000.0) * (cldis1[1] / 1000.0) * sin(rad) ) / X;
	x = (X / 2.0) - sqrt( pow(cldis1[1] / 1000.0 , 2.0) - pow(y , 2.0) );
	printf("coordinate : (%f [m], %f [m])\n",x,y);

	//slope

	x0 = sqrt( pow((X / 2.0) , 2.0) - pow((y - sp) , 2.0) );
	printf("x0:%lf    x:%lf    y:%lf\n",x0,x,y);
	cldis0 = x0 - x;
//	stddeg = acos( x0 * (X / 2) - pow(y, 2.0) / (cldis0 * (cldis1[0] / 1000.0))) * 180.0 / PI;
	printf("cldis1[0] : %ld      clcon[0] : %d\n",cldis1[0],clcon[0]);
	stddeg = acos( ( (X / 2.0) - x) / (cldis1[0] / 1000.0) ) * (180.0 / PI);
	cmpdeg = (clcon[0] - 180) * 0.25;

	slope = (stddeg - cmpdeg);
	printf("stddeg : %lf    cmpdeg : %lf\n",stddeg,cmpdeg);
	printf("slope : %lf [deg]\n",slope);*/


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
	enum {
		CAPTURE_TIMES = 1,
	};
	urg_t urg;
	long *data = NULL;
	long time_stamp;
	int n;
	int i;
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
	while (1) {
		n = urg_get_distance(&urg, data, &time_stamp);
		if (n <= 0) {
			printf("urg_get_distance: %s\n", urg_error(&urg));
			free(data);
			urg_close(&urg);
			return 1;
		}
		print_data(&urg, data, n, time_stamp, 0, 1080);
	}

	// 切断
	free(data);
	urg_close(&urg);

#if defined(URG_MSC)
	getchar();
#endif
	return 0;
}

