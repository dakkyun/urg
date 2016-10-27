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

#define PI 3.1415


static void print_data(urg_t *urg, long data[], int data_n, long time_stamp, int rad_s, int rad_e)
{
#if 1

	int front_index;
	int i,j;
	int convex[1080] = {0},clcon[] = {0};
	long cldis[] = {0};
	
	double X = 10.0,Y = 7.0;
	double x,y;
	double deg,rad;	
	
	(void)data_n;

	// 前方のデータのみを表示
	for(i = rad_s;i <= rad_e;i++){
		//front_index = urg_step2index(urg, i);
		printf("%d    :   %ld [mm], (%ld [msec])\n", i, data[i], time_stamp);
		if(i > 0 && i+1 < rad_e){
			if(data[i-1] < data[i] && data[i] > data[i+1])
				convex[i] = 1;
		}
		//printf("%d\n",front_index);
	}
	j = 0;
	for(i = rad_s;i <= rad_e;i++){
		if(convex[i] == 1){
			printf("Convex : %d Distance : %ld [mm]\n",i,data[i]);
			cldis[j] = data[i];
			clcon[j] = i;
			j++;
		}
		
	}
	
	//calculation
	
	deg = (clcon[3] - clcon[1]) * 0.25;
	rad = deg * PI / 180.0;

	y = cldis[1] * 1000 * cldis[3] * 1000 * sin(rad) / X;
	x = (X / 2.0) - sqrt( pow(cldis[3] * 1000 , 2.0) - pow(y , 2.0) );
	printf("coordinate : (%f , %f)\n",x,y);

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
		CAPTURE_TIMES = 2,
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

