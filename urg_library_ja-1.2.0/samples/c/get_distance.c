/*!
\example get_distance.c �����f�[�^���擾����
\author Satofumi KAMIMURA

$Id: get_distance.c,v c5747add6615 2015/05/07 03:18:34 alexandr $
*/

#include "urg_sensor.h"
#include "urg_utils.h"
#include "open_urg_sensor.h"
#include <stdlib.h>
#include <stdio.h>


static void print_data(urg_t *urg, long data[], int data_n, long time_stamp, int rad_s, int rad_e)
{
#if 1

	int front_index;
	int i;
	int convex[1080] = {0};

	(void)data_n;

	// �O���̃f�[�^�݂̂�\��
	for(i = rad_s;i <= rad_e;i++){
		front_index = urg_step2index(urg, i);
		printf("%d    :   %ld [mm], (%ld [msec])\n", i, data[i], time_stamp);
		if(i > 0 && i+1 < rad_e){
			if(data[i-1] < data[i] && data[i] > data[i+1])
				convex[i] = 1;
		}
		//printf("%d\n",front_index);
	}
	for(i = rad_s;i <= rad_e;i++){
		if(convex[i] == 1)
			printf("Convex : %d Distance : %ld [mm]\n",i,data[i]);
	}

#else
	(void)time_stamp;

	int i;
	long min_distance;
	long max_distance;

	// �S�Ẵf�[�^�� X-Y �̈ʒu��\��
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

	// �f�[�^�擾
#if 0
	// �f�[�^�̎擾�͈͂�ύX����ꍇ
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

	// �ؒf
	free(data);
	urg_close(&urg);

#if defined(URG_MSC)
	getchar();
#endif
	return 0;
}

