#include<stdio.h>
int main()
{
	FILE *fp;
	long data[1081] = {0};

	fp = fopen("test.txt","r");
	while( ( fgets(data,4,fp) ) != NULL ){
		printf("%ld",data);

	fclose(fp);
	return 0;
}


