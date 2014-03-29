#include <stdio.h>
#include <math.h>

int main(void ){
	int a[3] = {0};
	a[1] = 2;
	a[ a[1]] = 21;

	printf("%d\t%d\t%d\n\r", a[0], a[1], a[2]);
}
