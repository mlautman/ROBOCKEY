#include <stdio.h>
#include <math.h>

int main(void ){
	int a  =1234;
	char* low = (char*)&a;
	int* b = ((int*)low);
	printf("%d", *b);
}
