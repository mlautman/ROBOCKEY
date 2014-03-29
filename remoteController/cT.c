#include <stdio.h>
#include <math.h>

int main(void ){
	int j = 123;
	char* pCH =(char*)&j;

	char* pCL=pCH+1;// (char*)((int)&j+1);

	int a;
	int* aP=(int*)pCH;
	printf("%d\n",(int)*aP);
}
