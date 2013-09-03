#include<stdio.h>
#include<unistd.h>
#include <itead.h>

#define pin 0

void main(void)
{
	int i;
	
	pinMode(pin,PWM_OUTPUT);
	for(i=0;i<255;i++)
	{
		printf("the value is %d\n",i);
		analogWrite(pin,i);
		delay(50);
	}
}

