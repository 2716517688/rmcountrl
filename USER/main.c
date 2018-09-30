# include "stm32f4xx.h"
# include "stm32f4xx_gpio.h"
# include "stdio.h"
# include "usart.h"
# include "sys.h"
# include "delay.h"
# include "iic.h"
# include "stm32f4xx_wwdg.h"
# include "stm32f4xx_rcc.h"

# define FALSE 0
# define TRUE 1
#define u8 unsigned char
# define u32 unsigned int
# define u16 unsigned short
int rm_chanel[6];
void rm_deal()
{
  rm_chanel[0]=USART1_RX_BUF[12]+(USART1_RX_BUF[13]&0x07)*256;
	rm_chanel[1]=((USART1_RX_BUF[13]&0xf8)>>3)+(USART1_RX_BUF[14]&0x3f)*32;
	rm_chanel[2]=((USART1_RX_BUF[14]&0xc0)>>6)+USART1_RX_BUF[15]*4+(USART1_RX_BUF[16]&0x01)*512;
	rm_chanel[3]=((USART1_RX_BUF[16]&0xfe)>>1)+(USART1_RX_BUF[17]&0x0f)*128;
	rm_chanel[4]=(USART1_RX_BUF[17]&0xc0)>>6;
	rm_chanel[5]=(USART1_RX_BUF[17]&0x30)>>4;
}

void main()
{ 
	unsigned char res=0;
	int i,j;
	uart1_init(100000);
	uart4_init(100000);
	delay_init(7000);
	while(1)
	{
    if(USART1_RX_STA>17)
		{
//			for(i=0;i<2;i++)
//			{
//			 printf("%x ",USART1_RX_BUF[i]);
//			}
//			delay_ms(2);
		  rm_deal();
		   for(i=0;i<=5;i++)
			{
			printf("%d ",rm_chanel[i]);
			}
			printf("\n");
			
			for(j=0;j<18;j++)
			{
					USART1_RX_BUF[j]=0;
			}
			USART1_RX_STA=0;
		}
	}
}