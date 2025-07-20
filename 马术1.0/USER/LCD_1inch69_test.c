#include "image.h"
#include "LCD_Test.h"
#include "LCD_1in69.h"
#include "DEV_Config.h"
extern 	
	uint8_t buf[12];

extern TIM_HandleTypeDef htim1;

void LCD_1in69_test()
{

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

    LCD_1IN69_SetBackLight(900);
    LCD_1IN69_Init(HORIZONTAL);
    

    Paint_NewImage(LCD_1IN69_WIDTH, LCD_1IN69_HEIGHT+60, 0, WHITE);


    Paint_SetClearFuntion(LCD_1IN69_Clear);
    Paint_SetDisplayFuntion(LCD_1IN69_DrawPoint);

    LCD_1IN69_Clear(WHITE);

    //Paint_SetRotate(180);
//		Paint_DrawString_EN(30, 0, "POS:", &Font24, WHITE, BLACK);  
		//Paint_DrawString_EN(10, 0, "POWER:", &Font24, WHITE, BLACK);
		//Paint_DrawString_EN(10, 34, "the1:", &Font24, WHITE, BLACK);
	//	Paint_DrawString_EN(10, 68, "the2:", &Font24, WHITE, BLACK);
    
   //Paint_DrawFloatNum (30, 50, 987.654321,3, &Font12, YELLOW, RED);
//	  Paint_DrawFloatNum (10, 0, buf[0],3 , &Font24, WHITE, BLACK);
//		Paint_DrawFloatNum (10, 34,buf[1],3, &Font24, WHITE, BLACK);
//	  Paint_DrawFloatNum (10, 68,buf[2],3, &Font24, WHITE, BLACK);
  // Paint_DrawString_CN(100, 30, "buf[0]", &Font24CN, WHITE, BLUE);


    
//    Paint_DrawImage(gImage_adm, 20, 100, 180, 176);

//    Paint_DrawRectangle(185, 10, 285, 58, RED, DOT_PIXEL_2X2, DRAW_FILL_EMPTY);
//    Paint_DrawLine(185, 10, 285, 58, MAGENTA, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
//    Paint_DrawLine(285, 10, 185, 58, MAGENTA, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
//    
//    Paint_DrawCircle(120, 60, 25, BLUE, DOT_PIXEL_2X2, DRAW_FILL_EMPTY);
//    Paint_DrawCircle(150, 60, 25, BLACK, DOT_PIXEL_2X2, DRAW_FILL_EMPTY);
//    Paint_DrawCircle(190, 60, 25, RED, DOT_PIXEL_2X2, DRAW_FILL_EMPTY);
//    Paint_DrawCircle(145, 85, 25, YELLOW, DOT_PIXEL_2X2, DRAW_FILL_EMPTY);
//    Paint_DrawCircle(165, 85, 25, GREEN, DOT_PIXEL_2X2, DRAW_FILL_EMPTY);
//    
    //DEV_Delay_ms(3000);

    //DEV_Module_Exit();
}

