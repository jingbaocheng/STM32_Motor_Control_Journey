#include "Arc_Control.h"
#include <stdlib.h>

static void Output_X(void) {
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);
    for(volatile int d=0; d<100; d++);
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);
}

static void Output_Y(void) {
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_RESET);
    for(volatile int d=0; d<100; d++);
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_SET);
}
void Arc_Interpolate_Process(Arc_Task_t *pTask) {
    int32_t centerX = pTask->startX + pTask->I;
    int32_t centerY = pTask->startY + pTask->J;
    
    int32_t x = pTask->startX - centerX;
    int32_t y = pTask->startY - centerY;
    int32_t x_end = pTask->endX - centerX;
    int32_t y_end = pTask->endY - centerY;
    
    int32_t F = 0;
    uint32_t safe_limit = (abs(x_end - x) + abs(y_end - y)) * 2;

    for (uint32_t i = 0; i < safe_limit; i++) {
        if (x == x_end && y == y_end) break;

        // 象限判定及进给逻辑
        if (x > 0 && y >= 0) { // 第一象限
            if (pTask->dir == DIR_CW) {
                if (F >= 0) { y--; F = F - 2*y + 1; Output_Y(); }
                else        { x++; F = F + 2*x + 1; Output_X(); }
            } else {
                if (F >= 0) { x--; F = F - 2*x + 1; Output_X(); }
                else        { y++; F = F + 2*y + 1; Output_Y(); }
            }
        }
        else if (x <= 0 && y > 0) { // 第二象限
            if (pTask->dir == DIR_CW) {
                if (F >= 0) { x++; F = F + 2*x + 1; Output_X(); }
                else        { y++; F = F + 2*y + 1; Output_Y(); }
            } else {
                if (F >= 0) { y--; F = F - 2*y + 1; Output_Y(); }
                else        { x--; F = F - 2*x + 1; Output_X(); }
            }
        }
        else if (x < 0 && y <= 0) { // 第三象限
            if (pTask->dir == DIR_CW) {
                if (F >= 0) { y++; F = F + 2*y + 1; Output_Y(); }
                else        { x--; F = F - 2*x + 1; Output_X(); }
            } else {
                if (F >= 0) { x++; F = F + 2*x + 1; Output_X(); }
                else        { y--; F = F - 2*y + 1; Output_Y(); }
            }
        }
        else if (x >= 0 && y < 0) { // 第四象限
            if (pTask->dir == DIR_CW) {
                if (F >= 0) { x--; F = F - 2*x + 1; Output_X(); }
                else        { y--; F = F - 2*y + 1; Output_Y(); }
            } else {
                if (F >= 0) { y++; F = F + 2*y + 1; Output_Y(); }
                else        { x++; F = F + 2*x + 1; Output_X(); }
            }
        }
        HAL_Delay(pTask->stepDelay);
    }
}
