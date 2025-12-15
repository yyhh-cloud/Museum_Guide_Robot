#ifndef __K210_H
#define __K210_H

#include "main.h"

// 帧头帧尾 (对应 Python)
#define K210_HEAD 0xCC
#define K210_END  0xDD

// 数据包结构体 (必须对齐)
typedef struct __attribute__((packed)) { 
    uint8_t  head;      
    uint16_t cam_w;     
    uint16_t cam_h;     
    
    // === 核心数据 ===
    uint16_t x;         // 偏移量
    uint16_t y;         // Tag ID (或 Y坐标)
    uint16_t z;         // 距离   (或 面积)
    
    uint8_t  bcc;       
    uint8_t  end;       
} K210_Recvmsg;

extern K210_Recvmsg k210; 
uint8_t k210_data_callback(uint8_t recv);

#endif