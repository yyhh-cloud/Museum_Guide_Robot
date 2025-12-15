#include "k210.h"

K210_Recvmsg k210; // 全局变量

// 校验函数
unsigned char calculateBCC(const unsigned char *data, uint16_t length) {
    unsigned char bcc = 0;
    uint16_t i = 0;
    for (i = 0; i < length; i++) {
        bcc ^= data[i];
    }
    return bcc;
}

// 接收回调 (无需改动逻辑，只要引用了新的 .h 即可)
uint8_t k210_data_callback(uint8_t recv)
{
    static uint8_t recvlen = sizeof(k210);
    static uint8_t recv_data[32];
    static uint8_t recv_counts = 0;
    uint8_t data_ready = 0;
    
    recv_data[recv_counts] = recv;
    
    if(recv == K210_HEAD || recv_counts > 0) recv_counts++;
    else recv_counts = 0;
    
    if(recv_counts == recvlen)
    {
        recv_counts = 0;
        if(recv_data[recvlen-1] == K210_END)
        {
            if(recv_data[recvlen-2] == calculateBCC(recv_data, recvlen-2))
            {
                uint8_t *recvptr = (uint8_t*)&k210;
                for(uint8_t i=0; i<recvlen; i++) {
                    *recvptr = recv_data[i];
                    recvptr++;
                }
                data_ready = 1;
            }
        }
    }
    return data_ready;
}