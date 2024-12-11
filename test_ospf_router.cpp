#include "OSPF.h"

int main(void){
    
    OSPF ospf(0x1111111, 10, 40, 1050);
    
    ospf.start();

    return 0;
}