#include <stdio.h>
#include <stdlib.h>
#include <math.h>

int t[1000];
double sin_out[1000];
double cos_out[1000];
double trap_out[1000];
float dt = 0.0001;

float Trapezoid(float trap, int t){
    trap += (dt/2)*(cos_out[t]+cos_out[t+1]);
    return trap;
}

void realfunc(){
    for(int i=0;i<1000;i++){
        sin_out[i] = sin(i*dt);
        cos_out[i] = cos(i*dt);
        t[i]=i;
    }
}

int main(void){

    float trap = 0;

    realfunc();
    for(int i=0;i<1000;i++){
        printf("\nsin(%.2f) = %.5f",i*dt,sin_out[i]);
        trap = Trapezoid(trap,i);
        trap_out[i] = trap;
        printf(", trap(%.2f) = %.5f",i*dt,trap_out[i]);
        printf(", Error: = %.5f",sin_out[i]-trap_out[i]);
    }

}
