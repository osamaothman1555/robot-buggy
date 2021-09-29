#include "mbed.h"
#include "QEI.h"
#define FORWARD 1
#define BACKWARD 0
//variables to check and print intermediate values
float u_read;
float error_read;
float tmp_right_pwm;
float tmp_left_pwm;
float rpwm, lpwm;

    // input: 
    // output: 
// used to implement bluetooth
    int count_ir = 0;
    int count_ar = 0;
    int count_al = 0;
    int count_il =0;

//used to print values in arduino
Serial PC(USBTX, USBRX);

class LED { 
private: 
 DigitalOut outputSignal; 
public: 
 LED(PinName pin) : outputSignal(pin) {} 
 void on(void) {outputSignal = 1;} 
 void off(void) {outputSignal = 0;} 
 void toggle(void) { 
 if (outputSignal.read()) 
 outputSignal = 0; 
 else 
 outputSignal = 1; 
 } 
 
 int status(void) {return outputSignal.read();} 
}; //end of class
/////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////



class speed {

    private:
    //Ticker timer;
    int prev_pulses_count, current_pulses_count;
    QEI motor_speed; // where i use the QEI class
    float wheel_radius, sample_time, wheel_speed, tick_rate;

    public:
    speed(float stime, PinName chA, PinName chB): prev_pulses_count(0), current_pulses_count(0),motor_speed(chA, chB, NC, 256),wheel_radius(0.04),sample_time(stime){
        // previously used for calculating speed every sample time but here we use pulses
        //  timer.attach(callback(this, &speed::calc_wheel_speed), sample_time); 
          motor_speed.reset(); // resets pulse count to 0
          };
    ~speed(){ };
    
    public:
    
       int get_wheel_pulses(){return motor_speed.getPulses();};
        
       float get_sample_time(){ return sample_time;};
       void reset(){ motor_speed.reset();}
    
    // input: current Pulses 
    // output: wheel speed in m/s
       float calc_wheel_speed(){
            current_pulses_count = get_wheel_pulses();
            tick_rate = (current_pulses_count - prev_pulses_count)/sample_time;
            wheel_speed = ((tick_rate))/(256*2); // 256 is the counts per revolution but we use it in X2 mode so we get 512counting at every rising edge
            prev_pulses_count = current_pulses_count;
            return wheel_speed;
            //timer.attach(callback(this, &speed::write_speed), sample_time);
       };
};// end of class
/////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

       
       
class motor_control{
    
    private:
    DigitalOut control_mode, direc;
    float pwm_period, pwm_duty;
    PwmOut motor;
    
    public:
    motor_control( int mode, int direction ,PinName pin_control_mode,PinName pin_direc, PinName pin_motor): control_mode(pin_control_mode), direc(pin_direc), motor(pin_motor)
    { 
    if (mode) { unipolar();} 
        else{ bipolar();}
    if (direction){ forward();}
    else{ backward();}
    stop();
    };
    ~motor_control() { };
    
    
    void unipolar (){ 
        control_mode = 0;
        };
    void bipolar (){ 
       control_mode = 1;
        };
    
        
    void set_pwm(float period, float duty){
        pwm_period = period;
        pwm_duty = duty; 
        motor.period(pwm_period);
        motor.write(pwm_duty);
        };
    
    // input: N/A
    // output: stop motors completely
    void stop(){
        motor.write(1);
        };
    
    // input: N/A
    // output: setting wheel direction
    void forward (){ 
        direc = 0;
        };
    void backward() { 
        direc = 1;
        };
        
    };// end of class
/////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

class PID_speed{
    
    private:
        float sample_time; 
        float Ki, Kp, Kd;
        float error0, error1, error2;
        float u1, u0; // for right motor
    
        float ref_rps;
        float speed_sample_time;
        Ticker speed_control;
        speed  encmotor_right, encmotor_left;
        float rps_right, rps_left, average_rps;
    
        float rwheel_speed, rtick_rate;
        int rprev_pulses_count, rcurrent_pulses_count;
        float  lwheel_speed, ltick_rate;
        int lprev_pulses_count, lcurrent_pulses_count;
    
        float right_pwm, left_pwm, base_pwm;
        motor_control mright, mleft;
        float mright_period, mleft_period;
        float bright_pwm, bleft_pwm;
        
        int stop_truth;
        int moving;
        int rotate_truth;
    
    public:
         //  speed(float stime, PinName chA, PinName chB)
         PID_speed(float pwm_base_right, float pwm_base_left, float sample_time, float rpm, float mright_period, float mleft_period, float Kp, float Ki, float Kd): 
         mleft( 1, 1,PA_11, PB_2, PB_14), mright( 1, 1,PB_12, PB_1, PB_13)
         , bright_pwm(pwm_base_right), bleft_pwm(pwm_base_left),
         speed_sample_time(sample_time), ref_rps(rpm), mright_period(mright_period), mleft_period(mleft_period),
         Kp(Kp), Ki(Ki), Kd(Kd),
         encmotor_right(sample_time,PC_14, PC_15),encmotor_left(sample_time, PC_10, PC_12)
         { 
         speed_control.attach(callback(this, &PID_speed::PID), speed_sample_time);
         u1 = 0;
         rps_left = 0;
         rps_right = 0;
         average_rps = 0;
         stop_truth = 0;
         moving = 0;
         rotate_truth = 0;
         }; // sample time to sample pulses 
         
        ~PID_speed(){ };
    
    
    // input: control action
    // output: change of PWM of right and left wheel in response to control
    void do_speed(float u_sense){
        stop_truth = 0;
        rotate_truth = 0;
        right_pwm =  0.6  -u1 - u_sense;
        left_pwm =   0.6025  -u1 + u_sense;
        //tmp_right_pwm = right_pwm;
        //tmp_left_pwm= left_pwm;
        limit_0_to_1(right_pwm);
        limit_0_to_1(left_pwm);
        mright.forward();
        mleft.forward();
        mright.set_pwm(mright_period, right_pwm);
        mleft.set_pwm(mleft_period, left_pwm);
        };
  
    void stop() {
        stop_truth = 1;
        mright.set_pwm(0.00001, 1);
        mleft.set_pwm(0.00001, 1);
        u1 = 0;
        };
     
    void rotate_left(){
        rotate_truth = 1;
        mright.set_pwm(0.00001, 0.65);
        mleft.set_pwm(0.00001, 0.6525);
        mright.backward();
        mleft.forward();
        u1=0;
        };
        
    void rotate_right(){
        rotate_truth = 1;
        mright.set_pwm(0.00001, 0.65);
        mleft.set_pwm(0.00001, 0.6525);
        mright.forward();
        mleft.backward();
        u1=0;
        };
  
   void set_moving(int var){moving = var;} 
  
  void zero_u1(){u1=0;};
            
   void update_rps(){
        lwheel_speed = encmotor_left.calc_wheel_speed();
        rwheel_speed = -(encmotor_right.calc_wheel_speed());
        };
    
    // function to limit var from 0 to 1
    void limit_0_to_1(float &var){
        if(var>1){ var = 1;}
        else if(var < 0){ var = 0;}
        };

    void get_avg_rps(){
        update_rps();
        average_rps = (lwheel_speed + rwheel_speed)/2.0;
        };
    
    void calc_u_rps(){
        u0 = u1;
        error0 = error1;
        error1 = error2;
        error2 = ref_rps - average_rps;
        u1 =   (u0 - (error1*Kp) + (Kp*error2) + (Ki*speed_sample_time*error2) + ((Kd*(error2-(2*error1)+error0))/speed_sample_time));
        /*if (u2 > umax) {u2 = umax;}
        if (u2 < umin) {u2 = umin;}*/
        //PC.printf("error = %.2f \n", error2); //found that avg still zero
        };
    
    void PID(){
        if(rotate_truth==0){
        if(stop_truth==0){
        get_avg_rps();
        calc_u_rps();
        }
        }
        //PC.printf("u = %.3f \n", u1);
        //PC.printf("error = %.2f \n", error2); //found that avg still zero
        };
    
};// end of class
/////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////


class PID_sensors {
    
    private:
        float Ki, Kp, Kd, s1, s2, s3, s4, s5, s6;
        float err0, err1, err2;
        float  u2, u1, umin, umax;
        AnalogIn sensor1, sensor2, sensor3, sensor4, sensor5, sensor6;
        LED led1,led2,led3,led4,led5,led6;
        Ticker calc_control;
        float sampling_time;
        PID_speed* wheel_eyes;
        int bluetooth_right;
        int bluetooth_left;
    
    public:
    PID_sensors(float ki, float kp, float kd, float stime_sense, float right_base_pwm, float left_base_pwm, PID_speed* wheel_sense): 
    sensor1(A0), sensor2(A1), sensor3(A2), sensor4(A3), sensor5(A4), sensor6(A5),
    led1(D2), led2(D3), led3(D4), led4(D5), led5(D6), led6(D7),
    Ki(ki), Kp(kp), Kd(kd),
    sampling_time(stime_sense), wheel_eyes(wheel_sense)
    {
        calc_control.attach(callback(this, &PID_sensors::whole_PID), sampling_time);
        err1=0;
        err0=0;
        umax = -0.5;
        umin = 0.5;
        u1 = 0;
        u2 = 0;
        bluetooth_right =0;
        bluetooth_left=0;
        led1.on();
        led2.on();
        led3.on();
        led4.on();
        led5.on();
        led6.on();
        };
        
    ~PID_sensors(){ };
    
    
    void sampleAll(){
        led1.on();
        led2.on();
        led3.on();
        led4.on();
        led5.on();
        led6.on();
        s5 = sensor5.read_u16() >> 6;
        s2 = sensor2.read_u16() >> 6;
        s3 = sensor3.read_u16() >> 6;
        s4 = sensor4.read_u16() >> 6;
        s1 = sensor1.read_u16() >> 6;
        s6 = sensor6.read_u16() >> 6;
        };
    
    void set_bluetooth_left(int var){
        bluetooth_left = var;
        }; 
           
    void set_bluetooth_right(int var){
        bluetooth_right = var;
        };    
    
    
    // input: sensor integer voltage 
    // output: sensors integer voltage normalized 
    // Purpose: 
    void normalize() {
        int black1=254, black2=227, black3=248, black4=215, black5=274, black6=165;
        int white=1023;
        if (s5 < black5) {s5 = black5;}
        if (s5 > white) {s5 = white;}
        s5 = ((s5 - black5)/(white - black5))*1000;
        if (s1 < black1) {s1 = black1;}
        if (s1 > white) {s1 = white;}
        s1 = ((s1 - black1)/(white - black1))*1000;    
        if (s2 < black2) {s2 = black2;}
        if (s2 > white) {s2 = white;}
        s2 = ((s2 - black2)/(white - black2))*1000;
        if (s3 < black3) {s3 = black3;}
        if (s3 > white) {s3 = white;}
        s3 = ((s3 - black3)/(white - black3))*1000;
        if (s4 < black4) {s4 = black4;}
        if (s4 > white) {s4 = white;}
        s4 = ((s4 - black4)/(white - black4))*1000;
       /* if (s6 < black6) {s6 = black6;}
        if (s6 > white) {s6 = white;}
        s6 = ((s6 - black6)/(white - black6))*1000;*/
        };
    
    // input: normalized sensor values
    // output: error from left most sensor to center of line
    float calc_weighted(){
        float w1=15, w2=30, w3=45, w4=60, w5=75;
        float numerator = ((s1*w1)+(s2*w2)+(s3*w3)+(s4*w4)+(s5*w5));
        float denominator = s1+s2+s3+s4+s5;
        float avg = numerator/denominator;
        float error = avg - 45;
        return error;// weighted average
        };
        
    void calc_u(){
        u1 = u2;
        err0 = err1;
        err1 = err2;
        err2 = calc_weighted();
        u2 = ( u1 - (err1*Kp) + (Kp*err2) + (Ki*sampling_time*err2) + ((Kd*(err2-(2*err1)+err0))/sampling_time))/1000;
    //PC.printf("u2 = %f  \n err2 = %f \n", u2,err2);
    //u_read= u2;
        };
            
        
    void whole_PID(){
        //PC.printf("ki = %f \n kd = %f \n kp =%f \n", Ki, Kd, Kp);
        sampleAll();
        normalize();
        if (bluetooth_left == 1){
            bluetooth_right = 0;
            if(count_al<1){
                wheel_eyes->stop();
                wait(0.5);
                count_al++;
            }
            wheel_eyes->rotate_right();
            if(count_il<1){
            wait(0.2);
            count_il++;
            }
            if(s5>800||s4>800){
            wheel_eyes->zero_u1();
            bluetooth_left = 0;
            count_il=0;
            count_al=0;
            }    
        } 
        
        else if (bluetooth_right == 1){
            bluetooth_left = 0;
            if(count_ar<1){
                wheel_eyes->stop();
                wait(0.5);
                count_ar++;
            }
            wheel_eyes->rotate_left();
            if(count_ir<1){
            wait(0.8);
            count_ir++;
            }
            if(s1>800||s2>800){
            wheel_eyes->zero_u1();
            bluetooth_right = 0;
            count_ir=0;
            count_ar=0;
            }    
        } 
        
        
        else{
        if(s3<500 && s2<500 && s5<500 && s4<500 && s1<500) {
           wheel_eyes->stop();
        }
        else{
        calc_u();
        wheel_eyes->do_speed(u2);
        }
        }
     };   
        
    
};// end of class
/////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////


int main(){
    
DigitalOut enable(PB_15); 
enable = 1;
PID_speed* wheel_eyes = new PID_speed(0.5, 0.525, 0.001, 4, 0.00001, 0.00001, 0.02, 0.02  , 0);
PID_sensors line_eyes(100000, 0.1, 1, 0.0007, 0.5225, 0.52, wheel_eyes);

Serial hm10(PC_6, PA_12); //UART6 TX,RX
 
// all codes
//char code_led_on[] = "I'm blue   "; // 8 characters
//char code_led_off[] = "I'm offs   "; // 8 characters
//char code_turn_180[] = "turn around"; // 11 characters
char hm_10_recieve[] = "0"; // 11 characters
int bluetooth_truth_statement = 1;
int i = 0;
int command_length = 11;
hm10.baud(9600);
  
  while(1) {
    //PC.printf("a = %d \n", a);
    if(hm10.readable()){ // issue now is that hm10 readable is only working if the thing is reading multiple letters so i wont ever reach 8 so wont input correct value 
    // works fine however there cant be any operation other tjan getc
    //hm10.readable is a bool function that returns 1 if a character is being read 
      while(i<1){
        hm_10_recieve[i++] = hm10.getc();     
      }
         if(strcmp(hm_10_recieve, "l" )==0){
            line_eyes.set_bluetooth_left(bluetooth_truth_statement);
            memset(hm_10_recieve, '0', 1);
         }
         
         if(strcmp(hm_10_recieve, "r" )==0){
            line_eyes.set_bluetooth_right(bluetooth_truth_statement);
            memset(hm_10_recieve, '0', 1);
         }

        i=0;
    }
  }
}
