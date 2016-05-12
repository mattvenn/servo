#include <Encoder.h>
#include <PID_v1.h>

#define ENCB 2               //hardware ints
#define ENCA 3               //hardware ints
#define REV 9                //timer 1
#define FOR 10               //timer 1
#define LED 13

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,2.47, 0.03, 0.02, DIRECT);


Encoder myEnc(ENCA, ENCB);

volatile bool calc = false;
bool dir = false;

//pid globals
unsigned int posref = 0; //servo setpoint in mm
long curpos = 0;
float b0 = 0;
float b1 = 0;
float b2 = 0;
double yn = 0;
double ynm1 = 0;
float xn = 0;
float xnm1 = 0;
float xnm2 = 0;
float kp = 0;
float ki = 0;
float kd = 0;

typedef struct
{
    unsigned int p;
    unsigned int i;
    unsigned int d;
} PIDs;

typedef struct
{
    unsigned int time;
    unsigned int curpos;
    unsigned int posref;
} Response;

void setup()
{
    Serial.begin(115200);

    setup_timers();

    pinMode(LED, OUTPUT);
    pinMode(FOR, OUTPUT);
    digitalWrite(FOR,LOW);
    pinMode(REV, OUTPUT);
    digitalWrite(REV,LOW);
/*
    while(true)
    {
        if(Serial.available() == sizeof(PIDs))
        {
            char buf[sizeof(PIDs)];
            PIDs pid;
            Serial.readBytes(buf, sizeof(PIDs));
            memcpy(&pid, &buf, sizeof(PIDs));
            kp = pid.p / 1000.0;
            ki = pid.i / 1000.0;
            kd = pid.d / 1000.0;
            myPID.SetTunings(kp, ki, kd);
            digitalWrite(LED, HIGH);
            break;
        }
    }
    */
    myPID.SetOutputLimits(-127,127);
    myPID.SetSampleTime(10);

    // turn on interrupts
    interrupts();

    //turn the PID on
    myPID.SetMode(AUTOMATIC);

}

void loop()
{
    if(Serial.available() == 2)
    {
        char buf[2];
        Serial.readBytes(buf, 2);
        memcpy(&posref, &buf, 2);
      Setpoint = posref;
    }
    Input = myEnc.read();

    if(calc)
    {
        calc = false;
        Response resp;
        resp.time = millis();
        resp.posref = Setpoint;
        resp.curpos = Input;
        char buf[sizeof(Response)];
        memcpy(&buf, &resp, sizeof(Response));
        for(int i = 0; i < sizeof(Response); i ++)
        Serial.write(buf[i]);
    }

  myPID.Compute();

  drive(Output);
}

void drive(int yn)
{
    //limit
    if(yn > 127)
        yn = 127;
    if(yn < -128)
        yn = -128;

    int pwm = 128 + int(yn);   
    analogWrite(FOR,255-pwm);
    analogWrite(REV,pwm);
}

volatile byte timer2_overflows = 0;
uint8_t timer2_overflow = 92;
uint8_t timer2_remainder = 0;

ISR(TIMER2_OVF_vect)        
{
        calc = true;
        TCNT2 = timer2_overflow;
}

void setup_timers()
{
    // timer 2 setup for calc
    TCCR2A = 0;
    TCCR1B = 0;

    // set timer prescalar to 1024
    TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);
    // turn on interrupt overflow
    TIMSK2 |= (1 << TOIE2);

    // preload timer2 (8 bits)
    TCNT2 = timer2_overflow; 

    // PWM output for motor driver
    // set pwm frequency on pins 9&10 (timer1) to 31250Hz
    TCCR1B = TCCR1B & 0b11111000 | 0x01;
}
