
#include <stdio.h>
#include "pico/stdlib.h"
#include <string>
#include <string.h>
#include <iostream>
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "stepper.pio.h"

using namespace std;
const uint LED_PIN = PICO_DEFAULT_LED_PIN;

char inbuff[100];
int buffcounter = 0;
bool dataavailable = false;
string inString = "";
string outstr = "";

#define STM_EN 15

float fullRevolutionAngle = 360.0;
int directionchangeDelay = 50; // ms

/****************************************************/

struct MECHPART
{
    int StepsToTake = 0;
    bool activedir = true;
    bool dirchange = true;
    float ActiveAngle = 0.0; // 360/res
    int DIR_PIN = 0;
    int STEP_PIN = 0;
    int HOME_PIN = 0;
    int FULLSTEP = 800;
    int directionchangeDelayCounter = 0;
    PIO stm_pio = pio0;
    int stm_sm;
};

struct MECHPART stepperMotor1;

void setupPIO();
void setupGPIO();
uint32_t construct32bits(int pulsecnt);
void moveMECHPART(MECHPART *mpart);
void enableStepper(bool act);
void getStepperSteps(MECHPART *mpart, float degree, int dir);
void readtobuffer();
void processinData();
float countAngle(float currentAng, float counterval, bool dir);
bool changeMotorDirection(MECHPART *mpart);
static inline void put_steps(uint32_t steps);

int main()
{

    stdio_init_all();
    setupGPIO();
    setupPIO();
    int changeDegreeCounter = 0;

    float degPos[7] = {45, 315, 225, 90, 180, 135, 0.5};
    int activedegindex = 0;
    while (true)
    {
        if ((time_us_64() / 1000) - changeDegreeCounter > 2000 && activedegindex < 7)
        {
            activedegindex += 1;
            changeDegreeCounter = time_us_64() / 1000;
        }
        getStepperSteps(&stepperMotor1, degPos[activedegindex], 0);
        moveMECHPART(&stepperMotor1);
        readtobuffer();
        processinData();
    }
}

bool changeMotorDirection(MECHPART *mpart)
{
    if (mpart->activedir != mpart->dirchange && mpart->directionchangeDelayCounter == 0) // first change detected
    {
        if (pio_sm_is_tx_fifo_empty(mpart->stm_pio, mpart->stm_sm))
        {
            mpart->directionchangeDelayCounter = (time_us_64() / 1000);
        }
        return false;
    }
    else if (mpart->activedir != mpart->dirchange && mpart->directionchangeDelayCounter != 0)
    {
        if (((time_us_64() / 1000) - mpart->directionchangeDelayCounter) > directionchangeDelay) // pre change delay
        {
            mpart->activedir = mpart->dirchange;
            gpio_put(mpart->DIR_PIN, !mpart->activedir);
            mpart->directionchangeDelayCounter = (time_us_64() / 1000);
        }
        return false;
    }
    else if (mpart->activedir == mpart->dirchange && mpart->directionchangeDelayCounter != 0)
    {
        if (((time_us_64() / 1000) - mpart->directionchangeDelayCounter) > directionchangeDelay) // post change delay
        {
            mpart->directionchangeDelayCounter = 0;
            return true;
        }
        return false;
    }

    return true;
}
void setupPIO()
{
    // stepper 1 pio
    uint stm_offset = pio_add_program(stepperMotor1.stm_pio, &stepper_1_program);
    stepper_1_program_init(stepperMotor1.stm_pio, stepperMotor1.stm_sm, stm_offset, stepperMotor1.STEP_PIN, 10000, true);
}
void setupGPIO()
{
    stepperMotor1.DIR_PIN = 17;
    stepperMotor1.stm_sm = 1;
    stepperMotor1.STEP_PIN = 16;
    stepperMotor1.FULLSTEP = 800;
    // stepper enable pin
    gpio_init(STM_EN);
    gpio_set_dir(STM_EN, GPIO_OUT);
    // gpio_put(STM_EN, 0);

    // wheel 1 stepper dir pin
    gpio_init(stepperMotor1.DIR_PIN);
    gpio_set_dir(stepperMotor1.DIR_PIN, GPIO_OUT);
    // gpio_put(stepperMotor1.DIR_PIN, 1);

    // inbuilt led
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
}
uint32_t construct32bits(int pulsecnt)
{
    uint32_t outbits = 0b0;
    for (size_t i = 0; i < pulsecnt; i++)
    {
        outbits = (outbits << 1) | 0b1;
    }
    return outbits;
}
void moveMECHPART(MECHPART *mpart)
{

    if (mpart->StepsToTake > 0 && pio_sm_is_tx_fifo_empty(mpart->stm_pio, mpart->stm_sm))
    {
        if (changeMotorDirection(mpart))
        {
            int remBits = 32;
            if (mpart->StepsToTake < 32)
            {
                remBits = mpart->StepsToTake;
            }

            gpio_put(LED_PIN, 1);
            uint32_t BitRes;
            BitRes = construct32bits(remBits);
            put_steps(BitRes);
            mpart->StepsToTake -= remBits;
        }
    }
    else
    {
        gpio_put(LED_PIN, 0);
    }
}
float countAngle(float currentAng, float counterval, bool dir)
{
    float anglecounter = currentAng;
    if (dir) // count up
    {
        if ((counterval + anglecounter) > 360)
        {
            anglecounter = (counterval + anglecounter) - 360;
        }
        else
        {
            anglecounter += counterval;
        }
    }
    else
    { // count down
        if ((anglecounter - counterval) < 0)
        {
            anglecounter = 360 + (anglecounter - counterval);
        }
        else
        {
            anglecounter -= counterval;
        }
    }
    return anglecounter;
}
void getStepperSteps(MECHPART *mpart, float degree, int dir) //(degree position, steppernumber, direction to move (0-fastest route,1-forward,2-backward))
{
    if (degree > 0 && abs(degree - mpart->ActiveAngle) != 360 && degree <= 360)
    { // no negative angle
        float mStepRes = fullRevolutionAngle / float(mpart->FULLSTEP);
        float precRange = 0.55;
        bool mostRecentDir = mpart->dirchange;
        int stepsToTake = 0;
        int i = 0;
        int j = 0;
        bool ibool = true;
        bool jbool = false;
        // while (mpart->StepsToTake != 0)
        // {
        //     moveMECHPART(mpart);
        // }

        if (mpart->StepsToTake != 0) // take care of unreached position
        {
            //printf("remsteps %d activeangle %f dir %d", mpart->StepsToTake, mpart->ActiveAngle, mpart->activedir);
            for (size_t k = 0; k < mpart->StepsToTake; k++)
            {
                if (mpart->dirchange == ibool)
                {
                    mpart->ActiveAngle = countAngle(mpart->ActiveAngle, mStepRes, false);
                }
                else if (mpart->dirchange == jbool)
                {
                    mpart->ActiveAngle = countAngle(mpart->ActiveAngle, mStepRes, true);
                }
            }
            mpart->StepsToTake = 0;
            //printf(" //adjangle %f \n", mpart->ActiveAngle);
        }
        float i_tempdeg = mpart->ActiveAngle;
        float j_tempdeg = mpart->ActiveAngle;

        if (dir == 0 || dir == 1)
        {
            // degDelta = abs(i_tempdeg - degree);
            while ((abs(i_tempdeg - degree) / mStepRes) > precRange)
            {
                i_tempdeg = countAngle(i_tempdeg, mStepRes, true);
                i += 1;
                // degDelta = abs(i_tempdeg - degree);
            }
        }
        if (dir == 0 || dir == 2)
        {
            // degDelta = abs(j_tempdeg - degree);
            while ((abs(j_tempdeg - degree) / mStepRes) > precRange)
            {
                j_tempdeg = countAngle(j_tempdeg, mStepRes, false);
                j += 1;
                // degDelta = abs(j_tempdeg - degree);
            }
        }

        if (i != 0 && j != 0)
        {
            if (i <= j)
            { // move forward to angle
                stepsToTake = i;
                mpart->ActiveAngle = i_tempdeg;
                mpart->StepsToTake += stepsToTake;
                mpart->dirchange = ibool;
            }
            else
            { // move backward to angle
                stepsToTake = j;
                mpart->ActiveAngle = j_tempdeg;
                mpart->StepsToTake += stepsToTake;
                mpart->dirchange = jbool;
            }
        }
        else if (i > 0)
        { // move forward to angle
            stepsToTake = i;
            mpart->ActiveAngle = i_tempdeg;
            mpart->StepsToTake += stepsToTake;
            mpart->dirchange = ibool;
        }
        else if (j > 0)
        { // move backward to angle
            stepsToTake = j;
            mpart->ActiveAngle = j_tempdeg;
            mpart->StepsToTake += stepsToTake;
            mpart->dirchange = jbool;
        }
        enableStepper(true);
        //printf(" calcDir %d \n", mpart->dirchange);
    }
}

void enableStepper(bool act)
{
    if (act)
    {
        gpio_put(STM_EN, 0);
    }
    else
    {
        gpio_put(STM_EN, 1);
    }
}

void connecttoPC(string str)
{
    inString = str;
    outstr = inString;
    // printf(outstr.c_str());
    if (inString != "")
    {

        if (inString.find("moveto=") == 0)
        {
            inString.replace(inString.find("moveto="), 7, "");
            getStepperSteps(&stepperMotor1, stof(inString), 0);
            printf("activeangle %f stepstotake %d dir %d \n", stepperMotor1.ActiveAngle, stepperMotor1.StepsToTake, stepperMotor1.activedir);
            inString = "";
        }
        else
        {
            inString = "";
        }
    }
}

void readtobuffer()
{
    char chr = getchar_timeout_us(0);
    if (chr != 255)
    {
        if (chr == '*')
        {
            dataavailable = true;
        }
        // cout << chr << buffcounter << std::endl;
        if (buffcounter < sizeof(inbuff))
        {
            inbuff[buffcounter] = chr;
            buffcounter += 1;
        }
        else
        {
            buffcounter = 0;
        }
    }
    else
    {
        dataavailable = false;
    }
}

void processinData()
{
    if (dataavailable)
    {
        // cout << inbuff << std::endl;
        int tempcnt = 0;
        while (inbuff[tempcnt] != '*')
        {
            inString += inbuff[tempcnt];
            tempcnt += 1;
        }
        // cout << inString << std::endl;
        connecttoPC(inString);
        buffcounter = 0;
        dataavailable = false;
        memset(inbuff, 0, sizeof(inbuff));
    }
}

static inline void put_steps(uint32_t steps)
{
    pio_sm_put_blocking(pio0, 1, steps);
}