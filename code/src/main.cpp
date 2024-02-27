//===================================================================================================================================================
// REQUIRED HEADER FILES:
//===================================================================================================================================================
#include <mbed.h>                                           //INCLUDING THE MBED HAL FRAMEWORK INTO THE CODE
#include <stdio.h>                                          //INCLUDING THE STDIO.H HEADER FILE
#include <math.h>                                           //INCLUDING THE MATH.H HEADER FILE
#include <chrono>                                           //INCLUDING THE CHRONO HEADER FILES
#include "stm32f4xx_hal.h"                                  //Header File for the STM32F4 Header File (Additional Features of HAL).
#include "drivers/LCD_DISCO_F429ZI.h"                      //Header File for the LCD Display.
#include <stdlib.h>
#include <float.h>
#define _USE_MATH_DEFINES 

//===================================================================================================================================================
// GYROSCOPE CONFIGURATION ESSENTIALS:
//===================================================================================================================================================
#define CTRL_REG1 0x20
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1
#define CTRL_REG4 0x23 // Sqond configure to set the DPS // page 33
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0
#define CTRL_REG3 0x22 // page 32
#define CTRL_REG3_CONFIG 0b0'0'0'0'1'000
#define OUT_X_L 0x28
#define deg2AngScalingFactor (1.0f* 0.017453292519943295769236907684886f / 1000.0f)
#define Radius 0.5f     //In Meters
#define WINDOW_SIZE 6 // WINDOW SIZE CAN BE CHANGED FOR THE FILTER
#define DUR_SAMPLE_COUNT 40
#define DIM_COUNT 3
#define RESET_TIMERLIMIT 20
#define TICKER_LIMIT 500ms
#define GRYOMULFACTOR1 1000000   //To Multiply 1000000 with the current read Gyro Reading
#define GRYOMULFACTOR2 100
#define GYRO_THRESHOLD 104.85f

//#define FILTER_COEFFICIENT 0.85f // Adjust this value as needed if LPF is used.
int i_cnt=0;
float linearVelocity[DUR_SAMPLE_COUNT][DIM_COUNT];

//FILE *file = fopen("output.csv", "w");

//GYROSCOPE STATES:
#define IDLE 0
#define MOVING_DATARECORD 1

//Declare Semaphore:
//Semaphore sem(1); //1 for available to share      //wait -> acquire,  signal -> release
//float linearLength[DUR_SAMPLE_COUNT];

//Timer for 0.5seconds:
Ticker t;

//Timer for 20 seconds:
Timer resetTimer;

volatile int8_t state_chk = IDLE;
volatile float totalDist = 0.0f;
int8_t step_cnt=0;
float window_gx[WINDOW_SIZE] = {0};
float window_gy[WINDOW_SIZE] = {0};
float window_gz[WINDOW_SIZE] = {0};
int window_index = 0;
float filtered_gx = 0.0f, filtered_gy = 0.0f, filtered_gz = 0.0f;
volatile float gX_ref=0.0f, gY_ref=0.0f, gZ_ref=0.0f;

//_______________________
// SPI CONFIGURATION ESSENTIALS:
//_______________________
EventFlags flags;
#define MOSI_PIN PF_9
#define MISO_PIN PF_8
#define SCLK_PIN PF_7
#define CS_PIN  PC_1
#define SPI_FLAG 1
#define DATA_READY_FLAG 2
#define WRITELIMIT_SIZE 2
#define READLIMIT_SIZE 2
#define WRITELIMIT_SIZE1 7
#define READLIMIT_SIZE1 7

uint8_t write_buf[32];
uint8_t read_buf[32];
volatile int flag = 0;    // mosi, miso, sclk
float result[3];

//spi initialization:
SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);

// spi callback function
void spi_cb(int event) 
{
    //sem.acquire();
    flags.set(SPI_FLAG);
    //sem.release();
}

//CALLBACK FOR THE TICKER:
void cb() 
{
    spi_cb(0); // Call the spi_cb with a default event value
}

// DATA READY CALLBACK FUNCTION:
void data_cb() 
{
    //sem.acquire();
    flags.set(DATA_READY_FLAG);
    //sem.release();
}

//___
//LCD ESSENTIALS:
//___
#define BACKGROUND 1
#define FOREGROUND 0

//LCD OBJECT:
LCD_DISCO_F429ZI lcd;

// Fucntion Prototype:
void Initial_ScreenDisp();
void CALC_ScreenDisp(float totalDist);

//sets the background layer to be visible, transparent, and resets its colors to all black
void setup_background_layer(){
  lcd.SelectLayer(BACKGROUND);
  lcd.Clear(LCD_COLOR_BLACK);
  lcd.SetBackColor(LCD_COLOR_BLACK);
  lcd.SetTextColor(LCD_COLOR_GREEN);
  lcd.SetLayerVisible(BACKGROUND,ENABLE);
  lcd.SetTransparency(BACKGROUND,0x7Fu);
}
//resets the foreground layer to all black
void setup_foreground_layer(){
    lcd.SelectLayer(FOREGROUND);
    lcd.Clear(LCD_COLOR_BLACK);
    lcd.SetBackColor(LCD_COLOR_BLACK);
    lcd.SetTextColor(LCD_COLOR_LIGHTGREEN);
}

// UI funcs
void Initial_ScreenDisp()
{
  lcd.Clear(LCD_COLOR_BLACK);
  lcd.DisplayStringAt(0, LINE(4), (uint8_t *)"ECE 6483", CENTER_MODE);
  lcd.DisplayStringAt(0, LINE(6), (uint8_t *)"Final Challenge", CENTER_MODE);
  lcd.DisplayStringAt(0, LINE(7), (uint8_t *)"Embedded Gyrometer", CENTER_MODE);
  lcd.DisplayStringAt(0, LINE(8), (uint8_t *)"Need for Speed", CENTER_MODE);
  lcd.DisplayStringAt(0, LINE(10), (uint8_t *)"Group: 14", CENTER_MODE);
  lcd.DisplayStringAt(0, LINE(15), (uint8_t *)"Auto Starting...", CENTER_MODE);
  HAL_Delay(1500);

}

void CALC_ScreenDisp(float totalDist, int8_t stepcnt)      //Function for Displaying current Distance on LCD
{
//HAL_Delay(20);
float distance = totalDist;
lcd.Clear(LCD_COLOR_BLACK);

// Declare a character array to hold the formatted distance string
char distance_buf[50]; 

//Declare a character array for Step Count:
char stepcnt_buf[50];

// Assuming totalDist is the variable holding the calculated distance
lcd.DisplayStringAt(LINE(0),LINE(10), (uint8_t *)"Computing for 20 sec...", CENTER_MODE);
snprintf(distance_buf, 50, "Current Calc: %.3f m", distance); // Format the string with the calculated distance
snprintf(stepcnt_buf, 50, "Current Step Cnt: %d", stepcnt);

// Display the formatted distance string on the LCD screen
lcd.DisplayStringAt(LINE(0),LINE(10), (uint8_t *)distance_buf, CENTER_MODE);
lcd.DisplayStringAt(LINE(0),LINE(11), (uint8_t *)stepcnt_buf, CENTER_MODE);

}

void CALC_Final_ScreenDisp(float totalDist, int8_t stepcnt)      //Function for Displaying current Distance on LCD
{
float distance = totalDist;

// Declare a character array to hold the formatted distance string
char distance_buf[50]; 

// Declare a character array to hold the formatted distance string
char stepcnt_buf[50]; 

// Assuming totalDist is the variable holding the calculated distance
snprintf(distance_buf, 50, "Distance: %.2f m", distance);
snprintf(stepcnt_buf, 50, "Step Count: %d", stepcnt);

lcd.Clear(LCD_COLOR_BLACK);
// Display the formatted distance string on the LCD screen
lcd.DisplayStringAt(LINE(0),LINE(8), (uint8_t *)distance_buf, CENTER_MODE);
lcd.DisplayStringAt(LINE(0),LINE(9), (uint8_t *)stepcnt_buf, CENTER_MODE);
lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"[Result in 20 sec]", CENTER_MODE);

//Note Section:
lcd.DisplayStringAt(0, LINE(13), (uint8_t *)"Press Black Button", CENTER_MODE);
lcd.DisplayStringAt(0, LINE(14), (uint8_t *)"To Restart", CENTER_MODE);
}

//======================================================================
//Function Calculates the linear velocity and lengths of all 3 coordinates
//======================================================================
float* getGyroData(int32_t xDimOut_L)           
{
    //Unfiltered data:
    int16_t raw_gx, raw_gy, raw_gz;
    float gx, gy, gz;

    flags.wait_all(DATA_READY_FLAG);
    write_buf[0] = xDimOut_L | 0x80 | 0x40;

    spi.transfer(write_buf, WRITELIMIT_SIZE1, read_buf, READLIMIT_SIZE1, spi_cb,SPI_EVENT_COMPLETE);
    flags.wait_all(SPI_FLAG);

    // Process raw data
    raw_gx = (((uint16_t)read_buf[2]) << 8) | ((uint16_t)read_buf[1]);
    raw_gy = (((uint16_t)read_buf[4]) << 8) | ((uint16_t)read_buf[3]);
    raw_gz = (((uint16_t)read_buf[6]) << 8) | ((uint16_t)read_buf[5]);

    //Storing the Filtered Linear Velocity Values in the array:
    linearVelocity[i_cnt][0]=raw_gx;
    linearVelocity[i_cnt][1]=raw_gy;
    linearVelocity[i_cnt][2]=raw_gy;
    float avg_LinVel= linearVelocity[i_cnt][0]+linearVelocity[i_cnt][1]+linearVelocity[i_cnt][2]/DIM_COUNT;
    printf("\nFiltered Linear Velocity:-> \tgx_LinVel: %f \t gy_LinVel: %f \t gz_LinVel: %f\t Avg_LinVel:%f\n",linearVelocity[i_cnt][0],linearVelocity[i_cnt][1],linearVelocity[i_cnt][2], avg_LinVel );
    i_cnt++;

    if(i_cnt==DUR_SAMPLE_COUNT)
    {
        i_cnt=0;
    }

    //Conversion to Angular Velocity from degrees per second:
    gx = ((float)raw_gx) * deg2AngScalingFactor;
    gy = ((float)raw_gy) * deg2AngScalingFactor;
    gz = ((float)raw_gz) * deg2AngScalingFactor;

    window_gx[window_index]  = gx;
    window_gy[window_index]  = gy;
    window_gz[window_index]  = gz;

    //If LPF is wanted to be used:
    // filtered_gx = FILTER_COEFFICIENT * gx + (1 - FILTER_COEFFICIENT) * filtered_gx;
    // filtered_gy = FILTER_COEFFICIENT * gy + (1 - FILTER_COEFFICIENT) * filtered_gy;
    // filtered_gz = FILTER_COEFFICIENT * gz + (1 - FILTER_COEFFICIENT) * filtered_gz;

    //Compute the moving average using a moving average filter:   
    float avg_gx = 0.0f, avg_gy = 0.0f, avg_gz = 0.0f;
    for (int i = 0; i < WINDOW_SIZE; i++) 
    {
        avg_gx += window_gx[i];
        avg_gy += window_gy[i];
        avg_gz += window_gz[i];
    }
    avg_gx /= WINDOW_SIZE;
    avg_gy /= WINDOW_SIZE;
    avg_gz /= WINDOW_SIZE;

    //Outputting the contents to thev file:
    // if (file == NULL) {
    //     fprintf(stderr, "Error opening file for writing.\n");
        
    // }
    // fprintf(file, "%f, %f, %f", avg_gx, avg_gy, avg_gz);
    // fflush(file);
    
	
	//Length Calculation:  
    // Time = 0.5s  // Distance = Angular Velocity * Radius * Time;
    float gx_length = (avg_gx)* (Radius) * 0.5; 
    float gy_length = (avg_gy)* (Radius) * 0.5;
    float gz_length = (avg_gz)* (Radius) * 0.5;

    //printf("\nFiltered LENGTH:-> \tgx: %f \t gy: %f \t gz: %f\n", gx_length, gy_length, gz_length);
    thread_sleep_for(500);

    result[0]=gx_length;
    result[1]=gy_length;
    result[2]=gz_length;

    //Resetting the window index for the Moving Average Filter:
    window_index = (window_index + 1) % WINDOW_SIZE;

    return result;
}

float calculateDist3Dim(float gyroDimDegRes[])    // Function to calculate resultant Distance of all 3 co ordinates
{
    //Distance Calculation:
    float xSqr=(gyroDimDegRes[0]-gX_ref)*(gyroDimDegRes[0]-gX_ref);
    float ySqr=(gyroDimDegRes[1]-gY_ref)*(gyroDimDegRes[1]-gY_ref);
    float zSqr=(gyroDimDegRes[2]-gZ_ref)*(gyroDimDegRes[2]-gZ_ref);
    float distcalc=sqrt(xSqr+ ySqr + zSqr);

    printf("\nCurrent Input Calc: \t%f\n", distcalc);

    //Updating the Reference Point to the newly moved point:
    gX_ref=gyroDimDegRes[0];
    gY_ref=gyroDimDegRes[1];
    gZ_ref=gyroDimDegRes[2];

    //Increementing the Step Count:
    if(distcalc>=0.00280)
    {
        ++step_cnt;
    }
    // else 
    // {
    //     step_cnt=step_cnt;
    // }

    //Returning the Distance Value:
  return distcalc;
}

//======================================================================
int main()
 {   
    //Setupping the Background LCD layer to Black:
    setup_background_layer();
    
    //Setupping the foreground LCD layer to Green:
    setup_foreground_layer();

    //interrupt initialization:
    InterruptIn int2(PA_2, PullDown);
    int2.rise(&data_cb);

    //Sampling the data every 0.5seconds using the Ticker Timer Interrupts:
    t.attach(&cb, TICKER_LIMIT);
    
    //spi format and frequency:
    spi.format(8, 3);
    spi.frequency(1'000'000);

    // Write to control registers => spi transfer
    write_buf[0] = CTRL_REG1;
    write_buf[1] = CTRL_REG1_CONFIG;
    spi.transfer(write_buf, WRITELIMIT_SIZE, read_buf, READLIMIT_SIZE, spi_cb,SPI_EVENT_COMPLETE);
    flags.wait_all(SPI_FLAG);

    write_buf[0] = CTRL_REG4;
    write_buf[1] = CTRL_REG4_CONFIG;
    spi.transfer(write_buf, WRITELIMIT_SIZE, read_buf, READLIMIT_SIZE, spi_cb,SPI_EVENT_COMPLETE);
    flags.wait_all(SPI_FLAG);

    write_buf[0] = CTRL_REG3;
    write_buf[1] = CTRL_REG3_CONFIG;
    spi.transfer(write_buf, WRITELIMIT_SIZE, read_buf, READLIMIT_SIZE, spi_cb,SPI_EVENT_COMPLETE);
    flags.wait_all(SPI_FLAG);

    write_buf[1] = 0xFF;

    //Polling data ready flag:
    if (!(flags.get() & DATA_READY_FLAG) && (int2.read() == 1)) 
    {
        flags.set(DATA_READY_FLAG);
    }

    //Welcome Message on LCD:
    Initial_ScreenDisp();

    //Setting the Timer to start:
	resetTimer.start();

    //while(1){} => for infinite duration if its to be implemented.

    //While to execute for 20 second duration only:
    while (chrono::duration_cast<chrono::seconds>(resetTimer.elapsed_time()).count() <= RESET_TIMERLIMIT) 
    {
        float *gyroCurrDimData=getGyroData(OUT_X_L);
        printf("\nInput Gyro Data: %f\t%f\t%f", gyroCurrDimData[0], gyroCurrDimData[1], gyroCurrDimData[2] );
        //thread_sleep_for(500);
        
		//sem.acquire();
		
		//Using the default Reset Button on the board, Hence not configuring a separate code section for the button.
        switch(state_chk)
        {
            case IDLE:
            {
                //sem.acquire();
                totalDist=totalDist;
                step_cnt=step_cnt;

                if((abs(gyroCurrDimData[0]*GRYOMULFACTOR1)>=GYRO_THRESHOLD) || (abs(gyroCurrDimData[1]*GRYOMULFACTOR1)>=GYRO_THRESHOLD) || (abs(gyroCurrDimData[2]*GRYOMULFACTOR1)>=GYRO_THRESHOLD))
                {
                    state_chk=MOVING_DATARECORD;

                }
                else
                {
                    state_chk=IDLE;
                    gyroCurrDimData[0]=0.0f;
                    gyroCurrDimData[1]=0.0f;
                    gyroCurrDimData[2]=0.0f;
                }

                //sem.release();
            }
            break;

            case MOVING_DATARECORD:
            {

                //sem.acquire();
                totalDist=totalDist+ (calculateDist3Dim(gyroCurrDimData)*GRYOMULFACTOR2);        // Calculating the total distance
                printf("\nTotal Distance Travelled So Far:%f\t", totalDist);
                printf("\nTotal Step Counts So Far:\t %d",step_cnt);
                //thread_sleep_for(5000);
                state_chk=IDLE;

                //sem.release();

				// if(chrono::duration_cast<chrono::seconds>(resetTimer.elapsed_time()).count() == 20)
				// {
                //     break;
				// }
                
            }
            break;
        }
		//sem.release();

		//Printing the Total Distance Travelled on the LCD Screen:
	    CALC_ScreenDisp(totalDist, step_cnt); // If needs to be shown at every instant.
    }

    CALC_Final_ScreenDisp(totalDist, step_cnt);

	//Reset Timer after 20 seconds:
    resetTimer.stop();

    //Reverting back to idle state
    state_chk=IDLE;

	//file Close:
    //fclose(file);
 }