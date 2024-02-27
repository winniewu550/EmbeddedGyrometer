# Term Project: The Embedded Gyrometer

Group 14

SRIRAM NARAYAN KOUSHIK CHITRAPU 

Weining Wu

Daichong Meng

Devashish Gawde

## Preparation  

- Link the STM32 board to the computer, run PlatformIO

### Method 1: Create a new project
- Unzip the `zip` file, put the `cpp` file and the entire `drivers` folder in the `src` folder, and include the `mbed_app.json` file in the home library of this project

### Method 2: Load the whole project
- Unzip the `zip` file, use PlatformIO to load the whole project

## Building and Running

The code has the following status:
- `IDLE`, for standing
- `MOVING_DATARECORD`, for moving

Build the project, upload it to the board, and it will automatically run

## Board Fixing

Fix the board under the knee, run the program, and then start moving.

### LCD Screen Display

Once starting the program, there will be a welcome interface that lasts 2 seconds. Then the measurement starts automatically.

During the measurement, the screen will show the current value of distance.

If the program runs for entire 20 seconds, there will be a calculating interface to show that the measurement finishes normally, lasting 2 seconds as well.

Finally comes the result interface, displaying the final result of measurement, as wel as a guide to restart the program.

At any period of this program, if you press the black button on the STM32 board, the screen will turn white for less than 1 second, and the program will restart from the very beginning (the welcome interface). 

### Serial Monitor Display

If the serial monitor is open when the program runs, it will display "Input Gyro Data" and "Filtered Linear Velocity" for 3 axis every 0.5 seconds when the gyroscope is measuring.

There will be no other effective outputs.