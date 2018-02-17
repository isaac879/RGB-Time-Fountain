/*--------------------------------------------------------------------------------------------------------------------------------------------------------
 * 
 * CHECK THE CODE FOR "TODO:" AND EDIT APPROPRIATELY 
 * 
 * The code is developed for a RGB Time Fountain. The RGB Time Fountain is controlled by an Arduino Uno, HC-05 Bluetooth module and custom circuits.
 * 
 * CAD files available at: Not currently publicly available.
 * 
 * All measurements are in SI units unless otherwise specified
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS 
 * IN THE SOFTWARE
 * 
 * Code written by isaac879
 * 
 * Last modified 17/02/2018
 *--------------------------------------------------------------------------------------------------------------------------------------------------------*/

#include <Iibrary.h>//A library I created for Arduino that contains some simple functions I commonly use. Library available at: https://github.com/isaac879/Iibrary

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

#define PUMP_PIN PD4
#define RED_PIN PD6
#define GREEN_PIN PD7
#define BLUE_PIN PD5

#define MAX_STRING_LENGTH 10

#define PUMP_MAX_COUNT 255

#define POSITIVE 1
#define NEGATIVE -1

#define ON true
#define OFF false
/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/
//Global variables
unsigned int base_loop_count = 1010;

unsigned int phase_offset = 10;

float red_duty = 8.0;
float green_duty = 8.0;
float blue_duty = 8.0;
float base_duty = 8.0;

unsigned int red_duty_value = (float)base_loop_count * red_duty / 100.0;//switching value to achieve the desired duty cycle
unsigned int green_duty_value = (float)base_loop_count * green_duty / 100.0;//switching value to achieve the desired duty cycle
unsigned int blue_duty_value = (float)base_loop_count * blue_duty / 100.0;//switching value to achieve the desired duty cycle

unsigned int  pump_duty_value = 0;

unsigned int red_max_count = base_loop_count;
unsigned int green_max_count = base_loop_count;
unsigned int blue_max_count = base_loop_count;

unsigned int base_count = 0;
unsigned int red_count = 0;
unsigned int green_count = 0;
unsigned int blue_count = 0;
unsigned int pump_count = 0;

int red_sin_direction = POSITIVE;
int green_sin_direction = POSITIVE;
int blue_sin_direction = POSITIVE;

String stringText = "";

short sin_amplitude = 180;

bool flag_pump_off = true;
bool flag_red_sin = false;
bool flag_green_sin = false;
bool flag_blue_sin = false;

short mode_status = 0;//just used for a status reprot
float base_loop_count_temp = base_loop_count * 0.00277777777777777777777;

float sin_count = 0;
float sin_count_increment = 0.0174533;

bool flag_ms_start = true;
bool flag_ms_stop = false;
unsigned long ms;

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void serialFlush(){
    while(Serial.available() > 0) {
        char c = Serial.read();
    }
} 

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void setBaseStrobeFrequency(unsigned int targetFreq){
    if(targetFreq < 0 || targetFreq > 65535){
        printi("ERROR: loop count out of range (0-65,535)... value entered: ", (long)targetFreq);
    }
    else{
        flag_ms_start = true;
        int diff = targetFreq - base_loop_count;
        base_loop_count = targetFreq;
        base_loop_count_temp = base_loop_count * 0.00277777777777777777777;
        
        red_max_count += diff;
        red_duty_value = (float)red_max_count * red_duty / 100.0;
        green_max_count += diff;
        green_duty_value = (float)green_max_count * green_duty / 100.0;
        blue_max_count += diff;
        blue_duty_value = (float)blue_max_count * blue_duty / 100.0;
        printi("Loop count set to: ", base_loop_count);
    }
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void setPumpPWM(int PWM){
    if(PWM > PUMP_MAX_COUNT){
        printi("ERROR: PWM out of range (0-255)... PWM value entered = ", PWM);
    }
    else if(PWM == 0){
       flag_pump_off = true; 
       float percentage = float(PWM) / 2.55;
       printi("Pump duty cycle set to: ", percentage, 1, "%\n");
    }
    else{
        flag_pump_off = false;
        pump_duty_value = PWM;
        float percentage = float(PWM) / 2.55;
        printi("Pump duty cycle set to: ", percentage, 1, "%\n");
    }  
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void setPhase(int offset, String colour){
    red_count = 0;
    green_count = 0;
    blue_count = 0;
    
    if(colour == "R"){
        red_max_count += offset;
        printi("red_max_count set to: ", (int)red_max_count);
    }
    else if(colour == "G"){
        green_max_count += offset;
        printi("green_max_count set to: ", (int)green_max_count);
    }
    else if(colour == "B"){
        blue_max_count += offset;
        printi("blue_max_count set to: ", (int)blue_max_count);
    }
    else if(colour == "RG"){
        red_max_count += offset;
        green_max_count += offset;
        printi("red_max_count set to: ", (int)red_max_count);
        printi("green_max_count set to: ", (int)green_max_count);
    }
    else if(colour == "RB"){
        red_max_count +=  offset;
        blue_max_count += offset;
        printi("red_max_count set to: ", (int)red_max_count);
        printi("blue_max_count set to: ", (int)blue_max_count);
    }
    else if(colour == "GB"){
        green_max_count += offset;
        blue_max_count += offset;
        printi("green_max_count set to: ", (int)green_max_count);
        printi("blue_max_count set to: ", (int)blue_max_count);
    }
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void setSingleDuty(int duty, char led){
    if(duty < 0 || duty > 100){
        printi("ERROR: Duty must be between 0-100%... Duty entered: ", duty);
    }    
    else if(led == 'r' || led == 'R' ){
        red_duty = duty;
        red_duty_value = (float)base_loop_count * red_duty / 100.0;//switching value to achieve the desired duty cycle
        printi("Red duty set to: ", duty);
    }
    else if(led == 'g' || led == 'G' ){
        green_duty = duty;
        green_duty_value = (float)base_loop_count * green_duty / 100.0;//switching value to achieve the desired duty cycle
        //printi("Green duty set to: ", duty);
    }
    else if(led == 'b' || led == 'B' ){
        blue_duty = duty;
        blue_duty_value = (float)base_loop_count * blue_duty / 100.0;//switching value to achieve the desired duty cycle
        printi("Blue duty set to: ", duty);
    }
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void setLedDuty(int duty){
    if(duty < 0 || duty > 100){
        printi("ERROR: Duty must be between 0-100%... Duty entered: ", duty);
    }
    else{
        base_duty = duty;
        setSingleDuty(duty, 'R');
        setSingleDuty(duty, 'G');
        setSingleDuty(duty, 'B');
    }
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void coloursOn(bool redOn, bool greenOn, bool blueOn){
    if(redOn){
        setSingleDuty(base_duty, 'R');
    }
    else{
        setSingleDuty(0, 'R');
    }
    
    if(greenOn){
        setSingleDuty(base_duty, 'G');
    }
    else{
        setSingleDuty(0, 'G');
    }
    
    if(blueOn){
        setSingleDuty(base_duty, 'B');
    }
    else{
        setSingleDuty(0, 'B');
    }
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void sinDirections(int redDirection, int greenDirection, int blueDirection){
    red_sin_direction = redDirection;
    green_sin_direction = greenDirection;
    blue_sin_direction = blueDirection;
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

ISR(TIMER1_COMPA_vect){//timer1 interrupt
    setPosition(random(360), 'R');
    setPosition(random(360), 'G');
    setPosition(random(360), 'B');
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void setPosition(float phase, char colour){
    if(colour == 'r' || colour == 'R'){
        red_count = base_loop_count_temp * phase;
    }
    else if(colour == 'g' || colour == 'G'){
        green_count = base_loop_count_temp * phase;
    }
    else if(colour == 'b' || colour == 'B'){
        blue_count = base_loop_count_temp * phase;
    }
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void setSinCountIncrement(float deg){
    if(deg < 0 || deg > 360){
        printi("ERROR: Increment must be between 0-360... Increment entered: ", deg);
    }
    else{
        sin_count_increment = degToRads(deg);
        printi("Increment set to: ", deg);
    }
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void setMode(int mode){
    mode_status = mode;
    red_count = 0;
    green_count = 0;
    blue_count = 0;
    base_count = 0;
    sin_count = 0;
    flag_red_sin = false;
    flag_green_sin = false;
    flag_blue_sin = false;
    red_max_count = base_loop_count;
    green_max_count = base_loop_count;
    blue_max_count = base_loop_count;
    sinDirections(POSITIVE, POSITIVE, POSITIVE);
    sin_amplitude = 45;
    setLedDuty(base_duty);
    TIMSK1 &= ~(1 << OCIE1A);// disable timer compare interrupt
    
    switch(mode){
        case 1: //Red
            coloursOn(ON, OFF, OFF);
            break;
        case 2://green
            coloursOn(OFF, ON, OFF);
            break;
        case 3://blue
            coloursOn(OFF, OFF, ON);
            break;
        case 4://yellow
            coloursOn(ON, ON, OFF);
            break;
        case 5://purple
            coloursOn(ON, OFF, ON);
            break;
        case 6://cyan
            coloursOn(OFF, ON, ON);
            break;
        case 7://white
            coloursOn(ON, ON, ON);
            break;
        case 8://red green
            red_count = base_loop_count / 2;
            coloursOn(ON, ON, OFF);
            break;
        case 9://red blue
            red_count = base_loop_count / 2;
            coloursOn(ON, OFF, ON);
            break;
        case 10://red cyan
            red_count = base_loop_count / 2;
            coloursOn(ON, ON, ON);
            break;
        case 11://green blue
            green_count = base_loop_count / 2;
            coloursOn(OFF, ON, ON);
            break;
        case 12://green purple
            green_count = base_loop_count / 2;
            coloursOn(ON, ON, ON);
            break;
        case 13://blue yellow
            blue_count = base_loop_count / 2;
            coloursOn(ON, ON, ON);
            break;
        case 14://red green up-down
            setPhase(phase_offset, "R");
            setPhase(-phase_offset, "G");
            coloursOn(ON, ON, OFF);
            break;
        case 15://red blue up-down
            setPhase(phase_offset, "R");
            setPhase(-phase_offset, "B");
            coloursOn(ON, OFF, ON);
            break;
        case 16://red cyan up-down
            setPhase(phase_offset, "R");
            setPhase(-phase_offset, "GB");
            coloursOn(ON, ON, ON);
            break;
        case 17://green blue up-down
            setPhase(phase_offset, "B");
            setPhase(-phase_offset, "G");
            coloursOn(OFF, ON, ON);
            break;
        case 18://green purble up-down
            setPhase(phase_offset, "G");
            setPhase(-phase_offset, "RB");
            coloursOn(ON, ON, ON);
            break;
        case 19://blue yellow up-down
            setPhase(phase_offset, "B");
            setPhase(-phase_offset, "RG");
            coloursOn(ON, ON, ON);
            break;  
        case 20://red green blue
            red_count = base_loop_count;
            green_count = base_loop_count / 3;
            blue_count = base_loop_count * 2 / 3;
            coloursOn(ON, ON, ON);
            break;
        case 21://red green blue up still down
            setPhase(phase_offset, "R");
            setPhase(-phase_offset, "B");
            coloursOn(ON, ON, ON);
            break;
        case 22://red green blue up up down
            setPhase(phase_offset, "R");
            setPhase(phase_offset, "G");
            setPhase(-phase_offset, "B");
            red_count = base_loop_count;
            green_count = base_loop_count / 2;
            coloursOn(ON, ON, ON);
            break;  
        case 23://red green blue up down down
            setPhase(phase_offset, "R");
            setPhase(-phase_offset, "G");
            setPhase(-phase_offset, "B");
            green_count = base_loop_count;
            blue_count = base_loop_count / 2;
            coloursOn(ON, ON, ON);
            break;
        case 24://red sin
            flag_red_sin = true;
            coloursOn(ON, OFF, OFF);
            break;
        case 25://green sin
            flag_green_sin = true;
            coloursOn(OFF, ON, OFF);
            break;
        case 26://blue sin
            flag_blue_sin = true;
            coloursOn(OFF, OFF, ON);
            break;
        case 27://yellow
            flag_red_sin = true;
            flag_green_sin = true;
            coloursOn(ON, ON, OFF);
            break;
        case 28://purple sin
            flag_red_sin = true;
            flag_blue_sin = true;
            coloursOn(ON, OFF, ON);
            break;
        case 29://cyan sin
            flag_green_sin = true;
            flag_blue_sin = true;
            coloursOn(OFF, ON, ON);
            break;
        case 30://white sin
            flag_red_sin = true;
            flag_green_sin = true;
            flag_blue_sin = true;
            coloursOn(ON, ON, ON);
            break;
        case 31://red green merging
            flag_red_sin = true;
            flag_green_sin = true;
            coloursOn(ON, ON, OFF);
            sinDirections(POSITIVE, NEGATIVE, POSITIVE);
            break;
        case 32://red blue merging
            flag_red_sin = true;
            flag_blue_sin = true;
            coloursOn(ON, OFF, ON);
            sinDirections(POSITIVE, POSITIVE, NEGATIVE);
            break;
        case 33://green blue merging
            flag_green_sin = true;
            flag_blue_sin = true;
            coloursOn(OFF, ON, ON);
            sinDirections(POSITIVE, POSITIVE, NEGATIVE);
            break;
        case 34://red cyan merging
            flag_red_sin = true;
            flag_green_sin = true;
            flag_blue_sin = true;
            coloursOn(ON, ON, ON);
            sinDirections(NEGATIVE, POSITIVE, POSITIVE);
            break;
        case 35://green purple merging
            flag_red_sin = true;
            flag_green_sin = true;
            flag_blue_sin = true;
            coloursOn(ON, ON, ON);
            sinDirections(POSITIVE, NEGATIVE, POSITIVE);
            break;
        case 36://blue yellow merging
            flag_red_sin = true;
            flag_green_sin = true;
            flag_blue_sin = true;
            coloursOn(ON, ON, ON);
            sinDirections(POSITIVE, POSITIVE, NEGATIVE);
            break;
        case 37://red green blue purple merging 
            flag_red_sin = true;
            //flag_green_sin = true;
            flag_blue_sin = true;
            coloursOn(ON, ON, ON);
            sinDirections(POSITIVE, POSITIVE, NEGATIVE);
            sin_amplitude = 90;
            break;
        case 38://red green blue cyan merging 
            flag_green_sin = true;
            flag_blue_sin = true;
            coloursOn(ON, ON, ON);
            sinDirections(POSITIVE, POSITIVE, NEGATIVE);
            sin_amplitude = 90;
            break;
        case 39://red green blue yellow merging 
            flag_red_sin = true;
            flag_green_sin = true;
            coloursOn(ON, ON, ON);
            sinDirections(POSITIVE, NEGATIVE, POSITIVE);
            sin_amplitude = 90;
            break;
        case 40://red green blue random positions
            coloursOn(ON, ON, ON);
            TIMSK1 |= (1 << OCIE1A);// enable timer compare interrupt
            break;
        case 41://red green blue rainbow
            coloursOn(ON, ON, ON);
            red_count = base_loop_count;
            green_count = base_loop_count / 3;
            blue_count = base_loop_count * 2 / 3;
            setSingleDuty(45, 'R');
            setSingleDuty(45, 'G');
            setSingleDuty(45, 'B');
            break;
        case 42://red green blue rainbow
            coloursOn(ON, ON, ON);
            red_count = base_loop_count;
            green_count = base_loop_count / 3;
            blue_count = base_loop_count * 2 / 3;
            setSingleDuty(70, 'R');
            setSingleDuty(70, 'G');
            setSingleDuty(70, 'B');
            break;  
        case 43://red cyan flow
            red_count = base_loop_count / 2;
            coloursOn(ON, ON, ON);
            setSingleDuty(70, 'R');
            setSingleDuty(70, 'G');
            setSingleDuty(70, 'B');
            break;
        case 44://green blue flow
            green_count = base_loop_count / 2;
            coloursOn(OFF, ON, ON);
            setSingleDuty(70, 'G');
            setSingleDuty(70, 'B');
            break;
        case 45://green purple flow
            green_count = base_loop_count / 2;
            coloursOn(ON, ON, ON);
            setSingleDuty(70, 'R');
            setSingleDuty(70, 'G');
            setSingleDuty(70, 'B');
            break;
        case 46://blue yellow flow
            blue_count = base_loop_count / 2;
            coloursOn(ON, ON, ON);
            setSingleDuty(70, 'R');
            setSingleDuty(70, 'G');
            setSingleDuty(70, 'B');
            break;
    }
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void setup(){
    Serial.begin(9600);//HC-05 Bluetooth module serial baud rate
    pinMode(PUMP_PIN, OUTPUT);
    pinMode(RED_PIN, OUTPUT);
    pinMode(GREEN_PIN, OUTPUT);
    pinMode(BLUE_PIN, OUTPUT);
    PORTD &= ~_BV(RED_PIN);//Sets digital pin low
    PORTD &= ~_BV(GREEN_PIN);//Sets digital pin low
    PORTD &= ~_BV(BLUE_PIN);//Sets digital pin low  

    //set timer1 interrupt at 1Hz
    TCCR1A = 0;// set entire TCCR1A register to 0
    TCCR1B = 0;// same for TCCR1B
    TCNT1  = 0;//initialize counter value to 0
    OCR1A = 15624;//m16000000 / (1*1024) - 1;// = (16*10^6) / (36*1024) - 1 (must be <65536)  // set compare match register for 1hz increments
    TCCR1B |= (1 << WGM12);// turn on CTC mode
    TCCR1B |= (1 << CS12) | (1 << CS10);  // Set CS12 and CS10 bits for 1024 prescaler
    setMode(37);
    flag_pump_off = false;
    pump_duty_value = 245;
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void loop(){   
    if(Serial.available()){//Checks if serial data is available
        delay(10);//wait to make sure all data in the serial message has arived
        char c = Serial.read();//read and store the first character sent
        stringText = "";//clear stringText
        while(Serial.available()){//set elemetns of stringText to the serial values sent
            char digit = Serial.read();
            //if(isDigit(digit)){//concatinates any digitis in the received data
            stringText += digit; 
            //}
            if(stringText.length() >= MAX_STRING_LENGTH) break;//exit the loop when the stringText array is full
        }
        serialFlush();//Clear any excess data in the serial buffer
        
        int serialCommandValueInt = stringText.toInt();
        float serialCommandValueFloat = stringText.toFloat();

        switch(c){
            case 'f'://set base strobe frequency
                setBaseStrobeFrequency(serialCommandValueInt);
                break;
            case 'p'://set pump PWM
                setPumpPWM(serialCommandValueInt);
            break;
            case 'm'://set mode
                if(serialCommandValueInt > 0 && serialCommandValueInt <= 46){//set maximum mode number
                    setMode(serialCommandValueInt);
                    printi("Mode set to: ", serialCommandValueInt);
                }
                else{
                    printi("Invalid mode number... Mode number entered: ", serialCommandValueInt);
                }
                break;
            case 'o'://everything off
                printi("Time Fountain switched off...\n");
                coloursOn(OFF, OFF, OFF);
                flag_pump_off = true;
                break;
            case 'a':
                sin_amplitude = abs(serialCommandValueInt);
                printi("sin_amplitude set to: ", abs(sin_amplitude));
                break;
            case 'd':
                setLedDuty(serialCommandValueInt);
                break;
            case 'r':
                setPhase(serialCommandValueInt, "R");
                break;
            case 'R':
                setSingleDuty(serialCommandValueInt, c);
                break;
            case 'g':
                setPhase(serialCommandValueInt, "G");
                break;
            case 'G':
                setSingleDuty(serialCommandValueInt, c);
                break;
            case 'b':
                setPhase(serialCommandValueInt, "B");
                break;
            case 'B':
                setSingleDuty(serialCommandValueInt, c);
                break;
            case 'q'://set base strobe frequency
                setBaseStrobeFrequency(base_loop_count + 10);
                break;
            case 'w'://set base strobe frequency
                setBaseStrobeFrequency(base_loop_count - 10);
                break;
            case 's':
                setSinCountIncrement(serialCommandValueFloat);
                break;
            case 'H':
                printi("______________Status______________\n");
                printi("Base loop count: ", base_loop_count, "\n\n"); 
                printi("Red loop count: ", red_max_count);
                printi("Red duty: ", red_duty, 1, "\n\n");
                printi("Green loop count: ", green_max_count);
                printi("Green duty: ", green_duty, 1, "\n\n");
                printi("Blue loop count: ", blue_max_count);
                printi("Blue duty: ", blue_duty, 1, "\n\n");
                printi("Mode: ", mode_status, "\n\n");
                printi("Pump duty: ", ((float)pump_duty_value/2.55), 1, "%\n");
                printi("Sin increment:", sin_count_increment);
                printi("Sin peak", sin_amplitude);
                printi("_____________________________\n\n");              
                break;
            case 'h'://TODO: correct all values and ensure all commands are included they are correct
                printi("_____________________________\n");
                printi("loop count: f\n");
                printi("pump duty: p\n");
                printi("Mode: m\n");
                printi("Off: o\n");
                printi("Sin peak: a\n");
                printi("All LED duty cycles: d\n");
                printi("Red duty: r\n");
                printi("Green duty: g\n");
                printi("Blue duty: b\n");
                printi("Red count change: R\n");
                printi("Green count change: G\n");
                printi("Blue count change: b\n");
                printi("Increase loop count: q\n");
                printi("Decrease loop count: w\n");
                printi("Sin increment: s\n");
                printi("Sin peak a\n");
                printi("Display current settings: H\n");
                printi("_____________________________\n\n");
                break;
            case 'M':
                printi("_____________________________\n");
                printi("Modes:\n");
                printi("1-7: Single colours\n");
                printi("8-13: Double colours\n");
                printi("14-19: Bidirectional\n");
                printi("20-23: Tripple variations\n");
                printi("24-30: Single sins\n");
                printi("31-36: Double merging\n");
                printi("37-39: Triple merging\n");
                printi("40: Random Positions\n");
                printi("41-46: Flowing colours\n");
                printi("_____________________________\n\n");
                break;
        }
    }

    if(red_count < red_duty_value){//Switch LEDs on/off for the set duty/frequency
        PORTD |= _BV(RED_PIN);  //write port HIGH
    }
    else{
        PORTD &= ~_BV(RED_PIN);  //write port LOW
    }
    
    if(green_count < green_duty_value){
        PORTD |= _BV(GREEN_PIN);  //write port HIGH
    }
    else {
        PORTD &= ~_BV(GREEN_PIN);  //write port LOW
    }
    
    if(blue_count < blue_duty_value){
        PORTD |= _BV(BLUE_PIN);  //write port B5 HIGH
    }
    else {
        PORTD &= ~_BV(BLUE_PIN);  //write port B5 LOW
    }

    if(pump_count == pump_duty_value){
        PORTD &= ~_BV(PUMP_PIN);  //write port B5 LOW
    }
    else if(pump_count >= PUMP_MAX_COUNT && !flag_pump_off){
        PORTD |= _BV(PUMP_PIN);  //write port B5 HIGH
        pump_count = 0;
    }

    base_count++;//Increment counts  
    red_count++;
    green_count++;
    blue_count++;
    pump_count++;

    if(base_count >= base_loop_count){
        base_count = 0;
        if(flag_ms_stop){
            ms = millis() - ms;
            flag_ms_stop = false;
            flag_ms_start = false;
            float hz = 1/((float)ms/1000.0);
            printi("Estimated frequency: ", hz, 3, "Hz\n");
        }
        if(flag_ms_start){
            ms = millis();
            flag_ms_stop = true;
        }
        if(flag_red_sin){
            if(red_sin_direction == POSITIVE){
                red_count = base_loop_count_temp * (sin_amplitude * cos(sin_count) + sin_amplitude);
            }
            else{
                red_count = base_loop_count_temp * (360 - (sin_amplitude * cos(sin_count) + sin_amplitude));
            }
        }
        if(flag_green_sin){
            if(green_sin_direction == POSITIVE){
                green_count = base_loop_count_temp * (sin_amplitude * cos(sin_count) + sin_amplitude);
            }
            else{
                green_count = base_loop_count_temp * (360 - (sin_amplitude * cos(sin_count) + sin_amplitude));
            }
            
        }
        if(flag_blue_sin){
            if(blue_sin_direction == POSITIVE){
                blue_count = base_loop_count_temp * (sin_amplitude * cos(sin_count) + sin_amplitude);
            }
            else{
                blue_count = base_loop_count_temp * (360 - (sin_amplitude * cos(sin_count) + sin_amplitude));
            }
        }
        
        sin_count += sin_count_increment;//0.0174533 rads = 1 deg
        if(sin_count >= 6.283188){//6.283188 radians in 360 degrees
        }
    }

    if(red_count >= red_max_count){
        red_count = 0;
    }
    if(green_count >= green_max_count){
        green_count = 0;
    }
    if(blue_count >= blue_max_count){
        blue_count = 0;
    }  
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/
