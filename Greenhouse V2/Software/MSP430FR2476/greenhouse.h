//This program is designed for the Greenhouse V2 Control Board
/*
 * Greenhouse V2 Control Board brief description:
 * Supports 3 separate hydroponic systems
 *  - Overall system components:
 *      - Drivers: 1 water pump (+1 Driver)
 *      - Float Level Sensor: 1 (+1 Float Level Sensor)
 *  - System specific components:
 *      - Drivers: 3 perialistic pumps + 1 water pump + 1 solenoid (+5 Drivers)
 *      - Float Level Sensor: 4 (+4 Float Level Sensors)
 *      - pH Sensor: 1 (+1 pH Sensor)
 *      - TDS Sensor: (+1 TDS Sensor)
 *  - Totals (3 Systems Max):
 *      - Drivers: 15 + 1 = 16
 *      - Float Level Sensors = 12+1 = 13
 *      - pH Sensors = 3
 *      - TDS Sensors = 3
 */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <msp430.h>

#define START_ADDRESS 0x1800

#define FLOAT 0
#define PH 1
#define PH_UP 3
#define PH_DOWN 4
#define TDS 2

#define MAX_PROFILES 10
#define MAX_SENSORS 19
#define MAX_DRIVERS 16
#define MAX_SYSTEMS 3

struct plant_profile{
    char name[25];
    unsigned char system_num;
    unsigned short tds_min;
    unsigned short ph_max; //Conversion: (ph_x/65535)*14
    unsigned short ph_min;
};

struct plant_profile_list{
    struct plant_profile profiles[MAX_PROFILES];
    unsigned char size;
};

struct sensor{
    unsigned char system_num; //0 = overall, 4 = disabled. Used for additional float switches
    unsigned char sensor_type;
    unsigned char sensor_num;
    unsigned int min_timeout_ms;
    unsigned int current_level;
};

enum i2c_mode { TX, RX, TIMEOUT, IDLE};

struct i2c{
    unsigned char system_num;
    unsigned char sensor_num;
    unsigned char addresses[3];
    unsigned char i2c_transmit_buffer[32];
    unsigned char i2c_recieve_buffer[32];
    enum i2c_mode mode;
    unsigned char tx_count;
    unsigned char rx_count;
    unsigned char tx_idx;
    unsigned char rx_idx;
};

struct driver{
    unsigned char system_num;
    unsigned char driver_type;
    unsigned char driver_num;
    unsigned char *port;
    unsigned char bit;
    unsigned long activation_ms;
    unsigned char enabled;
};

struct plant_profile_list plant_profiles;
struct plant_profile active_profiles[MAX_SYSTEMS];
struct sensor sensors[MAX_SENSORS];
struct driver drivers[MAX_DRIVERS];
struct i2c tds_i2c;
volatile unsigned long driver_map[16][2];
unsigned char system_data;
unsigned char number_of_systems;



void configureSensors();
void readPlantProfiles();
void updatePlantProfiles();
void resetPlantProfiles();
void setActiveProfiles();
void addPlantProfile(struct plant_profile p);
void removePlantProfile(struct plant_profile p);
void shiftPlantProfiles(unsigned char idx);
unsigned char comparePlantProfile(struct plant_profile p, unsigned char i);
unsigned char verifyProfile(struct plant_profile p);
void readSystemData();
void updateSystemData();
void reset();
void checkSystemData();
void configureGPIO();
void configureDrivers();
void enableDrivers(unsigned char system_num);
void disableDrivers(unsigned char system_num);
void initI2c();
void readSensor(struct sensor s);
void readFloatSensor(struct sensor s);
void readTDSSensor(struct sensor s);
void readpHSensor(struct sensor s);
void enableADC(struct sensor s);
void enableTimeout(struct sensor s);
unsigned long msToCycles(unsigned long ms, unsigned long clock); //REMOVE
void initialize(unsigned char num_of_systems);

void initialize(unsigned char num_of_systems){
    number_of_systems = num_of_systems;
    //FOR TESTING BEGIN
    reset();
    //END
    readPlantProfiles();
    configureSensors();
    configureDrivers();
    initI2c();
    configureGPIO();
}

void configureGPIO(){
    //Multiplexers
    P2SEL0 &= ~(BIT0 | BIT1 | BIT2);
    P2SEL1 &= ~(BIT0 | BIT1 | BIT2);
    P2DIR |= (BIT0 | BIT1 | BIT2);
    P2OUT = 0;
    P4SEL0 &= ~(BIT1 | BIT2 | BIT5);
    P4SEL1 &= ~(BIT1 | BIT2 | BIT5);
    P4DIR &= ~(BIT1 | BIT2);
    P4DIR |= BIT5;
    P4OUT = 0;

    //Drivers
    P5SEL0 &= ~(0xFF);
    P5SEL1 &= ~(0xFF);
    P5DIR |= (0xFF);
    P5OUT = 0;
    P3SEL0 &= ~(0xFF);
    P3SEL1 &= ~(0xFF);
    P3DIR |= (0xFF);
    P3OUT = 0;
    //pH Sensors
    P1SEL0 |= 0b11000010;
    P1SEL1 |= 0b11000010;
    //EC/TDS Sensors I2C
    P1SEL0 |= BIT2 | BIT3;
    P1SEL1 &= ~(BIT2 | BIT3);
    //Wi-Fi Module UART
    P2SEL0 |= BIT5 | BIT6;
    PM5CTL0 &= ~LOCKLPM5;
}

void readSystemData(){
    memcpy(&system_data,(unsigned char *)START_ADDRESS,sizeof(unsigned char));
}

void updateSystemData(){
    SYSCFG0 = FRWPPW | PFWP;
    memcpy((unsigned char *)(START_ADDRESS), &system_data, sizeof(unsigned char));
    SYSCFG0 = FRWPPW | DFWP | PFWP;
}

void reset(){
    system_data = 255;
    updateSystemData();
    checkSystemData();
}

void checkSystemData(){
    readSystemData();
    if(system_data == 255){
        system_data = 0;
        updateSystemData();
        resetPlantProfiles();
        updatePlantProfiles();
    }
}

void resetPlantProfiles(){
    unsigned char i=0;
    for(i;i<10;i++){
        memset(&(plant_profiles.profiles[i]),0,sizeof(struct plant_profile));
    }
}

void updatePlantProfiles(){
    SYSCFG0 = FRWPPW | PFWP;
    unsigned short i=0;
    size_t profile_size = sizeof(struct plant_profile);
    for(i;i<10;i++){
        memcpy((unsigned char *)(START_ADDRESS+1+(i*profile_size)), &(plant_profiles.profiles[i]),profile_size);
    }
    SYSCFG0 = FRWPPW | DFWP | PFWP;
}

void readPlantProfiles(){
    size_t profile_size = sizeof(struct plant_profile);
    unsigned char i=0;
    for(i;i<10;i++){
        memcpy(&(plant_profiles.profiles[i]),(unsigned char *)(START_ADDRESS+1+((i)*profile_size)),profile_size);

    }
    setActiveProfiles();
}

void addPlantProfile(struct plant_profile p){
    if(verifyProfile(p)) {
        plant_profiles.profiles[plant_profiles.size++] = p;
        updatePlantProfiles();
        setActiveProfiles();
    }
}

void removePlantProfile(struct plant_profile p){
    unsigned char i=0;
    for(i;i<plant_profiles.size;i++){
        if(comparePlantProfile(p,i)) {
            memset(&(plant_profiles.profiles[i]),0,sizeof(struct plant_profile));
            shiftPlantProfiles(i);
            //break; //If we dont want to look for duplicates
        }
    }
    updatePlantProfiles();
    setActiveProfiles();
}

void shiftPlantProfiles(unsigned char idx){
    unsigned char i=idx;
    for(i;i<plant_profiles.size-1;i++){
        plant_profiles.profiles[i] = plant_profiles.profiles[i+1];
    }
    memset(&(plant_profiles.profiles[--plant_profiles.size]),0,sizeof(struct plant_profile));
}

unsigned char comparePlantProfile(struct plant_profile p, unsigned char i){
    if(!strcmp(p.name,plant_profiles.profiles[i].name) && p.system_num == plant_profiles.profiles[i].system_num && p.ph_max==plant_profiles.profiles[i].ph_max && p.ph_min == plant_profiles.profiles[i].ph_min && p.tds_min == plant_profiles.profiles[i].tds_min) return 1;
    return 0;
}

unsigned char verifyProfile(struct plant_profile p){
    unsigned char i=0;
    for(i;i<plant_profiles.size;i++){
        if(p.system_num <= 3 && p.system_num >= 1){
            if(active_profiles[p.system_num-1].system_num ==0) return 1;
            else return 0;
        }
    }
}

void setActiveProfiles(){
    memset(&active_profiles, 0, sizeof(struct plant_profile) * 3);
    unsigned char i=0;
    for(i;i<10;i++){
        if(plant_profiles.profiles[i].system_num <= 3 && plant_profiles.profiles[i].system_num >= 1){
            active_profiles[plant_profiles.profiles[i].system_num - 1] = plant_profiles.profiles[i];
        }
    }
}

void configureSensors(){
    if(number_of_systems == 0) return;
    unsigned char sensor_count=0;
    unsigned char float_sensor_num = 1;
    unsigned char pH_sensor_num = 1;
    unsigned char tds_sensor_num = 1;

    sensors[sensor_count].current_level = 0;
    sensors[sensor_count].sensor_type = FLOAT;
    sensors[sensor_count].sensor_num = float_sensor_num++;
    sensors[sensor_count++].system_num = 0;
    unsigned char i = 0;
    for(i;i<number_of_systems;i++){
        unsigned char j;
        for(j=0;j<4;j++){
            sensors[sensor_count].current_level = 0;
            sensors[sensor_count].sensor_type = FLOAT;
            sensors[sensor_count].sensor_num = float_sensor_num++;
            sensors[sensor_count++].system_num = i;
        }
        sensors[sensor_count].current_level = 0;
        sensors[sensor_count].sensor_type = PH;
        sensors[sensor_count].sensor_num = pH_sensor_num++;
        sensors[sensor_count].min_timeout_ms = 50;
        sensors[sensor_count++].system_num = i;
        sensors[sensor_count].current_level = 0;
        sensors[sensor_count].sensor_type = TDS;
        sensors[sensor_count].sensor_num = tds_sensor_num++;
        sensors[sensor_count].min_timeout_ms = 650;
        sensors[sensor_count++].system_num = i;
    }
    updateSensorTriggers();
}

void readSensor(struct sensor s){
    switch(s.sensor_type){
    case FLOAT:
        readFloatSensor(s);
        break;
    case TDS:
        readTDSSensor(s);
        break;
    case PH:
        readpHSensor(s);
        break;
    }
}

void readFloatSensor(struct sensor s){
    P4OUT |= BIT5;
    P2OUT = s.sensor_num-1;
    if(s.sensor_num <= 8) s.current_level = (P4OUT >> 1) & BIT0;
    else s.current_level = (P4OUT >> 2) & BIT0;
    P4OUT &= ~BIT5;
}

//Need probe to test functionality!
void readTDSSensor(struct sensor s){
    tds_i2c.mode = TX;

}

void initI2c(){
    tds_i2c.addresses[0] = 100;   //For chaining, addresses cannot be duplicate
    tds_i2c.addresses[1] = 100;
    tds_i2c.addresses[2] = 100;
    tds_i2c.mode = IDLE;
    memset(&tds_i2c.i2c_recieve_buffer, 0 , sizeof(unsigned char) * 32);
    memset(&tds_i2c.i2c_transmit_buffer, 0 , sizeof(unsigned char) * 32);
    tds_i2c.rx_count = 0;
    tds_i2c.tx_count = 0;
    tds_i2c.rx_idx = 0;
    tds_i2c.tx_idx = 0;
    tds_i2c.sensor_num = 0;
    tds_i2c.system_num = 0;
}

//Need probe to test functionality!
void i2cWrite(){

}

//Need probe to test functionality!
void readpHSensor(struct sensor s){
    enableADC(s);
    enableTimeout(s);
}

void enableADC(struct sensor s){
    ADCCTL0 &= ~ADCENC;
    ADCMCTL0 &= ~ADCINCH_15;
    switch(s.sensor_num){
    case 1:
        ADCMCTL0 |= ADCINCH_0;
        break;
    case 2:
        ADCMCTL0 |= ADCINCH_6;
        break;
    case 3:
        ADCMCTL0 |= ADCINCH_7;
        break;
    default:
        break;
    }
    ADCCTL0 |= ADCENC | ADCSC;
}

void enableTimeout(struct sensor s){

}

void updateSensorTriggers(){

}

void configureDrivers(){
    unsigned char driver_count = 0;
    unsigned char tds_driver_num = 1;
    unsigned char ph_up_driver_num = 1;
    unsigned char ph_down_driver_num = 1;
    unsigned char float_driver_num = 1;
    unsigned char system_num = 0;
    drivers[driver_count].driver_type = FLOAT;
    drivers[driver_count].driver_num = float_driver_num++;
    drivers[driver_count].enabled = 0;
    drivers[driver_count].system_num = system_num++;
    drivers[driver_count].port = &P5OUT;
    drivers[driver_count].bit = 0b00000001;
    drivers[driver_count++].activation_ms = 0;
    unsigned char i=0;
    i=driver_count;
    for(;i<MAX_DRIVERS;i++){
        switch((i-1)%4){
        case 0:
            drivers[driver_count].driver_type = FLOAT;
            drivers[driver_count].driver_num = float_driver_num++;
            drivers[driver_count].system_num = system_num;
            break;
        case 1:
            drivers[driver_count].driver_type = TDS;
            drivers[driver_count].driver_num = tds_driver_num++;
            drivers[driver_count].system_num = system_num;
            break;
        case 2:
            drivers[driver_count].driver_type = PH_UP;
            drivers[driver_count].driver_num = ph_up_driver_num++;
            drivers[driver_count].system_num = system_num;
            break;
        case 3:
            drivers[driver_count].driver_type = PH_DOWN;
            drivers[driver_count].driver_num = ph_down_driver_num++;
            drivers[driver_count].system_num = system_num++;
            break;
        }
        if(i<7) drivers[driver_count].port = &P5OUT;
        else drivers[driver_count].port = &P3OUT;
        drivers[driver_count].bit = 0b00000001 << (i % 8);
        drivers[driver_count].enabled = 0;
        drivers[driver_count++].activation_ms = 0;

    }
}

void enableDrivers(unsigned char system_num){
    unsigned char i=0;
    for(i;i<MAX_DRIVERS;i++){
        if(drivers[i].system_num == system_num) drivers[i].enabled = 1;
    }
}

void disableDrivers(unsigned char system_num){
    unsigned char i=0;
    for(i;i<MAX_DRIVERS;i++){
        if(drivers[i].system_num == system_num) drivers[i].enabled = 0;
    }
}

//DOESNT WORK
unsigned long msToCycles(unsigned long ms, unsigned long clock){
    unsigned long cycles = (double)(ms*clock)/1000;
    return cycles;
}

#pragma vector=TIMER0_B0_VECTOR
__interrupt void TIMEOUT_TIMER (void){
    switch(TB0IV){
    case TBIV__TBCCR1:
        break;

    }
}
