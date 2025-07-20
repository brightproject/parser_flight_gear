#include <Arduino.h>
#include "parserfgfs.h"

ParserFGFS::ParserFGFS() {
}

// void ParserFGFS::parseData(Print& output) {
void ParserFGFS::parseData(Stream& output) {
    // int bytesRead = Serial.readBytesUntil(LINE_END, buffer, BUFF_SIZE);
    int bytesRead = output.readBytesUntil(LINE_END, buffer, BUFF_SIZE);
    buffer[bytesRead] = '\0';

    char* token = strtok(buffer, "\t");

    for (int i = 0; i < CHUNK; ++i) {
        if (token != nullptr) { // Проверка на наличие токена
            datasim[i] = atof(token); // Парсинг данных в массив
            token = strtok(NULL, "\t");
        }
    }

    // блоки для парсера данных, заданных в файле ias-alt-qnh.xml
    // 1
    elapsed_sec = datasim[0];
    #ifdef DEBUG_PARSER
    output.print("Elapsed time in sec: ");
    output.print(elapsed_sec, 2);
    output.print(BLOCK_END);
    #endif
    // 2
    roll_deg = datasim[1];
    #ifdef DEBUG_PARSER
    output.print("Roll in deg: ");
    output.print(roll_deg, 2);
    output.print(BLOCK_END);
    #endif
    // 3   
    pitch_deg = datasim[2];
    #ifdef DEBUG_PARSER
    output.print("Pitch in deg: ");
    output.print(pitch_deg, 2);
    output.print(BLOCK_END);
    #endif 
    // 4   
    heading_magnetic_deg = datasim[3];
    #ifdef DEBUG_PARSER
    output.print("Heading in deg: ");
    output.print(heading_magnetic_deg, 2);
    output.print(BLOCK_END);
    #endif 
    // 5   
    jsbsim_accel_x = datasim[4];
    #ifdef DEBUG_PARSER
    output.print("Jsbsim accel x (m/s/s): ");
    output.print(jsbsim_accel_x, 2);
    output.print(BLOCK_END);
    #endif 
    // 6   
    jsbsim_accel_y = datasim[5];
    #ifdef DEBUG_PARSER
    output.print("Jsbsim accel y (m/s/s): ");
    output.print(jsbsim_accel_y, 2);
    output.print(BLOCK_END);
    #endif 
    // 7  
    jsbsim_accel_z = datasim[6];
    #ifdef DEBUG_PARSER
    output.print("Jsbsim accel z (m/s/s): ");
    output.print(jsbsim_accel_z, 2);
    output.print(BLOCK_END);
    #endif 
    // 8  
    accel_x = datasim[7];
    #ifdef DEBUG_PARSER
    output.print("Accel x (m/s/s): ");
    output.print(accel_x, 2);
    output.print(BLOCK_END);
    #endif 
    // 9  
    accel_y = datasim[8];
    #ifdef DEBUG_PARSER
    output.print("Accel y (m/s/s): ");
    output.print(accel_y, 2);
    output.print(BLOCK_END);
    #endif 
    // 10  
    accel_z = datasim[9];
    #ifdef DEBUG_PARSER
    output.print("Accel z (m/s/s): ");
    output.print(accel_z, 2);
    output.print(BLOCK_END);
    #endif 
    // 11  
    indicated_slip_skid = datasim[10];
    #ifdef DEBUG_PARSER
    output.print("Slip-skid: ");
    output.print(indicated_slip_skid, 2);
    output.print(LINE_END);
    #endif 
}
