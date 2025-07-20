#include <Arduino.h>

// надо добавить, иначе ошибка компиляции кода, т.к. библиотека также добавлена в общий каталог 
#include "Adafruit_BNO08x.h"

// библиотека CAN шины
// https://github.com/coryjfowler/MCP_CAN_lib
#include <mcp_can_2017.h>

// библиотека парсера FlightGear
#include <parserfgfs.h>

// #define  SERIAL_PORT_SPEED  115200
#define  SERIAL_PORT_SPEED  230400

// #define  PLOTTER_ON

//CS pin for CAN board
const int CAN_CS_PIN = PA4;

//CAN Message details
// unsigned char canMsg[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
unsigned char can_msg_roll[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
unsigned char can_msg_pitch[8] = {0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27};
unsigned char can_msg_heading[8] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37};
unsigned char can_msg_slip[8] = {0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47};

// Глобальные переменные для хранения состояния
unsigned long last_time = 0; // Предыдущее время
const long parsingInterval = 10; // интервал в миллисекундах
const unsigned int CAN_Ahrs_Period = 20; // How often message sent in milliseconds
const unsigned int CAN_Slip_Period = 20; // How often message sent in milliseconds
unsigned long CAN_Ahrs_Timestamp = 0; // when was the last message sent
unsigned long CAN_Slip_Timestamp = 0; // when was the last message sent

unsigned long previousMillis = 0;  // хранит время последнего вызова

// Flight state identifier definitions from can aero space - CAN_AS
// https://github.com/makerplane/FIX-Gateway/blob/master/fixgw/config/canfix/map.yaml

#define GRAVITY    (9.81)

// #define BODY_EULER_ANGLES_ID    0x15E    /* in DEC 350 a/c euler angles */
#define BODY_PITCH_ANGLE_ID    0x137    /* in DEC 311 a/c pitch angle */
#define BODY_ROLL_ANGLE_ID     0x138    /* in DEC 312 a/c roll angle */
#define BODY_SIDESLIP_ID       0x139    /* in DEC 313 a/c sideslip */
#define HEADING_ANGLE_ID       0x141    /* in DEC 321 heading angle */
// #define TURN_COORD_RATE_ID     0x14B    /* in DEC 331 turn coordination rate */

// объявление функций
float calculateSlipSkidBall(float y_accel, float z_accel, unsigned long current_time);

// объявление объектов классов
ParserFGFS parser;
MCP_CAN CAN(CAN_CS_PIN);

// Инициализация последовательного порта USART2
HardwareSerial ANALIZER(USART2);

void setup(void) {

  // Запуск последовательного порта для мониторинга и передачи данных
  Serial.begin(SERIAL_PORT_SPEED); // PA9, PA10
  ANALIZER.begin(SERIAL_PORT_SPEED); // PA2, PA3

  while(CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
  // while(CAN_OK != CAN.begin(CAN_125KBPS, MCP_8MHZ)) {
    Serial.println("Unable to begin CAN BUS");
    delay(1000);
  }
  CAN.setMode(MCP_NORMAL);
  Serial.println("CAN BUS Shield init ok!");

} // SETUP

void loop() {

  // if (Serial.available()) {
  if (ANALIZER.available()) {
  
  if (millis() - previousMillis >= parsingInterval) {
    // Сохраняем текущее время
    previousMillis = millis();
    
    // Вызов метода и указание из какого COM-порта парсить данные
    // parser.parseData(Serial);
    parser.parseData(ANALIZER);

    Serial.print("Fgfs ROLL: ");
    Serial.println(parser.roll_deg);
    Serial.print("Fgfs PITCH: ");
    Serial.println(parser.pitch_deg);
    Serial.print("Fgfs HEADING: ");
    Serial.println(parser.heading_magnetic_deg);
  
    Serial.print("Fgfs SLIP: ");
    Serial.println(parser.indicated_slip_skid);
  
    // Serial.println("jsbsim");
    // Serial.println(parser.jsbsim_accel_y);
    // Serial.println(parser.jsbsim_accel_z);

    // Serial.println("accel");
    // Serial.println(parser.accel_y);
    // Serial.println(parser.accel_z);
  }
}

    if (millis() - CAN_Ahrs_Timestamp >= CAN_Ahrs_Period) {

    // секция получения, обработки и отправки ROLL
    float roll = parser.roll_deg;

    const float SCALE_FACTOR_ROLL = 100; // Например, умножаем на 100 для сохранения двух знаков после запятой

      if (roll < 0.0) {
          can_msg_roll[0] = '-'; // Если roll отрицательное 45 в ASCII
      } else {
          can_msg_roll[0] = '+'; // Если roll положительное 43 в ASCII
      }

    // Преобразуем float в unsigned long всегда положительное с учетом SCALE_FACTOR_ROLL 
    unsigned long scaledRoll = fabs(roll * SCALE_FACTOR_ROLL);

    can_msg_roll[1] = scaledRoll;
    can_msg_roll[2] = scaledRoll >> 8;

    Serial.print("ROLL: ");
    Serial.println(roll);

    Serial.print("CAN ROLL: ");
    Serial.print(can_msg_roll[0]); 
    Serial.print(" "); 
    Serial.println((can_msg_roll[2] << 8 | can_msg_roll[1]) / SCALE_FACTOR_ROLL, 2); // Возвращаем к значению с плавающей точкой

    // секция получения, обработки и отправки PITCH
    float pitch = parser.pitch_deg;

    const float SCALE_FACTOR_PITCH = 100; // Например, умножаем на 100 для сохранения двух знаков после запятой

      if (pitch < 0.0) {
          can_msg_pitch[0] = '-'; // Если pitch отрицательное 45 в ASCII
      } else {
          can_msg_pitch[0] = '+'; // Если pitch положительное 43 в ASCII
      }

    // Преобразуем float в unsigned long всегда положительное с учетом SCALE_FACTOR_PITCH 
    unsigned long scaledPitch = fabs(pitch * SCALE_FACTOR_PITCH);

    can_msg_pitch[1] = scaledPitch;
    can_msg_pitch[2] = scaledPitch >> 8;

    Serial.print("PITCH: ");
    Serial.println(pitch);

    Serial.print("CAN PITCH: ");
    Serial.print(can_msg_pitch[0]); 
    Serial.print(" "); 
    Serial.println((can_msg_pitch[2] << 8 | can_msg_pitch[1]) / SCALE_FACTOR_PITCH, 2); // Возвращаем к значению с плавающей точкой

    // секция получения, обработки и отправки HEADING
    // float heading = parser.heading_magnetic_deg;

    unsigned long heading = int(parser.heading_magnetic_deg);
    can_msg_heading[1] = heading;
    can_msg_heading[2] = heading >> 8;

    // const float SCALE_FACTOR_HDG = 100; // Например, умножаем на 100 для сохранения двух знаков после запятой

      // if (heading < 0.0) {
      //     can_msg_heading[0] = '-'; // Если heading отрицательное 45 в ASCII
      // } else {
      //     can_msg_heading[0] = '+'; // Если heading положительное 43 в ASCII
      // }

    // Преобразуем float в unsigned long всегда положительное с учетом SCALE_FACTOR_HDG 
    // unsigned long scaledHdg = fabs(heading * SCALE_FACTOR_HDG);

    // can_msg_heading[1] = scaledHdg;
    // can_msg_heading[2] = scaledHdg >> 8;

    Serial.print("HEADING: ");
    Serial.println(heading);

    Serial.print("CAN HEADING: ");
    // Serial.print(can_msg_heading[0]); 
    // Serial.print(" "); 
    Serial.println((can_msg_heading[2] << 8 | can_msg_heading[1]));

    CAN.sendMsgBuf(BODY_PITCH_ANGLE_ID, 0, 8, can_msg_pitch); 
    CAN.sendMsgBuf(BODY_ROLL_ANGLE_ID, 0, 8, can_msg_roll); 
    CAN.sendMsgBuf(HEADING_ANGLE_ID, 0, 8, can_msg_heading); 

    CAN_Ahrs_Timestamp = millis();

    }

    // send slip indicator data
    if (millis() - CAN_Slip_Timestamp >= CAN_Slip_Period) {

        // unsigned long slipInt = calculateSlipSkidBall(parser.accel_y, parser.accel_z, millis());
        // влево положителен, вправо отрицателен
        float slip = calculateSlipSkidBall(parser.jsbsim_accel_y / GRAVITY, parser.jsbsim_accel_z / GRAVITY, millis());


        const float SCALE_FACTOR_SLIP = 100; // Например, умножаем на 100 для сохранения двух знаков после запятой

          if (slip < 0.0) {
              can_msg_slip[0] = '-'; // Если pitchInt отрицательное
          } else {
              can_msg_slip[0] = '+'; // Если pitchInt положительное
          }

        // Преобразуем float в unsigned long всегда положительное с учетом SCALE_FACTOR_SLIP 
        unsigned long scaledSlip = fabs(slip * SCALE_FACTOR_SLIP);

        can_msg_slip[1] = scaledSlip;
        can_msg_slip[2] = scaledSlip >> 8;

        Serial.print("SLIP: ");
        Serial.println(slip);

        Serial.print("CAN SLIP: ");
        Serial.print(can_msg_slip[0]); 
        Serial.print(" "); 
        Serial.println((can_msg_slip[2] << 8 | can_msg_slip[1]) / SCALE_FACTOR_SLIP, 2); // Возвращаем к значению с плавающей точкой

        CAN.sendMsgBuf(BODY_SIDESLIP_ID, 0, 8, can_msg_slip); 

        CAN_Slip_Timestamp = millis(); // Обновляем временную метрику
    }



} //LOOP

// Функция для расчета SlipSkidBall, принимающая текущее время, ускорения по осям Y и Z
// https://sourceforge.net/p/flightgear/flightgear/ci/next/tree/src/Instrumentation/slip_skid_ball.cxx
float calculateSlipSkidBall(float y_accel, float z_accel, unsigned long current_time) {
    // Рассчитываем delta_time_sec, время с последнего обновления
    float delta_time_sec = (current_time - last_time) / 1000.0; // миллисекунды -> секунды
    last_time = current_time; // Обновляем время

    // Вычисление переменной d
    float d = -z_accel; // Отрицательное значение ускорения по оси Z
    if (d < 1.0)
        d = 1.0; // Устанавливаем минимальное значение для d

    // Рассчитываем позицию
    float pos = y_accel / d * 10.0;

    // Применяем фильтр низких частот (Low Pass Filter)
    static float last_pos = 0.0; // Хранит последнее значение pos
    float alpha = 0.1; // Коэффициент сглаживания (можно настроить)

    float filtered_pos = (alpha * pos) + ((1 - alpha) * last_pos);
    last_pos = filtered_pos;

    // Возвращаем отфильтрованное значение SlipSkidBall
    return filtered_pos;
}
