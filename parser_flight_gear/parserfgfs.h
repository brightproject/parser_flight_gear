#ifndef PARSERFGFS_H
#define PARSERFGFS_H

// #define DEBUG_PARSER
// PARSER for accel-slip-skid.xml

#define BUFF_SIZE 200 // Чем больше пакетов, тем больше буфер, но не слишком большой
#define CHUNK 11
#define BLOCK_END '\t'
#define LINE_END '\n'

class ParserFGFS
{
    public:
        ParserFGFS();
        // void parseData(Print& output);
        void parseData(Stream& output);

        float elapsed_sec;
        float roll_deg;
        float pitch_deg;
        float heading_magnetic_deg;
        float jsbsim_accel_x;
        float jsbsim_accel_y;
        float jsbsim_accel_z;
        float accel_x;
        float accel_y;
        float accel_z;
        float indicated_slip_skid;

    private:
        char buffer[BUFF_SIZE];
        float datasim[CHUNK]; // Массив для хранения спарсенных данных
};

#endif // PARSERFGFS_H
