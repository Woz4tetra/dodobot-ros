#include <Arduino.h>
#include <Encoder.h>

#define COMM_SERIAL Serial

#define MOTORA_ENCA 23
#define MOTORA_ENCB 22
#define MOTORB_ENCA 21
#define MOTORB_ENCB 20

#define ENCODER_SAMPLERATE_DELAY_MS 33  // ~30 Hz

#define CURRENT_TIME millis()

Encoder motorA_enc(MOTORA_ENCB, MOTORA_ENCA);
Encoder motorB_enc(MOTORB_ENCA, MOTORB_ENCB);

long encA_pos, encB_pos = 0;
double enc_speedA, enc_speedB = 0.0;  // ticks/s, smoothed
double enc_speedA_raw, enc_speedB_raw = 0.0;  // ticks/s

uint32_t prev_enc_time = 0;

double speed_smooth_kA = 1.0;
double speed_smooth_kB = 1.0;

void reset_encoders()
{
    encA_pos = 0;
    encB_pos = 0;
    motorA_enc.write(0);
    motorB_enc.write(0);
}

bool read_encoders()
{
    if (CURRENT_TIME - prev_enc_time < ENCODER_SAMPLERATE_DELAY_MS) {
        return false;
    }

    long new_encA_pos = motorA_enc.read();
    long new_encB_pos = motorB_enc.read();

    // bool should_report = false;
    // if (new_encA_pos != encA_pos || new_encB_pos != encB_pos) {
    //     should_report = true;
    // }

    enc_speedA_raw = (double)(new_encA_pos - encA_pos) / (CURRENT_TIME - prev_enc_time) * 1000.0;
    enc_speedB_raw = (double)(new_encB_pos - encB_pos) / (CURRENT_TIME - prev_enc_time) * 1000.0;
    enc_speedA += speed_smooth_kA * (enc_speedA_raw - enc_speedA);
    enc_speedB += speed_smooth_kB * (enc_speedB_raw - enc_speedB);

    encA_pos = new_encA_pos;
    encB_pos = new_encB_pos;

    prev_enc_time = CURRENT_TIME;

    // return should_report;
    return true;
}

void setup()
{
    COMM_SERIAL.begin(9600);
}

void loop()
{
    if (COMM_SERIAL.available()) {
        reset_encoders();
    }

    if (read_encoders()) {
        COMM_SERIAL.print("A:\t");
        COMM_SERIAL.print(encA_pos);
        COMM_SERIAL.print("\tAv:\t");
        COMM_SERIAL.print(enc_speedA);
        COMM_SERIAL.print("\nB:\t");
        COMM_SERIAL.print(encB_pos);
        COMM_SERIAL.print("\tBv:\t");
        COMM_SERIAL.print(enc_speedB);
        COMM_SERIAL.print("\n\n");
    }
}
