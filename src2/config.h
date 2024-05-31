#ifndef CONFIG_H
#define CONFIG_H

#include <EEPROM.h>
#include <SerialBT.h>
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#define MOTOR_PIN_0 12
#define MOTOR_PIN_1 13
#define MOTOR_PIN_2 14
#define MOTOR_PIN_3 15

#define SENSOR_PIN_0 26
#define SENSOR_PIN_1 22
#define SENSOR_PIN_2 21
#define SENSOR_PIN_3 20
#define SENSOR_PIN_4 19
#define SENSOR_PIN_5 18
#define SENSOR_PIN_6 17
#define SENSOR_PIN_7 16

#define ANALOG_SEED_PIN 27

const int motor_pins[] = {MOTOR_PIN_0, MOTOR_PIN_1, MOTOR_PIN_2, MOTOR_PIN_3};  // lf lb rf rb
const int senser_pins[] = {SENSOR_PIN_0, SENSOR_PIN_1, SENSOR_PIN_2, SENSOR_PIN_3,
                           SENSOR_PIN_4, SENSOR_PIN_5, SENSOR_PIN_6, SENSOR_PIN_7};
const float senser_weights[] = {-5.6, -4., -2.4, -0.8, 0.8, 2.4, 4., 5.6};
float kp, ki, kd, kr, ky, score;

void io_init() {
    for (int i = 0; i < 8; i++) {
        gpio_init(senser_pins[i]);
        gpio_set_dir(senser_pins[i], GPIO_IN);
    }

    for (int i = 0; i < 4; i++) {
        gpio_init(motor_pins[i]);
        gpio_set_dir(motor_pins[i], GPIO_OUT);
        gpio_set_function(motor_pins[i], GPIO_FUNC_PWM);
        uint slice_num = pwm_gpio_to_slice_num(motor_pins[i]);
        // pwm_set_clkdiv(slice_num, 25);
        pwm_set_clkdiv(slice_num, 133000000. / 20000.);
        pwm_set_wrap(slice_num, 255);
        pwm_set_enabled(slice_num, true);
    }
}

unsigned long generate_seed() {
    unsigned long seed = 0;
    for (int i = 0; i < 32; i++) {
        seed = (seed << 1) | (analogRead(ANALOG_SEED_PIN) & 1);
    }
    return seed;
}

float uniform_random(float x) {
    int rate = random(0, 1000);
    float ret = (float)rate / 1000. * (x);
    return ret;
}

char read_char_BT() {
    char ret;
    while (SerialBT.available()) {
        char c = SerialBT.read();
        if (isAlphaNumeric(c) || c == '.' || c == '-' || c == ' ') {
            ret = c;
        }
    }
    return ret;
}
String read_string_BT() {
    String ret = "";
    while (SerialBT.available()) {
        char c = SerialBT.read();
        if (isAlphaNumeric(c) || c == '.' || c == '-' || c == ' ') {
            ret += c;
        }
    }
    return ret;
}
void setup_BT() {
    SerialBT.begin();
    while (!SerialBT.available()) {
        delay(2000);
    }
    char tmp = read_char_BT();
    SerialBT.println("Connection Successful!!");
}

void put_float(int addr, float val) {
    union {
        float val;
        byte bytes[4];
    } data;
    data.val = val;
    for (int i = 0; i < 4; i++) {
        EEPROM.write(addr + i, data.bytes[i]);
    }
    EEPROM.commit();
}
float get_float(int addr) {
    union {
        float val;
        byte bytes[4];
    } data;
    for (int i = 0; i < 4; i++) {
        data.bytes[i] = EEPROM.read(addr + i);
    }
    return data.val;
}
void put_int(int addr, int val) {
    union {
        int val;
        byte bytes[4];
    } data;
    data.val = val;
    for (int i = 0; i < 4; i++) {
        EEPROM.write(addr + i, data.bytes[i]);
    }
    EEPROM.commit();
}
float get_int(int addr) {
    union {
        int val;
        byte bytes[4];
    } data;
    for (int i = 0; i < 4; i++) {
        data.bytes[i] = EEPROM.read(addr + i);
    }
    return data.val;
}
void clear_eeprom() {
    for (int i = 0; i < 512; i++) {
        EEPROM.write(i, 0);
    }
    EEPROM.commit();
}
void print_params(float kp_, float ki_, float kd_, float kr_, float ky_, float score_) {
    SerialBT.printf("kp: %.2f\n", kp_);
    SerialBT.printf("ki: %.2f\n", ki_);
    SerialBT.printf("kd: %.2f\n", kd_);
    SerialBT.printf("kr: %.2f\n", kr_);
    SerialBT.printf("ky: %.2f\n", ky_);
    SerialBT.printf("score: %.2f\n", score_);
}
void read_params(float& kp_, float& ki_, float& kd_, float& kr_, float& ky_, float& score_, int offset) {
    kp_ = get_float(offset);
    ki_ = get_float(offset + 4 * 1);
    kd_ = get_float(offset + 4 * 2);
    kr_ = get_float(offset + 4 * 3);
    ky_ = get_float(offset + 4 * 4);
    score_ = get_float(offset + 4 * 5);
    // print_params(kp_, ki_, kd_, kr_, ky_, score_);
}
void write_params(float kp_, float ki_, float kd_, float kr_, float ky_, float score_, int offset) {
    put_float(offset, kp_);
    put_float(offset + 4 * 1, ki_);
    put_float(offset + 4 * 2, kd_);
    put_float(offset + 4 * 3, kr_);
    put_float(offset + 4 * 4, ky_);
    put_float(offset + 4 * 5, score_);
    // print_params(kp_, ki_, kd_, kr_, ky_, score_);
}
int count_bits(int x) {
    int cnt = 0;
    while (x) {
        cnt += x & 1;
        x >>= 1;
    }
    return cnt;
}
float calc_bit_weights(int x) {
    float sum = 0;
    for (int i = 0; i < 8; i++) {
        if (x & (1 << (7 - i))) {
            sum += senser_weights[i];
        }
    }
    return sum;
}
float convert_bit(int x) {
    int x2 = x & (x >> 1);
    int le = x & ~(x >> 1);
    int ri = x & ~(x << 1);

    float lv, rv, v;
    lv = calc_bit_weights(le);
    rv = calc_bit_weights(ri);
    v = calc_bit_weights(x);

    int ls, rs, s;
    s = count_bits(x);
    ls = count_bits(le);
    rs = count_bits(ri);
    if (s == 0) {
        ls = 1;
        rs = 1;
        s = 1;
    }
    lv /= ls;
    rv /= rs;
    if (s == 2 && x2) {
        return v / s;
    } else if (lv == rv) {
        return lv;
    } else if (abs(lv) == abs(rv)) {
        return 0;
    } else if (abs(lv) > abs(rv)) {
        return lv;
    } else {
        return rv;
    }
}
bool is_number(String str) {
    if (str.length() == 0) {
        return false;
    }

    bool decimal = false;
    int start = (str[0] == '-');
    for (int i = start; i < str.length(); i++) {
        if (isDigit(str.charAt(i))) {
            continue;
        } else if (str.charAt(i) == '.' && !decimal) {
            decimal = true;
        } else {
            return false;
        }
    }
    return true;
}
float constrain16(float x, float low, float high) {
    if (x < low) {
        return low;
    } else if (x > high) {
        return high;
    } else {
        return x;
    }
}
#endif