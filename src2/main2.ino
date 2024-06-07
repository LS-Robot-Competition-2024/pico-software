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

#define MAIN_ADDRESS 0
#define TUNING_ADDRESS_1 40
#define TUNING_ADDRESS_2 80
#define SPEED_ADDRESS 120
#define TIME_ADDRESS 124

const unsigned long delay_time = 1;
const int motor_pins[] = {MOTOR_PIN_0, MOTOR_PIN_1, MOTOR_PIN_2, MOTOR_PIN_3};  // lf lb rf rb
const int senser_pins[] = {SENSOR_PIN_0, SENSOR_PIN_1, SENSOR_PIN_2, SENSOR_PIN_3,
                           SENSOR_PIN_4, SENSOR_PIN_5, SENSOR_PIN_6, SENSOR_PIN_7};

int k_speed;
int limit_time;

float err;
float prev_err;

int non_line_period;
unsigned long start;

float kp, ki, kd, kr, score;
float pt, it, dt;
float position_weights[5];
float senser_weights[8];

float score_patterns[256];
float score_weights[] = {1, 2, 3, 4, 4, 3, 2, 1};
float line_positions[256];

int last_edge;
float param_range[9][2] = {{20., 100.}, {0., 3.},   {0., 50.},  {0, 0.96},  {0.08, 0.4},
                           {0.2, 0.9},  {0.4, 1.4}, {0.6, 1.7}, {0.76, 2.2}};
// p i d r pos1 pos2 pos3 pod4 pos5
void setup() {
    io_init();
    setup_BT();
    randomSeed(generate_seed());
    EEPROM.begin(512);
}
void loop() {
    choose_mode();
}

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
void setup_BT() {
    SerialBT.begin();
    while (!SerialBT.available()) {
        delay(2000);
    }
    char tmp = read_char_BT();
    SerialBT.println("Connection Successful!!");
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

void choose_mode() {
    print_test_mode();
    while (1) {
        if (SerialBT.available()) {
            char mode = read_char_BT();
            SerialBT.printf("mode: %c\n", mode);
            switch (mode) {
                case '1':
                    run_line_trace();
                    break;
                case '2':
                    tune_line_trace();
                    break;
                case '3':
                    set_main_params();
                    break;
                case '4':
                    init_tune_params();
                    break;
                case '5':
                    print_all_params();
                    break;
                case '6':
                    check_line_senser();
                    break;
                case '7':
                    set_speed();
                    break;
                case '8':
                    set_limit_time();
                    break;
                case '9':
                    check_motor();
                    break;
                case 'a':
                    clear_eeprom();
                    break;
            }
            print_test_mode();
        }
        delay(1000);
    }
}
void print_test_mode() {
    SerialBT.println("main mode");
    SerialBT.println("main: 1");
    SerialBT.println("tune: 2");
    SerialBT.println("set main_params: 3");
    SerialBT.println("init tune_params: 4");
    SerialBT.println("print all_params: 5");
    SerialBT.println("check senser: 6");
    SerialBT.println("set speed: 7");
    SerialBT.println("set limit time: 8");
    SerialBT.println("check motor: 9");
    SerialBT.println("clear eeprom: a");
}
void init_val() {
    err = prev_err = non_line_period = 0;
    start = millis();
    pt = it = dt = 0;
    score = 0;
    last_edge = 0;
}
void run_line_trace() {
    limit_time = get_int(TIME_ADDRESS);
    k_speed = get_int(SPEED_ADDRESS);

    read_params(kp, ki, kd, kr, position_weights, score, MAIN_ADDRESS);
    print_params(kp, ki, kd, kr, position_weights, score);

    precompute_positions();
    precompute_score();
    init_val();
    bool run = true;
    while (1) {
        if (run) {
            pid_control();
            if ((millis() - start >= limit_time) || (non_line_period > (2000. / delay_time))) {
                run = false;
                score = score / 1000 * k_speed;
                SerialBT.printf("time: %.1f\n", (millis() - start) / 1000);
                write_params(kp, ki, kd, kr, position_weights, score, MAIN_ADDRESS);
                print_params(kp, ki, kd, kr, position_weights, score);
            }
        } else {
            move_motors(0, 0);
            break;
        }
    }
}
void tune_line_trace() {
    limit_time = get_int(TIME_ADDRESS);
    k_speed = get_int(SPEED_ADDRESS);

    read_params(kp, ki, kd, kr, position_weights, score, TUNING_ADDRESS_1);
    float kp2, ki2, kd2, kr2, score2, weights2[5];
    read_params(kp2, ki2, kd2, kr2, weights2, score2, TUNING_ADDRESS_2);
    float best_kp, best_ki, best_kd, best_kr, best_score, best_weights[5];
    read_params(best_kp, best_ki, best_kd, best_kr, best_weights, best_score, MAIN_ADDRESS);

    float score_diff = score - score2;
    update_param(kp, kp2, score_diff, param_range[0]);
    update_param(ki, ki2, score_diff, param_range[1]);
    update_param(kd, kd2, score_diff, param_range[2]);
    update_param(kr, kr2, score_diff, param_range[3]);
    update_param(position_weights[0], weights2[0], score_diff, param_range[4]);

    float tmp;
    tmp = param_range[5][0];
    param_range[5][0] = max(param_range[5][0], position_weights[0] + 0.1);
    update_param(position_weights[1], weights2[1], score_diff, param_range[5]);
    param_range[5][0] = tmp;

    tmp = param_range[6][0];
    param_range[6][0] = max(param_range[6][0], position_weights[1] + 0.1);
    update_param(position_weights[2], weights2[2], score_diff, param_range[6]);
    param_range[6][0] = tmp;

    tmp = param_range[7][0];
    param_range[7][0] = max(param_range[7][0], position_weights[2] + 0.1);
    update_param(position_weights[3], weights2[3], score_diff, param_range[7]);
    param_range[7][0] = tmp;

    tmp = param_range[8][0];
    param_range[8][0] = max(param_range[8][0], position_weights[3] + 0.1);
    update_param(position_weights[4], weights2[4], score_diff, param_range[8]);
    param_range[8][0] = tmp;

    score2 = score;

    precompute_positions();
    precompute_score();
    init_val();
    bool run = true;
    while (1) {
        if (run) {
            pid_control();
            if ((millis() - start >= limit_time) || (non_line_period > (2000. / delay_time))) {
                run = false;
                score = score / 1000 * k_speed;
                write_params(kp, ki, kd, kr, position_weights, score, TUNING_ADDRESS_1);
                SerialBT.println("current");
                print_params(kp, ki, kd, kr, position_weights, score);
                write_params(kp2, ki2, kd2, kr2, weights2, score2, TUNING_ADDRESS_2);
                SerialBT.println("previous");
                print_params(kp2, ki2, kd2, kr2, weights2, score2);
                if (best_score < score) {
                    SerialBT.println("update!!");
                    write_params(kp, ki, kd, kr, position_weights, score, MAIN_ADDRESS);
                }
            }
        } else {
            move_motors(0, 0);
            break;
        }
    }
}
void read_params(float& kp_, float& ki_, float& kd_, float& kr_, float (&weight_)[5], float& score_, int offset) {
    kp_ = get_float(offset);
    ki_ = get_float(offset + 4 * 1);
    kd_ = get_float(offset + 4 * 2);
    kr_ = get_float(offset + 4 * 3);
    weight_[0] = get_float(offset + 4 * 4);
    weight_[1] = get_float(offset + 4 * 5);
    weight_[2] = get_float(offset + 4 * 6);
    weight_[3] = get_float(offset + 4 * 7);
    weight_[4] = get_float(offset + 4 * 8);
    score_ = get_float(offset + 4 * 9);
}
void print_params(float kp_, float ki_, float kd_, float kr_, float (&weight_)[5], float score_) {
    SerialBT.printf("kp: %.2f\n", kp_);
    SerialBT.printf("ki: %.2f\n", ki_);
    SerialBT.printf("kd: %.2f\n", kd_);
    SerialBT.printf("kr: %.2f\n", kr_);
    SerialBT.printf("pos1: %.2f\n", weight_[0]);
    SerialBT.printf("pos2: %.2f\n", weight_[1]);
    SerialBT.printf("pos3: %.2f\n", weight_[2]);
    SerialBT.printf("pos4: %.2f\n", weight_[3]);
    SerialBT.printf("pos5: %.2f\n", weight_[4]);
    SerialBT.printf("score: %.2f\n", score_);
}
void write_params(float kp_, float ki_, float kd_, float kr_, float (&weight_)[5], float score_, int offset) {
    put_float(offset, kp_);
    put_float(offset + 4 * 1, ki_);
    put_float(offset + 4 * 2, kd_);
    put_float(offset + 4 * 3, kr_);
    put_float(offset + 4 * 4, weight_[0]);
    put_float(offset + 4 * 5, weight_[1]);
    put_float(offset + 4 * 6, weight_[2]);
    put_float(offset + 4 * 7, weight_[3]);
    put_float(offset + 4 * 8, weight_[4]);
    put_float(offset + 4 * 9, score_);
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
int get_int(int addr) {
    union {
        int val;
        byte bytes[4];
    } data;
    for (int i = 0; i < 4; i++) {
        data.bytes[i] = EEPROM.read(addr + i);
    }
    return data.val;
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
void pid_control() {
    delay(delay_time);

    err = read_senser();
    pt = err;
    it = it * kr + err;
    dt = err - prev_err;

    int speed = pt * kp + it * ki + dt * kd;

    move_motors(k_speed + speed, k_speed - speed);
    prev_err = err;
}
void move_motors(int l_speed, int r_speed) {
    l_speed = constrain16(l_speed, -255, 255);
    r_speed = constrain16(r_speed, -255, 255);

    if (l_speed >= 0) {
        pwm_set_gpio_level(MOTOR_PIN_0, l_speed);
        pwm_set_gpio_level(MOTOR_PIN_1, 0);
    } else {
        pwm_set_gpio_level(MOTOR_PIN_0, 0);
        pwm_set_gpio_level(MOTOR_PIN_1, -l_speed);
    }
    if (r_speed >= 0) {
        pwm_set_gpio_level(MOTOR_PIN_2, r_speed);
        pwm_set_gpio_level(MOTOR_PIN_3, 0);
    } else {
        pwm_set_gpio_level(MOTOR_PIN_2, 0);
        pwm_set_gpio_level(MOTOR_PIN_3, -r_speed);
    }
}
float read_senser() {
    int x = 0;
    for (int i = 0; i < 8; i++) {
        if (gpio_get(senser_pins[i])) {
            x |= (1 << (7 - i));
        }
    }
    if (x) {
        if ((x & 0x1) && !(x & 0x80)) {
            last_edge = 1;
        } else if (!(x & 0x1) && (x & 0x80)) {
            last_edge = -1;
        }
    } else {
        if (0 < last_edge) {
            return position_weights[4];
        } else if (last_edge < 0) {
            return -position_weights[4];
        }
    }
    if (score_patterns[x]) {
        score += score_patterns[x];
        non_line_period = 0;
    } else {
        non_line_period++;
    }
    return line_positions[x];
}
void precompute_positions() {
    for (int i = 0; i < 4; i++) {
        senser_weights[i + 4] = position_weights[i];
        senser_weights[3 - i] = -position_weights[i];
    }
    for (int i = 0; i < 256; i++) {
        line_positions[i] = convert_bit(i);
    }
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
    v /= s;
    if (s == 2 && x2) {
        return v;
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
float calc_bit_weights(int x) {
    float sum = 0;
    for (int i = 0; i < 8; i++) {
        if (x & (1 << (7 - i))) {
            sum += senser_weights[i];
        }
    }
    return sum;
}
int count_bits(int x) {
    int cnt = 0;
    while (x) {
        cnt += x & 1;
        x >>= 1;
    }
    return cnt;
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
void update_param(float& p, float& pre_p, float err_, float range_[2]) {
    float noise = uniform_random(range_[1] - range_[0]) / 30.;
    if (random(2)) {
        noise = -noise;
    }
    float cp;
    if (err_ == 0) {
        cp = p;
    } else if (err_ * (p - pre_p) >= 0) {
        cp = p + abs((p - pre_p) / 2.);
    } else {
        cp = p - abs((p - pre_p) / 2.);
    }
    pre_p = p;
    p = constrain16(cp + noise, range_[0], range_[1]);
}
float uniform_random(float x) {
    int rate = random(1001);
    float ret = (float)rate / 1000. * (x);
    return ret;
}
void set_main_params() {
    read_params(kp, ki, kd, kr, position_weights, score, MAIN_ADDRESS);
    print_params(kp, ki, kd, kr, position_weights, score);

    String param_name[] = {"kp", "ki", "kd", "kr", "pos1", "pos2", "pos3", "pos4", "pos5"};
    float param_val[] = {kp,
                         ki,
                         kd,
                         kr,
                         position_weights[0],
                         position_weights[1],
                         position_weights[2],
                         position_weights[3],
                         position_weights[4]};

    for (int i = 0; i < 9; i++) {
        SerialBT.println(param_name[i] + "?");
        while (1) {
            if (SerialBT.available()) {
                String tmp = read_string_BT();
                if (is_number(tmp)) {
                    param_val[i] = tmp.toFloat();
                }
                param_val[i] = constrain16(param_val[i], param_range[i][0], param_range[i][1]);
                SerialBT.println(param_val[i]);
                break;
            }
        }
    }
    float weight[] = {param_val[4], param_val[5], param_val[6], param_val[7], param_val[8]};
    write_params(param_val[0], param_val[1], param_val[2], param_val[3], weight, 0., MAIN_ADDRESS);
    print_params(param_val[0], param_val[1], param_val[2], param_val[3], weight, 0.);
}
void init_tune_params() {
    SerialBT.println("read");
    SerialBT.println("main");
    read_params(kp, ki, kd, kr, position_weights, score, MAIN_ADDRESS);
    print_params(kp, ki, kd, kr, position_weights, score);

    float kp2, ki2, kd2, kr2, score2, weights2[5];
    SerialBT.println("tuning1");
    read_params(kp2, ki2, kd2, kr2, weights2, score2, TUNING_ADDRESS_1);
    print_params(kp2, ki2, kd2, kr2, weights2, score2);

    SerialBT.println("tuning2");
    read_params(kp2, ki2, kd2, kr2, weights2, score2, TUNING_ADDRESS_2);
    print_params(kp2, ki2, kd2, kr2, weights2, score2);

    SerialBT.println("write");
    write_params(kp, ki, kd, kr, position_weights, score, TUNING_ADDRESS_1);
    write_params(kp, ki, kd, kr, position_weights, score, TUNING_ADDRESS_2);
}
void print_all_params() {
    SerialBT.println("main");
    read_params(kp, ki, kd, kr, position_weights, score, MAIN_ADDRESS);
    print_params(kp, ki, kd, kr, position_weights, score);
    SerialBT.println("tuning1");
    read_params(kp, ki, kd, kr, position_weights, score, TUNING_ADDRESS_1);
    print_params(kp, ki, kd, kr, position_weights, score);
    SerialBT.println("tuning2");
    read_params(kp, ki, kd, kr, position_weights, score, TUNING_ADDRESS_2);
    print_params(kp, ki, kd, kr, position_weights, score);
}
void check_line_senser() {
    SerialBT.println("stop:s");
    while (1) {
        if (SerialBT.available()) {
            char val = read_char_BT();
            if (val == 's') {
                break;
            }
        }
        int x = 0;
        for (int i = 0; i < 8; i++) {
            if (gpio_get(senser_pins[i])) {
                x |= (1 << (7 - i));
            }
        }
        String v = String(x, BIN);
        while (v.length() < 8) {
            v = "0" + v;
        }
        SerialBT.println(v);
    }
}
void set_speed() {
    k_speed = get_int(SPEED_ADDRESS);
    SerialBT.println(k_speed);
    while (1) {
        if (SerialBT.available()) {
            String tmp = read_string_BT();
            if (is_number(tmp)) {
                k_speed = tmp.toInt();
            }
            k_speed = constrain16(k_speed, 50, 150);
            put_int(SPEED_ADDRESS, k_speed);
            break;
        }
    }
    SerialBT.println(k_speed);
}
void set_limit_time() {
    limit_time = get_int(TIME_ADDRESS);
    SerialBT.println(limit_time);
    while (1) {
        if (SerialBT.available()) {
            String tmp = read_string_BT();
            if (is_number(tmp)) {
                limit_time = tmp.toInt();
            }
            limit_time = constrain16(limit_time, 10 * 1000, 60 * 1000);
            put_int(TIME_ADDRESS, limit_time);
            break;
        }
    }
    SerialBT.println(limit_time);
}
void precompute_score() {
    for (int i = 0; i < 256; i++) {
        float sum = 0;
        int size = 0;
        for (int j = 0; j < 8; j++) {
            if (i & (1 << (7 - j))) {
                sum += score_weights[j];
                size++;
            }
        }
        if (size == 1 || size == 2) {
            score_patterns[i] = sum / size;
        }
    }
}
void check_motor() {
    move_motors(100, 100);
    delay(2000);
    move_motors(0, 0);
    delay(2000);
    move_motors(50, -50);
    delay(2000);
    move_motors(0, 0);
    delay(2000);
    move_motors(-50, 50);
    delay(2000);
    move_motors(0, 0);
}
void clear_eeprom() {
    for (int i = 0; i < 512; i++) {
        EEPROM.write(i, 0);
    }
    EEPROM.commit();
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