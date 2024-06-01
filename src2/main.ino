#include "config.h"

#define MAIN_ADDRESS 0
#define TUNING_ADDRESS_1 28
#define TUNING_ADDRESS_2 56
#define SPEED_ADDRESS 84
#define TIME_ADDRESS 88

float max_angle;
int k_speed;
int limit_time;

float err;
float prev_err;

int non_line_period;
unsigned long start;
const unsigned long delay_time = 1;

float pt, it, dt;

float angle_patterns[256];
float score_weights[] = {1, 2, 3, 4, 4, 3, 2, 1};
float score_patterns[256];

int last_edge;
float param_range[5][2] = {{90., 170.}, {0., 3.}, {0., 50.}, {0.3, 0.96}, {8., 25}};  // p i d r y

void setup() {
    io_init();
    setup_BT();
    randomSeed(generate_seed());
    EEPROM.begin(512);
}
void loop() {
    choose_mode();
}

void init_val() {
    err = prev_err = non_line_period = 0;
    start = millis();
    pt = it = dt = 0;
    score = 0;
    last_edge = 0;
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
            }
            print_test_mode();
        }
        delay(1000);
    }
}

void set_main_params() {
    read_params(kp, ki, kd, kr, ky, score, MAIN_ADDRESS);
    print_params(kp, ki, kd, kr, ky, score);

    String param_name[] = {"kp", "ki", "kd", "kr", "ky"};
    float param_val[] = {kp, ki, kd, kr, ky};
    for (int i = 0; i < 5; i++) {
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
    write_params(param_val[0], param_val[1], param_val[2], param_val[3], param_val[4], 0., MAIN_ADDRESS);
    print_params(param_val[0], param_val[1], param_val[2], param_val[3], param_val[4], 0.);
}

void init_tune_params() {
    SerialBT.println("read");
    SerialBT.println("main");
    read_params(kp, ki, kd, kr, ky, score, MAIN_ADDRESS);
    print_params(kp, ki, kd, kr, ky, score);

    float kp2, ki2, kd2, kr2, ky2, score2;
    SerialBT.println("tuning1");
    read_params(kp2, ki2, kd2, kr2, ky2, score2, TUNING_ADDRESS_1);
    print_params(kp2, ki2, kd2, kr2, ky2, score2);

    SerialBT.println("tuning2");
    read_params(kp2, ki2, kd2, kr2, ky2, score2, TUNING_ADDRESS_2);
    print_params(kp2, ki2, kd2, kr2, ky2, score2);

    SerialBT.println("write");
    write_params(kp, ki, kd, kr, ky, score, TUNING_ADDRESS_1);
    write_params(kp, ki, kd, kr, ky, score, TUNING_ADDRESS_2);
}

void print_all_params() {
    SerialBT.println("main");
    read_params(kp, ki, kd, kr, ky, score, MAIN_ADDRESS);
    print_params(kp, ki, kd, kr, ky, score);
    SerialBT.println("tuning1");
    read_params(kp, ki, kd, kr, ky, score, TUNING_ADDRESS_1);
    print_params(kp, ki, kd, kr, ky, score);
    SerialBT.println("tuning2");
    read_params(kp, ki, kd, kr, ky, score, TUNING_ADDRESS_2);
    print_params(kp, ki, kd, kr, ky, score);
}

void run_line_trace() {
    limit_time = get_int(TIME_ADDRESS);
    k_speed = get_int(SPEED_ADDRESS);

    bool run = true;
    read_params(kp, ki, kd, kr, ky, score, MAIN_ADDRESS);
    print_params(kp, ki, kd, kr, ky, score);

    precompute_angle();
    precompute_score();
    init_val();
    while (1) {
        if (run) {
            if (!pid_control()) {
                run = false;
                score = score / 1000 * k_speed;
                SerialBT.printf("time: %.1f\n", (millis() - start) / 1000);
                write_params(kp, ki, kd, kr, ky, score, MAIN_ADDRESS);
                print_params(kp, ki, kd, kr, ky, score);
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

    bool run = true;
    read_params(kp, ki, kd, kr, ky, score, TUNING_ADDRESS_1);
    float kp2, ki2, kd2, kr2, ky2, score2;
    read_params(kp2, ki2, kd2, kr2, ky2, score2, TUNING_ADDRESS_2);
    float best_kp, best_ki, best_kd, best_kr, best_ky, best_score;
    read_params(best_kp, best_ki, best_kd, best_kr, best_ky, best_score, MAIN_ADDRESS);

    float score_diff = score - score2;
    update_param(kp, kp2, score_diff, param_range[0][0], param_range[0][1]);
    update_param(ki, ki2, score_diff, param_range[1][0], param_range[1][1]);
    update_param(kd, kd2, score_diff, param_range[2][0], param_range[2][1]);
    update_param(kr, kr2, score_diff, param_range[3][0], param_range[3][1]);
    update_param(ky, ky2, score_diff, param_range[4][0], param_range[4][1]);
    score2 = score;

    precompute_angle();
    precompute_score();
    init_val();
    while (1) {
        if (run) {
            if (!pid_control()) {
                run = false;
                score = score / 1000 * k_speed;
                write_params(kp, ki, kd, kr, ky, score, TUNING_ADDRESS_1);
                SerialBT.println("current");
                print_params(kp, ki, kd, kr, ky, score);
                write_params(kp2, ki2, kd2, kr2, ky2, score2, TUNING_ADDRESS_2);
                SerialBT.println("previous");
                print_params(kp2, ki2, kd2, kr2, ky2, score2);
                if (best_score < score) {
                    SerialBT.println("update!!");
                    write_params(kp, ki, kd, kr, ky, score, MAIN_ADDRESS);
                }
            }
        } else {
            move_motors(0, 0);
            break;
        }
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
            return max_angle;
        } else if (last_edge < 0) {
            return -max_angle;
        }
    }
    if (score_patterns[x]) {
        score += score_patterns[x];
        non_line_period = 0;
    } else {
        non_line_period++;
    }
    return angle_patterns[x];
}
bool pid_control() {
    unsigned long cur_time = millis();
    delay(delay_time);
    if (cur_time - start >= limit_time) {
        return 0;
    }

    err = read_senser();
    pt = err;
    it = it * kr + err;
    dt = err - prev_err;

    int speed = pt * kp + it * ki + dt * kd;

    move_motors(k_speed + speed, k_speed - speed);
    prev_err = err;
    return non_line_period < (2000. / delay_time);
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

void update_param(float& p, float& pre_p, float err_, float min_, float max_) {
    float noise = uniform_random((max_ - min_) / 20.);
    if(random(2)){
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
    p = constrain16(cp + noise, min_, max_);
}

void precompute_angle() {
    for (int i = 0; i < 256; i++) {
        float pos = convert_bit(i);
        angle_patterns[i] = atan(pos / ky);
    }
    max_angle = atan(12. / ky);
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
            k_speed = constrain16(k_speed, 50, 200);
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