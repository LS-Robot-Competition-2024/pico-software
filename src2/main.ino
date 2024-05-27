#include "config.h"

#define MAIN_ADDRESS 0
#define TUNING_ADDRESS_1 28
#define TUNING_ADDRESS_2 56

float err;
float prev_err;
bool cross;

int cross_count;
int blind_steps;
int loop_count;

unsigned long start;
unsigned long prev_time;
const unsigned long delay_time = 1;

float pt, it, dt;

int true_pos;

float angle_patterns[256];

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
    err = prev_err = cross = cross_count = blind_steps = loop_count = 0;
    start = millis();
    prev_time = 0;
    pt = it = dt = 0;
    true_pos = 0;
}

void print_test_mode() {
    SerialBT.println("main mode");
    SerialBT.println("main: 1");
    SerialBT.println("tune: 2");
    SerialBT.println("set main_params: 3");
    SerialBT.println("init tune_params: 4");
    SerialBT.println("print all_params: 5");
    SerialBT.println("check senser: 6");
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
            }
            print_test_mode();
        }
        delay(1000);
    }
}

void set_main_params() {
    read_params(kp, ki, kd, kr, ky, score, MAIN_ADDRESS);

    String param_name[] = {"kp", "ki", "kd", "kr", "ky", "score"};
    float param_val[] = {kp, ki, kd, kr, ky, score};

    for (int i = 0; i < 6; i++) {
        SerialBT.println(param_name[i] + "?");
        while (1) {
            if (SerialBT.available()) {
                String tmp = read_string_BT();
                SerialBT.println(tmp);
                if (is_number(tmp)) {
                    param_val[i] = tmp.toFloat();
                }
                SerialBT.println(param_val[i]);
                break;
            }
        }
    }
    write_params(param_val[0], param_val[1], param_val[2], param_val[3], param_val[4], param_val[5], MAIN_ADDRESS);
}

void init_tune_params() {
    SerialBT.println("main");
    read_params(kp, ki, kd, kr, ky, score, MAIN_ADDRESS);

    float kp2, ki2, kd2, kr2, ky2, score2;
    SerialBT.println("tuning1");
    read_params(kp2, ki2, kd2, kr2, ky2, score2, TUNING_ADDRESS_1);

    SerialBT.println("tuning2");
    read_params(kp2, ki2, kd2, kr2, ky2, score2, TUNING_ADDRESS_2);

    write_params(kp, ki, kd, kr, ky, score, TUNING_ADDRESS_1);
    write_params(kp, ki, kd, kr, ky, score, TUNING_ADDRESS_2);
}

void print_all_params() {
    SerialBT.println("main");
    read_params(kp, ki, kd, kr, ky, score, MAIN_ADDRESS);

    SerialBT.println("tuning1");
    read_params(kp, ki, kd, kr, ky, score, TUNING_ADDRESS_1);

    SerialBT.println("tuning2");
    read_params(kp, ki, kd, kr, ky, score, TUNING_ADDRESS_2);
}

void run_line_trace() {
    bool run = true;
    read_params(kp, ki, kd, kr, ky, score, MAIN_ADDRESS);

    precompute_angle();
    init_val();
    while (1) {
        if (run) {
            loop_count++;
            if (!pid_control()) {
                run = false;
                score = (float)(millis() - start) / 1000.;
                SerialBT.printf("loop count: %d\n", loop_count);
                write_params(kp, ki, kd, kr, ky, score, MAIN_ADDRESS);
            }
        } else {
            move_motors(0, 0);
            break;
        }
    }
}

void tune_line_trace() {
    bool run = true;
    read_params(kp, ki, kd, kr, ky, score, TUNING_ADDRESS_1);
    float kp2, ki2, kd2, kr2, ky2, score2;
    read_params(kp2, ki2, kd2, kr2, ky2, score2, TUNING_ADDRESS_2);
    float best_kp, best_ki, best_kd, best_kr, best_ky, best_score;
    read_params(best_kp, best_ki, best_kd, best_kr, best_ky, best_score, MAIN_ADDRESS);

    float score_diff = score - score2;
    update_param(kp, kp2, score_diff, 50, 150);
    update_param(ki, ki2, err, 0, 50);
    update_param(kd, kd2, err, 0, 50);
    update_param(kr, kr2, err, 0.5, 1);
    update_param(ky, ky2, err, 5, 20);
    score2 = score;

    precompute_angle();
    init_val();
    while (1) {
        if (run) {
            if (!pid_control()) {
                run = false;
                score = (float)(millis() - start) / 1000.;
                write_params(kp, ki, kd, kr, ky, score, TUNING_ADDRESS_1);
                write_params(kp2, ki2, kd2, kr2, ky2, score2, TUNING_ADDRESS_2);
                if (best_score < score) {
                    write_params(best_kp, best_ki, best_kd, best_kr, best_ky, best_score, MAIN_ADDRESS);
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
    int for (int i = 0; i < 8; i++) {
        if (gpio_get(senser_pins[i])) {
            x |= (1 << (7 - i));
        }
    }

    if (x) {
        blind_steps = 0;
        if (x == 0xff) {
            cross = true;
        } else if (cross) {
            cross = false;
            cross_count++;
        }
    } else {
        blind_steps++;
        cross = false;
    }
    return angle_patterns[x];
}

bool pid_control() {
    unsigned long cur_time = millis();
    if (cur_time - prev_time < delay_time) {
        return 1;
    }
    err = read_senser();
    if (err == 0) {
        if (prev_err > 0) {
            err = max_angle;
        } else if (prev_err < 0) {
            err = -max_angle;
        }
    }
    pt = err;
    it = it * kr + err;
    dt = err - prev_err;

    int speed = pt * kp + it * ki + dt * kd;

    move_motors(k_speed + speed, k_speed - speed);
    prev_err = err;
    return (cross_count < 3 && blind_steps < 1000);
}

void move_motors(int l_speed, int r_speed) {
    l_speed = constrain(l_speed, -255, 255);
    r_speed = constrain(r_speed, -255, 255);

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
    float cp;
    if (err_ == 0) {
        cp = p;
    } else if (err_ * (p - pre_p) >= 0) {
        cp = p + abs((p - pre_p) / 2.);
    } else {
        cp = p - abs((p - pre_p) / 2.);
    }
    pre_p = p;
    p = constrain(cp + noise, min_, max_);
}

void precompute_angle() {
    for (int i = 0; i < 256; i++) {
        angle_patterns[i] = 0;
        int size = count_bits(i);
        float val = calc_bit_weights(i);
        switch (size) {
            case 0:
                break;
            case 1:
            case 2:
            case 3:
                angle_patterns[i] = atan(val / size / ky);
                break;
            case 4:
            case 5:
            case 6:
            case 7:
            case 8:
                if (val > 0) {
                    angle_patterns[i] = max_angle;
                } else if (val < 0) {
                    angle_patterns[i] = -max_angle;
                }
                break;
        }
    }
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
