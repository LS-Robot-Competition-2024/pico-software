#include <LittleFS.h>
#include <SerialBT.h>

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
#define SENSER_COUNT 8

#define MAX_ANGLE M_PI / 3.;

const int k_speed = 100;
const int senser_pins[] = {SENSOR_PIN_0, SENSOR_PIN_1, SENSOR_PIN_2, SENSOR_PIN_3,
                           SENSOR_PIN_4, SENSOR_PIN_5, SENSOR_PIN_6, SENSOR_PIN_7};
int senser_weights[] = {-4, -3, -2, -1, 1, 2, 3, 4};

float kp, ki, kd, kr, ky, score;

int cross_count;
int blind_steps;
int total_line_steps;

float angle_patterns[256];

void setup() {
    SerialBT.begin();

    pinMode(SENSOR_PIN_0, INPUT);
    pinMode(SENSOR_PIN_1, INPUT);
    pinMode(SENSOR_PIN_2, INPUT);
    pinMode(SENSOR_PIN_3, INPUT);
    pinMode(SENSOR_PIN_4, INPUT);
    pinMode(SENSOR_PIN_5, INPUT);
    pinMode(SENSOR_PIN_6, INPUT);
    pinMode(SENSOR_PIN_7, INPUT);

    pinMode(MOTOR_PIN_0, OUTPUT);
    pinMode(MOTOR_PIN_1, OUTPUT);
    pinMode(MOTOR_PIN_2, OUTPUT);
    pinMode(MOTOR_PIN_3, OUTPUT);
}

void loop() {
    regular_run();
    test_run();
}

void read_params(float& kp_, float& ki_, float& kd_, float& kr_, float& ky_, float& score_, String file_name) {
    File file = LittleFS.open(file_name, "r");
    if (!file) {
        SerialBT.println("Error opening file for writing");
        return;
    }
    String data = file.readString();
    file.close();

    sscanf(data.c_str(), "%f,%f,%f,%f,%f,%d\n", &kp_, &ki_, &kd_, &kr_, &ky_, &score_);
    SerialBT.printf("%.3f,%.3f,%.3f,%.3f,%.3f,%d\n", kp_, ki_, kd_, kr_, ky_, score_);
}

void write_params(float kp_, float ki_, float kd_, float kr_, float ky_, float score_, String file_name) {
    File file = LittleFS.open(file_name, "w");
    file.printf("%f,%f,%f,%f,%f,%d\n", kp_, ki_, kd_, kr_, ky_, score_);
    SerialBT.printf("%.3f,%.3f,%.3f,%.3f,%.3f,%d\n", kp_, ki_, kd_, kr_, ky_, score_);
    file.close();
}

void regular_run() {
    bool active = true;
    read_params(kp, ki, kd, kr, ky, score, "params.txt");
    calculate_angle();
    while (1) {
        if (active) {
            if (!pid_control()) {
                active = false;
                write_params(kp, ki, kd, kr, ky, score, "params.txt");
            }
        } else {
            move_motors(0, 0);
        }
    }
}
void test_run() {
    bool active = true;

    float kp2, ki2, kd2, kr2, ky2, score2;
    float best_kp, best_ki, best_kd, best_kr, best_ky, best_score;

    kp2 = ki2 = kd2 = kr2 = ky2 = score2 = 0;
    best_kp = best_ki = best_kd = best_kr = best_ky = best_score = 0;

    read_params(kp, ki, kd, kr, ky, score, "test.txt");
    read_params(kp2, ki2, kd2, kr2, ky2, score2, "test2.txt");
    read_params(best_kp, best_ki, best_kd, best_kr, best_ky, best_score, "params.txt");

    float err = score - score2;
    randomSeed(analogRead(ANALOG_SEED_PIN));
    update_params(kp, kp2, err, 200, 50);
    update_params(ki, ki2, err, 100, 0);
    update_params(kd, kd2, err, 100, 0);
    update_params(kr, kr2, err, 1, 0.5);
    update_params(ky, ky2, err, 20, 5);
    score2 = score;

    calculate_angle();
    while (1) {
        if (active) {
            if (!pid_control()) {
                active = false;
                write_params(kp2, ki2, kd2, kr2, ky2, score2, "test2.txt");
                write_params(kp, ki, kd, kr, ky, score, "test.txt");
                if (best_score < score) {
                    write_params(kp, ki, kd, kr, ky, score, "params.txt");
                }
            }
        } else {
            move_motors(0, 0);
        }
    }
}
bool pid_control() {
    static float err = 0;
    static float prev_err = 0;
    static float integral = 0;

    err = read_senser();
    if (err == 0) {
        if (prev_err > 0) {
            err = MAX_ANGLE;
        } else if (prev_err < 0) {
            err = -MAX_ANGLE;
        }
    }

    float pt = err;
    float it = integral * kr + err;
    float dt = err - prev_err;

    int speed = pt * kp + it * ki + dt * kd;
    move_motors(k_speed + speed, k_speed - speed);

    prev_err = err;
    return (cross_count < 3 || blind_steps < 20);
}

float read_senser() {
    static bool cross = false;
    int x = 0;
    for (int i = 0; i < 8; i++) {
        if (digitalRead(senser_pins[i])) {
            x |= (1 << (7 - i));
        }
    }
    if (x) {
        blind_steps = 0;
        total_line_steps++;
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

void move_motors(int l_speed, int r_speed) {
    l_speed = constrain(l_speed, -255, 255);
    r_speed = constrain(r_speed, -255, 255);

    if (l_speed > 0) {
        analogWrite(MOTOR_PIN_0, l_speed);
        analogWrite(MOTOR_PIN_1, 0);
    } else {
        analogWrite(MOTOR_PIN_0, 0);
        analogWrite(MOTOR_PIN_1, -l_speed);
    }
    if (r_speed > 0) {
        analogWrite(MOTOR_PIN_2, r_speed);
        analogWrite(MOTOR_PIN_3, 0);
    } else {
        analogWrite(MOTOR_PIN_2, 0);
        analogWrite(MOTOR_PIN_3, -r_speed);
    }
}

void calculate_angle() {
    for (int i = 0; i < 256; i++) {
        int bits = clear_bits(i);
        int size = count_bits(bits);
        int val = calc_bit_weights(bits);
        switch (size) {
            case 0:
                angle_patterns[i] = 0;
                break;
            case 1:
                if (val > 0) {
                    angle_patterns[i] = atan((4. + 5.) / 2. / ky);
                } else {
                    angle_patterns[i] = atan(-(4. + 5.) / 2. / ky);
                }
                break;
            case 2:
            case 3:
                angle_patterns[i] = atan((float)val / size / ky);
                break;
            case 4:
            case 5:
            case 6:
            case 7:
            case 8:
                if (val > 0) {
                    angle_patterns[i] = MAX_ANGLE;
                } else if (val < 0) {
                    angle_patterns[i] = -MAX_ANGLE;
                } else {
                    angle_patterns[i] = 0;
                }
        }
    }
}

int clear_bits(int x) {
    if (x == 0x1 || x == 0x80) {
        return x;
    }
    int mask = x & (x >> 1);
    mask |= x & (x << 1);
    return mask;
}

int count_bits(int x) {
    int cnt = 0;
    while (x) {
        cnt += x & 1;
        x >>= 1;
    }
    return cnt;
}

int calc_bit_weights(int x) {
    int sum = 0;
    for (int i = 0; i < 8; i++) {
        if (x & (1 << (7 - i))) {
            sum += senser_weights[i];
        }
    }
    return sum;
}

void update_params(float& p, float& pre_p, float err, float max_, float min_) {
    float n = uniform_random((max_ - min_) / 20.);
    float cp;
    if (err / (p - pre_p)) {
        cp = p + abs((p - pre_p) / 2.);
    } else {
        cp = p - abs((p - pre_p) / 2.);
    }
    pre_p = p;
    p = constrain(cp + n, min_, max_);
}

float uniform_random(float x) {
    int rate = random(0, 1000);
    float ret = (float)rate / 1000 * (x);
    return ret;
}
