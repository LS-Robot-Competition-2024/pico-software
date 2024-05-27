#include "config.h"

#define LOOP_READ 10000
#define LOOP_WRITE 100

void setup() {
    io_init();
    setup_BT();
    randomSeed(generate_seed());
    EEPROM.begin(512);
}

void loop() {
    choose_mode();
}
void print_test_mode() {
    SerialBT.println("test mode");
    SerialBT.println("check seed: 1");
    SerialBT.println("check senser: 2");
    SerialBT.println("check read time: 3");
    SerialBT.println("check write time: 4");
    SerialBT.println("check eeprom: 5");
    SerialBT.println("clear eeprom: 6");
    SerialBT.println("clear motor: 7");
}
void choose_mode() {
    print_test_mode();
    while (1) {
        if (SerialBT.available()) {
            char mode = read_char_BT();
            SerialBT.printf("mode: %c\n", mode);
            switch (mode) {
                case '1':
                    check_analog_seed();
                    break;
                case '2':
                    check_line_senser();
                    break;
                case '3':
                    time_Read();
                    break;
                case '4':
                    time_write();
                    break;
                case '5':
                    check_eeprom();
                    break;
                case '6':
                    clear_eeprom();
                    break;
                case '7':
                    check_motor();
                    break;
            }
            print_test_mode();
        }
        delay(1000);
    }
}

void check_analog_seed() {
    unsigned long start = millis();
    for (int i = 0; i < 10; i++) {
        SerialBT.println(generate_seed());
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
        delay(1000);
        String v = String(x, BIN);
        while (v.length() < 8) {
            v = "0" + v;
        }
        SerialBT.println(v);
    }
}

void time_Read() {
    int x;
    unsigned long start;
    start = millis();
    float runtime;

    x = 0;
    for (int t = 0; t < LOOP_READ; t++) {
        for (int i = 0; i < 8; i++) {
            if (digitalRead(senser_pins[i])) {
                x |= (1 << (7 - i));
            }
        }
    }
    runtime = (float)(millis() - start) / 1000.;
    SerialBT.printf("digitalRead time: %.2fs\n", runtime);

    start = millis();
    x = 0;
    for (int t = 0; t < LOOP_READ; t++) {
        for (int i = 0; i < 8; i++) {
            if (gpio_get(senser_pins[i])) {
                x |= (1 << (7 - i));
            }
        }
    }
    runtime = (float)(millis() - start) / 1000.;
    SerialBT.printf("gpio_get time: %.2fs\n", runtime);
}

void time_write() {
    unsigned long start;
    float runtime;
    /*
    start = millis();
    for (int t = 0; t < LOOP_WRITE; t++) {
        for (int i = 0; i < 4; i++) {
            for (int v = 0; v < 256; v++) {
                analogWrite(motor_pins[i], v);
            }
            for (int v = 255; v >= 0; v--) {
                analogWrite(motor_pins[i], v);
            }
        }
    }
    runtime = (float)(millis() - start) / 1000.;
    SerialBT.printf("analogWrite time: %.2fs\n", runtime);
    */
    start = millis();
    for (int t = 0; t < LOOP_WRITE; t++) {
        for (int i = 0; i < 4; i++) {
            for (int v = 0; v < 256; v++) {
                pwm_set_gpio_level(motor_pins[i], v);
            }
            for (int v = 255; v >= 0; v--) {
                pwm_set_gpio_level(motor_pins[i], v);
            }
        }
    }
    runtime = (float)(millis() - start) / 1000.;
    SerialBT.printf("pwm_set_gpio_level time: %.2fs\n", runtime);
}

void check_eeprom() {
    int addr = 508;
    float val = get_float(addr);
    SerialBT.println(val);
    if (val) {
        val = 0;
    } else {
        val = 7.777;
    }
    put_float(addr, val);
    SerialBT.println(val);
}

void check_motor() {
    for (int i = 0; i < 4; i++) {
        SerialBT.println(i);
        delay(2000);
        pwm_set_gpio_level(motor_pins[i], 100);
        delay(2000);
        pwm_set_gpio_level(motor_pins[i], 0);
    }
}