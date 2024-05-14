#include <stdint.h>
#include <stdlib.h>  // abs関数のために追加

// モータアクセル値の配列
uint32_t motor_accel_values[256];

// 値を0と255の間に制限する関数
int clip_value(int value) {
    if (value < -255) {
        return -255;
    } else if (value > 255) {
        return 255;
    } else {
        return value;
    }
}

#define LINE_MIDDLE 0
#define MAX_SPEED   255

float get_average_line_position(uint8_t sensor_value, int8_t* sensor_weight){
    int length = 0;
    int sum    = 0;
    for(int n = 0; n < 8; n++){
        if(sensor_value >> n & 1){
            sum += sensor_weight[n];
            length++;
        }
    }
    if(length == 0){
        return 0;
    }else{
       return sum / length;
    //    return sum;
    }
}

static float error[2] = {0.0, 0.0};
static float integral   = 0.0;
static float last_error = 0.0;

void init_pid() {
	error[0] = 0.0;
	error[1] = 0.0;
	integral = 0.0;
	last_error = 0.0;
}

uint32_t control_motors(uint8_t sensor_value, int8_t* sensor_weight, float speed, float Kp, float Ki, float Kd, float deltaT) {

    float position = get_average_line_position(sensor_value, sensor_weight);
    
	error[0]  = error[1];
	error[1]  = -position;
    integral += error[1];

	float p = Kp * error[1];
	float i = Ki * integral;
    float d = (error[1] - error[0]) / deltaT;

    // PID制御の計算
    float correction = p + i + d;

    // モーター速度の設定
    // int motor_speed_left  = clip_value((int)(MAX_SPEED * (speed + correction)));
    // int motor_speed_right = clip_value((int)(MAX_SPEED * (speed - correction)));

    int motor_speed_left  = 0;
    int motor_speed_right = 0;

	if (correction >= 0){
    	motor_speed_left  = clip_value((int)(MAX_SPEED * (speed)));
    	motor_speed_right = clip_value((int)(MAX_SPEED * (speed - correction)));
	} else {
    	motor_speed_right = clip_value((int)(MAX_SPEED * (speed)));
    	motor_speed_left  = clip_value((int)(MAX_SPEED * (speed + correction)));
	}
	
	uint8_t leftMotorAccelForward   = (motor_speed_left <= 0) ? 0 : motor_speed_left;
    uint8_t leftMotorAccelBackward  = (motor_speed_left >  0) ? 0 : -motor_speed_left;
    uint8_t rightMotorAccelForward  = (motor_speed_right <= 0) ? 0 : motor_speed_right;
    uint8_t rightMotorAccelBackward = (motor_speed_right >  0) ? 0 : -motor_speed_right;

	return (uint32_t)(leftMotorAccelForward << 24) | (uint32_t)(leftMotorAccelBackward << 16) | (uint32_t)(rightMotorAccelForward << 8) | (uint32_t)(rightMotorAccelBackward);
}