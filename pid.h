#ifndef __PID_CONTROLLER_H
#define __PID_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

#define CONSTRAIN(x, lower, upper) ((x) < (lower) ? (lower) : ((x) > (upper) ? (upper) : (x)))

typedef enum
{
	DIRECT,
	REVERSE
} PIDDirection;

class PIDControl
{
public:
	PIDControl(float kp, float ki, float kd, float sampleTimeSeconds,
			   float minOutput, float maxOutput,
			   PIDDirection controllerDirection);

	void run(float current, float target, float &out);
	void tuningsSet(float kp, float ki, float kd); // 设置参数

	inline void tuningKpSet(float kp) { this->tuningsSet(kp, alteredKi, alteredKd); };
	inline void tuningKiSet(float ki) { this->tuningsSet(alteredKp, ki, alteredKd); };
	inline void tuningKdSet(float kd) { this->tuningsSet(alteredKp, alteredKi, kd); };

	inline float kpGet() { return this->alteredKp; }
	inline float kiGet() { return this->alteredKi; }
	inline float kdGet() { return this->alteredKd; }

private:
	float input;	// 输入值
	float setpoint; // 设定值
	float output;	// 输出值

	float lastInput;
	float sampleTime; // 单位s
	float outMin;
	float outMax;

	float iTerm; // 积分项

	float alteredKp;
	float alteredKi;
	float alteredKd;

	PIDDirection controllerDirection; // 反馈方向, 选择DIRECT

	void compute(void); // 其实是调节的核心函数, 但是传递参数有点麻烦

	void outputLimitsSet(float min, float max);
	void sampleTimeSet(float sampleTimeSeconds);
};

#endif

/*********************PID_CONTROLLER_H******************/
