#include "pid.h"

/**
  * @brief  Initializes the PIDControl instantiation.
  * @description: This should be called at least once before any other PID functions are called on the instantiation.
  * @param	
  * kp - Positive P gain constant value.
  * ki - Positive I gain constant value.
  * kd - Positive D gain constant value.
  * sampleTimeSeconds - Interval in seconds on which PIDCompute will be called.
  * minOutput - Constrain PID output to this minimum value.
  * maxOutput - Constrain PID output to this maximum value.
  * mode - Tells how the controller should respond if the user has taken over manual control or not.
        MANUAL:    PID controller is off. User can manually control the output.
        AUTOMATIC: PID controller is on. PID controller controls the output.
  * controllerDirection - The sense of direction of the controller
        DIRECT:  A positive setpoint gives a positive output.
        REVERSE: A positive setpoint gives a negative output.
  * @retval null
  */
PIDControl::PIDControl(float kp, float ki, float kd,
					   float sampleTimeSeconds, float minOutput,
					   float maxOutput, PIDDirection direction)
{
	this->controllerDirection = direction;
	this->iTerm = 0.0f;
	this->input = 0.0f;
	this->lastInput = 0.0f;
	this->output = 0.0f;
	this->setpoint = 0.0f;

	this->sampleTime = (sampleTimeSeconds > 0.0f) ? (sampleTimeSeconds) : (1.0f);
	this->outputLimitsSet(minOutput, maxOutput);
	this->tuningsSet(kp, ki, kd);
}

/**
  * @brief  PID运行
  * @description: 开始调节
  * @param	
  * @retval null
  */
void PIDControl::run(float current, float target, float &out)
{
	this->setpoint = target; // 设置目标值
	this->input = current;	 // 输入当前值
	this->compute();
	out = this->output;
}

/**
  * @brief  PID Compute
  * @description: 中介
  * @param	
  * @retval null
  */
void PIDControl::compute()
{
	float error, dInput;

	error = setpoint - input;
	iTerm += alteredKi * error;

	iTerm = CONSTRAIN(iTerm, outMin, outMax);

	dInput = input - lastInput;
	output = alteredKp * error + iTerm - alteredKd * dInput;
	output = CONSTRAIN(output, outMin, outMax);

	lastInput = input;
}

/**
  * @brief  PID outputLimitsSet
  * @description: 输出限幅
  * @param	
  * @retval null
  */
void PIDControl::outputLimitsSet(float min, float max)
{
	if (min >= max)
	{
		return;
	}

	outMin = min;
	outMax = max;

	output = CONSTRAIN(output, min, max);
	iTerm = CONSTRAIN(iTerm, min, max);
}

/**
  * @brief  PID tuningsSet
  * @description: 参数设置
  * @param	
  * @retval null
  */
void PIDControl::tuningsSet(float kp, float ki, float kd)
{
	// Check if the parameters are valid
	if (kp < -0.001f || ki < -0.001f || kd < -0.001f)
	{
		return;
	}

	// Alter the parameters for PID
	alteredKp = kp;
	alteredKi = ki * sampleTime;
	alteredKd = kd / sampleTime;

	// Apply reverse direction to the altered values if necessary
	if (controllerDirection == REVERSE)
	{
		alteredKp = -(alteredKp);
		alteredKi = -(alteredKi);
		alteredKd = -(alteredKd);
	}
}

/**
  * @brief  PID sampleTimeSet
  * @description: 设置采样时间(s)
  * @param	
  * @retval null
  */
void PIDControl::sampleTimeSet(float sampleTimeSeconds)
{
	float ratio;

	if (sampleTimeSeconds > 0.0f)
	{
		// 计算修改的比率, 然后修改参数
		ratio = sampleTimeSeconds / this->sampleTime;
		this->alteredKi *= ratio;
		this->alteredKd /= ratio;

		this->sampleTime = sampleTimeSeconds;
	}
}
