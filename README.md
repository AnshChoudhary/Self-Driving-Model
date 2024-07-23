# Self-Driving-Model

![Self Driving GIF](https://github.com/AnshChoudhary/Self-Driving-Model/blob/main/self-driving-curve-trim.gif)

### PID Controller

The basic idea behind a PID controller is to read a sensor, then compute the desired actuator output by calculating proportional, integral, and derivative responses and summing those three components to compute the output.

![image](https://user-images.githubusercontent.com/59261333/73613652-94495680-4600-11ea-9e6d-124fb5bfc188.png)

#### P - Proportional part
- The proportional part is easiest to understand: The output of the proportional part is the product of gain and measured error e. Hence, larger proportional gain or error makes for greater output from the proportional part. Setting the proportional gain too high causes a controller to repeatedly overshoot the setpoint, leading to oscillation.
- e = r – y
e: error,    r: user input reference,    y: actual measured output
- Proportional part = Kp * e
Kp: input gain for P – Regulator (Proportional Regulator)

```
# Proportional part
Proportional = kp * e_current
```

#### I - Integral part
- Think of the integral as a basket in which the loop stores all measured errors in E. Remember that error can be positive or negative, so sometimes error fills the basket (when positive error is added to positive error or negative error is added to negative) and sometimes it empties the basket — as when positive error is added to negative, or vice versa.
- collect all of these errors over time (take integral over time).
- Integral part = Ki * E  
  where E = E + e(current) * ∆t,         the loop stores all measured errors in E

```
# integral part
self.vars.E = self.vars.E + e_current * delta_t 
integral = ki * self.vars.E
```
#### D - Derivative part
- the derivative is looking at the rate of change of the error. The more error changes, the larger the derivative factor becomes.
- Kd have to be less than 1.
- Derivative part = Kd * ((e(current) - e(previous)) / ∆t)

```
# derivative part
if delta_t == 0:
  derivate = 0
else:
  derivate = kd * ((e_current - self.vars.e_previous)/delta_t)
```
#### u - System input signal (PID controller output signal)
```
u = Proportional + integral + derivate    # u : input signal

if u >= 0:
  throttle_output = u
  brake_output    = 0
elif u < 0:
  throttle_output = 0
  brake_output    = -u
```
