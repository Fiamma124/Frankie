// firmware_twiddle_pwm.c
// Autotune (Twiddle) + PWM-space PID helpers + 200 ms tachometers
// Facu: drop this file into your firmware project and compile instead of the old tuning module.
// If you keep your original file name, copy/paste the content.

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// ===== External symbols provided by your project =====
extern QueueHandle_t q_tacometro_left;
extern QueueHandle_t q_tacometro_right;
extern void motor_set_direction(int motor_id, int dir);
extern void motor_set_speed(int motor_id, int pwm);

// Optional: if you want to persist to EEPROM, uncomment these and implement in your project
// extern void at24c32_write_log(void *i2c_port, uint16_t addr, const char *msg);
// extern void *i2c1;

// ===== Motor IDs (adjust to your project's values) =====
#ifndef MOTOR_LEFT
#define MOTOR_LEFT   (0)
#endif
#ifndef MOTOR_RIGHT
#define MOTOR_RIGHT  (1)
#endif

// ===== Control timing =====
#define TS_SEC          (0.20f) // 200 ms; match tachometer publish rate
#define TS_TICKS        (pdMS_TO_TICKS((uint32_t)(TS_SEC*1000.0f)))
#define RX_TIMEOUT_MS   (60)    // wait a bit for fresh tacho sample
#define RX_TO_TICKS     (pdMS_TO_TICKS(RX_TIMEOUT_MS))

// ===== Calibrations: PWM <-> velocity (per wheel) =====
// Start with your measured line: u ≈ a*v + b (PWM for m/s)
// Update these when you have per-wheel calibration!
#ifndef A_LEFT
#define A_LEFT   (75.0f)
#endif
#ifndef B_LEFT
#define B_LEFT   (165.0f)
#endif
#ifndef A_RIGHT
#define A_RIGHT  (75.0f)
#endif
#ifndef B_RIGHT
#define B_RIGHT  (165.0f)
#endif

// ===== Utilities =====
static inline float clampf(float x, float lo, float hi){
    return (x < lo) ? lo : (x > hi) ? hi : x;
}
static inline int clamp_pwm(float u){
    if (u < 0.0f) return 0;
    if (u > 255.0f) return 255;
    return (int)u;
}
static inline float lpf(float prev, float x, float alpha){
    // alpha in (0..1); closer to 1 = slower filter
    return alpha*prev + (1.0f - alpha)*x;
}

// ===== PID in PWM-space =====
typedef struct {
    float Kp, Ki, Kd;   // gains (dimensionless in PWM-space)
    float I_pwm;        // integral accumulator (PWM units)
    float vel_prev;     // last velocity (m/s)
    float d_filt;       // filtered derivative term (PWM units)
} pid_t;

// One control step (vref, vel in m/s). a,b from calibration (PWM/(m/s), PWM).
static inline int pid_step_pwm(pid_t *pid, float a, float b, float vref, float vel, float Ts){
    // Feed-forward from calibration
    float u_ff  = a*vref + b;

    // Error and derivative in PWM domain
    float e_pwm = a * (vref - vel);
    float d_raw = -a * (vel - pid->vel_prev) / Ts;   // derivative over measurement
    // Filter derivative to reduce noise
    pid->d_filt = lpf(pid->d_filt, d_raw, 0.7f);     // adjust alpha if needed

    // Unsaturated output (for anti-windup logic)
    float u_uns = u_ff + pid->Kp*e_pwm + pid->Ki*pid->I_pwm + pid->Kd*pid->d_filt;

    // Anti-windup: integrate only if not pushing into saturation
    int sat_hi = (u_uns >= 255.0f) && (e_pwm > 0.0f);
    int sat_lo = (u_uns <=   0.0f) && (e_pwm < 0.0f);
    if (!(sat_hi || sat_lo)) {
        pid->I_pwm += e_pwm * Ts;
        pid->I_pwm  = clampf(pid->I_pwm, -600.0f, 600.0f); // integral clamp in PWM
    }

    float u = u_ff + pid->Kp*e_pwm + pid->Ki*pid->I_pwm + pid->Kd*pid->d_filt;
    pid->vel_prev = vel;
    return clamp_pwm(u);
}

// ===== Twiddle autotuner (generic runner) =====
typedef struct {
    float Kp, Ki, Kd;
} gains_t;

// Cost function options
typedef enum {
    COST_IAE_REL = 0,   // average relative IAE per setpoint
    COST_RMSE_REL       // relative RMSE (less robust to outliers)
} cost_mode_t;

typedef struct {
    // wheel side
    int motor_id;               // MOTOR_LEFT / MOTOR_RIGHT
    float a_cal, b_cal;         // calibration a,b for this wheel
    QueueHandle_t q_tacho;      // tachometer queue
    // experiment settings
    const float *setpoints;     // m/s setpoints array
    int n_sp;                   // number of setpoints
    int warmup_cycles;          // warm-up cycles (not scored)
    int measure_cycles;         // measurement cycles
    cost_mode_t cost_mode;      // which metric to use
} trial_cfg_t;

// Run one trial with given gains; returns cost (lower = better)
static float run_trial_pwm(const trial_cfg_t *cfg, const gains_t *g){
    pid_t pid = {.Kp=g->Kp, .Ki=g->Ki, .Kd=g->Kd, .I_pwm=0, .vel_prev=0, .d_filt=0};
    float cost_total = 0.0f;

    for (int s = 0; s < cfg->n_sp; s++) {
        float vref = cfg->setpoints[s];

        // short stop between setpoints
        motor_set_speed(cfg->motor_id, 0);
        vTaskDelay(pdMS_TO_TICKS(200));

        TickType_t last = xTaskGetTickCount();
        float vel = 0.0f, vtmp = 0.0f;

        // Warm-up
        for (int n = 0; n < cfg->warmup_cycles; n++) {
            if (xQueueReceive(cfg->q_tacho, &vtmp, RX_TO_TICKS) == pdTRUE) vel = vtmp;
            int u = pid_step_pwm(&pid, cfg->a_cal, cfg->b_cal, vref, vel, TS_SEC);
            motor_set_speed(cfg->motor_id, u);
            vTaskDelayUntil(&last, TS_TICKS);
        }

        // Measurement
        float accumulator = 0.0f;
        for (int n = 0; n < cfg->measure_cycles; n++) {
            if (xQueueReceive(cfg->q_tacho, &vtmp, RX_TO_TICKS) == pdTRUE) vel = vtmp;
            int u = pid_step_pwm(&pid, cfg->a_cal, cfg->b_cal, vref, vel, TS_SEC);
            motor_set_speed(cfg->motor_id, u);

            float e = vref - vel;
            if (cfg->cost_mode == COST_IAE_REL) {
                accumulator += fabsf(e) / fmaxf(vref, 0.05f);
            } else { // COST_RMSE_REL
                float denom = vref*vref + 1e-3f;
                accumulator += (e*e)/denom;
            }
            vTaskDelayUntil(&last, TS_TICKS);
        }

        float sp_cost;
        if (cfg->cost_mode == COST_IAE_REL) {
            sp_cost = accumulator / (float)cfg->measure_cycles;
        } else { // RMSE
            sp_cost = sqrtf(accumulator / (float)cfg->measure_cycles);
        }
        cost_total += sp_cost;
    }

    // stop at the end for safety
    motor_set_speed(cfg->motor_id, 0);
    vTaskDelay(pdMS_TO_TICKS(300));

    // average across setpoints
    return cost_total / (float)cfg->n_sp;
}

// ===== Twiddle task (left) =====
void task_tune_left_twiddle(void *params){
    // Setpoints
    static const float SP[] = {0.20f, 0.60f, 1.00f};
    trial_cfg_t cfg = {
        .motor_id = MOTOR_LEFT,
        .a_cal = A_LEFT, .b_cal = B_LEFT,
        .q_tacho = q_tacometro_left,
        .setpoints = SP, .n_sp = (int)(sizeof(SP)/sizeof(SP[0])),
        .warmup_cycles = 6,
        .measure_cycles = 12,
        .cost_mode = COST_IAE_REL
    };

    gains_t p = {.Kp=1.8f, .Ki=0.05f, .Kd=0.04f};        // seeds (PWM-space)
    float   dp[3] = {0.25f, 0.015f, 0.010f};            // initial steps
    const float tol = 0.04f;                             // stop threshold on sum(dp)
    const int   max_iters = 40;

    float best = run_trial_pwm(&cfg, &p);
    printf("[Twiddle L] init  Kp=%.3f Ki=%.4f Kd=%.4f  cost=%.4f\n", p.Kp, p.Ki, p.Kd, best);

    for (int iter=1; (dp[0]+dp[1]+dp[2] > tol) && (iter<=max_iters); iter++){
        float *p_arr[3] = {&p.Kp, &p.Ki, &p.Kd};
        for (int i=0; i<3; i++){
            *p_arr[i] += dp[i];
            float err = run_trial_pwm(&cfg, &p);
            if (err < best){
                best = err; dp[i] *= 1.10f;
                printf("[Twiddle L] iter=%d param=%d  improve+  p=%.5f  cost=%.5f  dp=%.5f\n",
                       iter, i, *p_arr[i], best, dp[i]);
            } else {
                *p_arr[i] -= 2.0f*dp[i];
                err = run_trial_pwm(&cfg, &p);
                if (err < best){
                    best = err; dp[i] *= 1.10f;
                    printf("[Twiddle L] iter=%d param=%d  improve-  p=%.5f  cost=%.5f  dp=%.5f\n",
                           iter, i, *p_arr[i], best, dp[i]);
                } else {
                    *p_arr[i] += dp[i];
                    dp[i] *= 0.90f;
                    printf("[Twiddle L] iter=%d param=%d  nochange  p=%.5f  cost=%.5f  dp->%.5f\n",
                           iter, i, *p_arr[i], best, dp[i]);
                }
            }
        }
        printf("[Twiddle L] iter=%d  Kp=%.3f Ki=%.4f Kd=%.4f  best=%.4f  sum(dp)=%.4f\n",
               iter, p.Kp, p.Ki, p.Kd, best, (dp[0]+dp[1]+dp[2]));
    }

    printf("Twiddle L DONE  Kp=%.3f Ki=%.4f Kd=%.4f  best=%.4f\n", p.Kp, p.Ki, p.Kd, best);

    // Optionally persist per your API:
    // char buf[64]; snprintf(buf,sizeof(buf),"PIDL:%.4f,%.4f,%.4f", p.Kp,p.Ki,p.Kd);
    // at24c32_write_log(i2c1, 0x100, buf);

    motor_set_speed(MOTOR_LEFT, 0);
    vTaskDelete(NULL);
}

// ===== Twiddle task (right) =====
void task_tune_right_twiddle(void *params){
    static const float SP[] = {0.20f, 0.60f, 1.00f};
    trial_cfg_t cfg = {
        .motor_id = MOTOR_RIGHT,
        .a_cal = A_RIGHT, .b_cal = B_RIGHT,
        .q_tacho = q_tacometro_right,
        .setpoints = SP, .n_sp = (int)(sizeof(SP)/sizeof(SP[0])),
        .warmup_cycles = 6,
        .measure_cycles = 12,
        .cost_mode = COST_IAE_REL
    };

    gains_t p = {.Kp=1.8f, .Ki=0.05f, .Kd=0.04f};
    float   dp[3] = {0.25f, 0.015f, 0.010f};
    const float tol = 0.04f;
    const int   max_iters = 40;

    float best = run_trial_pwm(&cfg, &p);
    printf("[Twiddle R] init  Kp=%.3f Ki=%.4f Kd=%.4f  cost=%.4f\n", p.Kp, p.Ki, p.Kd, best);

    for (int iter=1; (dp[0]+dp[1]+dp[2] > tol) && (iter<=max_iters); iter++){
        float *p_arr[3] = {&p.Kp, &p.Ki, &p.Kd};
        for (int i=0; i<3; i++){
            *p_arr[i] += dp[i];
            float err = run_trial_pwm(&cfg, &p);
            if (err < best){
                best = err; dp[i] *= 1.10f;
                printf("[Twiddle R] iter=%d param=%d  improve+  p=%.5f  cost=%.5f  dp=%.5f\n",
                       iter, i, *p_arr[i], best, dp[i]);
            } else {
                *p_arr[i] -= 2.0f*dp[i];
                err = run_trial_pwm(&cfg, &p);
                if (err < best){
                    best = err; dp[i] *= 1.10f;
                    printf("[Twiddle R] iter=%d param=%d  improve-  p=%.5f  cost=%.5f  dp=%.5f\n",
                           iter, i, *p_arr[i], best, dp[i]);
                } else {
                    *p_arr[i] += dp[i];
                    dp[i] *= 0.90f;
                    printf("[Twiddle R] iter=%d param=%d  nochange  p=%.5f  cost=%.5f  dp->%.5f\n",
                           iter, i, *p_arr[i], best, dp[i]);
                }
            }
        }
        printf("[Twiddle R] iter=%d  Kp=%.3f Ki=%.4f Kd=%.4f  best=%.4f  sum(dp)=%.4f\n",
               iter, p.Kp, p.Ki, p.Kd, best, (dp[0]+dp[1]+dp[2]));
    }

    printf("Twiddle R DONE  Kp=%.3f Ki=%.4f Kd=%.4f  best=%.4f\n", p.Kp, p.Ki, p.Kd, best);

    // Optionally persist:
    // char buf[64]; snprintf(buf,sizeof(buf),"PIDR:%.4f,%.4f,%.4f", p.Kp,p.Ki,p.Kd);
    // at24c32_write_log(i2c1, 0x140, buf);

    motor_set_speed(MOTOR_RIGHT, 0);
    vTaskDelete(NULL);
}

// ===== Tachometer tasks (200 ms), clean queue usage =====
// Counts edges and publishes velocity (m/s) every 200 ms using xQueueOverwrite.
// NOTE: You must provide gpio_get(), IN_PIN_* and wheel radius R_WHEEL in your project.
extern int gpio_get(int pin);
#ifndef PI
#define PI 3.14159265358979323846f
#endif
extern float R_WHEEL; // wheel radius in meters
extern int IN_PIN_TACOMETRO_LEFT;
extern int IN_PIN_TACOMETRO_RIGHT;

// Pulses per revolution (update to your encoder spec)
#ifndef PULSES_PER_REV
#define PULSES_PER_REV (20) // example: 20 rising edges per turn
#endif

static void tacho_publish_velocity(QueueHandle_t q, int pin){
    static int estado_anterior = 0;
    static int counter = 0;

    int estado_actual = gpio_get(pin);
    if (estado_actual != estado_anterior){
        estado_anterior = estado_actual;
        if (estado_actual == 1) counter++;
    }

    static TickType_t last_pub = 0;
    TickType_t now = xTaskGetTickCount();
    if ((now - last_pub) >= TS_TICKS){ // publish every 200 ms
        // revolutions in the window
        float vueltas = ((float)counter) / (float)PULSES_PER_REV;
        // distance per revolution
        float vel = (vueltas * 2.0f * PI * R_WHEEL) / TS_SEC; // m/s over 200 ms window
        xQueueOverwrite(q, &vel);
        counter = 0;
        last_pub = now;
    }
}

void task_tacometro_left(void *params){
    // Optional: init state
    (void)params;
    for(;;){
        tacho_publish_velocity(q_tacometro_left, IN_PIN_TACOMETRO_LEFT);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void task_tacometro_right(void *params){
    (void)params;
    for(;;){
        tacho_publish_velocity(q_tacometro_right, IN_PIN_TACOMETRO_RIGHT);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
