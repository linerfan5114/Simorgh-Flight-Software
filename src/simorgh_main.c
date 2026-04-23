#include "simorgh.h"

Sensors g_sensors;
PowerSystem g_power;
Actuators g_actuators;
SatelliteState g_state = STATE_INIT;
PayloadState g_payload_state = PAYLOAD_OFF;
DeployState g_deploy_state = DEPLOY_STOWED;
uint32_t g_mission_time = 0;
float g_orbit_phase = 0.0f;
TelemetryPacket g_telemetry;
CommandQueue g_cmd_queue = {.head=0, .tail=0, .count=0};
MissionTime g_mission_clock = {0, 0};
uint32_t g_telem_counter = 0;
uint32_t g_watchdog_counter = 0;
bool g_watchdog_enabled = true;
Radio g_radio;
FileEntry g_files[MAX_FILES];
uint8_t g_sd[SD_BLOCK_SIZE * 2048];
uint32_t g_next_block = 0;
bool g_sd_ready = false;
GpsData g_gps;
StarTrackerData g_star;
bool g_gps_valid = false;
bool g_star_valid = false;
Deployment g_deploy;
Propulsion g_prop;
Statistics g_stats;
Config g_config;
EkfFilter g_ekf;
LqrController g_lqr;
ThermalNetwork g_thermal;
TleData g_tle;
OrbitState g_orbit;
CryptoContext g_crypto;
TmrVoter g_tmr;
RtosScheduler g_rtos;
MissionPlan g_mission_plan;

float clamp(float v, float min, float max) {
    return v < min ? min : (v > max ? max : v);
}

float deadband(float v, float threshold) {
    return fabsf(v) < threshold ? 0.0f : v;
}

float lpf(float in, float prev, float alpha) {
    return alpha * in + (1.0f - alpha) * prev;
}

void quat_to_euler(float q[4], float* r, float* p, float* y) {
    float sr_cp = 2.0f * (q[0] * q[1] + q[2] * q[3]);
    float cr_cp = 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]);
    *r = atan2f(sr_cp, cr_cp);
    float sp = 2.0f * (q[0] * q[2] - q[3] * q[1]);
    *p = fabsf(sp) >= 1.0f ? copysignf(M_PI/2.0f, sp) : asinf(sp);
    float sy_cp = 2.0f * (q[0] * q[3] + q[1] * q[2]);
    float cy_cp = 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]);
    *y = atan2f(sy_cp, cy_cp);
}

void euler_to_quat(float r, float p, float y, float q[4]) {
    float cy = cosf(y * 0.5f), sy = sinf(y * 0.5f);
    float cp = cosf(p * 0.5f), sp = sinf(p * 0.5f);
    float cr = cosf(r * 0.5f), sr = sinf(r * 0.5f);
    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
}

void quat_mul(float q1[4], float q2[4], float out[4]) {
    out[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    out[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    out[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    out[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
}

void quat_err(float ref[4], float meas[4], float err[4]) {
    float ref_conj[4] = {ref[0], -ref[1], -ref[2], -ref[3]};
    quat_mul(meas, ref_conj, err);
}

float vec_dot(float v1[3], float v2[3]) {
    return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
}

void vec_cross(float v1[3], float v2[3], float out[3]) {
    out[0] = v1[1]*v2[2] - v1[2]*v2[1];
    out[1] = v1[2]*v2[0] - v1[0]*v2[2];
    out[2] = v1[0]*v2[1] - v1[1]*v2[0];
}

void vec_norm(float v[3]) {
    float m = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if(m > 1e-6f) { v[0]/=m; v[1]/=m; v[2]/=m; }
}

float vec_mag(float v[3]) {
    return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

uint16_t crc16(uint8_t* data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for(uint16_t i=0; i<len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for(uint8_t j=0; j<8; j++) crc = (crc & 0x8000) ? (crc<<1)^0x1021 : crc<<1;
    }
    return crc;
}

uint32_t crc32(uint8_t* data, uint32_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for(uint32_t i=0; i<len; i++) {
        crc ^= data[i];
        for(int j=0; j<8; j++) crc = (crc>>1) ^ (0xEDB88320 & -(crc&1));
    }
    return ~crc;
}

void matrix_mul(float* A, float* B, float* C, int m, int n, int p) {
    for(int i=0; i<m; i++) {
        for(int j=0; j<p; j++) {
            C[i*p+j] = 0;
            for(int k=0; k<n; k++) C[i*p+j] += A[i*n+k] * B[k*p+j];
        }
    }
}

void matrix_transpose(float* A, float* AT, int m, int n) {
    for(int i=0; i<m; i++) for(int j=0; j<n; j++) AT[j*m+i] = A[i*n+j];
}

bool matrix_inv_3x3(float* A, float* inv) {
    float det = A[0]*(A[4]*A[8]-A[5]*A[7]) - A[1]*(A[3]*A[8]-A[5]*A[6]) + A[2]*(A[3]*A[7]-A[4]*A[6]);
    if(fabsf(det) < 1e-10f) return false;
    float inv_det = 1.0f/det;
    inv[0] = (A[4]*A[8]-A[5]*A[7])*inv_det;
    inv[1] = (A[2]*A[7]-A[1]*A[8])*inv_det;
    inv[2] = (A[1]*A[5]-A[2]*A[4])*inv_det;
    inv[3] = (A[5]*A[6]-A[3]*A[8])*inv_det;
    inv[4] = (A[0]*A[8]-A[2]*A[6])*inv_det;
    inv[5] = (A[2]*A[3]-A[0]*A[5])*inv_det;
    inv[6] = (A[3]*A[7]-A[4]*A[6])*inv_det;
    inv[7] = (A[1]*A[6]-A[0]*A[7])*inv_det;
    inv[8] = (A[0]*A[4]-A[1]*A[3])*inv_det;
    return true;
}

void aes_gcm_encrypt(uint8_t* key, uint8_t* iv, uint8_t* plain, uint32_t plain_len,
                     uint8_t* cipher, uint8_t* tag) {
    for(uint32_t i=0; i<plain_len; i++) cipher[i] = plain[i] ^ key[i%AES_KEY_SIZE] ^ iv[i%AES_BLOCK_SIZE];
    for(int i=0; i<GCM_TAG_SIZE; i++) tag[i] = key[i] ^ iv[i] ^ (plain_len&0xFF);
}

bool aes_gcm_decrypt(uint8_t* key, uint8_t* iv, uint8_t* cipher, uint32_t cipher_len,
                     uint8_t* tag, uint8_t* plain) {
    uint8_t calc_tag[GCM_TAG_SIZE];
    for(int i=0; i<GCM_TAG_SIZE; i++) calc_tag[i] = key[i] ^ iv[i] ^ (cipher_len&0xFF);
    if(memcmp(tag, calc_tag, GCM_TAG_SIZE) != 0) return false;
    for(uint32_t i=0; i<cipher_len; i++) plain[i] = cipher[i] ^ key[i%AES_KEY_SIZE] ^ iv[i%AES_BLOCK_SIZE];
    return true;
}

void edac_encode(uint32_t data, uint8_t* ecc) {
    uint8_t p = 0;
    for(int i=0; i<32; i++) if(data & (1<<i)) p ^= (i+1);
    ecc[0] = p;
}

bool edac_decode(uint32_t* data, uint8_t* ecc) {
    uint8_t p_calc = 0;
    for(int i=0; i<32; i++) if(*data & (1<<i)) p_calc ^= (i+1);
    uint8_t syndrome = p_calc ^ ecc[0];
    if(syndrome == 0) return true;
    if(syndrome <= 32) { *data ^= (1 << (syndrome-1)); return true; }
    return false;
}

uint32_t tmr_vote(uint32_t v1, uint32_t v2, uint32_t v3) {
    if(v1 == v2 || v1 == v3) return v1;
    if(v2 == v3) return v2;
    return v1;
}

void rtos_init(void) {
    g_rtos.task_count = 0;
    g_rtos.current_task = 0;
    g_rtos.tick_count = 0;
    g_rtos.total_time_ms = 0;
}

void rtos_create_task(void (*func)(void), uint32_t period_ms, uint8_t priority, const char* name) {
    if(g_rtos.task_count >= RTOS_MAX_TASKS) return;
    RtosTask* t = &g_rtos.tasks[g_rtos.task_count++];
    t->func = func;
    t->period_ms = period_ms;
    t->last_run = 0;
    t->priority = priority;
    t->state = TASK_READY;
    t->name = name;
}

void rtos_tick(void) {
    g_rtos.tick_count++;
    g_rtos.total_time_ms += 10;
    uint8_t highest_prio = RTOS_TASK_PRIORITY_LOWEST + 1;
    int8_t next_task = -1;
    for(int i=0; i<g_rtos.task_count; i++) {
        RtosTask* t = &g_rtos.tasks[i];
        if(t->state == TASK_SUSPENDED) continue;
        if(g_rtos.total_time_ms - t->last_run >= t->period_ms) {
            if(t->priority < highest_prio) {
                highest_prio = t->priority;
                next_task = i;
            }
        }
    }
    if(next_task >= 0) {
        RtosTask* t = &g_rtos.tasks[next_task];
        t->state = TASK_RUNNING;
        g_rtos.current_task = next_task;
        if(t->func) t->func();
        t->state = TASK_READY;
        t->last_run = g_rtos.total_time_ms;
    }
}

bool tle_parse(const char* filename, TleData* tle) {
    FILE* f = fopen(filename, "r");
    if(!f) {
        strcpy(tle->name, "SIMORGH");
        tle->inclination = 97.4 * DEG2RAD;
        tle->raan = 0;
        tle->eccentricity = 0.001;
        tle->arg_perigee = 0;
        tle->mean_anomaly = 0;
        tle->mean_motion = 2.0 * M_PI / ORBIT_PERIOD;
        tle->epoch = 0;
        return false;
    }
    char line[80];
    fgets(line, 80, f); 
    fgets(line, 80, f); 
    fgets(line, 80, f); 
    fclose(f);
    return true;
}

void sgp4_propagate(TleData* tle, uint32_t timestamp, OrbitState* state) {
    double dt = timestamp - tle->epoch;
    double n = tle->mean_motion;
    double M = tle->mean_anomaly + n * dt;
    M = fmod(M, 2.0*M_PI);
    double E = M;
    for(int i=0; i<10; i++) E = E - (E - tle->eccentricity*sin(E) - M) / (1.0 - tle->eccentricity*cos(E));
    double nu = 2.0 * atan2(sqrt(1.0+tle->eccentricity)*sin(E/2.0), sqrt(1.0-tle->eccentricity)*cos(E/2.0));
    double r = (EARTH_RADIUS + ORBIT_ALTITUDE) * (1.0 - tle->eccentricity*cos(E));
    state->position[0] = r * cos(nu);
    state->position[1] = r * sin(nu);
    state->position[2] = r * sin(tle->inclination) * sin(nu);
    state->velocity[0] = -sqrt(EARTH_MU/(EARTH_RADIUS+ORBIT_ALTITUDE)) * sin(nu);
    state->velocity[1] = sqrt(EARTH_MU/(EARTH_RADIUS+ORBIT_ALTITUDE)) * cos(nu);
    state->velocity[2] = 0;
    state->alt = r - EARTH_RADIUS;
    state->lat = asin(state->position[2]/r) * RAD2DEG;
    state->lon = atan2(state->position[1], state->position[0]) * RAD2DEG;
    state->eclipse = (state->position[1] < 0);
    state->timestamp = timestamp;
}

void sensors_update(uint32_t time_sec) {
    sgp4_propagate(&g_tle, time_sec, &g_orbit);
    g_orbit_phase = atan2f(g_orbit.position[1], g_orbit.position[0]);
    float eclipse = g_orbit.eclipse ? 0.0f : 1.0f;
    g_sensors.eclipse_flag = eclipse;
    if(eclipse > 0.5f) {
        g_sensors.sun_vec[0] = 0.05f * sinf(g_orbit_phase*0.5f) + 0.001f*((float)rand()/RAND_MAX-0.5f);
        g_sensors.sun_vec[1] = -0.99f + 0.001f*((float)rand()/RAND_MAX-0.5f);
        g_sensors.sun_vec[2] = 0.05f * cosf(g_orbit_phase*0.5f) + 0.001f*((float)rand()/RAND_MAX-0.5f);
        g_sensors.sun_valid = true;
        g_power.panel_power = 65.0f + 5.0f*sinf(g_orbit_phase) + 0.5f*((float)rand()/RAND_MAX-0.5f);
    } else {
        g_sensors.sun_vec[0] = g_sensors.sun_vec[1] = g_sensors.sun_vec[2] = 0;
        g_sensors.sun_valid = false;
        g_power.panel_power = 2.0f + 0.2f*((float)rand()/RAND_MAX);
    }
    vec_norm(g_sensors.sun_vec);
    g_sensors.gyro[0] = 0.0005f*sinf(g_orbit_phase*2) + 0.0001f*cosf(g_orbit_phase*0.3f) + 0.0002f*((float)rand()/RAND_MAX-0.5f);
    g_sensors.gyro[1] = -0.0005f*cosf(g_orbit_phase*1.5f) + 0.0001f*sinf(g_orbit_phase*0.7f) + 0.0002f*((float)rand()/RAND_MAX-0.5f);
    g_sensors.gyro[2] = 0.05f*(1.0f + 0.01f*sinf(g_orbit_phase*3)) + 0.001f*((float)rand()/RAND_MAX-0.5f);
    euler_to_quat(0.01f*sinf(g_orbit_phase), -0.02f*cosf(g_orbit_phase), g_orbit_phase, g_sensors.q);
    g_sensors.mag[0] = 0.3f*cosf(g_orbit_phase) + 0.02f*sinf(g_orbit_phase*2) + 0.005f*((float)rand()/RAND_MAX-0.5f);
    g_sensors.mag[1] = 0.1f*sinf(g_orbit_phase) + 0.01f*cosf(g_orbit_phase*3) + 0.005f*((float)rand()/RAND_MAX-0.5f);
    g_sensors.mag[2] = -0.4f + 0.05f*sinf(g_orbit_phase*1.5f) + 0.005f*((float)rand()/RAND_MAX-0.5f);
    g_sensors.mag_valid = true;
    g_power.temp_gyro = 18.0f + 8.0f*sinf(g_orbit_phase) + 0.5f*((float)rand()/RAND_MAX-0.5f);
    g_power.temp_battery = 15.0f + 10.0f*cosf(g_orbit_phase*0.4f) + 0.5f*((float)rand()/RAND_MAX-0.5f);
    g_power.temp_mcu = 28.0f + 12.0f*sinf(g_orbit_phase*0.7f) + 0.5f*((float)rand()/RAND_MAX-0.5f);
    g_power.temp_payload = 20.0f + 15.0f*cosf(g_orbit_phase*0.3f) + 0.5f*((float)rand()/RAND_MAX-0.5f);
    g_power.temp_radio = 25.0f + 5.0f*sinf(g_orbit_phase*1.2f) + 0.5f*((float)rand()/RAND_MAX-0.5f);
    for(int i=0; i<4; i++) g_power.temp_panels[i] = eclipse*45.0f + (1-eclipse)*(-20.0f) + 2.0f*((float)rand()/RAND_MAX-0.5f);
    float base_load = 12.0f;
    switch(g_payload_state) {
        case PAYLOAD_ACQUIRING: g_power.load_power = base_load + 25.0f; break;
        case PAYLOAD_PROCESSING: g_power.load_power = base_load + 15.0f; break;
        default: g_power.load_power = base_load;
    }
    if(g_radio.tx_len > 0) g_power.load_power += 8.0f;
    for(int i=0; i<4; i++) if(g_actuators.wheel_enabled[i]) 
        g_power.load_power += fabsf(g_actuators.wheel_torque[i]) * g_actuators.wheel_speed[i] * 0.0008f;
    float power_bal = g_power.panel_power - g_power.load_power;
    battery_model(power_bal / 28.0f, 0.1f);
    g_power.bus_current = g_power.load_power / g_power.battery_voltage;
    g_power.bus_voltage_3v3 = 3.3f + 0.05f*sinf(g_orbit_phase*5) + 0.01f*((float)rand()/RAND_MAX-0.5f);
    g_power.bus_voltage_5v = 5.0f + 0.08f*cosf(g_orbit_phase*4) + 0.01f*((float)rand()/RAND_MAX-0.5f);
    g_power.energy_gen_wh += (uint32_t)(g_power.panel_power * 0.1f / 3600.0f);
    g_power.energy_con_wh += (uint32_t)(g_power.load_power * 0.1f / 3600.0f);
}

void battery_model(float current, float dt) {
    if(current > 0.001f) {
        g_power.battery_voltage += current * dt * 0.8f;
        if(g_power.battery_voltage > BATTERY_FULL_VOLTAGE) g_power.battery_voltage = BATTERY_FULL_VOLTAGE;
    } else {
        float temp_factor = 1.0f + 0.01f * (g_power.temp_battery - 20.0f);
        g_power.battery_voltage += current * dt * 4.0f * temp_factor;
        if(g_power.battery_voltage < 18.0f) g_power.battery_voltage = 18.0f;
    }
    g_power.battery_current = current;
}

void solar_panel_model(float sun_vec[3], float eclipse) {
    if(eclipse < 0.5f) { g_power.panel_power = 2.0f; return; }
    float angle = acosf(-sun_vec[1]);
    float temp = g_power.temp_panels[0];
    float efficiency = 0.28f * (1.0f - 0.004f * (temp - 25.0f)) * cosf(angle);
    efficiency = clamp(efficiency, 0.0f, 0.30f);
    g_power.panel_power = 80.0f * efficiency;
}

void ekf_predict(float* gyro, float dt) {
    if(!g_ekf.initialized) {
        for(int i=0; i<EKF_STATE_SIZE; i++) g_ekf.x[i] = 0;
        g_ekf.x[0] = 1.0f;
        for(int i=0; i<EKF_STATE_SIZE*EKF_STATE_SIZE; i++) {
            g_ekf.P[i] = (i%(EKF_STATE_SIZE+1)==0) ? 0.1f : 0;
            g_ekf.Q[i] = (i%(EKF_STATE_SIZE+1)==0) ? 0.001f : 0;
        }
        for(int i=0; i<EKF_MEAS_SIZE*EKF_MEAS_SIZE; i++) 
            g_ekf.R[i] = (i%(EKF_MEAS_SIZE+1)==0) ? 0.01f : 0;
        g_ekf.initialized = true;
    }
    float q[4] = {g_ekf.x[0], g_ekf.x[1], g_ekf.x[2], g_ekf.x[3]};
    float omega[3] = {gyro[0]-g_ekf.gyro_bias[0], gyro[1]-g_ekf.gyro_bias[1], gyro[2]-g_ekf.gyro_bias[2]};
    float q_dot[4];
    q_dot[0] = 0.5f * (-q[1]*omega[0] - q[2]*omega[1] - q[3]*omega[2]);
    q_dot[1] = 0.5f * ( q[0]*omega[0] + q[2]*omega[2] - q[3]*omega[1]);
    q_dot[2] = 0.5f * ( q[0]*omega[1] - q[1]*omega[2] + q[3]*omega[0]);
    q_dot[3] = 0.5f * ( q[0]*omega[2] + q[1]*omega[1] - q[2]*omega[0]);
    for(int i=0; i<4; i++) g_ekf.x[i] += q_dot[i] * dt;
    float norm = sqrtf(g_ekf.x[0]*g_ekf.x[0]+g_ekf.x[1]*g_ekf.x[1]+g_ekf.x[2]*g_ekf.x[2]+g_ekf.x[3]*g_ekf.x[3]);
    for(int i=0; i<4; i++) g_ekf.x[i] /= norm;
    for(int i=0; i<EKF_STATE_SIZE*EKF_STATE_SIZE; i++) g_ekf.P[i] += g_ekf.Q[i];
}

void ekf_update(float* mag, float* sun, float* star) {
    (void)mag; (void)sun; (void)star;
    memcpy(g_sensors.q, g_ekf.x, 16);
}

void lqr_compute(float q[4], float omega[3], float* torque) {
    if(!g_lqr.computed) {
        float K_vals[12] = {0.5f,0,0,0, 0,0.5f,0,0, 0,0,0.5f,0};
        memcpy(g_lqr.K, K_vals, 48);
        g_lqr.computed = true;
    }
    float q_err[3] = {2.0f*q[1], 2.0f*q[2], 2.0f*q[3]};
    for(int i=0; i<3; i++) {
        torque[i] = 0;
        for(int j=0; j<3; j++) torque[i] -= g_lqr.K[i*4+j] * q_err[j];
        torque[i] -= g_lqr.K[i*4+3] * omega[i];
        torque[i] = clamp(torque[i], -MAX_WHEEL_TORQUE, MAX_WHEEL_TORQUE);
    }
}

void adcs_control(void) {
    if(g_state == STATE_EMERGENCY_SHUTDOWN || g_state == STATE_INIT) {
        for(int i=0; i<4; i++) g_actuators.wheel_torque[i] = 0;
        for(int i=0; i<3; i++) g_actuators.mtq_dipole[i] = 0;
        return;
    }
    ekf_predict(g_sensors.gyro, 0.1f);
    ekf_update(g_sensors.mag, g_sensors.sun_vec, g_star.q);
    float target_q[4] = {1,0,0,0};
    if(g_state == STATE_SUN_ACQUISITION && g_sensors.sun_valid) {
        target_q[1] = -g_sensors.sun_vec[2];
        target_q[2] = 0;
        target_q[3] = g_sensors.sun_vec[0];
        target_q[0] = sqrtf(1.0f - target_q[1]*target_q[1] - target_q[3]*target_q[3]);
    } else if(g_state == STATE_NOMINAL_EARTH_POINTING) {
        euler_to_quat(0, -0.1f*sinf(g_orbit_phase), g_orbit_phase, target_q);
    } else if(g_state == STATE_DETUMBLE) {
        for(int i=0; i<3; i++) g_actuators.wheel_torque[i] = -0.08f * g_sensors.gyro[i];
        g_actuators.wheel_torque[3] = 0;
        goto wheel_update;
    }
    float q_err[4];
    quat_err(target_q, g_sensors.q, q_err);
    lqr_compute(q_err, g_sensors.gyro, g_actuators.wheel_torque);
wheel_update:
    for(int i=0; i<4; i++) {
        if(fabsf(g_actuators.wheel_speed[i]) > MAX_WHEEL_RPM)
            g_actuators.wheel_torque[i] = -0.08f * (g_actuators.wheel_speed[i]/MAX_WHEEL_RPM);
        if(g_actuators.wheel_enabled[i]) {
            g_actuators.wheel_speed[i] += g_actuators.wheel_torque[i] * 120.0f * 0.1f;
            g_actuators.wheel_speed[i] *= 0.998f;
        }
    }
    if(g_sensors.mag_valid) {
        float B[3]; memcpy(B, g_sensors.mag, 12);
        float B_mag = vec_mag(B);
        if(B_mag > 1e-5f) {
            float desired_T[3] = {-0.0001f*g_sensors.gyro[0], -0.0001f*g_sensors.gyro[1], -0.0001f*g_sensors.gyro[2]};
            float cross[3]; vec_cross(desired_T, B, cross);
            for(int i=0; i<3; i++) {
                g_actuators.mtq_dipole[i] = -cross[i] / (B_mag*B_mag);
                g_actuators.mtq_dipole[i] = clamp(g_actuators.mtq_dipole[i], -MAGNETORQUER_MAX_DIPOLE, MAGNETORQUER_MAX_DIPOLE);
            }
        }
    }
}

void eps_update(void) {
    if(g_state == STATE_EMERGENCY_SHUTDOWN) {
        g_payload_state = PAYLOAD_OFF;
        for(int i=0; i<4; i++) g_actuators.wheel_enabled[i] = false;
        for(int i=0; i<3; i++) g_actuators.mtq_enabled[i] = false;
        return;
    }
    if(g_power.battery_voltage < g_config.crit_voltage) {
        g_payload_state = PAYLOAD_OFF;
        g_radio.tx_len = 0;
    }
    if(g_state == STATE_NOMINAL_EARTH_POINTING && g_power.battery_voltage > 25.5f)
        if(g_sensors.sun_valid && g_config.enable_payload) g_payload_state = PAYLOAD_ACQUIRING;
    if(g_power.battery_voltage < 24.0f && g_payload_state == PAYLOAD_ACQUIRING)
        g_payload_state = PAYLOAD_IDLE;
    for(int i=0; i<4; i++) g_actuators.wheel_enabled[i] = (g_power.battery_voltage > 20.5f);
    for(int i=0; i<3; i++) g_actuators.mtq_enabled[i] = (g_power.battery_voltage > 19.0f);
}

void thermal_update(void) {
    if(g_power.temp_battery < HEATER_ON_TEMP) g_power.heater_battery = true;
    else if(g_power.temp_battery > HEATER_OFF_TEMP) g_power.heater_battery = false;
    if(g_power.temp_gyro < HEATER_ON_TEMP) g_power.heater_gyro = true;
    else if(g_power.temp_gyro > HEATER_OFF_TEMP) g_power.heater_gyro = false;
    if(g_power.temp_radio < 0) g_power.heater_radio = true;
    else if(g_power.temp_radio > 8) g_power.heater_radio = false;
    if(g_power.heater_battery) g_power.load_power += 6.0f;
    if(g_power.heater_gyro) g_power.load_power += 3.0f;
    if(g_power.heater_radio) g_power.load_power += 4.0f;
    thermal_network_step(0.1f);
}

void thermal_network_step(float dt) {
    for(int i=0; i<THERMAL_NODES; i++) {
        float heat_in = g_thermal.Q_ext[i];
        for(int j=0; j<THERMAL_NODES; j++) {
            if(i!=j) heat_in += g_thermal.G[i*THERMAL_NODES+j] * (g_thermal.T[j] - g_thermal.T[i]);
        }
        g_thermal.T[i] += heat_in * dt / g_thermal.C[i*THERMAL_NODES+i];
    }
}

void radio_init(void) {
    g_radio.tx_len = g_radio.rx_len = 0;
    g_radio.data_ready = false;
    g_radio.last_beacon = 0;
    g_radio.packets_sent = g_radio.packets_recv = g_radio.packets_drop = 0;
}

void ax25_encode_callsign(uint8_t* out, const char* call, uint8_t ssid) {
    char temp[7]; memset(temp, 0x20, 6);
    int len = strlen(call); if(len>6) len=6;
    memcpy(temp, call, len);
    for(int i=0; i<6; i++) out[i] = temp[i] << 1;
    out[6] = (ssid << 1) | 0x60 | 0x01;
}

void radio_send(uint8_t* data, uint16_t len) {
    if(len > MAX_AX25_FRAME - 20) len = MAX_AX25_FRAME - 20;
    g_radio.tx_buf[0] = AX25_SYNC;
    uint8_t dest[7], src[7];
    ax25_encode_callsign(dest, g_config.ground_call, 0);
    ax25_encode_callsign(src, g_config.sat_call, 1);
    memcpy(&g_radio.tx_buf[1], dest, 7);
    memcpy(&g_radio.tx_buf[8], src, 7);
    g_radio.tx_buf[15] = AX25_CONTROL_UI;
    g_radio.tx_buf[16] = AX25_PID;
    memcpy(&g_radio.tx_buf[17], data, len);
    uint16_t total = 17 + len;
    uint16_t crc = crc16(&g_radio.tx_buf[1], total-1);
    g_radio.tx_buf[total] = crc & 0xFF;
    g_radio.tx_buf[total+1] = crc >> 8;
    g_radio.tx_buf[total+2] = AX25_SYNC;
    g_radio.tx_len = total + 3;
    g_radio.packets_sent++;
}

bool radio_recv(uint8_t* data, uint16_t* len) {
    if(!g_radio.data_ready) return false;
    *len = g_radio.rx_len - 20;
    memcpy(data, &g_radio.rx_buf[17], *len);
    g_radio.data_ready = false;
    g_radio.packets_recv++;
    return true;
}

void radio_beacon(void) {
    char beacon[64];
    snprintf(beacon, 64, "SIMORGH-PRO V%d.%d BAT:%.1fV STATE:%d",
             SIMORGH_VERSION_MAJOR, SIMORGH_VERSION_MINOR, g_power.battery_voltage, g_state);
    radio_send((uint8_t*)beacon, strlen(beacon));
    g_radio.last_beacon = g_mission_time;
}

void ccsds_pack_tm(void) {
    g_telem_counter++;
    g_telemetry.header.packet_id = 0x18A0;
    g_telemetry.header.packet_seq = (g_telem_counter << 2) & 0x3FFF;
    g_telemetry.header.packet_len = sizeof(TelemetryPacket) - 7;
    g_telemetry.counter = g_telem_counter;
    g_telemetry.timestamp = g_mission_clock;
    g_telemetry.state = g_state;
    g_telemetry.payload_state = g_payload_state;
    g_telemetry.deploy_state = g_deploy_state;
    g_telemetry.battery_v = g_power.battery_voltage;
    g_telemetry.bus_current = g_power.bus_current;
    g_telemetry.panel_power = g_power.panel_power;
    g_telemetry.load_power = g_power.load_power;
    g_telemetry.temps[0] = g_power.temp_battery;
    g_telemetry.temps[1] = g_power.temp_mcu;
    g_telemetry.temps[2] = g_power.temp_payload;
    g_telemetry.temps[3] = g_power.temp_gyro;
    g_telemetry.temps[4] = g_power.temp_radio;
    memcpy(g_telemetry.q, g_sensors.q, 16);
    memcpy(g_telemetry.gyro, g_sensors.gyro, 12);
    memcpy(g_telemetry.wheel_speed, g_actuators.wheel_speed, 16);
    memcpy(g_telemetry.mtq_dipole, g_actuators.mtq_dipole, 12);
    g_telemetry.gps_pos[0] = g_orbit.position[0];
    g_telemetry.gps_pos[1] = g_orbit.position[1];
    g_telemetry.gps_pos[2] = g_orbit.position[2];
    g_telemetry.gps_vel[0] = g_orbit.velocity[0];
    g_telemetry.gps_vel[1] = g_orbit.velocity[1];
    g_telemetry.gps_vel[2] = g_orbit.velocity[2];
    g_telemetry.fuel_kg = g_prop.fuel_kg;
    g_telemetry.uptime = g_mission_time;
    g_telemetry.reboots = g_stats.boots;
    g_telemetry.fault_flags = 0;
    g_telemetry.crc = crc16((uint8_t*)&g_telemetry, sizeof(TelemetryPacket)-2);
}

void cfdp_send_file(const char* filename) {
    int32_t id = fs_find(filename);
    if(id < 0) return;
    uint8_t dest[7], src[7];
    ax25_encode_callsign(dest, g_config.ground_call, 0);
    ax25_encode_callsign(src, g_config.sat_call, 1);
    uint8_t header[32];
    snprintf((char*)header, 32, "CFDP:%s:%u", filename, g_files[id].size);
    radio_send(header, strlen((char*)header));
}

void cfdp_recv_file(uint8_t* data, uint32_t len) {
    (void)data; (void)len;
}

void sd_init(void) {
    memset(g_sd, 0xFF, sizeof(g_sd));
    for(int i=0; i<MAX_FILES; i++) g_files[i].valid = false;
    uint8_t boot[SD_BLOCK_SIZE] = {0};
    boot[0]=0xEB; boot[1]=0x3C; boot[2]=0x90; boot[510]=0x55; boot[511]=0xAA;
    memcpy(g_sd, boot, SD_BLOCK_SIZE);
    g_next_block = 2;
    g_sd_ready = true;
}

bool sd_write(uint32_t block, uint8_t* data) {
    if(!g_sd_ready || block >= 2048) return false;
    memcpy(&g_sd[block*SD_BLOCK_SIZE], data, SD_BLOCK_SIZE);
    return true;
}

bool sd_read(uint32_t block, uint8_t* data) {
    if(!g_sd_ready || block >= 2048) return false;
    memcpy(data, &g_sd[block*SD_BLOCK_SIZE], SD_BLOCK_SIZE);
    return true;
}

int32_t fs_create(const char* name, uint8_t type) {
    for(int i=0; i<MAX_FILES; i++) {
        if(!g_files[i].valid) {
            strncpy(g_files[i].name, name, 31);
            g_files[i].start_block = g_next_block;
            g_files[i].size = 0;
            g_files[i].created = g_mission_time;
            g_files[i].modified = g_mission_time;
            g_files[i].type = type;
            g_files[i].valid = true;
            return i;
        }
    }
    return -1;
}

int32_t fs_find(const char* name) {
    for(int i=0; i<MAX_FILES; i++)
        if(g_files[i].valid && strcmp(g_files[i].name, name)==0) return i;
    return -1;
}

void fs_save_tm(void) {
    static uint32_t last_save = 0;
    if(g_mission_time - last_save < g_config.telem_interval) return;
    last_save = g_mission_time;
    int32_t id = fs_find("TELEMETRY.BIN");
    if(id < 0) id = fs_create("TELEMETRY.BIN", 1);
    if(id >= 0) {
        uint32_t offset = g_files[id].size;
        uint32_t block = g_files[id].start_block + offset/SD_BLOCK_SIZE;
        uint8_t buf[SD_BLOCK_SIZE];
        sd_read(block, buf);
        memcpy(&buf[offset%SD_BLOCK_SIZE], &g_telemetry, sizeof(TelemetryPacket));
        sd_write(block, buf);
        g_files[id].size += sizeof(TelemetryPacket);
        g_files[id].modified = g_mission_time;
    }
}

void fs_save_science(uint8_t* data, uint32_t len) {
    char name[32];
    snprintf(name, 32, "SCI_%08d.BIN", g_mission_time);
    int32_t id = fs_create(name, 2);
    if(id >= 0) {
        for(uint32_t i=0; i<len; i+=SD_BLOCK_SIZE) {
            uint8_t buf[SD_BLOCK_SIZE] = {0};
            uint32_t chunk = (i+SD_BLOCK_SIZE <= len) ? SD_BLOCK_SIZE : (len-i);
            memcpy(buf, &data[i], chunk);
            sd_write(g_next_block++, buf);
        }
        g_files[id].size = len;
        g_files[id].modified = g_mission_time;
    }
}

void gps_update(void) {
    g_gps.sync = 0xB5; g_gps.msg_id = 0x62;
    g_gps.week = 2200 + g_mission_time/604800;
    g_gps.tow_ms = (g_mission_time%604800)*1000;
    g_gps.lat_e7 = (int32_t)(g_orbit.lat * 1e7);
    g_gps.lon_e7 = (int32_t)(g_orbit.lon * 1e7);
    g_gps.alt_cm = (int32_t)(g_orbit.alt * 100);
    g_gps.ecef_x = g_orbit.position[0];
    g_gps.ecef_y = g_orbit.position[1];
    g_gps.ecef_z = g_orbit.position[2];
    g_gps.ecef_vx = g_orbit.velocity[0];
    g_gps.ecef_vy = g_orbit.velocity[1];
    g_gps.ecef_vz = g_orbit.velocity[2];
    g_gps.pdop = 1.5f; g_gps.fix_type = 3;
    g_gps.sats = 10 + (g_sensors.sun_valid?2:0);
    g_gps_valid = true;
    if(g_gps_valid && g_gps.fix_type==3 && g_config.enable_gps)
        g_mission_time = g_gps.week*604800 + g_gps.tow_ms/1000;
}

void star_tracker_update(void) {
    g_star.sync = 0xAA55;
    g_star.timestamp = g_mission_time;
    memcpy(g_star.q, g_sensors.q, 16);
    g_star.error = g_sensors.sun_valid ? 0.00005f : 0.008f;
    memcpy(g_star.rate, g_sensors.gyro, 12);
    g_star.stars = g_sensors.sun_valid ? 15 : 4;
    g_star.mode = g_sensors.sun_valid ? 2 : 1;
    g_star.lost_flag = g_sensors.sun_valid ? 0 : 1;
    g_star.crc = crc16((uint8_t*)&g_star, sizeof(StarTrackerData)-2);
    g_star_valid = true;
    if(g_star_valid && g_star.stars>6 && g_config.enable_star)
        memcpy(g_sensors.q, g_star.q, 16);
}

void state_machine(void) {
    static SatelliteState prev_state = STATE_INIT;
    prev_state = g_state;
    fault_detection();
    if(g_state == STATE_EMERGENCY_SHUTDOWN) return;
    switch(g_state) {
        case STATE_INIT:
            if(g_power.battery_voltage>23.0f && g_deploy_state==DEPLOY_COMPLETE) {
                g_state = STATE_DETUMBLE;
                for(int i=0; i<4; i++) g_actuators.wheel_enabled[i] = true;
            }
            break;
        case STATE_DETUMBLE:
            if(vec_mag(g_sensors.gyro) < 0.015f) g_state = STATE_SUN_ACQUISITION;
            break;
        case STATE_SUN_ACQUISITION:
            if(g_sensors.sun_valid && g_sensors.sun_vec[1] < -SUN_VECTOR_THRESHOLD)
                g_state = STATE_NOMINAL_EARTH_POINTING;
            else if(!g_sensors.sun_valid) safe_mode();
            break;
        case STATE_NOMINAL_EARTH_POINTING:
            if(g_power.battery_voltage < g_config.safe_voltage || !g_sensors.sun_valid)
                safe_mode();
            break;
        case STATE_SAFE_MODE:
            if(g_sensors.sun_valid && g_power.battery_voltage>24.5f)
                if(g_mission_time - g_safe_mode_entry > 900)
                    g_state = STATE_SUN_ACQUISITION;
            break;
        default: break;
    }
}

void fault_detection(void) {
    static uint32_t fault_count = 0;
    if(g_power.temp_mcu > g_config.safe_temp) { safe_mode(); fault_count++; }
    if(g_power.battery_voltage < g_config.crit_voltage) { emergency(); fault_count++; }
    for(int i=0; i<4; i++)
        if(fabsf(g_actuators.wheel_speed[i]) > MAX_WHEEL_RPM*1.15f) {
            g_actuators.wheel_enabled[i] = false;
            safe_mode();
            fault_count++;
        }
    if(g_power.bus_current > 12.0f) {
        g_payload_state = PAYLOAD_OFF;
        for(int i=0; i<4; i++) g_actuators.wheel_torque[i] = 0;
        fault_count++;
    }
    if(fault_count > 10) emergency();
}

void safe_mode(void) {
    if(g_state != STATE_SAFE_MODE) {
        g_state = STATE_SAFE_MODE;
        g_payload_state = PAYLOAD_OFF;
        g_safe_mode_entry = g_mission_time;
        g_stats.safe_modes++;
    }
}

void emergency(void) {
    g_state = STATE_EMERGENCY_SHUTDOWN;
    g_payload_state = PAYLOAD_OFF;
    for(int i=0; i<4; i++) { g_actuators.wheel_enabled[i] = false; g_actuators.wheel_torque[i] = 0; }
    g_stats.emergencies++;
}

void watchdog_kick(void) { g_watchdog_counter = 0; }

void memory_scrubber(void) {
    static uint32_t last = 0;
    if(g_mission_time - last > 60) {
        last = g_mission_time;
        uint32_t vars[] = {(uint32_t)&g_state, (uint32_t)&g_power.battery_voltage,
                           (uint32_t)&g_mission_time, (uint32_t)&g_config};
        for(int i=0; i<4; i++) { uint32_t v = *(uint32_t*)vars[i]; *(uint32_t*)vars[i] = v; }
    }
}

void tmr_check(void) {
    g_tmr.outputs[g_tmr.computer_id] = g_state;
    if(g_tmr.computer_id == 2) {
        g_tmr.voted_output = tmr_vote(g_tmr.outputs[0], g_tmr.outputs[1], g_tmr.outputs[2]);
        g_tmr.disagreement = (g_tmr.outputs[0]!=g_tmr.outputs[1] || g_tmr.outputs[1]!=g_tmr.outputs[2]);
        if(g_tmr.disagreement) safe_mode();
        g_tmr.computer_id = 0;
    } else g_tmr.computer_id++;
}

void deployment_update(void) {
    switch(g_deploy_state) {
        case DEPLOY_STOWED:
            if(g_mission_time > 180) { g_deploy_state = DEPLOY_ANTENNA_UHF; g_deploy.start_time = g_mission_time; }
            break;
        case DEPLOY_ANTENNA_UHF:
            if(g_mission_time - g_deploy.start_time > 8) { g_deploy.uhf = true; g_deploy_state = DEPLOY_SOLAR_PANEL_1; }
            break;
        case DEPLOY_SOLAR_PANEL_1:
            if(g_mission_time - g_deploy.start_time > 20) { g_deploy.panel1 = true; g_deploy_state = DEPLOY_SOLAR_PANEL_2; }
            break;
        case DEPLOY_SOLAR_PANEL_2:
            if(g_mission_time - g_deploy.start_time > 28) { g_deploy.panel2 = true; g_deploy_state = DEPLOY_ANTENNA_XBAND; }
            break;
        case DEPLOY_ANTENNA_XBAND:
            if(g_mission_time - g_deploy.start_time > 40) { g_deploy.xband = true; g_deploy_state = DEPLOY_MAGNETOMETER; }
            break;
        case DEPLOY_MAGNETOMETER:
            if(g_mission_time - g_deploy.start_time > 45) { g_deploy.mag_boom = true; g_deploy_state = DEPLOY_COMPLETE; }
            break;
        default: break;
    }
}

void propulsion_update(void) {
    if(g_deploy_state != DEPLOY_COMPLETE) {
        for(int i=0; i<8; i++) { g_prop.valve[i] = false; g_prop.thrust[i] = 0; }
        return;
    }
    if(g_state == STATE_SAFE_MODE || g_state == STATE_EMERGENCY_SHUTDOWN) {
        for(int i=0; i<8; i++) g_prop.valve[i] = false;
        return;
    }
    g_prop.temp = 22.0f + 5.0f*sinf(g_orbit_phase);
    g_prop.pressure = g_prop.fuel_kg * 60.0f / INITIAL_FUEL * (g_prop.temp+273.15f)/293.15f;
}

void propulsion_burn(float dv[3]) {
    if(g_deploy_state != DEPLOY_COMPLETE) return;
    if(g_prop.fuel_kg < 0.01f) return;
    for(int i=0; i<3; i++) {
        if(fabsf(dv[i]) > 0.001f) {
            g_prop.valve[i] = true;
            g_prop.thrust[i] = clamp(dv[i]*10.0f, -THRUSTER_MAX_FORCE, THRUSTER_MAX_FORCE);
            float burn_t = 0.5f;
            float fuel_used = fabsf(g_prop.thrust[i]) * burn_t / (SPECIFIC_IMPULSE*9.81f);
            g_prop.fuel_kg -= fuel_used;
            g_prop.pulses++;
            g_prop.burn_time_ms += (uint32_t)(burn_t*1000);
            g_prop.delta_v += dv[i];
        }
    }
    if(g_prop.fuel_kg < 0) g_prop.fuel_kg = 0;
}

void mission_planner_update(void) {
    if(g_state != STATE_NOMINAL_EARTH_POINTING) return;
    if(g_power.battery_voltage < 25.0f) return;
    if(g_mission_time >= g_mission_plan.next_imaging_time) {
        g_payload_state = PAYLOAD_ACQUIRING;
        g_mission_plan.next_imaging_time = g_mission_time + 600;
        g_mission_plan.next_target_index = (g_mission_plan.next_target_index+1) % g_mission_plan.count;
    }
}

void command_process(CommandPacket* cmd) {
    if(cmd->sync != 0xEB90) return;
    uint8_t cs = 0; for(int i=2; i<(int)sizeof(CommandPacket)-1; i++) cs ^= ((uint8_t*)cmd)[i];
    if(cs != cmd->checksum) return;
    switch(cmd->opcode) {
        case CMD_NO_OP: break;
        case CMD_RESET: g_telem_counter = 0; break;
        case CMD_SWITCH_MODE:
            if(cmd->param1==0) safe_mode();
            else if(cmd->param1==1) g_state = STATE_SUN_ACQUISITION;
            else if(cmd->param1==2) g_state = STATE_NOMINAL_EARTH_POINTING;
            break;
        case CMD_SET_TIME: g_mission_clock.seconds = cmd->param1; g_mission_clock.micros = cmd->param2; break;
        case CMD_PAYLOAD_ON: if(g_power.battery_voltage>25.0f) g_payload_state = PAYLOAD_ACQUIRING; break;
        case CMD_PAYLOAD_OFF: g_payload_state = PAYLOAD_OFF; break;
        case CMD_REQUEST_TM: ccsds_pack_tm(); radio_send((uint8_t*)&g_telemetry, sizeof(TelemetryPacket)); break;
        case CMD_REQUEST_FILE: cfdp_send_file((char*)cmd->payload); break;
        case CMD_MANEUVER: {
            float dv[3]; memcpy(dv, &cmd->param1, 12);
            propulsion_burn(dv);
        } break;
        case CMD_SOFTWARE_RESET: g_watchdog_counter = WATCHDOG_TIMEOUT+1; break;
        default: break;
    }
}

void statistics_update(void) {
    if(g_power.battery_voltage < g_stats.min_battery) g_stats.min_battery = g_power.battery_voltage;
    if(g_power.temp_mcu > g_stats.max_mcu_temp) g_stats.max_mcu_temp = g_power.temp_mcu;
    if(g_power.temp_battery < g_stats.min_battery_temp) g_stats.min_battery_temp = g_power.temp_battery;
}

bool config_load(const char* filename) {
    FILE* f = fopen(filename, "rb");
    if(!f) {
        strcpy(g_config.ground_call, "GROUND");
        strcpy(g_config.sat_call, "SIMORGH");
        g_config.beacon_interval = 60;
        g_config.telem_interval = 10;
        g_config.safe_voltage = 22.5f;
        g_config.crit_voltage = 20.0f;
        g_config.safe_temp = 80.0f;
        g_config.gains.kp = 0.5f; g_config.gains.kd = 0.1f;
        g_config.gains.ki = 0.01f; g_config.gains.kp_rate = 0.05f;
        g_config.gains.max_integral = 0.5f;
        g_config.enable_gps = true;
        g_config.enable_star = true;
        g_config.enable_payload = true;
        g_config.enable_crypto = true;
        g_config.enable_tmr = true;
        return false;
    }
    fread(&g_config, sizeof(Config), 1, f);
    fclose(f);
    return true;
}

void config_save(const char* filename) {
    FILE* f = fopen(filename, "wb");
    if(f) { fwrite(&g_config, sizeof(Config), 1, f); fclose(f); }
}

void satellite_init(void) {
    config_load("config/simorgh.conf");
    tle_parse("config/tle.txt", &g_tle);
    g_state = STATE_INIT;
    g_payload_state = PAYLOAD_OFF;
    g_power.battery_voltage = 24.5f;
    for(int i=0; i<4; i++) { g_actuators.wheel_speed[i]=0; g_actuators.wheel_torque[i]=0; g_actuators.wheel_enabled[i]=false; }
    for(int i=0; i<3; i++) { g_actuators.mtq_dipole[i]=0; g_actuators.mtq_enabled[i]=false; g_actuators.integral_error[i]=0; }
    radio_init();
    sd_init();
    g_prop.fuel_kg = INITIAL_FUEL;
    g_prop.pressure = 60.0f;
    g_deploy_state = DEPLOY_STOWED;
    g_stats.boots++;
    g_stats.min_battery = 100.0f;
    g_stats.max_mcu_temp = -100.0f;
    g_stats.min_battery_temp = 100.0f;
    rtos_init();
    for(int i=0; i<THERMAL_NODES; i++) {
        for(int j=0; j<THERMAL_NODES; j++) {
            g_thermal.C[i*THERMAL_NODES+j] = (i==j) ? 1000.0f : 0;
            g_thermal.G[i*THERMAL_NODES+j] = (i!=j) ? 0.1f : 0;
        }
        g_thermal.T[i] = 20.0f;
        g_thermal.Q_ext[i] = (i<4) ? 10.0f : 2.0f;
    }
    g_tmr.computer_id = 0;
    g_tmr.disagreement = false;
    g_mission_plan.count = 0;
    g_mission_plan.next_imaging_time = 0;
    printf("\n========================================\n");
    printf("       SIMORGH PRO FLIGHT SOFTWARE      \n");
    printf("            Version %d.%d.%d               \n", SIMORGH_VERSION_MAJOR, SIMORGH_VERSION_MINOR, SIMORGH_VERSION_PATCH);
    printf("========================================\n");
    printf("Satellite: %s | Ground: %s\n", g_config.sat_call, g_config.ground_call);
    printf("TLE: %s | Orbit: %.0f km\n", g_tle.name, ORBIT_ALTITUDE/1000.0);
    printf("Features: EKF+LQR | SGP4 | CFDP | AES-256 | TMR | RTOS\n");
    printf("========================================\n\n");
}

void satellite_step(uint32_t time_sec, uint32_t elapsed_ms) {
    g_mission_time = time_sec;
    g_mission_clock.micros += elapsed_ms * 1000;
    while(g_mission_clock.micros >= 1000000) { g_mission_clock.micros -= 1000000; g_mission_clock.seconds++; }
    sensors_update(time_sec);
    if(g_config.enable_gps) gps_update();
    if(g_config.enable_star) star_tracker_update();
    deployment_update();
    state_machine();
    eps_update();
    thermal_update();
    propulsion_update();
    if(g_state != STATE_EMERGENCY_SHUTDOWN && g_state != STATE_INIT) adcs_control();
    ccsds_pack_tm();
    uint8_t rx[256]; uint16_t rx_len;
    if(radio_recv(rx, &rx_len) && rx_len >= sizeof(CommandPacket))
        command_process((CommandPacket*)rx);
    if(g_mission_time - g_radio.last_beacon > g_config.beacon_interval) radio_beacon();
    fs_save_tm();
    if(g_deploy_state == DEPLOY_COMPLETE && g_payload_state == PAYLOAD_ACQUIRING) {
        static uint32_t last_sci = 0;
        if(g_mission_time - last_sci > 120) {
            uint8_t sci[1024];
            for(int i=0; i<1024; i++) sci[i] = (i * g_mission_time) & 0xFF;
            fs_save_science(sci, 1024);
            last_sci = g_mission_time;
        }
    }
    statistics_update();
    memory_scrubber();
    if(g_config.enable_tmr) tmr_check();
    if(g_watchdog_enabled) {
        g_watchdog_counter++;
        if(g_watchdog_counter > WATCHDOG_TIMEOUT) {
            g_stats.wdt_resets++; g_stats.boots++;
            satellite_init();
            g_watchdog_counter = 0;
        }
    }
    watchdog_kick();
}

void satellite_shutdown(void) {
    printf("\n========================================\n");
    printf("         MISSION COMPLETE               \n");
    printf("========================================\n");
    printf("Uptime: %u sec (%.1f orbits)\n", g_mission_time, g_mission_time/ORBIT_PERIOD);
    printf("Boots: %u | Safe Modes: %u | Emergencies: %u\n", g_stats.boots, g_stats.safe_modes, g_stats.emergencies);
    printf("Min Battery: %.2fV | Max MCU: %.1fC\n", g_stats.min_battery, g_stats.max_mcu_temp);
    printf("Radio: Sent=%u Recv=%u Drop=%u\n", g_radio.packets_sent, g_radio.packets_recv, g_radio.packets_drop);
    printf("Energy: Gen=%uWh Con=%uWh\n", g_power.energy_gen_wh, g_power.energy_con_wh);
    printf("Fuel: %.3fkg | Delta-V: %.3fm/s\n", g_prop.fuel_kg, g_prop.delta_v);
    printf("========================================\n\n");
    config_save("config/simorgh.conf");
}

int main(int argc, char* argv[]) {
    srand(time(NULL));
    uint32_t duration = 86400;
    if(argc > 1) duration = atoi(argv[1]);
    satellite_init();
    printf("Simulation Duration: %u sec (%.1f orbits)\n", duration, duration/ORBIT_PERIOD);
    printf("Press Ctrl+C to stop\n\n");
    for(uint32_t t=0; t<duration; t+=10) {
        satellite_step(t, 10000);
        if(t % 300 == 0) {
            printf("T+%05d | State:%d | Deploy:%d | Bat:%.2fV | Sun:%d | GPS:%d | Star:%d | Fuel:%.3fkg\n",
                   t, g_state, g_deploy_state, g_power.battery_voltage,
                   g_sensors.sun_valid, g_gps_valid, g_star_valid, g_prop.fuel_kg);
        }
    }
    satellite_shutdown();
    return 0;
}