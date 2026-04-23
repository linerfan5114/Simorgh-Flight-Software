#ifndef SIMORGH_PRO_H
#define SIMORGH_PRO_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#define SIMORGH_VERSION_MAJOR 3
#define SIMORGH_VERSION_MINOR 0
#define SIMORGH_VERSION_PATCH 0

#define M_PI 3.14159265358979323846
#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define EARTH_MU 3.986004418e14
#define EARTH_RADIUS 6371000.0
#define EARTH_J2 1.08262668e-3
#define EARTH_ROTATION_RATE 7.2921150e-5

#define ORBIT_ALTITUDE 500000.0
#define ORBIT_INCLINATION_DEG 97.4
#define ORBIT_PERIOD 5670.0

#define SUN_VECTOR_THRESHOLD 0.985f
#define BATTERY_FULL_VOLTAGE 28.0f
#define BATTERY_CRITICAL_VOLTAGE 20.0f
#define SAFE_MODE_VOLTAGE 22.5f
#define HEATER_ON_TEMP 5.0f
#define HEATER_OFF_TEMP 10.0f
#define MAX_WHEEL_RPM 6000.0f
#define MAX_WHEEL_TORQUE 0.02f
#define MAGNETORQUER_MAX_DIPOLE 0.2f

#define CCSDS_HEADER_SIZE 6
#define TM_PACKET_SIZE 256
#define MAX_CMD_QUEUE 32
#define WATCHDOG_TIMEOUT 10000
#define MAX_TELEMETRY_STORAGE 2048

#define AX25_SYNC 0x7E
#define AX25_CONTROL_UI 0x03
#define AX25_PID 0xF0
#define MAX_AX25_FRAME 512
#define SD_BLOCK_SIZE 512
#define MAX_FILES 128
#define SCIENCE_BUFFER_SIZE 4096

#define THRUSTER_MAX_FORCE 0.5f
#define INITIAL_FUEL 2.5f
#define SPECIFIC_IMPULSE 200.0f

#define EKF_STATE_SIZE 7
#define EKF_MEAS_SIZE 6
#define THERMAL_NODES 10

#define AES_BLOCK_SIZE 16
#define AES_KEY_SIZE 32
#define GCM_TAG_SIZE 16

#define TMR_NUM_COMPUTERS 3
#define EDAC_DATA_BITS 32
#define EDAC_CHECK_BITS 7

#define RTOS_TASK_PRIORITY_HIGHEST 0
#define RTOS_TASK_PRIORITY_LOWEST 15
#define RTOS_MAX_TASKS 32
#define RTOS_TICK_RATE_HZ 100

typedef enum {
    STATE_INIT,
    STATE_DETUMBLE,
    STATE_SUN_ACQUISITION,
    STATE_NOMINAL_EARTH_POINTING,
    STATE_TARGET_TRACKING,
    STATE_ORBIT_MANEUVER,
    STATE_SAFE_MODE,
    STATE_EMERGENCY_SHUTDOWN,
    STATE_FIRMWARE_UPDATE
} SatelliteState;

typedef enum {
    PAYLOAD_OFF,
    PAYLOAD_IDLE,
    PAYLOAD_ACQUIRING,
    PAYLOAD_PROCESSING,
    PAYLOAD_ERROR
} PayloadState;

typedef enum {
    DEPLOY_STOWED,
    DEPLOY_ANTENNA_UHF,
    DEPLOY_SOLAR_PANEL_1,
    DEPLOY_SOLAR_PANEL_2,
    DEPLOY_ANTENNA_XBAND,
    DEPLOY_MAGNETOMETER,
    DEPLOY_COMPLETE
} DeployState;

typedef enum {
    CMD_NO_OP = 0x00,
    CMD_RESET = 0x01,
    CMD_SWITCH_MODE = 0x02,
    CMD_SET_TIME = 0x10,
    CMD_PAYLOAD_ON = 0x20,
    CMD_PAYLOAD_OFF = 0x21,
    CMD_REQUEST_TM = 0x30,
    CMD_REQUEST_FILE = 0x31,
    CMD_DOWNLOAD_FILE = 0x32,
    CMD_UPLOAD_FW = 0x40,
    CMD_MANEUVER = 0x50,
    CMD_SOFTWARE_RESET = 0xFF
} CommandOpCode;

typedef struct {
    float q[4];
    float gyro[3];
    float mag[3];
    float sun_vec[3];
    float star_vec[3];
    bool sun_valid;
    bool mag_valid;
    bool star_valid;
    float eclipse_flag;
} Sensors;

typedef struct {
    float battery_voltage;
    float battery_current;
    float bus_voltage_3v3;
    float bus_voltage_5v;
    float bus_current;
    float panel_power;
    float load_power;
    float temp_battery;
    float temp_mcu;
    float temp_payload;
    float temp_gyro;
    float temp_radio;
    float temp_panels[4];
    bool heater_battery;
    bool heater_gyro;
    bool heater_radio;
    uint32_t energy_gen_wh;
    uint32_t energy_con_wh;
} PowerSystem;

typedef struct {
    float wheel_speed[4];
    float wheel_torque[4];
    bool wheel_enabled[4];
    float mtq_dipole[3];
    bool mtq_enabled[3];
    float attitude_error;
    float integral_error[3];
} Actuators;

typedef struct __attribute__((packed)) {
    uint16_t packet_id;
    uint16_t packet_seq;
    uint16_t packet_len;
} CcsdsHeader;

typedef struct __attribute__((packed)) {
    uint32_t seconds;
    uint32_t micros;
} MissionTime;

typedef struct __attribute__((packed)) {
    CcsdsHeader header;
    uint32_t counter;
    MissionTime timestamp;
    uint8_t state;
    uint8_t payload_state;
    uint8_t deploy_state;
    uint8_t fault_flags;
    float battery_v;
    float bus_current;
    float panel_power;
    float load_power;
    float temps[8];
    float q[4];
    float gyro[3];
    float wheel_speed[4];
    float mtq_dipole[3];
    float gps_pos[3];
    float gps_vel[3];
    float fuel_kg;
    uint32_t uptime;
    uint32_t reboots;
    uint16_t crc;
} TelemetryPacket;

typedef struct __attribute__((packed)) {
    uint16_t sync;
    CcsdsHeader header;
    uint8_t opcode;
    uint8_t checksum;
    uint32_t param1;
    uint32_t param2;
    uint32_t param3;
    uint32_t param4;
    uint8_t payload[64];
} CommandPacket;

typedef struct {
    CommandPacket buffer[MAX_CMD_QUEUE];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
} CommandQueue;

typedef struct {
    uint8_t tx_buf[MAX_AX25_FRAME];
    uint8_t rx_buf[MAX_AX25_FRAME];
    uint16_t tx_len;
    uint16_t rx_len;
    bool data_ready;
    uint32_t last_beacon;
    uint32_t packets_sent;
    uint32_t packets_recv;
    uint32_t packets_drop;
} Radio;

typedef struct __attribute__((packed)) {
    char name[32];
    uint32_t start_block;
    uint32_t size;
    uint32_t created;
    uint32_t modified;
    uint8_t type;
    uint8_t checksum;
    bool valid;
} FileEntry;

typedef struct __attribute__((packed)) {
    uint8_t sync;
    uint8_t msg_id;
    uint16_t week;
    uint32_t tow_ms;
    int32_t lat_e7;
    int32_t lon_e7;
    int32_t alt_cm;
    float ecef_x;
    float ecef_y;
    float ecef_z;
    float ecef_vx;
    float ecef_vy;
    float ecef_vz;
    float pdop;
    uint8_t fix_type;
    uint8_t sats;
    uint8_t checksum;
} GpsData;

typedef struct __attribute__((packed)) {
    uint16_t sync;
    uint32_t timestamp;
    float q[4];
    float error;
    float rate[3];
    uint8_t stars;
    uint8_t mode;
    uint16_t lost_flag;
    uint16_t crc;
} StarTrackerData;

typedef struct {
    bool panel1;
    bool panel2;
    bool uhf;
    bool xband;
    bool mag_boom;
    DeployState state;
    uint32_t start_time;
} Deployment;

typedef struct {
    float thrust[8];
    float fuel_kg;
    float pressure;
    float temp;
    bool valve[8];
    uint32_t pulses;
    uint32_t burn_time_ms;
    float delta_v;
} Propulsion;

typedef struct {
    float kp;
    float kd;
    float ki;
    float kp_rate;
    float max_integral;
} AdcsGains;

typedef struct {
    char ground_call[8];
    char sat_call[8];
    uint32_t beacon_interval;
    uint32_t telem_interval;
    float safe_voltage;
    float crit_voltage;
    float safe_temp;
    AdcsGains gains;
    bool enable_gps;
    bool enable_star;
    bool enable_payload;
    bool enable_crypto;
    bool enable_tmr;
} Config;

typedef struct {
    uint32_t boots;
    uint32_t wdt_resets;
    uint32_t safe_modes;
    uint32_t emergencies;
    float min_battery;
    float max_mcu_temp;
    float min_battery_temp;
} Statistics;

typedef struct {
    float P[EKF_STATE_SIZE * EKF_STATE_SIZE];
    float Q[EKF_STATE_SIZE * EKF_STATE_SIZE];
    float R[EKF_MEAS_SIZE * EKF_MEAS_SIZE];
    float x[EKF_STATE_SIZE];
    float gyro_bias[3];
    bool initialized;
} EkfFilter;

typedef struct {
    float K[3 * 4];
    bool computed;
} LqrController;

typedef struct {
    float C[THERMAL_NODES * THERMAL_NODES];
    float G[THERMAL_NODES * THERMAL_NODES];
    float T[THERMAL_NODES];
    float Q_ext[THERMAL_NODES];
} ThermalNetwork;

typedef struct {
    char name[24];
    double inclination;
    double raan;
    double eccentricity;
    double arg_perigee;
    double mean_anomaly;
    double mean_motion;
    uint32_t epoch;
} TleData;

typedef struct {
    double position[3];
    double velocity[3];
    double lat;
    double lon;
    double alt;
    bool eclipse;
    uint32_t timestamp;
} OrbitState;

typedef struct {
    uint8_t key[AES_KEY_SIZE];
    uint8_t iv[AES_BLOCK_SIZE];
    bool initialized;
} CryptoContext;

typedef struct {
    uint32_t data[EDAC_DATA_BITS / 8];
    uint8_t ecc[EDAC_CHECK_BITS / 8 + 1];
    bool corrected;
    uint8_t errors;
} EdacWord;

typedef struct {
    uint8_t computer_id;
    uint32_t outputs[TMR_NUM_COMPUTERS];
    uint32_t voted_output;
    bool disagreement;
} TmrVoter;

typedef enum {
    TASK_SUSPENDED,
    TASK_READY,
    TASK_RUNNING,
    TASK_BLOCKED
} TaskState;

typedef struct {
    void (*func)(void);
    uint32_t period_ms;
    uint32_t last_run;
    uint8_t priority;
    TaskState state;
    const char* name;
} RtosTask;

typedef struct {
    RtosTask tasks[RTOS_MAX_TASKS];
    uint8_t task_count;
    uint8_t current_task;
    uint32_t tick_count;
    uint32_t total_time_ms;
} RtosScheduler;

typedef struct {
    double lat;
    double lon;
    uint32_t min_sun_elevation;
    uint32_t priority;
} Target;

typedef struct {
    Target targets[32];
    uint8_t count;
    uint32_t next_imaging_time;
    uint32_t next_target_index;
} MissionPlan;

extern Sensors g_sensors;
extern PowerSystem g_power;
extern Actuators g_actuators;
extern SatelliteState g_state;
extern PayloadState g_payload_state;
extern DeployState g_deploy_state;
extern uint32_t g_mission_time;
extern float g_orbit_phase;
extern TelemetryPacket g_telemetry;
extern CommandQueue g_cmd_queue;
extern MissionTime g_mission_clock;
extern uint32_t g_telem_counter;
extern uint32_t g_watchdog_counter;
extern bool g_watchdog_enabled;
extern Radio g_radio;
extern FileEntry g_files[MAX_FILES];
extern uint8_t g_sd[SD_BLOCK_SIZE * 2048];
extern uint32_t g_next_block;
extern bool g_sd_ready;
extern GpsData g_gps;
extern StarTrackerData g_star;
extern bool g_gps_valid;
extern bool g_star_valid;
extern Deployment g_deploy;
extern Propulsion g_prop;
extern Statistics g_stats;
extern Config g_config;
extern EkfFilter g_ekf;
extern LqrController g_lqr;
extern ThermalNetwork g_thermal;
extern TleData g_tle;
extern OrbitState g_orbit;
extern CryptoContext g_crypto;
extern TmrVoter g_tmr;
extern RtosScheduler g_rtos;
extern MissionPlan g_mission_plan;

float clamp(float v, float min, float max);
float deadband(float v, float threshold);
float lpf(float in, float prev, float alpha);
void quat_to_euler(float q[4], float* r, float* p, float* y);
void euler_to_quat(float r, float p, float y, float q[4]);
void quat_mul(float q1[4], float q2[4], float out[4]);
void quat_err(float ref[4], float meas[4], float err[4]);
float vec_dot(float v1[3], float v2[3]);
void vec_cross(float v1[3], float v2[3], float out[3]);
void vec_norm(float v[3]);
float vec_mag(float v[3]);
uint16_t crc16(uint8_t* data, uint16_t len);
uint32_t crc32(uint8_t* data, uint32_t len);
void matrix_mul(float* A, float* B, float* C, int m, int n, int p);
void matrix_transpose(float* A, float* AT, int m, int n);
bool matrix_inv_3x3(float* A, float* inv);
void aes_gcm_encrypt(uint8_t* key, uint8_t* iv, uint8_t* plain, uint32_t plain_len,
                     uint8_t* cipher, uint8_t* tag);
bool aes_gcm_decrypt(uint8_t* key, uint8_t* iv, uint8_t* cipher, uint32_t cipher_len,
                     uint8_t* tag, uint8_t* plain);
void edac_encode(uint32_t data, uint8_t* ecc);
bool edac_decode(uint32_t* data, uint8_t* ecc);
uint32_t tmr_vote(uint32_t v1, uint32_t v2, uint32_t v3);
void rtos_init(void);
void rtos_create_task(void (*func)(void), uint32_t period_ms, uint8_t priority, const char* name);
void rtos_start(void);
void rtos_tick(void);
bool tle_parse(const char* filename, TleData* tle);
void sgp4_propagate(TleData* tle, uint32_t timestamp, OrbitState* state);

void sensors_update(uint32_t time_sec);
void ekf_predict(float* gyro, float dt);
void ekf_update(float* mag, float* sun, float* star);
void lqr_compute(float q[4], float omega[3], float* torque);
void adcs_control(void);
void eps_update(void);
void battery_model(float current, float dt);
void solar_panel_model(float sun_vec[3], float eclipse);
void thermal_update(void);
void thermal_network_step(float dt);
void radio_init(void);
void ax25_encode_callsign(uint8_t* out, const char* call, uint8_t ssid);
void radio_send(uint8_t* data, uint16_t len);
bool radio_recv(uint8_t* data, uint16_t* len);
void radio_beacon(void);
void ccsds_pack_tm(void);
void cfdp_send_file(const char* filename);
void cfdp_recv_file(uint8_t* data, uint32_t len);
void sd_init(void);
bool sd_write(uint32_t block, uint8_t* data);
bool sd_read(uint32_t block, uint8_t* data);
int32_t fs_create(const char* name, uint8_t type);
int32_t fs_find(const char* name);
void fs_save_tm(void);
void fs_save_science(uint8_t* data, uint32_t len);
void gps_update(void);
void star_tracker_update(void);
void state_machine(void);
void fault_detection(void);
void safe_mode(void);
void emergency(void);
void watchdog_kick(void);
void memory_scrubber(void);
void tmr_check(void);
void deployment_update(void);
void propulsion_update(void);
void propulsion_burn(float dv[3]);
void mission_planner_update(void);
void command_process(CommandPacket* cmd);
void statistics_update(void);
bool config_load(const char* filename);
void config_save(const char* filename);
void satellite_init(void);
void satellite_step(uint32_t time_sec, uint32_t elapsed_ms);
void satellite_shutdown(void);

#endif