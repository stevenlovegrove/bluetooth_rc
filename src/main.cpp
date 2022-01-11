// For examples of ps3 use see
// https://github.com/jvpernis/esp32-ps3/blob/master/examples/Ps3Demo/Ps3Demo.ino

#include <Ps3Controller.h>
#include <Arduino.h>
#include <array>

constexpr int pwm_resolution_bits = 10;
constexpr int pwm_freq_hz = 1000;
constexpr int num_pwm_pins = 6;
constexpr int motor_hbridge_pwm_pins[] = {12,13,32,33,0,2};
char mac[] = "01:02:03:04:05:06";
const float deadzone_radius = 0.03;

// The steering modulation rate when turning.
// Lower number steers tighter
constexpr float sf = 0.4f;
constexpr float bf = 1.0f;

float stick_offsets[] = {0.0f, 0.0f, 0.0f};

float clamp(float val, float lo, float hi)
{
    return std::max(lo, std::min(val, hi));
}

// return linear interpolation of a and b at val (\in [0,1])
float lerp(float a, float b, float val)
{
    return (1.0f-val)*a + val*b;
}

// bilinear interpolation of quad:
//   x11, x12 
//   x21, x22  
float lerp(float x11, float x12, float x21, float x22, float valx, float valy)
{
    return lerp( 
        lerp(x11, x12, valx),
        lerp(x21, x22, valx),
        valy
    );
}

float table_lerp(float table[4][4], float x, float y)
{
    const int x_int = int(x);
    const float x_frac = x - x_int;
    const int y_int = int(y);
    const float y_frac = y - y_int;

    const float a = table[y_int][x_int];
    const float b = table[y_int][x_int+1];
    const float c = table[y_int+1][x_int];
    const float d = table[y_int+1][x_int+1];

    return lerp(a,b,c,d, x_frac, y_frac);
}

// Channel A of H-Bridge control input
float hbridge_a(float signed_ratio)
{
    return std::max(0.0f, signed_ratio);
}

// Channel B of H-Bridge control input
float hbridge_b(float signed_ratio)
{
    return std::max(0.0f, -signed_ratio);
}

// \param left_right \in [-1, 1]
// \param up_down \in [-1, 1]
std::array<float,6> GetMotorPwm(float left_right, float up_down, float bucket_up_down)
{
    // Left / Right motor duty period fraction (+ive forward, -ive backward)
    // for the extremes of motion of the analog stick
    //  sf,  1      1, 1     1, sf
    //  -1,  1      0, 0     1, -1
    // -sf, -1     -1,-1    -1, -sf
    // 
    // We pad the table right and bottom (to make 4x4 instead of 3x3) to simplifiy rounding lerp logic below
    // to stay in bounds.
    float table_motor_left[4][4] = {
        {sf, 1.0f, 1.0f, 0.0f},
        {-1.0f, 0.0f, 1.0f, 0.0f},
        {-sf, -1.0f, -1.0f, 0.0f},
        {0.0f, 0.0f, 0.0f, 0.0f},
    };
    float table_motor_right[4][4] = {
        {1.0, 1.0f, sf, 0.0f},
        {1.0f, 0.0f, -1.0f, 0.0f},
        {1.0, -1.0f, -sf, 0.0f},
        {0.0f, 0.0f, 0.0f, 0.0f},
    };

    // Move x, y from range [-1, 1] to [0, 2] for table
    const float x = 1.0f + left_right; // [0, 2]
    const float y = 1.0f + up_down;    // [0, 2]
    const float left_motor = table_lerp(table_motor_left, x, y);
    const float right_motor = table_lerp(table_motor_right, x, y);
    const float bucket_motor = bf * bucket_up_down;

    return {
        // left motor A & B duty fractions for H-Bridge
        hbridge_a(left_motor), hbridge_b(left_motor),
        // right motor A & B duty fractions for H-Bridge
        hbridge_a(right_motor), hbridge_b(right_motor),
        // Bucket motor A & B duty fractions for H-Bridge
        hbridge_a(bucket_motor), hbridge_b(bucket_motor)
    };
}

uint32_t fraction_to_duty_int(float v)
{
    const float vv = v * ((1 << pwm_resolution_bits) - 1);
    return uint32_t(vv);
}

void stop_pwm()
{
    // Turn everything off.
    for(int i=0; i < num_pwm_pins; ++i) {
        ledcWrite(i, 0 );
    }
}

// x,y,bucket in range [-1,1]
void update_pwm(float x, float y, float bucket)
{
    // Take controller input from range [-128,128] into [-1, 1]
    // Map to H-Bridge input PWM duty cycle fraction [0, 1]
    const auto pwms = GetMotorPwm(x, y, bucket);
    
    // Map to PWM HW duty integer and update.
    for(int i=0; i < num_pwm_pins; ++i) {
        ledcWrite(i, fraction_to_duty_int(pwms[i]) );
    }
}

float make_deadzone(float val, float dead_radius)
{
    return val > dead_radius ? val - dead_radius :
        (val < -dead_radius ? val + dead_radius : 0.0f);
}

float map_stick(float val)
{
    const float linear = clamp(make_deadzone(val, deadzone_radius), -1.0f, 1.0f);
    return linear;
}

void notify()
{
    // Stick ranges from -128 to +127
    // x ranges from 0 to 127+128 = 255
    float x = 2.0f * ((int)Ps3.data.analog.stick.rx + 128) / 255.0f - 1.0f;
    float y = 2.0f * ((int)Ps3.data.analog.stick.ry + 128) / 255.0f - 1.0f;
    float bucket = 2.0 - 2.0f * ((int)Ps3.data.analog.stick.ly + 128) / 255.0f - 1.0f;

    if( Ps3.event.button_down.select ) {
        // Calibration
        stick_offsets[0] = x;
        stick_offsets[1] = y;
        stick_offsets[2] = bucket;
    }
    
    if(Ps3.data.button.triangle) {
        bucket = 1.0f;
    }else if(Ps3.data.button.cross) {
        bucket = -0.6f;
    }else{
        bucket += stick_offsets[2];
    }

    x -= stick_offsets[0];
    y -= stick_offsets[1];

    update_pwm( map_stick(x), map_stick(y), map_stick(bucket) );

    // TODO: Go into some charging notification loop for controller
    //     if( Ps3.data.status.battery == ps3_status_battery_low)       Serial.println("LOW");
    //     else if( Ps3.data.status.battery == ps3_status_battery_dying )    Serial.println("DYING");
    //     else if( Ps3.data.status.battery == ps3_status_battery_shutdown ) Serial.println("SHUTDOWN");
}

void onConnect(){
    Ps3.setPlayer(1);
}

void setup_bluetooth()
{
    Serial.begin(115200);

    Ps3.attach(notify);
    Ps3.attachOnConnect(onConnect);
    Ps3.begin(mac);
}

void setup_pwm()
{
    // Set Motor pins to low ASAP to prevent
    // rogue motion.
    for(int i=0; i < num_pwm_pins; ++i) {
        pinMode(motor_hbridge_pwm_pins[i], OUTPUT);
        digitalWrite(motor_hbridge_pwm_pins[i], LOW);
    }

    // Setup timers
    for(int i=0; i < num_pwm_pins; ++i) {
        ledcSetup(i, pwm_freq_hz, pwm_resolution_bits);
        ledcAttachPin(motor_hbridge_pwm_pins[i], i);
        ledcWrite(i, 0);
    }
}

void setup()
{
    setup_pwm();
    setup_bluetooth();
}

void loop()
{
    if(!Ps3.isConnected()) {
        stop_pwm();
    }

    delay(1000);
}
