// For examples of ps3 use see
// https://github.com/jvpernis/esp32-ps3/blob/master/examples/Ps3Demo/Ps3Demo.ino

#include <Ps3Controller.h>
#include <Arduino.h>
#include <array>
#include <Adafruit_NeoPixel.h>

constexpr int pwm_resolution_bits = 16;
constexpr int pwm_freq_hz = 50;
constexpr int num_pwm_pins = 3;
constexpr int motor_hbridge_pwm_pins[] = {33,32,13}; // 0, 2 not used atm
char mac[] = "01:02:03:04:05:06";
const float deadzone_radius = 0.1;

Adafruit_NeoPixel lights(2, 12);

float stick_offsets[] = {0.0f, 0.0f, 0.0f};

bool light_highbeams = false;
bool light_blink_left = false;
bool light_blink_right = false;
uint8_t light_brightness = 128;

uint32_t anim;

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

// \param x \in [-1, 1]
// \param y \in [-1, 1]
std::array<float,num_pwm_pins> GetMotorPwm(float x, float y)
{
    return {
        // motor A & B duty fractions for H-Bridge
        hbridge_a(y), hbridge_b(y),
        0.075f + x * 0.025f
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
void update_pwm(float x, float y)
{
    // Take controller input from range [-128,128] into [-1, 1]
    // Map to H-Bridge input PWM duty cycle fraction [0, 1]
    const auto pwms = GetMotorPwm(x, y);
    
    // Map to PWM HW duty integer and update.
    for(int i=0; i < num_pwm_pins; ++i) {
        ledcWrite(i, fraction_to_duty_int(pwms[i]) );
    }
}

void setup_lights()
{
    lights.begin();
    lights.fill(0xffffffff);
    lights.show();
}

void update_lights()
{
    lights.setBrightness(light_brightness);

    const uint32_t color_base = light_highbeams ? 0xffffffff : 0x00000000;
    const uint32_t color_turn = 0x00BFFF00;
    const uint32_t color_left = (light_blink_left && (anim%2) ) ? color_turn : color_base;
    const uint32_t color_right = (light_blink_right && (anim%2) ) ? color_turn : color_base;

    lights.setPixelColor(0, lights.gamma32(color_left) );
    lights.setPixelColor(1, lights.gamma32(color_right) );
    lights.show();
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

    if( Ps3.event.button_down.select ) {
        // Calibration
        stick_offsets[0] = x;
        stick_offsets[1] = y;
    }
    
    if(Ps3.event.button_down.triangle) {
        light_highbeams = true;
        light_blink_left = false;
        light_blink_right = false;
    }else if(Ps3.event.button_down.cross) {
        light_highbeams = false;
        light_blink_left = false;
        light_blink_right = false;
    }else if(Ps3.event.button_down.square) {
        anim = 1;
        light_blink_right = false;
        light_blink_left = !light_blink_left;
    }else if(Ps3.event.button_down.circle) {
        anim = 1;
        light_blink_left = false;
        light_blink_right = !light_blink_right;
    }else if(Ps3.event.button_down.start) {
        anim = 1;
        light_blink_left = !light_blink_left;
        light_blink_right = !light_blink_right;
    }

    x -= stick_offsets[0];
    y -= stick_offsets[1];

    update_pwm( map_stick(x), map_stick(y) );
    update_lights();

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
    setup_lights();
    setup_pwm();
    setup_bluetooth();
}

void loop()
{
    if(!Ps3.isConnected()) {
        stop_pwm();
    }

    update_lights();

    delay(500);
    ++anim;
}
