#include <Arduino.h>
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "hardware/pio_instructions.h"
#include "hardware/clocks.h"

// PINS

const uint8_t AILERON_PIN = 8;  //Might need to change this
const uint8_t ELEVATOR_PIN = 7;//based on channel in from PWM RX

const uint8_t LEFT_SERVO_PIN = 3;
const uint8_t RIGHT_SERVO_PIN = 4;

const uint8_t LED_PIN = 16;

const int PWM_MIN = 1000;
const int PWM_CENTER = 1500;
const int PWM_MAX = 2000;

const int INPUT_MIN = 900;
const int INPUT_MAX = 2100;

// DIRECTIONS

const int AILERON_DIRECTION = -1;  //Might need to change this
const int ELEVATOR_DIRECTION = -1;//based on if your servos are
                                 //facing eachother or away
const int LEFT_SERVO_DIRECTION = 1;
const int RIGHT_SERVO_DIRECTION = 1;

volatile uint32_t ailStart = 0;
volatile uint32_t eleStart = 0;

volatile uint16_t ailValue = PWM_CENTER;
volatile uint16_t eleValue = PWM_CENTER;

// 50 Hz / 1 us

void setupServo(uint8_t pin)
{
    gpio_set_function(
        pin,
        GPIO_FUNC_PWM);

    uint slice =
        pwm_gpio_to_slice_num(pin);

    uint channel =
        pwm_gpio_to_channel(pin);

    pwm_config config =
        pwm_get_default_config();

    pwm_config_set_clkdiv(
        &config,
        125.0f);

    pwm_config_set_wrap(
        &config,
        19999);

    pwm_init(
        slice,
        channel,
        &config,
        true);

    pwm_set_chan_level(
        slice,
        channel,
        PWM_CENTER);
}

void writeServo(
    uint8_t pin,
    int pulse)
{
    pulse = constrain(
        pulse,
        PWM_MIN,
        PWM_MAX);

    uint slice =
        pwm_gpio_to_slice_num(pin);

    uint channel =
        pwm_gpio_to_channel(pin);

    pwm_set_chan_level(
        slice,
        channel,
        pulse);
}

PIO wsPIO = pio0;

uint wsSM = 0;

static const uint16_t ws2812_instructions[] =
    {
        (uint16_t)(pio_encode_out(
                       pio_x,
                       1) |
                   pio_encode_sideset(
                       1,
                       0) |
                   pio_encode_delay(
                       3)),

        (uint16_t)(pio_encode_jmp_not_x(
                       3) |
                   pio_encode_sideset(
                       1,
                       1) |
                   pio_encode_delay(
                       2)),

        (uint16_t)(pio_encode_jmp(
                       0) |
                   pio_encode_sideset(
                       1,
                       1) |
                   pio_encode_delay(
                       2)),

        (uint16_t)(pio_encode_nop() |
                   pio_encode_sideset(
                       1,
                       0) |
                   pio_encode_delay(
                       2))};

static const struct pio_program ws2812_program =
    {
        ws2812_instructions,
        4,
        -1};

void setupWS2812()
{
    pinMode(
        LED_PIN,
        OUTPUT);

    digitalWrite(
        LED_PIN,
        LOW);

    wsPIO = pio0;

    int sm =
        pio_claim_unused_sm(
            wsPIO,
            false);

    if (sm < 0)
    {
        return;
    }

    wsSM =
        (uint)sm;

    uint offset =
        pio_add_program(
            wsPIO,
            &ws2812_program);

    pio_gpio_init(
        wsPIO,
        LED_PIN);

    pio_sm_set_consecutive_pindirs(
        wsPIO,
        wsSM,
        LED_PIN,
        1,
        true);

    pio_sm_config config =
        pio_get_default_sm_config();

    sm_config_set_wrap(
        &config,
        offset,
        offset + 3);

    sm_config_set_sideset(
        &config,
        1,
        false,
        false);

    sm_config_set_sideset_pins(
        &config,
        LED_PIN);

    sm_config_set_out_shift(
        &config,
        false,
        true,
        24);

    sm_config_set_fifo_join(
        &config,
        PIO_FIFO_JOIN_TX);

    float sysClock =
        (float)clock_get_hz(
            clk_sys);

    float divider =
        sysClock / 8000000.0f;

    sm_config_set_clkdiv(
        &config,
        divider);

    pio_sm_init(
        wsPIO,
        wsSM,
        offset,
        &config);

    pio_sm_set_enabled(
        wsPIO,
        wsSM,
        true);
}

void ws2812(
    uint8_t r,
    uint8_t g,
    uint8_t b)
{
    uint32_t pixel =
        (((uint32_t)g << 16) |
         ((uint32_t)r << 8) |
         ((uint32_t)b));

    pixel <<= 8;

    while (
        pio_sm_is_tx_fifo_full(
            wsPIO,
            wsSM))
    {
        tight_loop_contents();
    }

    pio_sm_put(
        wsPIO,
        wsSM,
        pixel);

    delayMicroseconds(60);
}

void aileronISR()
{
    uint32_t now =
        micros();

    if (
        digitalRead(
            AILERON_PIN))
    {
        ailStart = now;
    }
    else
    {
        uint32_t width =
            now - ailStart;

        if (
            width >= INPUT_MIN &&
            width <= INPUT_MAX)
        {
            ailValue =
                (uint16_t)width;
        }
    }
}

void elevatorISR()
{
    uint32_t now =
        micros();

    if (
        digitalRead(
            ELEVATOR_PIN))
    {
        eleStart = now;
    }
    else
    {
        uint32_t width =
            now - eleStart;

        if (
            width >= INPUT_MIN &&
            width <= INPUT_MAX)
        {
            eleValue =
                (uint16_t)width;
        }
    }
}

void updateLED(
    int ail,
    int ele)
{
    int a =
        ail - PWM_CENTER;

    int e =
        ele - PWM_CENTER;

    const int DEADZONE = 30;

    if (abs(a) < DEADZONE)
        a = 0;

    if (abs(e) < DEADZONE)
        e = 0;

    if (
        a == 0 &&
        e == 0)
    {
        ws2812(
            0,
            0,
            0);

        return;
    }

    int aBrightness =
        map(
            abs(a),
            0,
            500,
            0,
            80);

    int eBrightness =
        map(
            abs(e),
            0,
            500,
            0,
            80);

    aBrightness =
        constrain(
            aBrightness,
            0,
            80);

    eBrightness =
        constrain(
            eBrightness,
            0,
            80);

    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;

    if (a < 0)
    {
        b =
            (uint8_t)aBrightness;
    }
    else if (a > 0)
    {
        g =
            (uint8_t)aBrightness;
    }

    if (e != 0)
    {
        r =
            (uint8_t)eBrightness;
    }

    ws2812(
        r,
        g,
        b);
}

void setup()
{
    pinMode(
        AILERON_PIN,
        INPUT);

    pinMode(
        ELEVATOR_PIN,
        INPUT);

    setupServo(
        LEFT_SERVO_PIN);

    setupServo(
        RIGHT_SERVO_PIN);

    setupWS2812();

    attachInterrupt(
        digitalPinToInterrupt(
            AILERON_PIN),
        aileronISR,
        CHANGE);

    attachInterrupt(
        digitalPinToInterrupt(
            ELEVATOR_PIN),
        elevatorISR,
        CHANGE);

    writeServo(
        LEFT_SERVO_PIN,
        PWM_CENTER);

    writeServo(
        RIGHT_SERVO_PIN,
        PWM_CENTER);

    ws2812(
        0,
        0,
        0);
}

void loop()
{
    uint16_t ail;
    uint16_t ele;

    // Atomic copy
    noInterrupts();

    ail =
        ailValue;

    ele =
        eleValue;

    interrupts();

    int aileron =
        ((
             (int)ail -
             PWM_CENTER) *
         AILERON_DIRECTION);

    int elevator =
        ((
             (int)ele -
             PWM_CENTER) *
         ELEVATOR_DIRECTION);

    int left =
        PWM_CENTER +
        elevator +
        aileron;

    int right =
        PWM_CENTER +
        elevator -
        aileron;

    left =
        PWM_CENTER +
        ((
             left -
             PWM_CENTER) *
         LEFT_SERVO_DIRECTION);

    right =
        PWM_CENTER +
        ((
             right -
             PWM_CENTER) *
         RIGHT_SERVO_DIRECTION);

    left =
        constrain(
            left,
            PWM_MIN,
            PWM_MAX);

    right =
        constrain(
            right,
            PWM_MIN,
            PWM_MAX);

    writeServo(
        LEFT_SERVO_PIN,
        left);

    writeServo(
        RIGHT_SERVO_PIN,
        right);

    static uint32_t lastLED = 0;

    uint32_t now =
        millis();

    if (
        now - lastLED >= 33)
    {
        lastLED = now;

        updateLED(
            ail,
            ele);
    }
}