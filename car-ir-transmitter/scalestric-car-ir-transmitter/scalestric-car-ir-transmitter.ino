/*
 * ATtiny85_IR_SlotCar_Controller.ino
 * 
 * ATtiny85 IR LED Slot Car Controller
 * Supports configurable timer (Timer0/Timer1) and clock speeds (1/8/16 MHz)
 * 
 * Pin Connections:
 * - Timer0: IR LED on PB0 (Physical Pin 5)
 * - Timer1: IR LED on PB1 (Physical Pin 6)
 * - PB2 (Physical Pin 7): Digital input (HIGH=straight, LOW=lane change)
 * - PB3 (Physical Pin 2): XTAL1 (if using external crystal)
 * - PB4 (Physical Pin 3): XTAL2 (if using external crystal)
 * 
 * Hardware Requirements:
 * - ATtiny85 microcontroller
 * - IR LED connected to appropriate pin (PB0 or PB1)
 * - Digital control signal on PB2
 * - Optional: External crystal on PB3/PB4
 * 
 * Configuration:
 * - Set CAR_ID (1-6) for unique car identification
 * - Set TIMER_SELECT (0 or 1) for Timer0 or Timer1
 * - Set CLOCK_MHZ (1, 8, or 16) to match ATtiny clock speed
 * 
 * Author: Slot Car Project
 * Version: 2.1
 * Date: 2026
 */

#include <avr/io.h>

// ===== CONFIGURATION =====
const uint8_t CAR_ID = 1;           // Set your car ID (1-6)
const uint8_t TIMER_SELECT = 1;     // 0 = Timer0, 1 = Timer1
const uint8_t CLOCK_MHZ = 8;        // 1, 8, or 16 MHz
// =========================

// Duty cycle values for ~3.9kHz base frequency
// These create recognizable patterns for each car ID
const uint8_t idDutyStraight[] = {191, 170, 150, 128, 106, 85};  // 75% equivalent for each ID
const uint8_t idDutyLaneChange[] = {64, 85, 106, 128, 150, 170}; // 25% equivalent for each ID

uint8_t dutyStraight;
uint8_t dutyLaneChange;

// Pin definitions based on timer selection
#define TIMER0_PIN PB0  // OC0A on PB0 (Pin 5)
#define TIMER1_PIN PB1  // OC1A on PB1 (Pin 6)
#define INPUT_PIN  PB2  // Digital input (Pin 7)

void setup() {
    // Configure output pin based on timer selection
    if (TIMER_SELECT == 0) {
        DDRB |= (1 << TIMER0_PIN);  // Timer0 output
    } else {
        DDRB |= (1 << TIMER1_PIN);  // Timer1 output
    }
    
    DDRB &= ~(1 << INPUT_PIN); // Digital input on PB2
    PORTB |= (1 << INPUT_PIN); // Pull-up for input
    
    // Get duty cycles for selected car ID
    uint8_t idx = CAR_ID - 1;
    if (idx > 5) idx = 0;
    dutyStraight = idDutyStraight[idx];
    dutyLaneChange = idDutyLaneChange[idx];
    
    // Calculate prescaler needed for ~3.9kHz PWM
    // Target: 3.9kHz = Clock / (Prescaler * 256)
    uint8_t prescaler_bits;
    
    if (CLOCK_MHZ == 1) {
        prescaler_bits = 0x01;  // No prescaling (CS=001)
    } else if (CLOCK_MHZ == 8) {
        prescaler_bits = 0x02;  // Prescaler /8 (CS=010)
    } else { // 16 MHz
        prescaler_bits = 0x03;  // Prescaler /64 (CS=011)
    }
    
    if (TIMER_SELECT == 0) {
        // Timer0 Fast PWM on OC0A (PB0)
        TCCR0A = (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);  // Fast PWM, Clear on match
        TCCR0B = prescaler_bits;
        OCR0A = dutyStraight;  // Start with straight mode
    } else {
        // Timer1 Fast PWM on OC1A (PB1)
        // Timer1 is more complex - using PWM mode with OCR1C as TOP
        TCCR1 = (1 << CTC1) | (1 << PWM1A) | (1 << COM1A1) | prescaler_bits;
        GTCCR = (1 << PWM1B);  // Enable PWM mode
        OCR1C = 255;  // TOP value for 8-bit resolution
        OCR1A = dutyStraight;  // Start with straight mode
    }
}

void loop() {
    uint8_t newDuty;
    
    if (PINB & (1 << INPUT_PIN)) {
        newDuty = dutyStraight;     // Input HIGH - straight mode
    } else {
        newDuty = dutyLaneChange;   // Input LOW - lane change mode
    }
    
    // Update appropriate timer register
    if (TIMER_SELECT == 0) {
        OCR0A = newDuty;
    } else {
        OCR1A = newDuty;
    }
}
