//its reccomended to lower the CPU frequency to save power

#include <SPI.h>
#include <LoRa.h> //https://github.com/sandeepmistry/arduino-LoRa
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/power.h>

#define MOISTURE_SENSOR_ID 0x00

#define LORA_FREQ  869525000 //869.525mhz is allowed to be used at 100mW 10% duty cycle (360 sec an hour) in germany
#define LORA_TX_PWR 18 //25mW
#define LORA_MAGIC 42

//#define SLEEP_SECONDS 1800 //30 minutes
#define SLEEP_SECONDS 15

#define MOISTURE_PIN A0
#define MOISTURE_ADC_DIV 5
#define MOISTURE_ADC_OFFSET 512

//this calculates automatically
#define TIMER_FREQ ((uint32_t)F_CPU / (uint32_t)1024) //15.something kilohertz for 16mhz clock
#define SECONDS_PER_TIMER_OVERFLOW ((double)0xFFFF / (double)TIMER_FREQ) //4.194304 seconds for 16mhz
#define TIMER_OVERFLOWS_UNTIL_SLEEP_END uint32_t((float)SLEEP_SECONDS/(float)SECONDS_PER_TIMER_OVERFLOW)

uint8_t lora_outgoing_packet_id = 1;

void setup() {
  wdt_enable(WDTO_8S);
  power_timer0_disable();
  power_timer2_disable();
  power_twi_disable();

  //timer setup
  //clock is F_CPU (usually 16mhz) divided by 1024
  //setup imer for 1024 divider and normal mode
  //[0][0][0][0][0][0][WGM11][WGM10]
  TCCR1A = 0b00000000;
  //[0][0][0][WGM13][WGM12][CS12][CS11][CS10]
  TCCR1B = 0b00000101;
  //enable overflow interrupt
  TIMSK1 |= (1 << TOIE1);

  pinMode(LED_BUILTIN, HIGH);
  pinMode(MOISTURE_PIN, INPUT);
  Serial.begin(9600);
  Serial.println(F("First boot."));

  LoRa.setPins(10, 9, 2);
  LoRa.setSPIFrequency(1E5); //100khz is fast enough and works with lower clockspeeds
  if (LoRa.begin(LORA_FREQ)) {
    LoRa.idle();
    LoRa.setSyncWord(0x12);
    LoRa.setTxPower(LORA_TX_PWR);
    LoRa.setSpreadingFactor(12);
    LoRa.setSignalBandwidth(250E3);
    LoRa.setCodingRate4(8); //sf,bw,cr make a data rate of 366 bits per sec or 45,75 bytes per sec
    LoRa.enableCrc();
  }
  else {
    Serial.println(F("Error: LoRa Module not reachable via SPI. Check wires."));
    while (true) ;; //reset mcu
  }
}

void loop() {
  pinMode(LED_BUILTIN, HIGH);
  wdt_enable(WDTO_8S);
  wdt_reset();
  Serial.println ("I'm Awake!");
  power_adc_enable();
  power_spi_enable();

  LoRa.idle();

  double ground_moisture  = 0;
  for (uint8_t s = 0; s < 64; s++) {
    ground_moisture  += analogRead(MOISTURE_PIN);
  }
  ground_moisture /= 64;

  ground_moisture  -= MOISTURE_ADC_OFFSET; //remove offset
  ground_moisture  /= MOISTURE_ADC_DIV; //divide to 0-200 vals
  ground_moisture = min(ground_moisture, 200); //limit to 0-200
  ground_moisture = max(ground_moisture, 0);

  Serial.print(F("Gound Moisture is "));
  Serial.print(ground_moisture / 2);
  Serial.println('%');

  LoRa.beginPacket();
  LoRa.write(LORA_MAGIC);
  LoRa.write(lora_outgoing_packet_id);
  if (++lora_outgoing_packet_id < 1) lora_outgoing_packet_id = 0; //increment packet id then check if its over one.
  LoRa.write(100); //id 100 for packet type moisture sensor
  LoRa.write(MOISTURE_SENSOR_ID);
  LoRa.write((uint8_t)ground_moisture); //uint8_t cast auto-rounds and ensures this is only 1 byte big
  LoRa.endPacket(false);
  Serial.println(F("LoRa Packet sent!"));

  delay(2900);
  LoRa.sleep();
  delay(100);

  Serial.print(F("Going to Sleep for"));
  Serial.print(SLEEP_SECONDS);
  Serial.println(F("seconds now..."));
  Serial.flush();

  delay(100);
  pinMode(LED_BUILTIN, LOW);

  wdt_reset();
  wdt_disable(); //stop watchdog from waking us up

  power_adc_disable();
  power_spi_disable();
  power_timer0_disable();
  power_timer2_disable();
  power_twi_disable();

  for (uint32_t cycles = 0; cycles < TIMER_OVERFLOWS_UNTIL_SLEEP_END; cycles++) {
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();

    sleep_cpu();

    //////////////////////////////////
    // sleepytime until wakeup here //
    //////////////////////////////////

    sleep_disable();
  }
}

ISR(TIMER1_OVF_vect) { //interrupt wakes up CPU
  //do nothing
}
