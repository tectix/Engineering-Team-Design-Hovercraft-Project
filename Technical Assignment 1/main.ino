                    #define F_CPU 20000000
                    
                    #include <avr/io.h>
                    #include <avr/interrupt.h>
                    #include <util/delay.h>

                    // Connected to P6 Connector on Board
                    #define echoPin 2
                    #define trigPin PB3

                    // Constants 
                    #define MAX_DISTANCE 46.0 // cm
                    #define MIN_DISTANCE 12.0 // cm

                    // Mode control
                    int irMode = 1; // 0 = US, 1 = IR
                    long dutyCycle = 0;
                    float duration;
                    float distance;

        
                    void initADC()
                    {
                    // Reference voltage AVcc/ARef
                    ADMUX |= (1<<REFS0);
                    //set prescaller to 128
                    //Enable ADC
                    ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);
                    }


                    uint16_t getADC(uint8_t PIN)
                    {
                    ADMUX = (ADMUX & 0xF0) | (PIN & 0x0F);
                    //single conversion mode
                    ADCSRA |= (1<<ADSC);
                    while( ADCSRA & (1<<ADSC) );
                    return ADC;
                    }

                    float calcIrDistance(uint16_t pwr){
                    float distance =(float) pwr;
                    distance = 13363 * pow(distance, -1.15);
                    return distance;
                    }

                    void setLedBrightness(float distanceCm) {
                      if(distanceCm >= MAX_DISTANCE) {
                        dutyCycle = 100;  //0% brightness
                        PORTB |= ((1<<PB5)); // TURN L ON
                      } else if(distanceCm <= MIN_DISTANCE) {
                        dutyCycle = 0;  //100% brightness
                        PORTB |= ((1<<PB5));
                      } else {
                        dutyCycle = map(distanceCm, MIN_DISTANCE, MAX_DISTANCE, 0, 100);
                        PORTB &= ~((1<<PB5)); // TURN L OFF
                      }
                     
                    }
                    
                    void initPWM(double dC)
                    {
                    DDRB = (1 << PORTB3); // LED BOARD
                    // initialize TCCR2 as: fast pwm mode (p.130) --> mode 3, inverting mode
                    TCCR2A = (1 << COM2A1) | (1 << WGM20) | (1 << WGM21);
                    TIMSK2 = (1 << TOIE2);
                    // Set duty cycle
                    OCR2A = (dC/100.0)*255.0;
                    // Divides the outputted timer clock frequency by the number of clock select bits
                    TCCR2B |= (1<<CS21) | (1<<CS22) ;
                    }
                    ISR(TIMER2_OVF_vect)
                    {
                    OCR2A = (dutyCycle/100.0)*255;
                    }
                    float getUSCm()
                    {
                    DDRB |= (0 << PORTB5);
                    _delay_us(2);
                    DDRB |= (1 << PORTB5);
                    _delay_us(10);
                    DDRB |= (0 << PORTB5);
                    duration =  pulseIn(echoPin, HIGH);
                    // Calculating the distance
                    distance = duration * 0.034 / 2;
                    return distance;
                    }
                    float getUSPulseWidth()
                    {
                    DDRB |= (0 << PORTB5);
                    _delay_us(2);
                    DDRB |= (1 << PORTB5);
                    _delay_us(10);
                    DDRB |= (0 << PORTB5);
                    duration = pulseIn(echoPin, HIGH);
                    return duration;
                    }


                    int main(void) {
                    // LED L set to output on arduino
                    DDRB |= (1 << PORTB5);
                    Serial.begin(9600);
                    // allows for external interrupt
                    sei();
                    initPWM(dutyCycle);
                    initADC();
                   
                    if(irMode)
                    {
                    while (1)
                    {

                    Serial.print("\n\nIR ADC digital Reading: ");
                    Serial.println(getADC(PC0));
                    float distIR = calcIrDistance(getADC(PC0));
                    Serial.print("IR Distance: ");
                    Serial.print(distIR);
                    setLedBrightness(distIR);
                    _delay_ms(1000); //long delay to avoid spikes
                    }
                    }
                    else{
                    while (1)
                    {

                    Serial.print("\nUS Distance Reading: ");
                    float distanceLoop = getUSCm();
                    Serial.print(distanceLoop);
                    Serial.print(" cm");
                    Serial.print("\nUS Pulse Width : ");
                    long durationLoop = getUSPulseWidth();
                    Serial.print(durationLoop);
                    Serial.println(" us");
                    setLedBrightness(distanceLoop);
                    _delay_ms(1000); //long delay to avoid spikes
                    }
                    }
                    }