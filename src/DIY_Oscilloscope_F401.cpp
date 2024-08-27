#ifdef STM32F4
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Arduino-STM32-8bitTFT.h>
#include <SPI.h>
#include <SPIMemory.h>
#include <SD.h>
#include <SerialCommand.h>
#include <TouchScreen.h>

// PB_LOW (entire PB realisticly) for data lines
// PA 10 9 8 3 2 for commands, this way 7 6 5 4 can be used as spi for sdcard and 1 0 will be free
// SPI1: PA7 MOSI		PA6 MISO		PA5 SCK		PA4 EEPROM CS		PA10 SDCARD CS
#define ADCPIN PA0
#define MOSI PA7
#define MISO PA6
#define SCK PA5
#define CS_EEPROM PA4
#define CS_SDCARD PA10
#define TOUCH_XP PB0 // D0
#define TOUCH_XM PA1 // RS
#define TOUCH_YP PA3 // CS
#define TOUCH_YM PB1 // D1
#define TOUCH_Res 326 // Resistance between X+ and X-
// #define TOUCH_CALIB_X //680 to 3560
// #define TOUCH_CALIB_Y //575(including black bar at buttom) or 760 to 3690 
#define TOUCH_X_MIN 770
#define TOUCH_X_MAX 3600
#define TOUCH_Y_MIN 750
#define TOUCH_Y_MAX 3700
#define DISP_W 320
#define DISP_H 240
#define LANDSCAPE 1
#define PORTRAIT 0
#define MAX_SAMPLES (27925) // * 0.358us/s = ~10ms window

// Display stuff
#define BEAM1_COLOUR GREEN
#define BEAM2_COLOUR RED
#define GRATICULE_COLOUR DARKCYAN
#define BEAM_OFF_COLOUR BLACK
#define TEXT1_COLOUR GREENYELLOW
#define TEXT2_COLOUR ORANGE
STM32_TFT_8bit tft;
uint16_t tft_ID;
float frametime = 0;
TouchScreen ts(TOUCH_XP, TOUCH_YP, TOUCH_XM, TOUCH_YM, TOUCH_Res);
TSPoint p;
uint8_t touch_cnt = 0;

// Scope stuff
float sampletime = 0;
// Variables for the beam position
uint16_t signalX;
uint16_t signalY;
uint16_t signalY1;
int16_t x_zoom_factor = 100;
// yZoomFactor (percentage)
int16_t y_zoom_factor = 100; //Adjusted to get 3.3V wave to fit on screen
int16_t y_pos = 0;
uint32_t start_sample = 0; // Start of the window
uint32_t end_sample = MAX_SAMPLES; // End of the window
uint8_t trigger_held = 0; // Startup with sweep hold off or on
const char* tdivus = " uS/Div";
const char* tdivms = " mS/Div";
float tdiv = 0;
//Trigger stuff
uint8_t not_triggered;
// Sensitivity is the necessary change in AD value which will cause the scope to trigger.
// If VAD=3.3 volts, then 1 unit of sensitivity is around 0.8mV but this assumes no external attenuator.
// Calibration is needed to match this with the magnitude of the input signal.
// Trigger is setup in one of 32 positions
#define TRIGGER_STEP 128 // (4096/32)
int32_t trigger_value = 2048; // Trigger default position
int16_t retrigger_delay = 0;
int8_t trigger_type = 1; //0-both 1-negative 2-positive 3-no trigger
uint16_t trigger_points[2]; //Array for trigger points

// Settings stuff
HardwareTimer test_sig_timer(TIM3);
uint16_t buffer[MAX_SAMPLES];
uint16_t dataPlot[DISP_W]; //array for computed data (speedup)
uint32_t _settings_alloc = 0;
// ADC Clock Prescaler (0 1 2 3), Sample Cycle (0 1 2 3 4 5 6 7), 
uint8_t *settings = (uint8_t *)(&_settings_alloc);
SerialCommand cmd;

// Storage stuff
SPIClass _eeprom_spi(MOSI, MISO, SCK, CS_EEPROM);
SPIFlash eeprom(CS_EEPROM, &_eeprom_spi);
Sd2Card sdcard;
SdVolume sdvol;
SdFile sdroot;

/* Function Prototypes: */
// Re-initialize ADC using settings and fill the buffer
void timer2_update_callback();
void fill_sample_buffer(); 
void dbg_print_on_usb();
void show_graticule();
void show_labels();
void trigger();
void trigger_both();
void trigger_p();
void trigger_n();
void tft_samples_clear(uint16_t beam_color);
void tft_samples(uint16_t beam_color);
void read_touch();
// Commands:
void toggle_hold();
void dec_xzoom();
void inc_xzoom();
void dec_yzoom();
void inc_yzoom();
void scroll_right();
void scroll_left();
void inc_edgetype();
void dec_ypos();
void inc_ypos();
void dec_triggerpoint();
void inc_triggerpoint();
void unrecognized(const char *command);
void set_sig_gen_freq();
void inc_samplecycle();

void setup() {
	delay(1000);
	SerialUSB.begin();
	SerialUSB.dtr(false);
	SerialUSB.println("Initializing...");
	// // Activating clocks to pripherals and ports
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	delay(1); // wait for clock stablization
	GPIOA->MODER |= GPIO_MODER_MODE0; // Analog mode on PA0
	GPIOB->MODER &= ~GPIO_MODER_MODE0;
	GPIOB->MODER |= GPIO_MODER_MODE0_1; // AF mode pn PB0
	GPIOB->AFR[0] = GPIO_AFRL_AFRL0_0; // AF1: TIM3_CH3
	analogReadResolution(12);

	/* Init Serial Commands: */
	cmd.addCommand("h", toggle_hold);			   // Turns triggering on/off
	cmd.addCommand("t", inc_xzoom);		   
	cmd.addCommand("T", dec_xzoom);		   
	cmd.addCommand("z", dec_yzoom);	   
	cmd.addCommand("Z", inc_yzoom);	   
	cmd.addCommand("r", scroll_right);			   // start onscreen trace further right
	cmd.addCommand("l", scroll_left);			   // start onscreen trae further left
	cmd.addCommand("e", inc_edgetype);			   // increment the trigger edge type 0 1 2 3 0 1 2 3 etc
	cmd.addCommand("y", dec_ypos);	   // move trace Down
	cmd.addCommand("Y", inc_ypos);	   // move trace Up
	cmd.addCommand("g", dec_triggerpoint); // move trigger position Down
	cmd.addCommand("G", inc_triggerpoint); // move trigger position Up
	cmd.addCommand("k", set_sig_gen_freq);
	cmd.addCommand("c", inc_samplecycle);
	cmd.setDefaultHandler(unrecognized);
	cmd.clearBuffer();

	/* Init LCD: */
	tft_ID = tft.readID();
	tft.begin(tft_ID);
	SerialUSB.println("Initializing LCD...");
	SerialUSB.print("Device ID: 0x"); SerialUSB.println(tft_ID, HEX);
	tft.fillScreen(BEAM_OFF_COLOUR);
	tft.setRotation(LANDSCAPE);
	tft.setTextColor(TEXT1_COLOUR, BEAM_OFF_COLOUR);
	tft.println("LCD Initialization Done...");
	delay(100);

	/* Init test signal: */
	// Note that Arduino has to handle interrupts or it will get stuck.
	pinMode(LED_BUILTIN, OUTPUT);
	// TIM3->PSC = 300*10-1;
	// TIM3->ARR = 8400;
	// TIM3->SR &= ~TIM_SR_UIF; // clear interrupt flag
	// TIM3->DIER |= TIM_DIER_UIE; // enable update interrupt
	// NVIC_EnableIRQ(TIM3_IRQn);
	// TIM3->CR1 |= TIM_CR1_CEN; // 84M / (PSC+1) / ARR = 1000hz
	test_sig_timer.pause();
	// test_sig_timer.setPrescaleFactor(sig_gen_ms*10-1); //no workie
	test_sig_timer.setOverflow(1000, HERTZ_FORMAT);
	test_sig_timer.attachInterrupt(timer2_update_callback);
	test_sig_timer.refresh();
	test_sig_timer.resume();

	/* Init SPI devices: */
	eeprom.begin();
	sdcard.init(SPI_HALF_SPEED, CS_SDCARD);
	sdvol.init(sdcard);
	sdroot.openRoot(sdvol);

	// Quick diagnostics
	tft.setTextColor(TEXT2_COLOUR, BEAM_OFF_COLOUR);
	tft.print("EEPROM ID: ");
	tft.println(eeprom.getJEDECID());
	tft.print("EEPROM SIZE(B): ");
	tft.println(eeprom.getCapacity());
	tft.print("SDCARD TYPE: ");
	tft.println(sdcard.type());
	tft.print("SDCARD SIZE(KB): ");
	tft.println(sdvol.blocksPerCluster() * sdvol.clusterCount() / 2);
	tft.setTextColor(TEXT1_COLOUR, BEAM_OFF_COLOUR);

	/* End of Setup */
	delay(2500);
	SerialUSB.println("h: Hold trigger");
	SerialUSB.println("t: Decrease TDiv");
	SerialUSB.println("T: Increase TDiv");
	SerialUSB.println("z: Decrease YZoom");
	SerialUSB.println("Z: Increase YZoom");
	SerialUSB.println("r: Scroll Right");
	SerialUSB.println("l: Scroll Left");
	SerialUSB.println("e: Change Trigger Type");
	SerialUSB.println("y: Decrease Offset");
	SerialUSB.println("Y: Increase Offset");
	SerialUSB.println("g: Decrease Trigger Point");
	SerialUSB.println("G: Increase Trigger Point");
	SerialUSB.println("k: Input Test Signal Frequency (k 1000)");
	SerialUSB.println("c: Increase sampling cycles");
	not_triggered = true;
	tft.fillScreen(BEAM_OFF_COLOUR);
	show_graticule();
	show_labels();
}
void loop() {
	/*
	* the entire capture window is MAX_SAMPLE * sampletime,
	* so for lower frequency signals it's better to lower the sample rate,
	* so that we have a bigger capture window to get the entire signal.
	* Also more ram helps too
	* To lower sample rate there is three options
	* -increase ADC prescaler (only 4 steps)
	* -increase conversion cycle (8 steps)
	* -set up a timer as conversion trigger
	*
	* Basic workflow:
	* 1. Read input from touch or SerialUSB
	* 2. Trigger
	* 3. Fill sample buffer
	* 4. Decimate samples
	* 5. Draw screen / Save buffer to SD / Print on SerialUSB
	*/
	read_touch();
	cmd.readSerial();
	if(!trigger_held) { // check for hold command
		trigger(); // get trigger
		if(!not_triggered) {
			fill_sample_buffer(); // get samples - ~10ms here
			tft_samples_clear(BEAM_OFF_COLOUR); // faster clear screen, just trace the previous samples black
			show_graticule();
			tft_samples(BEAM1_COLOUR);
		}
		else show_graticule();
	}
	frametime = micros() - frametime; // needs to be calculated before show_labels()
	show_labels();
	frametime = micros();
	// dbg_print_on_usb();
	delay(retrigger_delay);
}
/* IRQs: */
void timer2_update_callback() {
	digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Still Alive!!!
}
/* Functions Definitions: */
void read_touch() {
	p = ts.getPoint();
	touch_cnt++;
	pinMode(TOUCH_XM, OUTPUT);
	pinMode(TOUCH_YP, OUTPUT);
	// SerialUSB.print(p.x); // debug
	// SerialUSB.print(","); // debug
	// SerialUSB.println(p.y); // debug
	if(p.x > TOUCH_X_MIN && p.x < TOUCH_X_MAX && p.y > TOUCH_Y_MIN && p.y < TOUCH_Y_MAX && touch_cnt >= 5) {
		touch_cnt = 0;
		p.x = map(p.x, TOUCH_X_MIN, TOUCH_X_MAX, 0, 240);
		p.y = map(p.y, TOUCH_Y_MIN, TOUCH_Y_MAX, 320, 0);
		// SerialUSB.print(p.x); // debug
		// SerialUSB.print(","); // debug
		// SerialUSB.println(p.y); // debug
		// tft.drawPixel(p.x, p.y, BEAM2_COLOUR); // debug
		if(p.x>170 && p.x<190 && p.y>295 && p.y<310) toggle_hold();
		else if(p.x>150 && p.x<170 && p.y>295 && p.y<320) inc_edgetype();
		else if(p.x>20 && p.x<40 && p.y>295 && p.y<320) inc_samplecycle();
		if(p.x<30 && p.y>140 && p.y<170) inc_xzoom();
		if(p.x<30 && p.y>240 && p.y<270) dec_xzoom();
		if(p.x<30 && p.y<30) inc_yzoom();
		if(p.x<30 && p.y>75 && p.y<105) dec_yzoom();
	}
}
void fill_sample_buffer() {
	/* Init ADC: */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // Calling arduino function (in trigger) resets the registers
	// for optimal conversions, adc sample rate must not exceed 2.4msps
	// ~~we have to lower cpu clock from 84 to 72 for higher sample rate~~
	// Turns out 72Mhz is incompatible with 48Mhz for USB to work... (only whole numbers can be set in PLLQ)
	// 84Mhz/2 = 42Mhz -> (3+12)/42Mhz = 0.357us per sample -> ~2.8MSPS
	// 0: F_CPU/2	1: F_CPU/4		2: F_CPU/6		3: F_CPU/8
	ADC1_COMMON->CCR = ((settings[0]&0b11)<<ADC_CCR_ADCPRE_Pos); // 42Mhz (highest)
	ADC1->CR1 |= ADC_CR1_EOCIE;
	ADC1->CR2 |= ADC_CR2_ADON | ADC_CR2_CONT; //external event rising edge tim2 cc2
	ADC1->SQR3 = (0 << 0); // ADC_CH0
	ADC1->SMPR2 = ((settings[1]&0b111)<<ADC_SMPR2_SMP0_Pos); // 3 cycles (lowest)
	ADC1->CR2 |= ADC_CR2_SWSTART; // Start conversions
	sampletime = micros();
	// Note: better to use DMA but can't be arsed really. So, bootleg DMA goes brrr.
	for(int i=0; i<MAX_SAMPLES; i++) {
		while(!(ADC1->SR & ADC_SR_EOC)) {;}
		buffer[i] = ADC1->DR;
	}
	sampletime = (micros() - sampletime) / MAX_SAMPLES;
	ADC1->CR2 &= (ADC_CR2_SWSTART | ADC_CR2_ADON); // Stop ADC
}
void dbg_print_on_usb() {
	for(int i=0; i<MAX_SAMPLES; i++) {
		SerialUSB.print(sampletime, 3);
		SerialUSB.print(",");
		SerialUSB.println(buffer[i]);
		delayMicroseconds(100);
	}
}
void show_graticule() {
	tft.setRotation(PORTRAIT);
	tft.drawRect(0, 0, DISP_H, DISP_W, GRATICULE_COLOUR);
	// Dot grid - ten distinct divisions (9 dots) in both X and Y axis.
	for (uint16_t TicksX = 1; TicksX < 10; TicksX++)
	{
		for (uint16_t TicksY = 1; TicksY < 10; TicksY++)
		{
			tft.drawPixel(TicksX * (DISP_H / 10), TicksY * (DISP_W / 10), GRATICULE_COLOUR);
		}
	}
	// Horizontal and Vertical centre lines 5 ticks per grid square with a longer tick in line with our dots
	for (uint16_t TicksX = 0; TicksX < DISP_W; TicksX += (DISP_H / 50))
	{
		if (TicksX % (DISP_W / 10) > 0)
		{
			tft.drawFastHLine((DISP_H / 2) - 1, TicksX, 3, GRATICULE_COLOUR);
		}
		else
		{
			tft.drawFastHLine((DISP_H / 2) - 3, TicksX, 7, GRATICULE_COLOUR);
		}
	}
	for (uint16_t TicksY = 0; TicksY < DISP_H; TicksY += (DISP_H / 50))
	{
		if (TicksY % (DISP_H / 10) > 0)
		{
			tft.drawFastVLine(TicksY, (DISP_W / 2) - 1, 3, GRATICULE_COLOUR);
		}
		else
		{
			tft.drawFastVLine(TicksY, (DISP_W / 2) - 3, 7, GRATICULE_COLOUR);
		}
	}
}
void show_labels() {
	tft.setRotation(LANDSCAPE);
	tft.setTextColor(TEXT1_COLOUR, BEAM_OFF_COLOUR);
	tft.setTextSize(1);
	if(!trigger_held) {
		tft.setCursor(260, 10);
		tft.print(1000000.0 / frametime);
		tft.print(" fps");
	}
	tft.setCursor(260, 30);
	tft.print(sampletime, 3);
	tft.println(" uS/S");
	tft.setTextSize(2);
	tft.setCursor(290, 50);
	if(!trigger_held) tft.setTextColor(TEXT2_COLOUR, BEAM_OFF_COLOUR);
	else tft.setTextColor(TEXT1_COLOUR, BEAM_OFF_COLOUR);
	tft.print("H");
	tft.setCursor(290, 70);
	switch (trigger_type) {
		case 1:
			tft.print("TN");
			break;
		case 2:
			tft.print("TP");
			break;
		case 3:
			tft.print("NT");
			break;
		default:
			tft.print("TB");
			break;
	}
	tft.setCursor(290, 190);
	tft.print("SC");
	tft.setTextColor(TEXT2_COLOUR, BEAM_OFF_COLOUR);
	tft.setCursor(10, 210);
	tft.print("<<");
	tft.setTextColor(TEXT1_COLOUR, BEAM_OFF_COLOUR);
	tft.print(200.0 / y_zoom_factor); //2.0v per div * 100 / yzoom
	tft.setTextColor(TEXT2_COLOUR, BEAM_OFF_COLOUR);
	tft.print(">>");
	tft.setTextColor(TEXT1_COLOUR, BEAM_OFF_COLOUR);
	tft.setTextSize(1);
	tft.print(" V/Div ");
	tft.setTextSize(2);
	tft.setTextColor(TEXT2_COLOUR, BEAM_OFF_COLOUR);
	tft.print("<<");
	tft.setTextColor(TEXT1_COLOUR, BEAM_OFF_COLOUR);
	tdiv = (end_sample - start_sample) * sampletime * 10.0 / x_zoom_factor;
	if(tdiv >= 1000.0) {
		tdiv /= 1000.0;
		if(tdiv < 10) tft.print("00");
		else tft.print("0");
		tft.print(tdiv, 2);
		tft.setTextColor(TEXT2_COLOUR, BEAM_OFF_COLOUR);
		tft.print(">>");
		tft.setTextColor(TEXT1_COLOUR, BEAM_OFF_COLOUR);
		tft.setTextSize(1);
		tft.print(tdivms);
	}
	else {
		if(tdiv < 10) tft.print("00");
		else if(tdiv < 100) tft.print("0");
		tft.print(tdiv, 2);
		tft.setTextColor(TEXT2_COLOUR, BEAM_OFF_COLOUR);
		tft.print(">>");
		tft.setTextColor(TEXT1_COLOUR, BEAM_OFF_COLOUR);
		tft.setTextSize(1);
		tft.print(tdivus);
	}
	tft.setRotation(PORTRAIT);
}
void tft_samples_clear(uint16_t beam_color) {
	for (signalX = 1; signalX < DISP_W - 2; signalX++) {
		// use saved data to improve speed
		tft.drawLine(dataPlot[signalX - 1], signalX, dataPlot[signalX], signalX + 1, beam_color);
	}
}
void tft_samples(uint16_t beam_color) {
	// calculate first sample
	uint32_t bins = ((end_sample - start_sample) / (DISP_W * x_zoom_factor / 100)) + 1;
	uint32_t buffer_dat_point;
	signalY = ((DISP_H * buffer[0 * bins]) / 4096) * y_zoom_factor / 100 + y_pos;
	dataPlot[0] = signalY * 99 / 100 + 1;
	for (signalX = 1; signalX < DISP_W - 2; signalX++) {
		// Scale our samples to fit our screen. Most scopes increase this in steps of 5,10,25,50,100 250,500,1000 etc
		// Pick the nearest suitable samples for each of our myWidth screen resolution points
		buffer_dat_point = (signalX + 1) * bins;
		if(buffer_dat_point >= MAX_SAMPLES) buffer_dat_point = MAX_SAMPLES-1;
		signalY1 = ((DISP_H * buffer[buffer_dat_point]) / 4096) * y_zoom_factor / 100 + y_pos;
		dataPlot[signalX] = signalY1 * 99 / 100 + 1;
		tft.drawLine(dataPlot[signalX - 1], signalX, dataPlot[signalX], signalX + 1, beam_color);
		signalY = signalY1;
	}
}
// Crude triggering on positive or negative or either change from previous to current sample.
void trigger() {
	not_triggered = true;
	switch (trigger_type) {
	case 1:
		trigger_n();
		break;
	case 2:
		trigger_p();
		break;
	case 3:
		not_triggered = false;
		break;
	default:
		trigger_both();
		break;
	}
}
void trigger_both() {
	trigger_points[0] = analogRead(ADCPIN);
	while (not_triggered) {
		trigger_points[1] = analogRead(ADCPIN);
		if (((trigger_points[1] < trigger_value) && (trigger_points[0] > trigger_value)) ||
			((trigger_points[1] > trigger_value) && (trigger_points[0] < trigger_value))) {
			not_triggered = false;
		}
		trigger_points[0] = trigger_points[1];
	}
}
void trigger_p() {
	trigger_points[0] = analogRead(ADCPIN);
	while (not_triggered) {
		trigger_points[1] = analogRead(ADCPIN);
		if ((trigger_points[1] > trigger_value) && (trigger_points[0] < trigger_value)) {
			not_triggered = false;
		}
		trigger_points[0] = trigger_points[1];
	}
}
void trigger_n() {
	trigger_points[0] = analogRead(ADCPIN);
	while (not_triggered) {
		trigger_points[1] = analogRead(ADCPIN);
		if ((trigger_points[1] < trigger_value) && (trigger_points[0] > trigger_value)) {
			not_triggered = false;
		}
		trigger_points[0] = trigger_points[1];
	}
}
void toggle_hold() { 
	trigger_held ^= 1;
	if (trigger_held) SerialUSB.println("# Hold is on.");
	else SerialUSB.println("# Hold is off.");
}
void dec_xzoom() {
	tft_samples(BEAM_OFF_COLOUR);
	show_graticule();
	if(x_zoom_factor > 10) {
		if(x_zoom_factor > 100) x_zoom_factor -= 100;
		else x_zoom_factor -= 10;
	}
	show_labels();
	tft_samples(BEAM1_COLOUR);
	SerialUSB.print("# X Zoom = ");
	SerialUSB.println(x_zoom_factor);
}
void inc_xzoom() {
	tft_samples(BEAM_OFF_COLOUR);
	show_graticule();
	if(x_zoom_factor < 10000) {
		if(x_zoom_factor < 100) x_zoom_factor += 10;
		else x_zoom_factor += 100;
	}
	show_labels();
	tft_samples(BEAM1_COLOUR);
	SerialUSB.print("# X Zoom = ");
	SerialUSB.println(x_zoom_factor);
}
void dec_yzoom() {
	tft_samples(BEAM_OFF_COLOUR);
	show_graticule();
	if(y_zoom_factor > 10) y_zoom_factor -= 10;
	show_labels();
	tft_samples(BEAM1_COLOUR);
	SerialUSB.print("# Y Zoom = ");
	SerialUSB.println(y_zoom_factor);
}
void inc_yzoom() {
	tft_samples(BEAM_OFF_COLOUR);
	show_graticule();
	if(y_zoom_factor < 1000) y_zoom_factor += 10;
	show_labels();
	tft_samples(BEAM1_COLOUR);
	SerialUSB.print("# Y Zoom = ");
	SerialUSB.println(y_zoom_factor);
}
void scroll_right() {
	tft_samples(BEAM_OFF_COLOUR);
	show_graticule();
	if(start_sample < (end_sample - 1000)) start_sample += 1000;
	show_labels();
	tft_samples(BEAM1_COLOUR);
	SerialUSB.print("# Start Sample = ");
	SerialUSB.println(start_sample);
}
void scroll_left() {
	tft_samples(BEAM_OFF_COLOUR);
	show_graticule();
	if(start_sample > 0) start_sample -= 1000;
	show_labels();
	tft_samples(BEAM1_COLOUR);
	SerialUSB.print("# Start Sample = ");
	SerialUSB.println(start_sample);
}
void inc_edgetype() {
	trigger_type++;
	if(trigger_type>3) trigger_type = 0;
	SerialUSB.print("# Trigger Type = ");
	switch (trigger_type) {
	case 1:
		SerialUSB.println("Negative");
		break;
	case 2:
		SerialUSB.println("Positive");
		break;
	case 3:
		SerialUSB.println("None");
		break;
	default:
		SerialUSB.println("Both");
		break;
	}
}
void dec_ypos() {
	tft_samples(BEAM_OFF_COLOUR);
	show_graticule();
	if(y_pos > -DISP_H+5) y_pos-=5;
	show_labels();
	tft_samples(BEAM1_COLOUR);
	SerialUSB.print("# Offset = ");
	SerialUSB.println(y_pos);
}
void inc_ypos() {
	tft_samples(BEAM_OFF_COLOUR);
	show_graticule();
	if(y_pos < DISP_H-5) y_pos+=5;
	show_labels();
	tft_samples(BEAM1_COLOUR);
	SerialUSB.print("# Offset = ");
	SerialUSB.println(y_pos);
}
void dec_triggerpoint() {
	if(trigger_value > 0) trigger_value -= TRIGGER_STEP;
	SerialUSB.print("# Trigger Value = ");
	SerialUSB.println(trigger_value);
}
void inc_triggerpoint() {
	if(trigger_value < 4096) trigger_value += TRIGGER_STEP;
	SerialUSB.print("# Trigger Value = ");
	SerialUSB.println(trigger_value);
}
void unrecognized(const char *command) { SerialUSB.println("# Command not found.");}
void set_sig_gen_freq() {
	uint32_t freq = 0;
	char *arg;
	//only get upto 999999 hz
	for (int i=0; i<6; i++){
		arg = cmd.next();
		if(arg != NULL) {
			freq = (freq*10) + atoi(arg);
		}
	}
	if(freq == 0 || freq > 90000) freq = 1000;
	test_sig_timer.pause();
	test_sig_timer.setOverflow(freq, HERTZ_FORMAT);
	test_sig_timer.refresh();
	test_sig_timer.resume();
	SerialUSB.print("# Test Signal Frequency(Hz) = ");
	SerialUSB.println(test_sig_timer.getOverflow(HERTZ_FORMAT));
}
void inc_samplecycle() {
	settings[1] += 1;
	settings[1] = settings[1] & 0b111;
	SerialUSB.print("# Sampling Cycles = ");
	switch(settings[1]) {
		case 0:
			SerialUSB.print(3);
		break;
		case 1:
			SerialUSB.print(15);
		break;
		case 2:
			SerialUSB.print(28);
		break;
		case 3:
			SerialUSB.print(56);
		break;
		case 4:
			SerialUSB.print(84);
		break;
		case 5:
			SerialUSB.print(112);
		break;
		case 6:
			SerialUSB.print(144);
		break;
		case 7:
			SerialUSB.print(480);
		break;
	}
	SerialUSB.println(" + 12");
}








#endif