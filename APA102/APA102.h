#ifndef __APA102_H
#define __APA102_H
#ifdef __cplusplus
 extern "C" {
#endif

	#include <stdint.h>
	#define LED_INIT					0xE0
	#define LED_GLOBAL				0x1F	
	#define LED_COUNT					143										//Number of LEDs in strip
	#define FirstThird				48										//LED end position of first third of the strip
	#define SecondThird				95										//LED end position of second third of the strip
	#define FRAME_SIZE				4*LED_COUNT						//Number of LEDs in strip
	#define START_FRAME_SIZE	4											//0x00, 0x00, 0x00, 0x00
	#define END_FRAME_SIZE		(LED_COUNT + 15)/16		//
	#define END_POSITION			(START_FRAME_SIZE + FRAME_SIZE)
	#define BUFFER_SIZE				(END_POSITION + END_FRAME_SIZE)
	#define TIME_OUT					1000
	#define PI								3.14159265358979323846
	#define MAX_BRIGHTNESS		31
	#define SINE_TABLE_SIZE		72
	// Define a macro for the maximum of two values
	#define MAX(a, b) ((a) > (b) ? (a) : (b))
	#define MIN(a, b) (((a) < (b)) ? (a) : (b))
	// Because we have 4 bytes for start frame we need to offset
	#define getPosition(pos)	(4*(pos + 1))

	extern uint8_t effectDone;
	void APA_init(void);
	void APA_setColor(uint16_t led, uint8_t red, uint8_t green, uint8_t blue);
	void APA_setColorBright(uint16_t led, uint8_t red, uint8_t green, uint8_t blue, uint8_t bright);
	void APA_setAllColor(uint8_t red, uint8_t green, uint8_t blue);
	void APA_setRGB(uint16_t led, uint32_t rgb, uint8_t bright);
	void APA_setAllRGB(uint32_t rgb, uint8_t bright);
	void APA_setllumination(uint8_t intensity);
	void APA_setLedIllumination(uint16_t led, uint8_t intensity);
	void APA_setAllBrightness(uint8_t intensity);
	void APA_setLedOff(uint16_t led);
	void APA_setLedOn(uint16_t led);
	void APA_update(uint8_t crtLED_Strip);
	void APA_AllOff();
	//Light effects functions
	void FluidRainbowCycle(uint16_t duration_ms);
	void SnowEffect(uint16_t duration_ms);
	void PulseEffect(uint16_t duration_ms);
	void FlashingEffect(uint16_t duration_ms);
	void FadingEffect(uint16_t duration_ms);
	void ColorWipe(uint16_t duration_ms);
	void TheaterChase(uint8_t red, uint8_t green, uint8_t blue, uint16_t duration_ms);
	void BreathEffect(uint16_t duration_ms);
	void WaveEffect(uint16_t duration_ms);
#ifdef __cplusplus
}
#endif
#endif /*__ APA102_H */

