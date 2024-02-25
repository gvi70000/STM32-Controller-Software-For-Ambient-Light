#include "APA102.h"
#include "spi.h"
#include "usart.h"
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
static const uint8_t sineTable[SINE_TABLE_SIZE] = {
	0, 1, 2, 3, 5, 6, 7, 9, 10, 11, 12, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 31,
	31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0};
static uint8_t SpiSendFrame[BUFFER_SIZE];
uint8_t effectDone = 1;
uint8_t prevEffect = 0;
// Set up the buffer for the strip
void APA_init(void) {
	// Set the value for brightness
	memset(SpiSendFrame, LED_INIT, BUFFER_SIZE);
	// Add the start frame
	memset(SpiSendFrame, 0x00, START_FRAME_SIZE);
	// Add the end frame
	memset(&SpiSendFrame[END_POSITION], 0xFF, END_FRAME_SIZE);
}

//Set RGB color of a specified LED
void APA_setColor(uint16_t led, uint8_t red, uint8_t green, uint8_t blue){
	uint16_t pos = 1 + getPosition(led);
	SpiSendFrame[pos] = blue;
	SpiSendFrame[pos + 1] = green;
	SpiSendFrame[pos + 2] = red;
}

//Set RGB color and Brightness of a specified LED
void APA_setColorBright(uint16_t led, uint8_t red, uint8_t green, uint8_t blue, uint8_t bright){
	uint16_t pos = getPosition(led);
	SpiSendFrame[pos] = LED_INIT | bright;
	SpiSendFrame[pos + 1] = blue;
	SpiSendFrame[pos + 2] = green;
	SpiSendFrame[pos + 3] = red;
}

//Set RGB color all LEDs
void APA_setAllColor(uint8_t red, uint8_t green, uint8_t blue){
	for (uint16_t led = 0; led < LED_COUNT; led++)	{
		APA_setColor(led, red, green, blue);
	}
}

//Set RGB color of a specified LED in 24 bit format
void APA_setRGB(uint16_t led, uint32_t rgb, uint8_t bright) {
	uint16_t pos = getPosition(led);
	SpiSendFrame[pos] = LED_INIT | bright;
	SpiSendFrame[pos + 3] = (uint8_t)(rgb);
	SpiSendFrame[pos + 2] = (uint8_t)(rgb >> 8);
	SpiSendFrame[pos + 1] = (uint8_t)(rgb >> 16);
}
//Set RGB color of all LEDs in 24 bit format
void APA_setAllRGB(uint32_t rgb, uint8_t bright){
	for (uint16_t led = 0; led < LED_COUNT; led++)	{
		APA_setRGB(led, rgb, bright);
	}
}

//Set brightness of a specified LED
void APA_setLedBrightness(uint16_t led, uint8_t bright) {
	SpiSendFrame[getPosition(led)] = LED_INIT | bright;
}

//Set brightness for all LEDs
void APA_setAllBrightness(uint8_t intensity) {
	for (uint8_t led = 0; led < LED_COUNT; led++)	{
		APA_setLedBrightness(led, intensity);
	}
}

//Turn On a specified LED
void APA_setLedOff(uint16_t led) {
	SpiSendFrame[getPosition(led)] =  LED_INIT;
}

//Turn Off a specified LED
void APA_setLedOn(uint16_t led) {
	SpiSendFrame[getPosition(led)] =  LED_GLOBAL;
}

//Turns Off both strips
void APA_AllOff(){
	APA_setAllColor(0, 0, 0);
	APA_update(0);
	APA_update(1);
}

//Send the buffer data to LED strip on SPI1 or 2
void APA_update(uint8_t crtLED_Strip) {
	SPI_HandleTypeDef crtSPI;
	if(crtLED_Strip) {
		crtSPI = hspi1;
	} else {
		crtSPI = hspi2;
	}
	// send spi frame with all led values
	HAL_SPI_Transmit(&crtSPI, (uint8_t *)&SpiSendFrame, BUFFER_SIZE, TIME_OUT);
}

//Check if we got new data from MQTT
static uint8_t checkForUpdates(){
	if(prevEffect != frame_MQTT[Effect]){
		prevEffect = frame_MQTT[Effect];
		APA_setAllBrightness(frame_MQTT[Intensity]);
	}
	if (ESP_Ready) {
		APA_setAllBrightness(frame_MQTT[Intensity]);
	}
	if(!frame_MQTT[Power]) {
		effectDone = 1;
		return 1;
	}
	return 0;
}

//Get a color from color wheel
static uint8_t* ColorWheel(uint8_t WheelPos) {
    static uint8_t c[3];
    WheelPos = 255 - WheelPos;
    if (WheelPos < 85) {
        c[0] = 255 - WheelPos * 3;
        c[1] = 0;
        c[2] = WheelPos * 3;
    }
    else if (WheelPos < 170) {
        WheelPos -= 85;
        c[0] = 0;
        c[1] = WheelPos * 3;
        c[2] = 255 - WheelPos * 3;
    } else {
        WheelPos -= 170;
        c[0] = WheelPos * 3;
        c[1] = 255 - WheelPos * 3;
        c[2] = 0;
    }
    return c;
}


void FluidRainbowCycle(uint16_t duration_ms) {
	uint16_t steps = duration_ms / 10; // Assuming a step every 10 ms
	uint8_t segment_size = LED_COUNT / 7;
	for (uint16_t step = 0; step < steps; ++step) {
		if (checkForUpdates()) {
			return;
		}
		// Iterate over each segment
		for (uint8_t segment = 0; segment < 7; ++segment) {
			// Calculate the color for the current segment
			uint8_t gradient = (step * 256 / steps + segment * 30) % 256;
			uint8_t *color = ColorWheel(gradient);
			// Shift the segment along the strip
			for (uint8_t i = 0; i < segment_size; ++i) {
				uint16_t index = segment * segment_size + i;
				// Calculate the shifted index
				uint16_t shiftedIndex = (index + step) % LED_COUNT;
				APA_setColor(shiftedIndex, color[0], color[1], color[2]);
			}
			// Update both segments
			APA_update(0);
			APA_update(1);
		}
		// Introduce a slight delay to control the speed of the effect
		HAL_Delay(10);
	}
	effectDone = 1;
}

// Adjust the divisor in the if (rand() % 10 == 0) statement to control the density of the snowflakes.
// A smaller divisor will result in a denser snowfall.
void SnowEffect(uint16_t duration_ms) {
	uint16_t pixel;
	APA_setAllColor(255, 255, 255);
  APA_setAllBrightness(0);
  pixel = rand() % 144;
	APA_setLedBrightness(pixel, 31);
	APA_update(0);
	HAL_Delay(20);
	APA_setLedBrightness(pixel, 0);
	APA_update(0);
	HAL_Delay(20);
	
  pixel = rand() % 144;
	APA_setLedBrightness(pixel, 31);
	APA_update(1);
	HAL_Delay(20);
	APA_setLedBrightness(pixel, 0);
	APA_update(1);
  HAL_Delay(duration_ms);
	effectDone = 1;
}

void PulseEffect(uint16_t duration_ms) {
	uint16_t steps = duration_ms / 10;
	for (uint16_t step = 0; step < steps; ++step) {
		if (checkForUpdates()) {
			return;
		}
		// Calculate brightness based on sineTable to create a pulsating effect
		uint8_t brightness = sineTable[(int)((float)step / steps * SINE_TABLE_SIZE)];
		// Set color for the left side of the strip
		APA_setAllColor((brightness * frame_MQTT[LeftRed]) / MAX_BRIGHTNESS,
                        (brightness * frame_MQTT[LeftGreen]) / MAX_BRIGHTNESS,
                        (brightness * frame_MQTT[LeftBlue]) / MAX_BRIGHTNESS);
		APA_update(0);
		// Set color for the right side of the strip
		APA_setAllColor((brightness * frame_MQTT[RightRed]) / MAX_BRIGHTNESS,
                        (brightness * frame_MQTT[RightGreen]) / MAX_BRIGHTNESS,
                        (brightness * frame_MQTT[RightBlue]) / MAX_BRIGHTNESS);
			APA_update(1);
			HAL_Delay(10);
	}
	effectDone = 1;
}

void FlashingEffect(uint16_t duration_ms) {
	uint16_t steps = duration_ms / 10; // Assuming a step every 10 ms
	for (uint16_t step = 0; step < steps && frame_MQTT[Power]; ++step) {
		if(checkForUpdates()) {
			return;
		}
		if (step % 2 == 0) {
			APA_setAllColor(frame_MQTT[LeftRed], frame_MQTT[LeftGreen], frame_MQTT[LeftBlue]);
		} else {
			APA_setAllColor(0, 0, 0);
		}
		APA_update(0);
		if (step % 2 == 0) {
			APA_setAllColor(frame_MQTT[RightRed], frame_MQTT[RightGreen], frame_MQTT[RightBlue]);
		} else {
			APA_setAllColor(0, 0, 0);
		}
		APA_update(1);
		HAL_Delay(10);
	}
	effectDone = 1;
}

// Function to create a fading effect
void FadingEffect(uint16_t duration_ms) {
	uint16_t steps = duration_ms / 25; // Assuming a step every 10 ms
	for (uint16_t step = 0; step < steps && frame_MQTT[Power]; ++step) {
		if (checkForUpdates()) {
			return;
		}
		// Calculate the current color based on the step
		uint8_t gradient = (step * 256 / steps) % 256;
		uint8_t *color = ColorWheel(gradient);
		// Set the calculated color with fading effect for each LED in the strip
		for (uint16_t i = 0; i < LED_COUNT; ++i) {
			// Calculate the fading effect based on the current step
			uint8_t fadingFactor = (steps - step) * 31 / steps;
			// Apply fading to each channel
			uint8_t fadedRed = (color[0] * fadingFactor) / 31;
			uint8_t fadedGreen = (color[1] * fadingFactor) / 31;
			uint8_t fadedBlue = (color[2] * fadingFactor) / 31;
			APA_setColor(i, fadedRed, fadedGreen, fadedBlue);
		}
		// Update both segments
		APA_update(0);
		APA_update(1);
		HAL_Delay(25);
	}
	effectDone = 1;
}

// Function to create a color wipe effect with changing brightness
void ColorWipe(uint16_t duration_ms) {
	const uint16_t steps = duration_ms / 10;
	for (uint16_t step = 0; step < steps; ++step) {
		if (checkForUpdates()) {
			return;
		}
		// Calculate the color based on a smooth gradient for the rainbow effect
		uint8_t gradient = (step * 256 / steps) % 256;
		uint8_t *color = ColorWheel(gradient);
		// Set the color for each LED in the strip
		for (uint16_t i = 0; i < LED_COUNT; ++i) {
			// Calculate the brightness based on a sine wave
			float brightness = sineTable[(int)((float)step / (steps / 2) * SINE_TABLE_SIZE)];
			// Scale brightness to a range from 0 to 31
			uint8_t scaledBrightness = (uint8_t)(brightness * MAX_BRIGHTNESS);
			// Set the color and brightness for each LED
			APA_setColorBright(i, color[0], color[1], color[2], scaledBrightness);
		}
		// Update the LED strip for both segments
		APA_update(0);
		APA_update(1);
		// Introduce a slight delay to control the speed of the effect
		HAL_Delay(25);
	}
	effectDone = 1;
}

void TheaterChase(uint8_t red, uint8_t green, uint8_t blue, uint16_t duration_ms) {
	uint16_t steps = duration_ms / (10 * 3); // Adjusted the total steps for a smoother effect
	uint8_t gapSize = 3;
	for (uint16_t j = 0; j < 10; ++j) {
		for (uint16_t q = 0; q < gapSize; ++q) {
			if (checkForUpdates()) {
				return;
			}
			// Set color for every third LED in a smooth gradient
			for (uint16_t i = 0; i < LED_COUNT; i += 3 + gapSize) {
				// Calculate the intermediate color for the wipe effect
				uint8_t stepRed = (red * (i + q + 1)) / LED_COUNT;
				uint8_t stepGreen = (green * (i + q + 1)) / LED_COUNT;
				uint8_t stepBlue = (blue * (i + q + 1)) / LED_COUNT;
				APA_setColor(i + q, stepRed, stepGreen, stepBlue);
			}
			// Update both segments
			APA_update(0);
			APA_update(1);
			HAL_Delay(steps);
			// Turn off every third LED
			for (uint16_t i = 0; i < LED_COUNT; i += 3 + gapSize) {
				APA_setColor(i + q, 0, 0, 0);
			}
		}
	}
	effectDone = 1;
}

// Function to create a breathing effect on an APA102 LED strip
void BreathEffect(uint16_t duration_ms) {
	const uint16_t steps = duration_ms / 10;
	const uint16_t halfSteps = steps / 2;
	for (uint16_t step = 0; step < steps; ++step) {
		if (checkForUpdates()) {
			return;
		}
		// Calculate the brightness based on the predefined sine table
		uint8_t brightness = sineTable[(int)((float)step / halfSteps * SINE_TABLE_SIZE)];
		// Iterate over each LED in the strip
		for (uint16_t i = 0; i < LED_COUNT; ++i) {
			// Calculate the distance from the center
			float distanceFromCenter = abs(LED_COUNT / 2 - i);
			// Scale brightness based on the distance from the center
			uint8_t scaledBrightness = (uint8_t)(brightness * (1 - distanceFromCenter / (LED_COUNT / 2)));
			// Set the color and brightness for each LED
			APA_setColorBright(i, frame_MQTT[LeftRed], frame_MQTT[LeftGreen], frame_MQTT[LeftBlue], scaledBrightness);
		}
		// Update the LED strip for both segments
		APA_update(0);
		APA_update(1);
		// Introduce a slight delay to control the speed of the effect
		HAL_Delay(30);
	}
	effectDone = 1;
}

// Function to create a wave effect on an APA102 LED strip
void WaveEffect(uint16_t duration_ms) {
	const uint16_t steps = duration_ms / 10;
	const uint8_t segment_size = LED_COUNT / 2;
	for (uint16_t step = 0; step < steps; ++step) {
		if (checkForUpdates()) {
			return;
		}
		// Calculate the position of the wave peak based on the step using sineTable
		float position = segment_size * sineTable[(int)((float)step / steps * SINE_TABLE_SIZE)];
		// Iterate over each LED in the strip
		for (uint16_t i = 0; i < LED_COUNT; ++i) {
			// Calculate the distance from the wave position
			float distanceFromWave = fabs(position - i);
			// Scale brightness based on the distance from the wave
			uint8_t brightness = (uint8_t)(31 * (1 - distanceFromWave / segment_size));
			// Set the color and brightness for each LED
			APA_setColorBright(i, frame_MQTT[LeftRed], frame_MQTT[LeftGreen], frame_MQTT[LeftBlue], brightness);
		}
		// Update the LED strip for both segments
		APA_update(0);
		APA_update(1);
		// Introduce a slight delay to control the speed of the effect
		HAL_Delay(10);
	}
	effectDone = 1;
}