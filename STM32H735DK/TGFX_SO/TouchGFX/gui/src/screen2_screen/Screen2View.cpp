#include <gui/screen2_screen/Screen2View.hpp>
#include <math.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

extern uint8_t bandSelected, fromView1, fromView2, nowTransmittingHW, nowTransmittingSW;
extern float vco_80m, vco_40m, vco_20m;
extern float vco_80m_s, vco_40m_s, vco_20m_s;
extern float vco_80m_dec, vco_40m_dec, vco_20m_dec;
extern uint32_t vco_80m_5351, vco_40m_5351, vco_20m_5351;
extern xQueueHandle messageQ_VFO;
extern xQueueHandle messageQ_Transmit;

struct FreqandBand
{
	uint32_t frequencyValue;
	uint8_t bandValue;
};
extern FreqandBand queueUpdateValue;

Screen2View::Screen2View()
{

}

void Screen2View::setupScreen()
{
	Screen2ViewBase::setupScreen();

	if (bandSelected == 80)
	{
		vco_80m = vco_80m_s + vco_80m_dec;
		slider1.setValueRange(3500, 3800, vco_80m_s);
		slider1.invalidate();
	}
	if (bandSelected == 40)
	{
		vco_40m = vco_40m_s + vco_40m_dec;
		slider1.setValueRange(7000, 7200, vco_40m_s);
		slider1.invalidate();
	}
	if (bandSelected == 20)
	{
		vco_20m = vco_20m_s + vco_20m_dec;
		slider1.setValueRange(14000, 14350, vco_20m_s);
		slider1.invalidate();
	}

	pixelDataWidget.setPosition(70, 165, PIXEL_W, PIXEL_H);
	pixelDataWidget.setPixelData(pixel_data);
	pixelDataWidget.setBitmapFormat(Bitmap::RGB888);
	pixelDataWidget.setAlpha(255);
	pixelDataWidget.setTouchable(true);

	pixelCounter = 0;
	for (int i = 0; i < (PIXEL_H * PIXEL_W); i++)
	{
		pixelCounter++;
		if (pixelCounter <= 3000)
		{
			pixel_data[(i * 3) + 0] = 200;
			pixel_data[(i * 3) + 1] = 100;
			pixel_data[(i * 3) + 2] = 100;

		}
		if (pixelCounter > 3000 && pixelCounter <= 6000)
		{
			pixel_data[(i * 3) + 0] = 100;
			pixel_data[(i * 3) + 1] = 250;
			pixel_data[(i * 3) + 2] = 200;

			if (pixelCounter == 6000)
			{
				pixelCounter = 0;
			}
		}

	}
	add (pixelDataWidget);
	pixelDataWidget.invalidate();
}

void Screen2View::tearDownScreen()
{
	fromView2 = 1;
	Screen2ViewBase::tearDownScreen();
}

void Screen2View::handleTickEvent()
{

	tickcounter++;

	if (tickcounter == 10)
	{
		for (int i = 0; i < (PIXEL_H * PIXEL_W); i++)
		{
			pixelCounter++;
			if (pixelCounter <= 3000)
			{
				pixel_data[(i * 3) + 0] = (3800 - (int) vco_80m_s) / 2 + 105;
				pixel_data[(i * 3) + 1] = 100;
				pixel_data[(i * 3) + 2] = (3800 - (int) vco_80m_s) / 2 + 105;

			}
			if (pixelCounter > 3000 && pixelCounter <= 6000)
			{
				pixel_data[(i * 3) + 0] = (3800 - (int) vco_80m_s) / 2 + 105;
				pixel_data[(i * 3) + 1] = 250;
				pixel_data[(i * 3) + 2] = (3800 - (int) vco_80m_s) / 2 + 105;

				if (pixelCounter == 6000)
				{
					pixelCounter = 0;
				}
			}
			pixelDataWidget.invalidate();

		}
		tickcounter = 0;
	}

	if (counter < 100)
	{
		counter++;
	}
	else
	{
		counter = 0;
	}
	for (uint16_t i = 0; i < 300; i++)
	{
		result = 50 + 50 * sin((((2 * M_PI) + counter / 2) / 300) * i);
		dynamicGraph1.addDataPoint(result);
	}

	if (nowTransmittingHW == 1 && toggleButton1.getState() == pdFALSE)
	{
		toggleButton1.forceState(pdTRUE);
		toggleButton1.invalidate();
	}
	if (nowTransmittingHW == 0 && toggleButton1.getState() == pdTRUE && nowTransmittingSW == 0)
	{
		toggleButton1.forceState(pdFALSE);
		toggleButton1.invalidate();
	}

}

void Screen2View::setValue(int value)
{
	switch (bandSelected)
	{
	case 80:
		vco_80m_s = (float) value;
		if (fromView1)
		{
			fromView1 = 0;
		}
		else
		{
			vco_80m_dec = 0;
		}
		vco_80m = vco_80m_s + vco_80m_dec;
		vco_80m_5351 = 9000000 + (uint32_t) (vco_80m * 1000);
		mustUpdateVFO(vco_80m_5351, bandSelected);
		Unicode::snprintfFloat(textArea2Buffer, TEXTAREA2_SIZE, "%.1f", vco_80m);
		textArea2.invalidate();

		pixelCounter = 0;

		break;
	case 40:
		vco_40m_s = (float) value;
		if (fromView1)
		{
			fromView1 = 0;
		}
		else
		{
			vco_40m_dec = 0;
		}
		vco_40m = vco_40m_s + vco_40m_dec;
		vco_40m_5351 = 9000000 + (uint32_t) (vco_40m * 1000);
		mustUpdateVFO(vco_40m_5351, bandSelected);
		Unicode::snprintfFloat(textArea2Buffer, TEXTAREA2_SIZE, "%.1f", vco_40m);
		textArea2.invalidate();
		break;
	case 20:
		vco_20m_s = (float) value;
		if (fromView1)
		{
			fromView1 = 0;
		}
		else
		{
			vco_20m_dec = 0;
		}
		vco_20m = vco_20m_s + vco_20m_dec;
		vco_20m_5351 = 9000000 + (uint32_t) (vco_20m * 1000);
		mustUpdateVFO(vco_20m_5351, bandSelected);
		Unicode::snprintfFloat(textArea2Buffer, TEXTAREA2_SIZE, "%.1f", vco_20m);
		textArea2.invalidate();

		break;
	}
}

void Screen2View::mustUpdateVFO(uint32_t frequency, uint8_t band)
{
	queueUpdateValue.frequencyValue = frequency;
	queueUpdateValue.bandValue = band;
	if (uxQueueMessagesWaiting(messageQ_VFO) == 0)
	{
		xQueueGenericSend(messageQ_VFO, &queueUpdateValue, 0, 0);
	}
}

void Screen2View::decreaseVCO()
{
	switch (bandSelected)
	{
	case 80:

		vco_80m_dec = vco_80m_dec - 0.1;
		vco_80m = vco_80m_s + vco_80m_dec;
		vco_80m_5351 = 9000000 + (uint32_t) (vco_80m * 1000);
		mustUpdateVFO(vco_80m_5351, bandSelected);
		Unicode::snprintfFloat(textArea2Buffer, TEXTAREA2_SIZE, "%.1f", vco_80m);
		textArea2.invalidate();
		break;
	case 40:
		vco_40m_dec = vco_40m_dec - 0.1;
		vco_40m = vco_40m_s + vco_40m_dec;
		vco_40m_5351 = 9000000 + (uint32_t) (vco_40m * 1000);
		mustUpdateVFO(vco_40m_5351, bandSelected);
		Unicode::snprintfFloat(textArea2Buffer, TEXTAREA2_SIZE, "%.1f", vco_40m);
		textArea2.invalidate();
		break;
	case 20:
		vco_20m_dec = vco_20m_dec - 0.1;
		vco_20m = vco_20m_s + vco_20m_dec;
		vco_20m_5351 = 9000000 + (uint32_t) (vco_20m * 1000);
		mustUpdateVFO(vco_20m_5351, bandSelected);
		Unicode::snprintfFloat(textArea2Buffer, TEXTAREA2_SIZE, "%.1f", vco_20m);
		textArea2.invalidate();
		break;
	}
}

void Screen2View::increaseVCO()
{
	switch (bandSelected)
	{
	case 80:
		vco_80m_dec = vco_80m_dec + 0.1;
		vco_80m = vco_80m_s + vco_80m_dec;
		vco_80m_5351 = 9000000 + (uint32_t) (vco_80m * 1000);
		mustUpdateVFO(vco_80m_5351, bandSelected);
		Unicode::snprintfFloat(textArea2Buffer, TEXTAREA2_SIZE, "%.1f", vco_80m);
		textArea2.invalidate();
		break;
	case 40:
		vco_40m_dec = vco_40m_dec + 0.1;
		vco_40m = vco_40m_s + vco_40m_dec;
		vco_40m_5351 = 9000000 + (uint32_t) (vco_40m * 1000);
		mustUpdateVFO(vco_40m_5351, bandSelected);
		Unicode::snprintfFloat(textArea2Buffer, TEXTAREA2_SIZE, "%.1f", vco_40m);
		textArea2.invalidate();
		break;
	case 20:
		vco_20m_dec = vco_20m_dec + 0.1;
		vco_20m = vco_20m_s + vco_20m_dec;
		vco_20m_5351 = 9000000 + (uint32_t) (vco_20m * 1000);
		mustUpdateVFO(vco_20m_5351, bandSelected);
		Unicode::snprintfFloat(textArea2Buffer, TEXTAREA2_SIZE, "%.1f", vco_20m);
		textArea2.invalidate();
		break;
	}
}

void Screen2View::isTransmitting()
{
	if (toggleButton1.getState() && nowTransmittingHW == 0)
	{
		nowTransmittingSW = 1;
		mustUpdateTransmit(nowTransmittingSW);
		toggleButton1.invalidate();
	}
	else if (toggleButton1.getState() == pdFALSE && nowTransmittingHW == 0)
	{
		nowTransmittingSW = 0;
		mustUpdateTransmit(nowTransmittingSW);
		toggleButton1.invalidate();
	}
}

void Screen2View::mustUpdateTransmit(uint8_t boolTransmit)
{
	if (boolTransmit)
	{
		xQueueGenericSend(messageQ_Transmit, &boolTransmit, 0, 0);
	}
	else
	{
		xQueueGenericSend(messageQ_Transmit, &boolTransmit, 0, 0);
	}
}
