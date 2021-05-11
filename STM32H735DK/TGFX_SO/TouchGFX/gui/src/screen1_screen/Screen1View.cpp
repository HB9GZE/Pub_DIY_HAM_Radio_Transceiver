#include <gui/screen1_screen/Screen1View.hpp>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

extern uint8_t input, output, bandSelected, fromView1, fromView2, bandHasChanged;
extern float bfo_80m, bfo_40m, bfo_20m;
extern float bfo_80m_s, bfo_40m_s, bfo_20m_s;
extern float bfo_80m_dec, bfo_40m_dec, bfo_20m_dec;
extern uint32_t bfo_80m_5351, bfo_40m_5351, bfo_20m_5351;
uint8_t fromBandSelect = 0;
extern xQueueHandle messageQ_BFO;

struct FreqandBand
{
	uint32_t frequencyValue;
	uint8_t bandValue;
};
extern FreqandBand queueUpdateValue;

Screen1View::Screen1View()
{

}

void Screen1View::setupScreen()
{
	Screen1ViewBase::setupScreen();

	if (bandSelected == 80)
	{
		bfo_80m = bfo_80m_s + bfo_80m_dec;
		toggleButton1.forceState(true);
		toggleButton2.forceState(false);
		toggleButton3.forceState(false);
		slider1.setValue(bfo_80m_s);
		slider1.invalidate();
	}
	if (bandSelected == 40)
	{
		bfo_40m = bfo_40m_s + bfo_40m_dec;
		toggleButton1.forceState(false);
		toggleButton2.forceState(true);
		toggleButton3.forceState(false);
		slider1.setValue(bfo_40m_s);
		slider1.invalidate();
	}
	if (bandSelected == 20)
	{
		bfo_20m = bfo_20m_s + bfo_20m_dec;
		toggleButton1.forceState(false);
		toggleButton2.forceState(false);
		toggleButton3.forceState(true);
		slider1.setValue(bfo_20m_s);
		slider1.invalidate();
	}
}

void Screen1View::tearDownScreen()
{
	fromView1 = 1;
	Screen1ViewBase::tearDownScreen();
}

void Screen1View::txIsClicked()
{

}

void Screen1View::selected80m()
{
	bandSelected = 80;
	//bandHasChanged = 1;
	fromBandSelect = 1;
	slider1.setValue(bfo_80m_s);
	slider1.invalidate();
	bfo_80m = bfo_80m_s + bfo_80m_dec;
	Unicode::snprintfFloat(textArea1Buffer, TEXTAREA1_SIZE, "%.1f", bfo_80m);
	textArea1.invalidate();
	bfo_80m_5351 = (uint32_t)(bfo_80m * 1000);
	mustUpdateBFO(bfo_80m_5351, bandSelected);
}

void Screen1View::selected40m()
{
	bandSelected = 40;
	//bandHasChanged = 1;
	fromBandSelect = 1;
	slider1.setValue(bfo_40m_s);
	slider1.invalidate();
	bfo_40m = bfo_40m_s + bfo_40m_dec;
	Unicode::snprintfFloat(textArea1Buffer, TEXTAREA1_SIZE, "%.1f", bfo_40m);
	textArea1.invalidate();
	bfo_40m_5351 = (uint32_t)(bfo_40m * 1000);
	mustUpdateBFO(bfo_40m_5351, bandSelected);
}

void Screen1View::selected20m()
{
	bandSelected = 20;
	//bandHasChanged = 1;
	fromBandSelect = 1;
	slider1.setValue(bfo_20m_s);
	slider1.invalidate();
	bfo_20m = bfo_20m_s + bfo_20m_dec;
	Unicode::snprintfFloat(textArea1Buffer, TEXTAREA1_SIZE, "%.1f", bfo_20m);
	textArea1.invalidate();
	bfo_20m_5351 = (uint32_t)(bfo_20m * 1000);
	mustUpdateBFO(bfo_20m_5351, bandSelected);
}

void Screen1View::setValue(int value)
{
	switch (bandSelected)
	{
	case 80:
		bfo_80m_s = (float) value;
		if (fromView2 || fromBandSelect)
		{
			fromView2 = 0;
			fromBandSelect = 0;
		}
		else
		{
			bfo_80m_dec = 0;
		}
		bfo_80m = bfo_80m_s + bfo_80m_dec;
		Unicode::snprintfFloat(textArea1Buffer, TEXTAREA1_SIZE, "%.1f", bfo_80m);
		textArea1.invalidate();
		bfo_80m_5351 = (uint32_t)(bfo_80m * 1000);
		mustUpdateBFO(bfo_80m_5351, bandSelected);
		break;

	case 40:
		bfo_40m_s = (float) value;
		if (fromView2 || fromBandSelect)
		{
			fromView2 = 0;
			fromBandSelect = 0;
		}
		else
		{
			bfo_40m_dec = 0;
		}
		bfo_40m = bfo_40m_s + bfo_40m_dec;
		Unicode::snprintfFloat(textArea1Buffer, TEXTAREA1_SIZE, "%.1f", bfo_40m);
		textArea1.invalidate();
		bfo_40m_5351 = (uint32_t)(bfo_40m * 1000);
		mustUpdateBFO(bfo_40m_5351, bandSelected);
		break;

	case 20:
		bfo_20m_s = (float) value;
		if (fromView2 || fromBandSelect)
		{
			fromView2 = 0;
			fromBandSelect = 0;
		}
		else
		{
			bfo_20m_dec = 0;
		}
		bfo_20m = bfo_20m_s + bfo_20m_dec;
		Unicode::snprintfFloat(textArea1Buffer, TEXTAREA1_SIZE, "%.1f", bfo_20m);
		textArea1.invalidate();
		bfo_20m_5351 = (uint32_t)(bfo_20m * 1000);
		mustUpdateBFO(bfo_20m_5351, bandSelected);
		break;
	}
}

void Screen1View::mustUpdateBFO(uint32_t frequency, uint8_t band)
{
	queueUpdateValue.frequencyValue = frequency;
	queueUpdateValue.bandValue = band;
	if (uxQueueMessagesWaiting(messageQ_BFO) == 0)
	{
		xQueueGenericSend(messageQ_BFO, &queueUpdateValue, 0, 0);
	}
}

void Screen1View::decreaseBFO()
{
	switch (bandSelected)
	{
	case 80:

		bfo_80m_dec = bfo_80m_dec - 0.1;
		bfo_80m = bfo_80m_s + bfo_80m_dec;
		bfo_80m_5351 = (uint32_t)(bfo_80m * 1000);
		mustUpdateBFO(bfo_80m_5351, bandSelected);
		Unicode::snprintfFloat(textArea1Buffer, TEXTAREA1_SIZE, "%.1f", bfo_80m);
		textArea1.invalidate();
		break;
	case 40:
		bfo_40m_dec = bfo_40m_dec - 0.1;
		bfo_40m = bfo_40m_s + bfo_40m_dec;
		bfo_40m_5351 = (uint32_t)(bfo_40m * 1000);
		mustUpdateBFO(bfo_40m_5351, bandSelected);
		Unicode::snprintfFloat(textArea1Buffer, TEXTAREA1_SIZE, "%.1f", bfo_40m);
		textArea1.invalidate();
		break;
	case 20:
		bfo_20m_dec = bfo_20m_dec - 0.1;
		bfo_20m = bfo_20m_s + bfo_20m_dec;
		bfo_20m_5351 = (uint32_t)(bfo_20m * 1000);
		mustUpdateBFO(bfo_20m_5351, bandSelected);
		Unicode::snprintfFloat(textArea1Buffer, TEXTAREA1_SIZE, "%.1f", bfo_20m);
		textArea1.invalidate();
		break;
	}
}

void Screen1View::increaseBFO()
{
	switch (bandSelected)
	{
	case 80:

		bfo_80m_dec = bfo_80m_dec + 0.1;
		bfo_80m = bfo_80m_s + bfo_80m_dec;
		bfo_80m_5351 = (uint32_t)(bfo_80m * 1000);
		mustUpdateBFO(bfo_80m_5351, bandSelected);
		Unicode::snprintfFloat(textArea1Buffer, TEXTAREA1_SIZE, "%.1f", bfo_80m);
		textArea1.invalidate();
		break;
	case 40:
		bfo_40m_dec = bfo_40m_dec + 0.1;
		bfo_40m = bfo_40m_s + bfo_40m_dec;
		bfo_40m_5351 = (uint32_t)(bfo_40m * 1000);
		mustUpdateBFO(bfo_40m_5351, bandSelected);
		Unicode::snprintfFloat(textArea1Buffer, TEXTAREA1_SIZE, "%.1f", bfo_40m);
		textArea1.invalidate();
		break;
	case 20:

		bfo_20m_dec = bfo_20m_dec + 0.1;
		bfo_20m = bfo_20m_s + bfo_20m_dec;
		bfo_20m_5351 = (uint32_t)(bfo_20m * 1000);
		mustUpdateBFO(bfo_20m_5351, bandSelected);
		Unicode::snprintfFloat(textArea1Buffer, TEXTAREA1_SIZE, "%.1f", bfo_20m);
		textArea1.invalidate();
		break;
	}
}

