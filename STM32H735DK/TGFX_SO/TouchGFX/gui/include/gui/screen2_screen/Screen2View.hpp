#ifndef SCREEN2VIEW_HPP
#define SCREEN2VIEW_HPP

#include <gui_generated/screen2_screen/Screen2ViewBase.hpp>
#include <gui/screen2_screen/Screen2Presenter.hpp>
#include <touchgfx/widgets/PixelDataWidget.hpp>

#define PIXEL_W 300
#define PIXEL_H 100

class Screen2View: public Screen2ViewBase
{
public:
	Screen2View();
	virtual ~Screen2View()
	{
	}
	virtual void setupScreen();
	virtual void tearDownScreen();
	virtual void handleTickEvent();
	virtual void setValue(int value);
	virtual void increaseVCO();
	virtual void decreaseVCO();
	virtual void isTransmitting();
	virtual void mustUpdateVFO(uint32_t frequency, uint8_t band);
	virtual void mustUpdateTransmit(uint8_t boolTransmit);

protected:
	int counter;
	float result;
	uint16_t tickcounter = 0, pixelCounter = 0;
	unsigned char pixel_data[PIXEL_H * PIXEL_W * 3];
	touchgfx::PixelDataWidget pixelDataWidget;
};

#endif // SCREEN2VIEW_HPP
