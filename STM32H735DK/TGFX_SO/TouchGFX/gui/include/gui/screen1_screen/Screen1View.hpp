#ifndef SCREEN1VIEW_HPP
#define SCREEN1VIEW_HPP

#include <gui_generated/screen1_screen/Screen1ViewBase.hpp>
#include <gui/screen1_screen/Screen1Presenter.hpp>

extern uint8_t input, output, bandSelected;
extern float vco_20m;

class Screen1View: public Screen1ViewBase
{
public:
	Screen1View();
	virtual ~Screen1View()
	{
	}
	virtual void setupScreen();
	virtual void tearDownScreen();
	virtual void txIsClicked();
	virtual void selected80m();
	virtual void selected40m();
	virtual void selected20m();
	virtual void setValue(int value);
	virtual void increaseBFO();
	virtual void decreaseBFO();
	virtual void mustUpdateBFO(uint32_t frequency, uint8_t band);
protected:
};

#endif // SCREEN1VIEW_HPP
