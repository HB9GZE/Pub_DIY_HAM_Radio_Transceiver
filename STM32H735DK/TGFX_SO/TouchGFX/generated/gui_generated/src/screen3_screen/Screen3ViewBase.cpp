/*********************************************************************************/
/********** THIS FILE IS GENERATED BY TOUCHGFX DESIGNER, DO NOT MODIFY ***********/
/*********************************************************************************/
#include <gui_generated/screen3_screen/Screen3ViewBase.hpp>
#include <touchgfx/Color.hpp>
#include "BitmapDatabase.hpp"

Screen3ViewBase::Screen3ViewBase() :
    buttonCallback(this, &Screen3ViewBase::buttonCallbackHandler)
{

    __background.setPosition(0, 0, 480, 272);
    __background.setColor(touchgfx::Color::getColorFrom24BitRGB(0, 0, 0));

    box1.setPosition(0, 0, 480, 272);
    box1.setColor(touchgfx::Color::getColorFrom24BitRGB(232, 227, 207));

    button1.setXY(427, 205);
    button1.setBitmaps(touchgfx::Bitmap(BITMAP_RIGHT_ID), touchgfx::Bitmap(BITMAP_RIGHT_ID));
    button1.setAction(buttonCallback);

    add(__background);
    add(box1);
    add(button1);
}

void Screen3ViewBase::setupScreen()
{

}

void Screen3ViewBase::buttonCallbackHandler(const touchgfx::AbstractButton& src)
{
    if (&src == &button1)
    {
        //Interaction1
        //When button1 clicked change screen to Screen2
        //Go to Screen2 with no screen transition
        application().gotoScreen2ScreenNoTransition();
    }
}