/**
  ******************************************************************************
  * This file is part of the TouchGFX 4.16.0 distribution.
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

#include <platform/hal/simulator/sdl2/HALSDL2.hpp>

namespace touchgfx
{
uint16_t HALSDL2::icon[] =
{
    0x0000, 0x0AC1, 0x0AC9, 0x0ACD, 0x0ACE, 0x0ACE, 0x0ACE, 0x0ACE, 0x0ACE, 0x0ACE, 0x0ACE, 0x0ACE, 0x0ACE, 0x0ACE, 0x0ACE, 0x0ACE, 0x0ACE, 0x0ACE, 0x0ACE, 0x0ACE, 0x0ACE, 0x0ACE, 0x0ACE, 0x0ACE, 0x0ACE, 0x0ACE, 0x0ACE, 0x0ACE, 0x0ACD, 0x0AC9, 0x0AC1, 0x0000,
    0x0AC1, 0x0ACE, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACE, 0x0AC1,
    0x0AC9, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x09C9,
    0x0ACD, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACD,
    0x0ACE, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x6CDF, 0x7CEF, 0x7CEF, 0x7CEF, 0x5CDF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x5CDF, 0x7CEF, 0x7CEF, 0x7CEF, 0x6CDF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACD,
    0x0ACE, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0xEFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0x5BDF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x4BDF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xEFFF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x09CD,
    0x09CE, 0x09CF, 0x09CF, 0x09CF, 0x0ACF, 0x0ACF, 0x09CF, 0x6CDF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xDFFF, 0x0ACF, 0x0ACF, 0x0ACF, 0x0ACF, 0x09CF, 0x09CF, 0xCEFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0x7CEF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CD,
    0x09CE, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x0ACF, 0xDEFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0x6CDF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x5BDF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xEFFF, 0x0ACF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CD,
    0x09CE, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x5BDF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xDFFF, 0x09CF, 0x09CF, 0x09CF, 0x0ACF, 0xDFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0x6CDF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CD,
    0x09CE, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0xCEFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xBEEF, 0x09CF, 0x09CF, 0x09CF, 0x6CDF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xDEFF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CD,
    0x0ACE, 0x09CF, 0x09CF, 0x0ACF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x3BDF, 0xFFFF, 0xFFFF, 0xFFFF, 0x3ACF, 0x09CF, 0x09CF, 0x0ACF, 0xEFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0x4BDF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CD,
    0x09CE, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0xBEEF, 0xFFFF, 0xADEF, 0x09CF, 0x09CF, 0x09CF, 0x7CDF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xCEFF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CD,
    0x09CE, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x2ACF, 0xFFFF, 0x2ACF, 0x09CF, 0x09CF, 0x1ACF, 0xEFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0x3BCF, 0x09CF, 0x09BF, 0x09BF, 0x09BF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CF, 0x09CD,
    0x09CE, 0x09BF, 0x09BF, 0x09CF, 0x09CF, 0x09CF, 0x09BF, 0x09CF, 0x09BF, 0x09BF, 0x09CF, 0x09CF, 0x09BF, 0x09BF, 0x09BF, 0x8CEF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xBDEF, 0x09CF, 0x09BF, 0x09BF, 0x09CF, 0x09CF, 0x09BF, 0x09CF, 0x09BF, 0x09BF, 0x09BF, 0x09CD,
    0x09CE, 0x09BF, 0x09BF, 0x09BF, 0x09CF, 0x09CF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x1ACF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0x2ACF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09CF, 0x09CF, 0x09BD,
    0x09BE, 0x09BF, 0x09CF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x9DEF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xADEF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BD,
    0x09BE, 0x09CF, 0x09CF, 0x09CF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x8CDF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xBEEF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BD,
    0x09BE, 0x09BF, 0x09BF, 0x09BF, 0x09CF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x19BF, 0xEFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0x4BCF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BD,
    0x09BE, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x3ACF, 0x09BF, 0x09BF, 0x09BF, 0x6CDF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xCEFF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BD,
    0x09BE, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x4BCF, 0xFFFF, 0x3ACF, 0x09BF, 0x09BF, 0x09BF, 0xDEFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0x6BCF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BD,
    0x09BE, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x08BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0xDEFF, 0xFFFF, 0xCEEF, 0x09BF, 0x09BF, 0x09BF, 0x4BCF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xEFFF, 0x09BF, 0x08BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BD,
    0x09BE, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x08BF, 0x09BF, 0x09BF, 0x6BDF, 0xFFFF, 0xFFFF, 0xFFFF, 0x5BCF, 0x09BF, 0x09BF, 0x09BF, 0xBEEF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0x7CDF, 0x09BF, 0x09BF, 0x08BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x09BD,
    0x09BE, 0x09BF, 0x09BF, 0x09BF, 0x09BF, 0x08BF, 0x08BF, 0x09BF, 0x19BF, 0xEFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xDEFF, 0x09BF, 0x09BF, 0x09BF, 0x3ACF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0x19BF, 0x08BF, 0x08BF, 0x08BF, 0x09BF, 0x08BF, 0x09BF, 0x09BF, 0x09BD,
    0x08BE, 0x08BF, 0x08BF, 0x08BF, 0x08BF, 0x08BF, 0x09BF, 0x09BF, 0x8CDF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xADEF, 0x08BF, 0x08BF, 0x09BF, 0x08BF, 0x9DEF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0x9DEF, 0x08AF, 0x08AF, 0x08BF, 0x08BF, 0x08BF, 0x08BF, 0x08AF, 0x08AD,
    0x08AE, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x29BF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0x29BF, 0x08AF, 0x09BF, 0x08AF, 0x08AF, 0x19BF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0x3ABF, 0x08BF, 0x08AF, 0x08AF, 0x08BF, 0x08AF, 0x08AF, 0x09BD,
    0x09BE, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0xADEF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0x9CDF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x8CDF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xBEEF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AD,
    0x09BE, 0x09BF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x29BF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xEFFF, 0x19BF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08BF, 0x08AF, 0xEFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0x3ABF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AD,
    0x09BE, 0x09BF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x29BF, 0x39BF, 0x39BF, 0x39BF, 0x19BF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08BF, 0x18AF, 0x3ABF, 0x3ABF, 0x39BF, 0x2ABF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AD,
    0x09BD, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AD,
    0x08A9, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08A9,
    0x08A1, 0x08AE, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x089F, 0x089F, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x08AF, 0x079F, 0x079E, 0x0791,
    0x0000, 0x07A1, 0x08A9, 0x08AD, 0x09BD, 0x08AD, 0x08AD, 0x079D, 0x079D, 0x079D, 0x079D, 0x08AD, 0x08AD, 0x08AD, 0x08AD, 0x089D, 0x079D, 0x079D, 0x08AD, 0x08AD, 0x08AD, 0x08AD, 0x08AD, 0x079D, 0x079D, 0x079D, 0x08AD, 0x08AD, 0x08AD, 0x0799, 0x0791, 0x0000
};
} // namespace touchgfx