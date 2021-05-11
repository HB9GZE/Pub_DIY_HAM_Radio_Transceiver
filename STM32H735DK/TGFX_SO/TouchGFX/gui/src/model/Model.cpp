#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

struct FreqandBand
{
	uint32_t frequencyValue;
	uint8_t bandValue;
};
FreqandBand queueUpdateValue;

extern "C"
{
xQueueHandle messageQ_VFO;
xQueueHandle messageQ_BFO;
xQueueHandle messageQ_Transmit;
}

Model::Model() :
		modelListener(0)
{
	messageQ_VFO = xQueueGenericCreate(1, 1, 0);
	messageQ_BFO = xQueueGenericCreate(1, 1, 0);
	messageQ_Transmit = xQueueGenericCreate(1, 1, 0);
}

void Model::tick()
{
}
