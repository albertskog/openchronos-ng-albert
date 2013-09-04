#include <openchronos.h>

#include "libs/buzzer.h"
#include "drivers/buzzer.h"


note shortBip[2] = {0x4b08,0x000F};

void buzzer_shortBip(void)
{
    buzzer_play(shortBip);
}