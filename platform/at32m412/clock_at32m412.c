#include "clock_at32m412.h"
#include "at32m412_416_wk_config.h"

void clock_at32m412_init(void)
{
    wk_system_clock_config();
}
