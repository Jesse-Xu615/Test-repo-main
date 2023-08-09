/*
 * Copyright 2018-2019 Senscomm Semiconductor Co., Ltd.	All rights reserved.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS 
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#include "cmsis_os.h"
#include "callout.h"

#include <stdio.h>
#include <string.h>

#include <soc.h>
#include <hal/kernel.h>
#include <hal/init.h>
#include <hal/console.h>

#include <cli.h>
#include <wlan.h>
#include <lwip/tcpip.h>

static struct callout co;
static int s_index = 1;
static int s_period = 0;
static void callout_func(void *arg)
{
	char *desc = (char *)arg;
	printf("** %s (%d)\n", desc, s_index++);
	callout_schedule(&co, s_period);
}

int main(void)
{
	callout_init(&co);
	s_period = PERIOD_SEC(1);
	callout_reset(&co, s_period, callout_func, (void *)"This should be 1 sec period");
	osDelay(PERIOD_SEC(11));
	callout_drain(&co);
	printf("============================\n");
	printf("You must have seen 10 times.\n");
	printf("============================\n");
	s_index = 1;
	s_period = PERIOD_SEC(1);
	callout_schedule(&co, s_period);
	osDelay(PERIOD_SEC(6));
	callout_drain(&co);
	printf("============================\n");
	printf("You must have seen 5 times. \n");
	printf("============================\n");
	s_index = 1;
	s_period = PERIOD_SEC(2);
	callout_reset(&co, s_period, callout_func, (void *)"This should be 2 sec period");
	osDelay(PERIOD_SEC(11));
	callout_drain(&co);
	printf("============================\n");
	printf("You must have seen 5 times. \n");
	printf("============================\n");
	s_index = 1;
	callout_stop(&co);
	s_period = PERIOD_MSEC(200);
	callout_reset(&co, s_period, callout_func, (void *)"This should be 200 msec period");
	osDelay(PERIOD_MSEC(2200));
	callout_drain(&co);
	printf("============================\n");
	printf("You must have seen 10 times. \n");
	printf("============================\n");

	return 0;
}
