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
#include "taskqueue.h"
#include "condvar.h"

#include <stdio.h>
#include <string.h>

#include <soc.h>
#include <hal/kernel.h>
#include <hal/init.h>
#include <hal/console.h>

#include <cli.h>
#include <wlan.h>
#include <lwip/tcpip.h>

static struct taskqueue *s_tq;
static struct task s_task;
static struct cv s_cv;
static struct mtx s_mtx;

static void tq_task(void *arg, int pending)
{
	char *desc = (char *)arg;

	cv_wait(&s_cv, &s_mtx);
	printf("** %s (pending:%d)\n", desc, pending);
}

static void test_task(void *param)
{
	/* Create and initialize a taskqueue */
	s_tq = taskqueue_create("tq", 0,
			taskqueue_thread_enqueue, &s_tq);
	taskqueue_start_threads(&s_tq, 1, PI_NET, "%s taskq",
	    "test");
	/* Create and initialize a conditional variable */
	cv_init(&s_cv, "test");
	/* Create and initialize a task */
	TASK_INIT(&s_task, 0, tq_task, (void *)"Task spawned!");
	mtx_init(&s_mtx, "test", NULL, MTX_DEF);

	for (;;) {
		osDelay(PERIOD_SEC(1));
		//taskqueue_block(s_tq);
		taskqueue_enqueue(s_tq, &s_task);
	}

	osThreadExit();
}

static void signal_task(void *param)
{
	for (;;) {
		//osDelay(PERIOD_SEC(4));
		//taskqueue_unblock(s_tq);
		osDelay(PERIOD_MSEC(500));
		cv_signal(&s_cv);
	}
	osThreadExit();
}

int main(void)
{
	osThreadId_t tid;
	osThreadAttr_t attr = {"taskq-test", 0, NULL, 0, NULL, 2048, osPriorityNormal, 0, 0};

	tid = osThreadNew(test_task, NULL, &attr);
	if (tid)
		printk("[%s, %d] task creation succeeded!(0x%x)\n",
				__func__, __LINE__, tid);
	else
		printk("[%s, %d] task creation failed!\n",
				__func__, __LINE__);

	attr.name = "taskq-test2";
	tid = osThreadNew(signal_task, NULL, &attr);
	if (tid)
		printk("[%s, %d] task creation succeeded!(0x%x)\n",
				__func__, __LINE__, tid);
	else
		printk("[%s, %d] task creation failed!\n",
				__func__, __LINE__);

	return 0;
}
