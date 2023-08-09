/*
 * Copyright 2023-2024 Senscomm Semiconductor Co., Ltd.	All rights reserved.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <stdio.h>
#include <stdbool.h>

#include "hal/io.h"

int porbor(void)
{
#ifdef CONFIG_ATE_PORBOR

#ifdef CONFIG_SYS_FPGA
#error "It doesn't work on the FPGA!!!"
#endif

	u32 v;

	/* Clear reset cause */
	v = readl(SMU(TOP_CFG));
	writel(v | (0x3 << 24), SMU(TOP_CFG));

	/* Enable BOR reset. */
	v = readl(SMU(TOP_CFG));
	writel(v | (0x1 << 0), SMU(TOP_CFG));
#endif

	return 0;
}
