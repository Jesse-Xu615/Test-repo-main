/*
 * Copyright 2021-2023 Senscomm Semiconductor Co., Ltd.	All rights reserved.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef _HASH_H_
#define _HASH_H_

#include <stdint.h>

struct hash_ctx {
    uint32_t size;
    uint32_t remain_size;
    uint8_t remain[64];
};

void hash_init(struct hash_ctx *ctx, uint8_t algo);
void hash_update(struct hash_ctx *ctx, uint8_t *m, uint32_t size);
void hash_finish(struct hash_ctx *ctx, uint8_t *digest);

#endif // _HASH_H_
