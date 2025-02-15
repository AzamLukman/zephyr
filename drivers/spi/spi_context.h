/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Private API for SPI drivers
 */

#ifndef ZEPHYR_DRIVERS_SPI_SPI_CONTEXT_H_
#define ZEPHYR_DRIVERS_SPI_SPI_CONTEXT_H_

#include <drivers/gpio.h>
#include <drivers/spi.h>

#ifdef __cplusplus
extern "C" {
#endif

enum spi_ctx_runtime_op_mode {
	SPI_CTX_RUNTIME_OP_MODE_MASTER = BIT(0),
	SPI_CTX_RUNTIME_OP_MODE_SLAVE  = BIT(1),
};

struct spi_context {
	const struct spi_config *config;
	const struct spi_config *owner;

	struct k_sem lock;
	struct k_sem sync;
	int sync_status;

#ifdef CONFIG_SPI_ASYNC
	struct k_poll_signal *signal;
	bool asynchronous;
#endif /* CONFIG_SPI_ASYNC */
	const struct spi_buf *current_tx;
	size_t tx_count;
	const struct spi_buf *current_rx;
	size_t rx_count;

	const uint8_t *tx_buf;
	size_t tx_len;
	uint8_t *rx_buf;
	size_t rx_len;

#ifdef CONFIG_SPI_SLAVE
	int recv_frames;
#endif /* CONFIG_SPI_SLAVE */
};

#define SPI_CONTEXT_INIT_LOCK(_data, _ctx_name)				\
	._ctx_name.lock = Z_SEM_INITIALIZER(_data._ctx_name.lock, 0, 1)

#define SPI_CONTEXT_INIT_SYNC(_data, _ctx_name)				\
	._ctx_name.sync = Z_SEM_INITIALIZER(_data._ctx_name.sync, 0, 1)

static inline bool spi_context_configured(struct spi_context *ctx,
					  const struct spi_config *config)
{
	return !!(ctx->config == config);
}

static inline bool spi_context_is_slave(struct spi_context *ctx)
{
	return (ctx->config->operation & SPI_OP_MODE_SLAVE);
}

static inline void spi_context_lock(struct spi_context *ctx,
				    bool asynchronous,
				    struct k_poll_signal *signal,
				    const struct spi_config *spi_cfg)
{
	if ((spi_cfg->operation & SPI_LOCK_ON) &&
		(k_sem_count_get(&ctx->lock) == 0) &&
		(ctx->owner == spi_cfg)) {
			return;
	}

	k_sem_take(&ctx->lock, K_FOREVER);
	ctx->owner = spi_cfg;

#ifdef CONFIG_SPI_ASYNC
	ctx->asynchronous = asynchronous;
	ctx->signal = signal;
#endif /* CONFIG_SPI_ASYNC */
}

static inline void spi_context_release(struct spi_context *ctx, int status)
{
#ifdef CONFIG_SPI_SLAVE
	if (status >= 0 && (ctx->config->operation & SPI_LOCK_ON)) {
		return;
	}
#endif /* CONFIG_SPI_SLAVE */

#ifdef CONFIG_SPI_ASYNC
	if (!ctx->asynchronous || (status < 0)) {
		ctx->owner = NULL;
		k_sem_give(&ctx->lock);
	}
#else
	if (!(ctx->config->operation & SPI_LOCK_ON)) {
		ctx->owner = NULL;
		k_sem_give(&ctx->lock);
	}
#endif /* CONFIG_SPI_ASYNC */
}

static inline int spi_context_wait_for_completion(struct spi_context *ctx)
{
	int status = 0;
	k_timeout_t timeout;

	/* Do not use any timeout in the slave mode, as in this case it is not
	 * known when the transfer will actually start and what the frequency
	 * will be.
	 */
	if (IS_ENABLED(CONFIG_SPI_SLAVE) && spi_context_is_slave(ctx)) {
		timeout = K_FOREVER;
	} else {
		uint32_t timeout_ms;

		timeout_ms = MAX(ctx->tx_len, ctx->rx_len) * 8 * 1000 /
			     ctx->config->frequency;
		timeout_ms += CONFIG_SPI_COMPLETION_TIMEOUT_TOLERANCE;

		timeout = K_MSEC(timeout_ms);
	}

#ifdef CONFIG_SPI_ASYNC
	if (!ctx->asynchronous) {
		if (k_sem_take(&ctx->sync, timeout)) {
			LOG_ERR("Timeout waiting for transfer complete");
			return -ETIMEDOUT;
		}
		status = ctx->sync_status;
	}
#else
	if (k_sem_take(&ctx->sync, timeout)) {
		LOG_ERR("Timeout waiting for transfer complete");
		return -ETIMEDOUT;
	}
	status = ctx->sync_status;
#endif /* CONFIG_SPI_ASYNC */

#ifdef CONFIG_SPI_SLAVE
	if (spi_context_is_slave(ctx) && !status) {
		return ctx->recv_frames;
	}
#endif /* CONFIG_SPI_SLAVE */

	return status;
}

static inline void spi_context_complete(struct spi_context *ctx, int status)
{
#ifdef CONFIG_SPI_ASYNC
	if (!ctx->asynchronous) {
		ctx->sync_status = status;
		k_sem_give(&ctx->sync);
	} else {
		if (ctx->signal) {
#ifdef CONFIG_SPI_SLAVE
			if (spi_context_is_slave(ctx) && !status) {
				/* Let's update the status so it tells
				 * about number of received frames.
				 */
				status = ctx->recv_frames;
			}
#endif /* CONFIG_SPI_SLAVE */
			k_poll_signal_raise(ctx->signal, status);
		}

		if (!(ctx->config->operation & SPI_LOCK_ON)) {
			ctx->owner = NULL;
			k_sem_give(&ctx->lock);
		}
	}
#else
	ctx->sync_status = status;
	k_sem_give(&ctx->sync);
#endif /* CONFIG_SPI_ASYNC */
}

static inline
gpio_dt_flags_t spi_context_cs_active_level(struct spi_context *ctx)
{
	if (ctx->config->operation & SPI_CS_ACTIVE_HIGH) {
		return GPIO_ACTIVE_HIGH;
	}

	return GPIO_ACTIVE_LOW;
}

static inline void spi_context_cs_configure(struct spi_context *ctx)
{
	if (ctx->config->cs && ctx->config->cs->gpio_dev) {
		/* Validate CS active levels are equivalent */
		__ASSERT(spi_context_cs_active_level(ctx) ==
			 (ctx->config->cs->gpio_dt_flags & GPIO_ACTIVE_LOW),
			 "Devicetree and spi_context CS levels are not equal");
		gpio_pin_configure(ctx->config->cs->gpio_dev,
				   ctx->config->cs->gpio_pin,
				   ctx->config->cs->gpio_dt_flags |
				   GPIO_OUTPUT_INACTIVE);
	} else {
		LOG_INF("CS control inhibited (no GPIO device)");
	}
}

static inline void _spi_context_cs_control(struct spi_context *ctx,
					   bool on, bool force_off)
{
	if (ctx->config && ctx->config->cs && ctx->config->cs->gpio_dev) {
		if (on) {
			gpio_pin_set(ctx->config->cs->gpio_dev,
				     ctx->config->cs->gpio_pin, 1);
			k_busy_wait(ctx->config->cs->delay);
		} else {
			if (!force_off &&
			    ctx->config->operation & SPI_HOLD_ON_CS) {
				return;
			}

			k_busy_wait(ctx->config->cs->delay);
			gpio_pin_set(ctx->config->cs->gpio_dev,
				     ctx->config->cs->gpio_pin, 0);
		}
	}
}

static inline void spi_context_cs_control(struct spi_context *ctx, bool on)
{
	_spi_context_cs_control(ctx, on, false);
}

static inline void spi_context_unlock_unconditionally(struct spi_context *ctx)
{
	/* Forcing CS to go to inactive status */
	_spi_context_cs_control(ctx, false, true);

	if (!k_sem_count_get(&ctx->lock)) {
		ctx->owner = NULL;
		k_sem_give(&ctx->lock);
	}
}

static inline void *spi_context_get_next_buf(const struct spi_buf **current,
					     size_t *count,
					     size_t *buf_len,
					     uint8_t dfs)
{
	/* This loop skips zero-length buffers in the set, if any. */
	while (*count) {
		if (((*current)->len / dfs) != 0) {
			*buf_len = (*current)->len / dfs;
			return (*current)->buf;
		}
		++(*current);
		--(*count);
	}

	*buf_len = 0;
	return NULL;
}

static inline
void spi_context_buffers_setup(struct spi_context *ctx,
			       const struct spi_buf_set *tx_bufs,
			       const struct spi_buf_set *rx_bufs,
			       uint8_t dfs)
{
	LOG_DBG("tx_bufs %p - rx_bufs %p - %u", tx_bufs, rx_bufs, dfs);

	ctx->current_tx = tx_bufs ? tx_bufs->buffers : NULL;
	ctx->tx_count = ctx->current_tx ? tx_bufs->count : 0;
	ctx->tx_buf = (const uint8_t *)
		spi_context_get_next_buf(&ctx->current_tx, &ctx->tx_count,
					 &ctx->tx_len, dfs);

	ctx->current_rx = rx_bufs ? rx_bufs->buffers : NULL;
	ctx->rx_count = ctx->current_rx ? rx_bufs->count : 0;
	ctx->rx_buf = (uint8_t *)
		spi_context_get_next_buf(&ctx->current_rx, &ctx->rx_count,
					 &ctx->rx_len, dfs);

	ctx->sync_status = 0;

#ifdef CONFIG_SPI_SLAVE
	ctx->recv_frames = 0;
#endif /* CONFIG_SPI_SLAVE */

	LOG_DBG("current_tx %p (%zu), current_rx %p (%zu),"
		" tx buf/len %p/%zu, rx buf/len %p/%zu",
		ctx->current_tx, ctx->tx_count,
		ctx->current_rx, ctx->rx_count,
		ctx->tx_buf, ctx->tx_len, ctx->rx_buf, ctx->rx_len);
}

static ALWAYS_INLINE
void spi_context_update_tx(struct spi_context *ctx, uint8_t dfs, uint32_t len)
{
	if (!ctx->tx_len) {
		return;
	}

	if (len > ctx->tx_len) {
		LOG_ERR("Update exceeds current buffer");
		return;
	}

	ctx->tx_len -= len;
	if (!ctx->tx_len) {
		/* Current buffer is done. Get the next one to be processed. */
		++ctx->current_tx;
		--ctx->tx_count;
		ctx->tx_buf = (const uint8_t *)
			spi_context_get_next_buf(&ctx->current_tx,
						 &ctx->tx_count,
						 &ctx->tx_len, dfs);
	} else if (ctx->tx_buf) {
		ctx->tx_buf += dfs * len;
	}

	LOG_DBG("tx buf/len %p/%zu", ctx->tx_buf, ctx->tx_len);
}

static ALWAYS_INLINE
bool spi_context_tx_on(struct spi_context *ctx)
{
	return !!(ctx->tx_len);
}

static ALWAYS_INLINE
bool spi_context_tx_buf_on(struct spi_context *ctx)
{
	return !!(ctx->tx_buf && ctx->tx_len);
}

static ALWAYS_INLINE
void spi_context_update_rx(struct spi_context *ctx, uint8_t dfs, uint32_t len)
{
#ifdef CONFIG_SPI_SLAVE
	if (spi_context_is_slave(ctx)) {
		ctx->recv_frames += len;
	}

#endif /* CONFIG_SPI_SLAVE */

	if (!ctx->rx_len) {
		return;
	}

	if (len > ctx->rx_len) {
		LOG_ERR("Update exceeds current buffer");
		return;
	}

	ctx->rx_len -= len;
	if (!ctx->rx_len) {
		/* Current buffer is done. Get the next one to be processed. */
		++ctx->current_rx;
		--ctx->rx_count;
		ctx->rx_buf = (uint8_t *)
			spi_context_get_next_buf(&ctx->current_rx,
						 &ctx->rx_count,
						 &ctx->rx_len, dfs);
	} else if (ctx->rx_buf) {
		ctx->rx_buf += dfs * len;
	}

	LOG_DBG("rx buf/len %p/%zu", ctx->rx_buf, ctx->rx_len);
}

static ALWAYS_INLINE
bool spi_context_rx_on(struct spi_context *ctx)
{
	return !!(ctx->rx_len);
}

static ALWAYS_INLINE
bool spi_context_rx_buf_on(struct spi_context *ctx)
{
	return !!(ctx->rx_buf && ctx->rx_len);
}

/*
 * Returns the maximum length of a transfer for which all currently active
 * directions have a continuous buffer, i.e. the maximum SPI transfer that
 * can be done with DMA that handles only non-scattered buffers.
 */
static inline size_t spi_context_max_continuous_chunk(struct spi_context *ctx)
{
	if (!ctx->tx_len) {
		return ctx->rx_len;
	} else if (!ctx->rx_len) {
		return ctx->tx_len;
	}

	return MIN(ctx->tx_len, ctx->rx_len);
}

static inline size_t spi_context_longest_current_buf(struct spi_context *ctx)
{
	return ctx->tx_len > ctx->rx_len ? ctx->tx_len : ctx->rx_len;
}

static inline size_t spi_context_total_tx_len(struct spi_context *ctx)
{
	size_t n;
	size_t total_len = 0;

	for (n = 0; n < ctx->tx_count; ++n) {
		total_len += ctx->current_tx[n].len;
	}

	return total_len;
}

static inline size_t spi_context_total_rx_len(struct spi_context *ctx)
{
	size_t n;
	size_t total_len = 0;

	for (n = 0; n < ctx->rx_count; ++n) {
		total_len += ctx->current_rx[n].len;
	}

	return total_len;
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_SPI_SPI_CONTEXT_H_ */
