#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/spinlock.h>
#include <net/dsa.h>

#include "fes.h"
#include "fes-regs.h"

static u64 frtc_read(struct fes_priv *priv, u32 offset)
{
	void __iomem *addr = priv->regs + FRTC_OFFSET + offset;

	return readq(addr);
}

static void frtc_write(struct fes_priv *priv, u32 offset, u64 value)
{
	void __iomem *addr = priv->regs + FRTC_OFFSET + offset;

	return writeq(value, addr);
}

static int frtc_cmd(struct fes_priv *priv, u32 offset, u64 cmd)
{
	int timeout = 10;

	frtc_write(priv, offset, cmd);
	do {
		if (frtc_read(priv, offset) & cmd)
			break;
		cpu_relax();
	} while (timeout-- > 0);

	return (timeout) ? 0 : -EIO;
}

static int __frtc_gettime(struct ptp_clock_info *ptp, struct timespec64 *ts)
{
	struct fes_priv *priv = container_of(ptp, struct fes_priv, ptp_clock_info);
	int ret;

	ret = frtc_cmd(priv, FRTC_TIME_CMD, TIME_CMD_READ_TIME);
	if (ret)
		return ret;

	ts->tv_sec = frtc_read(priv, FRTC_CUR_SEC);
	ts->tv_nsec = frtc_read(priv, FRTC_CUR_NSEC) >> 32;

	return 0;
}

static int frtc_gettime(struct ptp_clock_info *ptp, struct timespec64 *ts)
{
	struct fes_priv *priv = container_of(ptp, struct fes_priv, ptp_clock_info);
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&priv->ptp_lock, flags);
	ret = __frtc_gettime(ptp, ts);
	spin_unlock_irqrestore(&priv->ptp_lock, flags);

	return ret;
}

static int frtc_enable(struct ptp_clock_info *ptp,
			  struct ptp_clock_request *rq, int on)
{
	return -EOPNOTSUPP;
}

static int __frtc_adjtime(struct ptp_clock_info *ptp, struct timespec *ts)
{
	struct fes_priv *priv = container_of(ptp, struct fes_priv, ptp_clock_info);

	frtc_write(priv, FRTC_ADJUST_NSEC, ts->tv_nsec << 32);
	frtc_write(priv, FRTC_ADJUST_SEC, ts->tv_sec);
	return frtc_cmd(priv, FRTC_TIME_CMD, TIME_CMD_ADJUST_TIME);
}

static int frtc_adjtime(struct ptp_clock_info *ptp, s64 delta)
{
	struct fes_priv *priv = container_of(ptp, struct fes_priv, ptp_clock_info);
	struct timespec ts = ns_to_timespec(delta);
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&priv->ptp_lock, flags);
	ret = __frtc_adjtime(ptp, &ts);
	spin_unlock_irqrestore(&priv->ptp_lock, flags);

	return ret;
}

static int frtc_settime(struct ptp_clock_info *ptp, const struct timespec64 *ts)
{
	struct fes_priv *priv = container_of(ptp, struct fes_priv, ptp_clock_info);
	struct timespec orig_ts, delta;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&priv->ptp_lock, flags);

	ret = __frtc_gettime(ptp, &orig_ts);
	if (ret)
		goto out;

	delta = timespec64_sub(*ts, orig_ts);
	ret = __frtc_adjtime(ptp, &delta);

out:
	spin_unlock_irqrestore(&priv->ptp_lock, flags);
	return ret;
}

static int frtc_adjust_step(struct fes_priv *priv, u64 subnsec)
{
	frtc_write(priv, FRTC_STEP_SIZE, subnsec);
	return frtc_cmd(priv, FRTC_TIME_CMD, TIME_CMD_ADJUST_STEP);
}

#define NS_STEP_SIZE 8ULL
#define INITIAL_SUBNS (NS_STEP_SIZE << 32)
#define ADJFREQ_SCALING_FACTOR ((1ULL<<32) * NS_STEP_SIZE / 1000000000)
static int frtc_adjfreq(struct ptp_clock_info *ptp, s32 delta)
{
	struct fes_priv *priv = container_of(ptp, struct fes_priv, ptp_clock_info);
	u64 subnsec = INITIAL_SUBNS + delta * ADJFREQ_SCALING_FACTOR;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&priv->ptp_lock, flags);
	ret = frtc_adjust_step(priv, subnsec);
	spin_unlock_irqrestore(&priv->ptp_lock, flags);

	return ret;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,10,0)
static int frtc_adjfine(struct ptp_clock_info *ptp, long scaled_ppm)
{
	struct fes_priv *priv = container_of(ptp, struct fes_priv, ptp_clock_info);
	unsigned long flags;
	u64 subnsec;
	s64 delta;
	int ret;

	/*
	 *   delta = ns_step_size * 2^32 / 1000000 * scaled_ppm / 2^16
	 * simplifies to:
	 *   delta = ns_step_size * scaled_ppm / 15625 * 2^10
	 */
	delta = NS_STEP_SIZE * scaled_ppm;
	delta <<= 10;
	delta = div_s64(delta, 15625);
	subnsec = INITIAL_SUBNS + delta;

	spin_lock_irqsave(&priv->ptp_lock, flags);
	ret = frtc_adjust_step(priv, subnsec);
	spin_unlock_irqrestore(&priv->ptp_lock, flags);

	return ret;
}
#endif

static struct ptp_clock_info frtc_ptp_clock_info = {
	.owner = THIS_MODULE,
	.name = "frtc",
	.max_adj = 999999999,
	.n_alarm = 0,
	.n_ext_ts = 0,
	.n_per_out = 0,
	.n_pins = 0,
	.pps = 0,
	.gettime64 = frtc_gettime,
	.settime64 = frtc_settime,
	.adjtime = frtc_adjtime,
	.adjfreq = frtc_adjfreq,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,10,0)
	.adjfine = frtc_adjfine,
#endif
	.enable = frtc_enable,
};

void fes_ptp_register(struct fes_priv *priv)
{
	spin_lock_init(&priv->ptp_lock);

	priv->ptp_clock_info = frtc_ptp_clock_info;
	priv->ptp_clock = ptp_clock_register(&priv->ptp_clock_info, priv->dev);
	if (IS_ERR(priv->ptp_clock)) {
		priv->ptp_clock = NULL;
		dev_err(priv->dev, "ptp_clock_register failed\n");
	} else if (priv->ptp_clock) {
		priv->phc_index = ptp_clock_index(priv->ptp_clock);
		dev_info(priv->dev, "added PHC\n");
	}
}

void fes_ptp_unregister(struct fes_priv *priv)
{
	if (priv->ptp_clock) {
		ptp_clock_unregister(priv->ptp_clock);
		priv->ptp_clock = NULL;
	}
}
