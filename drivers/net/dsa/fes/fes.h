#ifndef __FES_H
#define __FES_H

#include <linux/ptp_clock_kernel.h>

/* base addresses */
#define FES_OFFSET		0x00000
#define FES_PORT_OFFSET(i)	(0x10000 * ((i) + 1))
#define FRTC_OFFSET		0x80000
#define FES_PHY_OFFSET		0x200000

struct fes_priv {
	struct device *dev;
	void __iomem *regs;
	struct fes_chip_data *cd;
	struct delayed_work ptp_tx_work;
	struct sk_buff *ptp_tx_skb;

	struct ptp_clock_info ptp_clock_info;
	struct ptp_clock *ptp_clock;
	spinlock_t ptp_lock;
	int phc_index;
	u64 *counters;
};

void fes_ptp_register(struct fes_priv *priv);
void fes_ptp_unregister(struct fes_priv *priv);

static inline u16 fes_read(struct fes_priv *priv, u32 offset)
{
	void __iomem *addr = priv->regs + offset;

	return ioread16(addr);
}

static inline void fes_write(struct fes_priv *priv, u32 offset, u16 value)
{
	void __iomem *addr = priv->regs + offset;

	iowrite16(value, addr);
}

static inline int fes_cmd(struct fes_priv *priv, u32 offset, u16 cmd)
{
	int timeout = 10;

	fes_write(priv, offset, cmd);
	do {
		if (fes_read(priv, offset) & cmd)
			break;
		cpu_relax();
	} while (timeout-- > 0);

	return (timeout) ? 0 : -EIO;
}

static inline u16 fes_port_read(struct fes_priv *priv, int port, u32 offset)
{
	return fes_read(priv, FES_PORT_OFFSET(port) + offset);
}

static inline void fes_port_write(struct fes_priv *priv, int port,
				  u32 offset, u16 value)
{
	fes_write(priv, FES_PORT_OFFSET(port) + offset, value);
}

static inline int fes_port_cmd(struct fes_priv *priv, int port,
			       u32 offset, u16 cmd)
{
	return fes_cmd(priv, FES_PORT_OFFSET(port) + offset, cmd);
}

#endif /* __FES_H */
