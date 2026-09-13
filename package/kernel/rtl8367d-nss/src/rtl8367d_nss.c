// SPDX-License-Identifier: GPL-2.0-only
/*
 * RTL8367D fabric re-arm after rtl8365mb teardown, for the IPQ5018 NSS plane.
 *
 * The NSS firmware parses 802.1Q natively but cannot parse a DSA tag placed
 * where the ethertype belongs, and DSA user ports never get an NSS interface
 * number -- so on IPQ5018 DSA user ports and ECM acceleration are mutually
 * exclusive.  The answer used on the QCA8337 boards is to let the DSA driver
 * do the full bring-up, unbind it, and program the fabric directly; this is
 * the same thing for the Realtek side.
 *
 * What survives rtl8365mb's teardown, verified on a TP-Link EX511 v2: the
 * external-interface SerDes configuration does.  eth0 stays up at 1 Gbps with
 * the driver unbound, so nothing here re-does the SerDes or the PCS.  What
 * does NOT survive is everything dsa_unregister_switch() undoes on the way
 * out: every port is left in BR_STATE_DISABLED by port_stp_state_set(), and
 * phy_detach() suspends the front PHYs.  That is what this module repairs.
 *
 * Like qca8337-nss it is not a probe-based driver: nothing binds
 * automatically, it reaches the switch through a *named* MDIO device.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/mdio.h>
#include <linux/module.h>
#include <linux/phy.h>

/* realtek-mdio register access protocol */
#define RTL_MDIO_CTRL0_REG	31
#define RTL_MDIO_CTRL1_REG	21
#define RTL_MDIO_ADDRESS_REG	23
#define RTL_MDIO_DATA_WRITE_REG	24
#define RTL_MDIO_DATA_READ_REG	25
#define RTL_MDIO_ADDR_OP	0x000E
#define RTL_MDIO_READ_OP	0x0001
#define RTL_MDIO_WRITE_OP	0x0003

/* CPU tag */
#define RTL_CPU_PORT_MASK_REG	0x1219
#define RTL_CPU_CTRL_REG	0x121A
#define RTL_CPU_CTRL_EN_MASK	0x0001

/* MSTP port state, instance 0 */
#define RTL_MSTI_CTRL_REG(p)	(0x0A00 + ((p) >> 3))
#define RTL_MSTI_STATE_OFF(p)	((p) << 1)
#define RTL_MSTI_STATE_MASK(p)	(0x3 << RTL_MSTI_STATE_OFF(p))
#define RTL_STP_FORWARDING	3

/* Port isolation (forwarding port mask) */
#define RTL_PORT_ISOLATION_REG(p)	(0x08A2 + (p))

/* VLAN */
#define RTL_VLAN_CTRL_REG	0x07A8
#define RTL_VLAN_CTRL_EN_MASK	0x0001
#define RTL_VLAN_INGRESS_REG	0x07A9

/* Unknown DA flooding port masks */
#define RTL_FLOOD_UC_REG	0x0890
#define RTL_FLOOD_MC_REG	0x0891
#define RTL_FLOOD_BC_REG	0x0892

/* Indirect access to the internal GPHYs */
#define RTL_IA_CTRL_REG		0x1F00
#define RTL_IA_STATUS_REG	0x1F01
#define RTL_IA_ADDRESS_REG	0x1F02
#define RTL_IA_WRITE_DATA_REG	0x1F03
#define RTL_GPHY_OCP_MSB_0_REG	0x1D15
#define RTL_GPHY_OCP_MSB_0_MASK	0x0FC0
#define RTL_PHY_OCP_PREFIX_MASK	0xFC00
#define RTL_PHY_OCP_PHYREG_BASE	0xA400
#define RTL_PHY_BASE		0x2000
#define RTL_IA_ADDR_OCP_5_1	GENMASK(4, 0)
#define RTL_IA_ADDR_PHYNUM	GENMASK(7, 5)
#define RTL_IA_ADDR_OCP_9_6	GENMASK(11, 8)

#define RTL_NPORTS		11

static char *switch_dev = "90000.mdio-1:1d";
module_param(switch_dev, charp, 0444);
MODULE_PARM_DESC(switch_dev, "MDIO device name of the switch itself");

static unsigned int cpu_port = 6;
module_param(cpu_port, uint, 0444);
MODULE_PARM_DESC(cpu_port, "Switch port wired to the SoC GMAC (default 6)");

static unsigned int ports = 0x5f;
module_param(ports, uint, 0444);
MODULE_PARM_DESC(ports, "Bitmask of switch ports to enable, CPU port included (default 0x5f = 0-4 + 6)");

static unsigned int wake_phys = 0x1f;
module_param(wake_phys, uint, 0444);
MODULE_PARM_DESC(wake_phys, "Bitmask of internal GPHYs to power back up (default 0x1f = PHY 0-4)");

static bool switch_fixup = true;
module_param(switch_fixup, bool, 0444);
MODULE_PARM_DESC(switch_fixup, "Re-arm the fabric, CPU tag off (default on)");

static struct mii_bus *sw_bus;
static int sw_addr;

static int rtl_read(u32 reg, u32 *val)
{
	int ret;

	mutex_lock(&sw_bus->mdio_lock);
	ret = sw_bus->write(sw_bus, sw_addr, RTL_MDIO_CTRL0_REG, RTL_MDIO_ADDR_OP);
	if (ret)
		goto out;
	ret = sw_bus->write(sw_bus, sw_addr, RTL_MDIO_ADDRESS_REG, reg);
	if (ret)
		goto out;
	ret = sw_bus->write(sw_bus, sw_addr, RTL_MDIO_CTRL1_REG, RTL_MDIO_READ_OP);
	if (ret)
		goto out;
	ret = sw_bus->read(sw_bus, sw_addr, RTL_MDIO_DATA_READ_REG);
	if (ret >= 0) {
		*val = ret;
		ret = 0;
	}
out:
	mutex_unlock(&sw_bus->mdio_lock);
	return ret;
}

static int rtl_write(u32 reg, u32 val)
{
	int ret;

	mutex_lock(&sw_bus->mdio_lock);
	ret = sw_bus->write(sw_bus, sw_addr, RTL_MDIO_CTRL0_REG, RTL_MDIO_ADDR_OP);
	if (ret)
		goto out;
	ret = sw_bus->write(sw_bus, sw_addr, RTL_MDIO_ADDRESS_REG, reg);
	if (ret)
		goto out;
	ret = sw_bus->write(sw_bus, sw_addr, RTL_MDIO_DATA_WRITE_REG, val);
	if (ret)
		goto out;
	ret = sw_bus->write(sw_bus, sw_addr, RTL_MDIO_CTRL1_REG, RTL_MDIO_WRITE_OP);
out:
	mutex_unlock(&sw_bus->mdio_lock);
	return ret;
}

static int rtl_update(u32 reg, u32 mask, u32 val)
{
	u32 cur;
	int ret;

	ret = rtl_read(reg, &cur);
	if (ret)
		return ret;

	return rtl_write(reg, (cur & ~mask) | (val & mask));
}

/* Indirect write to an internal GPHY register, as rtl8365mb_phy_write does. */
static int rtl_phy_write(int phy, int regnum, u16 data)
{
	u32 ocp_addr = RTL_PHY_OCP_PHYREG_BASE + regnum * 2;
	u32 val;
	int ret, i;

	for (i = 0; i < 10; i++) {
		ret = rtl_read(RTL_IA_STATUS_REG, &val);
		if (ret)
			return ret;
		if (!val)
			break;
		usleep_range(10, 100);
	}
	if (val)
		return -ETIMEDOUT;

	ret = rtl_update(RTL_GPHY_OCP_MSB_0_REG, RTL_GPHY_OCP_MSB_0_MASK,
			 FIELD_PREP(RTL_GPHY_OCP_MSB_0_MASK,
				    FIELD_GET(RTL_PHY_OCP_PREFIX_MASK, ocp_addr)));
	if (ret)
		return ret;

	val = RTL_PHY_BASE;
	val |= FIELD_PREP(RTL_IA_ADDR_PHYNUM, phy);
	val |= FIELD_PREP(RTL_IA_ADDR_OCP_5_1, (ocp_addr >> 1) & 0x1f);
	val |= FIELD_PREP(RTL_IA_ADDR_OCP_9_6, (ocp_addr >> 6) & 0xf);
	ret = rtl_write(RTL_IA_ADDRESS_REG, val);
	if (ret)
		return ret;

	ret = rtl_write(RTL_IA_WRITE_DATA_REG, data);
	if (ret)
		return ret;

	/* CMD | RW=write */
	return rtl_write(RTL_IA_CTRL_REG, 0x3);
}

static int rtl_phy_read(int phy, int regnum, u32 *data)
{
	u32 ocp_addr = RTL_PHY_OCP_PHYREG_BASE + regnum * 2;
	u32 val;
	int ret, i;

	for (i = 0; i < 10; i++) {
		ret = rtl_read(RTL_IA_STATUS_REG, &val);
		if (ret)
			return ret;
		if (!val)
			break;
		usleep_range(10, 100);
	}

	ret = rtl_update(RTL_GPHY_OCP_MSB_0_REG, RTL_GPHY_OCP_MSB_0_MASK,
			 FIELD_PREP(RTL_GPHY_OCP_MSB_0_MASK,
				    FIELD_GET(RTL_PHY_OCP_PREFIX_MASK, ocp_addr)));
	if (ret)
		return ret;

	val = RTL_PHY_BASE;
	val |= FIELD_PREP(RTL_IA_ADDR_PHYNUM, phy);
	val |= FIELD_PREP(RTL_IA_ADDR_OCP_5_1, (ocp_addr >> 1) & 0x1f);
	val |= FIELD_PREP(RTL_IA_ADDR_OCP_9_6, (ocp_addr >> 6) & 0xf);
	ret = rtl_write(RTL_IA_ADDRESS_REG, val);
	if (ret)
		return ret;

	/* CMD | RW=read */
	ret = rtl_write(RTL_IA_CTRL_REG, 0x1);
	if (ret)
		return ret;

	return rtl_read(0x1F04, data);
}

static void fixup_wake_phys(void)
{
	int phy;

	for (phy = 0; phy < 8; phy++) {
		u32 bmcr = 0;

		if (!(wake_phys & BIT(phy)))
			continue;

		if (rtl_phy_read(phy, MII_BMCR, &bmcr)) {
			pr_warn("rtl8367d-nss: PHY%d BMCR read failed\n", phy);
			continue;
		}
		pr_info("rtl8367d-nss: PHY%d BMCR=0x%04x%s\n", phy, bmcr,
			(bmcr & BMCR_PDOWN) ? " (POWER-DOWN)" : "");

		bmcr = (bmcr & ~BMCR_PDOWN) | BMCR_ANENABLE | BMCR_ANRESTART;
		if (rtl_phy_write(phy, MII_BMCR, bmcr))
			pr_warn("rtl8367d-nss: PHY%d wake failed\n", phy);
	}

	/* Give autonegotiation a chance, then report what actually came up. */
	msleep(3000);
	for (phy = 0; phy < 8; phy++) {
		u32 bmcr = 0, bmsr = 0;

		if (!(wake_phys & BIT(phy)))
			continue;
		rtl_phy_read(phy, MII_BMCR, &bmcr);
		rtl_phy_read(phy, MII_BMSR, &bmsr);
		pr_info("rtl8367d-nss: PHY%d after wake BMCR=0x%04x BMSR=0x%04x link=%s\n",
			phy, bmcr, bmsr, (bmsr & BMSR_LSTATUS) ? "UP" : "down");
	}
}

static void fixup_switch(void)
{
	int p;
	u32 v, v2;

	if (!(ports & BIT(cpu_port))) {
		pr_warn("rtl8367d-nss: ports=0x%02x lacks cpu_port %u, adding it\n",
			ports, cpu_port);
		ports |= BIT(cpu_port);
	}

	/*
	 * VLAN off.  rtl8365mb leaves the VLAN table and per-port ingress
	 * filtering behind when it is unbound, and with filtering on an
	 * untagged frame is discarded unless the port's PVID matches a VTU
	 * entry that still lists the CPU port.  A flat AP wants none of that:
	 * with VLAN disabled the fabric forwards purely on the isolation
	 * masks below, which is what an unmanaged switch does.
	 */
	rtl_update(RTL_VLAN_CTRL_REG, RTL_VLAN_CTRL_EN_MASK, 0);
	rtl_write(RTL_VLAN_INGRESS_REG, 0);
	rtl_read(RTL_VLAN_CTRL_REG, &v);
	pr_info("rtl8367d-nss: vlan_ctrl=0x%04x (vlan off)\n", v);

	/*
	 * CPU tag off.  The firmware parses 802.1Q, not the 8-byte Realtek tag
	 * that would otherwise sit where the ethertype belongs.
	 */
	rtl_write(RTL_CPU_PORT_MASK_REG, BIT(cpu_port));
	rtl_update(RTL_CPU_CTRL_REG, RTL_CPU_CTRL_EN_MASK, 0);
	rtl_read(RTL_CPU_CTRL_REG, &v);
	rtl_read(RTL_CPU_PORT_MASK_REG, &v2);
	pr_info("rtl8367d-nss: cpu_ctrl=0x%04x port_mask=0x%04x (tag off, cpu port kept)\n",
		v, v2);

	for (p = 0; p < RTL_NPORTS; p++) {
		if (!(ports & BIT(p)))
			continue;

		/* dsa_unregister_switch() left every port BR_STATE_DISABLED */
		rtl_update(RTL_MSTI_CTRL_REG(p), RTL_MSTI_STATE_MASK(p),
			   RTL_STP_FORWARDING << RTL_MSTI_STATE_OFF(p));

		/* forward to every other enabled port */
		rtl_write(RTL_PORT_ISOLATION_REG(p), ports & ~BIT(p));

		rtl_read(RTL_PORT_ISOLATION_REG(p), &v);
		pr_info("rtl8367d-nss: port%d isolation=0x%04x\n", p, v);
	}

	/* flood unknown UC/MC/BC to every enabled port */
	rtl_write(RTL_FLOOD_UC_REG, ports);
	rtl_write(RTL_FLOOD_MC_REG, ports);
	rtl_write(RTL_FLOOD_BC_REG, ports);
	pr_info("rtl8367d-nss: flooding mask 0x%04x on ports 0x%02x, cpu_port %u\n",
		ports, ports, cpu_port);

	rtl_read(0x1107, &v);
	pr_info("rtl8367d-nss: port linkup_ind=0x%04x\n", v);
	for (p = 0; p < RTL_NPORTS; p++) {
		if (!(ports & BIT(p)))
			continue;
		rtl_read(RTL_MSTI_CTRL_REG(p), &v);
		pr_info("rtl8367d-nss: port%d msti_reg=0x%04x state=%lu\n", p, v,
			(unsigned long)((v & RTL_MSTI_STATE_MASK(p)) >> RTL_MSTI_STATE_OFF(p)));
	}
}

static int __init rtl8367d_nss_init(void)
{
	struct mdio_device *mdiodev;
	struct device *d;
	u32 v;

	d = bus_find_device_by_name(&mdio_bus_type, NULL, switch_dev);
	if (!d) {
		pr_err("rtl8367d-nss: no mdio device '%s'\n", switch_dev);
		return -ENODEV;
	}

	mdiodev = to_mdio_device(d);
	sw_bus = mdiodev->bus;
	sw_addr = mdiodev->addr;

	/*
	 * The chip id/version registers only answer after a magic write, which
	 * doubles as a check that the register protocol is working at all.
	 */
	rtl_write(0x13C2, 0x0249);
	if (rtl_read(0x1300, &v)) {
		pr_err("rtl8367d-nss: register access failed on %s\n", switch_dev);
		put_device(d);
		return -EIO;
	}
	pr_info("rtl8367d-nss: %s addr %d chip_id=0x%04x\n", switch_dev, sw_addr, v);

	fixup_wake_phys();
	if (switch_fixup)
		fixup_switch();

	put_device(d);
	return 0;
}

static void __exit rtl8367d_nss_exit(void)
{
}

module_init(rtl8367d_nss_init);
module_exit(rtl8367d_nss_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("RTL8367D fabric re-arm after rtl8365mb teardown (headerless mode)");
