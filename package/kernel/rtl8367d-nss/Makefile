include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=rtl8367d-nss
PKG_RELEASE:=1
PKG_LICENSE:=GPL-2.0-only

include $(INCLUDE_DIR)/package.mk

define KernelPackage/rtl8367d-nss
  SECTION:=kernel
  CATEGORY:=Kernel modules
  SUBMENU:=Network Devices
  TITLE:=RTL8367D fabric re-arm after rtl8365mb teardown
  DEPENDS:=@TARGET_qualcommax_ipq50xx
  FILES:=$(PKG_BUILD_DIR)/rtl8367d-nss.ko
endef

define KernelPackage/rtl8367d-nss/description
  Runtime helper for the IPQ5018 NSS data path on Realtek-switched boards:
  after rtl8365mb is unbound (dsa_unregister_switch leaves every port in
  BR_STATE_DISABLED and phy_detach suspends the front PHYs), wake the PHYs
  and re-arm the fabric with the CPU tag off, so the conduit carries plain
  802.1q frames the NSS firmware can parse. Loaded on demand, never at boot.
endef

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

define Build/Compile
	+$(KERNEL_MAKE) M="$(PKG_BUILD_DIR)" modules
endef

$(eval $(call KernelPackage,rtl8367d-nss))
