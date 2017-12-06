#include <linux/etherdevice.h>
#include <linux/bitops.h>
#include <linux/errno.h>
#include "rtl8xxxu.h"
#include "rtl8xxxu_regs.h"

struct rtl8814au_firmware_header {
	__le16	signature;
	u8	category;		/* AP/NIC and USB/PCI */
	u8	function;
	__le16	major_version;		/* FW Version */
	u8	minor_version;		/* FW Subversion, default 0x00 */
	u8	minor_version_idx;	/* FW Subversion Index */

	__le32	svn_idx;		/* SVN entry index */
	__le32	reserved1;

	u8	month;			/* Release time Month field */
	u8	date;			/* Release time Date field */
	u8	hour;			/* Release time Hour field */
	u8	minute;			/* Release time Minute field */
	__le16	year;			/* Release time Year field */
	u8	foundry;		/* Release time Foundry field */
	u8	reserved2;

	u8	mem_usage_dl_from_3081:1;
	u8	mem_usage_boot_from_3081:1;
	u8	mem_usage_boot_loader_3081:1;
	u8	mem_usage_iram_3081:1;
	u8	mem_usage_eram_3081:1;
	u8	reserved4:3;
	u8	reserved3;
	__le16	boot_loader_size;
	u32	reserved5;

	/*
	 * fw_cfg_size and fw_attr_size overlap total_dmem_size in the upstream
	 * headers, but they also skip the preceding 32 bits. Making an educated
	 * guess that this is where they really belong.
	 */
	__le16	fw_cfg_size;
	__le16	fw_attr_size;
	__le32	total_dmem_size;

	__le32	irom;
	__le32	erom;

	__le32	iram_size;
	__le32	eram_size;

	u32	reserved6;
	u32	reserved7;

	u8	data[0];
};

static int rtl8814au_parse_efuse(struct rtl8xxxu_priv *priv)
{
	struct rtl8814au_efuse *efuse = &priv->efuse_wifi.efuse8814;
	int i;

	if (efuse->rtl_id != cpu_to_le16(0x8129)) {
		return -EINVAL;
	}

	priv->rx_paths = hweight8(efuse->rf_antenna_option & 0x0f);
	priv->tx_paths = hweight8(efuse->rf_antenna_option & 0xf0);

	ether_addr_copy(priv->mac_addr, efuse->mac_addr);

	memcpy(priv->cck_tx_power_index_A,
		   efuse->tx_power_index_A.cck_base,
		   sizeof(efuse->tx_power_index_A.cck_base));
	memcpy(priv->cck_tx_power_index_B,
		   efuse->tx_power_index_B.cck_base,
		   sizeof(efuse->tx_power_index_B.cck_base));

	memcpy(priv->ht40_1s_tx_power_index_A,
		   efuse->tx_power_index_A.ht40_base,
		   sizeof(efuse->tx_power_index_A.ht40_base));
	memcpy(priv->ht40_1s_tx_power_index_B,
		   efuse->tx_power_index_B.ht40_base,
		   sizeof(efuse->tx_power_index_B.ht40_base));

	priv->ht20_tx_power_diff[0].a =
		efuse->tx_power_index_A.ht20_ofdm_1s_diff.b;
	priv->ht20_tx_power_diff[0].b =
		efuse->tx_power_index_B.ht20_ofdm_1s_diff.b;

	priv->ht40_tx_power_diff[0].a = 0;
	priv->ht40_tx_power_diff[0].b = 0;

	for(i = 1; i < RTL8723B_TX_COUNT; i++) {
		priv->ofdm_tx_power_diff[i].a =
			efuse->tx_power_index_A.pwr_diff[i - 1].ofdm;
		priv->ofdm_tx_power_diff[i].b =
			efuse->tx_power_index_B.pwr_diff[i - 1].ofdm;

		priv->ht20_tx_power_diff[i].a =
			efuse->tx_power_index_A.pwr_diff[i - 1].ht20;
		priv->ht20_tx_power_diff[i].b =
			efuse->tx_power_index_B.pwr_diff[i - 1].ht20;

		priv->ht40_tx_power_diff[i].a =
			efuse->tx_power_index_A.pwr_diff[i - 1].ht40;
		priv->ht40_tx_power_diff[i].b =
			efuse->tx_power_index_B.pwr_diff[i - 1].ht40;
	}

	priv->has_xtalk = 1;
	priv->xtalk = priv->efuse_wifi.efuse8814.xtal_k & 0x3f;
	return 0;
}

static int rtl8814au_send_firmware_page(struct rtl8xxxu_priv *priv,
					u8 *ptr, size_t page_size)
{
	struct ieee80211_tx_control control = {};
	struct ieee80211_tx_info *tx_info;
	struct sk_buff *skb;
	struct rtl8xxxu_tx_urb *tx_urb;
	u8 *data;
	u16 val16;
	int count, ret = 0;

	skb = netdev_alloc_skb(NULL,
			       (rtl8814au_fops.tx_desc_size + page_size));
	if(!skb) {
		ret = -ENOMEM;
		goto exit;
	}
	skb_reserve(skb, rtl8814au_fops.tx_desc_size);
	data = skb_put(skb, page_size);
	memcpy(data, ptr, page_size);

	/*
	 * Temporarily allocate a tx urb for sending the firmware beacons
	 */
	tx_urb = kmalloc(sizeof(struct rtl8xxxu_tx_urb), GFP_KERNEL);
	if(!tx_urb) {
		ret = -ENOMEM;
		goto exit;
	}
	usb_init_urb(&tx_urb->urb);
	INIT_LIST_HEAD(&tx_urb->list);
	tx_urb->hw = priv->hw;
	list_add(&tx_urb->list, &priv->tx_urb_free_list);
	priv->tx_urb_free_count++;
	priv->tx_stopped = false;

	tx_info = IEEE80211_SKB_CB(skb);
	tx_info->control.flags = 0;

	// Clear the beacon valid bit
	val16 = rtl8xxxu_read16(priv, REG_FIFOPAGE);
	val16 &= ~FIFOPAGE_BEACON_VALID;
	rtl8xxxu_write16(priv, REG_FIFOPAGE, val16);

	// Send the frame
	rtl8xxxu_tx(priv->hw, &control, skb);

	// Poll for beacon valid flag
	for (count = RTL8XXXU_MAX_REG_POLL; count; count--) {
		val16 = rtl8xxxu_read16(priv, REG_FIFOPAGE);
		if (val16 & FIFOPAGE_BEACON_VALID)
			break;

		udelay(10);
	}

	if(!count) {
		dev_err(&priv->udev->dev, "Failed to send firmware page");
		ret = -EBUSY;
		goto exit;
	}

	// TODO: Initiate a DMA transfer from the TX FIFO to the MCU
	// See IDDMADownLoadFW_3081
exit:
	//consume_skb(skb);
	return ret;
}

static int rtl8814au_send_firmware(struct rtl8xxxu_priv *priv, u8 *fw_bin,
				    size_t len) {
	size_t page_size = RTL8814A_TX_EXTBUF_SIZE - RTL8814A_TXDESC_SIZE;
	int ret = 0;

	while(len > 0) {
		len -= page_size;
		if(page_size > len) {
			// This is the last page
			ret = rtl8814au_send_firmware_page(priv, fw_bin, len);
			break;
		}else{
			// This is not the last page
			ret = rtl8814au_send_firmware_page(priv, fw_bin, len);
			if(ret)
				break;
		}
	}

	return ret;
}

static int rtl8814au_load_firmware(struct rtl8xxxu_priv *priv)
{
	struct rtl8814au_firmware_header *fw_data;
	u32 firmware_size;
	int ret;

	ret = rtl8xxxu_load_firmware(priv, "rtlwifi/rtl8814aufw.bin");
	if(ret) {
		goto exit;
	}

	fw_data = (struct rtl8814au_firmware_header *)priv->fw_data;

	dev_dbg(&priv->udev->dev,
		"RTL8814AU firmware %04d-%02d-%02dT%02d:%02d",
		le16_to_cpu(fw_data->year), fw_data->month, fw_data->date,
		fw_data->hour, fw_data->minute);
	dev_dbg(&priv->udev->dev,
		"3081 DL:%d BOOT_FROM:%d BOOT_LOADER:%d IRAM:%d ERAM:%d\n",
		fw_data->mem_usage_dl_from_3081,
		fw_data->mem_usage_boot_from_3081,
		fw_data->mem_usage_boot_loader_3081,
		fw_data->mem_usage_iram_3081,
		fw_data->mem_usage_eram_3081);
	dev_dbg(&priv->udev->dev,
		"3081 boot_loader=%d total_dmem=%d\n",
		le16_to_cpu(fw_data->boot_loader_size),
		le32_to_cpu(fw_data->total_dmem_size) +
		RTL8814A_FIRMWARE_CHKSUM_SIZE);
	dev_dbg(&priv->udev->dev,
		"3081 fw_cfg_size=%d fw_attr_size=%d\n",
		le16_to_cpu(fw_data->fw_cfg_size),
		le16_to_cpu(fw_data->fw_attr_size));
	dev_dbg(&priv->udev->dev,
		"3081 irom=%d erom=%d iram=%d eram=%d\n",
		le32_to_cpu(fw_data->irom),
		le32_to_cpu(fw_data->erom),
		le32_to_cpu(fw_data->iram_size) +
		RTL8814A_FIRMWARE_CHKSUM_SIZE,
		le32_to_cpu(fw_data->eram_size));

	firmware_size =	(sizeof(struct rtl8814au_firmware_header) +
			 le32_to_cpu(fw_data->total_dmem_size) +
			 RTL8814A_FIRMWARE_CHKSUM_SIZE +
			 le32_to_cpu(fw_data->iram_size) +
			 RTL8814A_FIRMWARE_CHKSUM_SIZE);

	if(firmware_size != priv->fw_size) {
		dev_warn(&priv->udev->dev,
			 "Firmware header size mismatch (%d != %zu)\n",
			 firmware_size, priv->fw_size);

		ret = -EINVAL;
		goto exit;
	}

exit:
	return ret;
}

static int rtl8814au_emu_to_active(struct rtl8xxxu_priv *priv)
{
	u32 val32;
	int count, ret = 0;

	/* disable PFM_LDKP */
	val32 = rtl8xxxu_read8(priv, REG_APS_FSMCO);
	val32 &= ~APS_FSMCO_PFM_LDKP;
	rtl8xxxu_write32(priv, REG_APS_FSMCO, val32);

	/* wait till autoload done */
	for (count = RTL8XXXU_MAX_REG_POLL; count; count--) {
		val32 = rtl8xxxu_read32(priv, REG_APS_FSMCO);
		if (val32 & APS_FSMCO_PFM_ALDN)
			break;

		udelay(10);
	}

	if(!count) {
		ret = -EBUSY;
		goto exit;
	}

	/* disable WL suspend*/
	val32 = rtl8xxxu_read32(priv, REG_APS_FSMCO);
	val32 &= ~APS_FSMCO_HW_SUSPEND;
	rtl8xxxu_write32(priv, REG_APS_FSMCO, val32);

	val32 = rtl8xxxu_read32(priv, REG_SYS_CFG);
	val32 &= ~SYS_CFG_SW_OFFLOAD_EN;
	rtl8xxxu_write32(priv, REG_SYS_CFG, val32);

	val32 = rtl8xxxu_read32(priv, REG_WOL_EVENT);
	val32 |= BIT(5);
	val32 &= ~BIT(4);
	rtl8xxxu_write32(priv, REG_WOL_EVENT, val32);

	/* polling until return 0*/
	for (count = RTL8XXXU_MAX_REG_POLL; count; count--) {
		val32 = rtl8xxxu_read32(priv, REG_APS_FSMCO);
		if ((val32 & APS_FSMCO_PFM_LDALL) == 0)
			break;

		udelay(10);
	}

	if(!count) {
		dev_err(&priv->udev->dev, "Hit MAX_REG_POLL waiting for APS_FSMCO_PFM_LDALL: REG_APS_FSMCO=0x%08x\n", val32);
		ret = -EBUSY;
		goto exit;
	}

exit:
	return ret;
}

static void rtl8814au_set_led(struct rtl8xxxu_priv *priv, int on)
{
	struct rtl8814au_gpio_ctrl gpio2;
	u32 val32;

	val32 = rtl8xxxu_read32(priv, REG_GPIO_PIN_CTRL_2);
	memcpy(&gpio2, &val32, sizeof(val32));

	// 0 = input; 1 = output
	gpio2.direction |= GPIO_IO_PINS_LED0;

	if(on == 1) {
		gpio2.out &= ~GPIO_IO_PINS_LED0;
	} else {
		gpio2.out |= GPIO_IO_PINS_LED0;
	}

	memcpy(&val32, &gpio2, sizeof(val32));
	rtl8xxxu_write32(priv, REG_GPIO_PIN_CTRL_2, val32);
}

static int rtl8814au_power_on(struct rtl8xxxu_priv *priv)
{
	u16 val16;
	u8 val8;
	int ret = -EINVAL;

	rtl8xxxu_disabled_to_emu(priv);

	/* absolutely no idea what this register is */
	val8 = rtl8xxxu_read8(priv, 0x10C2);
	val8 |= BIT(1);
	rtl8xxxu_write8(priv, 0x10C2, val8);

	ret = rtl8814au_emu_to_active(priv);
	if(ret != 0) {
		goto exit;
	}

	/*
	 * Enable MAC DMA/WMAC/SCHEDULE/SEC block
	 * Set CR bit10 to enable 32k calibration.
	 */
	val16 = rtl8xxxu_read16(priv, REG_CR);
	val16 |= (CR_HCI_TXDMA_ENABLE | CR_HCI_RXDMA_ENABLE |
			CR_TXDMA_ENABLE | CR_RXDMA_ENABLE |
			CR_PROTOCOL_ENABLE | CR_SCHEDULE_ENABLE |
			CR_MAC_TX_ENABLE | CR_MAC_RX_ENABLE |
			CR_SECURITY_ENABLE | CR_CALTIMER_ENABLE);
	rtl8xxxu_write16(priv, REG_CR, val16);

	rtl8814au_set_led(priv, 1);

exit:
	return ret;
}

static int rtl8814au_init_phy_rf(struct rtl8xxxu_priv *priv)
{
	return -EINVAL;
}

static int rtl8814au_init_llt_table(struct rtl8xxxu_priv *priv)
{
	u8 val8;
	int count, ret = 0;

	val8 = rtl8xxxu_read8(priv, REG_AUTO_LLT_8814A);
	val8 |= BIT(0);
	rtl8xxxu_write8(priv, REG_AUTO_LLT_8814A, val8);

	for (count = RTL8XXXU_MAX_REG_POLL; count; count--) {
		val8 = rtl8xxxu_read8(priv, REG_AUTO_LLT_8814A);
		if (!(val8 & BIT(0)))
			break;

		udelay(10);
	}

	if(!count) {
		ret = -EBUSY;
		goto exit;
	}

exit:
	return ret;
}

static int rtl8814au_download_firmware(struct rtl8xxxu_priv *priv)
{
	/*
	 * RTL8814A has a fancy new MCU (3081?) that loads firmware by reading
	 * faked beacon frames. Enable downloads, disable hardware beacon
	 * handling, and send some frames the hard way.
	 */
	u32 val32;
	u16 val16;
	u8 val8;
	int count, ret = 0;

	struct rtl8814au_firmware_header *fw_data;
	u8 *dmem_ptr;
	u8 *iram_ptr;

	fw_data = (struct rtl8814au_firmware_header *)priv->fw_data;

	/*
	 * MCU firmware download enable
	 *
	 * This bitwise math makes no sense, but it's what the official driver
	 * does, so we'll copy the behavior for now.
	 */
	val16 = rtl8xxxu_read16(priv, REG_MCU_FW_DL);
	val16 &= 0x3000;
	val16 &= ~BIT(12);
	val16 |= BIT(13);
	val16 |= MCU_FW_DL_ENABLE;
	rtl8xxxu_write16(priv, REG_MCU_FW_DL, val16);

	/* 3081 Disable */
	val16 = rtl8xxxu_read16(priv, REG_SYS_FUNC);
	val16 &= ~SYS_FUNC_CPU_ENABLE;
	rtl8xxxu_write16(priv, REG_SYS_FUNC, val16);

	/* Set REG_CR bit 8, SW DMA beacon */
	val16 = rtl8xxxu_read16(priv, REG_CR);
	val16 |= CR_SW_BEACON_ENABLE;
	rtl8xxxu_write16(priv, REG_CR, val16);

	/* Toggle hardware beacon protection? */
	val8 = rtl8xxxu_read8(priv, REG_BEACON_CTRL);
	val8 |= BEACON_ATIM;
	val8 &= ~BEACON_FUNCTION_ENABLE;
	rtl8xxxu_write8(priv, REG_BEACON_CTRL, val8);

	val8 = rtl8xxxu_read8(priv, REG_BEACON_CTRL);
	val8 |= BEACON_DISABLE_TSF_UPDATE;
	val8 &= ~BEACON_ATIM;
	rtl8xxxu_write8(priv, REG_BEACON_CTRL, val8);

	/*
	 * Set FWHW_TXQ_CTRL 0x422[6]=0 to tell hw the packet is not a real
	 * beacon frame
	 */
	val8 = rtl8xxxu_read8(priv, REG_FWHW_TXQ_CTRL + 2);
	val8 &= ~BIT(6);
	rtl8xxxu_write8(priv, REG_FWHW_TXQ_CTRL + 2, val8);

	/* Set the head page of bcnq packet */
	rtl8xxxu_write16(priv, REG_FIFOPAGE, TX_TOTAL_PAGE_NUM_8814A);

	dmem_ptr = fw_data->data +
		   (fw_data->total_dmem_size + RTL8814A_FIRMWARE_CHKSUM_SIZE);
	ret = rtl8814au_send_firmware(priv, dmem_ptr, fw_data->total_dmem_size);
	if(ret) {
		dev_err(&priv->udev->dev, "Error loading MCU dmem image\n");
		goto exit;
	}
	dev_info(&priv->udev->dev, "Loaded MCU dmem image\n");

	if(fw_data->mem_usage_iram_3081) {
		iram_ptr = fw_data->data +
			   (fw_data->iram_size + RTL8814A_FIRMWARE_CHKSUM_SIZE);
		rtl8814au_send_firmware(priv, iram_ptr, fw_data->iram_size);
		if(ret) {
			dev_err(&priv->udev->dev, "Error loading MCU iram image\n");
			goto exit;
		}
		dev_info(&priv->udev->dev, "Loaded MCU iram image\n");
	}

	/* 3081 Enable */
	val16 = rtl8xxxu_read16(priv, REG_SYS_FUNC);
	val16 |= SYS_FUNC_CPU_ENABLE;
	rtl8xxxu_write16(priv, REG_SYS_FUNC, val16);

	/* MCU firmware download disable */
	val16 = rtl8xxxu_read16(priv, REG_MCU_FW_DL);
	val16 &= ~MCU_FW_DL_ENABLE;
	rtl8xxxu_write16(priv, REG_MCU_FW_DL, val16);

	for (count = RTL8XXXU_MAX_REG_POLL; count; count--) {
		val32 = rtl8xxxu_read32(priv, REG_MCU_FW_DL);
		if ((val32 & MCU_CPU_DL_READY)) {
			dev_info(&priv->udev->dev, "MCU_CPU_DL_READY!\n");
			break;
		}

		udelay(10);
	}

	if(!count) {
		dev_err(&priv->udev->dev, "3081 firmware did not start\n");
		ret = -EBUSY;
		goto exit;
	}

exit:
	return ret;
}

struct rtl8xxxu_fileops rtl8814au_fops = {
	.parse_efuse = rtl8814au_parse_efuse,
	.load_firmware = rtl8814au_load_firmware,
	.download_firmware = rtl8814au_download_firmware,
	.power_on = rtl8814au_power_on,
	.power_off = rtl8xxxu_power_off,
	.reset_8051 = rtl8xxxu_reset_8051,
	.llt_init = rtl8814au_init_llt_table,
	.init_phy_bb = rtl8xxxu_gen1_init_phy_bb,
	.init_phy_rf = rtl8814au_init_phy_rf,
	.phy_iq_calibrate = rtl8xxxu_gen1_phy_iq_calibrate,
	.config_channel = rtl8xxxu_gen1_config_channel,
	.parse_rx_desc = rtl8xxxu_parse_rxdesc16,
	.init_aggregation = rtl8xxxu_gen1_init_aggregation,
	.enable_rf = rtl8xxxu_gen1_enable_rf,
	.disable_rf = rtl8xxxu_gen1_disable_rf,
	.usb_quirks = rtl8xxxu_gen1_usb_quirks,
	.set_tx_power = rtl8xxxu_gen1_set_tx_power,
	.update_rate_mask = rtl8xxxu_update_rate_mask,
	.report_connect = rtl8xxxu_gen1_report_connect,
	.fill_txdesc = rtl8xxxu_fill_txdesc_v2,
	.writeN_block_size = 1024,
	.rx_agg_buf_size = 16000,
	.tx_desc_size = sizeof(struct rtl8xxxu_txdesc40),
	.rx_desc_size = sizeof(struct rtl8xxxu_rxdesc16),
	.adda_1t_init = 0x0b1b25a0,
	.adda_1t_path_on = 0x0bdb25a0,
	.adda_2t_path_on_a = 0x04db25a4,
	.adda_2t_path_on_b = 0x0b1b25a4,
	.trxff_boundary = 0x27ff,
	.pbp_rx = PBP_PAGE_SIZE_128,
	.pbp_tx = PBP_PAGE_SIZE_128,
	.mactable = rtl8xxxu_gen1_mac_init_table,
	.total_page_num = TX_TOTAL_PAGE_NUM,
	.page_num_hi = TX_PAGE_NUM_HI_PQ,
	.page_num_lo = TX_PAGE_NUM_LO_PQ,
	.page_num_norm = TX_PAGE_NUM_NORM_PQ,
};
