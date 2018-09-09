/* SPDX-License-Identifier: GPL-2.0 */
/*
 * This header provides clock numbers for the ingenic,jz4760-cgu DT binding.
 */

#ifndef __DT_BINDINGS_CLOCK_JZ4760_CGU_H__
#define __DT_BINDINGS_CLOCK_JZ4760_CGU_H__

#define JZ4760_CLK_EXT		0	// EXCLK
#define JZ4760_CLK_RTC		1	// RTCLK
#define JZ4760_CLK_PLL0		2
#define JZ4760_CLK_PLL1		3
#define JZ4760_CLK_CCLK		4	// CPU
#define JZ4760_CLK_PCLK		7	// APB
#define JZ4760_CLK_MCLK		8	// MCLK (EMC or DDR)
#define JZ4760_CLK_SCLK		9	// NEMC (SRAM)

#define JZ4760_CLK_I2S		10	// USB Device
#define JZ4760_CLK_OTG		11	// USB Device
#define JZ4760_CLK_LP		12	// LPCLK (LCD Pixel Clock)
#define JZ4760_CLK_MSC0_MUX	13	// MMC0
#define JZ4760_CLK_UHC		14	// UHC48M (USB Host)
#define JZ4760_CLK_SSI0_MUX	15	// SSICLK
#define JZ4760_CLK_CIM		16	// CIM_MCLK
#define JZ4760_CLK_GPS		17	// GPSCLK
#define JZ4760_CLK_PCM		18	// PCMCLK
#define JZ4760_CLK_GPU		19	// GPUCLK

#define JZ4760_CLK_NEMC		20	// External NAND
#define JZ4760_CLK_BCH		21	// ECC encoder/decoder
#define JZ4760_CLK_I2C0		22
#define JZ4760_CLK_I2C1		23
#define JZ4760_CLK_SCC		24	// Smart Card Controller
#define JZ4760_CLK_TSSI		25	// MPEG-2 TS Slave Interface
#define JZ4760_CLK_OWI		26	// 1-Wire Interface
#define JZ4760_CLK_MSC1		27
#define JZ4760_CLK_MSC2		28
#define JZ4760_CLK_KBC		29	// PS/2 Keyboard Controller
#define JZ4760_CLK_SADC		30
#define JZ4760_CLK_UART0	31
#define JZ4760_CLK_UART1	32
#define JZ4760_CLK_UART2	33
#define JZ4760_CLK_UART3	34
#define JZ4760_CLK_SSI1		35
#define JZ4760_CLK_SSI2		35
#define JZ4760_CLK_DMA		36
#define JZ4760_CLK_MDMA		37	// External memory DMA controller
#define JZ4760_CLK_TVE		38	// TV encoder
#define JZ4760_CLK_IPU		39	// Image Process Unit
#define JZ4760_CLK_DDR		40

/* VPU blocks */
#define JZ4760_CLK_BDMA		41
#define JZ4760_CLK_MC		42	// Motion Compensation
#define JZ4760_CLK_DBLK		43	// De-Block
#define JZ4760_CLK_ME		44	// Motion Estimation
#define JZ4760_CLK_DCT		45	// Direct Cosine Transform
#define JZ4760_CLK_SRAM		46
#define JZ4760_CLK_CABAC	47	// Context-based Binary Arithmetic Decoding
#define JZ4760_CLK_AHB1		48

#endif /* __DT_BINDINGS_CLOCK_JZ4760_CGU_H__ */
