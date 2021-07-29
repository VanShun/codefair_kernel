#ifndef		_WOKOO_USB_H
#define		_WOKOO_USB_H

#define		USB_BASE									0xA0300000
#define		CTL_BASEADDR								0xA0060000

#define		CTL_USB_OTG_PHY_RST_CTRL					((volatile unsigned  int*)( CTL_BASEADDR + 0x150 ))
#define		CTL_USB_OTG_PHY_SUSPEND						((volatile unsigned  int*)( CTL_BASEADDR + 0x154 ))
#define		CTL_USB_OTG_PHY_CFG0						((volatile unsigned  int*)( CTL_BASEADDR + 0x168 ))
#define		CTL_USB_OTG_PHY_CFG1						((volatile unsigned  int*)( CTL_BASEADDR + 0x16c ))





#define 	USB_HOST_BASE								( USB_BASE + 0x400 )

#define 	USB_HFIR									((volatile unsigned  *)( USB_HOST_BASE + 0x04 ))
#define 	USB_HAINTMSK								((volatile unsigned  *)( USB_HOST_BASE + 0x18 ))
#define 	USB_HPRT									((volatile unsigned  *)( USB_HOST_BASE + 0x40 ))

#define 	USB_HCCHAR_CH0								((volatile unsigned  *)( USB_HOST_BASE + 0x100 ))
#define 	USB_HCINTMSK_CH0							((volatile unsigned  *)( USB_HOST_BASE + 0x10C ))
#define 	USB_HCTSIZ_CH0								((volatile unsigned  *)( USB_HOST_BASE + 0x110 ))
#define 	USB_HCINTMSK_CH1							((volatile unsigned  *)( USB_HOST_BASE + 0x12C ))
#define 	USB_DEVICE_BASE								( USB_BASE + 0x800 )


#define 	USB_GOTGCTL									((volatile unsigned  *)( USB_BASE + 0x00 ))
#define 	USB_GOTGINT									((volatile unsigned  *)( USB_BASE + 0x04 ))
#define 	USB_GAHBCFG									((volatile unsigned  *)( USB_BASE + 0x08 ))
#define 	USB_GUSBCFG									((volatile unsigned  *)( USB_BASE + 0x0C ))
#define 	USB_GRSTCTL									((volatile unsigned  *)( USB_BASE + 0x10 ))
#define 	USB_GINTSTS									((volatile unsigned  *)( USB_BASE + 0x14 ))
#define 	USB_GINTMSK									((volatile unsigned  *)( USB_BASE + 0x18 ))
#define 	USB_GRXSTSR									((volatile unsigned  *)( USB_BASE + 0x1C ))
#define 	USB_GRXSTSP									((volatile unsigned  *)( USB_BASE + 0x20 ))
#define 	USB_GRXFSIZ									((volatile unsigned  *)( USB_BASE + 0x24 ))
#define 	USB_GNPTXFSIZ		        				((volatile unsigned  *)( USB_BASE + 0x28 ))
#define 	USB_GNPTXSTS								((volatile unsigned  *)( USB_BASE + 0x2C ))

#define 	USB_GPWRGN								((volatile unsigned  *)( USB_BASE + 0x58 ))


#define 	USB_HPTXFSIZ 		        				((volatile unsigned  *)( USB_BASE + 0x100 ))
#define 	USB_DIEPTXF0 		        				((volatile unsigned  *)( USB_BASE + 0x104 ))
#define 	USB_DIEPTXF1		        				((volatile unsigned  *)( USB_BASE + 0x108 ))
#define 	USB_DIEPTXF2 		        				((volatile unsigned  *)( USB_BASE + 0x10c ))
#define 	USB_DIEPTXF3 		        				((volatile unsigned  *)( USB_BASE + 0x110 ))
#define 	USB_DIEPTXF4 		        				((volatile unsigned  *)( USB_BASE + 0x114 ))
#define 	USB_DIEPTXF5 		        				((volatile unsigned  *)( USB_BASE + 0x118 ))
#define 	USB_DIEPTXF6 		        				((volatile unsigned  *)( USB_BASE + 0x11c ))

#define 	USB_DCFG		     	    				((volatile unsigned  *)( USB_DEVICE_BASE + 0x00 ))
#define 	USB_DCTL		     	    				((volatile unsigned  *)( USB_DEVICE_BASE + 0x04 ))
#define 	USB_DSTS		     	    				((volatile unsigned  *)( USB_DEVICE_BASE + 0x08 ))
#define 	USB_DIEPMSK		     	    				((volatile unsigned  *)( USB_DEVICE_BASE + 0x10 ))
#define 	USB_DOEPMSK		     	    				((volatile unsigned  *)( USB_DEVICE_BASE + 0x14 ))
#define 	USB_DAINT		     	    				((volatile unsigned  *)( USB_DEVICE_BASE + 0x18 ))
#define 	USB_DAINTMSK		        				((volatile unsigned  *)( USB_DEVICE_BASE + 0x1C ))
#define 	USB_DVBUSDIS	      	    				((volatile unsigned  *)( USB_DEVICE_BASE + 0x28 ))
#define 	USB_DVBUSPULSE 		        				((volatile unsigned  *)( USB_DEVICE_BASE + 0x2C ))
#define 	USB_DTHRCTL 		        				((volatile unsigned  *)( USB_DEVICE_BASE + 0x30 ))
#define 	USB_DIEPEMPMSK 		        				((volatile unsigned  *)( USB_DEVICE_BASE + 0x34 ))
#define 	USB_DIEPCTL0    					     	((volatile unsigned *)( USB_BASE + 0x900 ))
#define 	USB_DIENINT0								((volatile unsigned *)( USB_BASE + 0x908 ))
#define 	USB_DIEPTSIZ0   					      	((volatile unsigned *)( USB_BASE + 0x910 ))
#define 	USB_DOEPCTL0    					     	((volatile unsigned *)( USB_BASE + 0xB00 ))
#define 	USB_DOEPINT0    					     	((volatile unsigned *)( USB_BASE + 0xB08 ))
#define 	USB_DOEPTSIZ0   					      	((volatile unsigned *)( USB_BASE + 0xB10 ))
#define 	USB_DOEPCTL8    					     	((volatile unsigned *)( USB_BASE + 0xC00 ))
#define 	USB_DOEPTSIZ8   					      	((volatile unsigned *)( USB_BASE + 0xC10 ))
#define 	USB_RXFIFO_EP0  					      	((volatile unsigned *)( USB_BASE + 0x1000 ))
#define 	USB_NPTXFIFO_EP0					        ((volatile unsigned *)( USB_BASE + 0x1000 ))
#define 	USB_RXFIFO_EP8  					      	((volatile unsigned *)( USB_BASE + 0x9000 ))
#define 	USB_DIEPCTL1								((volatile unsigned *)( USB_BASE +  0x920))
#define 	USB_DIEPTSIZ1								((volatile unsigned *)( USB_BASE +  0x930))

#define 	USB_PCGCCTL									((volatile unsigned *)( USB_BASE +  0xe00))

#define 	REG32(reg)									(*(volatile unsigned long *)(reg))
#define 	USB_DIEPCTL(x)								(USB_DIEPCTL0+(x)*0x20) //x=EP No.:frome ep1 to ep7
#define 	USB_DOEPCTL(x)								(USB_DOEPCTL0+(x)*0x20) //x=EP No.:frome ep8 to ep14

#define 	USB_DATA_FIFO0									((volatile unsigned *)( USB_BASE +  0x1000))

#endif
