#include <linux/usb.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/bitops.h>
#include <linux/usb/net2280.h>
#include <linux/uaccess.h>

#define PLX_VENDOR_ID 0x0525
#define PLX_PRODUCT_ID 0x3380
#define PLX_DEVICE_NAME "pciemem"

/* USB part */
#define PLX_PCI_ENDPOINT 0xe
#define PLX_CSR_ENDPOINT 0xd
#define PLX_DMA_ENDPOINT 0x2

/* General Purpose Endpoint registers */
#define REG_GPEP0_CFG		0x320
#define REG_GPEP0_RSP		0x324 // CHECK!
#define REG_GPEP0_STAT	0x32c
#define REG_GPEP0_AVAIL	0x330
#define REG_GPEP1_CFG		0x340
#define REG_GPEP1_RSP		0x344
#define REG_GPEP1_STAT	0x40c
#define REG_GPEP1_AVAIL	0x410

/* DMA registers */
#define REG_DMA0_CTL		0x180
#define REG_DMA0_STAT		0x184
#define REG_DMA0_COUNT	0x190
#define REG_DMA0_ADDR		0x194

#define REG_DMA1_CTL		0x1A0
#define REG_DMA1_STAT		0x1A4
#define REG_DMA1_COUNT	0x1B0
#define REG_DMA1_ADDR		0x1B4

static const u32 dmactl_default =
		  (1 << DMA_SCATTER_GATHER_DONE_INTERRUPT)
		| (1 << DMA_CLEAR_COUNT_ENABLE)
		/* erratum 0116 workaround part 1 (use POLLING) */
		| (POLL_100_USEC << DESCRIPTOR_POLLING_RATE)
		| (1 << DMA_VALID_BIT_POLLING_ENABLE)
		| (1 << DMA_VALID_BIT_ENABLE)
		//| (1 << DMA_SCATTER_GATHER_ENABLE)
		/* erratum 0116 workaround part 2 (no AUTOSTART) */
		| (1 << DMA_ENABLE);

/* Get a minor range for your devices from the usb maintainer */
#define USB_PLX_MINOR_BASE	192
#define WRITES_IN_FLIGHT	8

#define MAX_TRANSFER		(PAGE_SIZE - 512)
/* MAX_TRANSFER is chosen so that the VM is not stressed by
   allocations > PAGE_SIZE and the number of packets in a page
   is an integer 512 is the largest possible packet on EHCI */

#define DMA_BUFLEN 4 * 1024 * 1024
static int plx_probe(struct usb_interface *interface, const struct usb_device_id *id);
static void plx_disconnect(struct usb_interface *interface);
static int plx_suspend(struct usb_interface *intf, pm_message_t message);
static int plx_post_reset(struct usb_interface *intf);

/* direct copy from usb-skeleton.c */
static int skel_resume(struct usb_interface *intf);
static int skel_pre_reset(struct usb_interface *intf);

static struct usb_device_id plx_table [] = {
	    { USB_DEVICE(PLX_VENDOR_ID, PLX_PRODUCT_ID) },
			    { }
};
static struct usb_driver plx_driver = {
  .name = PLX_DEVICE_NAME,
  .id_table = plx_table,
  .probe = plx_probe,
  .disconnect = plx_disconnect,
	.suspend =	plx_suspend,
	.resume =	skel_resume,
	.pre_reset =	skel_pre_reset,
	.post_reset =	plx_post_reset,
	.supports_autosuspend = 1,
};

static ssize_t pciemem_read(struct file *, char *, size_t, loff_t *);
static ssize_t pciemem_write(struct file *, const char *, size_t, loff_t *);
static int pciemem_open(struct inode *, struct file *);
static int pciemem_release(struct inode *, struct file *);
static int pciemem_flush(struct file *file, fl_owner_t id);

static struct file_operations fops = {
	.owner = THIS_MODULE,
  .read = pciemem_read,
  .write = pciemem_write,
  .open = pciemem_open,
  .release = pciemem_release,
	.flush =	pciemem_flush,
	.llseek =	generic_file_llseek,
};

static struct usb_class_driver plx_class = {
	.name = PLX_DEVICE_NAME"%d",
	.fops = &fops,
	.minor_base =	USB_PLX_MINOR_BASE,
};

struct usb_plx {
	struct usb_device	*udev;			/* the usb device for this device */
	struct usb_interface	*interface;		/* the interface for this device */
	struct semaphore	limit_sem;		/* limiting the number of writes in progress */
	struct usb_anchor	submitted;		/* in case we need to retract our submissions */
	unsigned char           *bulk_in_buffer;	/* the buffer to receive data */
	size_t			bulk_in_size;		/* the size of the receive buffer */
	size_t			bulk_in_filled;		/* number of bytes in the buffer */
	size_t			bulk_in_copied;		/* already copied to user space */
	__u8			bulk_in_endpointAddr;	/* the address of the bulk in endpoint */
	unsigned char           *bulk_out_buffer;	/* the buffer to transmit data */
	size_t			bulk_out_size;		/* the size of the receive buffer */
	__u8			bulk_out_endpointAddr;	/* the address of the bulk out endpoint */
	__u8			csr_in_endpointAddr;	/* the address of the CSR in endpoint */
	__u8			csr_out_endpointAddr;	/* the address of the CSR out endpoint */
	int			errors;			/* the last request tanked */
	bool			ongoing_read;		/* a read is going on */
	spinlock_t		err_lock;		/* lock for errors */
	struct kref		kref;
	struct mutex		io_mutex;		/* synchronize I/O with disconnect */
	wait_queue_head_t	bulk_in_wait;		/* to wait for an ongoing read */
};

static void plx_draw_down(struct usb_plx *dev);

static int plx_readreg(struct usb_plx *dev, u16 reg);
#define to_plx_dev(d) container_of(d, struct usb_plx, kref)
