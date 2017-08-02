#include <linux/module.h>
#include "pciemem.h"

/* © 2016-2017 ANSSI
 * Yves-Alexis Perez <yves-alexis.perez@ssi.gouv.fr>
 *
 * This kernel module is heavily based on Greg KH usb-skeleton.c USB module
 *
 * It should be used with an USB3380 based board, in order to access the
 * physical memory of a machine under test (T). This kernel module is to be loaded
 * on the analysis machine (A). The USB3380 board should be inserted on a PCI
 * slot on machine T, and a USB cable should be plugged between USB3380 and
 * machine A.
 *
 */

MODULE_LICENSE("GPL");

/* USB part */

static void plx_delete(struct kref *kref)
{
	struct usb_plx *dev = to_plx_dev(kref);

	usb_put_dev(dev->udev);
	kfree(dev->bulk_in_buffer);
	kfree(dev->bulk_out_buffer);
	kfree(dev);
}

static int pciemem_open(struct inode *inode, struct file *file)
{
	struct usb_plx *dev;
	struct usb_interface *interface;
	int subminor;
	int retval = 0;
	pr_debug("open pciemem\n");

	subminor = iminor(inode);

	interface = usb_find_interface(&plx_driver, subminor);
	if (!interface) {
		pr_err("%s - error, can't find device for minor %d\n",
			__func__, subminor);
		retval = -ENODEV;
		goto exit;
	}

	dev = usb_get_intfdata(interface);
	if (!dev) {
		retval = -ENODEV;
		goto exit;
	}

	retval = usb_autopm_get_interface(interface);
	if (retval)
		goto exit;

	/* increment our usage count for the device */
	kref_get(&dev->kref);

	/* save our object in the file's private structure */
	file->private_data = dev;

	pr_debug("opened pciemem\n");

exit:
	return retval;
}

static int pciemem_release(struct inode *inode, struct file *file)
{
	struct usb_plx *dev;

	dev = file->private_data;
	if (dev == NULL)
		return -ENODEV;

	/* allow the device to be autosuspended */
	mutex_lock(&dev->io_mutex);
	if (dev->interface)
		usb_autopm_put_interface(dev->interface);
	mutex_unlock(&dev->io_mutex);

	/* decrement the count on our device */
	kref_put(&dev->kref, plx_delete);

	pr_debug("released pciemem\n");
	return 0;
}

static int pciemem_flush(struct file *file, fl_owner_t id)
{
	struct usb_plx *dev;
	int res;

	dev = file->private_data;
	if (dev == NULL)
		return -ENODEV;

	/* wait for io to stop */
	mutex_lock(&dev->io_mutex);
	plx_draw_down(dev);

	/* read out errors, leave subsequent opens a clean slate */
	spin_lock_irq(&dev->err_lock);
	res = dev->errors ? (dev->errors == -EPIPE ? -EPIPE : -EIO) : 0;
	dev->errors = 0;
	spin_unlock_irq(&dev->err_lock);

	mutex_unlock(&dev->io_mutex);

	return res;
}

static int plx_readreg(struct usb_plx *dev, u16 reg)
{
	int ret = -EINVAL;
	u32 csrctl = 0U; // Format p362
	u32 *buf;
	int tlen = 0;
	int pipe;

  csrctl |= 0b01 << 4; // select USB configuration registers
  csrctl |= 1 << 6; // CSR Start
  csrctl |= 1 << 7; // CSR Read transaction
  csrctl |= (uint32_t ) reg << 16; // destination address

	buf = kmalloc(4, GFP_KERNEL);
	if (!buf) {
		pr_err("Can't allocate memory for USB buffer");
		return -ENOMEM;
	}
	memcpy(buf, &csrctl, 4);

	pipe = usb_sndbulkpipe(dev->udev, dev->csr_out_endpointAddr);
	ret = usb_bulk_msg(dev->udev, pipe, buf, 4, &tlen, 1000);

	if (ret != 0 || tlen != 4) {
		pr_warn("Read reg USB out transfer failed: ret = %d, tlen = %d\n", ret, tlen);
		return ret;
	}

	memset(buf, 0, 4);
	ret = usb_bulk_msg(dev->udev,
			usb_rcvbulkpipe(dev->udev, dev->csr_in_endpointAddr),
			buf, 4, &tlen, 1000);
	if (ret != 0 || tlen != 4)
		pr_warn("USB in  transfer failed: ret = %d, tlen = %d\n", ret, tlen);

	memcpy(&ret, buf, 4);
	return ret;
}

static int plx_writereg(struct usb_plx *dev, u16 reg, u32 data)
{
	int ret = -EINVAL;
	u32 csrctl = 0U; // Format p362
	u32 *buf;
	int tlen = 0;
	unsigned int pipe;

	pr_debug("reg=0x%x, data=0x%.8x\n",reg, data);

	csrctl |= 0b1111; // CSR Bytes enable (all bytes)
  csrctl |= 0b01 << 4; // select USB configuration registers
  csrctl |= 1 << 6; // CSR Start
  csrctl |= 0 << 7; // CSR write transaction
  csrctl |= (uint32_t ) reg << 16; // destination address

	buf = kmalloc(8, GFP_KERNEL);
	if (!buf) {
		pr_err("Can't allocate memory for USB buffer");
		return -ENOMEM;
	}
	memcpy(buf, &csrctl, 4);
	memcpy(buf+4, &data, 4);

	pipe = usb_sndbulkpipe(dev->udev, dev->csr_out_endpointAddr);
	ret = usb_bulk_msg(dev->udev, pipe, buf, 8, &tlen, 1000);

	if (ret != 0 || tlen != 8)
		pr_warn("USB out transfer failed: ret = %d, tlen = %d\n", ret, tlen);

	return ret;
}
//
// read memory from addr to the buffer provided by caller using DMA
// engine
static int dma_readbuf(struct usb_plx *dev, const uint32_t dmaaddr, unsigned char* buf, size_t count)
{
	int ret = -EINVAL;
	u32 dmacount = 0;
	u32 dmactl = dmactl_default;
	u32 dmastat = 0;
	int tlen = 0;
	u32 tmp = 0;
	u32 ep_avail = 0;
	int pipe;

	pr_debug("dev=%p, dmaaddr = 0x%.8x, buf=%p, count=%zu\n", dev, dmaaddr, buf, count);

	// Configure DMA registers
	plx_writereg(dev, REG_DMA1_CTL, dmactl); // start fresh
	plx_writereg(dev, REG_GPEP1_STAT, BIT(USB_STALL_SENT));
	plx_writereg(dev, REG_GPEP1_STAT, BIT(FIFO_FLUSH) | BIT(SHORT_PACKET_TRANSFERRED_INTERRUPT));

	plx_writereg(dev, REG_DMA1_ADDR, dmaaddr);

	//readreg(dev, REG_DMA1_COUNT, &dmacount);
	dmacount |= count;
	dmacount |= BIT(DMA_DIRECTION); // DMA Direction: IN (read)
	dmacount |= BIT(END_OF_CHAIN);
	dmacount |= BIT(VALID_BIT);
	dmacount |= BIT(DMA_DONE_INTERRUPT_ENABLE);
	plx_writereg(dev, REG_DMA1_COUNT, dmacount);

	dmactl = plx_readreg(dev, REG_DMA1_CTL);
	dmactl |= BIT(DMA_ENABLE); // DMA Enable
	dmactl |= BIT(DMA_FIFO_VALIDATE); // Validate last short packet
	plx_writereg(dev, REG_DMA1_CTL, dmactl);

	// Start DMA transfer
	dmastat = plx_readreg(dev, REG_DMA1_STAT);
	dmastat |= BIT(DMA_START);
	plx_writereg(dev, REG_DMA1_STAT, dmastat);

	while((dmacount & 0x7fffff) && ep_avail) {
		udelay(1000);
		tmp = plx_readreg(dev, REG_DMA1_COUNT);
		if(tmp != dmacount)
		{
			dmacount = tmp;
			pr_debug("dmacount=0x%.8x", dmacount);
		}

		ep_avail = plx_readreg(dev, REG_GPEP1_AVAIL);
		if(!ep_avail)
			pr_warn("USB IN FIFO full");
	}

	pipe = usb_rcvbulkpipe(dev->udev, dev->bulk_in_endpointAddr);
	ret = usb_bulk_msg(dev->udev, pipe, buf, count, &tlen, 1000);

	if (ret != 0 || tlen != count)
	{
		pr_warn("DMA in transfer failed (ret=%d, tlen=%d, dmaaddr=0x%x)", ret, tlen, dmaaddr);

		//readreg(dev, REG_GPEP1_RSP, &tmp);
		//if (tmp & BIT(CLEAR_ENDPOINT_TOGGLE))

		//readreg(dev, REG_GPEP1_STAT, &tmp);
		//if (tmp & BIT(USB_STALL_SENT))
			plx_writereg(dev, REG_GPEP1_STAT, BIT(USB_STALL_SENT));
			plx_writereg(dev, REG_GPEP1_STAT, BIT(FIFO_FLUSH) | BIT(SHORT_PACKET_TRANSFERRED_INTERRUPT));
			plx_writereg(dev, REG_GPEP1_RSP, BIT(CLEAR_ENDPOINT_TOGGLE));
	}

	return ret;
}
// write memory to addr from the buffer provided by caller using DMA engine
static int dma_writebuf(struct usb_plx *dev, const uint32_t dmaaddr, unsigned char* buf, size_t count)
{
	int ret = -EINVAL;
	u32 dmacount = 0;
	u32 dmactl = dmactl_default;
	u32 dmastat = 0;
	int tlen = 0;
	int pipe;

	pr_debug("dev=%p, dmaaddr = 0x%.8x, buf=%p, count=%zu\n", dev, dmaaddr, buf, count);

	// Configure DMA registers
	plx_writereg(dev, REG_DMA0_CTL, dmactl); // start fresh
	plx_writereg(dev, REG_GPEP0_STAT, BIT(USB_STALL_SENT));
	plx_writereg(dev, REG_GPEP0_STAT, BIT(FIFO_FLUSH) | BIT(SHORT_PACKET_TRANSFERRED_INTERRUPT));

	plx_writereg(dev, REG_DMA0_ADDR, dmaaddr);

	//readreg(dev, REG_DMA0_COUNT, &dmacount);
	dmacount |= count;
	//dmacount |= BIT(DMA_DIRECTION); // DMA Direction: OUT (write)
	dmacount |= BIT(END_OF_CHAIN);
	dmacount |= BIT(VALID_BIT);
	dmacount |= BIT(DMA_DONE_INTERRUPT_ENABLE);
	plx_writereg(dev, REG_DMA0_COUNT, dmacount);

	dmactl = plx_readreg(dev, REG_DMA1_CTL);
	dmactl |= BIT(DMA_ENABLE); // DMA Enable
	dmactl |= BIT(DMA_FIFO_VALIDATE); // Validate last short packet
	plx_writereg(dev, REG_DMA0_CTL, dmactl);

	// Start DMA transfer
	dmastat = plx_readreg(dev, REG_DMA0_STAT);
	dmastat |= BIT(DMA_START);
	plx_writereg(dev, REG_DMA0_STAT, dmastat);

#if 0
	while((dmacount & 0x7fffff) && ep_avail) {
		udelay(1000);
		tmp = plx_readreg(dev, REG_DMA0_COUNT);
		if(tmp != dmacount)
		{
			dmacount = tmp;
			pr_debug("dmacount=0x%.8x", dmacount);
		}

		ep_avail = plx_readreg(dev, REG_GPEP0_AVAIL);
		if(!ep_avail)
			pr_warn("USB IN FIFO full");
	}
#endif

	pipe = usb_sndbulkpipe(dev->udev, dev->bulk_out_endpointAddr);
	ret = usb_bulk_msg(dev->udev, pipe, buf, count, &tlen, 1000);

	if (ret != 0 || tlen != count)
	{
		pr_warn("DMA in transfer failed (ret=%d, tlen=%d, dmaaddr=0x%x)", ret, tlen, dmaaddr);

		//readreg(dev, REG_GPEP1_RSP, &tmp);
		//if (tmp & BIT(CLEAR_ENDPOINT_TOGGLE))

		//readreg(dev, REG_GPEP1_STAT, &tmp);
		//if (tmp & BIT(USB_STALL_SENT))
			plx_writereg(dev, REG_GPEP0_STAT, BIT(USB_STALL_SENT));
			plx_writereg(dev, REG_GPEP0_STAT, BIT(FIFO_FLUSH) | BIT(SHORT_PACKET_TRANSFERRED_INTERRUPT));
			plx_writereg(dev, REG_GPEP0_RSP, BIT(CLEAR_ENDPOINT_TOGGLE));
	}

	return ret;
}

static ssize_t pciemem_read(struct file *file, char *buffer, size_t count,
			 loff_t *ppos)
{
	struct usb_plx *dev;
	int rv;

	if (count > DMA_BUFLEN)
		return -EINVAL;

	dev = file->private_data;
	pr_debug("dev=%p, ppos=%p, *ppos=%llx\n", dev, ppos, *ppos);
	rv = dma_readbuf(dev, *ppos, dev->bulk_in_buffer, count);
	pr_debug("dma_readbuf: rv=%d\n", rv);
	if (rv < 0)
		return rv;
	//print_hex_dump(KERN_DEBUG, "buf: ", DUMP_PREFIX_OFFSET, 16, 1, dev->bulk_in_buffer, count, true);
	if (copy_to_user(buffer, dev->bulk_in_buffer, count))
		rv = -EFAULT;
	else
	{
		rv = count;
		*ppos += count;
	}
	return rv;
}

static ssize_t pciemem_write(struct file *file, const char *user_buffer,
			  size_t count, loff_t *ppos)
{
	struct usb_plx *dev;
	int rv = 0;

	/* verify that we actually have some data to write */
	if (count == 0 || count > DMA_BUFLEN)
		return -EINVAL;

	dev = file->private_data;
	pr_debug("dev=%p, ppos=%p, *ppos=%llx\n", dev, ppos, *ppos);
	if (copy_from_user(dev->bulk_out_buffer, user_buffer, count))
		rv = -EFAULT;
	print_hex_dump(KERN_DEBUG, "buf: ", DUMP_PREFIX_OFFSET, 16, 1, dev->bulk_out_buffer, count, true);
	rv = dma_writebuf(dev, *ppos, dev->bulk_out_buffer, count);
	pr_debug("dma_writebuf: rv=%d\n", rv);
	if (rv < 0)
		return rv;
	else
	{
		rv = count;
		*ppos += count;
	}
	return rv;
}

static int plx_probe(struct usb_interface *interface,
		      const struct usb_device_id *id)
{
	struct usb_plx *dev;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	size_t buffer_size;
	int i;
	int retval = -ENOMEM;

	/* allocate memory for our device state and initialize it */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&interface->dev, "Out of memory\n");
		goto error;
	}
	kref_init(&dev->kref);
	sema_init(&dev->limit_sem, WRITES_IN_FLIGHT);
	mutex_init(&dev->io_mutex);
	spin_lock_init(&dev->err_lock);
	init_usb_anchor(&dev->submitted);
	init_waitqueue_head(&dev->bulk_in_wait);

	dev->udev = usb_get_dev(interface_to_usbdev(interface));
	dev->interface = interface;

	/* set up the endpoint information */
	/* use only the first bulk-in and bulk-out endpoints */
	iface_desc = interface->cur_altsetting;
	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
		endpoint = &iface_desc->endpoint[i].desc;

		if (!dev->bulk_in_endpointAddr &&
		    usb_endpoint_is_bulk_in(endpoint) &&
				endpoint->bEndpointAddress & PLX_DMA_ENDPOINT) {
			/* we found the DMA bulk in endpoint */
			buffer_size = usb_endpoint_maxp(endpoint);
			dev->bulk_in_size = buffer_size;
			dev->bulk_in_endpointAddr = endpoint->bEndpointAddress;
			dev->bulk_in_buffer = kzalloc(DMA_BUFLEN, GFP_KERNEL);
			if (!dev->bulk_in_buffer) {
				dev_err(&interface->dev,
					"Could not allocate bulk_in_buffer\n");
				goto error;
			}
		}
		if (!dev->bulk_out_endpointAddr &&
		    usb_endpoint_is_bulk_out(endpoint) &&
				endpoint->bEndpointAddress & PLX_DMA_ENDPOINT) {
			/* we found the DMA bulk out endpoint */
			buffer_size = usb_endpoint_maxp(endpoint);
			dev->bulk_out_size = buffer_size;
			dev->bulk_out_endpointAddr = endpoint->bEndpointAddress;
			dev->bulk_out_buffer = kzalloc(DMA_BUFLEN, GFP_KERNEL);
			if (!dev->bulk_out_buffer) {
				dev_err(&interface->dev,
					"Could not allocate bulk_out_buffer\n");
				goto error;
			}
		}

		if (!dev->csr_in_endpointAddr &&
		    usb_endpoint_is_bulk_in(endpoint) &&
				endpoint->bEndpointAddress & PLX_CSR_ENDPOINT) {
			/* we found the CSR bulk in endpoint */
			dev->csr_in_endpointAddr = endpoint->bEndpointAddress;
		}
		if (!dev->csr_out_endpointAddr &&
		    usb_endpoint_is_bulk_out(endpoint) &&
				endpoint->bEndpointAddress & PLX_CSR_ENDPOINT) {
			/* we found the CSR bulk out endpoint */
			dev->csr_out_endpointAddr = endpoint->bEndpointAddress;
		}

	}
	if (!(dev->bulk_in_endpointAddr && dev->bulk_out_endpointAddr)) {
		dev_err(&interface->dev,
			"Could not find both bulk-in and bulk-out endpoints\n");
		goto error;
	}
	pr_debug("DMA endpoints: 0x%x/0x%x - 0x%x/0x%x\n",
		 dev->bulk_in_endpointAddr,
		 dev->bulk_out_endpointAddr,
		 dev->csr_in_endpointAddr,
		 dev->csr_out_endpointAddr);

	/* save our data pointer in this interface device */
	usb_set_intfdata(interface, dev);

	/* we can register the device now, as it is ready */
	retval = usb_register_dev(interface, &plx_class);
	if (retval) {
		/* something prevented us from registering this driver */
		dev_err(&interface->dev,
			"Not able to get a minor for this device.\n");
		usb_set_intfdata(interface, NULL);
		goto error;
	}

	/* let the user know what node this device is now attached to */
	dev_info(&interface->dev,
		 "PLX device now attached to pciemem-%d",
		 interface->minor);
	return 0;

error:
	if (dev)
		/* this frees allocated memory */
		kref_put(&dev->kref, plx_delete);
	return retval;
}

static void plx_disconnect(struct usb_interface *interface)
{
	struct usb_plx *dev;
	int minor = interface->minor;

	dev = usb_get_intfdata(interface);
	usb_set_intfdata(interface, NULL);

	/* give back our minor */
	usb_deregister_dev(interface, &plx_class);

	/* prevent more I/O from starting */
	mutex_lock(&dev->io_mutex);
	dev->interface = NULL;
	mutex_unlock(&dev->io_mutex);

	usb_kill_anchored_urbs(&dev->submitted);

	/* decrement our usage count */
	kref_put(&dev->kref, plx_delete);

	dev_info(&interface->dev, "pciemem #%d now disconnected", minor);
}

static void plx_draw_down(struct usb_plx *dev)
{
	int time;

	time = usb_wait_anchor_empty_timeout(&dev->submitted, 1000);
	if (!time)
		usb_kill_anchored_urbs(&dev->submitted);
}

static int plx_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct usb_plx *dev = usb_get_intfdata(intf);

	if (!dev)
		return 0;
	plx_draw_down(dev);
	return 0;
}

static int skel_resume(struct usb_interface *intf)
{
	return 0;
}

static int skel_pre_reset(struct usb_interface *intf)
{
	struct usb_plx *dev = usb_get_intfdata(intf);

	mutex_lock(&dev->io_mutex);
	plx_draw_down(dev);

	return 0;
}

static int plx_post_reset(struct usb_interface *intf)
{
	struct usb_plx *dev = usb_get_intfdata(intf);

	/* we are sure no URBs are active - no locking needed */
	dev->errors = -EPIPE;
	mutex_unlock(&dev->io_mutex);

	return 0;
}



module_usb_driver(plx_driver);

MODULE_LICENSE("GPL");

