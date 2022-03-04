
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/sensor.h>

void main(void)
{
	const struct device *dev;

	dev = device_get_binding("LORAGW");
	if (dev == NULL) {
		printk("Could not obtain LORAGW device\n");
		return;
	}


	// if(dev == NULL){
		// printk("error %s",dev->name);
	// }
	// else
	// {
		printk("device connected %s",dev->name);


}
