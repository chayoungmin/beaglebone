/*
******************************************************************************
 *
 * \file   dd_gpio.c
 *
 * \brief  This file contains functions which performs the platform specific
 *         configurations of GPIO.
 *
******************************************************************************
*/

/*****************************************************************************
**					HEADER FILE INCLUDE
*****************************************************************************/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fcntl.h>

#include <asm/uaccess.h>
#include <asm/io.h>

#include <linux/time.h>
#include <linux/timer.h>

#include <linux/interrupt.h>
#include <linux/wait.h>

#include <linux/string.h>
#include <linux/ioport.h>
#include <linux/irq.h>
#include <linux/gpio.h>

#include "gpio_v2.h"
#include "hw_cm_per.h"
#include "hw_control_AM335x.h"
#include "hw_gpio_v2.h"
#include "soc_AM335x.h"
#include "hw_types.h"

/*****************************************************************************
**					INFORMATION
*****************************************************************************/
MODULE_LICENSE("GPL");
MODULE_AUTHOR("S.Moon");
MODULE_DESCRIPTION("BBB GPIO1 Module");

/*****************************************************************************
**					MACRO DEFINITIONS
*****************************************************************************/
/* Device Name & MajorNumber */
#define DEVICE_NAME "blockiodev"
#define MAJOR_NUMBER 240

/* GPIO1 PinNo */
#define GPIO_INSTANCE_OUT_PIN_NUMBER        (12)	//GPIO P8_12 pinNo $$$$$
#define GPIO_INSTANCE_INPUT_PIN_NUMBER      (28)	//GPIO P9_12 pinNo $$$$$

#define GPIO_IRQ_INSTANCE_INPUT_PIN_NUMBER	(60) 	//GPIO 1 pin inputt
/* DEFINE VARIABLE */
#define BLOCKIO_BUFF_MAX	64

/* TYPE DEFINE */
typedef struct
{
	unsigned long time;
} R_BLOCKIO_INFO;

/* Virtual Address Variable */
static void __iomem	*VIRT_SOC_CM_PER_REGS;		// GPIO1 Module Clk Virtual Address for Control module Register
static void __iomem	*VIRT_SOC_CONTROL_REGS;		// GPIO PinMux Virtual Address for Control module Register
static void __iomem	*VIRT_SOC_GPIO_1_REGS;

/* Variable define */

unsigned int gpio_irq_num;
unsigned int gpio_irq_cnt;
int	intcount = 0;

R_BLOCKIO_INFO intbuffer[BLOCKIO_BUFF_MAX];

/*****************************************************************************
**					LOW FUNCTION PROTOTYPES
*****************************************************************************/
int	gpio_pin_open(struct inode*, struct file*);
int	gpio_pin_release(struct inode*, struct file*);
int gpio_pin_read(struct file *filp,char *buf, size_t count, loff_t *f_pos);
int gpio_pin_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos);
int gpio_pin_init(void);
void gpio_pin_exit(void);

void GPIO1ModuleClkConfig(void);
void GPIO1PinMuxSetup(void);
void Gpio1PinModuleConfigOutput(unsigned int pinNo);
void Gpio1PinModuleConfigInput(unsigned int pinNo);


DECLARE_WAIT_QUEUE_HEAD(WaitQueue_Read);

/*****************************************************************************
**					LOW FUNCTION DEFINITIONS
*****************************************************************************/

/*********************************************************************************************************//**
* @param[in ] none
* @param[out] none
* @retval none
* @brief 
* @attention
*  This function must be registered to the kernel
*************************************************************************************************************/
void blockio_clear(void)
{
	int lp;

	for(lp = 0; lp <BLOCKIO_BUFF_MAX; lp++)
	{
		intbuffer[lp].time = 0;
	}

	intcount = 0;
}
/*********************************************************************************************************//**
* @param[in ] none
* @param[out] none
* @retval none
* @brief 
* @attention
*  This function must be registered to the kernel
*************************************************************************************************************/
irqreturn_t irq_gpio (int irq, void * ident)
{
	if(intcount < BLOCKIO_BUFF_MAX)
	{
		intbuffer[intcount].time = get_jiffies_64();
		intcount++;

	}

	wake_up_interruptible(&WaitQueue_Read);
/*
	if (!(VIRT_SOC_GPIO_1_REGS = ioremap(SOC_GPIO_1_REGS, 4096))) 
	{
		printk("[DEV] Unable to DMTIMER3 register\n");
	}

    GPIOPinIntClear((unsigned int)VIRT_SOC_GPIO_1_REGS, GPIO_INT_LINE_1, GPIO_INSTANCE_INPUT_PIN_NUMBER);
	gpio_irq_cnt++;

	if(gpio_irq_cnt%2)
	{
		GPIOPinWrite((unsigned int)VIRT_SOC_GPIO_1_REGS,GPIO_INSTANCE_OUT_PIN_NUMBER,GPIO_PIN_HIGH); //$$$$$
	}
	else
	{
		GPIOPinWrite((unsigned int)VIRT_SOC_GPIO_1_REGS,GPIO_INSTANCE_OUT_PIN_NUMBER,GPIO_PIN_LOW); //$$$$$
	}
*/
	printk("[DEV]Interrupt counter : %d\n", gpio_irq_cnt);

	return IRQ_HANDLED;
}
/*********************************************************************************************************//**
* @param[in ] none
* @param[out] none
* @retval none
* @brief 
* @attention
*  This function must be registered to the kernel
*************************************************************************************************************/
int gpio_pin_open(struct inode* inode_p, struct file* file_p)
{
	if (!(VIRT_SOC_GPIO_1_REGS = ioremap(SOC_GPIO_1_REGS, 4096))) 
	{
		printk("[DEV] Unable to DMTIMER3 register\n");
	}
	
	gpio_irq_num = gpio_to_irq(GPIO_IRQ_INSTANCE_INPUT_PIN_NUMBER);  //$$$$$
	
	// Set GPIO1 Interrupt
	if(request_irq(gpio_irq_num, irq_gpio, 0, "gpio_1", NULL) < 0)
	{
		printk("[DEV]request irq error\n");
	}
    GPIOIntTypeSet((unsigned int)VIRT_SOC_GPIO_1_REGS, GPIO_INSTANCE_INPUT_PIN_NUMBER, GPIO_INT_TYPE_BOTH_EDGE);

	GPIOPinWrite((unsigned int)VIRT_SOC_GPIO_1_REGS,GPIO_INSTANCE_OUT_PIN_NUMBER,GPIO_PIN_HIGH); //LED ON  

	blockio_clear();

	iounmap(VIRT_SOC_GPIO_1_REGS);
	
	printk("[DEV]request irq enable\n");

	return 0;
}
/*********************************************************************************************************//**
* @param[in ] none
* @param[out] none
* @retval none
* @brief 
* @attention
*  This function must be registered to the kernel
*************************************************************************************************************/
int gpio_pin_read(struct file *filp,char *buf, size_t count, loff_t *f_pos)
{
	int readcount;
	char *ptrdata;
	int loop;

	printk("1 intcount:%d filp->f_flags:%d O_NONBLOCK:%x\n",intcount,filp->f_flags,O_NONBLOCK);

	if(!intcount)
	{
		if(!(filp->f_flags & O_NONBLOCK)) // filp->f_flags:2,O_NONBLOCK:0x800
		{
			interruptible_sleep_on(&WaitQueue_Read);  //First
			printk("2 intcount:%d filp->f_flags:%x O_NONBLOCK:%d\n",intcount,filp->f_flags,O_NONBLOCK);
		}
		else
		{
			printk("3 intcount:%d filp->f_flags:%x O_NONBLOCK:%d\n",intcount,filp->f_flags,O_NONBLOCK);
			return -EAGAIN;
		}
	}

	readcount = count/sizeof(R_BLOCKIO_INFO);

	if(readcount > intcount)
	{
		readcount =  intcount;
	}

	ptrdata = (char*) &intbuffer[0];

	for(loop=0; loop<readcount * sizeof(R_BLOCKIO_INFO); loop++)
	{
		put_user(ptrdata[loop], (char *)& buf[loop]);
	}

	return readcount * sizeof(R_BLOCKIO_INFO);
	
	
}
/*********************************************************************************************************//**
* @param[in ] none
* @param[out] none
* @retval none
* @brief 
* @attention
*  This function must be registered to the kernel
*************************************************************************************************************/
int gpio_pin_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{
	unsigned char status; //write(dev,buff,1);
	int	loop;

	if (!(VIRT_SOC_GPIO_1_REGS = ioremap(SOC_GPIO_1_REGS, 4096))) 
	{
		printk("[DEV] Unable to DMTIMER3 register\n");
	}

	blockio_clear();

	for(loop = 0; loop < count; loop++)
	{
		get_user(status, (char *)buf);
		printk("status:%d\n",status);
		//GPIOPinWrite((unsigned int)VIRT_SOC_GPIO_1_REGS,GPIO_INSTANCE_OUT_PIN_NUMBER,(unsigned int)status); //$$$$$
		GPIOPinWrite((unsigned int)VIRT_SOC_GPIO_1_REGS,GPIO_INSTANCE_OUT_PIN_NUMBER, (unsigned int)status); 
		//outb(status, BLOCKIO_WRITE_ADDR);	
	}	

	iounmap(VIRT_SOC_GPIO_1_REGS);

	return count;

}
/*********************************************************************************************************//**
* @param[in ] none
* @param[out] none
* @retval none
* @brief 
* @attention
*  This function must be registered to the kernel
*************************************************************************************************************/
int gpio_pin_release(struct inode* inode_p, struct file* file_p)
{
	if (!(VIRT_SOC_GPIO_1_REGS = ioremap(SOC_GPIO_1_REGS, 4096))) 
	{
		printk("[DEV] Unable to DMTIMER3 register\n");
	}
	GPIOPinWrite((unsigned int)VIRT_SOC_GPIO_1_REGS,GPIO_INSTANCE_OUT_PIN_NUMBER,GPIO_PIN_LOW); 
	iounmap(VIRT_SOC_GPIO_1_REGS);
	free_irq(gpio_irq_num, NULL);
	
	printk("[DEV] led_release()\n");
	return 0;
}
/*********************************************************************************************************//**
* @param[in ] none
* @param[out] none
* @retval none
* @brief 
* @attention
*  This function must be registered to the kernel
*************************************************************************************************************/
struct file_operations gpio_pin_file_oper =
{
	.owner			=	THIS_MODULE,
	.read  			=   gpio_pin_read,
	.write			=   gpio_pin_write,
	.open			=	gpio_pin_open,
	.release		=	gpio_pin_release,
};
/*********************************************************************************************************//**
* @param[in ] none
* @param[out] none
* @retval none
* @brief 
* @attention
*  This function must be registered to the kernel
*************************************************************************************************************/
void GPIO1ModuleClkConfig(void)
{
	if (!(VIRT_SOC_CM_PER_REGS = ioremap(SOC_CM_PER_REGS, 4096))) 
	{
		printk("[DEV] Unable to Clk register\n");
	}

    /* Writing to MODULEMODE field of CM_PER_GPIO1_CLKCTRL register. */
    HWREG(VIRT_SOC_CM_PER_REGS + CM_PER_GPIO1_CLKCTRL) |=
          CM_PER_GPIO1_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_PER_GPIO1_CLKCTRL_MODULEMODE_ENABLE !=
          (HWREG(VIRT_SOC_CM_PER_REGS + CM_PER_GPIO1_CLKCTRL) &
           CM_PER_GPIO1_CLKCTRL_MODULEMODE));
    /*
    ** Writing to OPTFCLKEN_GPIO_1_GDBCLK bit in CM_PER_GPIO1_CLKCTRL
    ** register.
    */
    HWREG(VIRT_SOC_CM_PER_REGS + CM_PER_GPIO1_CLKCTRL) |=
          CM_PER_GPIO1_CLKCTRL_OPTFCLKEN_GPIO_1_GDBCLK;

    /*
    ** Waiting for OPTFCLKEN_GPIO_1_GDBCLK bit to reflect the desired
    ** value.
    */
    while(CM_PER_GPIO1_CLKCTRL_OPTFCLKEN_GPIO_1_GDBCLK !=
          (HWREG(VIRT_SOC_CM_PER_REGS + CM_PER_GPIO1_CLKCTRL) &
           CM_PER_GPIO1_CLKCTRL_OPTFCLKEN_GPIO_1_GDBCLK));

    /*
    ** Waiting for IDLEST field in CM_PER_GPIO1_CLKCTRL register to attain the
    ** desired value.
    */
    while((CM_PER_GPIO1_CLKCTRL_IDLEST_FUNC <<
           CM_PER_GPIO1_CLKCTRL_IDLEST_SHIFT) !=
           (HWREG(VIRT_SOC_CM_PER_REGS + CM_PER_GPIO1_CLKCTRL) &
            CM_PER_GPIO1_CLKCTRL_IDLEST));

    /*
    ** Waiting for CLKACTIVITY_GPIO_1_GDBCLK bit in CM_PER_L4LS_CLKSTCTRL
    ** register to attain desired value.
    */
    while(CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_GPIO_1_GDBCLK !=
          (HWREG(VIRT_SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL) &
           CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_GPIO_1_GDBCLK));

	iounmap(VIRT_SOC_CM_PER_REGS);
}
/*********************************************************************************************************//**
* @param[in ] none
* @param[out] none
* @retval none
* @brief 
* @attention
*  This function must be registered to the kernel
*************************************************************************************************************/
void GPIO1PinMuxSetup()
{
	if (!(VIRT_SOC_CONTROL_REGS = ioremap(SOC_CONTROL_REGS, 8192))) 
	{
		printk("[DEV] Unable to Control_module register\n");
	}

	// P9_12 = GPIO1_28 = GPMC_BE1N
	//HWREG((unsigned int)VIRT_SOC_CONTROL_REGS + CONTROL_CONF_GPMC_AD(12)) =  //$$$$$
	HWREG((unsigned int)VIRT_SOC_CONTROL_REGS + CONTROL_CONF_GPMC_BE1N) = //$$$$$
	CONTROL_CONF_GPMC_BE1N_CONF_GPMC_BE1N_SLEWCTRL | /*Slew rate slow */
	CONTROL_CONF_GPMC_BE1N_CONF_GPMC_BE1N_RXACTIVE | /*Receiver enabled */
	(CONTROL_CONF_GPMC_BE1N_CONF_GPMC_BE1N_PUDEN & (~CONTROL_CONF_GPMC_BE1N_CONF_GPMC_BE1N_PUDEN)) | /*PU_PD enabled */
	(CONTROL_CONF_GPMC_BE1N_CONF_GPMC_BE1N_PUTYPESEL & (~CONTROL_CONF_GPMC_BE1N_CONF_GPMC_BE1N_PUTYPESEL)) | /*PU_TY select */
	(CONTROL_CONF_MUXMODE(7));

	// P8_12 = GPIO01_12 = GPMC_AD12
	//HWREG((unsigned int)VIRT_SOC_CONTROL_REGS + CONTROL_CONF_GPMC_AD(GPIO_INSTANCE_OUT_PIN_NUMBER)) =  //$$$$$
	//(CONTROL_CONF_GPMC_AD_CONF_GPMC_AD_SLEWCTRL | 	/* Slew rate slow */
    // CONTROL_CONF_GPMC_AD_CONF_GPMC_AD_RXACTIVE |	/* Receiver enabled */
	//(CONTROL_CONF_GPMC_AD_CONF_GPMC_AD_PUDEN & (~CONTROL_CONF_GPMC_AD_CONF_GPMC_AD_PUDEN)) | /* PU_PD enabled */
	//(CONTROL_CONF_GPMC_AD_CONF_GPMC_AD_PUTYPESEL & (~CONTROL_CONF_GPMC_AD_CONF_GPMC_AD_PUTYPESEL)) | /* PD */
	//(CONTROL_CONF_MUXMODE(7))	/* Select mode 7 */
	//);
	

	iounmap(VIRT_SOC_CONTROL_REGS);
}
/*********************************************************************************************************//**
* @param[in ] none
* @param[out] none
* @retval none
* @brief 
* @attention
*  This function must be registered to the kernel
*************************************************************************************************************/
void Gpio1PinModuleConfigOutput(unsigned int pinNo)	//lwx
{
	/* Enabling the GPIO module. */
    GPIOModuleEnable((unsigned int)VIRT_SOC_GPIO_1_REGS);
    /* Resetting the GPIO module. */
    GPIOModuleReset((unsigned int)VIRT_SOC_GPIO_1_REGS);
    /* Setting the GPIO pin as an output pin. */
    GPIODirModeSet((unsigned int)VIRT_SOC_GPIO_1_REGS, pinNo, GPIO_DIR_OUTPUT);
}
/*********************************************************************************************************//**
* @param[in ] none
* @param[out] none
* @retval none
* @brief 
* @attention
*  This function must be registered to the kernel
*************************************************************************************************************/
void Gpio1PinModuleConfigInput(unsigned int pinNo)	//lwx
{
	/* Enabling the GPIO module. */
    GPIOModuleEnable((unsigned int)VIRT_SOC_GPIO_1_REGS);
    /* Resetting the GPIO module. */
    GPIOModuleReset((unsigned int)VIRT_SOC_GPIO_1_REGS);
    /* Setting the GPIO pin as an output pin. */
    GPIODirModeSet((unsigned int)VIRT_SOC_GPIO_1_REGS, pinNo, GPIO_DIR_INPUT);
	/* Enable Debouncing feature for the Intput GPIO Pin. */
    GPIODebounceFuncControl((unsigned int)VIRT_SOC_GPIO_1_REGS, pinNo, GPIO_DEBOUNCE_FUNC_ENABLE);
	/* Configure the Debouncing Time for all the input pins of the seleceted GPIO instance. */
    GPIODebounceTimeConfig((unsigned int)VIRT_SOC_GPIO_1_REGS, 1000);
}
/*********************************************************************************************************//**
* @param[in ] none
* @param[out] none
* @retval none
* @brief 
* @attention
*  This function must be registered to the kernel
*************************************************************************************************************/
int gpio_pin_init(void)
{	
	if (!(VIRT_SOC_GPIO_1_REGS = ioremap(SOC_GPIO_1_REGS, 4096))) 
	{
		printk("[DEV] Unable to DMTIMER3 register\n");
	}

	/* Enabling functional clocks for GPIO1 instance. */
    GPIO1ModuleClkConfig();
    /* Selecting GPIO1_12 for use. */
    GPIO1PinMuxSetup();
    /* Setting GPIO1 pin_x for use. */
	Gpio1PinModuleConfigInput(GPIO_INSTANCE_INPUT_PIN_NUMBER); //$$$$$
	Gpio1PinModuleConfigOutput(GPIO_INSTANCE_OUT_PIN_NUMBER); //$$$$$
	
	GPIOPinWrite((unsigned int)VIRT_SOC_GPIO_1_REGS,GPIO_INSTANCE_OUT_PIN_NUMBER,GPIO_PIN_HIGH); //$$$$$
/*
	//gpio_irq_num = gpio_to_irq(44); //$$$$$
	gpio_irq_num = gpio_to_irq(GPIO_IRQ_INSTANCE_INPUT_PIN_NUMBER);  //$$$$$
	
	// Set GPIO1 Interrupt
	if(request_irq(gpio_irq_num, irq_gpio, 0, "gpio_1", NULL) < 0)
		printk("[DEV]request irq error\n");

    GPIOIntTypeSet((unsigned int)VIRT_SOC_GPIO_1_REGS, GPIO_INSTANCE_INPUT_PIN_NUMBER, GPIO_INT_TYPE_BOTH_EDGE);
	//GPIOPinIntEnable((unsigned int)VIRT_SOC_GPIO_1_REGS, GPIO_INT_LINE_1, GPIO_INSTANCE_PIN_NUMBER);
*/
	if((register_chrdev(MAJOR_NUMBER, DEVICE_NAME, &gpio_pin_file_oper)) < 0)
	{
		printk("[DEV] LED initialization is fail\n");
		return -1;
	}

	iounmap(VIRT_SOC_GPIO_1_REGS);

	printk("[DEV] LED initialization success\n");
	return 0;
}
/*********************************************************************************************************//**
* @param[in ] none
* @param[out] none
* @retval none
* @brief 
* @attention
*  This function must be registered to the kernel
*************************************************************************************************************/
void gpio_pin_exit(void)
{
	if (!(VIRT_SOC_GPIO_1_REGS = ioremap(SOC_GPIO_1_REGS, 4096))) 
	{
		printk("[DEV] Unable to DMTIMER3 register\n");
	}
	GPIOPinWrite((unsigned int)VIRT_SOC_GPIO_1_REGS,GPIO_INSTANCE_OUT_PIN_NUMBER,GPIO_PIN_LOW); //$$$$$

	free_irq(gpio_irq_num, NULL);
	iounmap(VIRT_SOC_GPIO_1_REGS);

	unregister_chrdev(MAJOR_NUMBER, DEVICE_NAME);

	printk("[DEV] LED module is closed\n");
	return;
}

module_init(gpio_pin_init);
module_exit(gpio_pin_exit);

/****************************** End of file *********************************/
