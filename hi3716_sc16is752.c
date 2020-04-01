#include <linux/device.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/delay.h>
#include <linux/poll.h>
//#include <mach/hardware.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/sched.h>


//#include <pthread.h>



/**Define the chip type.*/
/**CNcomment: 定义芯片类型枚举 */
typedef enum hiCHIP_TYPE_E
{
    HI_CHIP_TYPE_HI3716M,  /**<HI3716M */
    HI_CHIP_TYPE_HI3716H,  /**<HI3716H */
    HI_CHIP_TYPE_HI3716C,  /**<HI3716C */
    HI_CHIP_TYPE_HI3716L,  /**<HI3716L */
    
    HI_CHIP_TYPE_HI3720,  /**<HI3720 */
    
    HI_CHIP_TYPE_BUTT
}HI_CHIP_TYPE_E;


#define CONFIG_HI_SYS_CLK_CFG_REG 0x101f5000

#define I2C0_PHY_ADDR     (0x101F7000)



#define CONFIG_PIN_MUTEX_BASE 0x10203000
#define CONFIG_PIN_MUTEX_I2C0 0x144


#define I2C_CTRL_REG 0x00
#define I2C_COM_REG 0x04
#define I2C_ICR_REG 0x08
#define I2C_SR_REG 0x0c
#define I2C_SCL_H_REG 0x10
#define I2C_SCL_L_REG 0x14
#define I2C_TXR_REG 0x18
#define I2C_RXR_REG 0x1c


#define I2C_DFT_RATE      (100000)
//sys clock
#define I2C_3716C_SYSCLK  (125000000)	//3716C
#define I2C_DFT_SYSCLK    (100000000)  // 3716H 3716M 3716L



#define I2C_WRITE_REG(Addr, Value) ((*(volatile unsigned int *)(Addr)) = (Value))
#define I2C_READ_REG(Addr)         (*(volatile unsigned int *)(Addr))



#define READ_OPERATION     (1)
#define WRITE_OPERATION    0xfe



/* I2C_CTRL_REG */
#define I2C_ENABLE             (1 << 8)
#define I2C_UNMASK_TOTAL       (1 << 7)
#define I2C_UNMASK_START       (1 << 6)
#define I2C_UNMASK_END         (1 << 5)
#define I2C_UNMASK_SEND        (1 << 4)
#define I2C_UNMASK_RECEIVE     (1 << 3)
#define I2C_UNMASK_ACK         (1 << 2)
#define I2C_UNMASK_ARBITRATE   (1<< 1)
#define I2C_UNMASK_OVER        (1 << 0)
#define I2C_UNMASK_ALL         (I2C_UNMASK_START | I2C_UNMASK_END | \
                                I2C_UNMASK_SEND | I2C_UNMASK_RECEIVE | \
                                I2C_UNMASK_ACK | I2C_UNMASK_ARBITRATE | \
                                I2C_UNMASK_OVER)
/*#define I2C_UNMASK_ALL         (I2C_UNMASK_ACK | I2C_UNMASK_ARBITRATE | \
									I2C_UNMASK_OVER)*/



/* I2C_COM_REB */
#define I2C_SEND_ACK (~(1 << 4))
#define I2C_START (1 << 3)
#define I2C_READ (1 << 2)
#define I2C_WRITE (1 << 1)
#define I2C_STOP (1 << 0)

/* I2C_ICR_REG */
#define I2C_CLEAR_START (1 << 6)
#define I2C_CLEAR_END (1 << 5)
#define I2C_CLEAR_SEND (1 << 4)
#define I2C_CLEAR_RECEIVE (1 << 3)
#define I2C_CLEAR_ACK (1 << 2)
#define I2C_CLEAR_ARBITRATE (1 << 1)
#define I2C_CLEAR_OVER (1 << 0)
#define I2C_CLEAR_ALL (I2C_CLEAR_START | I2C_CLEAR_END | \
                       I2C_CLEAR_SEND | I2C_CLEAR_RECEIVE | \
                       I2C_CLEAR_ACK | I2C_CLEAR_ARBITRATE | \
                       I2C_CLEAR_OVER)

/* I2C_SR_REG */
#define I2C_BUSY (1 << 7)
#define I2C_START_INTR (1 << 6)
#define I2C_END_INTR (1 << 5)
#define I2C_SEND_INTR (1 << 4)
#define I2C_RECEIVE_INTR (1 << 3)
#define I2C_ACK_INTR (1 << 2)
#define I2C_ARBITRATE_INTR (1 << 1)
#define I2C_OVER_INTR (1 << 0)



#define SET_STOP_BIT _IOW('k', 0, int)  
#define SET_BPS _IOW('k', 1, int)  
#define SET_NOE _IOW('k', 2, int)




static unsigned int g_I2cKernelAddr;


//-----------------752 reg addr
#define     RHR			0x00			//R
#define     THR			0x00			//W
 
 
#define     IER			0x01
 
 
#define     FCR			0x02			//R
#define     IIR			0x02			//W
 
 
#define     LCR			0x03
#define     MCR			0x04
#define     LSR			0x05
#define     MSR_1		0x06			//本来是直接定义为MSR的，但是这样定义与汇编指令MSR有冲突
#define     SPR			0x07
 
 
#define     TCR			0x06			//These registers are accessible only when EFR[4] = 1, and MCR[2] = 1
#define     TLR			0x07
 
 
#define     TXLVL		0x08
#define     RXLVL		0x09
#define     IODIR_752		0x0a
#define     IOSTATE		0x0b
#define     IOINTENA		0x0c
#define     RESERVED		0x0d
#define     IOCONTROL		0x0e
#define     EFCR		0x0f
 
 
 
 
//special register The Special Register set is accessible only when LCR[7] = 1 and not 0xBF
#define     DLL			0x00
#define     DLH			0x01
 
 
 
 
//enhanced register Enhanced Features Registers are only accessible when LCR = 0xBF
#define     EFR			0x02
#define     XON1		0x04
#define     XON2		0x05
#define     XOFF1		0x06
#define     XOFF2		0x07
 

//----------------------







typedef unsigned char       HI_U8;
typedef unsigned char       HI_UCHAR;
typedef unsigned short      HI_U16;
typedef unsigned int        HI_U32;
typedef unsigned long long  HI_U64;

typedef char                HI_S8;
typedef short               HI_S16;
typedef int                 HI_S32;
typedef long long           HI_S64;

typedef char                HI_CHAR;
typedef char*               HI_PCHAR;


#define I2C_WAIT_TIME_OUT   0x1000
#define HI_SUCCESS          (0)
#define HI_FAILURE          (-1)

#define HI_ERR_I2C_WRITE_TIMEOUT                    (HI_S32)(0x80440008)
#define HI_ERR_I2C_READ_TIMEOUT                     (HI_S32)(0x80440009)



//#define I2C_WRITE_REG(Addr, Value) ((*(volatile HI_U32 *)(Addr)) = (Value))
//#define I2C_READ_REG(Addr)         (*(volatile HI_U32 *)(Addr))


HI_S32 I2C_DRV_WaitWriteEnd_new(HI_U32 I2cNum)
{
    HI_U32  I2cSrReg;
    HI_U32  i = 0;

	do
	{
        I2cSrReg = I2C_READ_REG((g_I2cKernelAddr + I2C_SR_REG));

        if (i > I2C_WAIT_TIME_OUT)
        {
            printk("wait write data timeout!  I2cSrReg:0x%x\n",I2cSrReg);
            return HI_FAILURE;
        }
		//printk("wait write data timeout!  I2cSrReg:0x%x\n",I2cSrReg);
        i++;
	}while((I2cSrReg & I2C_OVER_INTR) != I2C_OVER_INTR);

    if (I2cSrReg & I2C_ACK_INTR)
    {
    	printk("I2C_DRV_WaitWriteEnd_new err!  I2cSrReg:0x%x\n",I2cSrReg);
        return HI_FAILURE;
    }
    I2C_WRITE_REG((g_I2cKernelAddr + I2C_ICR_REG), I2C_CLEAR_ALL);

	return HI_SUCCESS;
}


HI_S32 I2C_DRV_Write_new(HI_U32 I2cNum, HI_U8 I2cDevAddr, HI_U32 I2cRegAddr, HI_U32 I2cRegAddrByteNum, HI_U8 *pData, HI_U32 DataLen)
{

//	printk("I2C_DRV_Write_new,g_I2cKernelAddr map->0x%x \n",g_I2cKernelAddr);
    HI_U32          i;
//    unsigned long   IntFlag;
    HI_U32          RegAddr;

    //local_irq_save(IntFlag);


//	printk("I2C_DRV_Write_new I2C_CTRL_REG value:0x%x \n",I2C_READ_REG(g_I2cKernelAddr + I2C_CTRL_REG));

    /*  clear interrupt flag*/
    I2C_WRITE_REG((g_I2cKernelAddr + I2C_ICR_REG), I2C_CLEAR_ALL);

    /* send devide address */
//	printk("I2C_DRV_Write_new dev_addr:0x%x \n",I2cDevAddr);
    I2C_WRITE_REG((g_I2cKernelAddr + I2C_TXR_REG), (I2cDevAddr & WRITE_OPERATION));
    I2C_WRITE_REG((g_I2cKernelAddr + I2C_COM_REG),(I2C_WRITE | I2C_START));

    if (I2C_DRV_WaitWriteEnd_new(I2cNum))
    {
        //local_irq_restore(IntFlag);
        printk("wait write data timeout!%s, %d\n", __func__, __LINE__);
        return HI_ERR_I2C_WRITE_TIMEOUT;
    }

    /* send register address which will need to write */
	RegAddr = 0x00 | (I2cRegAddr << 3);
   // for(i=0; i<I2cRegAddrByteNum; i++)
   // {
     //   RegAddr = I2cRegAddr >> ((I2cRegAddrByteNum -i -1) * 8);
        I2C_WRITE_REG((g_I2cKernelAddr + I2C_TXR_REG), RegAddr);

        I2C_WRITE_REG((g_I2cKernelAddr + I2C_COM_REG), I2C_WRITE);

        if (I2C_DRV_WaitWriteEnd_new(I2cNum))
        {
            //local_irq_restore(IntFlag);
            printk("wait write data timeout!%s, %d\n", __func__, __LINE__);
            return HI_ERR_I2C_WRITE_TIMEOUT;
        }
   // }

	/* send data */
	for (i=0; i<DataLen; i++)
	{
        I2C_WRITE_REG((g_I2cKernelAddr + I2C_TXR_REG), (*(pData+i)));
    	I2C_WRITE_REG((g_I2cKernelAddr + I2C_COM_REG), I2C_WRITE);

        if (I2C_DRV_WaitWriteEnd_new(I2cNum))
        {
            //local_irq_restore(IntFlag);
            printk("wait write data timeout!%s, %d\n", __func__, __LINE__);
            return HI_ERR_I2C_WRITE_TIMEOUT;
        }
	}

	/*   send stop flag bit*/
	I2C_WRITE_REG((g_I2cKernelAddr + I2C_COM_REG), I2C_STOP);
    if (I2C_DRV_WaitWriteEnd_new(I2cNum))
    {
        //local_irq_restore(IntFlag);
        printk("wait write data timeout!%s, %d\n", __func__, __LINE__);
        return HI_ERR_I2C_WRITE_TIMEOUT;
    }
    //local_irq_restore(IntFlag);

    return HI_SUCCESS;
}



//static pthread_mutex_t g_I2cMutex = PTHREAD_MUTEX_INITIALIZER;



HI_S32 HI_I2C_Write_new(HI_U32 I2cNum, HI_U8 I2cDevAddr, HI_U32 I2cRegAddr, HI_U32 I2cRegAddrByteNum, HI_U8 *pData, HI_U32 DataLen)
{
    HI_S32  Ret;

   // Ret = down_interruptible(&g_I2cMutex);
   /* if (Ret)
    {
        printk("lock g_I2cMutex error.\n");
        return HI_FAILURE;
    }*/
    
    Ret = I2C_DRV_Write_new(I2cNum, I2cDevAddr, I2cRegAddr, I2cRegAddrByteNum, pData, DataLen);
 //   printk("Ret=0x%x, I2cNum=%d, DevAddr=0x%x, RegAddr=0x%x, Num=%d, Len=%d, data0=0x%x\n", Ret, I2cNum, I2cDevAddr, I2cRegAddr, I2cRegAddrByteNum, DataLen, pData[0]);

   // up(&g_I2cMutex);

    return Ret;
}


HI_S32 I2C_DRV_WaitRead_new(HI_U32 I2cNum)
{
    HI_U32  I2cSrReg;
    HI_U32  i = 0;

	do
	{
		I2cSrReg = I2C_READ_REG((g_I2cKernelAddr + I2C_SR_REG));

		if (i > I2C_WAIT_TIME_OUT)
        {
            printk("wait write data timeout!\n");
            return HI_FAILURE;
        }
        i++;
	}while((I2cSrReg & I2C_RECEIVE_INTR) != I2C_RECEIVE_INTR);

	return HI_SUCCESS;
}



int I2C_DRV_Read_new(HI_U32 I2cNum, HI_U8 I2cDevAddr, HI_U32 I2cRegAddr, HI_U32 I2cRegAddrByteNum, HI_U8 *pData, HI_U32 DataLen)
{

    HI_U32          dataTmp = 0xff;
    HI_U32          i;
//    unsigned long   IntFlag;
    HI_U32          RegAddr;

    //local_irq_save(IntFlag);

    /* clear interrupt flag*/
    I2C_WRITE_REG((g_I2cKernelAddr + I2C_ICR_REG), I2C_CLEAR_ALL);

    /* send devide address*/
    I2C_WRITE_REG((g_I2cKernelAddr + I2C_TXR_REG), (I2cDevAddr & WRITE_OPERATION));
    I2C_WRITE_REG((g_I2cKernelAddr + I2C_COM_REG),(I2C_WRITE | I2C_START));

    if (I2C_DRV_WaitWriteEnd_new(I2cNum))
    {
        //local_irq_restore(IntFlag);
        printk("wait write data timeout!\n");
        return HI_ERR_I2C_WRITE_TIMEOUT;
    }

    /* send register address which will need to write*/
	RegAddr = 0x00 | (I2cRegAddr << 3);
//    for(i=0; i<I2cRegAddrByteNum; i++)
  //  {
 //       RegAddr = I2cRegAddr >> ((I2cRegAddrByteNum -i -1) * 8);

        I2C_WRITE_REG((g_I2cKernelAddr + I2C_TXR_REG), RegAddr);

        I2C_WRITE_REG((g_I2cKernelAddr + I2C_COM_REG), I2C_WRITE);

        if (I2C_DRV_WaitWriteEnd_new(I2cNum))
        {
            //local_irq_restore(IntFlag);
            printk("wait write data timeout!\n");
            return HI_ERR_I2C_WRITE_TIMEOUT;
        }
 //   }

    /* send register address which will need to read */
    I2C_WRITE_REG((g_I2cKernelAddr + I2C_TXR_REG), (I2cDevAddr | READ_OPERATION));
    I2C_WRITE_REG((g_I2cKernelAddr + I2C_COM_REG), I2C_WRITE | I2C_START);

    if (I2C_DRV_WaitWriteEnd_new(I2cNum))
    {
        //local_irq_restore(IntFlag);
        return HI_ERR_I2C_WRITE_TIMEOUT;
    }

    /* repetitivily read data */
    for(i=0; i<DataLen; i++)
    {
        /*  the last byte don't need send ACK*/
        if (i == (DataLen - 1))
        {
            I2C_WRITE_REG((g_I2cKernelAddr + I2C_COM_REG), (I2C_READ | (~I2C_SEND_ACK)));
        }
        /*  if i2c master receive data will send ACK*/
        else
        {
            I2C_WRITE_REG((g_I2cKernelAddr + I2C_COM_REG), I2C_READ);
        }

        if (I2C_DRV_WaitRead_new(I2cNum))
        {
            //local_irq_restore(IntFlag);
            printk("wait read data timeout!\n");
            return HI_ERR_I2C_READ_TIMEOUT;
        }

        dataTmp = I2C_READ_REG((g_I2cKernelAddr + I2C_RXR_REG));
        *(pData+i)= dataTmp & 0xff;

        if (I2C_DRV_WaitWriteEnd_new(I2cNum)) 
        {
            //local_irq_restore(IntFlag);
            printk("wait write data timeout!\n");
            return HI_ERR_I2C_WRITE_TIMEOUT;
        }
    }

    /* send stop flag bit*/
    I2C_WRITE_REG((g_I2cKernelAddr + I2C_COM_REG), I2C_STOP);
    if (I2C_DRV_WaitWriteEnd_new(I2cNum))
    {
        //local_irq_restore(IntFlag);
        printk("wait write data timeout!\n");
        return HI_ERR_I2C_WRITE_TIMEOUT;
    }

    //local_irq_restore(IntFlag);

    return HI_SUCCESS;
}


HI_S32 HI_I2C_Read_new(HI_U32 I2cNum, HI_U8 I2cDevAddr, HI_U32 I2cRegAddr, HI_U32 I2cRegAddrByteNum, HI_U8 *pData, HI_U32 DataLen)
{
    HI_S32  Ret;

  //  Ret = down_interruptible(&g_I2cMutex);
 /*   if (Ret)
    {
        HI_ERR_I2C("lock g_I2cMutex error.\n");
        return HI_FAILURE;
    }*/
    
    Ret = I2C_DRV_Read_new(I2cNum, I2cDevAddr, I2cRegAddr, I2cRegAddrByteNum, pData, DataLen);
  //  printk("Ret=0x%x, I2cNum=%d, DevAddr=0x%x, RegAddr=0x%x, Num=%d, Len=%d\n", Ret, I2cNum, I2cDevAddr, I2cRegAddr, I2cRegAddrByteNum, DataLen);

 //   up(&g_I2cMutex);

    return Ret;
}

void hi_i2c_set_rate_new(unsigned int I2cRate)
{

#if 0
	unsigned int apb_clk, scl_h, scl_l, hold;

	/* get apb bus clk for diff plat */
	apb_clk = get_apb_clk();

	/* set SCLH and SCLL depend on apb_clk and def_rate */
	if (pinfo->pdata->clk_limit <= I2C_DFT_RATE) {
		scl_h = (apb_clk / I2C_DFT_RATE) / 2;
		scl_l = scl_h;
	} else {
		scl_h = (apb_clk * 36) / (pinfo->pdata->clk_limit * 100);
		scl_l = (apb_clk * 64) / (pinfo->pdata->clk_limit * 100);
	}

	writel(scl_h, pinfo->regbase + I2C_SCL_H_REG);
	writel(scl_l, pinfo->regbase + I2C_SCL_L_REG);

	/* set hi_i2c hold time */
	hold = scl_h / 2;
	writel(hold, pinfo->regbase + I2C_SDA_HOLD_REG);
	#else
	  unsigned int   Value = 0;
    unsigned int  SclH = 0;
    unsigned int  SclL = 0;
    unsigned int u32ChipVersion;
    unsigned int  SysClock;

    /* read i2c I2C_CTRL register*/
	Value = I2C_READ_REG((g_I2cKernelAddr + I2C_CTRL_REG));

    /* close all i2c  interrupt */
	I2C_WRITE_REG(g_I2cKernelAddr + I2C_CTRL_REG,(Value & (~I2C_UNMASK_TOTAL)));



  	SysClock = I2C_DFT_SYSCLK; 

    SclH = (SysClock / (I2cRate * 2)) / 2 - 1;
 //  SclH = 269;
    I2C_WRITE_REG(g_I2cKernelAddr + I2C_SCL_H_REG,SclH);

 printk("hi_i2c_set_rate->SclH value:0x%x \n",SclH);

    SclL = (SysClock / (I2cRate * 2)) / 2 - 1;
   // SclL = 269;
    I2C_WRITE_REG(g_I2cKernelAddr + I2C_SCL_L_REG,SclL);
	printk("hi_i2c_set_rate->SclL value:0x%x \n",SclL);
	
    /*enable i2c interrupt, resume original  interrupt*/
    I2C_WRITE_REG((g_I2cKernelAddr + I2C_CTRL_REG),Value);	

    return;
	#endif
}


void HI_I2C_Open_new(void)
{
#if 0
    HI_S32  Ret;
    HI_U32  i;

    if (1 == i2cState)
    {
        return ;
    }

    Ret = down_interruptible(&g_I2cMutex); 
    if (Ret)
    {
        HI_ERR_I2C("lock g_I2cMutex error.\n");
        return ;
    }
    
    if(1) //if (1 == atomic_inc_return(&g_I2cCount))
    {
        for (i=0; i<HI_I2C_MAX_NUM; i++)
        {
            if (i > HI_UNF_I2C_CHANNEL_QAM)
            {
                break;
            }
            /*disable all i2c interrupt*/
            I2C_WRITE_REG((g_I2cKernelAddr[i] + I2C_CTRL_REG), 0x0);

            /*  config scl clk rate*/
            I2C_DRV_SetRate(i, I2C_DFT_RATE);

            /*clear all i2c interrupt*/
            I2C_WRITE_REG((g_I2cKernelAddr[i] + I2C_ICR_REG), I2C_CLEAR_ALL);

            /*enable relative interrupt*/
            I2C_WRITE_REG((g_I2cKernelAddr[i] + I2C_CTRL_REG), (I2C_ENABLE | I2C_UNMASK_TOTAL | I2C_UNMASK_ALL));
        }
        i2cState = 1;
    }

    up(&g_I2cMutex);
	#else
		/*disable all i2c interrupt*/
	I2C_WRITE_REG(g_I2cKernelAddr + I2C_CTRL_REG,0x0);
	/*  config scl clk rate*/
	hi_i2c_set_rate_new(I2C_DFT_RATE);

	/*clear all i2c interrupt*/
	I2C_WRITE_REG(g_I2cKernelAddr + I2C_ICR_REG,I2C_CLEAR_ALL);
	printk("hi_i2c_hw_init,I2C_ICR_REG:%x \n",I2C_READ_REG(g_I2cKernelAddr + I2C_ICR_REG));

	printk("hi_i2c_hw_init,I2C_SR_REG:%x \n",I2C_READ_REG(g_I2cKernelAddr + I2C_SR_REG));
	

	/*enable relative interrupt*/
	I2C_WRITE_REG(g_I2cKernelAddr + I2C_CTRL_REG,(I2C_ENABLE | I2C_UNMASK_TOTAL | I2C_UNMASK_ALL));

	printk("hi_i2c_hw_init,I2C_CTRL_REG:%x \n",I2C_READ_REG(g_I2cKernelAddr + I2C_CTRL_REG));
	#endif
    return;
}

unsigned int gpio9_reg_base = 0;

int I2C_ModeInit_0_new(void)
{ 



//hi3716_gpio9_3,for reset,hi3716_gpio9_2,irq,pin mutex
unsigned int pin_mutex_base = ioremap(CONFIG_PIN_MUTEX_BASE,0x200);
printk("CONFIG_PIN_MUTEX_BASE map->0x%x \n",pin_mutex_base);
I2C_WRITE_REG(pin_mutex_base+0x158,0x0);


//i2c0 scl sda pin mux


I2C_WRITE_REG(pin_mutex_base+CONFIG_PIN_MUTEX_I2C0,0x02);

iounmap(pin_mutex_base);



//set reset pin hi3716_gpio9_3
gpio9_reg_base = ioremap(0x101ef000,0x500);

unsigned char value = I2C_READ_REG(gpio9_reg_base+0x400);
//set gpio_9_2 input,irq
I2C_WRITE_REG(gpio9_reg_base+0x400,value & (~0x04));
//edge trigger
I2C_WRITE_REG(gpio9_reg_base+0x404,0);
//falling trigger
I2C_WRITE_REG(gpio9_reg_base+0x40c,0);
I2C_WRITE_REG(gpio9_reg_base+0x408,0);
//clear irq
I2C_WRITE_REG(gpio9_reg_base+0x41c,0xff);
//enable irq
I2C_WRITE_REG(gpio9_reg_base+0x410,0x04);



iounmap(gpio9_reg_base);


//open i2c sys clock
	unsigned int u32RegVal = 0;

	unsigned int sys_clock_reg_base = ioremap(CONFIG_HI_SYS_CLK_CFG_REG,0x110);
	printk("CONFIG_HI_SYS_CLK_CFG_REG map->0x%x \n",sys_clock_reg_base);
	u32RegVal = I2C_READ_REG(sys_clock_reg_base+0xd8);
    u32RegVal |= (0x1 << 8);
    u32RegVal &= ~0x1;
    I2C_WRITE_REG(sys_clock_reg_base+0xd8,u32RegVal); //1/////10000000

//fix qam clock
	u32RegVal = (((((u32RegVal & ~(0x3)) | (0x7 << 8)) | (0x1 << 16)) & ~(0x3 << 17)) | (0x1 << 19));
	printk("CONFIG_HI_SYS_CLK_CFG_REG 0xa0 value->0x%x \n",u32RegVal);
    I2C_WRITE_REG(sys_clock_reg_base+0xa0, u32RegVal);
	
	iounmap(sys_clock_reg_base);
	



//i2c0 reg base
	g_I2cKernelAddr = ioremap(I2C0_PHY_ADDR,0x20);
	printk("g_I2cKernelAddr map->0x%x \n",g_I2cKernelAddr);

    return 0;
}

HI_S32 write_752_reg(unsigned int reg_addr,unsigned char reg_value)
{
	unsigned  char ss[2];
	memset(ss,0,2);
	ss[0] = reg_value;
	return HI_I2C_Write_new(0,0x9a,reg_addr,1,ss,1);
}

HI_U8 read_752_reg(unsigned int reg_addr)
{
	unsigned  char ss[2];
	memset(ss,0,2);
	HI_I2C_Read_new(0,0x9a,reg_addr,1,ss,1);
	return ss[0];

}








//misc dev drv
struct sc16is752_dev {
	struct mutex lock;
	unsigned char *buf;
	int irq;
	struct work_struct *sc752_irq_work;
	wait_queue_head_t sc16is752_q;
	unsigned int condition;
};

static struct sc16is752_dev *sc16is752 = NULL;



static void write_channel(unsigned char *buf,unsigned int len)
{

	unsigned char status;
	//cmd = 0x00 | (THR << 3);
	
	status = read_752_reg(LSR);
	
	if (status & 0x20)
    	{
		/*spi_message_init(&m);
		memset(t, 0, (sizeof t));
	
		t[0].tx_buf = &cmd;
		t[0].len = 1;
		spi_message_add_tail(&t[0], &m);
	
		t[1].tx_buf = buf;
		t[1].len = len;
		spi_message_add_tail(&t[1], &m);
		
		spin_lock_irq(&sc16is752->spi_lock);
		
		spi_sync(sc16is752->spi, &m);
	
		spin_unlock_irq(&sc16is752->spi_lock);*/
		//write_752_reg();
		HI_I2C_Write_new(0,0x9a,THR,1,buf,len);
	}
	return;
}


static ssize_t read_channel(int len)
{
 
	unsigned int length = 0;
	unsigned char status = 0;
	do{
		status = read_752_reg(LSR);
		if(status & 0x01)
		{	
			sc16is752->buf[length] = read_752_reg(RHR);
			length ++;
		}
	}while(status & 0x01);
	return length;
}



void uart_send_test()
{
	/*initlizate channel A*/
	write_752_reg(LCR, 0x80);
	write_752_reg(DLL, 0x14);
	write_752_reg(DLH, 0x00);
	write_752_reg(LCR, 0xbf);
	write_752_reg(EFR, 0x10);
	write_752_reg(LCR, 0x03);
	write_752_reg(IER, 0x01);
	write_752_reg(FCR, 0xf1);
	//write_752_reg(FCR, 0x51);
	write_752_reg(SPR, 0x41);
	write_752_reg(IODIR_752, 0xff);
	write_752_reg(IOSTATE, 0x00);
	//set uart para
 	//set stop bit 1
 	unsigned char tmp = read_752_reg(LCR);
	tmp &= (~(0x01<<2));//设置1位停止位
	write_752_reg(LCR,tmp);
	//set baud rate 9600
	tmp = read_752_reg(IER);
	tmp &= (~(0x01<<4));
	//禁止睡眠模式才可以设置波特率。1.8432mHZ
	write_752_reg(IER,tmp);
	write_752_reg(LCR, 0x80);
	write_752_reg(DLL, 0x0C);
	write_752_reg(DLH,0x00);
	write_752_reg(LCR,0xbf);
	write_752_reg(EFR,0x10);
	write_752_reg(LCR,0x03);
	write_752_reg(IER,0x01);
	write_752_reg(FCR,0xf1);
	write_752_reg(SPR,0x41);
	write_752_reg(IODIR_752,0xff);
	write_752_reg(IOSTATE,0x00);

	unsigned char tx_buff[5];
	memset(tx_buff,0x55,5);
	write_channel(tx_buff,5);
			
}


static int sc16is752_open(struct inode * inode, struct file * filp)
{
	
	filp->private_data = sc16is752;
	
	/*initlizate channel A*/
	write_752_reg(LCR, 0x80);
	write_752_reg(DLL, 0x14);
	write_752_reg(DLH, 0x00);
	write_752_reg(LCR, 0xbf);
	write_752_reg(EFR, 0x10);
	write_752_reg(LCR, 0x03);
	write_752_reg(IER, 0x01);
	write_752_reg(FCR, 0xf1);
	//write_752_reg(FCR, 0x51);
	write_752_reg(SPR, 0x41);
	write_752_reg(IODIR_752, 0xff);
	write_752_reg(IOSTATE, 0x00);
	//set uart para
 	//set stop bit 1
 	unsigned char tmp = read_752_reg(LCR);
	tmp &= (~(0x01<<2));//设置1位停止位
	write_752_reg(LCR,tmp);
	//set baud rate 9600
	tmp = read_752_reg(IER);
	tmp &= (~(0x01<<4));
	//禁止睡眠模式才可以设置波特率。1.8432mHZ
	write_752_reg(IER,tmp);
	write_752_reg(LCR, 0x80);
	write_752_reg(DLL, 0x0C);
	write_752_reg(DLH,0x00);
	write_752_reg(LCR,0xbf);
	write_752_reg(EFR,0x10);
	write_752_reg(LCR,0x03);
	write_752_reg(IER,0x01);
	write_752_reg(FCR,0xf1);
	write_752_reg(SPR,0x41);
	write_752_reg(IODIR_752,0xff);
	write_752_reg(IOSTATE,0x00);


	sc16is752->buf = kmalloc(64,GFP_KERNEL);
	if(!sc16is752->buf)
	{
		printk(KERN_INFO "kzallo buf error\n");
		return -ENOMEM;
	}
	
	nonseekable_open(inode, filp);//设置为不可随机读取。


	//-------------------------------------------------清空fifo数据
	unsigned int length = 0;
	unsigned char status = 0;
	do{
		status = read_752_reg(LSR);
		if(status & 0x01)
		{
			read_752_reg(RHR);
			length ++;
		}
	}while(status & 0x01);
	//----------------------------------------------------
 
 
	printk(KERN_INFO "open and initlizate channel A succeed\n ");	
	
    return 0;     
}


static int sc16is752_close(struct inode * inode, struct file * filp)
{	
	//kfree(sc16is752);
	printk("sc16is752_close \n");
	if (NULL != sc16is752->buf)
		kfree(sc16is752->buf);
	return 0;
}



static int sc16is752_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long args)
{
	unsigned char tmp = 0;
	switch (cmd) { 
		case SET_STOP_BIT:
		{
			tmp = read_752_reg(LCR);
 
 
			if(1 ==  args)
			{
				tmp &= (~(0x01<<2));//设置1位停止位
				write_752_reg(LCR, tmp);
			}
			else if (2 == args)
			{
				tmp |= (0x01<<2);//设置1.5/2位停止位
				write_752_reg(LCR, tmp);
			}
			break;
		}
		case SET_BPS:
		{
			tmp = read_752_reg(IER);
			tmp &= (~(0x01<<4));
			//禁止睡眠模式才可以设置波特率。
			write_752_reg(IER, tmp);
			
			if(9600 == args)
			{	
				write_752_reg(LCR, 0x80);
				write_752_reg(DLL, 0x0c);//--------------------------------------set baud rate
				write_752_reg(DLH, 0x00);
				write_752_reg(LCR, 0xbf);
				write_752_reg(EFR, 0x10);
				write_752_reg(LCR, 0x03);
				write_752_reg(IER, 0x01);
				write_752_reg(FCR, 0xf1);
				write_752_reg(SPR, 0x41);
				write_752_reg(IODIR_752, 0xff);
				write_752_reg(IOSTATE, 0x00);
			}
			else if(19200 == args)
			{
				write_752_reg(LCR, 0x80);
				write_752_reg(DLL, 0x06);//--------------------------------------set baud rate
				write_752_reg(DLH, 0x00);
				write_752_reg(LCR, 0xbf);
				write_752_reg(EFR, 0x10);
				write_752_reg(LCR, 0x03);
				write_752_reg(IER, 0x01);
				write_752_reg(FCR, 0xf1);
				write_752_reg(SPR, 0x41);
				write_752_reg(IODIR_752, 0xff);
				write_752_reg(IOSTATE, 0x00);
			}
			else if(38400 == args)
			{
				write_752_reg(LCR, 0x80);
				write_752_reg(DLL, 0x03);//--------------------------------------set baud rate
				write_752_reg(DLH, 0x00);
				write_752_reg(LCR, 0xbf);
				write_752_reg(EFR, 0x10);
				write_752_reg(LCR, 0x03);
				write_752_reg(IER, 0x01);
				write_752_reg(FCR, 0xf1);
				write_752_reg(SPR, 0x41);
				write_752_reg(IODIR_752, 0xff);
				write_752_reg(IOSTATE, 0x00);
			}
			break;
		}
 		case SET_NOE:
			{
				if(1 == args)//none
				{
					tmp = read_752_reg(LCR);
					tmp &= (~(0x01<<3));
					write_752_reg(LCR, tmp);
				}
				else if(2 == args)//odd
				{
					tmp = read_752_reg(LCR);
					tmp |= ((0x01<<3));
					tmp &= (~(0x01<<4));
					tmp &= (~(0x01<<5));
					write_752_reg(LCR, tmp);
				}
				else//even
				{
					tmp = read_752_reg(LCR);
					tmp |= ((0x01<<3));
					tmp |= ((0x01<<4));
					tmp &= (~(0x01<<5));
					write_752_reg(LCR, tmp);
				}
				break;
 			}
		default:
			break;
	
	}
	
	return 0;
}





static ssize_t sc16is752_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)  
{  
	printk("sc16is752_read \n");
	struct sc16is752_dev *sc16is752 = filp->private_data;
	int length = 0;
	int missing;
	//wait_event(sc16is752->sc16is752_q,sc16is752->condition);//用此函数会导致进程杀不死。
	wait_event_interruptible(sc16is752->sc16is752_q,sc16is752->condition);

	mutex_lock(&sc16is752->lock);
	length = read_channel(size);
	missing = copy_to_user(buf, sc16is752->buf, length);
	if (missing == length)
		length = -EFAULT;
	else
		length = length - missing;
	mutex_unlock(&sc16is752->lock);


	sc16is752->condition = 0;
	return length;
}  

static ssize_t sc16is752_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)  
{  

    int missing ;
	struct sc16is752_dev *sc16is752 = filp->private_data;
	if(size > 64)
		return -EMSGSIZE;
//	missing = copy_from_user(sc16is752->buf, buf, size);
//	write_channel(sc16is752->buf, size);
	mutex_lock(&sc16is752->lock);
	missing = copy_from_user(sc16is752->buf, buf, size);
	write_channel(sc16is752->buf, size);
	mutex_unlock(&sc16is752->lock);
	return 0;  
}  







/*
 *      The various file operations we support.
 */
static struct file_operations sc16is752_fops = {
	.owner	= THIS_MODULE,
	.ioctl	= sc16is752_ioctl,
	.open	= sc16is752_open,
	.write = sc16is752_write,  
    .read = sc16is752_read,
	.release	= sc16is752_close
};

static struct miscdevice sc16is752_misc_dev = {
	MISC_DYNAMIC_MINOR,
	"sc16is752",
	&sc16is752_fops,
};


void sc752_work_func(struct work_struct *work)
{
	unsigned char tmp = 0;
	tmp = read_752_reg(IIR);
//	printk("enter sc752_work_func\n");
	if(tmp & 0x04)
	{
		sc16is752->condition = 1;
		wake_up(&sc16is752->sc16is752_q);
	}
}
 
 
irqreturn_t sc16is752_handler(int irq, void *dev_id)
{
	//clear irq
	printk("enter sc752_work_func123\n");
	I2C_WRITE_REG(gpio9_reg_base+0x41c,0xff);
	schedule_work(sc16is752->sc752_irq_work);


#if 0
	//clear irq
	I2C_WRITE_REG(gpio9_reg_base+0x41c,0xff);
	unsigned char tmp = 0;
	tmp = read_752_reg(IIR);
	printk("enter sc752_work_func\n");
	if(tmp & 0x04)
	{
		//tmp = read_752_reg(IIR);
		
		sc16is752->condition = 1;
		wake_up(&sc16is752->sc16is752_q);
	}
	
	printk("enter sc16is752_handler\n");
#endif	
	return IRQ_HANDLED;
}



static int __init sc16is752_init(void)
{
    int ret = 0;
	 HI_CHIP_TYPE_E enChipType;
    HI_U32 u32ChipVersion = 0;
	//SYS_GetChipVersion( &enChipType, &u32ChipVersion );

	//i2c_test
	I2C_ModeInit_0_new();
	HI_I2C_Open_new();
	#if 0
	unsigned char ss[2];
	unsigned char ss_read[2];


	memset(ss_read,0,2);
	HI_I2C_Read_new(0,0x9a,0x03,1,ss_read,1);
	printk("HI_I2C_Read_new:%d \n",ss_read[0]);
	
	ss[0] = 0xff;
	ss[1] = 0x00;
	HI_I2C_Write_new(0,0x9a,0x03,1,ss,1);


	
	memset(ss_read,0,2);
	HI_I2C_Read_new(0,0x9a,0x03,1,ss_read,1);
	printk("HI_I2C_Read_new:%d \n",ss_read[0]);
	#endif

//	uart_send_test();
	ret = misc_register(&sc16is752_misc_dev);
	if(ret)
	{
		printk("could not register sc16is752 devices. \n");
		return ret;
	}


//init irq for rxh,gpio9 irq 18+32
//free irq in hi_gpio.ko
	free_irq(50,NULL);

	/*ret = request_threaded_irq(50, NULL, sc16is752_handler,
		IRQF_TRIGGER_FALLING, "sc16is752", NULL);	*/
	ret = request_irq(50, sc16is752_handler,
		IRQF_TRIGGER_FALLING, "sc16is752", NULL);	
	if (ret < 0 )
	{	
		printk("Failed to request IRQ! %d\n",ret);
	}

 	sc16is752 = kzalloc(sizeof(*sc16is752), GFP_KERNEL);
	mutex_init(&sc16is752->lock);
	sc16is752->sc752_irq_work = kzalloc(sizeof(struct work_struct),GFP_KERNEL);
	INIT_WORK(sc16is752->sc752_irq_work, sc752_work_func);
	init_waitqueue_head(&sc16is752->sc16is752_q);
	sc16is752->condition = 0;//初始化等待条件为0
	

	printk("sc16is752 driver init successful!\n");
	return 0;
}  

static void __exit sc16is752_exit(void)
{
	printk("sc16is752_exit \n");
    misc_deregister(&sc16is752_misc_dev);
//	free_irq(50,NULL);
	kfree(sc16is752);
	
    
}



module_init(sc16is752_init);
module_exit(sc16is752_exit);

MODULE_DESCRIPTION("Driver for  SC16IS752");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("JHST");


