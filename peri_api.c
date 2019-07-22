#include "peri_api.h"

int fd_gpio;
int fd_pwm;
int fd_adc;
int fd_i2c;
const int MAX14514_ADDR = 0xED>>1;  //MAX14515 LENS DRIVER I2C ADDRESS
const int ADXL345_ADDR  = 0x53;      //ADXL345 I2C ADDRESS

/**
 * Open the input and output device.
 *
 * @param dev the device filename.
 * 
 * @returns a file descriptor.
 */
int open_gpio(const char *dev)
{
    int fd = 1;
    fd =  open(dev, O_RDWR);

    if(fd < 0)
    {
        return -1;
    }
    else

    {
        return fd;
    }
}

/**
 * Close the input and output device.
 *
 * @param fd the device file descriptor.
 *
 */
void close_gpio(int fd)
{
    if(fd == 0)
        return;
    close(fd);
    fd = 0;
}

/**
 * Set GPIO value.
 *
 * @param gpionumber the gpio number GPIO_xx
 *        val the gpio value that will be set. 
 *
 */
void setgpio(unsigned char  gpionumber,  unsigned char val)
{
	unsigned char gpiobuf[1];

	if(fd_gpio > 0)
	{
		if(val == 1)
			gpiobuf[0] = 0x80 | gpionumber;
		else if(val == 0)
			gpiobuf[0] = 0x00 | gpionumber;

		write(fd_gpio, gpiobuf, 1);

	}
}

/**
 * Get GPIO value.
 *
 * @param gpionumber the gpio number GPIO_xx
 * 
 * @return the gpio value 
 *
 */
unsigned char getgpio(unsigned char gpionumber)
{
	unsigned char gpiobuf[1];
	unsigned char retval;

	gpiobuf[0] = gpionumber;

	if(fd_gpio > 0)
	{
		retval = read(fd_gpio, gpiobuf, 1);

		if(gpiobuf[0] == 0)
			return 0;
		else if(gpiobuf[0] != 0)
			return 1;
	}
}

/**
 * Control gpio to output PWM.
 *
 * @param fd the device file descriptor
 *        cmd the command to decide which channel to enable or disable
 *        per the argurement to set frequency, and freq=1/sysclock*per
 *        phld the argurement to duty cycle, and duty_cycle=phld/per
 */
int ioctl_pwm(int fd, unsigned int cmd, int per,int ph1d)
{
    int data[2];
    data[0] = per;
    data[1] = ph1d;
    //ioctl(fd, cmd, per,ph1d);
    ioctl(fd, cmd, data);
}

/**
 * Open the pwm device.
 *
 * @param dev the device filename.
 * 
 * @returns a file descriptor.
 */
int open_pwm(const char *dev)
{
    int fd = 0;
    fd =  open(dev, O_RDWR);
    if(fd < 0)
    {
        return -1;
    }
    else
    {
        return fd;
    }
}

/**
 * Close the pwm device.
 *
 * @param fd the device file descriptor.
 *
 */
void close_pwm(int fd)
{
    if(fd == 0)
        return;
    close(fd);
    fd = 0;
}

int global_address;

static inline __s32 i2c_smbus_access(int file, char read_write, __u8 command,
                                     int size, union i2c_smbus_data *data)
{
        struct i2c_smbus_ioctl_data args;
 
        args.read_write = read_write;
        args.command = command;
        args.size = size;
        args.data = data;
        return ioctl(file,I2C_SMBUS,&args);
}
 
 
static inline __s32 i2c_smbus_write_quick(int file, __u8 value)
{
        return i2c_smbus_access(file,value,0,I2C_SMBUS_QUICK,NULL);
}
 
static inline __s32 i2c_smbus_read_byte(int file)
{
        union i2c_smbus_data data;
        if (i2c_smbus_access(file,I2C_SMBUS_READ,0,I2C_SMBUS_BYTE,&data))
                return -1;
        else
                return 0x0FF & data.byte;
}
 
static inline __s32 i2c_smbus_write_byte(int file, __u8 value)
{
        return i2c_smbus_access(file,I2C_SMBUS_WRITE,value,
                                I2C_SMBUS_BYTE,NULL);
}
 
static inline __s32 i2c_smbus_read_byte_data(int file, __u8 command)
{
        union i2c_smbus_data data;
        if (i2c_smbus_access(file,I2C_SMBUS_READ,command,
                             I2C_SMBUS_BYTE_DATA,&data))
                return -1;
        else
                return 0x0FF & data.byte;
}
 
static inline __s32 i2c_smbus_write_byte_data(int file, __u8 command,
                                              __u8 value)
{
        union i2c_smbus_data data;
        data.byte = value;
        return i2c_smbus_access(file,I2C_SMBUS_WRITE,command,
                                I2C_SMBUS_BYTE_DATA, &data);
}
 
static inline __s32 i2c_smbus_read_word_data(int file, __u8 command)
{
        union i2c_smbus_data data;
        if (i2c_smbus_access(file,I2C_SMBUS_READ,command,
                             I2C_SMBUS_WORD_DATA,&data))
                return -1;
        else
                return 0x0FFFF & data.word;
}
 
static inline __s32 i2c_smbus_write_word_data(int file, __u8 command,
                                              __u16 value)
{
        union i2c_smbus_data data;
        data.word = value;
        return i2c_smbus_access(file,I2C_SMBUS_WRITE,command,
                                I2C_SMBUS_WORD_DATA, &data);
}
 
static inline __s32 i2c_smbus_process_call(int file, __u8 command, __u16 value)
{
        union i2c_smbus_data data;
        data.word = value;
        if (i2c_smbus_access(file,I2C_SMBUS_WRITE,command,
                             I2C_SMBUS_PROC_CALL,&data))
                return -1;
        else
                return 0x0FFFF & data.word;
}
 
/* Returns the number of read bytes */
static inline __s32 i2c_smbus_read_block_data(int file, __u8 command,
                                              __u8 *values)
{
        union i2c_smbus_data data;
        int i;
        if (i2c_smbus_access(file,I2C_SMBUS_READ,command,
                             I2C_SMBUS_BLOCK_DATA,&data))
                return -1;
        else {
                for (i = 1; i <= data.block[0]; i++)
                        values[i-1] = data.block[i];
                return data.block[0];
        }
}
 
static inline __s32 i2c_smbus_write_block_data(int file, __u8 command,
                                               __u8 length, __u8 *values)
{
        union i2c_smbus_data data;
        int i;
        if (length > 32)
                length = 32;
        for (i = 1; i <= length; i++)
                data.block[i] = values[i-1];
        data.block[0] = length;
        return i2c_smbus_access(file,I2C_SMBUS_WRITE,command,
                                I2C_SMBUS_BLOCK_DATA, &data);
}
 
/* Returns the number of read bytes */
static inline __s32 i2c_smbus_read_i2c_block_data(int file, __u8 command,
                                                  __u8 *values)
{
        union i2c_smbus_data data;
        int i;
        if (i2c_smbus_access(file,I2C_SMBUS_READ,command,
                              I2C_SMBUS_I2C_BLOCK_DATA,&data))
                return -1;
        else {
                for (i = 1; i <= data.block[0]; i++)
                        values[i-1] = data.block[i];
                return data.block[0];
        }
}
 
static inline __s32 i2c_smbus_write_i2c_block_data(int file, __u8 command,
                                               __u8 length, __u8 *values)
{
        union i2c_smbus_data data;
        int i;
        if (length > 32)
                length = 32;
        for (i = 1; i <= length; i++)
                data.block[i] = values[i-1];
        data.block[0] = length;
        return i2c_smbus_access(file,I2C_SMBUS_WRITE,command,
                                I2C_SMBUS_I2C_BLOCK_DATA, &data);
}
 
/* Returns the number of read bytes */
static inline __s32 i2c_smbus_block_process_call(int file, __u8 command,
                                                 __u8 length, __u8 *values)
{
        union i2c_smbus_data data;
        int i;
        if (length > 32)
                length = 32;
        for (i = 1; i <= length; i++)
                data.block[i] = values[i-1];
        data.block[0] = length;
        if (i2c_smbus_access(file,I2C_SMBUS_WRITE,command,
                             I2C_SMBUS_BLOCK_PROC_CALL,&data))
                return -1;
        else {
                for (i = 1; i <= data.block[0]; i++)
                        values[i-1] = data.block[i];
                return data.block[0];
        }
}
 
#define CHECK_I2C_FUNC( var, label ) \
        do {    if(0 == (var & label)) { \
                fprintf(stderr, "\nError: " \
                        #label " function is required. Program halted.\n\n"); \
                exit(1); } \
        } while(0);

//int fd;


int set_slave_addr(int addr)
{	
	int ret;
	global_address = addr;

	if( ( ret = ioctl(fd_i2c, I2C_SLAVE, addr)) < 0)
	{
		fprintf(stderr, "Error2 : %s\n", strerror(errno));
		return ret;
	}

	return 0;
}

void read_write(int reg, int data)
{
	i2c_smbus_write_byte_data(fd_i2c, reg, data);
	//printf("addr[0x%02x:0x%02x]: set=0x%02x, get=0x%02x\n", (global_address<<1), reg, data, i2c_smbus_read_byte_data(fd_i2c, reg) );
}

void MAX14515_SLEEP_MODE()
{
	if (fd_i2c > 0) 
	{
		//set_slave_addr(MAX14514_ADDR);   //1.set i2c acess address
		read_write (0x00, 0x00);         //2.set reg0=0x00,sleep mode	
	}
}

void MAX14515_NORMAL_MODE()
{
	if (fd_i2c > 0) 
	{
		//set_slave_addr(MAX14514_ADDR);   //1.set i2c acess address
		read_write (0x00, 0x01);         //2.set reg0=0x01,normal mode
	}
}

void MAX14515_OUTPUT_LEVEL(unsigned char level)
{
	if (fd_i2c > 0) 
	{
		//set_slave_addr(MAX14514_ADDR);   //1.set i2c acess address
		read_write (0x01, level);         //2.set reg1=level	
	}
}

void init_adxl345()
{
    read_write(0x31,0x08);  //2b //测量范围,正负16g，13位模式
    read_write(0x2C,0x0A);  //0a //速率设定为12.5 参考pdf13页
    read_write(0x2D,0x08);  //28 //选择电源模式   参考pdf24页
    read_write(0x2E,0x00);   //不使用中断
    read_write(0x1E,0x00);   //X 偏移量 根据测试传感器的状态写入pdf29页
    read_write(0x1F,0x00);   //Y 偏移量 根据测试传感器的状态写入pdf29页
    read_write(0x20,0x00);   //Z 偏移量 根据测试传感器的状态写入pdf29页
}

/**
 * Inintialize the peripherals
 *
 * @throws IoError Thrown if devices do not exist or is not readable.
*/
int peri_init()
{
    fd_gpio = open_gpio("/dev/dm368_gpio");

    if(fd_gpio < 0)
    {
         printf("Can't open device /dev/dm368_gpio\r\n");
         return ERROR_OPEN;
	}
    fd_pwm = open_pwm("/dev/dm368_pwm");
    if(fd_pwm < 0)
    {
         printf("Can't open device /dev/dm368_pwm\r\n");
         return ERROR_OPEN;
    }
    fd_adc = open("/dev/dm365_adc", O_RDWR);        
	if (fd_adc < 0) 
	{
        printf("Can't open /dev/dm365_adc\r\n");
        return ERROR_OPEN;
    }
    fd_i2c = open("/dev/i2c-1", O_RDWR);
	if (fd_i2c < 0) 
	{
		printf("Can't open i2c-1\r\n", fd_i2c);
		return ERROR_OPEN;
    }
    return 0;
}


/**
 * close the peripherals
 */
void peri_release()
{
    close_gpio(fd_gpio);
    close_pwm(fd_pwm);
    close(fd_adc);
    close(fd_i2c);
}

/**
 * Determine if the button is triggered ,which can push the lens forward
 * 
 * @return value "1" means the button is pressed; otherwise the 
 *         value "0" means there is no action in button LENS_UP.
 */
bool peri_is_up_btn_triggered()    //GPIO 1
{
	bool state;
    unsigned char gpioval;
	gpioval=getgpio(GPIO_1);
    if (gpioval==0) {
		state=1; 
	}else
	{
		state=0;
	}
	return state;	
}

/**
 * Determine if the button is triggered ,which can pull the lens backward
 * 
 * @return value "1" means the button is pressed; otherwise the 
 *         value "0" means there is no action in button LENS_DOWN.
 */
bool peri_is_down_btn_triggered()   //GPIO 2
{
	bool state;
    unsigned char gpioval;
	gpioval=getgpio(GPIO_2);
    if (gpioval==0) {
		state=1; 
	}else
	{
		state=0;
	}
	return state;	
}

/**
 * Determine if the button to capture image is triggered 
 * 
 * @return value "1" means the button is pressed; otherwise the 
 *         value "0" means there is no action in button CAPTURE.
 */
bool peri_is_capture_btn_triggered()  //GPIO 3
{
	bool state;
    unsigned char gpioval;
	gpioval=getgpio(GPIO_3);
    if (gpioval==0) {
		state=1; 
	}else
	{
		state=0;
	}
	return state;	
}

/**
 * Determine if the power button is triggered
 * 
 * @return value "1" means the button is pressed; otherwise the 
 *         value "0" means there is no action in button POWER.
 */
bool peri_is_power_btn_triggerd() //GPIO 4
{
	bool state;
    unsigned char gpioval;
	gpioval=getgpio(GPIO_4);
    if (gpioval==1) {
		state=1; 
	}else
	{
		state=0;
	}
	return state;	
}

/**
 * Determine if the holder button is triggered
 * 
 * @return value "1" means the button is pressed, which will lead the 
 *         system to enter holder statement; otherwise the value "0" means 
 *         there is no action in button HOLDER.
 */
bool peri_is_in_holder()            //GPIO 7
{
	bool state;
    unsigned char gpioval;
	gpioval=getgpio(GPIO_7);
    if (gpioval==0) {
		state=1; 
	}else
	{
		state=0;
	}
	return state;	
}
/*bool peri_set_3led()
{
    bool state;
    unsigned char gpioval;
    gpioval=getgpio(GPIO_61,GPIO_62,GPIO_63);
    if (gpioval==0) {
		state=1; 
	}else
	{
		state=0;
	}
	return state;

    //pull down the GPIO_8
    
}
/**
 * Get the power supply voltage according to adc channel.
 * 
 * @return voltage value. 
 * 
 * @throws IoError Thrown if ADC channel is not readable.
 */
float peri_get_power_supply_voltage()
{
    unsigned short  adc_data[ADC_MAX_CHANNELS];
	unsigned int ret;
    float voltage;
    memset(adc_data, 0, sizeof(adc_data));
	ret = read(fd_adc, adc_data, sizeof(adc_data)); //read all adc channels
	if(ret < 0)
	{
		printf("ADC read error!\r\n");
        return ERROR_READ_ADC;
	}
	//printf("==adc value = %d  \r\n", adc_data[0]);  //get adc channel 0 value
	//usleep(1000);
    voltage=adc_data[0]*1.8/1023;//2^10 10_bit 
    return voltage;
}

/**
 * turn off the power
 */
void peri_power_off()
{
    //pull down the GPIO_8
    setgpio(GPIO_8, 0); 
}

void peri_set_3led()
{   
    //pull down the GPIO_8
    setgpio(GPIO_12, 0); 
    setgpio(GPIO_13, 1);
    setgpio(GPIO_14, 0);
    setgpio(GPIO_61, 0);
    setgpio(GPIO_62, 0);
    setgpio(GPIO_63, 0);

}
/**
 * Obtain 3D motion(adxl345) data.
 *
 * @param 'x', 'y','z' represent the acceleration of the x-axis, y-axis, and z-axis, respectively
 * 
 */
int peri_get_3D_motion(short*X, short*Y,short*Z)
{
    short mmm;
    unsigned char devid;
    set_slave_addr(ADXL345_ADDR);
    init_adxl345();
    devid = i2c_smbus_read_byte_data(fd_i2c, 0X00);
	//printf("ADXL345 DEVID: %02X \r\n", devid);
	unsigned char axisbuf[6];  //DX0,DX1,DY0,DY1,DZ0,DZ1
	unsigned char i;
	for(i=0; i<6; i++)
	{
		axisbuf[i] = i2c_smbus_read_byte_data(fd_i2c, 0x32 + i);
	}
      // printf("DX0=%02X, DX1=%02X, DY0=%02X, DY1=%02X, DZ0=%02X, DZ1=%02X \r\n", 
		//axisbuf[0], axisbuf[1], axisbuf[2], axisbuf[3] ,axisbuf[4], axisbuf[5]);
	usleep(300000);
    (*X)=(((short)axisbuf[1] & 0x03)*256+((short)axisbuf[0] & 0xFF));
    if(*X>511)
     {
      *X-=1024;
     }
    (*Y)=(((short)axisbuf[3] & 0x03)*256+((short)axisbuf[2] & 0xFF));
    if(*Y>511)
     {
      *Y-=1024;
     }
    (*Z)=(((short)axisbuf[5] & 0x03)*256+((short)axisbuf[4] & 0xFF));
    if(*Z>511)
     {
      *Z-=1024;
     }
    return 0;
}
// is moving?
int pre_x,pre_y,pre_z,x=0,y=0,z=0,a=0;
double p0;
int peri_3D_ismoving()
{   
   short X,Y,Z;
   bool state;
   a++;
   peri_get_3D_motion(&X,&Y,&Z);
   //printf("3D motion: X=%d Y=%d Z=%d \n", X, Y, Z);
   pre_x=x,pre_y=y,pre_z=z;
   //printf("pre_x=%d pre_y=%d pre_z=%d \n",pre_x, pre_y, pre_z);
   x=X;y=Y;z=Z;
   //printf("x=%d y=%d z=%d \n",x, y, z);
   p0=(x-pre_x)*(x-pre_x)+(y-pre_y)*(y-pre_y)+(z-pre_z)*(z-pre_z);
   printf("p0=%f \n",p0);
   if(a>1)
    {
       if(p0>500){ 
       state=1;
       }else{
       state=0;
       }
      printf("ismoving: %d\n",state); 
    }else{
      printf("ismoving: 0 \n");
    }
   return 0;
}


/**
 * Set the lens voltage level
 *
 * @param lens digital quantity corresponding to the output voltage
 */
int peri_set_lens_vlevel(unsigned char lens)
{
    unsigned char outlevel = 0;
    set_slave_addr(MAX14514_ADDR); 
	MAX14515_SLEEP_MODE();
    MAX14515_NORMAL_MODE();
    outlevel = lens;
   // printf("MAX14514 OUT LEVEL = %02X\r\n", outlevel);
    MAX14515_OUTPUT_LEVEL(outlevel);
    //printf("Set sucessfully\n");
    //usleep(200); //delay 2000s
}

/**
 * Enable the PWM output channel
 *
 * @param channel output pwm
 *        enabled has the options : PWM_ON, PWM_OFF: PWM_ON means turning on the channel 
 *                                  to output PWM , PWM_OFF means turning off the channel
 */
void peri_enable_pwm(int channel,bool enabled)
{   
    int GPIO_PWM;
    if (channel==PWM0_CHN)
    {
        GPIO_PWM=GPIO_88;
    }else if(channel==PWM1_CHN)
    {
        GPIO_PWM=GPIO_89;
    }
    
    if (enabled==1){
	    setgpio(GPIO_PWM, PWM_ON);  
        //usleep(2000*1000);
    }else
    {
       setgpio(GPIO_PWM, PWM_OFF);  
       //usleep(2000*1000);

    }
    return ;
}

/**
 * Set the PWM frequency and duty cycle 
 *
 * @param channel output pwm
 *        frequency the argurement to set frequency, and freq=1/sysclock*per
 *        phld the argurement to set duty cycle, and duty_cycle=phld/per
 *        note: when you turn the PWM_ON to PWM_OFF, PWM controlling will be 
 *              disabled and frequency,phld parameters have no effect. 
 */
void peri_set_pwm_freq_duty_cycle(int channel,int frequency,int phld)
{
     ioctl_pwm(fd_pwm, channel | PWM_ON, frequency, phld);
 
}
