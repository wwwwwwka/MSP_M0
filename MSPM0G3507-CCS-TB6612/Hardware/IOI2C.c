#include "IOI2C.h"

int IIC_Start(void)
{
	SDA_OUT();     
	IIC_SCL(1);
 	IIC_SDA(0); 
	IIC_SDA(1);
	delay_us(5);
	IIC_SDA(0);
	delay_us(5);
	IIC_SCL(0);
	return 1;
}

/**************************************************************************
Function: Analog IIC end signal
Input   : none
Output  : none
�������ܣ�ģ��IIC�����ź�
��ڲ�������?
����  ֵ����
**************************************************************************/	  
void IIC_Stop(void)
{
	SDA_OUT();//sda�����?
	IIC_SCL(0);
	IIC_SDA(0);//STOP:when CLK is high DATA change form low to high
 	delay_us(5);
	IIC_SCL(1);
    delay_us(5);	
	IIC_SDA(1);
	delay_us(5);							   	
}

int IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	SDA_IN();      
	IIC_SDA(1);
	delay_us(5);	   
	IIC_SCL(1);
	delay_us(1);	 
	while(SDA_GET())
	{
		ucErrTime++;
		if(ucErrTime>20)
		{
			IIC_Stop();
			return 0;
		}
	  delay_us(5);
	}
	IIC_SCL(0);   
	return 1;  
} 

void IIC_Ack(void)
{
	
	SDA_OUT();
	IIC_SCL(0);
	IIC_SDA(0);
	delay_us(5);
	IIC_SCL(1);
	delay_us(5);
	IIC_SCL(0);
	IIC_SDA(1);
}
	
    
void IIC_NAck(void)
{
	IIC_SCL(0);
	SDA_OUT();
	IIC_SDA(1);
	delay_us(5);
	IIC_SCL(1);
	delay_us(5);
	IIC_SCL(0);
	IIC_SDA(0);
}

void IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
	SDA_OUT(); 	    
    IIC_SCL(0);
    for(t=0;t<8;t++)
    {              
			IIC_SDA((txd&0x80)>>7);
			txd<<=1; 	  
			delay_us(5);   
			IIC_SCL(1);
			delay_us(5); 
			IIC_SCL(0);	
			delay_us(5);
    }	 
} 	 
  
int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
		int i;
    if (!IIC_Start())
        return 1;
    IIC_Send_Byte(addr << 1 );
    if (!IIC_Wait_Ack()) {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
		for (i = 0; i < len; i++) {
        IIC_Send_Byte(data[i]);
        if (!IIC_Wait_Ack()) {
            IIC_Stop();
            return 0;
        }
    }
    IIC_Stop();
    return 0;
}

int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    if (!IIC_Start())
        return 1;
    IIC_Send_Byte(addr << 1);
    if (!IIC_Wait_Ack()) {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte((addr << 1)+1);
    IIC_Wait_Ack();
    while (len) {
        if (len == 1)
            *buf = IIC_Read_Byte(0);
        else
            *buf = IIC_Read_Byte(1);
        buf++;
        len--;
    }
    IIC_Stop();
    return 0;
}


uint8_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();
    for(i=0;i<8;i++ )
	 {
			IIC_SCL(0); 
			delay_us(5);
			IIC_SCL(1);
			receive<<=1;
			if(SDA_GET())receive++;   
			delay_us(5); 
    }					 
    if (ack)
        IIC_Ack(); //����ACK 
    else
        IIC_NAck();//����nACK  
    return receive;
}

/**************************************************************************
Function: IIC reads a byte
Input   : I2C_Addr��Device IIC address��addr:Register address
Output  : res��Data read
�������ܣ���ȡָ���豸ָ���Ĵ�����һ��ֵ
��ڲ�����I2C_Addr���豸IIC��ַ��addr:�Ĵ�����ַ
����  ֵ��res����ȡ������
**************************************************************************/ 
unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
{
	unsigned char res=0;
	
	IIC_Start();	
	IIC_Send_Byte(I2C_Addr);	   //����д����
	res++;
	IIC_Wait_Ack();
	IIC_Send_Byte(addr); res++;  //���͵�ַ
	IIC_Wait_Ack();	  
	//IIC_Stop();//����һ��ֹͣ����	
	IIC_Start();
	IIC_Send_Byte(I2C_Addr+1); res++;          //�������ģ�?			   
	IIC_Wait_Ack();
	res=IIC_Read_Byte(0);	   
  IIC_Stop();//����һ��ֹͣ����

	return res;
}
 
/**************************************************************************
Function: IIC continuous reading data
Input   : dev��Target device IIC address��reg:Register address��
					length��Number of bytes��*data:The pointer where the read data will be stored
Output  : count��Number of bytes read out-1
�������ܣ�IIC����������
��ڲ�����dev��Ŀ���豸IIC��ַ��reg:�Ĵ�����ַ��length���ֽ�����
					*data:���������ݽ�Ҫ��ŵ�ָ��?
����  ֵ��count�����������ֽ�����-1
**************************************************************************/ 
uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data){
    uint8_t count = 0;
	
	IIC_Start();
	IIC_Send_Byte(dev);	   //����д����
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //���͵�ַ
  IIC_Wait_Ack();	  
	IIC_Start();
	IIC_Send_Byte(dev+1);  //�������ģ�?	
	IIC_Wait_Ack();
	
    for(count=0;count<length;count++){
		 
		 if(count!=length-1)   data[count]=IIC_Read_Byte(1);  //��ACK�Ķ�����
		 else                  data[count]=IIC_Read_Byte(0);  //���һ���ֽ�NACK
	}
    IIC_Stop();//����һ��ֹͣ����
    return count;
}
/**************************************************************************
Function: Writes multiple bytes to the specified register of the specified device
Input   : dev��Target device IIC address��reg��Register address��length��Number of bytes��
					*data��The pointer where the read data will be stored
Output  : 1
�������ܣ�������ֽ�д��ָ���豸ָ���Ĵ���?
��ڲ�����dev��Ŀ���豸��ַ��reg���Ĵ�����ַ��length��Ҫд���ֽ�����
					*data����Ҫд�����ݵ��׵�ַ
����  ֵ��1�������Ƿ�ɹ�?
**************************************************************************/ 
uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data){
  
 	uint8_t count = 0;
	IIC_Start();
	IIC_Send_Byte(dev);	   //����д����
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //���͵�ַ
  IIC_Wait_Ack();	  
	for(count=0;count<length;count++){
		IIC_Send_Byte(data[count]); 
		IIC_Wait_Ack(); 
	 }
	IIC_Stop();//����һ��ֹͣ����

    return 1; //status == 0;
}

/**************************************************************************
Function: Reads a byte of the specified register of the specified device
Input   : dev��Target device IIC address��reg��Register address��*data��The pointer where the read data will be stored
Output  : 1
�������ܣ���ȡָ���豸ָ���Ĵ�����һ��ֵ
��ڲ�����dev��Ŀ���豸��ַ��reg���Ĵ�����ַ��*data����Ҫд�����ݵ��׵�ַ
����  ֵ��1�������Ƿ�ɹ�?
**************************************************************************/ 
uint8_t IICreadByte(uint8_t dev, uint8_t reg, uint8_t *data){
	*data=I2C_ReadOneByte(dev, reg);
    return 1;
}

/**************************************************************************
Function: Write a byte to the specified register of the specified device
Input   : dev��Target device IIC address��reg��Register address��data��Data to be writtenwill be stored
Output  : 1
�������ܣ�д��ָ���豸ָ���Ĵ���һ���ֽ�
��ڲ�����dev��Ŀ���豸��ַ��reg���Ĵ�����ַ��data����Ҫд������
����  ֵ��1
**************************************************************************/ 
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data){
    return IICwriteBytes(dev, reg, 1, &data);
}

/**************************************************************************
Function: Read, modify, and write multiple bits in a byte of the specified device specified register
Input   : dev��Target device IIC address��reg��Register address��length��Number of bytes��
					bitStart��Start bit of target byte��data��Stores the value of the target byte bit to be changed
Output  : 1��success��0��fail
�������ܣ��� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �еĶ���?
��ڲ�����dev��Ŀ���豸��ַ��reg���Ĵ�����ַ��bitStart��Ŀ���ֽڵ���ʼλ��
					data����Ÿı�Ŀ���ֽ�λ���?
����  ֵ��1���ɹ���0��ʧ��
**************************************************************************/ 
uint8_t IICwriteBits(uint8_t dev,uint8_t reg,uint8_t bitStart,uint8_t length,uint8_t data)
{

    uint8_t b;
    if (IICreadByte(dev, reg, &b) != 0) {
        uint8_t mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        b &= mask;
        b |= data;
        return IICwriteByte(dev, reg, b);
    } else {
        return 0;
    }
}


/**************************************************************************
Function: Read, modify, and write one bit in a byte of the specified device specified register
Input   : dev��Target device IIC address��reg��Register address��
					bitNum��To modify the bitnum bit of the target byte��data��When it is 0, the target bit will be cleared, otherwise it will be set
Output  : 1��success��0��fail
�������ܣ��� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �е�1��λ
��ڲ�����dev��Ŀ���豸��ַ��reg���Ĵ�����ַ��bitNum��Ҫ�޸�Ŀ���ֽڵ�bitNumλ��
					data��Ϊ0ʱ��Ŀ��λ�����壬���򽫱���λ
����  ֵ��1���ɹ���0��ʧ��
**************************************************************************/ 
uint8_t IICwriteBit(uint8_t dev,uint8_t reg, uint8_t bitNum, uint8_t data){
   uint8_t b;
    IICreadByte(dev, reg, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return IICwriteByte(dev, reg, b);
}


