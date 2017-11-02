#include "SHT15.h"
#include <math.h>


/*************************************************************
  Function   : SHT15_Config  
  Description: Initialize the SHT15 pin
  Input      : none        
  return     : none    
*************************************************************/
void SHT15_Init(void)
{
	  //Structure for GPIOG -> SDA, SCK
    GPIO_InitTypeDef GPIO_InitStruct;
    //DATA Push-pull output        
    GPIO_InitStruct.Pin = SHT15_DATA_PIN;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(SHT15_DATA_PORT, &GPIO_InitStruct);
    //SCK Push-pull output
    GPIO_InitStruct.Pin = SHT15_SCK_PIN;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;    
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(SHT15_SCK_PORT, &GPIO_InitStruct);

}


/*************************************************************
  Function   : SHT15_DATAOut
  Description: Set the DATA pin to output
  Input      : none        
  return     : none    
*************************************************************/
void SHT15_DATAOut(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    //DATA output       
    GPIO_InitStruct.Pin = SHT15_DATA_PIN;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; 
    HAL_GPIO_Init(SHT15_DATA_PORT, &GPIO_InitStruct);
}


/*************************************************************
  Function   : SHT15_DATAIn  
  Description: Set the DATA pin to input
  Input      : none        
  return     : none    
*************************************************************/
void SHT15_DATAIn(void)
{
   GPIO_InitTypeDef GPIO_InitStruct;
    //DATA input        
    GPIO_InitStruct.Pin = SHT15_DATA_PIN;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT; 
    HAL_GPIO_Init(SHT15_DATA_PORT, &GPIO_InitStruct);
}


/*************************************************************
  Function   : SHT15_WriteByte  
  Description: Write 1 byte
  Input      : The bytes to be written 
  return     : 0 - correct 1 - error  
*************************************************************/
uint8_t SHT15_WriteByte(uint8_t value)
{
    uint8_t i, err = 0;
    
    SHT15_DATAOut();                //Set the DATA data line to output

    for(i = 0x80; i > 0; i /= 2)    //Write 1 byte
    {
        if(i & value)
					
                SHT15_DATA_H;
        else
                SHT15_DATA_L;
        HAL_Delay(1);
        SHT15_SCK_H;
        HAL_Delay(1);
        SHT15_SCK_L;
        HAL_Delay(1);
    }
    SHT15_DATAIn();                 //Set the DATA line to input and release the DATA line
    SHT15_SCK_H;
    err = SHT15_DATA_R();           //Read the response bit of SHT15
    SHT15_SCK_L;

    return err;
}

/*************************************************************
  Function   : SHT15_ReadByte  
  Description: Read 1 byte data
  Input		   : Ack: 0 - No answer 1 - Answer
  Return: err: 0 - correct 1 - error   
*************************************************************/
uint8_t SHT15_ReadByte(uint8_t Ack)
{
    uint8_t i, val = 0;

    SHT15_DATAIn();                //Set the DATA data line to input
    for(i = 0x80; i > 0; i /= 2)   //Read 1 byte of data
    {
        HAL_Delay(1);
        SHT15_SCK_H;
        HAL_Delay(1);
        if(SHT15_DATA_R())
        {
           val = (val | i);
        }
        SHT15_SCK_L;
    }
    SHT15_DATAOut();               //Set the DATA data line to output
    if(Ack)
            SHT15_DATA_L;        //Response, it will read the next data (check data)
    else
            SHT15_DATA_H;        //Do not answer, the data ends here
    HAL_Delay(1);
    SHT15_SCK_H;
    HAL_Delay(1);
    SHT15_SCK_L;
    HAL_Delay(1);

    return val;                    //Returns the value read
}


/*************************************************************
  Function   :SHT15_TransStart  
  Description: Starts transmission of signals at the following timing:
                     _____         ________
               DATA:      |_______|
                         ___     ___
               SCK : ___|   |___|   |______        
  Input      : none        
  return     : none    
*************************************************************/
void SHT15_TransStart(void)
{
    SHT15_DATAOut();               //Set the DATA data line to output

    SHT15_DATA_H;
    SHT15_SCK_L;
    HAL_Delay(1);
    SHT15_SCK_H;
    HAL_Delay(1);
    SHT15_DATA_L;
    HAL_Delay(1);
    SHT15_SCK_L;
    HAL_Delay(1);
    SHT15_SCK_H;
    HAL_Delay(1);
    SHT15_DATA_H;
    HAL_Delay(1);
    SHT15_SCK_L;
			
}




/*************************************************************
  Function   : SHT15_Measure  
  Description: Reads temperature and humidity from the temperature and humidity sensor
  Input      : P value - the value read; ip_checksum - the number of checks to read      
  return     : Return: err: 0 - correct 1 - error
*************************************************************/
uint8_t SHT15_Measure(uint16_t *p_value, uint8_t mode)
{
    uint8_t err = 0;
    uint8_t value_H = 0;
    uint8_t value_L = 0;

    SHT15_TransStart();                                     //Start transmission
    switch(mode)                                                         
    {
    case TEMP:                                              //measure temperature
        err += SHT15_WriteByte(MEASURE_TEMP);           //Write MEASURE_TEMP to measure the temperature command
        break;
    case HUMI:
        err += SHT15_WriteByte(MEASURE_HUMI);           //Write MEASURE_HUMI measure the humidity command
        break;
    default:
        break;
    }
    if(err != 0)
    {
        return err;
    }
    SHT15_DATAIn();
		/*for(i = 0; i < 120000000; i++)                           //Wait for the DATA signal to be pulled low
    {
				if(SHT15_DATA_R() == 0) break;                  //DATA is pulled down, jump out of the loop
    }*/
		HAL_Delay(320);
    if(SHT15_DATA_R() == 1)                                //If waiting for a timeout
    {
        err += 1;
        return err;
    }
    value_H = SHT15_ReadByte(ACK);
    value_L = SHT15_ReadByte(ACK);
    *p_value = (value_H << 8) | value_L;
    return err;
}


/*************************************************************
  Function   : SHT15_Calculate  
  Description: Calculate the value of temperature and humidity
  Input      : Temp - the temperature value read from the sensor;
							 Humi- The humidity value read from the sensor
               P_humidity - the actual humidity value calculated;
							 p_temperature- the calculated actual temperature value
  return     : none    
*************************************************************/
void SHT15_Calculate(uint16_t t, uint16_t rh, float *p_temperature, float *p_humidity)
{
    //The parameter data is from the manual
    const float d1 = -39.55;
    const float d2 = +0.01;
    const float C1 = -4;
    const float C2 = +0.0405;
    const float C3 = -0.0000028;        
    const float T1 = +0.01;
    const float T2 = +0.00008;

    float RH_Lin;                                                     //RH Linearity        
    float RH_Ture;                                                    //RH True value
    float temp_C;

    temp_C = d1 + d2 * t;                                            //Calculate the temperature value        
    RH_Lin = C1 + C2 * rh + C3 * rh * rh;                            //Calculate the humidity value
    RH_Ture = (temp_C -25) * (T1 + T2 * rh) + RH_Lin;                //Humidity of the temperature compensation, calculate the actual humidity value
    RH_Ture = (RH_Ture > 100) ? 100 : RH_Ture;
    RH_Ture = (RH_Ture < 0.1) ? 0.1 : RH_Ture;                       //Set the lower limit of humidity

    *p_humidity = RH_Ture;
    *p_temperature = temp_C;

}


