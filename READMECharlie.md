# Code for Battery Pack using SPI Protocol

Most of this code comes from these documents but has been reconfigured for the teensy.

SD GUIDE:
https://www.ti.com/lit/an/sluaa11b/sluaa11b.pdf?ts=1658268039747&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FBQ76952

Example code (SPI):
https://github.com/CharlieMoll/STM_32.git

## What was added/reconfigured? 

For the most part, the TI example code was untouched. But since they used a different micro-controller,I had to pretty heavily change the SPI_WriteReg and SPI_Readreg functions. Here, I will detail these changes and explain why I did what I did

In place of the code that TI used to wrie to their MC, I added the following lines:

        digitalWrite(10, 0); // This is to set the CS pin low 
                             // (as per the spi mode that the BQ chip uses)
                             // This basically tells the chip to get ready for a transaction

        rxdata[0] = SPI.transfer(TX_Buffer[0]);
        rxdata[1] = SPI.transfer(TX_Buffer[1]);
                             // These two functions both transfer the data between the devices and 
                             // stores the received data in the universal rxdata array

        digitalWrite(10, 1); // This sets the CS pin high, ending the transaction


The rest of the code is explained in the above documentation. 

