# ADIS-16477 C-driver using stm32 controller/(HAL libraries)

## Setup
* NUCLEO board STM32H743ZIT6
* ADIS16477 Breakout board

## Configuration
Change the SPI handle, Chip_Select pin and Hard_Reset pin in the header file of the sensor driver.

<div align=center>
  <h2>Sensor</h2>
  <img src="/Pictures/board.jpg" width="400" height="350"/>
</div>

<div align=center>
  <h2>Hardware setup</h2>
  <img src="/Pictures/Hardware_setup.jpg" width="450" height="600"/>
</div>

<div align=center>
  <h2>Breakout_board_pins</h2>
  <img src="/Pictures/Breakout_board_pins.png" width="450" height="400"/>
</div>

<div align=center>
  <h2>Reading data sample with Osciloscope</h2>
  <img src="/Pictures/Sample read command.png" width="800" height="400"/>
  <img src="/Pictures/Osci_SPI_sample.jpg" width="800" height="400"/>
</div>

<div align=center>
  <h2>Live Raw Data</h2>
  <img src="/Pictures/Raw_data_live.png" width="600" height="200"/>
</div>
