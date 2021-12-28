# ParkingGuide
Parking guide for garage
<br>
This project is designed to run on the STM32 F0 Discovery board.  <br>
Attached to the discovery board is
<ul>
  <li> HC-SR04 utrasonic transiever
  <li> an extra Yellow LED for user interface
  <li> two 1K resistors for voltage division.
</ul>
<br>
This following circuit attached to the Discovery Board:<br>
<ul>
  <li> 5V -- HC-SR04 power
  <li> GND -- HC-SR04 Gnd
  <li> PB13 -- Yellow LED -- GND
  <li> PA12 -- HC-SR04 Trigger
  <li> GND -- 1K -- PC7 -- 1K -- HC-SR04 Echo
</ul>

## Setup
<ol>
  <li> attach circuit
  <li> power on and program
  <li> code is designed to default to 12" default distance
</ol>

## Usage
<ol>
  <li> Power on
  <li> Approach the HC-SR04 sensor.
    <ol>
      <li> if further than the program distance away the Green LED will appear
      <li> when within 1" of the program distance both Grean and Blue LED will appear
      <li> when closer than the program distance the Blue LED will appear
    </ol>
  <li> Re-program distance
    <ol>
      <li> Adjust target disance from the HC-SR-04 sensor
      <li> press and hold the user button.
        <ol>
          <li> Yellow LED will FLASH to indicate user button is pressed (2 seconds)
          <li> Grenn and BLUE LED will FLASH to indicate distance measurement being taken
          <li> Grenn and BLUE LED become solid to indicate distance measurement stored in FLASH
        </ol>
      <li> new distance will now be stored in FLASH for future use.
    </ol>
</ol>
