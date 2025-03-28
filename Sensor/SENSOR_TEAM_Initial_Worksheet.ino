void setup() {
  //"TESTBENCH":

  /*###########
    SETUP LCD #
  *///###########

  //SENSORS:

  /*#####################
    SETUP Accelerometer #
  *///#####################

  /*##############################
    SETUP pins for Photoresistor #
  *///##############################

  //NON-SENSORS:

  /*#########################################
    SETUP 03962a Ion battery charger module #
  *///#########################################

}

//"TESTBENCH":

/*##############
  FUNCTION LCD #
*///##############

  //convert to compatible text
  
  //write text.

  //append text

  //clear text

  //turn on plain, FIFO, FIFO with new line, panel back-and-forth, and panel back-and forth with new line, 

//SENSORS:

/*########################
  FUNCTION Accelerometer #
*///########################

  //function for read data

  //modify threshold

  //react to sufficient threshold of accelerometer

  //react to stop of accelerometer change - enable timer or count, so that it will tally whenever the bike is likely to be idle

/*#################################
  FUNCTION pins for Photoresistor #
*///#################################

  //read data

  //modify threshold.

  //react to a sufficient threshold of light

  //react to a sufficient threshold of darkness

//NON-SENSORS:

/*############################################
  FUNCTION 03962a Ion battery charger module #
*///############################################

  //read charge

  //modify charging strength

  //turn on charging

  //react to connection of battery

  //react to disconnection of battery

  //react to battery being overcharged

  //react to unidentifiable battery (?)

void loop() {
  //so far: this is where we test our program

}
