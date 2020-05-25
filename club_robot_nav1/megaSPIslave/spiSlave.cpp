#include "spiSlave.h"

#include <arduino.h>
#include <avr/io.h>     // per Dale Wheat / Arduino Internals page 35.  Explicitly included to reference Arduion registers, even though Arduino automatically picks it up when not included
#include <SPI.h>

// Note: 
//  how to handle ISR references from within the spiSlave class?
//  generally followed this example https://atadiat.com/en/e-arduino-trick-share-interrupt-service-routines-between-libraries-application/
//  -> except used an explicit default constructor https://www.geeksforgeeks.org/constructors-c/

// create an spiSlave instance
// Although the Arduino Mega can support 2 SPI masters,
//    Only one Arduino Mega SPI peripheral can serve as a slave
//    hence, no code external to spiSlave.h / spiSlave.cpp should create any spiSlave instances
spiSlave spiSlavePort;

// Constructor
//  use a default constructor, invoked when an instance is declared (?)
//  Set digital output pins, set initial/default values to transfer to master
spiSlave::spiSlave()
{
  // Digital Test Points
  pinMode (digTP28, OUTPUT);
  digitalWrite(digTP28, LOW);
  pinMode (digTP29, OUTPUT);
  digitalWrite(digTP29, LOW);
  
  // initialize the SPI watchdog time counter
  SPIwdPriorMillis = millis();  

  // queue some initial values to send to the Pi
  spiSlave::setDataForPi('A', 1, 2, 3, 4, 5, 6);

  // initialize the receive buffer
  int i; 
  for (i = 0; i <= 15; i++) 
  {
    receiveBuffer[0][i] = 0;
    receiveBuffer[1][i] = 0;
  }   

}


//  Purpose
//      Initialize the SPI port as a slave and enable SPI interrupts
void spiSlave::enable()
{

  // Setup SPI in slave mode (1) define MISO pin as output (2) set
  // enable bit of the SPI configuration register 
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);

  // turn on SPI interrupts
  SPI.attachInterrupt();
}


// Purpose
//  Fill the SPI transfer buffer with payload data to be sent to the SPI master on the next SPI byte exchange
// Algorithm
//  always prepares the sendBuffer for the next transfer
//  -> but queues the values to an alternate sendBuffer in case request to change the send buffer came during an ongoing SPI transfer
// Output
//  updates the sendBuffer with values to be shared at the next transfer(s)
//  returns 0 if values queued during without an SPI transfer in progress
//  returns 1 if values queued during an in-progress SPI transfer
unsigned char spiSlave::setDataForPi (char command, signed char TurnVelocity, signed char ForwardThrottle, signed char SidewaysThrottle, long param1, long param2, long param3 )
{
  unsigned char nextSendBufferSelect;

  // write to digTP29, to facilitate timing measurements via oscilloscope
  digitalWrite(digTP29, HIGH);

  // synchronize processes - avoid corruption from writing/reading at the same time
  noInterrupts();
  if ( SPIxferInProgress == 1)
  {
    toggleSendBuffer = 1;
    if (sendBufferSelect == 1)
    {
      nextSendBufferSelect = 0;
    }
    else
    {
      nextSendBufferSelect = 1;
    }
  }
  else
  {
    nextSendBufferSelect = sendBufferSelect;
  }
  
  toSPIBufferByte1.asChar = command;
  toSPIBufferByte2.asSignedChar = TurnVelocity;
  toSPIBufferByte3.asSignedChar = ForwardThrottle;
  toSPIBufferByte4.asSignedChar = SidewaysThrottle;
  toSPIBufferLong1.asLong = param1;
  toSPIBufferLong2.asLong = param2;
  toSPIBufferLong3.asLong = param3;

  sendBuffer[nextSendBufferSelect][0] = toSPIBufferByte1.asUnsignedChar;
  sendBuffer[nextSendBufferSelect][1] = toSPIBufferByte2.asUnsignedChar;
  sendBuffer[nextSendBufferSelect][2] = toSPIBufferByte3.asUnsignedChar;
  sendBuffer[nextSendBufferSelect][3] = toSPIBufferByte4.asUnsignedChar;
  int i; 
  // queue bytes 5 thru 8 (param1)     
  for (i = 4; i <= 7; i++) 
  {
    sendBuffer[nextSendBufferSelect][i] = toSPIBufferLong1.asByte[i-4];
  }   

  // queue bytes 9 thru 12 (param2)     
  for (i = 8; i <= 11; i++) 
  {
    sendBuffer[nextSendBufferSelect][i] = toSPIBufferLong2.asByte[i-8];
  }   

  // queue bytes 13 thru 16 (param3)     
  for (i = 12; i <= 15; i++) 
  {
    sendBuffer[nextSendBufferSelect][i] = toSPIBufferLong3.asByte[i-12];
  }               
  interrupts();
  digitalWrite(digTP29, LOW);

  if (toggleSendBuffer == 0)
  {
    return 0;   // indicate transfer request accepted, no collision detected
  }
  else
  {
    return 1;   // indicate transfer request accepted, but collision detected so will toggle sendBuffer
  }
}


// Purpose
//  Allow parent code to initialize the received from SPI values cache
//  Intended to allow human operators or error-detecting routines to quickly top all autonomous command values, 
//  e.g. in case the SPI link goes down while active commands are being sent
void spiSlave::initializeValuesCacheFromPi()
{
  // initialize the receive buffer
  int i; 
  for (i = 0; i <= 15; i++) 
  {
    receiveBuffer[0][i] = 0;
    receiveBuffer[1][i] = 0;
  }  

  commandFromPi = 0;
  TurnVelocityFromPi = 0;
  ForwardThrottleFromPi = 0;
  SidewaysThrottleFromPi = 0;
  param1FromPi = 0;
  param2FromPi = 0;
  param3FromPi = 0;

}

// Purpose
//  retrieve the latest data received from the SPI master on the most recent SPI byte exchange
// Output
//  returns 1 if data retreived was fresh, 0 if data has already been retrieved with a call to getLatestDataFromPi()
unsigned char spiSlave::getLatestDataFromPi ()
{
  // synchronize processes - avoid corruption from writing/reading at the same time
  noInterrupts();

  fromSPIBufferByte1.asUnsignedChar = receiveBuffer[lastCompletedReceiveBufferSelect][0];
  fromSPIBufferByte2.asUnsignedChar = receiveBuffer[lastCompletedReceiveBufferSelect][1];
  fromSPIBufferByte3.asUnsignedChar = receiveBuffer[lastCompletedReceiveBufferSelect][2];
  fromSPIBufferByte4.asUnsignedChar = receiveBuffer[lastCompletedReceiveBufferSelect][3];

  int i; 
  // fetch bytes 5 thru 8 (param1)     
  for (i = 4; i <= 7; i++) 
  {
    fromSPIBufferLong1.asByte[i-4] = receiveBuffer[lastCompletedReceiveBufferSelect][i];
  }   

  // fetch bytes 9 thru 12 (param2)     
  for (i = 8; i <= 11; i++) 
  {
    fromSPIBufferLong2.asByte[i-8] = receiveBuffer[lastCompletedReceiveBufferSelect][i];
  }   

  // fetch bytes 13 thru 16 (param3)     
  for (i = 12; i <= 15; i++) 
  {
    fromSPIBufferLong3.asByte[i-12] = receiveBuffer[lastCompletedReceiveBufferSelect][i];
  }      

  commandFromPi = fromSPIBufferByte1.asChar;
  TurnVelocityFromPi = fromSPIBufferByte2.asSignedChar;
  ForwardThrottleFromPi = fromSPIBufferByte3.asSignedChar;
  SidewaysThrottleFromPi = fromSPIBufferByte4.asSignedChar;
  param1FromPi = fromSPIBufferLong1.asLong;
  param2FromPi = fromSPIBufferLong2.asLong;
  param3FromPi = fromSPIBufferLong3.asLong;

  if (newSPIdataAvailable == 1)
  {
    newSPIdataAvailable = 0;   // indicate that this new SPI data has just been consumed
    interrupts();
    return 1;   // indicate that the data just returned has NOT been previously seen
  }
  else
  {
    newSPIdataAvailable = 0;   // indicate that this new SPI data has just been consumed
    interrupts();
    return 0; // indicate that the data just returned HAS been previously seen
  }
}


// public method to retrieve the current count of transfer errors counted by the Mega SPI Slave
unsigned int spiSlave::getErrorCountSPIrx()
{
  return errorCountSPIrx;
};

// Clears an internal register that tracks the maximum oberved SPI burst duration
void spiSlave::clearErrorCountSPIrx()
{
  errorCountSPIrx = 0; 
};


// public methods to retrieve the latest values received from the Pi SPI master
char spiSlave::getCommandFromPi()
{
  return commandFromPi;
};

signed char spiSlave::getTurnVelocityFromPi()
{
  return TurnVelocityFromPi;
};

signed char spiSlave::getForwardThrottleFromPi()
{
  return ForwardThrottleFromPi;
};

signed char spiSlave::getSidewaysThrottleFromPi()
{
  return SidewaysThrottleFromPi;
};

long spiSlave::getParam1FromPi()
{
  return param1FromPi;
};

long spiSlave::getParam2FromPi()
{
  return param2FromPi;
};

long spiSlave::getParam3FromPi()
{
  return param3FromPi;
};


// Returns maximum SPI observed burst duration in ms, since last cleared. 
// This should never be higher than the value of maxAllowedSPIburstDuration.
unsigned char spiSlave::getMaxBurstDuration()
{
  return maxBurstDuration;
}


// Clears an internal register that tracks the maximum oberved SPI burst duration
void spiSlave::clearMaxBurstDuration()
{
  maxBurstDuration = 0;
}


// Returns value of internal register that tracks the number of bursts rejected due to duration above max allowed threshold.
unsigned char spiSlave::getNumBurstsRejectedTooLong()
{
  return numBurstsRejectedTooLong;
};


// Clears an internal register that tracks the  number of bursts rejected due to duration above max allowed threshold.
void spiSlave::clearNumBurstsRejectedTooLong()
{
  numBurstsRejectedTooLong = 0;
};


// Returns maximum observed delay between SPI bursts in ms, since last cleared. 
// This can easily and often be higher than the value of maxAllowedSPIburstDuration, depending primarily how often the SPI Master chooses to initiate transfers.
unsigned long spiSlave::getMaxDelayBetweenBursts()
{
  return maxDelayBetweenBursts;
}


// Clears an internal register that tracks the maximum observed delay between SPI bursts.
void spiSlave::clearMaxDelayBetweenBursts()
{
  maxDelayBetweenBursts = 0;
}


    // Purpose
    //  this method allows using programs to check for and act on any commands received from the Pi SPI Master.
    // ToDo:  Implementation Refactor
    //    Refactor the following to better isolate arbitrary piMegaSPI commands from application layer commands
    //    do this by adding new private arrays 
    //      const char SPIlayerCommandHandlerCommands[n]  // ie. store up to n different 'SPI protocol' layer commands (including 'R'), 
    //        each with a 1:1 correspondance to the switch statement construct like the baseline below
    //      char applicationCommandHandlerCommands[n]  // ie. store up to n different application layer commands
    //      int applicationCommandHandlerFunctions[n]  // ie. store up to n different application layer function pointers
    //    provide a setter to allow applications to populate bindings for up to n different application layer commands and function pointers
    //      and in that function, reject any attempted application layer assignments to commands reserved in SPIlayerCommandHandlerCommands[n] 
    //    change handleCommandsFromPi() to act first on any commands identified in SPIlayerCommandHandlerCommands[n], 
    //      and then through any commands identified in applicationCommandHandlerFunctions[n]
    //    maybe have an additional array bool commandHandlerDefined[2][n], to allow iterating only across defined commands
    // Protocol Assumptions
    //  The following approach was taken assuming that the Pi SPI Master has the luxury of CPU overhead & time, or at least has more flexibilty than the Mega SPI Slave
    //  Specifically, this protocol assumes that the Mega SPI Slave needs to keep the ISR as short as possible and maintain only small variations in execution timing.
    //  Accordingly, this handleCommandsFromPi() method is designated as 'execute asynchronously as Mega CPU time permits' and NOT as 'synchronously execute immediately'
    //  Hence this protocol requires an application layer above the physical byte transfer layer to follow this simple process:
    // Input
    //  none
    // Algorithm
    //  When the Pi SPI Master needs to send a command and be sure that the command was executed,
    //  The Pi SPI master sends one of the defined command values, 
    //    and continues to send that command value with every transfer.
    //  The application program on Mega must occasionally call this handleCommandsFromPi() at it's convenience
    //  After the Mega application program completes the command, 
    //    it provides a "!" in the command byte indicating command executed sucessfully, 
    //    or it provides an "E" in the command byte indicating some error and the command was not executed
    //  and then the Mega continues to send that value in the command with every transfer, 
    //    until this handleCommandsFromPi() method notices that the Master has stopped sending a known command
    //  Note that this protocol relies on 'in-band' signaling,
    //    because commandFromPi is used for signaling to the Pi SPI master,
    //    and hence the Pi SPI master must normally check getNextSPIxferToPiReserved() before
    //    calling setDataForPi()
    // Output
    //  varies based on command, review the code implementation or see documentation if/when available
    void spiSlave::handleCommandsFromPi()
    {
      // until proven otherwise, presume that we'll need to reserve the next SPI transfer
      //   to respond to a command from the Pi SPI master.
      //   this avoids having to repeat the assignemt for every branch
      nextSPIxferToPiReserved = true;
      switch (commandFromPi)
      {
      case 'R': 
        // reset / re-initialize application layer parts of this SPI protocol, for any element above the lowest/physical transfer layer
        clearErrorCountSPIrx();
        clearMaxBurstDuration();
        clearNumBurstsRejectedTooLong();
        clearMaxDelayBetweenBursts();
        setDataForPi('!',0,0,0,0,0,0);  // queue confirmation to Pi that the command was handled
        break;     
      default:
        // clear this flag if no recognized command is detected
        //   this automatically signals to the SPI Master, on the next SPI transfer cycle,
        //   that any prior command has been completed
        nextSPIxferToPiReserved = false;
        break;
      }
    };

// Return status flag indicating whether the next payload for Pi SPI Master has already been reserved and must go through as-is
bool spiSlave::getNextSPIxferToPiReserved()
{
  return nextSPIxferToPiReserved;
};  


// Purpose
//  Interrupt handler
//  Assesses arrival of each byte received via SPI
// Algorithm
//  Ignore it, Reset the transfer protocol state machine, or add the latest transferred byte to an 'being catured' buffer
//  trigger on SPI received interrupts per this template
//   https://roboticsbackend.com/raspberry-pi-master-arduino-uno-slave-spi-communication-with-wiringpi/
/***************************************************************  
Algorithm & protocol derived from polled example code spiHandler()
  ie. crom http://robotics.hobbizine.com/raspiduino.html
   Uses the SPIxferIndex variable to keep track current position in the
   incoming data packet and execute accordingly
   State  -> Meaning & Action
   -2     - wait for to receive start byte
   -1     - once received start byte, send an acknowledge byte
   0..14  - exchange payoad bytes
   15     - exchange the last payload byte, queue a final 'end of burst' handshake byte
   16     - exchange & verify final 'end of burst' handshake byte, 
            flag new / valid data received or not
            reset the protocol burst state machine
****************************************************************/
void spiSlave::spiISR()
{
    unsigned long currentBurstMillis;
    unsigned long currentDelayBetweenBurstsMillis = 0;

    // write to digTP28, to facilitate timing measurements via oscilloscope
    digitalWrite(digTP28, HIGH);

    currentDelayBetweenBurstsMillis = (millis() - SPIwdPriorMillis);

    // update the maxDelayBetweenBursts register if this is a new record high delay between bursts
    if ( currentDelayBetweenBurstsMillis > maxDelayBetweenBursts)
    {
      maxDelayBetweenBursts = currentDelayBetweenBurstsMillis;
    }

    // reset the SPI listener state machine if the prior transfer has clearly taken too long or was prematurely cancelled
    // ie. this will force the listener to wait for another initial 'start new transfer' SPIxferIndex byte
    // choose threshold based on empirical timing observation by oscilloscope plus a little margin
    if ( currentDelayBetweenBurstsMillis > maxAllowedSPIburstDuration)
    {
      if (SPIxferIndex != -2)  // a burst was in progress and it clearly took too long
      {
        numBurstsRejectedTooLong += 1; 
      }
      SPIxferIndex = -2;
      SPIxferInProgress = 0;   // flag to external programs that the receiveBuffer 'can be trusted'
      newSPIdataAvailable = 0; // reset the new data flag, even if prior new data was not read
    }
         
    switch (SPIxferIndex)
    {
    case -2:  // ie. this is potentially the 1st header byte
      SPIwdPriorMillis = millis();   // reset the watchdog timer
      if (SPDR == 's')    // confirmed, we presume this is the 1st header byte
      {
        SPDR = 'a';             // queue an acknowledge byte to be sent on the next transfer
        SPIxferInProgress = 1;  // flag to external programs that the receiveBuffer has WIP / cannot be trusted
        newSPIdataAvailable = 0; // reset the new data flag, even if prior new data was not read
        SPIxferIndex++;
      } 
      break;    
    case -1:  // ie. we presume this is the 2nd header byte
      // => in this case, we have just transferred acknowledge 'a' to the master, and have just received a junk byte
      // Hence do not bother reading the SPDR, because in this protocol, the 2nd header byte contains no information for Mega
      // As a potential improvement, this byte could be used to pass additional data from master to slave
      if (toggleSendBuffer == 1)
      {
        toggleSendBuffer = 0;
        if (sendBufferSelect == 1)
        {
          sendBufferSelect = 0;
        }
        else
        {
          sendBufferSelect = 1;
        }
      }
      SPIxferIndex++;
      SPDR = sendBuffer[sendBufferSelect][SPIxferIndex];   // queue the first byte to transfer from Mega slave to Pi master
      break;
    case 15:   // ie. capture the 16th ie. final payload byte & queue and 'end of burst' acknowledge byte
      receiveBuffer[inProgressReceiveBufferSelect][SPIxferIndex] = SPDR;
      SPIxferIndex++; 
      SPDR = 'Z';   // queue an 'end of burst' handshake byte to be sent on the next transfer
                    // => give confidence (albeit not certainty) that the payload bytes transferred between 'a' and 'z' can be trusted
      break;
    case 16:   // ie. verify that just exchanged what should be a final handshaking byte - 'Z' for 'z'. Reset the state machine.
      if (SPDR == 'z')    // verify that the protocol burst state machines were in sync for both SPI master and slave 
      {
        // The receive buffer should now contain an aligned set of bytes
        // Flip the last completed and in progress receive buffer select indices
        if (inProgressReceiveBufferSelect == 0)
        {
          inProgressReceiveBufferSelect == 1;
          lastCompletedReceiveBufferSelect = 0;
        }
        else
        {
          inProgressReceiveBufferSelect == 0;
          lastCompletedReceiveBufferSelect = 1;
        }

        newSPIdataAvailable = 1;  // as best we can tell, the payload bytes between 'a' and 'z' can be trusted
                                  // flag to external functions that new data is now available                   
      } else
      {
        errorCountSPIrx++;      // count as an error - we got this far, should have received a finaly handshake byte
        newSPIdataAvailable = 0; // flag to external functions that NO new data is available
      }
      SPDR = 0;               // queue a 'defined value' null byte for the next SPI transfer, whenever that may be
      SPIxferIndex = -2;      // reset the state machine
      SPIxferInProgress = 0;  // flag to external functions that the receiveBuffer should be trustable.
      currentBurstMillis = millis() - SPIwdPriorMillis;
      if ( currentBurstMillis > maxBurstDuration)
      {
        maxBurstDuration = (unsigned char) currentBurstMillis;
      }
      break;
    default:   // otherwise, receive index 0..14 (ie. payload bytes 1..15) & advance SPIxferIndex to next state
      receiveBuffer[inProgressReceiveBufferSelect][SPIxferIndex] = SPDR;
      SPIxferIndex++; 
      SPDR = sendBuffer[sendBufferSelect][SPIxferIndex];    // queue the next byte to be transferred
    }
    digitalWrite(digTP28, LOW);   
}

ISR (SPI_STC_vect)
{
    spiSlavePort.spiISR();
}
