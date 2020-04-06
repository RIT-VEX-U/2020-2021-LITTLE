#ifndef _RGB_CON_
#define _RGB_CON_

#include "vex.h"

/**
 * A simple rgb controller for controlling the arduino, which in turn controls the led strip.
 * Sends binary over 3 seperate 3 wire ports for up to 7 color modes, and off.
 */
class RGBController
{
  private:
    vex::digital_out port1, port2, port3;

  public:

  enum rgb_t
  {
    Off, Blue, Red, Green, OrangeWhite, Rainbow
  };

  RGBController(rgb_t starting_color, triport::port port1, triport::port port2, triport::port port3)
  : port1(port1), port2(port2), port3(port3)
  {
    set(starting_color);
  }

  void set(rgb_t color)
  {
    switch(color)
    {
      case Blue:
        port1.set(1);
        port2.set(0);
        port3.set(0);
      break;
      case Red:
        port1.set(0);
        port2.set(1);
        port3.set(0);
      break;
      case Green:
        port1.set(1);
        port2.set(1);
        port3.set(0);
      break;
      case OrangeWhite:
        port1.set(0);
        port2.set(0);
        port3.set(1);
      break;
      case Rainbow:
        port1.set(1);
        port2.set(0);
        port3.set(1);
      break;

      default:
      case Off:
        port1.set(0);
        port2.set(0);
        port3.set(0);
      break;
    }
  }

  
};
#endif