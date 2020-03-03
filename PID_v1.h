/**********************************************************************************************
 * Arduino PID Library - Version 1.2.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under the MIT License
 **********************************************************************************************/

#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION	1.2.1

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif




template<typename Tinput = double, typename Toutput = double, typename Tgain = double>
class PID {

  public:

    //Constants used in some of the functions below
    static const uint8_t AUTOMATIC = 1;
    static const uint8_t MANUAL = 0;
    static const uint8_t DIRECT = 0;
    static const uint8_t REVERSE = 1;
    static const uint8_t P_ON_M = 0;
    static const uint8_t P_ON_E = 1;

    // * constructor.  links the PID to the Input, Output, and
    //   Setpoint.  Initial tuning parameters are also set here.
    //   (overload for specifying proportional mode)
    /*Constructor (...)*********************************************************
     *    The parameters specified here are those for for which we can't set up
     *    reliable defaults, so we need to have the user set them.
     ***************************************************************************/
    PID(
        Tinput *const Input, Toutput *const Output, Toutput *const Setpoint,
        const Tgain &Kp, const Tgain &Ki, const Tgain &Kd,
        const uint8_t POn, const uint8_t ControllerDirection
    ) : myInput(Input), myOutput(Output), mySetpoint(Setpoint)
    {
      inAuto = false;

      SetOutputLimits(0, 255);  //default output limit corresponds to the arduino pwm limits

      SampleTime = 100;  //default Controller Sample Time is 0.1 seconds

      SetControllerDirection(ControllerDirection);
      SetTunings(Kp, Ki, Kd, POn);

      lastTime = millis() - SampleTime;
    }


    // * constructor.  links the PID to the Input, Output, and
    //   Setpoint.  Initial tuning parameters are also set here
    /*Constructor (...)*********************************************************
     *    To allow backwards compatability for v1.1, or for people that just want
     *    to use Proportional on Error without explicitly saying so
     ***************************************************************************/
    PID(Tinput *const Input, Toutput *const Output, Toutput *const Setpoint,
        const Tgain &Kp, const Tgain &Ki, const Tgain &Kd,
        const uint8_t ControllerDirection
    ) : PID(Input, Output, Setpoint, Kp, Ki, Kd, P_ON_E, ControllerDirection)
    {
    }

    // * sets PID to either Manual (0) or Auto (non-0)
    /* SetMode(...)****************************************************************
     * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
     * when the transition from manual to auto occurs, the controller is
     * automatically initialized
     ******************************************************************************/
    void SetMode(const uint8_t Mode)
    {
      bool newAuto = (Mode == AUTOMATIC);
      if (newAuto && !inAuto) {  //we just went from manual to auto
        Initialize();
      }
      inAuto = newAuto;
    }

    // * performs the PID calculation.  it should be
    //   called every time loop() cycles. ON/OFF and
    //   calculation frequency can be set using SetMode
    //   SetSampleTime respectively
    /* Compute() **********************************************************************
     *     This, as they say, is where the magic happens.  this function should be called
     *   every time "void loop()" executes.  the function will decide for itself whether a new
     *   pid Output needs to be computed.  returns true when the output is computed,
     *   false when nothing has been done.
     **********************************************************************************/
    bool Compute()
    {
      if (!inAuto)
        return false;
      unsigned long now = millis();
      unsigned long timeChange = (now - lastTime);
      if (timeChange >= SampleTime) {
        // Compute all the working error variables
        Tinput input = *myInput;
        Toutput error = *mySetpoint - input;
        Tinput dInput = (input - lastInput);
        outputSum += (ki * error);

        // Add Proportional on Measurement, if P_ON_M is specified
        if (!pOnE)
          outputSum -= kp * dInput;

        if (outputSum > outMax)
          outputSum = outMax;
        else if (outputSum < outMin)
          outputSum = outMin;

        // Add Proportional on Error, if P_ON_E is specified
        Toutput output;
        if (pOnE)
          output = kp * error;
        else
          output = 0;

        // Compute Rest of PID Output
        output += outputSum - kd * dInput;

        if (output > outMax)
          output = outMax;
        else if (output < outMin)
          output = outMin;
        *myOutput = output;

        // Remember some variables for next time
        lastInput = input;
        lastTime = now;
        return true;
      }
      else {
        return false;
      }
    }

    // * clamps the output to a specific range. 0-255 by default, but
    //   it's likely the user will want to change this depending on
    //   the application
    /* SetOutputLimits(...)****************************************************
     *     This function will be used far more often than SetInputLimits.  while
     *  the input to the controller will generally be in the 0-1023 range (which is
     *  the default already,)  the output will be a little different.  maybe they'll
     *  be doing a time window and will need 0-8000 or something.  or maybe they'll
     *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
     *  here.
     **************************************************************************/
    void SetOutputLimits(const Toutput &Min, const Toutput &Max)
    {
      if (Min >= Max)
        return;
      outMin = Min;
      outMax = Max;

      if (inAuto) {
        if (*myOutput > outMax)
          *myOutput = outMax;
        else if (*myOutput < outMin)
          *myOutput = outMin;

        if (outputSum > outMax)
          outputSum = outMax;
        else if (outputSum < outMin)
          outputSum = outMin;
      }
    }

    // * While most users will set the tunings once in the
    //   constructor, this function gives the user the option
    //   of changing tunings during runtime for Adaptive control
    /* SetTunings(...)*************************************************************
     * Set Tunings using the last-rembered POn setting
     ******************************************************************************/
    void SetTunings(const Tgain &Kp, const Tgain &Ki, const Tgain &Kd)
    {
      SetTunings(Kp, Ki, Kd, pOn);
    }

    // * overload for specifying proportional mode
    /* SetTunings(...)*************************************************************
     * This function allows the controller's dynamic performance to be adjusted.
     * it's called automatically from the constructor, but tunings can also
     * be adjusted on the fly during normal operation
     ******************************************************************************/
    void SetTunings(const Tgain &Kp, const Tgain &Ki, const Tgain &Kd, const uint8_t POn)
    {
      if (Kp < 0 || Ki < 0 || Kd < 0)
        return;

      pOn = POn;
      pOnE = POn == P_ON_E;

      dispKp = Kp;
      dispKi = Ki;
      dispKd = Kd;

      double SampleTimeInSec = ((double) SampleTime) / 1000;
      kp = Kp;
      ki = Ki * SampleTimeInSec;
      kd = Kd / SampleTimeInSec;

      if (controllerDirection == REVERSE) {
        kp = (0 - kp);
        ki = (0 - ki);
        kd = (0 - kd);
      }
    }

    // * Sets the Direction, or "Action" of the controller. DIRECT
    //   means the output will increase when error is positive. REVERSE
    //   means the opposite.  it's very unlikely that this will be needed
    //   once it is set in the constructor.
    /* SetControllerDirection(...)*************************************************
     * The PID will either be connected to a DIRECT acting process (+Output leads
     * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
     * know which one, because otherwise we may increase the output when we should
     * be decreasing.  This is called from the constructor.
     ******************************************************************************/
    void SetControllerDirection(const uint8_t Direction)
    {
      if (inAuto && Direction != controllerDirection) {
        kp = (0 - kp);
        ki = (0 - ki);
        kd = (0 - kd);
      }
      controllerDirection = Direction;
    }

    // * sets the frequency, in Milliseconds, with which
    //   the PID calculation is performed.  default is 100
    /* SetSampleTime(...) *********************************************************
     * sets the period, in Milliseconds, at which the calculation is performed
     ******************************************************************************/
    void SetSampleTime(const unsigned long &NewSampleTime)
    {
      if (NewSampleTime > 0) {
        Tgain ratio = (Tgain) NewSampleTime / (Tgain) SampleTime;
        ki *= ratio;
        kd /= ratio;
        SampleTime = NewSampleTime;
      }
    }


    //Display functions ****************************************************************

    /* Status Funcions*************************************************************
     * Just because you set the Kp=-1 doesn't mean it actually happened.  these
     * functions query the internal state of the PID.  they're here for display
     * purposes.  this are the functions the PID Front-end uses for example
     ******************************************************************************/

    // These functions query the pid for interal values.
    //  they were created mainly for the pid front-end,
    // where it's important to know what is actually
    Tgain GetKp() const
    {
      return dispKp;
    }

    Tgain GetKi() const
    {
      return dispKi;
    }

    Tgain GetKd() const
    {
      return dispKd;
    }

    //  inside the PID:

    uint8_t GetMode() const
    {
      return inAuto ? AUTOMATIC : MANUAL;
    }

    uint8_t GetDirection() const
    {
      return controllerDirection;
    }

  private:

    /* Initialize()****************************************************************
     *	does all the things that need to happen to ensure a bumpless transfer
     *  from manual to automatic mode.
     ******************************************************************************/
    void Initialize()
    {
      outputSum = *myOutput;
      lastInput = *myInput;
      if (outputSum > outMax)
        outputSum = outMax;
      else if (outputSum < outMin)
        outputSum = outMin;
    }

    // * we'll hold on to the tuning parameters in user-entered format for display purposes
    Tgain dispKp;
    Tgain dispKi;
    Tgain dispKd;


    Tgain kp;  // * (P)roportional Tuning Parameter
    Tgain ki;  // * (I)ntegral Tuning Parameter
    Tgain kd;  // * (D)erivative Tuning Parameter

    uint8_t controllerDirection;
    uint8_t pOn;

    // * Pointers to the Input, Output, and Setpoint variables
    //   This creates a hard link between the variables and the
    //   PID, freeing the user from having to constantly tell us
    //   what these values are.  with pointers we'll just know.
    Tinput *const myInput;
    Toutput *const myOutput;
    Toutput *const mySetpoint;

    unsigned long lastTime;
    Toutput outputSum;
    Tinput lastInput;

    unsigned long SampleTime;
    Toutput outMin, outMax;
    bool inAuto, pOnE;
};

#endif

