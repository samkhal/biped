/* These function definitions are necessary for 
   std to work on the Teensy microcontrollers
 */

#ifndef SLAVE_FIX_STD_HPP_
#define SLAVE_FIX_STD_HPP_


namespace std {
  inline void __throw_bad_alloc()
  {
    Serial.println("Unable to allocate memory");
    while(1);
  }

  inline void __throw_length_error( char const*e )
  {
    Serial.print("Length Error :");
    Serial.println(e);
    while(1);
  }
}

#endif