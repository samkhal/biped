/* These function definitions are necessary for 
   std to work on the teensies
 */

#ifndef __SLAVE_FIX_STD_H__
#define __SLAVE_FIX_STD_H__


namespace std {
  inline void __throw_bad_alloc()
  {
    Serial.println("Unable to allocate memory");
  }

  inline void __throw_length_error( char const*e )
  {
    Serial.print("Length Error :");
    Serial.println(e);
  }
}

#endif