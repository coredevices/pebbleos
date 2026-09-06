/* Project Kraepelin, Main file
    Copyright (C) 2015 : Nathaniel T. Stockham

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

    This license is taken to apply to any other files in the Project Kraepelin
    Pebble App project.
*/

// #include <pebble_worker.h>
#include "helper_worker.h"

/* HELPER FUNCTIONS */

// TEST : PASSED
int16_t pow_int(int16_t x, int16_t y){
  // Returns x^y if y>0 and x,y are integers
  int16_t r = 1; // result
  for(int16_t i =1; i<= abs(y) ; i++ ){ r = x*r; }
  return r;
}


// TEST : PASSED
/* Take square roots */
uint32_t isqrt(uint32_t x){
  uint32_t op, res, one;

  op = x;
  res = 0;

  /* "one" starts at the highest power of four <= than the argument. */
  one = 1 << 30;  /* second-to-top bit set */
  while (one > op) one >>= 2;

  while (one != 0) {
    if (op >= res + one) {
      op -= res + one;
      res += one << 1;  // <-- faster than 2 * one
    }
    res >>= 1;
    one >>= 2;
  }
  return res;
}

// TEST : PASSED
int32_t integral_abs(int16_t *d, int16_t srti, int16_t endi){
  /* Integrate the absolute values between given srti and endi index*/
  int32_t int_abs = 0;

  for(int16_t i = srti; i <= endi; i++ ){
    int_abs += abs(d[i]);
  }
  return (int_abs > 0) ? int_abs : 1;
}
// TEST : PASSED
int32_t integral_l2(int16_t *d, int16_t srti, int16_t endi){
  /* Integrate the absolute values between given srti and endi index*/
  int32_t int_l2 = 0;

  for(int16_t i = srti; i <= endi; i++ ){
    // printf("n7.3: \n");
    int_l2 += (d[i]*d[i]);
  }
  // to prevent nasty divide by 0 problems
  return (int_l2 > 0) ? int_l2 : 1;
}



// TEST : PASSED
uint8_t get_angle_i(int16_t x, int16_t y, uint8_t n_ang ){

  // get the angle resolution
  int32_t ang_res = TRIG_MAX_ANGLE/n_ang;
  // Get the angle from the pebble lookup

  // !! MAKE SURE RANGE IS APPROPRIATE, ie -TRIG_MAX_ANGLE/2 to TRIG_MAX_ANGLE/2
  int32_t A = atan2_lookup(y, x);

  // IF the pebble has any consistency whatsoever, the -pi/2 to 0
  // for the atan2 will be mapped to the pi to 2*pi geometric angles.
  // This is the only thing that makes sense for consistency across
  // the various elements

  // BUT, in case it doesn't, here is the transformation to use
  // Shift the negative angles (-TRIG_MAX_ANGLE/2 to 0) so range is 0 to TRIG_MAX_ANGLE
//   A = A > 0 ? A : (A + TRIG_MAX_ANGLE);

  // divide out by ang_res to get the index of the angle
  // we need to make sure that in all cases that the returned
  //   index is at MOST one less that n_ang, cause 0-15

  // shift by (ang_res/2) so rounds int, not floor
  return (uint8_t) ( ((A + (ang_res/2))/ang_res < n_ang) ?
                    ((A + (ang_res/2))/ang_res) : 0);
}


/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* +++++++++++++++ ACTIGRAPHY FUNCTIONS +++++++++++++++ */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */



// TEST: PASSED
uint8_t orient_encode(int16_t *mean_ary, uint8_t n_ang){
  //MAX n_ang is 16, as 16*15 + 15 = 255
  // The range of the theta_i and phi_i is 0 to (n_ang-1)
  // get theta, in the x-y plane. theta relative to +x-axis
  uint8_t theta_i = get_angle_i(mean_ary[0], mean_ary[1], n_ang );

  // get phi, in the xy_vm-z plane
  int16_t xy_vm = isqrt(mean_ary[0]*mean_ary[0] + mean_ary[1]*mean_ary[1]);
  // phi rel to  +z-axis, so z is on hoz-axis and xy_vm is vert-axis
  uint8_t phi_i = get_angle_i(mean_ary[2],xy_vm, n_ang );

  return n_ang*phi_i + theta_i;
}


// TEST : NONE
void fft_mag(int16_t *d, int16_t dlenpwr){
  // NOTE! this function modifies the input array
  int16_t dlen = pow_int(2,dlenpwr);

  // evaluate the fourier coefficient magnitude
  // NOTE: coeff @ index 0 and dlen/2 only have real components
  //    so their magnitude is exactly that
  for(int16_t i = 1; i < (dlen/2); i++){
    // NOTE: eval coeff mag for real and imag : R(i) & I(i)
    d[i] = isqrt(d[i]*d[i] + d[dlen-i]*d[dlen-i]);
  }
}


