/* Project Kraepelin, Main file
The MIT License (MIT)

Copyright (c) 2015, Nathaniel T. Stockham

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

This license is taken to apply to any other files in the Project Kraepelin
Pebble App project.
*/


// #include <pebble_worker.h>
#include "fourier.h"
#include "helper_worker.h"

void fft_2radix_real(int16_t *d, int16_t dlenpwr){

  /* +++++++++++ REAL-VALUED, IN-PLACE, 2-RADIX FOURIER TRANSFORM +++++++++++
  *
  *   This implementation of the fourier transform is taken directly from
  *   Henrik V. Sorensen's 1987 paper "Real-valued Fast Fourier Transform
  *   Algorithms" with slight modifications to allow use of Pebble's cos and
  *   sin lookup functions with input range of 0 to 2*pi angle scaled to
  *   0 to 65536 and output range of -1 to 1 scaled to -65535 to 65536. This
  *   discretization introduces some discrepancies between the results of this
  *   function and the floating point equivalents that are not important for its
  *   use here, but nonetheless documented in the accompanying Julia test code.
  *   INPUT
  *     d = input signal array pointer
  *     dlenpwr = the exponent of the array length, ie: array length = 2^dlenpwr
  *   OUTPUT
  *     d = fourier transformed array pointer, with array of real coefficients of form
  *       [Re(0), Re(1),..., Re(N/2-1), Re(N/2), Im(N/2-1),..., Im(1)]
  */

  int16_t n = pow_int(2,dlenpwr);
  int16_t j = 1;
  int16_t n1 = n -1;
  int16_t k,dt;

  for(int16_t i = 1; i <= n1; i++){
    if(i < j){
      dt = d[j-1];
      d[j-1] = d[i-1];
      d[i-1] = dt;
    }
    k = n/2;
    while(k < j){
      j = j - k;
      k = k/2;
    }
    j = j + k;
  }

  for(int16_t i = 1; i <= n; i += 2){
    dt = d[i-1];
    d[i-1] = dt + d[i];
    d[i] = dt - d[i];
  }

  int16_t n2 = 1;
  int16_t n4,i1,i2,i3,i4,t1,t2;

  int32_t E,A,ss,cc;


  for(int16_t k = 2; k <= dlenpwr ; k++){
    n4 = n2;
    n2 = 2*n4;
    n1 = 2*n2;
    E = TRIG_MAX_ANGLE/n1;

    for(int16_t i = 1; i<= n; i+=n1 ){
      dt = d[i-1];
      d[i-1] = dt + d[i+n2-1];
      d[i+n2-1] = dt - d[i+n2-1];
      d[i+n4+n2-1] = - d[i+n4+n2-1];
      A = E;
      for(int16_t j = 1; j <= (n4-1); j++){
        i1 = i + j;
        i2 = i - j + n2;
        i3 = i + j + n2;
        i4 = i - j + n1;

        ss = sin_lookup(A);
        cc = cos_lookup(A);

        A = A + E;

        t1 = (int16_t) ((d[i3-1]*cc + d[i4-1]*ss)/TRIG_MAX_ANGLE);
        t2 = (int16_t) ((d[i3-1]*ss - d[i4-1]*cc)/TRIG_MAX_ANGLE);

        d[i4-1] = d[i2-1] - t2;
        d[i3-1] = -d[i2-1] - t2;
        d[i2-1] = d[i1-1] - t1;
        d[i1-1] = d[i1-1] + t1;
      }
    }
  }
}
