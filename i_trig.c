//*****************************************************************************
//
// T_LUT index is tan*100, entry is angle in degrees
int const t_lut[101] = {0, 1, 1, 2, 3, 3 ,4, 4, 5, 5, 6, 7 ,7, 8, 8, 9, 9,10,10,11,12,12,13,13,14,14,15,15,16,16,17,17,18,19,
                     19,20,20,21,21,22,22,23,23,24,24,24,25,25,26,26,27,27,28,28,29,29,29,30,30,31,31,32,32,32,33,33,34,34,
                     34,35,35,36,36,36,37,37,37,38,38,38,39,39,40,40,40,41,41,41,42,42,42,42,43,43,43,44,44,44,45,45,45 };
//
//*****************************************************************************
//
// integer atan(x,y)
// x, y >=0
//
static int i_atan(int x,int y) {
  int lu;
  if (x<y) {
      lu=(100*x+(y>>1))/y;
      return t_lut[lu];
  } else {
      lu=(100*y+(x>>1))/x;
      return 90-t_lut[lu];
  }
}
//
//*****************************************************************************
//
// s_lut index is angles in degrees, 0..90, entry is sin*100
//
int const s_lut[91] = { 1, 3, 4, 6, 8,10,11,13,15,17,18,20,22,23,25,27,28,30,32,33,35,37,38,40, 41, 43, 45, 46, 48, 49,
                        51,52,54,55,57,58,59,61,62,64,65,66,68,69,70,71,73,74,75,76,77,78,79,80, 81, 82, 83, 84, 85, 86,
                        87,88,89,89,90,91,92,92,93,94,94,95,95,96,96,97,97,98,98,98,99,99,99,99,100,100,100,100,100,100,100 };
//
//*****************************************************************************
//
// integer atan2(x,y)
//
// inputs integer x and y about -4k to +4k
// angle returned integer degrees  0..359
//
//*****************************************************************************
//
int i_atan2(int x,int y) {
  int result;
  if (x>=0) { // x + or 0
    if (y>=0) // y = or 0
      result=i_atan(x,y);
    else // y -
      result=180-i_atan(x,-y);
  } else { // x -
    if (y>=0) // y = or 0
      result=360-i_atan(-x,y);
    else // y -
      result=180+i_atan(-x,-y);
  }
  return result;
}
//
//*****************************************************************************
//
// integer sine and cosine
// input is angle in degrees
//
// output scaled 100 decimal
//
//*****************************************************************************
//
int i_sin(int a) {
  if (a<0) a+=360;
  if (a>=180) {
      a-=180;
      if (a>=90)
        return -s_lut[180-a];
      else
        return -s_lut[a];
  } else { //a<180
  if (a>=90)
    return s_lut[180-a];
  else
    return s_lut[a];
  }
}
//
int i_cos(int a) {
  return i_sin(90-a);
}
//
//*****************************************************************************
//
