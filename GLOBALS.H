      /* globals.h */
typedef unsigned char byte;
typedef unsigned short word;

typedef union
  {
    word w;
    byte i[2];
  } shalem;


#define LOW(wo, by) { shalem x;   x.w = wo;  by=x.i[1]; } 
#define HIGH(wo, by) { shalem x;   x.w = wo;  by=x.i[0]; } 
#define expand(b)  ( (b)?1:0 )
#define boolean(w) ( (w & 0x1)?1:0 )

#define CS0 0x2000
#define CS1 0x2800
#define CS2 0x3000
#define CS3 0x3800
#define CS4 0x0000
#define CS6 0x4000
#define CS7 0x6000


word adConvert(byte ) ;

