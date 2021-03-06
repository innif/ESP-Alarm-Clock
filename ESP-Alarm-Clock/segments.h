#define SEGMENT_0A  0
#define SEGMENT_0B  15
#define SEGMENT_0C  32
#define SEGMENT_0D  34
#define SEGMENT_0E  33
#define SEGMENT_0F  16
#define SEGMENT_0G  17

#define SEGMENT_1A  2
#define SEGMENT_1B  13
#define SEGMENT_1C  30
#define SEGMENT_1D  36
#define SEGMENT_1E  31
#define SEGMENT_1F  14
#define SEGMENT_1G  19

#define SEGMENT_2A  5
#define SEGMENT_2B  10
#define SEGMENT_2C  27
#define SEGMENT_2D  39
#define SEGMENT_2E  28
#define SEGMENT_2F  11
#define SEGMENT_2G  22

#define SEGMENT_3A  7
#define SEGMENT_3B  8
#define SEGMENT_3C  25
#define SEGMENT_3D  41
#define SEGMENT_3E  26
#define SEGMENT_3F  9
#define SEGMENT_3G  24

#define SEGMENT_DOT_UPPER 12
#define SEGMENT_DOT_LOWER 29

#define NUM_OF_SEGMENTS 7
#define NUM_OF_DIGITS 4

uint8_t segments[] = {SEGMENT_0A, SEGMENT_0B, SEGMENT_0C, SEGMENT_0D, SEGMENT_0E, SEGMENT_0F, SEGMENT_0G,
                      SEGMENT_1A, SEGMENT_1B, SEGMENT_1C, SEGMENT_1D, SEGMENT_1E, SEGMENT_1F, SEGMENT_1G,
                      SEGMENT_2A, SEGMENT_2B, SEGMENT_2C, SEGMENT_2D, SEGMENT_2E, SEGMENT_2F, SEGMENT_2G,
                      SEGMENT_3A, SEGMENT_3B, SEGMENT_3C, SEGMENT_3D, SEGMENT_3E, SEGMENT_3F, SEGMENT_3G};

uint8_t numberToSegments[10] = {0b1111110, 0b0110000, 0b1101101, 0b1111001, 0b0110011, 0b1011011, 0b1011111, 0b1110000, 0b1111111, 0b1111011};