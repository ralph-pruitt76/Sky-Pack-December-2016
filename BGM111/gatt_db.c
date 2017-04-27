/********************************************************************
 * Autogenerated file, do not edit.
 *******************************************************************/

#include <stdint.h>
#include "ubt/bg_gattdb_def.h"

#ifdef __GNUC__
#define GATT_HEADER(F) F __attribute__ ((section (".gatt_header"))) 
#define GATT_DATA(F) F __attribute__ ((section (".gatt_data"))) 
#else
#ifdef __ICCARM__
#define GATT_HEADER(F) _Pragma("location=\".gatt_header\"") F 
#define GATT_DATA(F) _Pragma("location=\".gatt_data\"") F 
#else
#define GATT_HEADER(F) F 
#define GATT_DATA(F) F 
#endif
#endif

GATT_DATA(const uint16_t bg_gattdb_uuidtable_16_map [])=
{
    0x2800,
    0x2801,
    0x2803,
    0x1800,
    0x2a00,
    0x2a01,
    0x2901,
    0x2902,
};

GATT_DATA(const uint8_t bg_gattdb_uuidtable_128_map [])=
{
0xab, 0x90, 0x78, 0x56, 0x34, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xb0, 0xb0, 
0xab, 0x90, 0x78, 0x56, 0x34, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xb6, 0xb6, 
0xab, 0x90, 0x78, 0x56, 0x34, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xb7, 0xb7, 
0xab, 0x90, 0x78, 0x56, 0x34, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xb8, 0xb8, 
0xab, 0x90, 0x78, 0x56, 0x34, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xb9, 0xb9, 
0xab, 0x90, 0x78, 0x56, 0x34, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xbb, 0xbb, 
0xab, 0x90, 0x78, 0x56, 0x34, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xbc, 0xbc, 
0xab, 0x90, 0x78, 0x56, 0x34, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xbd, 0xbd, 
0xab, 0x90, 0x78, 0x56, 0x34, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xbe, 0xbe, 
0xab, 0x90, 0x78, 0x56, 0x34, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xbf, 0xbf, 
0x05, 0x00, 0x00, 0x5a, 0x5a, 0x5a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xb1, 0xb1, 
0x05, 0x00, 0x00, 0x5a, 0x5a, 0x5a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x00, 
0x05, 0x00, 0x00, 0x5a, 0x5a, 0x5a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x29, 0x00, 
0x05, 0x00, 0x00, 0x5a, 0x5a, 0x5a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2a, 0x00, 
0x05, 0x00, 0x00, 0x5a, 0x5a, 0x5a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2b, 0x00, 
0x05, 0x00, 0x00, 0x5a, 0x5a, 0x5a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2c, 0x00, 
0x05, 0x00, 0x00, 0x5a, 0x5a, 0x5a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2d, 0x00, 
};




GATT_DATA(const struct bg_gattdb_buffer_with_len	bg_gattdb_attribute_field_66 ) = {
	.len=11,
	.data={0x48,0x65,0x61,0x72,0x74,0x20,0x42,0x65,0x61,0x74,0x32,}
};
uint8_t bg_gattdb_attribute_field_64_data[20]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,};
GATT_DATA(const struct bg_gattdb_attribute_chrvalue	bg_gattdb_attribute_field_64 ) = {
	.properties=0x12,
	.index=14,
	.max_len=20,
	.data=bg_gattdb_attribute_field_64_data,
};

GATT_DATA(const struct bg_gattdb_buffer_with_len	bg_gattdb_attribute_field_63 ) = {
	.len=19,
	.data={0x12,0x41,0x00,0x05,0x00,0x00,0x5a,0x5a,0x5a,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x2d,0x00,}
};
GATT_DATA(const struct bg_gattdb_buffer_with_len	bg_gattdb_attribute_field_62 ) = {
	.len=10,
	.data={0x48,0x65,0x61,0x72,0x74,0x20,0x42,0x65,0x61,0x74,}
};
uint8_t bg_gattdb_attribute_field_60_data[20]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,};
GATT_DATA(const struct bg_gattdb_attribute_chrvalue	bg_gattdb_attribute_field_60 ) = {
	.properties=0x12,
	.index=13,
	.max_len=20,
	.data=bg_gattdb_attribute_field_60_data,
};

GATT_DATA(const struct bg_gattdb_buffer_with_len	bg_gattdb_attribute_field_59 ) = {
	.len=19,
	.data={0x12,0x3d,0x00,0x05,0x00,0x00,0x5a,0x5a,0x5a,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x2c,0x00,}
};
GATT_DATA(const struct bg_gattdb_buffer_with_len	bg_gattdb_attribute_field_58 ) = {
	.len=10,
	.data={0x54,0x69,0x63,0x6b,0x20,0x43,0x6f,0x75,0x6e,0x74,}
};
uint8_t bg_gattdb_attribute_field_56_data[12]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,};
GATT_DATA(const struct bg_gattdb_attribute_chrvalue	bg_gattdb_attribute_field_56 ) = {
	.properties=0x12,
	.index=12,
	.max_len=12,
	.data=bg_gattdb_attribute_field_56_data,
};

GATT_DATA(const struct bg_gattdb_buffer_with_len	bg_gattdb_attribute_field_55 ) = {
	.len=19,
	.data={0x12,0x39,0x00,0x05,0x00,0x00,0x5a,0x5a,0x5a,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x2b,0x00,}
};
GATT_DATA(const struct bg_gattdb_buffer_with_len	bg_gattdb_attribute_field_54 ) = {
	.len=11,
	.data={0x44,0x65,0x76,0x69,0x63,0x65,0x20,0x43,0x6f,0x64,0x65,}
};
uint8_t bg_gattdb_attribute_field_52_data[7]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,};
GATT_DATA(const struct bg_gattdb_attribute_chrvalue	bg_gattdb_attribute_field_52 ) = {
	.properties=0x12,
	.index=11,
	.max_len=7,
	.data=bg_gattdb_attribute_field_52_data,
};

GATT_DATA(const struct bg_gattdb_buffer_with_len	bg_gattdb_attribute_field_51 ) = {
	.len=19,
	.data={0x12,0x35,0x00,0x05,0x00,0x00,0x5a,0x5a,0x5a,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x2a,0x00,}
};
GATT_DATA(const struct bg_gattdb_buffer_with_len	bg_gattdb_attribute_field_50 ) = {
	.len=10,
	.data={0x45,0x72,0x72,0x6f,0x72,0x20,0x43,0x6f,0x64,0x65,}
};
uint8_t bg_gattdb_attribute_field_48_data[7]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,};
GATT_DATA(const struct bg_gattdb_attribute_chrvalue	bg_gattdb_attribute_field_48 ) = {
	.properties=0x12,
	.index=10,
	.max_len=7,
	.data=bg_gattdb_attribute_field_48_data,
};

GATT_DATA(const struct bg_gattdb_buffer_with_len	bg_gattdb_attribute_field_47 ) = {
	.len=19,
	.data={0x12,0x31,0x00,0x05,0x00,0x00,0x5a,0x5a,0x5a,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x29,0x00,}
};
GATT_DATA(const struct bg_gattdb_buffer_with_len	bg_gattdb_attribute_field_46 ) = {
	.len=11,
	.data={0x45,0x72,0x72,0x6f,0x72,0x20,0x43,0x6f,0x75,0x6e,0x74,}
};
uint8_t bg_gattdb_attribute_field_44_data[6]={0x00,0x00,0x00,0x00,0x00,0x00,};
GATT_DATA(const struct bg_gattdb_attribute_chrvalue	bg_gattdb_attribute_field_44 ) = {
	.properties=0x12,
	.index=9,
	.max_len=6,
	.data=bg_gattdb_attribute_field_44_data,
};

GATT_DATA(const struct bg_gattdb_buffer_with_len	bg_gattdb_attribute_field_43 ) = {
	.len=19,
	.data={0x12,0x2d,0x00,0x05,0x00,0x00,0x5a,0x5a,0x5a,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x28,0x00,}
};
GATT_DATA(const struct bg_gattdb_buffer_with_len	bg_gattdb_attribute_field_42 ) = {
	.len=16,
	.data={0x05,0x00,0x00,0x5a,0x5a,0x5a,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xb1,0xb1,}
};
GATT_DATA(const struct bg_gattdb_buffer_with_len	bg_gattdb_attribute_field_41 ) = {
	.len=14,
	.data={0x53,0x4e,0x2f,0x48,0x57,0x52,0x45,0x56,0x2f,0x53,0x57,0x52,0x45,0x56,}
};
uint8_t bg_gattdb_attribute_field_39_data[20]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,};
GATT_DATA(const struct bg_gattdb_attribute_chrvalue	bg_gattdb_attribute_field_39 ) = {
	.properties=0x12,
	.index=8,
	.max_len=20,
	.data=bg_gattdb_attribute_field_39_data,
};

GATT_DATA(const struct bg_gattdb_buffer_with_len	bg_gattdb_attribute_field_38 ) = {
	.len=19,
	.data={0x12,0x28,0x00,0xab,0x90,0x78,0x56,0x34,0x12,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xbf,0xbf,}
};
GATT_DATA(const struct bg_gattdb_buffer_with_len	bg_gattdb_attribute_field_37 ) = {
	.len=8,
	.data={0x50,0x72,0x65,0x73,0x73,0x75,0x72,0x65,}
};
uint8_t bg_gattdb_attribute_field_35_data[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,};
GATT_DATA(const struct bg_gattdb_attribute_chrvalue	bg_gattdb_attribute_field_35 ) = {
	.properties=0x12,
	.index=7,
	.max_len=8,
	.data=bg_gattdb_attribute_field_35_data,
};

GATT_DATA(const struct bg_gattdb_buffer_with_len	bg_gattdb_attribute_field_34 ) = {
	.len=19,
	.data={0x12,0x24,0x00,0xab,0x90,0x78,0x56,0x34,0x12,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xbe,0xbe,}
};
GATT_DATA(const struct bg_gattdb_buffer_with_len	bg_gattdb_attribute_field_33 ) = {
	.len=16,
	.data={0x53,0x77,0x65,0x70,0x74,0x20,0x66,0x72,0x65,0x71,0x20,0x69,0x6e,0x64,0x65,0x78,}
};
uint8_t bg_gattdb_attribute_field_31_data[13]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,};
GATT_DATA(const struct bg_gattdb_attribute_chrvalue	bg_gattdb_attribute_field_31 ) = {
	.properties=0x12,
	.index=6,
	.max_len=13,
	.data=bg_gattdb_attribute_field_31_data,
};

GATT_DATA(const struct bg_gattdb_buffer_with_len	bg_gattdb_attribute_field_30 ) = {
	.len=19,
	.data={0x12,0x20,0x00,0xab,0x90,0x78,0x56,0x34,0x12,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xbd,0xbd,}
};
GATT_DATA(const struct bg_gattdb_buffer_with_len	bg_gattdb_attribute_field_29 ) = {
	.len=10,
	.data={0x57,0x69,0x70,0x65,0x72,0x20,0x46,0x72,0x65,0x71,}
};
uint8_t bg_gattdb_attribute_field_27_data[7]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,};
GATT_DATA(const struct bg_gattdb_attribute_chrvalue	bg_gattdb_attribute_field_27 ) = {
	.properties=0x12,
	.index=5,
	.max_len=7,
	.data=bg_gattdb_attribute_field_27_data,
};

GATT_DATA(const struct bg_gattdb_buffer_with_len	bg_gattdb_attribute_field_26 ) = {
	.len=19,
	.data={0x12,0x1c,0x00,0xab,0x90,0x78,0x56,0x34,0x12,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xbc,0xbc,}
};
GATT_DATA(const struct bg_gattdb_buffer_with_len	bg_gattdb_attribute_field_25 ) = {
	.len=10,
	.data={0x49,0x72,0x72,0x61,0x64,0x69,0x61,0x6e,0x63,0x65,}
};
uint8_t bg_gattdb_attribute_field_23_data[10]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,};
GATT_DATA(const struct bg_gattdb_attribute_chrvalue	bg_gattdb_attribute_field_23 ) = {
	.properties=0x12,
	.index=4,
	.max_len=10,
	.data=bg_gattdb_attribute_field_23_data,
};

GATT_DATA(const struct bg_gattdb_buffer_with_len	bg_gattdb_attribute_field_22 ) = {
	.len=19,
	.data={0x12,0x18,0x00,0xab,0x90,0x78,0x56,0x34,0x12,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xbb,0xbb,}
};
GATT_DATA(const struct bg_gattdb_buffer_with_len	bg_gattdb_attribute_field_21 ) = {
	.len=8,
	.data={0x58,0x59,0x5a,0x20,0x47,0x79,0x72,0x6f,}
};
uint8_t bg_gattdb_attribute_field_19_data[17]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,};
GATT_DATA(const struct bg_gattdb_attribute_chrvalue	bg_gattdb_attribute_field_19 ) = {
	.properties=0x12,
	.index=3,
	.max_len=17,
	.data=bg_gattdb_attribute_field_19_data,
};

GATT_DATA(const struct bg_gattdb_buffer_with_len	bg_gattdb_attribute_field_18 ) = {
	.len=19,
	.data={0x12,0x14,0x00,0xab,0x90,0x78,0x56,0x34,0x12,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xb9,0xb9,}
};
GATT_DATA(const struct bg_gattdb_buffer_with_len	bg_gattdb_attribute_field_17 ) = {
	.len=11,
	.data={0x54,0x65,0x6d,0x70,0x65,0x72,0x61,0x74,0x75,0x72,0x65,}
};
uint8_t bg_gattdb_attribute_field_15_data[7]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,};
GATT_DATA(const struct bg_gattdb_attribute_chrvalue	bg_gattdb_attribute_field_15 ) = {
	.properties=0x12,
	.index=2,
	.max_len=7,
	.data=bg_gattdb_attribute_field_15_data,
};

GATT_DATA(const struct bg_gattdb_buffer_with_len	bg_gattdb_attribute_field_14 ) = {
	.len=19,
	.data={0x12,0x10,0x00,0xab,0x90,0x78,0x56,0x34,0x12,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xb8,0xb8,}
};
GATT_DATA(const struct bg_gattdb_buffer_with_len	bg_gattdb_attribute_field_13 ) = {
	.len=7,
	.data={0x58,0x59,0x5a,0x20,0x4d,0x61,0x67,}
};
uint8_t bg_gattdb_attribute_field_11_data[17]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,};
GATT_DATA(const struct bg_gattdb_attribute_chrvalue	bg_gattdb_attribute_field_11 ) = {
	.properties=0x12,
	.index=1,
	.max_len=17,
	.data=bg_gattdb_attribute_field_11_data,
};

GATT_DATA(const struct bg_gattdb_buffer_with_len	bg_gattdb_attribute_field_10 ) = {
	.len=19,
	.data={0x12,0x0c,0x00,0xab,0x90,0x78,0x56,0x34,0x12,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xb7,0xb7,}
};
GATT_DATA(const struct bg_gattdb_buffer_with_len	bg_gattdb_attribute_field_9 ) = {
	.len=9,
	.data={0x58,0x59,0x5a,0x20,0x41,0x63,0x63,0x65,0x6c,}
};
uint8_t bg_gattdb_attribute_field_7_data[17]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,};
GATT_DATA(const struct bg_gattdb_attribute_chrvalue	bg_gattdb_attribute_field_7 ) = {
	.properties=0x12,
	.index=0,
	.max_len=17,
	.data=bg_gattdb_attribute_field_7_data,
};

GATT_DATA(const struct bg_gattdb_buffer_with_len	bg_gattdb_attribute_field_6 ) = {
	.len=19,
	.data={0x12,0x08,0x00,0xab,0x90,0x78,0x56,0x34,0x12,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xb6,0xb6,}
};
GATT_DATA(const struct bg_gattdb_buffer_with_len	bg_gattdb_attribute_field_5 ) = {
	.len=16,
	.data={0xab,0x90,0x78,0x56,0x34,0x12,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xb0,0xb0,}
};
GATT_DATA(const struct bg_gattdb_buffer_with_len	bg_gattdb_attribute_field_4 ) = {
	.len=2,
	.data={0x00,0x00,}
};
GATT_DATA(const struct bg_gattdb_buffer_with_len	bg_gattdb_attribute_field_3 ) = {
	.len=5,
	.data={0x02,0x05,0x00,0x01,0x2a,}
};
GATT_DATA(const struct bg_gattdb_buffer_with_len	bg_gattdb_attribute_field_2 ) = {
	.len=8,
	.data={0x53,0x6b,0x79,0x20,0x50,0x63,0x6b,0x32,}
};
GATT_DATA(const struct bg_gattdb_buffer_with_len	bg_gattdb_attribute_field_1 ) = {
	.len=5,
	.data={0x02,0x03,0x00,0x00,0x2a,}
};
GATT_DATA(const struct bg_gattdb_buffer_with_len	bg_gattdb_attribute_field_0 ) = {
	.len=2,
	.data={0x00,0x18,}
};
GATT_DATA(const struct bg_gattdb_attribute bg_gattdb_attributes_map[])={
    {.uuid=0x0000,.permissions=0x801,.datatype=0x00,.min_key_size=0x00,.constdata=&bg_gattdb_attribute_field_0},
    {.uuid=0x0002,.permissions=0x801,.datatype=0x00,.min_key_size=0x00,.constdata=&bg_gattdb_attribute_field_1},
    {.uuid=0x0004,.permissions=0x801,.datatype=0x00,.min_key_size=0x00,.constdata=&bg_gattdb_attribute_field_2},
    {.uuid=0x0002,.permissions=0x801,.datatype=0x00,.min_key_size=0x00,.constdata=&bg_gattdb_attribute_field_3},
    {.uuid=0x0005,.permissions=0x801,.datatype=0x00,.min_key_size=0x00,.constdata=&bg_gattdb_attribute_field_4},
    {.uuid=0x0000,.permissions=0x801,.datatype=0x00,.min_key_size=0x00,.constdata=&bg_gattdb_attribute_field_5},
    {.uuid=0x0002,.permissions=0x801,.datatype=0x00,.min_key_size=0x00,.constdata=&bg_gattdb_attribute_field_6},
    {.uuid=0x8001,.permissions=0x801,.datatype=0x01,.min_key_size=0x00,.dynamicdata=&bg_gattdb_attribute_field_7},
    {.uuid=0x0007,.permissions=0x807,.datatype=0x03,.min_key_size=0x00,.configdata={.flags=0x01,.index=0x00}},
    {.uuid=0x0006,.permissions=0x801,.datatype=0x00,.min_key_size=0x00,.constdata=&bg_gattdb_attribute_field_9},
    {.uuid=0x0002,.permissions=0x801,.datatype=0x00,.min_key_size=0x00,.constdata=&bg_gattdb_attribute_field_10},
    {.uuid=0x8002,.permissions=0x801,.datatype=0x01,.min_key_size=0x00,.dynamicdata=&bg_gattdb_attribute_field_11},
    {.uuid=0x0007,.permissions=0x807,.datatype=0x03,.min_key_size=0x00,.configdata={.flags=0x01,.index=0x01}},
    {.uuid=0x0006,.permissions=0x801,.datatype=0x00,.min_key_size=0x00,.constdata=&bg_gattdb_attribute_field_13},
    {.uuid=0x0002,.permissions=0x801,.datatype=0x00,.min_key_size=0x00,.constdata=&bg_gattdb_attribute_field_14},
    {.uuid=0x8003,.permissions=0x801,.datatype=0x01,.min_key_size=0x00,.dynamicdata=&bg_gattdb_attribute_field_15},
    {.uuid=0x0007,.permissions=0x807,.datatype=0x03,.min_key_size=0x00,.configdata={.flags=0x01,.index=0x02}},
    {.uuid=0x0006,.permissions=0x801,.datatype=0x00,.min_key_size=0x00,.constdata=&bg_gattdb_attribute_field_17},
    {.uuid=0x0002,.permissions=0x801,.datatype=0x00,.min_key_size=0x00,.constdata=&bg_gattdb_attribute_field_18},
    {.uuid=0x8004,.permissions=0x801,.datatype=0x01,.min_key_size=0x00,.dynamicdata=&bg_gattdb_attribute_field_19},
    {.uuid=0x0007,.permissions=0x807,.datatype=0x03,.min_key_size=0x00,.configdata={.flags=0x01,.index=0x03}},
    {.uuid=0x0006,.permissions=0x801,.datatype=0x00,.min_key_size=0x00,.constdata=&bg_gattdb_attribute_field_21},
    {.uuid=0x0002,.permissions=0x801,.datatype=0x00,.min_key_size=0x00,.constdata=&bg_gattdb_attribute_field_22},
    {.uuid=0x8005,.permissions=0x801,.datatype=0x01,.min_key_size=0x00,.dynamicdata=&bg_gattdb_attribute_field_23},
    {.uuid=0x0007,.permissions=0x807,.datatype=0x03,.min_key_size=0x00,.configdata={.flags=0x01,.index=0x04}},
    {.uuid=0x0006,.permissions=0x801,.datatype=0x00,.min_key_size=0x00,.constdata=&bg_gattdb_attribute_field_25},
    {.uuid=0x0002,.permissions=0x801,.datatype=0x00,.min_key_size=0x00,.constdata=&bg_gattdb_attribute_field_26},
    {.uuid=0x8006,.permissions=0x801,.datatype=0x01,.min_key_size=0x00,.dynamicdata=&bg_gattdb_attribute_field_27},
    {.uuid=0x0007,.permissions=0x807,.datatype=0x03,.min_key_size=0x00,.configdata={.flags=0x01,.index=0x05}},
    {.uuid=0x0006,.permissions=0x801,.datatype=0x00,.min_key_size=0x00,.constdata=&bg_gattdb_attribute_field_29},
    {.uuid=0x0002,.permissions=0x801,.datatype=0x00,.min_key_size=0x00,.constdata=&bg_gattdb_attribute_field_30},
    {.uuid=0x8007,.permissions=0x801,.datatype=0x01,.min_key_size=0x00,.dynamicdata=&bg_gattdb_attribute_field_31},
    {.uuid=0x0007,.permissions=0x807,.datatype=0x03,.min_key_size=0x00,.configdata={.flags=0x01,.index=0x06}},
    {.uuid=0x0006,.permissions=0x801,.datatype=0x00,.min_key_size=0x00,.constdata=&bg_gattdb_attribute_field_33},
    {.uuid=0x0002,.permissions=0x801,.datatype=0x00,.min_key_size=0x00,.constdata=&bg_gattdb_attribute_field_34},
    {.uuid=0x8008,.permissions=0x801,.datatype=0x01,.min_key_size=0x00,.dynamicdata=&bg_gattdb_attribute_field_35},
    {.uuid=0x0007,.permissions=0x807,.datatype=0x03,.min_key_size=0x00,.configdata={.flags=0x01,.index=0x07}},
    {.uuid=0x0006,.permissions=0x801,.datatype=0x00,.min_key_size=0x00,.constdata=&bg_gattdb_attribute_field_37},
    {.uuid=0x0002,.permissions=0x801,.datatype=0x00,.min_key_size=0x00,.constdata=&bg_gattdb_attribute_field_38},
    {.uuid=0x8009,.permissions=0x801,.datatype=0x01,.min_key_size=0x00,.dynamicdata=&bg_gattdb_attribute_field_39},
    {.uuid=0x0007,.permissions=0x807,.datatype=0x03,.min_key_size=0x00,.configdata={.flags=0x01,.index=0x08}},
    {.uuid=0x0006,.permissions=0x801,.datatype=0x00,.min_key_size=0x00,.constdata=&bg_gattdb_attribute_field_41},
    {.uuid=0x0000,.permissions=0x801,.datatype=0x00,.min_key_size=0x00,.constdata=&bg_gattdb_attribute_field_42},
    {.uuid=0x0002,.permissions=0x801,.datatype=0x00,.min_key_size=0x00,.constdata=&bg_gattdb_attribute_field_43},
    {.uuid=0x800b,.permissions=0x801,.datatype=0x01,.min_key_size=0x00,.dynamicdata=&bg_gattdb_attribute_field_44},
    {.uuid=0x0007,.permissions=0x807,.datatype=0x03,.min_key_size=0x00,.configdata={.flags=0x01,.index=0x09}},
    {.uuid=0x0006,.permissions=0x801,.datatype=0x00,.min_key_size=0x00,.constdata=&bg_gattdb_attribute_field_46},
    {.uuid=0x0002,.permissions=0x801,.datatype=0x00,.min_key_size=0x00,.constdata=&bg_gattdb_attribute_field_47},
    {.uuid=0x800c,.permissions=0x801,.datatype=0x01,.min_key_size=0x00,.dynamicdata=&bg_gattdb_attribute_field_48},
    {.uuid=0x0007,.permissions=0x807,.datatype=0x03,.min_key_size=0x00,.configdata={.flags=0x01,.index=0x0a}},
    {.uuid=0x0006,.permissions=0x801,.datatype=0x00,.min_key_size=0x00,.constdata=&bg_gattdb_attribute_field_50},
    {.uuid=0x0002,.permissions=0x801,.datatype=0x00,.min_key_size=0x00,.constdata=&bg_gattdb_attribute_field_51},
    {.uuid=0x800d,.permissions=0x801,.datatype=0x01,.min_key_size=0x00,.dynamicdata=&bg_gattdb_attribute_field_52},
    {.uuid=0x0007,.permissions=0x807,.datatype=0x03,.min_key_size=0x00,.configdata={.flags=0x01,.index=0x0b}},
    {.uuid=0x0006,.permissions=0x801,.datatype=0x00,.min_key_size=0x00,.constdata=&bg_gattdb_attribute_field_54},
    {.uuid=0x0002,.permissions=0x801,.datatype=0x00,.min_key_size=0x00,.constdata=&bg_gattdb_attribute_field_55},
    {.uuid=0x800e,.permissions=0x801,.datatype=0x01,.min_key_size=0x00,.dynamicdata=&bg_gattdb_attribute_field_56},
    {.uuid=0x0007,.permissions=0x807,.datatype=0x03,.min_key_size=0x00,.configdata={.flags=0x01,.index=0x0c}},
    {.uuid=0x0006,.permissions=0x801,.datatype=0x00,.min_key_size=0x00,.constdata=&bg_gattdb_attribute_field_58},
    {.uuid=0x0002,.permissions=0x801,.datatype=0x00,.min_key_size=0x00,.constdata=&bg_gattdb_attribute_field_59},
    {.uuid=0x800f,.permissions=0x801,.datatype=0x01,.min_key_size=0x00,.dynamicdata=&bg_gattdb_attribute_field_60},
    {.uuid=0x0007,.permissions=0x807,.datatype=0x03,.min_key_size=0x00,.configdata={.flags=0x01,.index=0x0d}},
    {.uuid=0x0006,.permissions=0x801,.datatype=0x00,.min_key_size=0x00,.constdata=&bg_gattdb_attribute_field_62},
    {.uuid=0x0002,.permissions=0x801,.datatype=0x00,.min_key_size=0x00,.constdata=&bg_gattdb_attribute_field_63},
    {.uuid=0x8010,.permissions=0x801,.datatype=0x01,.min_key_size=0x00,.dynamicdata=&bg_gattdb_attribute_field_64},
    {.uuid=0x0007,.permissions=0x807,.datatype=0x03,.min_key_size=0x00,.configdata={.flags=0x01,.index=0x0e}},
    {.uuid=0x0006,.permissions=0x801,.datatype=0x00,.min_key_size=0x00,.constdata=&bg_gattdb_attribute_field_66},
};

GATT_DATA(const uint16_t bg_gattdb_attributes_dynamic_mapping_map[])={
	0x0008,
	0x000c,
	0x0010,
	0x0014,
	0x0018,
	0x001c,
	0x0020,
	0x0024,
	0x0028,
	0x002d,
	0x0031,
	0x0035,
	0x0039,
	0x003d,
	0x0041,
};

GATT_DATA(const uint8_t bg_gattdb_adv_uuid16_map[])={0x0};
GATT_DATA(const uint8_t bg_gattdb_adv_uuid128_map[])={0xab, 0x90, 0x78, 0x56, 0x34, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xb0, 0xb0, 0x05, 0x00, 0x00, 0x5a, 0x5a, 0x5a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xb1, 0xb1, };
GATT_HEADER(const struct bg_gattdb_def bg_gattdb_data )={
    .attributes=bg_gattdb_attributes_map,
    .attributes_max=67,
    .uuidtable_16_size=8,
    .uuidtable_16=bg_gattdb_uuidtable_16_map,
    .uuidtable_128_size=17,
    .uuidtable_128=bg_gattdb_uuidtable_128_map,
    .attributes_dynamic_max=15,
    .attributes_dynamic_mapping=bg_gattdb_attributes_dynamic_mapping_map,
    .adv_uuid16=bg_gattdb_adv_uuid16_map,
    .adv_uuid16_num=0,
    .adv_uuid128=bg_gattdb_adv_uuid128_map,
    .adv_uuid128_num=2,
};

const struct bg_gattdb_def *bg_gattdb=&bg_gattdb_data;
