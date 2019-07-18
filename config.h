#pragma once


// UPDATE WITH YOUR TTN KEYS AND ADDR.
static PROGMEM u1_t NWKSKEY[16] = { 0xCC, 0xDF, 0x8C, 0x7F, 0xE5, 0x5F, 0xB0, 0x8E, 0x5D, 0xD3, 0x24, 0x0F, 0x03, 0x9A, 0x13, 0x3B }; // LoRaWAN NwkSKey, network session key 
static u1_t PROGMEM APPSKEY[16] = { 0x3F, 0x39, 0x5E, 0x32, 0x55, 0xCB, 0x5A, 0x82, 0x6F, 0xF2, 0xCC, 0x4B, 0xBA, 0xDA, 0x4C, 0xC1 }; // LoRaWAN AppSKey, application session key 
static const u4_t DEVADDR = 0x26011971 ; // LoRaWAN end-device address (DevAddr)
