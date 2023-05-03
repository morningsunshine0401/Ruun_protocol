// Define the data structures for the position and heading messages
struct Position {
  double latitude;     
  double longitude;    
  double altitude;  
  double vx;  
  double vy;  
  double vz; 
};

struct Heading {
  double airspeed;    
  double groundspeed;   
  double heading;   
  double throttle;  
  double altitude;     
  double climb_rate;  
};

// Define the message header structure
struct MessageHeader {
  double start_byte;          // start byte (0xFE)
  double payload_length;      // length of payload in bytes
  double message_sequence;    // message sequence number
  double system_id;           // ID of the system sending the message
  double component_id;        // ID of the component sending the message
  double message_id;          // ID of the message being sent
};

// Define the message structure
struct Ruun_Message {
  MessageHeader header;      // message header
  Position position;         // position message payload
  Heading heading;           // heading message payload
  double checksum;         // message checksum
};


#include <cmath>

double crc16(const double *data, double len) {
  uint16_t crc = 0xFFFF;
  uint16_t poly = 0x1021;

  // Cast data pointer to unsigned char to work with bytes
  const unsigned char *bytes = (const unsigned char *)data;

  for (int i = 0; i < len; i++) {
    for (int j = 0; j < 8; j++) {
      bool bit = ((bytes[i] >> (7 - j)) & 1) == 1;
      bool msb = ((crc >> 15) & 1) == 1;
      crc <<= 1;
      if (msb ^ bit) crc ^= poly;
    }
  }
  return (double)(crc & 0xFFFF);
}
