#define PPM_FRAME_LENGTH 22000
#define PPM_PULSE_LENGTH 300
#define PPM_CHANNELS 11
#define DEFAULT_CHANNEL_VALUE 1500
#define OUTPUT_PIN 14 // pin A7 on Arduino Nano ESP32
#define FAILSAFE 3000 // failsafe timeout in millis

unsigned int prev_timer;

struct packet_t {
  uint16_t header;
  uint16_t channels[5];
  uint16_t cksum;
};

union packet_union_t {
  struct packet_t packet;
  char bytes[14];
};

packet_union_t msg;

#define BUF_SIZE 255
char buf[BUF_SIZE];
int decode_index = 0;

uint16_t channelValue[PPM_CHANNELS] = {1000, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

enum ppmState_e
{
    PPM_STATE_IDLE,
    PPM_STATE_PULSE,
    PPM_STATE_FILL,
    PPM_STATE_SYNC
};

void IRAM_ATTR onPpmTimer()
{

    static uint8_t ppmState = PPM_STATE_IDLE;
    static uint8_t ppmChannel = 0;
    static uint8_t ppmOutput = HIGH;
    static int usedFrameLength = 0;
    uint16_t currentChannelValue;

    portENTER_CRITICAL(&timerMux);

    if (ppmState == PPM_STATE_IDLE)
    {
        ppmState = PPM_STATE_PULSE;
        ppmChannel = 0;
        usedFrameLength = 0;
        ppmOutput = HIGH;
    }

    if (ppmState == PPM_STATE_PULSE)
    {
        ppmOutput = LOW;
        usedFrameLength += PPM_PULSE_LENGTH;
        ppmState = PPM_STATE_FILL;

        timerAlarmWrite(timer, PPM_PULSE_LENGTH, true);
    }
    else if (ppmState == PPM_STATE_FILL)
    {
        ppmOutput = HIGH;
        currentChannelValue = channelValue[ppmChannel];

        ppmChannel++;
        ppmState = PPM_STATE_PULSE;// pulse type when symbol
        if (ppmChannel >= PPM_CHANNELS)
        {
            ppmChannel = 0;
            timerAlarmWrite(timer, PPM_FRAME_LENGTH - usedFrameLength, true);
            usedFrameLength = 0;
        }
        else
        {
            usedFrameLength += currentChannelValue - PPM_PULSE_LENGTH;
            timerAlarmWrite(timer, currentChannelValue - PPM_PULSE_LENGTH, true);
        }
    }
    portEXIT_CRITICAL(&timerMux);
    digitalWrite(OUTPUT_PIN, ppmOutput);
}


void setup()
{
    //Declare Pin output
    pinMode(OUTPUT_PIN, OUTPUT);

    //Setup Serial Connection
    Serial.begin(57600);
    Serial.setTimeout(100); /////RECHECK THIS default is 1ms
    memset(msg.bytes, 0, 14);


    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onPpmTimer, true);
    timerAlarmWrite(timer, 12000, true);
    timerAlarmEnable(timer);
}

void loop() {
  // see how many bytes are available
  size_t bytes_avail = Serial.available();

  // return if no bytes available
  if (bytes_avail == 0) {
    Serial.println("Bytes not available...");
    return;
  }

  // read bytes
  size_t bytes_to_read = bytes_avail;
  if (bytes_to_read > BUF_SIZE) {
      bytes_avail = BUF_SIZE;
  }
  size_t bytes_read = Serial.readBytes(buf, bytes_to_read);
  // Serial.print("bytes read: ");
  // Serial.println(bytes_read);

  // decode message
  for (int i=0;i<bytes_read;i++) {
    uint8_t curr_byte = buf[i];
    // Serial.print("byte: ");
    // Serial.println(curr_byte);
    if (decode_index == 0) { // looking for first byte of header
      if (curr_byte == 255) {
        decode_index = 1;
      }
    } else if (decode_index == 1) { // looking for second byte of header
      if (curr_byte == 255) {
        decode_index = 2;
      } else {
        decode_index = 0;
      }
    } else if (decode_index < 12) { // reading channels
      msg.bytes[decode_index] = curr_byte;
      decode_index++;
    } else if (decode_index == 12) { // first byte of checksum
      msg.bytes[decode_index] = curr_byte;
      decode_index++;
    } else if (decode_index == 13) { // second byte of checksum
      msg.bytes[decode_index] = curr_byte;
      uint16_t sum = 0;
      for (int i=0;i<5;i++) {
        sum += msg.packet.channels[i];
      }
      // Serial.print("Sum: ");
      // Serial.println(sum);
      // Serial.print("msg checkSum: ");
      // Serial.println(msg.packet.cksum);
      if (sum == msg.packet.cksum) {
        // we have good data, send to servos
        // Serial.print("good");
        for (int i=0;i<5;i++) {
          channelValue[i] = msg.packet.channels[i];
          Serial.print(msg.packet.channels[i]);

        }
        Serial.println();
    
      } else {
        Serial.println("checksum failed: ");
        Serial.println(sum);
      }
      decode_index = 0;
    } else { // should never get here
      Serial.println("should never get here!");
      decode_index = 0;
    }
    // Serial.print("decode index: ");
    // Serial.println(decode_index);
  }
}
