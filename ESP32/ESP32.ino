#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>

// Replace with your network's SSID and password.
#define WIFI_SSID       "SSID"
#define WIFI_PASSPHRASE "password"

// Replace with your host device's IP address and the port number passed to the
// "send_file" program.
#define HOST_IP "0.0.0.0"
#define PORT    1337U

// Rx/Tx pins and baud rate for the STM32F103-ESP32 UART connection.
#define RX_PIN 2
#define TX_PIN 3
#define BAUD_RATE 9600

// Output pin to connect to a STM32F103 input pin (this project uses PC_0);
// outputting high at boot will cause it to wait for a new program to be sent.
#define FLASH_SIGNAL_PIN 0

// Output pin to connect to the STM32F103 NRST pin; output low resets
// the chip.
#define RST_SIGNAL_PIN 1

// Byte sent from the STM32F103 to the ESP32 to request data, or from the host
// device to the ESP32 to indicate a file is ready to be sent.
#define RX_BYTE 0xEE

// Byte sent from the ESP32 to the host device to request data.
#define TX_BYTE 0xEE

// The size of the reading/writing buffer.
#define BUF_SIZE 1024

// Approximate maximum wait time for a response from either device in ms.
#define TIMEOUT 5000

WiFiMulti WiFiMulti;

void setup() {
  pinMode(FLASH_SIGNAL_PIN, OUTPUT);
  pinMode(RST_SIGNAL_PIN, OUTPUT);

  digitalWrite(FLASH_SIGNAL_PIN, LOW);
  digitalWrite(RST_SIGNAL_PIN, HIGH);
  
  Serial.begin(BAUD_RATE);
  
  // 8 data bits, 1 stop bit, no parity.
  Serial1.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
  
  // Connect to a WiFi network.
  WiFiMulti.addAP(WIFI_SSID, WIFI_PASSPHRASE);

  Serial.println();
  Serial.print("Waiting for WiFi... ");
  while(WiFiMulti.run() != WL_CONNECTED) {
      Serial.print(".");
      delay(500);
  }
  Serial.println();
  
  Serial.println("WiFi connected.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  delay(500);
}


void loop() {
  // Use WiFiClient class to create TCP connections.
  WiFiClient client;

  // Attempt a connection every 10 seconds.
  if (!client.connect(HOST_IP, PORT)) {
      delay(10000);
      return;
  }
  
  Serial.print("Connected to ");
  Serial.println(HOST_IP);

  if (HostResponse(client)) {
      // Host sent something other than "TX_BYTE"; end the connection.
      unsigned char rx_byte = client.read();
      if (rx_byte != RX_BYTE) {
        Serial.println("Reset signal not received. Closing connection.");
        client.stop();
        return;
      }
      // "TX_BYTE" was received from host; reboot STM32F103 and set flash
      // signal high.
      digitalWrite(RST_SIGNAL_PIN, LOW);
      digitalWrite(FLASH_SIGNAL_PIN, HIGH);
      delay(100);
      digitalWrite(RST_SIGNAL_PIN, HIGH);
  } else {
    // No response from host before timeout; end the connection.
    HostRxTimeout(client);
    return;
  }
  
  static unsigned char buf[BUF_SIZE];

  // Wait to receive "RX_BYTE" from the STM32F103, then send "TX_BYTE"
  // to the host.
  while (1) {
    if (UARTResponse(Serial1)) {
      digitalWrite(FLASH_SIGNAL_PIN, LOW);
      unsigned char rx_byte  = Serial1.read();
      if (rx_byte == RX_BYTE) {
        Serial.println("RX_BYTE received from STM32F103.");
        SendTxByte(client);
        break;
      }
    } else {
      UARTRxTimeout(client);
      return;
    }
  }
  
  for (int i = 0; i < sizeof(int); i++) {
    if (HostResponse(client)) {
      unsigned char rx_byte = client.read();
      buf[i] = rx_byte;
    } else {
      HostRxTimeout(client);
      return;
    }
  }

  // Send the program size to the STM32F103.
  Serial1.write(buf, sizeof(int));
  
  // Expect the size to be sent in network order.
  int program_size = (buf[3] << 0) | (buf[2] << 8) | (buf[1] << 16) | (buf[0] << 24);
  int left_to_write = program_size;
  Serial.printf("Program size: %u\n", program_size);

  // Continue waiting to receive "RX_BYTE" and sending "TX_BYTE," now sending
  // the program to the STM32F103 "BUF_SIZE" bytes at a time until EOF.
  while (left_to_write > 0) {
    if (UARTResponse(Serial1)) {
      unsigned char rx_byte  = Serial1.read();
      if (rx_byte == RX_BYTE) {
        Serial.println("RX_BYTE received from STM32F103.");
        SendTxByte(client);
        if (HostResponse(client)) {
          int bytes_to_read = (left_to_write < BUF_SIZE) ? 
                               left_to_write : BUF_SIZE;
          int bytes_read = client.read(buf, bytes_to_read);
          for (int i = 0; i < bytes_read; i++) {
            Serial1.write(buf[i]);
          }
          if (bytes_read != bytes_to_read) {
            Serial.printf("Bytes expected: %u\nBytes received: %u\n",
                          bytes_to_read, bytes_read);
            client.stop();
            return;
          }
          left_to_write -= bytes_read;
          if (left_to_write == 0) {
            Serial.println("Done sending.");
          }
        } else {
          HostRxTimeout(client);
          return;
        }
      }
    } else {
      UARTRxTimeout(client);
      return;
    }
  }
  
  Serial.println("Closing connection.");
  client.stop();
  delay(5000);
}

// Waits for approximately "TIMEOUT" ms max for the host's reply to become
// available. Returns true if a response was received before the timeout,
// or false otherwise.
bool HostResponse(WiFiClient& client) {
  int i = 0;
  while (!client.available() && (i < TIMEOUT)) {
    i++;
    delay(1);
  }
  return (client.available() > 0);
}

bool UARTResponse(HardwareSerial& serial) {
  int i = 0;
  while (!serial.available() && (i < TIMEOUT)) {
    i++;
    delay(1);
  }
  return (serial.available() > 0);
}

void HostRxTimeout(WiFiClient& client) {
  Serial.println("client.available() timed out.");
  Serial.println("Closing connection.");
  client.stop();
  Serial.println("Waiting 10 seconds before restarting...");
  delay(10000);
}

void UARTRxTimeout(WiFiClient& client) {
  Serial.println("Serial1.available() timed out.");
  Serial.println("Closing connection.");
  client.stop();
}

void SendTxByte(WiFiClient& client) {
  Serial.println("Sending TX_BYTE to host.");
  client.write(TX_BYTE);
}
