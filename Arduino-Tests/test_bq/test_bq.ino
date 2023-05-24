
//set all cells to active
void VoltageSetup() {
  byte adc_msg1[6] = {0xD0, 0x00, 0x03, 0x0A, 0xB8, 0x13};
  
  Serial3.write(adc_msg1, sizeof(adc_msg1));
  delay(100);
}

void VoltageSetup1() {
  byte adc_msg1[7] = {0x90, 0x01, 0x00, 0x03, 0x0A, 0xB8, 0x13};
  
  Serial3.write(adc_msg1, sizeof(adc_msg1));
  delay(100);
}
//turns on GPIOs 3 and 4 for input
void GPIOSetup() {
  byte adc_msg[6] = {0xD0, 0x03, 0x0A, 0x01, 0x0F, 0x84};
  byte adc_msg1[6] = {0xD0, 0x00, 0x0E, 0x12, 0xBC, 0x89};  
  byte adc_msg2[6] = {0xD0, 0x00, 0x0F, 0x12, 0xBD, 0x19};
  Serial3.write(adc_msg, sizeof(adc_msg));
  delay(100);
  Serial3.write(adc_msg1, sizeof(adc_msg1));
  delay(100);
  Serial3.write(adc_msg2, sizeof(adc_msg2));
  delay(100);
}

//pulse GPIO 1 output 2 seconds high/low
void GPIOOutputOne() {
  byte GPIOHigh[6] = {0xD0, 0x00, 0x0E, 0x04, 0x3D, 0x47};  
  Serial3.write(GPIOHigh, sizeof(GPIOHigh));
  delay(2000);
  byte GPIOLow[6] = {0xD0, 0x00, 0x0E, 0x05, 0xFC, 0x87};  
  Serial3.write(GPIOLow, sizeof(GPIOLow));
  delay(2000);
}


//starts main ADC
void ADCSetup() {
  byte adc_msg[6] = {0xD0, 0x03, 0x0D, 0x06, 0x4C, 0x76};
  
  Serial3.write(adc_msg, sizeof(adc_msg));
  delay(500);
}

void ADCSetup1() {
  byte adc_msg[7] = {0x90, 0x01, 0x03, 0x0D, 0x06, 0x91, 0x73};
  
  Serial3.write(adc_msg, sizeof(adc_msg));
  delay(500);
}
//function to loop read voltages
void VoltageRead() {
  byte readd[6] = {0xC0, 0x05, 0x68, 0x1F, 0x42, 0x2D};
  
  Serial3.write(readd, sizeof(readd));
  while(!Serial3.available()) {
    digitalWrite(13, HIGH);
    //Serial.println("Waiting...");
    delay(250);
    digitalWrite(13, LOW);
    delay(250);
  }
  Serial.print("DATA: ");
  int x = 0;
  
  while(Serial3.available()) {
    uint16_t msb = (Serial3.read());
    uint16_t lsb = Serial3.read();
    float voltage = (190.73) * ((msb << 8) | lsb);
    Serial.print(voltage/1000);
    Serial.print(" ");
    x++;
  }
  Serial.println(x);
  
  Serial3.flush();
}

void VoltageRead1() {
  byte readd[7] = {0x80, 0x01, 0x05, 0x68, 0x1F, 0x5A, 0x2B};
  
  Serial3.write(readd, sizeof(readd));
  while(!Serial3.available()) {
    digitalWrite(13, HIGH);
    //Serial.println("Waiting...");
    delay(250);
    digitalWrite(13, LOW);
    delay(250);
  }
  Serial.print("DATA: ");
  int x = 0;
  while(Serial3.available()) {
    uint16_t msb = (Serial3.read());
    uint16_t lsb = Serial3.read();
    float voltage = (190.73) * ((msb << 8) | lsb);
    Serial.print(voltage/1000);
    Serial.print(" ");
    x++;
  }
  Serial.println(x);
  
  Serial3.flush();
}

//function to loop read GPIO 3 and 4
void GPIORead() {
  byte readd[6] = {0xC0, 0x05, 0x92, 0x04, 0x40, 0x86};
  Serial3.write(readd, sizeof(readd));
  while(!Serial3.available()) {
    digitalWrite(13, HIGH);
    delay(250);
    digitalWrite(13, LOW);
    delay(250);
  }
  Serial.print("DATA: ");
  int x = 0;
  while(Serial3.available()) {
    uint16_t msb = (Serial3.read());
    uint16_t lsb = Serial3.read();
    float voltage = (152.59) * ((msb << 8) | lsb);
    Serial.print(voltage/1000);
    Serial.print(" ");
    x++;
  }
  Serial.println(x);
  Serial3.flush();
}

void autoAddressOne() {
  byte autoaddr_1[6] = {0xD0, 0x03, 0x4C, 0x00, 0xFC, 0x24};
  byte autoaddr_2[6] = {0xD0, 0x03, 0x09, 0x01, 0x0F, 0x74};
  byte autoaddr_3[6] = {0xD0, 0x03, 0x06, 0x00, 0xCB, 0x44};
  byte autoaddr_4[6] = {0xD0, 0x03, 0x08, 0x02, 0x4E, 0xE5};
  byte autoaddr_5[7] = {0x90, 0x00, 0x03, 0x08, 0x01, 0xd2, 0x1d};
  byte autoaddr_9[6] = {0xC0, 0x03, 0x06, 0x01, 0x0E, 0x44};

  
  Serial3.write(autoaddr_1, sizeof(autoaddr_1));
  delay(100);
  Serial3.write(autoaddr_2, sizeof(autoaddr_2));
  delay(100);
  Serial3.write(autoaddr_3, sizeof(autoaddr_3));
  delay(100);
  Serial3.write(autoaddr_4, sizeof(autoaddr_4));
  delay(100);
  Serial3.write(autoaddr_5, sizeof(autoaddr_5));
  delay(100);

  Serial3.write(autoaddr_9, sizeof(autoaddr_9));
  delay(100);

  while(!Serial3.available()) {
    digitalWrite(13, HIGH);
    delay(100);
    digitalWrite(13, LOW);
    delay(100);
  }
  Serial.print("Return: ");
  int x = 0;
  while(Serial3.available()) {
    Serial.println(Serial3.read());
  }
  
}

void readThingy() {
  //Serial.print("Return: ");
  int x = 0;
  while(Serial3.available()) {
    Serial.print(Serial3.read());
  }
  
}

void autoAddressTwo() {

  byte soft_reset[7] = {0xD0, 0x03, 0x09, 0x02, 0x4f, 0x75}; //SR 0 for DIR0_REG
  Serial3.write(soft_reset, sizeof(soft_reset));

  
  byte dummy1[6] = {0xD0, 0x03, 0x43, 0x00, 0xF9, 0xD4};
  byte dummy2[6] = {0xD0, 0x03, 0x44, 0x00, 0xFB, 0xE4};
  byte dummy3[6] = {0xD0, 0x03, 0x45, 0x00, 0xFA, 0x74};
  byte dummy4[6] = {0xD0, 0x03, 0x46, 0x00, 0xFA, 0x84};
  byte dummy5[6] = {0xD0, 0x03, 0x47, 0x00, 0xFB, 0x14};
  byte dummy6[6] = {0xD0, 0x03, 0x48, 0x00, 0xFE, 0xE4};
  byte dummy7[6] = {0xD0, 0x03, 0x49, 0x00, 0xFF, 0x74};
  byte dummy8[6] = {0xD0, 0x03, 0x4A, 0x00, 0xFF, 0x84};

  byte autoaddr_2[6] = {0xD0, 0x03, 0x09, 0x01, 0x0F, 0x74}; //BRW En auto addressing
  byte autoaddr_3[6] = {0xD0, 0x03, 0x06, 0x00, 0xCB, 0x44}; //BRW address 1
  byte autoaddr_4[6] = {0xD0, 0x03, 0x06, 0x01, 0x0A, 0x84}; //BRW address 2
  byte autoaddr_5[6] = {0xD0, 0x03, 0x08, 0x02, 0x4E, 0xE5}; //BRW all as stack
  byte autoaddr_6[7] = {0x90, 0x00, 0x03, 0x08, 0x00, 0x13, 0xDD}; //SW base as 1
  byte autoaddr_7[7] = {0x90, 0x01, 0x03, 0x08, 0x03, 0x52, 0x20}; //SW top as 2


  byte read1[6] = {0xC0, 0x03, 0x43, 0x01, 0x3C, 0xD4};
  byte read2[6] = {0xC0, 0x03, 0x44, 0x01, 0x3E, 0xE4};
  byte read3[6] = {0xC0, 0x03, 0x45, 0x01, 0x3F, 0x74};
  byte read4[6] = {0xC0, 0x03, 0x46, 0x01, 0x3F, 0x84};
  byte read5[6] = {0xC0, 0x03, 0x47, 0x01, 0x3E, 0x14};
  byte read6[6] = {0xC0, 0x03, 0x48, 0x01, 0x3B, 0xE4};
  byte read7[6] = {0xC0, 0x03, 0x49, 0x01, 0x3A, 0x74};
  byte read8[6] = {0xC0, 0x03, 0x4A, 0x01, 0x3A, 0x84};

  byte clear[6] = {0xD0, 0x03, 0x32, 0x03, 0x9D, 0x85};
  
  Serial3.write(dummy1, sizeof(dummy1));
  delay(10);
  Serial3.write(dummy2, sizeof(dummy2));
    delay(10);

  Serial3.write(dummy3, sizeof(dummy3));
    delay(10);

  Serial3.write(dummy4, sizeof(dummy4));
    delay(10);

  Serial3.write(dummy5, sizeof(dummy5));
    delay(10);

  Serial3.write(dummy6, sizeof(dummy6));
    delay(10);

  Serial3.write(dummy7, sizeof(dummy7));
    delay(10);

  Serial3.write(dummy8, sizeof(dummy8));
    delay(10);

  Serial3.write(autoaddr_2, sizeof(autoaddr_2));
    delay(10);

  Serial3.write(autoaddr_3, sizeof(autoaddr_3));
    delay(10);

  Serial3.write(autoaddr_4, sizeof(autoaddr_4));
    delay(10);

  Serial3.write(autoaddr_5, sizeof(autoaddr_5));
    delay(10);

  Serial3.write(autoaddr_6, sizeof(autoaddr_6));
    delay(10);

  Serial3.write(autoaddr_7, sizeof(autoaddr_7));
    delay(10);

  Serial3.write(read1, sizeof(read1));
  readThingy();
  Serial3.write(read2, sizeof(read2));
    readThingy();

  Serial3.write(read3, sizeof(read3));
    readThingy();

  Serial3.write(read4, sizeof(read4));
    readThingy();

  Serial3.write(read5, sizeof(read5));
    readThingy();

  Serial3.write(read6, sizeof(read6));
    readThingy();

  Serial3.write(read7, sizeof(read7));
    readThingy();

  Serial3.write(read8, sizeof(read8));
    readThingy();
  Serial3.write(clear, sizeof(clear));
  delay(1000);
  byte autoaddr_9[7] = {0x80, 0x01, 0x03, 0x06, 0x00, 0xd7, 0x82}; //SR 0 for DIR0_REG
  Serial3.write(autoaddr_9, sizeof(autoaddr_9));
  while(!Serial3.available()) {
    digitalWrite(13, HIGH);
    delay(100);
    digitalWrite(13, LOW);
    delay(100);
  }
  Serial.print("got back dir0_addr: ");
  int x = 0;
  while(Serial3.available()) {
    Serial.print(Serial3.read());
    Serial.print(", ");
  }
}

//function to autoaddress for 3 devices
void autoAddressThree() {


  byte autoaddr_1[6] = {0xD0, 0x03, 0x4C, 0x00, 0xFC, 0x24};
  byte autoaddr_2[6] = {0xD0, 0x03, 0x09, 0x01, 0x0F, 0x74};
  byte autoaddr_3[6] = {0xD0, 0x03, 0x06, 0x00, 0xCB, 0x44};
  byte autoaddr_4[6] = {0xD0, 0x03, 0x06, 0x01, 0x0A, 0x84};
  byte autoaddr_5[6] = {0xD0, 0x03, 0x06, 0x02, 0x4A, 0x85};
  byte autoaddr_6[6] = {0xD0, 0x03, 0x08, 0x02, 0x4E, 0xE5};
  byte autoaddr_7[7] = {0x90, 0x00, 0x03, 0x08, 0x00, 0x13, 0xdd};
  byte autoaddr_8[7] = {0x90, 0x02, 0x03, 0x08, 0x03, 0x52, 0x64};

  
  Serial3.write(autoaddr_1, sizeof(autoaddr_1));
  delay(100);
  Serial3.write(autoaddr_2, sizeof(autoaddr_2));
  delay(100);
  Serial3.write(autoaddr_3, sizeof(autoaddr_3));
  delay(100);
  Serial3.write(autoaddr_4, sizeof(autoaddr_4));
  delay(100);
  Serial3.write(autoaddr_5, sizeof(autoaddr_5));
  delay(100);
  Serial3.write(autoaddr_6, sizeof(autoaddr_6));
  delay(100);
  Serial3.write(autoaddr_7, sizeof(autoaddr_7));
  delay(100);
  Serial3.write(autoaddr_8, sizeof(autoaddr_8));
  delay(100);
  
}

void setup() {
  // put your setup code here, to run once:
//  pinMode(15, INPUT_PULLUP);
  pinMode(13, OUTPUT);
  pinMode(14, OUTPUT);
  delay(1000);
  digitalWrite(13, HIGH);
  digitalWrite(14, HIGH);
  delay(1000);
  digitalWrite(14, LOW);
  delay(3);
  digitalWrite(14, HIGH);

  delay(1000);
  Serial.begin(9600);
  Serial3.begin(1000000);
  //autoaddressing stuff for 2 device
//  byte soft_reset[7] = {0xD0, 0x03, 0x09, 0x02, 0x4f, 0x75}; //SR 0 for DIR0_REG
//  Serial3.write(soft_reset, sizeof(soft_reset));
  autoAddressOne();
  ADCSetup();
  VoltageSetup();

}

void loop() {
  // put your main code here, to run repeatedly:

  //reads 16 cells
//  ADCSetup();
//  VoltageSetup();
  VoltageRead();

  //reads GPIO 1 and 2
  //ADCSetup();
  //GPIOSetup();
  //GPIORead();

  //writes to GPIO
  //GPIOOutputOne();
  
}
