
#include <EEPROM.h>
#define S0 4
#define S1 5
#define S2 6
#define S3 7
#define sensorOut A0

int frequency_r = 0;
int frequency_g = 0;
int frequency_b = 0;

bool reading = false;
bool read_r = false;
bool read_g = false;
bool read_bk = false;
bool read_b = false;
const int Size = 3;
int nothing [6] = {0,0,0,0,0,0};
int black_block [Size] = {0, 0, 0};
int red_block[Size] = {0, 0, 0};
int green_block[Size] = {0, 0, 0};
int blue_block [Size] = {0, 0, 0};

int button = 3;
int color_counter =0;
int black, red, blue, green = 0;
volatile bool button_state = LOW;
void setup() {
 pinMode(button, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(button), blink, RISING);
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  
  // Setting frequency-scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);
  
  Serial.begin(9600);
    for (int x = 0; x < 5; x++){
    use_sensor();
    delay(100);
  }
  reading = true;
  nothing [0] = frequency_r - 15;
  nothing [1] = frequency_r + 15;
  nothing [2] = frequency_g - 15;
  nothing [3] = frequency_g + 15;
  nothing [4] = frequency_b - 15;
  nothing [5] = frequency_b + 15;
}
void loop() {
  calibrate_EEPROM();
  use_sensor();

  print_color();
  Serial.println("  ");
  delay(100);

}
void blink(){
   button_state = HIGH;
}
void use_sensor(){
   digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  // Reading the output frequency
  frequency_r = pulseIn(sensorOut, LOW);
  // Printing the value on the serial monitor
  Serial.print("R= ");//printing name
  Serial.print(frequency_r);//printing RED color frequency
  Serial.print("  ");
  delay(100);
  // Setting Green filtered photodiodes to be read
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  frequency_g = pulseIn(sensorOut, LOW);
  // Printing the value on the serial monitor
  Serial.print("G= ");//printing name
  Serial.print(frequency_g);//printing RED color frequency
  Serial.print("  ");
  delay(100);
  // Setting Blue filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  frequency_b = pulseIn(sensorOut, LOW);
  // Printing the value on the serial monitor
  Serial.print("B= ");//printing name
  Serial.print(frequency_b);//printing RED color frequency
   if (button_state){
  calibrate();
 }
}
void calibrate(){
  int acc = 4;
switch (color_counter){
    case 0:
  black_block[0] = frequency_r;
  black_block[1] = frequency_g;
  black_block[2] = frequency_b;
  read_bk = true;
  Serial.print("Reading black");
  delay(500);
  break;
  case 1:
  red_block[0] = frequency_r;
  red_block[1] = frequency_g;
  red_block[2] = frequency_b;
  read_r = true;
  Serial.print("Reading red"); // Calibrado
  delay(500);
  break;
  case 2:
  green_block[0] = frequency_r;
  green_block[1] = frequency_g;
  green_block[2] = frequency_b;
  read_g = true;
  Serial.print("Reading green");//Calibrado
  delay(500);
  break;
  case 3:
  blue_block[0] = frequency_r;
  blue_block[1] = frequency_g;
  blue_block[2] = frequency_b;
  read_b = true;
    Serial.print("Reading blue");
  delay(500);
  break;
  default :
      Serial.print("Print NOTHING");
  for (int l = 0; l < 6; l++){
    Serial.print(nothing[l]);
    Serial.print(" ");
  }
    Serial.print("Print BLACK");
  for (int b = 0; b < Size; b++){
    Serial.print(black_block[b]);
    Serial.print(" ");
  }
  Serial.print("Print red");
  for (int x = 0; x < Size; x++){
    Serial.print(red_block[x]);
    Serial.print(" ");
  }
    Serial.print("Print green");
  for (int v = 0; v < Size; v++){
    Serial.print(green_block[v]);
    Serial.print(" ");
  }
    Serial.print("Print blue");
  for (int y = 0; y < Size; y++){
    Serial.print(blue_block[y]);
    Serial.print(" ");
  }
  button_state = LOW;
  delay (200);
  if (button_state){
     for (int h = 0; h < 6; h++){
      nothing[h] = 0;
    }
    for (int q = 0; q < Size; q++){
      black_block[q] = 0;
    }
    for (int a = 0; a < Size; a++){
      red_block[a] = 0;
    }
    for (int s = 0; s < Size; s++){
      green_block[s] = 0;
    }
    for (int d = 0; d < Size; d++){
      blue_block[d] = 0;
    }
    reading = false;
    read_bk = false;
    read_r = false;
    read_g = false;
    read_b = false;
    color_counter = -1;
    Serial.print("Erase");
  }
  while (!button_state){
    delay(100);
  }
}
color_counter++;
button_state = LOW;
}

void print_color(){
  if (frequency_r >= nothing[0] && frequency_r <= nothing[1] && frequency_g >= nothing[2]
 && frequency_g <= nothing[3] && frequency_b >= nothing[4] && frequency_b <= nothing[5]){
  delay(1);
 }
else  if(reading) {
compare_red();
compare_green();
compare_blue();
if (red >= 2){
  if(frequency_r >= red_block[0] - 15 && frequency_r <= red_block[0] + 15){
  Serial.print(" RED");
  }
}
else if (black >= 2){
  Serial.print(" BLACK");
}
else if (green >= 2){
  if(frequency_b >= green_block[2] - 20 && frequency_b <= green_block[2] + 20){
  Serial.print(" GREEN");
  }
  else if(frequency_b >= blue_block[2] - 20 && frequency_b <= blue_block[2] + 20){
  Serial.print(" BLUE");
}
}
else if (blue >= 2){
  if(frequency_b >= blue_block[2] - 20 && frequency_b <= blue_block[2] + 20){
  Serial.print(" BLUE");
}
  else if(frequency_b >= green_block[2] - 20 && frequency_b <= green_block[2] + 20){
  Serial.print(" GREEN");
  }
}
else {
  Serial.print (" NOTHING");
}
red = 0;
black = 0;
green = 0;
blue = 0;
}
else {
  delay (1);
}
}
void compare_red(){
  int compare [4] = {0, 0, 0, 0};
  compare [0] = red_block[0] - frequency_r;
  compare [1] =green_block[0] - frequency_r;
  compare [2] = blue_block[0] - frequency_r;
  compare [3] = black_block[0] - frequency_r;
 for (int x = 0; x < 4; x++){
  if (compare[x] <0){
    compare[x] = compare[x] * -1;
  }
 }
if (compare[0] <= compare[1] && compare[0] <= compare[2] && compare [0] <= compare[3]){
  red++;
}
if (compare[1] <= compare[0] && compare[1] <= compare[2] && compare [1] <= compare[3]){
  green++;
}
if (compare[2] <= compare[0] && compare[2] <= compare[1] && compare [2] <= compare[3]){
  blue++;
}
if (compare[3] <= compare[0] && compare[3] <= compare[1] && compare [3] <= compare[2]){
  black++;
}
}
void compare_green(){

  int compare [4] = {0, 0, 0, 0};
  compare [0] = red_block[1] - frequency_g;
  compare [1] = green_block[1] - frequency_g;
  compare [2] = blue_block[1] - frequency_g;
  compare [3] = black_block[1] - frequency_g;
 for (int x = 0; x < 4; x++){
  if (compare[x] <0){
    compare[x] = compare[x] * -1;
  }
 }
if (compare[0] <= compare[1] && compare[0] <= compare[2] && compare [0] <= compare[3]){
  red++;
}
if (compare[1] <= compare[0] && compare[1] <= compare[2] && compare [1] <= compare[3]){
  green++;
}
if (compare[2] <= compare[0] && compare[2] <= compare[1] && compare [2] <= compare[3]){
  blue++;
}
if (compare[3] <= compare[0] && compare[3] <= compare[1] && compare [3] <= compare[2]){
  black++;
}
}
void compare_blue(){
  int compare [4] = {0, 0, 0, 0};
  compare [0] = red_block[2] - frequency_b;
  compare [1] = green_block[2] - frequency_b;
  compare [2] = blue_block[2] - frequency_b;
  compare [3] = black_block[2] - frequency_b;
 for (int x = 0; x < 4; x++){
  if (compare[x] <0){
    compare[x] = compare[x] * -1;
  }
 }
if (compare[0] <= compare[1] && compare[0] <= compare[2] && compare [0] <= compare[3]){
  red++;
}
if (compare[1] <= compare[0] && compare[1] <= compare[2] && compare [1] <= compare[3]){
  green++;
}
if (compare[2] <= compare[0] && compare[2] <= compare[1] && compare [2] <= compare[3]){
  blue++;
}
if (compare[3] <= compare[0] && compare[3] <= compare[1] && compare [3] <= compare[2]){
  black++;
}
}

void calibrate_EEPROM(){
  int counter;
  for (counter = 0; counter < 6; counter++){
    EEPROM.update(counter, nothing[counter]);
  }
  for ( counter = 0; counter < Size; counter++){
    if (read_bk){
    EEPROM.update(counter + 6, black_block[counter]);
    }
    black_block[counter] = EEPROM.read(counter + 6);
  }
    for ( counter = 0; counter < Size; counter++){
      if (read_r){
    EEPROM.update(counter + 9, red_block[counter]);
      }
      red_block[counter] = EEPROM.read(counter + 9);
  }
    for ( counter = 0; counter < Size; counter++){
      if( read_g){
    EEPROM.update(counter + 12, green_block[counter]);
      }
      green_block[counter] = EEPROM.read(counter + 12);
  }
    for ( counter = 0; counter < Size; counter++){
      if (read_b){
    EEPROM.update(counter + 15, blue_block[counter]);
      }
      blue_block[counter] = EEPROM.read (counter + 15);
  }
}
/*bool secure_color(int color_r, int color_g, int color_b){
  int count;
  count = 0;
  for (int h = 0; h < 10; h++){
     digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  frequency_r = pulseIn(sensorOut, LOW);
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  frequency_g = pulseIn(sensorOut, LOW);
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  frequency_b = pulseIn(sensorOut, LOW);
  if (frequency_r <= color_r + 5 && frequency_r >= color_r - 5 && frequency_g <= color_g + 5 && frequency_g >= color_g - 5 && frequency_b <= color_b + 5 && frequency_b >= color_b - 5){
    count++;
  }
  }
  if (count >= 6){
    return true;
  }
  else {
    return false;
  }
}*/

