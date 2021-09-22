/////// Biblioteca button

#define PULLUP HIGH
#define PULLDOWN LOW

#define CURRENT 0
#define PREVIOUS 1
#define CHANGED 2

class Button{
  public:
    Button(uint8_t buttonPin, uint8_t buttonMode=PULLDOWN);
    void pullup();
    void pulldown();
    bool isPressed();
    bool wasPressed();
    bool stateChanged();
  bool uniquePress();
  private:
    uint8_t pin;
    uint8_t mode;
    uint8_t state;
};

Button::Button(uint8_t buttonPin, uint8_t buttonMode){
  this->pin=buttonPin;
    pinMode(pin,INPUT);
  buttonMode==PULLDOWN ? pulldown() : pullup();
    state = 0;
    bitWrite(state,CURRENT,!mode);
}

/*
|| Set pin HIGH as default
*/
void Button::pullup(void){
  mode=PULLUP;
  digitalWrite(pin,HIGH);
}

/*
|| Set pin LOW as default
*/
void Button::pulldown(void){
  mode=PULLDOWN;
  //digitalWrite(pin,LOW);
}

/*
|| Return the bitWrite(state,CURRENT, of the switch
*/
bool Button::isPressed(void){
    bitWrite(state,PREVIOUS,bitRead(state,CURRENT));
    if (digitalRead(pin) == mode){
        bitWrite(state,CURRENT,false);
    } else {
        bitWrite(state,CURRENT,true);
    }
    if (bitRead(state,CURRENT) != bitRead(state,PREVIOUS)){
        bitWrite(state,CHANGED,true);
    }else{
        bitWrite(state,CHANGED,false);
    }
  return bitRead(state,CURRENT);
}

/*
|| Return true if the button has been pressed
*/
bool Button::wasPressed(void){
    if (bitRead(state,CURRENT)){
        return true;
    } else {
        return false;
    }
}

/*
|| Return true if state has been changed
*/
bool Button::stateChanged(void){
    return bitRead(state,CHANGED);
}

/*
|| Return true if the button is pressed, and was not pressed before
*/
bool Button::uniquePress(void){
    return (isPressed() && stateChanged());
}

Button b1=Button(23,CHANGE);//Boton en patilla 23 para cambiar de modo

//incluimos bibliortecas de Transformada de fourier y las distintas necesarias para el OLED
#include <pt.h>
#include <arduinoFFT.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
//Definimos parametros del OLED
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
//Definimos canal de entrada de la senal
#define CHANNEL A0
//Creamos variable de librería de la Transformada de Fourier
arduinoFFT FFT = arduinoFFT();
//Variables pines de salida
byte pinpt=15;
byte ping=2;
byte pinf=4;
byte pine=16;
byte pind=17;
byte pinc=5;
byte pinb=18;
byte pina=19;
byte pin_digito=26;
byte pinled_rojo=32;
byte pinled_verde=33;
//Definimos arrays
float Hz[12]={261.63,277.18,293.66,311.13,329.63,349.23,369.99,392,415.3,440,466,493.88};//Hertzios de las notas en la tercera octava, congruente con el siguiente array
byte NotasName7seg[12]={0x39,0xb9,0x3f,0xbf,0x79,0x71,0xf1,0x7d,0xfd,0x77,0xf7,0x7f};//Leds a encender para poner nombre de nota, considerando punto con sostenido
char *NotasName[12]={"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};//Nombre sin sostenido si lo hubiera de las notas, congruente con los dos anteriores
byte pines_display[8]={pina,pinb,pinc,pind,pine,pinf,ping,pinpt};//Pines a usar en el display de menos a más significativo
byte pines_digitos[4]={27,14,12,13};//Pines de control de los digitos del Display de 4 digitos
int potdiez[4]={1,10,100,1000};//Potencias de 10 (0,1,2,3)
byte tabla7seg[10]={0x3f,0x6,0x5b,0x4f,0x66,0x6d,0x7d,0x7,0x7f,0x6f};//Leds a encender para pones los numeros del 0 al 9

//Variables Fourier
const uint16_t samples = 2048; //This value MUST ALWAYS be a power of 2
double vReal[samples];
double vImag[samples];
const double samplingFrequency = 5000;
unsigned int sampling_period_us;
unsigned long microseconds;
double F_fund;

//Variables de ayuda y control
byte octava;
byte ind;
byte rndm;
bool modo=1;
int maximo;
int minimo;
int A;
int iterar;

//=====================================================================================================================================================================
//_____________________________________________________________________________________________________________________________________________________________________
void setup() {
  
  //Establecemos y definimos comunicacion serial
  Serial.begin(9600);
  
  //Iniciamos OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.display();
  delay(2000); // Pause for 2 seconds
  // Clear the buffer
  display.clearDisplay();

  //Definimos periodo de muestreo
  sampling_period_us = round(1000000*(1.0/samplingFrequency));

  //Definimos pines de entrada y salida
  pinMode(A0,INPUT);
  for (byte i=0;i<8;i++)
  {
    pinMode(pines_display[i],OUTPUT);
  }
  for (byte i=0;i<5;i++)
  {
    pinMode(pines_digitos[i],OUTPUT);
  }
  pinMode(pin_digito,OUTPUT);
  pinMode(pinled_rojo,OUTPUT);
  pinMode(pinled_verde,OUTPUT);
  pinMode(23,OUTPUT);

  //TODOS LOS DISPLAYS APAGADOS
  for (int i=0;i<5;i++)
  {
    digitalWrite(pines_digitos[i],LOW);
  }
  digitalWrite(pin_digito,LOW);
}

//=====================================================================================================================================================================
//LEER AUDIO Y HACER TRANSFORMACION DE FOURIER_________________________________________________________________________________________________________________________
void leeryfourier()
{
  
  //Variables de control para solo considerar senales de determinada amplitud. 1800 comprobado experimentalmente (idealmente 2048)
  maximo=1800;
  minimo=1800;
  
  microseconds = micros();

  for(int i=0; i<samples; i++){
    //Leemos del canal
    vReal[i] = analogRead(CHANNEL);
    vImag[i] = 0;
    
    //Calculamos maximo y minimo
    if(vReal[i]<minimo){
      minimo=vReal[i];
    }
    if(vReal[i]>maximo){
      maximo=vReal[i];
    }
    while(micros() - microseconds < sampling_period_us){
      //empty loop
    }
    microseconds += sampling_period_us;

    //Definimos amplitud
    A=maximo-minimo;
  }
  
  //Transformacion
  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  // Weigh data 
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD);
  F_fund = FFT.MajorPeak(vReal, samples, samplingFrequency);//Calculo de la frecuencia fundamental a partir de la transformada
}

//=====================================================================================================================================================================
//A PARTIR DE UNA FRECUENCIA FUNDAMENTAL, SE OBTIENE LA NOTA MUSICAL Y LA OCTAVA DE LA SENAL___________________________________________________________________________
void notaYoctava (float F_fund){
  
  octava=3;//Se resetea la variable octava
  
  //Bajamos la senal a la octava 3 musical y se cuenta las veces que se ha tenido que iterar
  while(!(F_fund<(Hz[11]+Hz[0]*2)/2))
  {
    F_fund=F_fund/2;
    octava++;
  }
  
  //Se detecta el intervalo al que pertenece la frecuencia fundamental para obtener el indice del array correspondiente y obtener así la nota musical en cuestion
  if ((((Hz[10]/2+Hz[0])/2)<F_fund)&(F_fund<(Hz[0]+Hz[1])/2)){
   ind=0;
  }
  if ((((Hz[0]+Hz[1])/2)<F_fund)&(F_fund<(Hz[1]+Hz[2])/2)){
   ind=1;
  }
  if ((((Hz[1]+Hz[2])/2)<F_fund)&(F_fund<(Hz[2]+Hz[3])/2)){
   ind=2;
  }
  if ((((Hz[2]+Hz[3])/2)<F_fund)&(F_fund<(Hz[3]+Hz[4])/2)){
   ind=3;
  }
  if ((((Hz[3]+Hz[4])/2)<F_fund)&(F_fund<(Hz[4]+Hz[5])/2)){
   ind=4; 
  }
  if ((((Hz[4]+Hz[5])/2)<F_fund)&(F_fund<(Hz[5]+Hz[6])/2)){
   ind=5;
  }
  if ((((Hz[5]+Hz[6])/2)<F_fund)&(F_fund<(Hz[6]+Hz[7])/2)){
   ind=6;
  }
  if ((((Hz[6]+Hz[7])/2)<F_fund)&(F_fund<(Hz[7]+Hz[8])/2)){
   ind=7;
  }
  if ((((Hz[7]+Hz[8])/2)<F_fund)&(F_fund<(Hz[8]+Hz[9])/2)){
   ind=8;
  }
  if ((((Hz[8]+Hz[9])/2)<F_fund)&(F_fund<(Hz[9]+Hz[10])/2)){
   ind=9;
  }
  if ((((Hz[9]+Hz[10])/2)<F_fund)&(F_fund<(Hz[10]+Hz[11])/2)){
   ind=10;
  }
  if ((((Hz[10]+Hz[11])/2)<F_fund)&(F_fund<(Hz[11]+Hz[0]*2)/2)){
   ind=11;
  } 
}

//=====================================================================================================================================================================
//MOSTRAR EN LOS 4 DIGITOS DEL DISPLAY 7 SEGMENTOS EL VALOR, SIN DECIMALES, DE UNA VARIABLE DOUBLE_____________________________________________________________________
void mostrarnum7seg(double num)
{
  int x=num;//Primero convertimos la variable a un entero
  //Controlamos con pines_digitos cuando encender cada digito e imprimimos el numero, con una frecuencia adecuada para no poder detectar el cambio con el ojo humano
  for (int k=0;k<4;k++)
  {
  muestra_7seg((x/(potdiez[k]))%10);
  digitalWrite(pines_digitos[k],HIGH);
  delay(2);
  digitalWrite(pines_digitos[k],LOW);
  }
}

//=====================================================================================================================================================================
//MOSTRAR EN UN DISPLAY 7 SEGMENTOS UN NUMERO ENTRE 0 Y 9 AYUDANDOSE DEL ARRAY "TABLA7SEG"_____________________________________________________________________________
void muestra_7seg(int j) {
  for (int i=0;i<8;i++)
  {
    digitalWrite(pines_display[i],bitRead(tabla7seg[j],i));
  }
}

//=====================================================================================================================================================================
//MOSTRAR EN UN DISPLAY 7 SEGMENTOS UNA NOTA MUSICAL EN NOTACION ANGLOSAJONA, CONSIDERANDO EL PUNTO COMO SOSTENIDO AYUDANDOSE DEL ARRAY NOTASNAME7SEG__________________
void mostrarnota7seg(byte j)//muestra en el display 7seg la nota que se está tocando, donde el punto es sostenido
{
  for (int i=0;i<8;i++){
    digitalWrite(pines_display[i],bitRead(NotasName7seg[j],i));
  }
  digitalWrite(pin_digito,HIGH);
  delay(2);
  digitalWrite(pin_digito,LOW);
}

//=====================================================================================================================================================================
//UNO DE LOS MODOS DE FUNCIONAMIENTO DEL SISTEMA. MOSTRAR EN DISPLAYS LA NOTA Y FRECUENCIA FUNDAMENTAL DE LA SEÑAL RECIBIDA SI LA AMPLITUD ES SUFICIENTEMENTE GRANDE___
void notayhzdisplays(){
  
  //Reseteamos todas las salidas de los pines
  for(byte i=0;i<8;i++)
  {
    digitalWrite(pines_display[i],LOW);
  }
  
 
  //Leemos del canal y hacemos la transformada de Fourier
  leeryfourier();
  //Condiciones de amplitud y frecuencia coherentes para considerar como senal
  if(A>1000){
    iterar=0;
  }
  if (iterar<2){
    iterar++;
    if ((280<F_fund)&(F_fund<1850)){
      //Calculamos la nota y la octava de la senal
      notaYoctava(F_fund);
      //Mostramos la frecuencia fundamental y la nota en los displays durante 0.25 segundos
      int t=millis();
      while(millis()<(t+250))
      {
      mostrarnum7seg (F_fund);
      mostrarnota7seg(ind);
      escribiramplitudyfrecuenciaenoled();
      }
    }
  }
  //Si no se recibe una senal coherente se encienden todos los displays con una ralla horizontal
  else
 {
  for (int k=0;k<4;k++)
  {
    digitalWrite(ping,HIGH);
    digitalWrite(pines_digitos[k],HIGH);
    delay(5);
    digitalWrite(pines_digitos[k],LOW);
  }
  digitalWrite(ping,HIGH);
  digitalWrite(pin_digito,HIGH);
  delay(5);
  digitalWrite(pin_digito,LOW);
  }
}

//=====================================================================================================================================================================
//MOSTRA EN EL OLED LAS VARIABLES AMPLITUD Y FRECUENCIA DE LA SENAL____________________________________________________________________________________________________
void escribiramplitudyfrecuenciaenoled(){
  //Reseteamos Display
  display.clearDisplay();
  //Asignamos valores a los parametros del texto que vamos a escribir
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.print("Amplitud y frecuencia");
  display.setTextSize(3);
  //Colocamos el cursor en punto
  display.setCursor(0,20);
  //Primero escribimos el nombre de la nota
  display.print("A:");
  display.println(A);
  display.print("F:");
  display.print(F_fund);
  display.display();
}

//=====================================================================================================================================================================
//ESCRIBIR EN MITAD DE LA PANTALLA OLED EL NOMBRE DE LA NOTA MUSICAL DETECTADA_________________________________________________________________________________________
void escribirnotaenoled(){
  
  //Reseteamos Display
  display.clearDisplay();
  
  //Mostramos "Toca:" en OLED al principio de la pantalla
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.print("Toca:");
  
  //Primero escribimos el nombre de una nota
  display.setTextSize(4);
  display.setCursor(45,30);
  display.print(NotasName[rndm]);

  display.display();
}

//=====================================================================================================================================================================
//UNO DE LOS MODOS DE FUNCIONAMIENTO DEL SISTEMA. SE IMPRIME EN OLED UNA NOTA ALEATORIA Y HASTA QUE NO SE RECIBA DICHA NOTA NO SE ENCIENDE EL LED VERDE________________
void juego()
{
  rndm=random(0,11);//Elegimos al azar una de las notas, haciendo indice a rndm
  
  escribirnotaenoled();//Mostramos en el OLED la nota que se pide
  
  ind=12;//Valor de variable de control para que siempre se haga por lo menos una vez el while

  //
  //Hasta que se acierte la nota (en cualquier escala),se lee la senal y se muestra en el display, se enciende el led rojo para saver que no se ha acertado
  while (!(rndm==ind))
  {
    //Leemos del canal y hacemos la transformada de Fourier
    leeryfourier();

    //Nos cercioramos de que tenemos una amplitud minima para considerar la senal
    if (A>1000){
      iterar=0;
    }
    
    //Consideramos la senal de mas de 1000 de amplitud y la siguiente
    if(iterar<2){
      
      notaYoctava(F_fund);//Calculamos la nota y la octava de la senal
      
      //Si no es la nota parpadea el led rojo
      if (!(rndm==ind)){
        digitalWrite(pinled_rojo,HIGH);
        delay(250);
        digitalWrite(pinled_rojo,LOW);
      }
    }
   //Iteramos las veces que se ha leido la senal desde la ultima de mas de 1000 de amplitud
   iterar++;
   
   //Condicion de cambio de modo del sistema
   if(b1.uniquePress()){
    modo=!modo;
    display.clearDisplay();
    display.display();
    break;
   }
   
  }
   
   //SE ENCIENDE EL LED VERDE SI SE ACIERTA
   if(rndm==ind){
    digitalWrite(pinled_verde,HIGH);
    delay(250);
    digitalWrite(pinled_verde,LOW);
    
    //Se muestra los Hz de la frecuencia fundamental del acierto y tambien la nota en los distintos displays durante 1 segundo
    t1=millis();
    while(t2-t1<=1000){
     t2=millis();
     mostrarnum7seg(F_fund);
     mostrarnota7seg(ind);
    }
   }
   
}

//=====================================================================================================================================================================
//_____________________________________________________________________________________________________________________________________________________________________
void loop() 
{
  if(b1.uniquePress()){
    modo=!modo;
  }
  if (modo==1)
  {
  notayhzdisplays();
  }
 
  if (modo==0)
  {
    juego();
  }
}
