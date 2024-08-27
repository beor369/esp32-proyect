#include <ESP32Encoder.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Arduino.h>
#include <EEPROM.h>
#include <Adafruit_BusIO_Register.h>
 #include <Adafruit_Sensor.h>
// Pines del sensor de ultrasonido HC-SR04

// Pines del sensor de ultrasonido HC-SR04
#define TRIG_PIN 41
#define ECHO_PIN 40
// Pines del encoder
#define ENCODER_PIN_A 42
#define ENCODER_PIN_B 2

// Configuración del LCD: dirección I2C, columnas y filas
int total_lineas = 2;          // Para indicar cuántas líneas tiene nuestro LCD
int total_columnas = 20;       // Para indicar cuántas columnas tiene nuestro LCD
int pin_SDA = 8;               // Para indicar el pin del SDA
int pin_SCL = 9;               // Para indicar el pin del SCL
int lcdAddress = 0x27;         // Dirección I2C del módulo LCD
LiquidCrystal_I2C lcd(lcdAddress, total_columnas, total_lineas); 

// Configuración del Encoder
ESP32Encoder encoder; // Instancia del encoder
const int botonPin = 1;        // SW al pin 1
const int botonBackPin = 5;    // Botón de retroceso al pin 5

// Definición de estados
enum Estado {
  MENU_PRINCIPAL,
  MENU_CONFIGURACION,
  MENU_PRUEBA,
  TECLADO_VIRTUAL
};

// Variables de estado
Estado estadoActual = MENU_PRUEBA;
int indiceMenu = 0;
int indiceTeclado = 0;

// Definir los menús
const char* menuPrincipal[] = {"Configuración", "Prueba"};
const char* menuConfiguracion[] = {"Valor de Resistencia", "Capacidad del Tanque", "Frecuencia del Click", "Atras"};
const char* subMenuPrueba[] = {"Manual", "Automatico", "Atras"};
const int tamanioMenuPrincipal = sizeof(menuPrincipal) / sizeof(menuPrincipal[0]);
const int tamanioMenuConfiguracion = sizeof(menuConfiguracion) / sizeof(menuConfiguracion[0]);
const int tamanioSubMenuPrueba = sizeof(subMenuPrueba) / sizeof(subMenuPrueba[0]);

// Variables del teclado virtual
String entradaTeclado = "";
const char tecladoVirtual[] = "0123456789-<>";

// Variables para el manejo del estado de los botones
bool estadoAnteriorBoton = HIGH;
bool estadoAnteriorBotonBack = HIGH;

// Prototipos de funciones
void setupEncoder();
void mostrarMenu();
void manejarEstado();
void mostrarTecladoVirtual();
void manejarTecladoVirtual();
float obtenerValorTeclado();
void compararValores();
void mostrarMensaje(const char* mensaje);

void setup() {
  // Inicializar la comunicación I2C
  Wire.begin(pin_SDA, pin_SCL);
  
  // Inicializar la comunicación serial
  Serial.begin(115200);
  
  // Inicializar el LCD con columnas y filas
  lcd.begin(total_columnas, total_lineas);
  lcd.backlight();

  // Inicializar el botón del encoder y el botón de retroceso
  pinMode(botonPin, INPUT_PULLUP);
  pinMode(botonBackPin, INPUT_PULLUP);

  setupEncoder();
  
  // Mostrar el menú inicial
  mostrarMenu();
}

void loop() {
  // Leer el valor del encoder
  int valorEncoder = encoder.getCount();

  if (valorEncoder > 0) {
    if (estadoActual == TECLADO_VIRTUAL) {
      indiceTeclado = (indiceTeclado + 1) % strlen(tecladoVirtual);
    }
    mostrarMenu();
    delay(200); // Debounce
  } else if (valorEncoder < 0) {
    if (estadoActual == TECLADO_VIRTUAL) {
      indiceTeclado = (indiceTeclado - 1 + strlen(tecladoVirtual)) % strlen(tecladoVirtual);
    }
    mostrarMenu();
    delay(200); // Debounce
  }

  // Leer el estado del botón del encoder
  bool estadoBoton = digitalRead(botonPin);
  if (estadoBoton == LOW && estadoAnteriorBoton == HIGH) {
    if (estadoActual == TECLADO_VIRTUAL) {
      manejarTecladoVirtual();
    } else {
      manejarEstado();
      mostrarMenu();
    }
    delay(300); // Debounce
  }
  estadoAnteriorBoton = estadoBoton;

  // Leer el estado del botón de retroceso
  bool estadoBotonBack = digitalRead(botonBackPin);
  if (estadoBotonBack == LOW && estadoAnteriorBotonBack == HIGH) {
    if (estadoActual == TECLADO_VIRTUAL) {
      if (entradaTeclado.length() > 0) {
        entradaTeclado.remove(entradaTeclado.length() - 1);
        mostrarTecladoVirtual();
      } else {
        estadoActual = MENU_PRINCIPAL;
        mostrarMenu();
      }
    } 
    delay(300); // Debounce
  }
  estadoAnteriorBotonBack = estadoBotonBack;
}

void mostrarMenu() {
  lcd.clear();
  switch (estadoActual) {
    case MENU_PRINCIPAL:
      lcd.setCursor(0, 0);
      lcd.print("Menu Principal");
      lcd.setCursor(0, 1);
      lcd.print(menuPrincipal[indiceMenu]);
      break;
    case TECLADO_VIRTUAL:
    mostrarTecladoVirtual();
      break;
    default:
      lcd.setCursor(0, 0);
      lcd.print("Opcion no valida");
      break;
  }
}

void manejarEstado() {
  switch (estadoActual) {
    case MENU_PRINCIPAL:
      if (indiceMenu == 0) {
        estadoActual = MENU_CONFIGURACION;
      } else if (indiceMenu == 1) {
        estadoActual = MENU_PRUEBA;
      }
      break;
    case MENU_CONFIGURACION:
      estadoActual = TECLADO_VIRTUAL;
      mostrarTecladoVirtual();
      break;
    case MENU_PRUEBA:
      compararValores();  // Llama a la función de comparación de valores
      break;
    default:
      break;
  }
}

void mostrarTecladoVirtual() {
  lcd.clear();
  lcd.setCursor(0, 0);
  for (int i = 0; i < strlen(tecladoVirtual); i++) {
    if (i == indiceTeclado) {
      lcd.print("*");
    } else {
      lcd.print(tecladoVirtual[i]);
    }
  }
  lcd.setCursor(0, 1);
  lcd.print(entradaTeclado);
}

float obtenerValorTeclado() {
  // Limpiar el teclado y configurar el estado
  entradaTeclado = "";
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Ingrese valor:");
  
  // Esperar hasta que se ingrese un valor
  while (estadoActual == TECLADO_VIRTUAL) {
    manejarTecladoVirtual();  // Manejar la entrada del teclado
    
    // Si se confirma el valor con '>'
    if (entradaTeclado.length() > 0 && tecladoVirtual[indiceTeclado] == '>') {
      break;
    }
    delay(100);  // Esperar un poco antes de verificar nuevamente
  }
  
  return entradaTeclado.toFloat();  // Convertir el valor ingresado a float y retornarlo
}

void manejarTecladoVirtual() {
  char teclaSeleccionada = tecladoVirtual[indiceTeclado];
  
  if (teclaSeleccionada == '<') {
    estadoActual = MENU_PRINCIPAL;
    mostrarMenu();
  } else if (teclaSeleccionada == '>') {
    // Salir del teclado virtual
    estadoActual = MENU_PRINCIPAL;
  } else if (teclaSeleccionada == '-') {
    if (entradaTeclado.length() > 0) {
      entradaTeclado.remove(entradaTeclado.length() - 1);
    }
  } else {
    entradaTeclado += teclaSeleccionada;
  }
  mostrarTecladoVirtual();
}

void compararValores() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Ingrese 1er valor");
  
  float valor1 = obtenerValorTeclado();
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Ingrese 2do valor");
  
  float valor2 = obtenerValorTeclado();
  
  lcd.clear();
  lcd.setCursor(0, 0);
  if (valor1 > valor2) {
    lcd.print("Valor 1 > Valor 2");
  } else if (valor1 < valor2) {
    lcd.print("Valor 1 < Valor 2");
  } else {
    lcd.print("Valor 1 = Valor 2");
  }
  delay(2000);
  mostrarMenu();
}

void setupEncoder() {
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  encoder.attachHalfQuad(ENCODER_PIN_A, ENCODER_PIN_B);
  encoder.clearCount();
}