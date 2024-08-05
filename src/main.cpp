#include <ESP32Encoder.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Arduino.h>
#include <EEPROM.h>

// Pines del sensor de ultrasonido HC-SR04
#define TRIG_PIN 41
#define ECHO_PIN 40
// Pines del encoder
#define ENCODER_PIN_A 42
#define ENCODER_PIN_B 2
// Pines del sensor de flujo YF-S201
#define FLOW_SENSOR_PIN 4

// Pin del micrófono
const int micPinA0 = 10;  // Pin analógico A2 del micrófono (GPIO36)

// Umbral para detectar un clic
const int threshold = 500;
// Configuración de la EEPROM
const int EEPROM_SIZE = 512; // Tamaño de la EEPROM
const int BASE_ADDRESS = 0; // Dirección base en la EEPROM para almacenar las resistencias de los inyectores base
const int CLICK_FREQUENCY_ADDRESS = BASE_ADDRESS + sizeof(float); // Dirección en la EEPROM para almacenar la frecuencia del clic

// Configuración del LCD: dirección I2C, columnas y filas
int total_lineas = 2;
int total_columnas = 16;
int pin_SDA = 8;
int pin_SCL = 9;
int lcdAddress = 0x27; // Dirección I2C del módulo LCD
LiquidCrystal_I2C lcd(lcdAddress, total_columnas, total_lineas);

volatile uint32_t pulseCount = 0; // Contador de pulsos
float calibrationFactor = 4.5; // Ajusta según tu sensor
float flowRate = 0;
float flowMillilitres = 0;
unsigned long oldTime = 0;
// Configuración del Encoder
ESP32Encoder encoder; // Instancia del encoder
const int botonPin = 1;        // SW al pin 1

unsigned long lastEncoderMoveTime = 0;
const unsigned long encoderMoveDelay = 200; // Retardo de 200 ms para el filtro de debouncing

// Definición de estados
enum Estado {
  MENU_SELECCION_INYECTOR,
  MENU_RESISTENCIA,
  MENU_PRUEBA,
  SUBMENU_MANUAL,
  SUBMENU_AUTOMATICO,
  SUBMENU_MID_RES,
  SUBMENU_FLUJO,
  SUBMENU_FUGAS,
  SUBMENU_CLICK,
  SUBMENU_ABANICO,
  SUBMENU_RESULTADOS
};
// Variables de estado
Estado estadoActual = MENU_SELECCION_INYECTOR;
int indiceMenu = 0;
int indiceSubMenu = 0;
int indiceSubSubMenu = 0;
int indiceVirtualKeypad = 0;

// Definir los menús
const char* tiposInyector[] = {"Tipo 1", "Tipo 2", "Tipo 3", "Tipo 4", "Tipo 5"};
const char* subMenuPrueba[] = {"Manual", "Automatico", "Atras"};
const char* subMenuManual[] = {"Medir Resistencia", "Flujo", "Fugas", "Click del Inyector", "Prueba de Abanico", "Resultados", "Atras"};
const char* virtualKeypad[] = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "."};
const int tamanioTiposInyector = sizeof(tiposInyector) / sizeof(tiposInyector[0]);
const int tamanioSubMenuPrueba = sizeof(subMenuPrueba) / sizeof(subMenuPrueba[0]);
const int tamanioSubMenuManual = sizeof(subMenuManual) / sizeof(subMenuManual[0]);
const int tamanioVirtualKeypad = sizeof(virtualKeypad) / sizeof(virtualKeypad[0]);

// Variable para el estado anterior del botón del encoder
bool estadoAnteriorBoton = HIGH;

// Prototipos de funciones
bool isButtonPressed();
void checkInjectorFlow();
float calculateFlowRate(uint32_t pulses);
void showResults();
void detectInjectorClickk();
void requestData(const char* prompt);
void checkInjectorResistance();
String readFloatFromVirtualKeypad();
void storeFloatInEEPROM(float value, int address);
float readFloatFromEEPROM(int address);
void compareResistances(float baseResistance, float testResistance);
void mostrarMenu();
void manejarEstado();
void mensajeBienvenida(const char* mensaje);
void IRAM_ATTR pulseCounter(); // Necesario para las ISR en ESP32
void setupLCD();
void setupFlowSensor();
void displayFlowRate();
void setupEncoder();
void displayEncoderPosition();
void pruebaDeAbanico();
void mostrarResultado();

void setup() {
   // Inicializar la comunicación serial
     Serial.begin(115200);
    // Inicializar la comunicación I2C
     Wire.begin(pin_SDA, pin_SCL);
  // Inicializar el LCD
      setupLCD();
    // Inicializar el sensor de flujo
    setupFlowSensor();
     // Inicializar el encoder
    setupEncoder();
    // Configurar interrupción para el sensor de flujo
     attachInterrupt(FLOW_SENSOR_PIN, pulseCounter, FALLING);
    // Inicializar la EEPROM
    EEPROM.begin(EEPROM_SIZE);
      // Inicializar botones
  pinMode(botonPin, INPUT_PULLUP);
    displayEncoderPosition();
  // Inicializar el micrófono
     pinMode(micPinA0, INPUT);
   // Mostrar el menú inicial
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
    mostrarMenu();
}

void loop() {
    displayEncoderPosition();
}

void displayEncoderPosition() {
  int valorEncoder = encoder.getCount();
 
  if (millis() - lastEncoderMoveTime > encoderMoveDelay) {
    if (valorEncoder > 0) {
      if (estadoActual == MENU_SELECCION_INYECTOR) {
        indiceMenu = (indiceMenu + 1) % tamanioTiposInyector;
      } else if (estadoActual == MENU_PRUEBA) {
        indiceSubMenu = (indiceSubMenu + 1) % tamanioSubMenuPrueba;
      } else if (estadoActual == SUBMENU_MANUAL) {
        indiceSubSubMenu = (indiceSubSubMenu + 1) % tamanioSubMenuManual;
      } else if (estadoActual == MENU_RESISTENCIA) {
        indiceVirtualKeypad = (indiceVirtualKeypad + 1) % tamanioVirtualKeypad;
      }
      encoder.clearCount();
      mostrarMenu();
      lastEncoderMoveTime = millis();
    } 
    else if (valorEncoder < 0) {
      if (estadoActual == MENU_SELECCION_INYECTOR) {
        indiceMenu = (indiceMenu - 1 + tamanioTiposInyector) % tamanioTiposInyector;
      } else if (estadoActual == MENU_PRUEBA) {
        indiceSubMenu = (indiceSubMenu - 1 + tamanioSubMenuPrueba) % tamanioSubMenuPrueba;
      } else if (estadoActual == SUBMENU_MANUAL) {
        indiceSubSubMenu = (indiceSubSubMenu - 1 + tamanioSubMenuManual) % tamanioSubMenuManual;
      } else if (estadoActual == MENU_RESISTENCIA) {
        indiceVirtualKeypad = (indiceVirtualKeypad - 1 + tamanioVirtualKeypad) % tamanioVirtualKeypad;
      }
      encoder.clearCount();
      mostrarMenu();
      lastEncoderMoveTime = millis();
    }
  }

  bool estadoBoton = digitalRead(botonPin);
  if (estadoBoton == LOW && estadoAnteriorBoton == HIGH) {
    manejarEstado();
    mostrarMenu();
    delay(300); // Debounce
  }
  estadoAnteriorBoton = estadoBoton;
}

void IRAM_ATTR pulseCounter() {
  pulseCount++;
}

void setupEncoder() {
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  encoder.attachHalfQuad(ENCODER_PIN_A, ENCODER_PIN_B);
  encoder.clearCount();
}

void setupLCD() {
  lcd.begin(total_columnas, total_lineas); // Inicializa el LCD con columnas y filas
  lcd.backlight(); // Encender la luz de fondo del LCD
}

void setupFlowSensor() {
  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP); // Configura el pin del sensor de flujo como entrada con pull-up
}

void mostrarMenu() {
  lcd.clear();
  switch (estadoActual) {
    case MENU_SELECCION_INYECTOR:
      lcd.setCursor(0, 0);
      lcd.print("Tipo de Inyector:");
      lcd.setCursor(0, 1);
      lcd.print(tiposInyector[indiceMenu]);
      break;
    case MENU_RESISTENCIA:
      lcd.setCursor(0, 0);
      lcd.print("Resistencia:");
      lcd.setCursor(0, 1);
      lcd.print(virtualKeypad[indiceVirtualKeypad]);
      break;
    case MENU_PRUEBA:
      lcd.setCursor(0, 0);
      lcd.print("Modo de Prueba");
      lcd.setCursor(0, 1);
      lcd.print(subMenuPrueba[indiceSubMenu]);
      break;
    case SUBMENU_MANUAL:
      lcd.setCursor(0, 0);
      lcd.print("Modo Manual");
      lcd.setCursor(0, 1);
      lcd.print(subMenuManual[indiceSubSubMenu]);
      break;
    case SUBMENU_AUTOMATICO:
      lcd.setCursor(0, 0);
      lcd.print("Modo Automatico");
      break;
    case SUBMENU_MID_RES:
      lcd.setCursor(0, 0);
      lcd.print("Medir Resistencia");
      break;
    case SUBMENU_FLUJO:
      lcd.setCursor(0, 0);
      lcd.print("Medir Flujo");
      break;
    case SUBMENU_FUGAS:
      lcd.setCursor(0, 0);
      lcd.print("Detectar Fugas");
      break;
    case SUBMENU_CLICK:
      lcd.setCursor(0, 0);
      lcd.print("Click del Inyector");
      break;
    case SUBMENU_ABANICO:
      lcd.setCursor(0, 0);
      lcd.print("Prueba de Abanico");
      break;
    case SUBMENU_RESULTADOS:
      lcd.setCursor(0, 0);
      lcd.print("Mostrar Resultados");
      break;
  }
}

void manejarEstado() {
  switch (estadoActual) {
    case MENU_SELECCION_INYECTOR:
      estadoActual = MENU_RESISTENCIA;
      break;
    case MENU_RESISTENCIA:
      checkInjectorResistance();
      break;
    case MENU_PRUEBA:
      if (indiceSubMenu == 0) {
        estadoActual = SUBMENU_MANUAL;
        indiceSubSubMenu = 0;
      } else if (indiceSubMenu == 1) {
        estadoActual = SUBMENU_AUTOMATICO;
      } else if (indiceSubMenu == 2) {
        estadoActual = MENU_SELECCION_INYECTOR;
        indiceSubMenu = 0;
      }
      break;
    case SUBMENU_MANUAL:
      if (indiceSubSubMenu == 0) {
        checkInjectorResistance();
      } else if (indiceSubSubMenu == 1) {
        displayFlowRate();
      } else if (indiceSubSubMenu == 2) {
        mostrarResultado();
      } else if (indiceSubSubMenu == 3) {
        detectInjectorClickk();
      } else if (indiceSubSubMenu == 4) {
        pruebaDeAbanico();
      } else if (indiceSubSubMenu == 5) {
        showResults();
      } else if (indiceSubSubMenu == 6) {
        estadoActual = MENU_PRUEBA;
        indiceSubSubMenu = 0;
      }
      break;
    case SUBMENU_AUTOMATICO:
      // Lógica para realizar las pruebas automáticas
      checkInjectorResistance();
      displayFlowRate();
      mostrarResultado();
      detectInjectorClickk();
      pruebaDeAbanico();
      showResults();
      estadoActual = MENU_PRUEBA;
      break;
    default:
      break;
  }
}

void checkInjectorResistance() {
  bool exit = false;
  String resistanceInput = "";
  while (!exit) {
    lcd.clear();
    lcd.print("Midiendo Resistencia");
   
    resistanceInput = readFloatFromVirtualKeypad(); // Leer valor usando el teclado virtual
    
    float testResistance = resistanceInput.toFloat();
    delay(1000); // Simulación de medición
    
    lcd.clear();
    lcd.print("Resultado:");
    lcd.setCursor(0, 1);
    if (testResistance < 12.0 || testResistance > 17.0) {
      lcd.print("Inyector Mal");
    } else {
      lcd.print("Inyector Bien");
      delay(1000);
      estadoActual = MENU_PRUEBA;
      return;
    }
    delay(1000);

    if (digitalRead(botonPin) == LOW) {
      delay(50); // Debounce
      if (digitalRead(botonPin) == LOW) {
        while (digitalRead(botonPin) == LOW) {
          delay(5); // Espera a que se suelte el botón
        }
        exit = true;
        estadoActual = MENU_PRUEBA;
      }
    }
  }
}

String readFloatFromVirtualKeypad() {
  String input = "";
  while (true) {
    mostrarMenu();
    bool estadoBoton = digitalRead(botonPin);
    if (estadoBoton == LOW && estadoAnteriorBoton == HIGH) {
      input += virtualKeypad[indiceVirtualKeypad];
      lcd.setCursor(0, 1);
      lcd.print(input);
      delay(300); // Debounce
      estadoAnteriorBoton = estadoBoton;
    }
    if (estadoBoton == LOW && virtualKeypad[indiceVirtualKeypad] == "#") {
      break;
    }
  }
  return input;
}

void showResults() {
  bool exit = false;
  while (!exit) {
    lcd.clear();
    lcd.print("Mostrando Resultados");
    delay(1000);
    
    if (digitalRead(botonPin) == LOW) {
      delay(50); // Debounce
      if (digitalRead(botonPin) == LOW) {
        while (digitalRead(botonPin) == LOW) {
          delay(10); // Espera a que se suelte el botón
        }
        exit = true;
        estadoActual = SUBMENU_MANUAL;
      }
    }
  }
}

void compareResistances(float baseResistance, float testResistance) {
  lcd.print("Comparando...");
  delay(2000); // Simulación de comparación

  lcd.clear();
  lcd.print("Resultado:");
  lcd.setCursor(0, 1);
  if (baseResistance == testResistance) {
    lcd.print("Iguales");
  } else {
    lcd.print("Diferentes");
  }
}

void storeFloatInEEPROM(float value, int address) {
  EEPROM.put(address, value);
  EEPROM.commit();
}

float readFloatFromEEPROM(int address) {
  float value;
  EEPROM.get(address, value);
  return value;
}

void detectInjectorClickk() {
  bool exit = false;
  while (!exit) {
    int micValue = analogRead(micPinA0);
    lcd.clear();
    Serial.print("Mic Value: ");
    Serial.println(micValue);
    lcd.setCursor(0, 0);
    lcd.print("Mic Value: ");
    lcd.print(micValue);

    if (micValue > threshold) {
      Serial.println("Click detectado!");
      lcd.setCursor(1, 1);
      lcd.print("Click detectado!");
    } else {
      lcd.setCursor(0, 2);
      lcd.print("                "); // Borra la línea
    }
    delay(1000); // Pequeño retardo para evitar rebotes y actualizar con frecuencia

    if (digitalRead(botonPin) == LOW) {
      delay(50); // Debounce
      if (digitalRead(botonPin) == LOW) {
        while (digitalRead(botonPin) == LOW) {
          delay(10); // Espera a que se suelte el botón
        }
        exit = true;
        estadoActual = SUBMENU_MANUAL;
      }
    }
  }
}

void pruebaDeAbanico() {
  bool exit = false;
  while (!exit) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Prueba de Abanico");

    // Lógica para la prueba de abanico
    delay(2000); // Simulación de la prueba

    lcd.setCursor(0, 1);
    lcd.print("Prueba Completada");

    delay(1000); // Pequeño retardo para evitar rebotes y actualizar con frecuencia

    if (digitalRead(botonPin) == LOW) {
      delay(50); // Debounce
      if (digitalRead(botonPin) == LOW) {
        while (digitalRead(botonPin) == LOW) {
          delay(10); // Espera a que se suelte el botón
        }
        exit = true;
        estadoActual = SUBMENU_MANUAL;
      }
    }
  }
}

void displayFlowRate() {
  bool exit = false;
  while (!exit) {
    if ((millis() - oldTime) > 1000) {
      detachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN));
      flowRate = ((1000.0 / (millis() - oldTime)) * pulseCount) / calibrationFactor;
      oldTime = millis();
      flowMillilitres = (flowRate / 60) * 1000;
      pulseCount = 0;
      attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, FALLING);
    }
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Flujo: ");
    lcd.setCursor(0, 1);
    lcd.print(flowRate);
    lcd.print(" L/min");
    delay(1000); // Pequeño retardo para evitar rebotes y actualizar con frecuencia

    if (digitalRead(botonPin) == LOW) {
      delay(50); // Debounce
      if (digitalRead(botonPin) == LOW) {
        while (digitalRead(botonPin) == LOW) {
          delay(10); // Espera a que se suelte el botón
        }
        exit = true;
        estadoActual = SUBMENU_MANUAL;
      }
    }
  }
}

void mostrarResultado() {
  bool exit = false;
  while (!exit) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Flujo (mL/s): ");
    lcd.setCursor(0, 1);
    lcd.print(flowRate);
    
    if (flowRate > 0) {
      lcd.setCursor(0, 2);
      lcd.print("Fuga detectada!");
    } else {
      lcd.setCursor(0, 2);
      lcd.print("No hay fuga");
    }
    delay(1000); // Pequeño retardo para evitar rebotes y actualizar con frecuencia
    
    if (digitalRead(botonPin) == LOW) {
      delay(50); // Debounce
      if (digitalRead(botonPin) == LOW) {
        while (digitalRead(botonPin) == LOW) {
          delay(5); // Espera a que se suelte el botón
        }
        exit = true;
        estadoActual = SUBMENU_MANUAL;
      }
    }
  }
}
