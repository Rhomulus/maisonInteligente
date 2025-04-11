#include <AccelStepper.h>
#include <HCSR04.h>
#include <Wire.h>
#include <LCD_I2C.h>

#define MOTOR_INTERFACE_TYPE 4
#define IN_1 31
#define IN_2 33
#define IN_3 35
#define IN_4 37

LCD_I2C lcd(0x27, 16, 2);

float mesure;
const int redPin = 9;    // rouge
const int bluePin = 11;  // bleu
const int buzzerPin = 8;

enum ETAT { FERMER,
            TRANSITION,
            OUVERT,
            ALARME };
ETAT etat = FERMER;

HCSR04 hc(5, 6);
AccelStepper myStepper(MOTOR_INTERFACE_TYPE, IN_1, IN_3, IN_2, IN_4);

const int positionInitiale = 0;
const int positionFinale = 2038;

unsigned long dernierAffichage = 0;
unsigned long lastTime = 0;
unsigned long debutAlarme = 0;
unsigned long dernierSon = 0;
bool buzzerEtat = false;

// Fonction RGB (rouge, bleu, blanc)
void setCouleur(int r, int b, int blanc) {
  analogWrite(redPin, r);   // Rouge
  analogWrite(bluePin, b);  // Bleu
  analogWrite(bluePin, blanc);
}

void debut() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("6225272");
  lcd.setCursor(0, 1);
  lcd.print("Labo 5");
  delay(2000);
  lcd.clear();
}

void setup() {
  Serial.begin(9600);
  lcd.backlight();
  lcd.begin();
  debut();

  myStepper.setMaxSpeed(700);
  myStepper.setAcceleration(200);
  myStepper.setSpeed(300);
  myStepper.setCurrentPosition(positionInitiale);

  pinMode(redPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
}

float distance() {
  return hc.dist();
}

void affichage(float dist, int positionActuelle, bool enTransition) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Dist : ");
  lcd.print(dist, 0);
  lcd.print(" cm");

  lcd.setCursor(0, 1);
  lcd.print("Porte: ");

  if (enTransition) {
    lcd.print(positionActuelle);
    lcd.print(" deg");
  } else {
    if (etat == OUVERT) {
      lcd.print("Ouverte");
    } else if (etat == FERMER) {
      lcd.print("Fermee");
    } else {
      lcd.print("RHOMU");
    }
  }
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - lastTime >= 50) {
    mesure = distance();
    lastTime = currentTime;
  }

  int positionActuelle = myStepper.currentPosition();
  int angle = map(positionActuelle, positionInitiale, positionFinale, 10, 170);

  switch (etat) {
    case FERMER:
      if (mesure < 15) {
        etat = ALARME;
        debutAlarme = currentTime;
      } else if (mesure < 30) {
        etat = TRANSITION;
        myStepper.moveTo(positionFinale);
      }
      break;

    case OUVERT:
      if (mesure < 15) {
        etat = ALARME;
        debutAlarme = currentTime;
        if (mesure > 60) {
          etat = TRANSITION;
          myStepper.moveTo(positionInitiale);
        }
      }

      break;

    case ALARME:
      digitalWrite(buzzerPin, HIGH);

      if (currentTime - dernierSon >= 250) {
        buzzerEtat = !buzzerEtat;

        if (buzzerEtat) {
          setCouleur(255, 0, 0);  // Rouge
        } else {
          setCouleur(0, 0, 255);  // Bleu
        }

        dernierSon = currentTime;
      }

      if (mesure < 15) {
        debutAlarme = currentTime;
      }

      if (currentTime - debutAlarme >= 3000 && mesure >= 15) {
        digitalWrite(buzzerPin, LOW);
        setCouleur(0, 0, 0);  // Éteindre RGB
        etat = FERMER;
      }
      break;

    case TRANSITION:
      myStepper.run();
      if (myStepper.distanceToGo() == 0) {
        if (positionActuelle >= positionFinale - 5) {
          etat = OUVERT;
        } else if (positionActuelle <= positionInitiale + 5) {
          etat = FERMER;
        }
      }
      break;
  }

  if (currentTime - dernierAffichage >= 100) {
    affichage(mesure, angle, etat == TRANSITION);
    dernierAffichage = currentTime;
  }
}
