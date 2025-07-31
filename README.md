
# AI Grijperrobot met Arduino & ESP32

## 📌 Overzicht
Dit project beschrijft de bouw en werking van een autonome grijperrobot die gebruikmaakt van AI (camera op ESP32) en Arduino Nano voor hardwaresturing.

## 🔧 Benodigde Hardware
- Arduino Nano
- ESP32-CAM
- Servo Motoren
- IR Sensoren
- Ultrasonic sensor
- Gewichtssensor
- Gripper (eigen ontwerp)
- Wielbasis + motoren
- Breadboard + jumper wires

## ⚙️ Functionaliteiten
- Objectherkenning via ESP32 camera
- Lijnvolging met IR-sensoren
- Object oppakken met servo-gecontroleerde grijper
- Navigatie op grid-systeem
- Gewichtsbepaling voor stabiliteit

## 🧠 Softwarestructuur
De software is opgesplitst in modules:
1. **Vision (ESP32)** → AI/CV
2. **Control (Nano)** → Servo’s, sensoren, motoren
3. **Communicatie** → Serieel tussen ESP32 en Nano

## 🧪 Bekende Problemen & Oplossingen
| Probleem | Oplossing |
|---------|-----------|
| Grijper werkte niet goed | Nieuw ontwerp gebruikt |
| Navigatie foutief | Line tracking + grid systeem |
| ESP32 errors | Overgestapt naar Nano, alleen camera blijft op ESP32 |
| Ground-problemen | GND altijd dubbelchecken |
| Lange code werkte niet | Opgesplitst in testmodules |

## 🛠️ Teamleden
- **Michelle** – Hoofd software / AI-integratie  
- **Martin** – Mechanisch ontwerp  
- **Mi-Ann** – Hardware-assemblage  
- **Dawson** – Presentatie & communicatie  
- **Bicele Lame** – Planning & documentatie

