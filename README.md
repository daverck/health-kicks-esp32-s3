# health-kicks-esp32-s3 : Firmware Embarqué Chaussure Connectée

Ce dépôt contient le firmware unifié pour la chaussure connectée **HealthKicks**, conçu pour le microcontrôleur **ESP32-S3-N16R8** (16 MB Flash Quad/Octal, 8 MB PSRAM Octal). Il remplace l'ancienne architecture hybride Raspberry Pi + Arduino Nano.

---

## 1. Câblage Matériel & Broches (ESP32-S3-N16R8)

### A. Avertissement Critique : Mémoire Octal SPI (N16R8)
> [!CAUTION]
> Sur les modules ESP32-S3 équipés de PSRAM et Flash Octale (N16R8), les broches **GPIO 33, 34, 35, 36 et 37** sont directement câblées en interne au bus mémoire Octal SPI.
> **IL EST FORMELLEMENT INTERDIT D'UTILISER CES BROCHES EN TANT QUE GPIO GÉNÉRAUX.** Tout raccordement sur ces broches corrompt la mémoire et cause un arrêt immédiat du CPU.

### B. Tableau Synthétique des Broches

| Composant | Broche Composant | Broche ESP32-S3 | Type / Fonction | Remarques |
| :--- | :--- | :--- | :--- | :--- |
| **MPU-6050 (IMU)** | VCC | 3V3 | Alimentation | Régulée 3.3V |
| **MPU-6050 (IMU)** | GND | GND | Masse | Masse commune |
| **MPU-6050 (IMU)** | **SDA** | **GPIO 4** | I2C Data (RTC IO 4) | Pull-up 4.7 kΩ vers 3.3V |
| **MPU-6050 (IMU)** | **SCL** | **GPIO 5** | I2C Clock (RTC IO 5) | I2C Fast Mode 400 kHz, pull-up 4.7 kΩ |
| **MPU-6050 (IMU)** | AD0 | GND | Adresse I2C | Fixe l'adresse à `0x68` |
| **MPU-6050 (IMU)** | INT | **GPIO 6** | GPIO Interrupt | Optionnel (Wake-on-motion) |
| **Vibreur Haptique** | Gate (MOSFET) | **GPIO 7** | LEDC PWM (0-255) | Fréquence 10 kHz, Timer 0 |
| **Bouton Appairage** | Contact | **GPIO 14** | RTC IO 14 (Input) | Pull-up interne, actif bas (GND), réveil `ext1` |
| **LED RGB Statut** | Data In | **GPIO 48** | RMT / WS2812 | LED RGB embarquée sur DevKitC-1 |
| **Mesure Batterie** | Diviseur $V_{bat}$ | **GPIO 10** | ADC1_CH9 | Diviseur 2x 100 kΩ ($V_{bat}/2$) |
| **Octal Flash/PSRAM**| - | **GPIO 33 à 37** | **INTERDIT** | **Dédié Flash & PSRAM Octal SPI** |
| **Strapping Pins** | - | **GPIO 0, 45, 46** | Boot Strapping | Ne pas connecter de charge externe |
| **USB Natif CDC** | USB D- / D+ | **GPIO 19, 20** | USB CDC/JTAG | Moniteur série & flash |

### C. Recommandation Transistor Vibreur (MOSFET N-Channel)
Pour garantir la pleine saturation du transistor avec le niveau logique 3.3V de l'ESP32-S3, utiliser un MOSFET N-Channel à très bas seuil de déclenchement (*Logic-Level* $V_{gs(th)} < 1.8\text{V}$, idéalement **AO3400** ou **2N7002** faible $V_{gs(th)}$) :
- **Gate** : Reliée à **GPIO 7** via une résistance de 100 Ω.
- **Pull-down Gate** : Résistance de 100 kΩ entre Gate et GND pour éviter les vibrations intempestives au boot.
- **Diode de roue libre** : Diode de redressement rapide (1N4148 ou Schottky BAT43 / SS14) en parallèle inverse aux bornes du moteur.

---

## 2. Compilation & Flash avec PlatformIO

Ce projet utilise [PlatformIO](https://platformio.org/).

### Commandes de base :
```bash
# Compilation du firmware
pio run

# Flash sur l'ESP32-S3
pio run --target upload

# Ouverture du moniteur série USB CDC (115200 bauds)
pio run --target monitor
```

---

## 3. Conformité aux Contrats GATT BLE

Le serveur BLE implémente les spécifications contractuelles définies dans `contracts/ble_gatt_specs.md` :
- **Service Footwear** : `7a5a0001-c529-4d64-8848-18e5904de22a`
- **Activity Detection** : `7a5a0002-...` (READ, NOTIFY - 7 octets Big-Endian)
- **Haptic Command** : `7a5a0003-...` (WRITE, WRITE_NR - 4 octets Big-Endian)
- **Studio Control** : `7a5a0004-...` (WRITE, NOTIFY - ASCII UTF-8)
- **Studio Data Burst** : `7a5a0005-...` (NOTIFY - Paquets MTU-adaptés avec CRC32)

