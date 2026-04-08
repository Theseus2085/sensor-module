# Wissenschaftlich-technische Dokumentation
## Filamentdurchmesser-Sensormodul (STM32F446RE, mbed OS)

## 1. Ziel und Systemgrenze
Dieses Firmware-Modul misst den Filamentdurchmesser in zwei orthogonalen Messachsen, bereitet die Messwerte digital auf und stellt sie einem externen Host (z. B. Drucker-Firmware) ueber I2C als Slave bereit.

Systemgrenze dieser Dokumentation:
- Implementierung in `src/main.cpp`
- Build- und Laufzeitkonfiguration aus `platformio.ini`
- Keine externe Persistenz, kein Dateisystem, keine Cloud- oder Netzwerkdienste

## 2. Hardware- und Softwarekontext
### 2.1 Hardware
- MCU: STM32F446RE (NUCLEO-F446RE)
- Sensorik: 2x Hall-Sensor an ADC-Eingaengen
  - Sensor 1: `PA_0` (`ADC1_IN0`)
  - Sensor 2: `PA_1` (`ADC1_IN1`)
- I2C-Slave:
  - SDA: `PB_9`
  - SCL: `PB_8`
  - Adresse (8-bit): `0x84` (entspricht 7-bit `0x42`)
- Bedienelemente:
  - Kalibrierstart: `PB_6` / Arduino D10 (Pull-up)
  - Kalibrierschritt: `PA_9` / Arduino D8 (Pull-up)
  - Logging-Toggle: `PB_5` / Arduino D4 (Pull-up)
- UART-Bridge zum Arduino Mega 2560:
  - TX: `PB_10` / Arduino D6, 115200 Baud
- Statusausgabe:
  - LED: `PA_5`
  - UART/USB-Serial: 115200 Baud

### 2.2 Software
- Framework: mbed OS (PlatformIO)
- Wichtige Build-Umgebungen:
  - `env:nucleo_f446re` (normal)
  - `env:nucleo_f446re_dbg` (Debug-Flags fuer I2C-Instrumentierung)

## 3. Laufzeitarchitektur (Nebenlaeufigkeit)
Die Firmware nutzt vier Ausfuehrungskontexte:

1. Main-Thread (`main`, Default-Prioritaet)
- Zyklische Messung und Signalverarbeitung
- Aktualisierung des I2C-Nutzdatenpuffers
- Abfrage und Durchfuehrung der Kalibrierung

2. I2C-Slave-Thread (`i2c_slave_thread`, `osPriorityRealtime`)
- Behandelt I2C-Busereignisse (`receive`)
- Liefert bei Host-Read den letzten konsistenten 10-Byte-Datensatz

3. LED-Thread (`led_heartbeat_thread`, `osPriorityNormal`)
- Reine Lebendanzeige (Heartbeat), funktional von der Messkette entkoppelt

4. UART-Stream-Thread (`uart_stream_thread`, `osPriorityNormal`)
- Tastergesteuert (`PB_5` / D4): Pro Tastendruck wird genau ein Messpunkt als CSV-Zeile ausgegeben
- Format: `index,s1_mm,s2_mm,s1_raw2,s2_raw2` (burst-gemittelt + 2. Rohwert fuer Rauschvergleich)
- Ausgabe ueber Hardware-UART `PB_10` / D6 an externe Arduino-Mega-Bridge

Fachbegriff:
- "Nebenlaeufigkeit" bedeutet, dass mehrere logische Aufgaben zeitlich ueberlappend ausgefuehrt werden. Hier wird das durch mbed-Threads umgesetzt.

## 4. Wo wird ein Messevent ausgeloest?
Ein Messevent (neue Sensordatenerfassung) entsteht an drei Stellen:

1. Initialisierung (`main`, einmalig)
- Bei Systemstart wird `measure_sensor_values()` einmal aufgerufen.

2. Zyklischer Betrieb (`main`-Schleife)
- In jeder Schleifeniteration wird (bei `TEST_MODE == 0`) `measure_sensor_values()` aufgerufen.
- Danach folgt `sleep_for(2ms)`.
- Resultat: nominale Mess- und Aufbereitungsperiode ca. 2 ms plus Rechenzeit.

3. Waehrend Kalibrier-Wartephasen
- In der Wartephase auf den `NEXT`-Taster wird alle 10 ms weiterhin `measure_sensor_values()` aufgerufen.

Wichtig:
- Das I2C-Leseereignis ist kein Messevent. Es sendet nur den zuletzt berechneten Zustand aus dem TX-Puffer.

## 5. End-to-End-Datenfluss (Input -> Verarbeitung -> Output)
### 5.1 Eingaben
- Analoge Sensorspannungen (2 Kanaele)
- Tastersignale fuer Kalibrierung
- I2C-Master-Transaktionen (Read/Write/General Call)

### 5.2 Verarbeitungskette
1. ADC-Lesung mit Burst-Oversampling (32 Samples pro Sensor, 2. Rohwert in `burst_second_sample` gespeichert)
2. Linearisierung in mm ueber stueckweise lineare Interpolation (3 Kalibrierpunkte)
3. Skalierung in Fixed-Point (`mm * 10000`)
4. Formatierung in 5 dezimale Ziffern je Sensor
5. Atomare Uebernahme in globalen TX-Puffer

Hinweis zur Glaettung:
- Bis FW 0.5.x wurde zusaetzlich ein 64-Sample-Ringpuffer (gleitender Mittelwert) im Sensormodul betrieben.
- Ab FW 0.6.0 entfaellt diese sensormodul-seitige Glaettung. Die Mittelung erfolgt stattdessen in der Druckerfirmware (Marlin) ueber ein Lookahead-Averaging-Verfahren: Fuer jeden neuen GCode-Block wird das Integral aller Messwerte gebildet, die innerhalb des zugehoerigen Extrusionsabschnitts anfallen. Damit ist die Glaettung exakt an die Planungsgranularitaet des Druckers gekoppelt und nicht mehr an eine willkuerliche Pufferlaenge im Sensor.

### 5.3 Ausgaben
- I2C-Payload: 10 Bytes
  - Bytes 0..4: Sensor 1, Ziffernformat `d4 d3 d2 d1 d0`
  - Bytes 5..9: Sensor 2, Ziffernformat `d4 d3 d2 d1 d0`
- Serielle Logausgaben (Diagnose)
- LED-Blinksignal

Hinweis zum Datenformat:
- Es werden Ziffernwerte `0..9` als Bytewerte gesendet, keine ASCII-Zeichen.

## 6. Signalverarbeitung im Detail
### 6.1 ADC-Burst-Oversampling
Pro Sensorkanal:
- 32 Einzelmessungen mit 12-bit-Skalierung (`read() * 4095`)
- Mittelung innerhalb des Bursts
- Der zweite Rohwert des Bursts (Index 1) wird in `burst_second_sample[sensor_idx]` gespeichert und dem UART-Stream-Thread zur Rauschanalyse bereitgestellt

Interpretation:
- FIR-Filter mit rechteckigem Fenster der Laenge 32: Rauschunterdrueckung ca. `10 * log10(32) ≈ 15 dB`
- Erhoehte Fensterglaettung genueber frueherer 16-Sample-Variante (~12 dB)

### 6.2 Ringpuffer-Mittelung (entfernt ab FW 0.6.0)
In frueheren Firmware-Versionen (bis 0.5.x) wurde ein 64-Sample-Ringpuffer je Sensor als gleitender Mittelwert (FIR-Filter mit rechteckigem Fenster) eingesetzt, um hochfrequente Schwankungen zu glaetten.

Grund der Entfernung:
- Die sensormodul-seitige Mittelung ueber ein festes Zeitfenster (~128 ms bei 2 ms Messperiode) war nicht an die tatsaechliche Planungsgranularitaet des Druckers gekoppelt.
- In der neuen Architektur uebernimmt die Druckerfirmware (Marlin) die Glaettung: Fuer jeden GCode-Extrusionsblock wird per Lookahead Averaging das Integral ueber alle im Planungsabschnitt anfallenden Sensorwerte gebildet.
- Dadurch entspricht das Mittelungsfenster exakt der jeweiligen Extrusionslaenge, statt einem willkuerlich gewaehlten Sample-Fenster im Sensor.
- Das Sensormodul liefert nun ungefilterte, burst-gemittelte Einzelmesswerte mit maximaler zeitlicher Aufloesung an die Druckerfirmware.

Ebenfalls entfernt: Die Variablen `last_valid_raw1/2` ("Spike Rejection"), die im bisherigen Code keine tatsaechliche Ausreisserlogik implementierten.

## 7. Ermittlung des Durchmessers
Die Umrechnung `raw_adc -> diameter_mm` erfolgt je Sensor ueber drei Kalibrierpunkte:

- Punkt 0: `(raw0, d0)`
- Punkt 1: `(raw1, d1)`
- Punkt 2: `(raw2, d2)`

Standardwerte zu Programmstart (sensorspezifisch kalibriert):
- Sensor 1: `raw`: `401`, `656`, `867` / `d`: `1.65`, `1.75`, `1.85` mm
- Sensor 2: `raw`: `703`, `900`, `1063` / `d`: `1.65`, `1.75`, `1.85` mm

Kalibriermodus ersetzt diese Werte durch neu erfasste ADC-Referenzen bei Soll-Durchmessern `1.65`, `1.75`, `1.85` mm.

### 7.1 Stueckweise lineare Interpolation
Fall A (`raw <= raw1`):

`d = d0 + (d1 - d0) * (raw - raw0) / (raw1 - raw0)`

Fall B (`raw > raw1`):

`d = d1 + (d2 - d1) * (raw - raw1) / (raw2 - raw1)`

Numerische Schutzmassnahme:
- Bei Nenner `0` wird auf den linken Punkt des Segments zurueckgefallen.

Wichtige Eigenschaft:
- Es wird nicht auf `[raw0, raw2]` geclamped. Bei Unter- oder Ueberschreitung erfolgt lineare Extrapolation.

## 8. Kalibrierprozess (Bedienlogik)
Die Kalibrierung wird durch den Taster an `PB_6` / Arduino D10 (Start) initiiert und durchlaeuft fuer beide Sensoren jeweils drei Referenzpunkte bei den Soll-Durchmessern `1.65`, `1.75` und `1.85` mm.

Ablauf:
1. Setze Ausgangswerte zunaechst auf sicheren Nominalwert `1.75 mm`
2. Fuer Sensor 1 und 2 jeweils drei Referenzpunkte durchlaufen
3. Benutzer drueckt `NEXT`, anschliessend wird eine tiefe zeitliche Mittelung (32 Burst-Ablesungen ueber 160 ms) durchgefuehrt und der gemittelte ADC-Rohwert mit dem Soll-Durchmesser gepaart
4. Nach jedem Schritt wird auf Tasterfreigabe gewartet (Debounce enthalten)
5. Nach Abschluss aller sechs Punkte werden die Kalibrierdaten persistent im internen Flash gespeichert

### 8.1 Persistente Speicherung (KVStore / TDB_INTERNAL)
- Die Firmware nutzt das mbed-OS KVStore-API mit dem Backend `TDB_INTERNAL` (TinyDB auf internem Flash).
- Der KV-Schluessel lautet `/kv/cal_tables`; gespeichert wird die gesamte Kalibrierungstabelle beider Sensoren als zusammenhaengender binaerer Block (`sizeof(calibration_tables)` = 24 Bytes: `2 x 3 x (uint16_t + float)`).
- Nach einem Reset oder Power-Cycle laedt die Funktion `load_calibration_from_flash()` beim Systemstart automatisch die zuletzt gespeicherten Kalibrierdaten. Ist kein gespeicherter Datensatz vorhanden (z. B. beim allerersten Programmstart oder nach einem vollstaendigen Re-Flash), werden die kompilierten Standardwerte beibehalten.
- Die Konfiguration erfolgt in `mbed_app.json`: `"storage.storage_type": "TDB_INTERNAL"`.
- Beim Systemstart gibt `load_calibration_from_flash()` die aktive Kalibrierungstabelle in syntaktisch korrekter C-Form ueber UART aus, sodass gemessene Kalibrierwerte direkt als Standardwerte in den Quellcode uebernommen werden koennen (Hardcode-Fallback gegen Flash-Datenverlust).

## 9. I2C-Kommunikationsvertrag
### 9.1 Busparameter
- Fast Mode: 400 kHz
- Slaveadresse: `0x84` (8-bit), entsprechend `0x42` (7-bit)

### 9.2 Reaktionsverhalten des Slave-Threads
- `NoData`: kurzer Sleep (`1 ms`) zur CPU-Entlastung
- `WriteGeneral` / `WriteAddressed`: 1 Byte best effort lesen und ignorieren
- `ReadAddressed`: 10-Byte-Payload schreiben

Fehlerpfad:
- Wenn `i2c_slave.write(...) != 0`, wird der Slave neu initialisiert (`stop`, `frequency`, `address`).

### 9.3 Datenkonsistenz zwischen Threads
Problemstellung:
- Main-Thread aktualisiert den TX-Puffer, I2C-Thread liest ihn asynchron.

Loesung:
- Main-Thread formatiert zuerst in lokalen `temp_buf[10]`.
- Danach kritischer Abschnitt mit `__disable_irq()` / `memcpy()` / `__enable_irq()`.

Fachbegriff:
- "Atomare Uebernahme" bedeutet hier: der globale Puffer wird als konsistentes Ganzes aktualisiert, damit keine "halb alten, halb neuen" Frames ausgesendet werden.

## 10. Zeitverhalten und deterministische Aspekte
- Hauptschleife: Zielperiode ca. 2 ms
- I2C-Dienst: Realtime-Thread, polling-basiert ueber `receive()`
- LED-Thread: 200 ms Ein / 200 ms Aus

Praktischer Effekt:
- Die Messwertaufbereitung laeuft kontinuierlich im Main-Thread.
- Die I2C-Ausgabe ist eventgetrieben durch den Host-Read, liefert aber stets den zuletzt stabil berechneten Frame.

## 11. Eingabe-/Ausgabeuebersicht als Schnittstellenvertrag
### 11.1 Funktionsorientierte Sicht
- `read_sensor_raw_adc(sensor_idx)`
  - Input: Sensorkanalindex (`0` oder `1`)
  - Output: 32-sample burst-gemittelter Roh-ADC-Wert (`uint16_t`); Nebeneffekt: `burst_second_sample[sensor_idx]` wird mit dem 2. Burst-Rohwert befuellt

- `measure_sensor_values()`
  - Input: implizit aktuelle ADC-Samples
  - Output: aktualisiert globale `sensor1_mm`, `sensor2_mm`

- `convert_raw_adc_to_mm(raw_adc, sensor_idx)`
  - Input: Rohwert + Sensorindex
  - Output: Durchmesser in mm (`float`)

- `mm_to_fixed_10000(val)`
  - Input: mm als `float`
  - Output: skaliertes Integerformat (`uint32_t`) mit 4 Nachkommastellen

- `format_sensor_data_fixed(val_x10000, buf)`
  - Input: skaliertes Integer + Zielpuffer
  - Output: 5 Ziffern im Buffer

- `i2c_slave_thread()`
  - Input: I2C-Ereignisse des Busses
  - Output: 10-Byte-Antwortframe auf Read-Requests

- `uart_stream_thread()`
  - Input: Tasterdruck `PB_5` / D4
  - Output: eine CSV-Zeile pro Tastendruck ueber `PB_10` / D6 UART an Mega-Bridge
  - Format: `index,s1_avg_mm,s2_avg_mm,s1_raw2_mm,s2_raw2_mm`

## 12. Bekannte Grenzen und technische Risiken
1. Abhaengigkeit vom internen Flash
- Die persistente Kalibrierung nutzt den internen Flash-Speicher des STM32F446RE ueber das KVStore-API. Ein vollstaendiges Re-Flash der Firmware loescht den KVStore-Bereich und setzt die Kalibrierdaten auf die kompilierten Standardwerte zurueck.

2. Keine sensormodul-seitige Glaettung oder Ausreisserlogik
- Das Sensormodul liefert ungefilterte Einzelmesswerte (nur Burst-Oversampling).
- Glaettung und Plausibilitaetspruefung muessen in der Druckerfirmware erfolgen.

3. Extrapolation ausserhalb Kalibrierbereich
- Kann bei extremen Rohwerten zu unplausiblen Durchmesserwerten fuehren.

4. Genauigkeit der Zeitbasis
- Effektive Messperiode haengt von Thread-Scheduling und Last ab.

## 13. Quellcode-Mapping (fuer Review und Nachvollzug)
Referenzdatei: `src/main.cpp` (FW 0.6.0, 530 Zeilen)

- Pinning, globale Konfiguration, Datenstrukturen: Zeilen 26-113
- Forward-Deklarationen: Zeilen 114-123
- Kalibrierung laden (`load_calibration_from_flash`): Zeilen 128-155
- ADC-Lesung und Durchmesserumrechnung: Zeilen 157-213
- Kalibrierlogik (`calibration`): Zeilen 220-275
- Datenformatierung und I2C-Reinit: Zeilen 281-310
- I2C-Slave-Thread: Zeilen 316-358
- LED-Heartbeat-Thread: Zeilen 364-372
- UART-Stream-Thread: Zeilen 378-429
- Systemstart und zyklischer Betrieb (`main`): Zeilen 435-529

## 14. Zusammenfassung
Das Modul implementiert eine klar getrennte Mess-, Aufbereitungs- und Kommunikationskette:
- robuste Sensordatenerfassung durch 32-faches Burst-Oversampling (~15 dB Rauschunterdrueckung; Glaettung erfolgt in der Druckerfirmware via Lookahead Averaging),
- lineare, kalibrierbare Abbildung von ADC-Rohwerten auf physikalische Durchmesser (sensorspezifische Kalibrierungstabellen, persistent im internen Flash),
- nebenlaeufige, zeitnahe Bereitstellung konsistenter Messframes ueber I2C,
- tastergesteuerte UART-Telemetrie mit Rauschvergleich (burst-gemittelt vs. 2. Burst-Rohwert).

Damit ist die Firmware fuer den Einsatz als externes Filamentdurchmesser-Messsystem geeignet, solange die genannten Grenzen (fehlende harte Ausreisserbehandlung, Extrapolation) systemseitig beruecksichtigt werden.
