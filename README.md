# ATB-Flow

**Active Thermal-spread Balancing & Flow Optimizer for under floor heating (and cooling)**

`atb-flow` ist eine auf **ESP32-S3 (Tasmota / BerryScript)** basierende, stetige Einzelkreis- und Rücklauftemperaturregelung für Fußbodenheizungs- und -kühlsysteme in Verbindung mit Wärmepumpen (aktuell Luxtronik Steuerung via ModbusTCP).

Anstelle klassischer thermoelektrischer Stellantriebe (2-Punkt-Regelung), die den Volumenstrom der Wärmepumpe abwürgen können, regelt **ATB-Flow** jeden Heizkreis stetig ($0\text{--}100\,\%$) anhand seiner realen Temperaturspreizung ($\Delta T = T_{VL} - T_{RL}$) sowie der Raumtemperatur und versucht dabei den Mindestvolumenstrom ($\dot{V}_{\min}$) der Wärmepumpe einzuhalten.

---

## ⚠ Problem

Klassische Einzelraumregelungen (ERR) arbeiten fast ausschließlich im **Pulsbetrieb (Auf/Zu)**. In modernen Wärmepumpensystemen führt dies zu bekannten Problemen:
* **Hydraulische Instabilität & Takten:** Schließen mehrere Raumthermostate gleichzeitig, bricht der Volumenstrom ein – die Wärmepumpe taktet oder geht auf Hochdruck-/Durchflussstörung.
* **Unzureichender Volumenstrom:** Durch die Taktung werden die Räume nicht warm, obwohl die Wärmepumpe genug Wärme liefern kann.

Der häufig präferierte manuelle thermische Abgleich ohne ERR funktioniert nur für Heizen **oder** Kühlen – beim Wechsel muss neu justiert werden.

Die am Markt erhältlichen Lösungen wie Homematic IP FALMOT-C12 oder Strawa EGO können diese Probleme bestenfalls teilweise lösen (keine direkte Wärmepumpen-/Volumenstrom-Kopplung) und kosten dazu noch ein Vielfaches.

## 🎯 Zielsetzung
**ATB-Flow verfolgt daher vier Kernziele:**
1. **Dynamischer thermischer Abgleich ($\Delta T$-Balancing):** Kontinuierliche Angleichung der Temperaturspreizung aller Heizkreise bei voll geöffneten Topmetern (bzw. eingestellt auf maximal erwünschten Volumenstrom, der Strömungsgeräusche vermeidet).

2. **Stetige Ventilpositionierung ($0\text{--}100\,\%$):** Einsatz leiser, stromsparender $3\,\text{V DC}$-Getriebemotoren (`Sygonix SY365` / `Hysen HY365` / `HmIP-VDMOT`) statt thermoelektrischer Stellantriebe, die kontinuierlich Strom verbrauchen.

3. **Wärmepumpen-Volumenstromgarantie ($\dot{V}_{\min}$):** Übergeordnete Koordination mit der Wärmepumpe (Heizen, Kühlen, Abtauen) und Normalisierung der Ventilstellungen, sodass immer mindestens der wärmeführendste Kreis nahezu vollständig geöffnet bleibt ($\approx 95\,\%$).

4. **Taupunktsicherer Kühlbetrieb:** Inverses Regelverhalten im Kühlmodus mit kontinuierlicher Taupunktberechnung je Raum und Verteiler-Aufstellort auf Basis von BLE-Raumklimasensoren.

---

## 🧠 Funktionskonzept

Das System arbeitet in drei aufeinander aufbauenden Ebenen:

1. **Stellantrieb-Kalibrierung:** Automatische Erkennung von Ventilkontakt (*Touch-Point*) und Endanschlägen über den Motorstrom (`INA219`), um den toten Leerhub herauszurechnen und das Ventil exakt zwischen $0\text{--}100\,\%$ zu positionieren.
2. **$\Delta T$- & Raumregelung:** Kontinuierliche Nachführung der einzelnen Ventile anhand der gemessenen Vor-/Rücklaufspreizung ($\Delta T$) und der Raumtemperatur – im Kühlbetrieb ergänzt um eine automatische Taupunktüberwachung.
3. **Wärmepumpen-Koordinator & Fail-Safe:** Ein Master-Knoten liest die Wärmepumpe via Modbus TCP aus, entdrosselt die Ventile (Führungsraum auf $\approx 95\,\%$) und sichert den Mindestvolumenstrom ($\dot{V}_{\min}$) bei Heizen, Kühlen und Abtauen.

---

## 🛠️ POC Hardware-Überblick

* **Spannungsversorgung:** Zentrale $12\,\text{V DC}$ (SELV) aus dem Heizraum (`MeanWell HDR-60-12`), im Verteiler per `LM2596S` auf $5\,\text{V}$ (Logik) und $3{,}0\text{--}3{,}3\,\text{V}$ (Motoren) gewandelt.
* **Controller:** `ESP32-S3-WROOM-1U-N16R8` mit Tasmota (WLAN & BLE).
* **Aktorik:** `Sygonix SY365` / `HmIP-VDMOT` Stellantriebe, angesteuert über `DRV8833` H-Brücken und überwacht per `INA219` I2C-Stromsensor.
* **Sensorik:** `DS18B20` 1-Wire-Anlegefühler an Vor- und Rückläufen sowie `Xiaomi LYWSD03MMC` BLE-Sensoren für Raumtemperatur und Luftfeuchtigkeit.

Die Hardwarekosten bei 4 HKV und 12 Heizkreisen liegen bei unter 400 €.

---

## 🔧 CLI-Tools (`tools/`)

* **`tools/profile_valve.py`** – Remote Auto-Profiling & 3-Punkt-Ventilkalibrierung über das ESP32-S3 Tasmota-Gateway.
* **`tools/telink_mi_flasher.py`** – Linux-CLI-Port von `pvvx/TelinkMiFlasher.html` (inkl. Xiaomi-Firmware `2.1.1_0159` Mi-Token-Login & `sign_2.1.1_0159c.bin` OTA-Unlock):
  ```bash
  pip install bleak
  # Extrahierte Xiaomi-Cloud-Tokens anzeigen & Sensoren in Reichweite scannen
  python3 tools/telink_mi_flasher.py list-tokens tokens.json --only-lywsd03mmc
  python3 tools/telink_mi_flasher.py scan --tokens-file tokens.json

  # Einzelnen LYWSD03MMC (2.1.1_0159) auf aktuelle pvvx/ATC-Firmware flashen
  python3 tools/telink_mi_flasher.py flash --mac A4:C1:38:XX:XX:XX --tokens-file tokens.json

  # Für neuere 2025er Hardware-Revisionen (B1.5 / B1.6 / B1.1) im 2-Stufen-Modus:
  python3 tools/telink_mi_flasher.py flash --mac A4:C1:38:XX:XX:XX --tokens-file tokens.json --two-stage
  ```

---

## 📄 Weiterführende Dokumentation

Alle mathematischen Formeln, Pinbelegungen, Stücklisten (BOM), der Referenz-Code für das Gateway sowie der detaillierte Umsetzungsplan sind im System-Design-Dokument beschrieben:

* 📘 **[System-Design: FBH-Steuerung (Heizen & Kühlen)](atb_flow_system_design.md)**
* 📐 **[Elektrischer Schaltplan (SVG)](schaltplan_hkv_knoten.svg)**

## 📜 Lizenz

Dieses Projekt steht unter der **GNU Affero General Public License v3.0 (AGPL-3.0)** – siehe [LICENSE](LICENSE).
