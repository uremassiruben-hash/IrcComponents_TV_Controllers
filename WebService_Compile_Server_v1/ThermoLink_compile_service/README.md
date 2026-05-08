# ThermoLink Compile Service

Questo è il web service che permette all'app Android di compilare un file `.ino` senza usare il PC come IDE.

Flusso:

```text
App Android editor .ino
↓ POST /compile
Server compila con arduino-cli
↓ restituisce firmware.bin
App invia firmware.bin all'ESP32 via OTA /update
```

## Avvio locale con Docker

Nella cartella del servizio:

```bash
docker compose up --build
```

oppure:

```bash
docker build -t thermolink-compile-service .
docker run --rm -p 5000:5000 thermolink-compile-service
```

## URL da mettere nell'app

Se il server gira sul tuo PC nella stessa rete del tablet:

```text
http://IP_DEL_PC:5000/compile
```

Esempio:

```text
http://192.168.1.20:5000/compile
```

## Test

```bash
curl http://IP_DEL_PC:5000/health
```

Deve rispondere con JSON `ok: true`.

## Test compilazione manuale

```bash
curl -X POST http://IP_DEL_PC:5000/compile \
  -F "code=@TestPinWifi__NTC_Bella8_OTA_EDITOR_READY.ino" \
  --output firmware.bin
```

## Librerie già installate nel Docker

- ESP32 Arduino core
- WiFiManager
- PID

Se in futuro aggiungi librerie Arduino nuove nel firmware, devi aggiungerle anche nel `Dockerfile`, per esempio:

```dockerfile
RUN arduino-cli lib install NomeLibreria
```

## Sicurezza

In locale puoi lasciare `THERMOLINK_COMPILE_TOKEN` vuoto.

Su un server online non lasciarlo vuoto. Imposta una password/token e poi bisogna farla inviare anche dall'app Android.

Per una versione pubblica seria servono anche:

- HTTPS
- autenticazione
- limiti dimensione codice
- timeout compilazione
- container isolato
