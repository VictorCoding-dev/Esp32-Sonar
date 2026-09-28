# Receptor ESP-NOW do MPU6050

Este firmware recebe em broadcast os pacotes do ESP32-C3 SuperMini no canal Wi-Fi 1 e imprime Roll, Pitch, temperatura e alerta na Serial a 115200 baud.

## Gravar o receptor

Abra um terminal nesta pasta e execute:

```powershell
pio run --target upload
pio device monitor --baud 115200
```

## Gravar o transmissor

No projeto `ESP32SUPERMINI`, execute:

```powershell
pio run --target upload
```

Os dois dispositivos precisam estar ligados ao mesmo tempo. O receptor transmite os dados recebidos na porta serial, que também pode ser selecionada pela GUI do cubo MPU.
