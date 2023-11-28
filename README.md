# PlantCabinet🪴 Controller

## Provision

### QR Code
<img src="docs/assets/provision_qr_code.png" />

SetupQRCode: [MT:Y.K9042C00KA0648G00](https://project-chip.github.io/connectedhomeip/qrcode.html?data=MT%3AY.K9042C00KA0648G00
)

### Manual Pairing
Manual pairing code: __34970112332__    

## Setup

## Generate code
Install zap tool:
https://developer.nordicsemi.com/nRF_Connect_SDK/doc/2.4.2/nrf/protocols/matter/getting_started/tools.html

Run in `/opt/nordic/ncs/v2.4.2/modules/lib/matter`
```bash
python ./scripts/tools/zap/generate.py /Users/philipp.manstein/repos/plant_cabinet_controller/src/plant-cabinet-controller.zap -t src/app/zap-templates/app-templates.json -z src/app/zap-templates/zcl/zcl.json  -o /Users/philipp.manstein/repos/plant_cabinet_controller/src/zap-generated
```

## Build chip-tool
```console
# Init submodules
git submodule update --init

# Build chip-tool
./scripts/examples/gn_build_example.sh examples/chip-tool .out/chip-tool

# Commission with code
./chip-tool pairing code 1 25938105393
```

## chip-tool Commands

### Temperature

#### Read temperature
```console
./chip-tool temperaturemeasurement read measured-value 1 1
```

### Pressure

#### Read pressure
```console
./chip-tool pressuremeasurement read measured-value 1 2
```

### Humidity

#### Read humidity
```console
./chip-tool relativehumiditymeasurement read measured-value 1 3
```

### On/Off cluster (The relay)

#### Read relay state
```console
./chip-tool onoff read on-off 1 4
```
#### Turn relay on
```console
./chip-tool onoff on 1 4
```

#### Turn relay off
```console
./chip-tool onoff off 1 4
```

#### Toggle relay
```console
./chip-tool onoff toggle 1 4
```