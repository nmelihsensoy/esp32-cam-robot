# esp32-cam-robot

## Build & Upload

```
git clone https://github.com/nmelihsensoy/esp32-cam-robot.git
cd esp32-cam-robot

## building vue.js app
cd front-end
yarn install
yarn build

## building filesystem image that contains our vue.js app
pio run --target buildfs --environment esp32cam

## flashing esp32's spiffs partition with the filesystem image
pio run --target uploadfs --environment esp32cam

## building and uploading web server code to the esp32
pio run --target upload --environment esp32cam

```