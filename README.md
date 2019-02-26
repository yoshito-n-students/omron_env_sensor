# omron_env_sensor

## Device
OMRON 2JCIE-BU01 https://www.components.omron.com/product-detail?partNumber=2JCIE-BU

## Dependencies
omron_env_sensor_msgs https://github.com/yoshito-n-students/omron_env_sensor_msgs

## Setup
1. install an udev rule to mount the sensor as `/dev/ttyUSB*`\
`sudo cp /path/to/repo/99-omron-2jcie-bu01.rules /etc/udev/rules.d/`

1. add the current user to the dialout group to permit to access `/dev/ttyUSB*`\
`sudo usermod -aG dialout $USER`

Above commands were tested only on Ubuntu 16.04 and may vary for other environments.

## Published Topics
**data** (omron_env_sensor_msgs/DataShort)

## Parameters
**~device** (string, default: "/dev/ttyUSB0")

**~interval** (int, default: 1000)
* interval between fetching data from the sensor in ms