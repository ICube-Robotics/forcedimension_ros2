#!/bin/bash
#findusbdev.sh

# from https://unix.stackexchange.com/questions/144029/command-to-determine-ports-of-a-device-like-dev-ttyusb0

if [[ "$1" =~ ^(-h|--help)$ ]]; then

echo "Find which USB devices are associated with which /dev/ nodes
Usage:
  $0 [-h|--help] [searchString]

  -h | --help   Prints this message
  searchString  Print only /dev/<device> of matching output
                With no arguments $0 prints information for all
                possible USB device nodes

E.g. $0 \"FTDI_FT232\" - will show /dev/ttyUSBX for a device using
the FTDI FT232 chipset.
"
    exit 0
fi

devs=$( (
for sysdevpath in $(find /sys/bus/usb/devices/usb*/ -name dev ); do
    # ( to launch a subshell here
    (
        syspath="${sysdevpath%/dev}"
        devname="$(udevadm info -q name -p $syspath)"
        [[ "$devname" == "bus/"* ]] && exit
        eval "$(udevadm info -q property --export -p $syspath)"
        [[ -z "$ID_SERIAL" ]] && exit
        echo "/dev/$devname - $ID_SERIAL"
    )& # & here is causing all of these queries to run simultaneously
done
# wait then gives a chance for all of the iterations to complete
wait
# output order is random due to multiprocessing so sort results
) | sort )


if [ -z "$1" ]; then
    echo "${devs}"
else
    echo "${devs}" | grep "$1" | awk '{print $1}'
fi