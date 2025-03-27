# P4-Group-460
Git repo for our 4 semester project

# How to run:
1. Run the setup bash by:
    bash setup.sh
2. If using CrazyRadio for the first time, give it USB permissions by running:
    sudo groupadd plugdev
    sudo usermod -a -G plugdev $USER
then close the terminal, open another and run:
    cat <<EOF | sudo tee /etc/udev/rules.d/99-bitcraze.rules > /dev/null
    # Crazyradio (normal operation)
    SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="7777", MODE="0664", GROUP="plugdev"
    # Bootloader
    SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="0101", MODE="0664", GROUP="plugdev"
    # Crazyflie (over USB)
    SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0664", GROUP="plugdev"
    EOF
