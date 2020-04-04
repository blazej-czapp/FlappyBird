Inspired by [this](https://www.youtube.com/watch?v=i1_X2KdUQe8)

# Setup
On Ubuntu, install the following:

    * libusb-dev
    * libudev-dev
    * libopencv-dev
    
Unpack and install `libk8055.0.4.1`.

## Board permissions

[link](https://www.robert-arnold.de/cms/en/2010/10/zugriff-fur-nicht-root-user-auf-usb-board-k8055-unter-ubuntu-9-10-erlauben/)

To grant yourself `udev` rights to interact with the Velleman K8055 board, first run `lsusb`, you should see something like:

```
Bus 001 Device 005: ID 10cf:5503 Velleman Components, Inc. 8055 Experiment Interface Board (address=3)
``` 

The interesting bit is `10cf:5503` - first group is vendor ID, second is product ID. Create file `/etc/udev/rules.d/my_k8055_1.rules` with the following content:

```
ATTRS{idVendor}=="10cf", ATTRS{idProduct}=="5503", SYMLINK+="k8055", OWNER="root", GROUP="k8055grp", MODE="660"
```

Substitute vendor and product ID as appropriate.

Now, create the group as per the rule file:
 
```
addgroup k8055grp
```

add yourself to it:

```
sudo usermod -a -G k8055grp $USER
```

Unplug the device, log out, reconnect and you should be good to go.