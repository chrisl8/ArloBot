# Use this to reset the USB ports, which I find helpful to get all of my devices working
# Reference:
# http://www.linux.org/threads/resetting-the-usb-subsystem.6256/
# http://davidjb.com/blog/2012/06/restartreset-usb-in-ubuntu-12-04-without-rebooting/
echo "Resetting all USB ports . . ."
for i in $(lspci|grep USB|grep Enhanced|awk '{ print $1}');do echo -n "0000:${i}" > /sys/bus/pci/drivers/ehci-pci/unbind;echo -n "0000:${i}" > /sys/bus/pci/drivers/ehci-pci/bind;done
for i in $(lspci|grep USB|grep EHCI|awk '{ print $1}');do echo -n "0000:${i}" > /sys/bus/pci/drivers/ehci-pci/unbind;echo -n "0000:${i}" > /sys/bus/pci/drivers/ehci-pci/bind;done
for i in $(lspci|grep USB|grep SuperSpeed|awk '{ print $1}');do echo -n "0000:${i}" > /sys/bus/pci/drivers/xhci_hcd/unbind;echo -n "0000:${i}" > /sys/bus/pci/drivers/xhci_hcd/bind;done
for i in $(lspci|grep USB|grep xHCI|awk '{ print $1}');do echo -n "0000:${i}" > /sys/bus/pci/drivers/xhci_hcd/unbind;echo -n "0000:${i}" > /sys/bus/pci/drivers/xhci_hcd/bind;done
echo "All USB ports reset, sleeping for 10 seconds..."
sleep 10
