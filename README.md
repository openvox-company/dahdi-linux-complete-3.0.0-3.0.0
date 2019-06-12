Introduction
==
This project is officially maintained and provides technical support by OpenVox Co. Ltd., and providing a communication platform for users and VoIP open source enthusiasts.

Due to the limitation of Official DAHDI drivers released by Digium, the original DAHDI drivers are incompatible with OpenVox Telephony Cards. This project was established in order to enable the DAHDI driver to support OpenVox products.

Compared to the original DAHDI drivers, we have added or modified the following modules:

#### Digital Cards
###### -wct4xxp:
* OpenVox D230E/P: PCI-Express or PCI dual-port T1/E1/J1
* OpenVox D430E/P: PCI-Express or PCI quad-port T1/E1/J1
* OpenVox D830E/P: PCI-Express or PCI eight-port T1/E1/J1
* OpenVox D1630E/P: PCI-Express or PCI 16-port T1/E1/J1

###### -opvxd115:
* OpenVox D130E/P: PCI-Express or PCI single-port T1/E1/J1

###### -wcb4xxp:
* OpenVox B400E/P: PCI-Express or PCI quad-port BRI  
* OpenVox B200E/P: PCI-Express or PCI dual-port BRI

###### -zaphfc:
* OpenVox B100E/P: PCI-Express or PCI single-port BRI

#### Analog Cards
###### - wctdm:
  * OpenVox A400E/P: PCI-Express or PCI 4 analog ports

###### - opvxa1200:
  * OpenVox A800E/P: PCI-Express or PCI 8 analog ports
  * OpenVox A1200E/P: PCI-Express or PCI 12 analog ports

###### - opvxa24xx:
  * OpenVox A810E/P: PCI-Express or PCI 8 analog ports
  * OpenVox A1610E/P: PCI-Express or PCI 16 analog ports
  * OpenVox A2410E/P: PCI-Express or PCI 24 analog ports

#### Hybrid Cards
###### - opvxx200
  * OpenVox X204E/P: PCI-Express or PCI Hybrid Cards (FXS/FXO/BRI/E1/T1)
***
The DAHDI-3.0.0 driver has tested on the following kernel version:
  * 3.10 (CentOS 7.6)
  * 4.15 (Ubuntu 18.06)
  * 4.20 (Ubuntu 18.06 with the updated kernel)

Since it is only tested in the limited operating system environment, it may be incompatible in some operating systems. When users encounter problems during compilation and installation, they can contact OpenVox official technical service via email support@openvox.cn. We will reply and solve the problem asap, thank you for your cooperation.

Declare
==
The source code of this project is distributed under the terms of the GNU General Public License Version 2, except for some components which are distributed under the terms of the GNU Lesser General Public License Version 2.1. Both licenses are included in this directory, and each file is clearly marked as to which license applies.

The DAHDI trademark is owned by Digium company. All other trademarks involved in this project are the property of their respective owners. OpenVox Inc. will not provide any official technical support service unless you are using OpenVox products with the modules listed above.
