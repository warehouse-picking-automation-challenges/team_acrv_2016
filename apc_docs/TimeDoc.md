# Running without internet

Steve has setup the router to both run an NTP time server and forward one.

The time server is called:

```
OpenWrt.lan
```

If the router is already on, Baxter will autonomously sync to the routers time
server at boot.

Other computers on the network must be manually configured to grab their time
from the router instead of whatever other NTP servers they were used to.

To force update time on other computers, do the following:

 * `sudo gedit /etc/ntp.conf`
 * Remove all other time servers
 * Add in: `server OpenWrt.lan prefer`
 * Save and exit
 * `sudo service ntp stop`
 * `sudo ntpd -gq`
 * `sudo service ntp start`

Note: The `-gq` tells the ntp daemon to correct the time regardless of the
offset (`g`) and exit immediately (`q`).

To check that Baxter and your computers are time synced, run:

```
ntpdate -q qutbaxter
```

To check what time server the computer is using, run:

```
ntpdc
peers
```

# Running with internet

So you can't just go add internet after setting up without internet. You need
to reboot everything. Also need to change the computers settings for internet
time sync.

The router will automatically sync to internet time if internet time is
available. The router must be rebooted when going either way between internet
and no internet.

To force update time on other computers, do the following:

 * `sudo gedit /etc/ntp.conf`
 * Remove all other time servers
 * Add in: `server time.qut.edu.au prefer`
 * Save and exit
 * `sudo service ntp stop`
 * `sudo ntpd -gq`
 * `sudo service ntp start`

Note: The `-gq` tells the ntp daemon to correct the time regardless of the
offset (`g`) and exit immediately (`q`).





On routers

ssh root@192.168.1.1
admin

vi /etc/ntp.conf


vi /etc/config/dhcp



# New Findings

I have found some settings that work for both with and without internet. Best
thing to do is reboot everything when changing between internet and no internet.

For some reason when going from no internet to internet, on the non-baxter
computers we need to run:

```
sudo service ntp stop
sudo ntpd -gq
sudo service ntp start
```
