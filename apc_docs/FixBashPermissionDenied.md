baxter - http://bob:11311] apc@apcCUDAQUT:~/co/apc_ws$ roslaunch baxter_per[rospack] Warning: error while crawling /home/apc: boost::filesystem::status: Permission denied: "/home/apc/.gvfs"
[rospack] Warning: error while crawling /home/apc: boost::filesystem::status: Permission denied: "/home/apc/.gvfs"
[rospack] Warning: error while crawling /home/apc: boost::filesystem::status: Permission denied: "/home/apc/.gvfs"
[rospack] Warning: error while crawling /home/apc: boost::filesystem::status: Permission denied: "/home/apc/.gvfs"
^C
[baxter - http://bob:11311] apc@apcCUDAQUT:~/co/apc_ws$ sudo bash
[sudo] password for apc:
root@apcCUDAQUT:~/co/apc_ws# umount /
/                          /run                       /sys/fs/cgroup/cpu         /sys/fs/cgroup/perf_event
/boot/efi                  /run/lock                  /sys/fs/cgroup/cpuacct     /sys/fs/cgroup/systemd
/dev                       /run/shm                   /sys/fs/cgroup/cpuset      /sys/fs/fuse/connections
/dev/pts                   /run/user                  /sys/fs/cgroup/devices     /sys/fs/pstore
/dev/sda1                  /run/user/1000/gvfs        /sys/fs/cgroup/freezer     /sys/kernel/debug
/dev/sda2                  /sys                       /sys/fs/cgroup/hugetlb     /sys/kernel/security
/home/apc/.gvfs            /sys/firmware/efi/efivars  /sys/fs/cgroup/memory      /var/lib/docker/aufs
/proc                      /sys/fs/cgroup             /sys/fs/cgroup/net_cls
/proc/sys/fs/binfmt_misc   /sys/fs/cgroup/blkio       /sys/fs/cgroup/net_prio
root@apcCUDAQUT:~/co/apc_ws# umount /home/apc/.gvfs
root@apcCUDAQUT:~/co/apc_ws# exit
[baxter - http://bob:11311] apc@apcCUDAQUT:~/co/apc_ws$
