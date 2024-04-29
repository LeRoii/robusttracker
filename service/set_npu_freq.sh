echo userspace > /sys/class/devfreq/fdab0000.npu/governor

echo 1000000000 > /sys/class/devfreq/fdab0000.npu/userspace/set_freq
