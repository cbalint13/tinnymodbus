FROM archlinux:base

## update package cache and install some packages
RUN pacman -Sy && pacman -S --noconfirm sudo git base-devel avr-gcc avr-libc

VOLUME "/opt/src"
WORKDIR "/opt/src"
CMD ["/bin/bash"]