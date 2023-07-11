FROM archlinux:base

## update package cache and install some packages
RUN pacman -Sy && pacman -S --noconfirm sudo git base-devel avr-gcc avr-libc

## need to create build user for makepkg because it cannot run as root..
RUN useradd --no-create-home --shell=/bin/false build && usermod -L build
RUN echo "build ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers
RUN echo "root ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

WORKDIR "/opt/avr"

## get PKGBUILD of downgrade script, build and install it
RUN git clone https://aur.archlinux.org/downgrade.git 
RUN chown build:root downgrade/* downgrade/.*
RUN cd downgrade && sudo -u build makepkg -si --noconfirm

## disable question if package should be added to ignore list and assume yes as answer
RUN sed -i 's/^      eval_gettext "add /      #eval_gettext "add /' /usr/sbin/downgrade
RUN sed -i 's/^      read -r ans/      #read -r ans\n      ans=y/' /usr/sbin/downgrade

## finaly downgrade avr-gcc to 8 branch
RUN downgrade 'avr-gcc=8.3.0' -- --noconfirm

VOLUME "/opt/src"
WORKDIR "/opt/src"
CMD ["/bin/bash"]
