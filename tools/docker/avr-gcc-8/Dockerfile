FROM quay.io/centos/centos:stream9

# centos extra repositories
RUN yum -y install dnf-plugins-core
RUN dnf -y install epel-release epel-next-release

# copr repositories
RUN dnf -y copr enable rezso/HDL

# fetch updates
RUN dnf -y update

##
## Install your package here
##

# install package
RUN dnf -y install git

# install package
RUN dnf -y install avr-gcc-8 avr-libc git

VOLUME "/opt/src/"
WORKDIR "/opt/src/"
CMD ["/bin/bash"]
