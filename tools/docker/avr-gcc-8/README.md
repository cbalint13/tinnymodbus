# AVR-GCC 8.x

To compile the tinymodbus project you have to use an old `avr-gcc` because of space requirements.

This is a Dockerfile using CentOS 9 with `avr-gcc` compiler version 8.x

## Clone the project

```
$ git clone https://github.com/cbalint13/tinnymodbus
$ cd tinnymodbus
```

## Create Docker Container

```
$ docker build -t avr-gcc8 tools/docker/avr-gcc-8/
```

## Compile code

```
$ docker run -it --rm -v "$PWD":/opt/src avr-gcc8 bash
[root@f86d24d031d3 src]# ./make.sh
[root@f86d24d031d3 src]# ls
boot.c  boot.eep  boot.hex  bootnew.hex  build  burn.sh  crt1.S  devs  docs  libs  LICENSE  main.c  main.hex  make.sh  MODBUS.md  README.md  tools
```
