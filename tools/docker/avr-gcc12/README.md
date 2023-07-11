# AVR-GCC 12.x

this is a Dockerfile to build Arch Linux with `avr-gcc` compiler version 12.x

## Create Docker Container

```
docker build -t avr-gcc12 .
```

## Compile code

open terminal in root folder of the project and type:
```
docker run -it --rm -v "$PWD":/opt/src avr-gcc12 bash
[root@f86d24d031d3 src]# ls
boot.c  boot.eep  boot.hex  bootnew.hex  build  burn.sh  crt1.S  devs  docs  libs  LICENSE  main.c  main.hex  make.sh  MODBUS.md  README.md  tools
[root@f86d24d031d3 src]# ./make.sh 
```