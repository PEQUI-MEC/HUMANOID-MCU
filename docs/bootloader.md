# Bootloader

## STM32F103

O bootloader utilizado para stm32f103 é disponibilizado no repositório [rogerclarkmelbourne/STM32duino-bootloader](https://github.com/rogerclarkmelbourne/STM32duino-bootloader), onde tem o código fonte, as instruções para compilar o bootloader, e alguns binários pré-compilados (na pasta *binaries*). Para escolher o binário adequado para o micro utilizado, siga as instruções do repositório.

O upload do binário (arquivo `.bin`) é realizado através do método serial utilizando um script disponível na instalação do PlatformIO. As ferramentas e scripts de upload ficam localizados dentro da pasta de instalação do PlatformIO (geralmente `~/.platformio`) no caminho `packages/tool-stm32duino`. Nesse diretório, basta executar o script `serial_upload` substituindo os parametros `porta` (sem o `/dev/`) e `caminho`, como o exemplo:

```sh
#./serial_upload porta {upload.altID} {upload.usbID} "caminho"
./serial_upload ttyUSB0 {upload.altID} {upload.usbID} "/home/humanoide/Documents/Hardware/generic_boot20_pc13.bin"
```
