# DMA Example

Exemplo [retirado do fórum stm32duino](https://www.stm32duino.com/viewtopic.php?t=3887) de como utilizar DMA para transmitir mensagens por serial no stm32f103c (bluepill). A funcionalidade depende da biblioteca `libmaple` presente no nucleo `maple` do arduino para stm32f1.

Para utilizar o nucleo, é necessário aplicar uma das configurações no arquivo `platformio.ini`:

- `board_build.core = maple` (**Utilizar essa opção**)
- `board_build.variant = custom` (*Workaround*)
- `platform = ststm32@~4.5.0` (*Workaround*)

Esses workarounds foi retirado dessa [resposta no github](https://github.com/platformio/platform-ststm32/issues/76#issuecomment-449624969).

```c
/* Exemple of UART Transmission using DMA
 * Only transmit from memory to peripheral
 * TX only
 * Tested on STM32F103ZET6
 *
 * DMA datasheet :
 * https://www.st.com/content/ccc/resource/technical/document/reference_manual/59/b9/ba/7f/11/af/43/d5/CD00171190.pdf/files/CD00171190.pdf/jcr:content/translations/en.CD00171190.pdf
 * DMA channels : p282
 * USART1_TX : DMA1 / channel 4
 *
 * Libraries used :
 * Flags and defines :
 * https://github.com/rogerclarkmelbourne/Arduino_STM32/blob/845508ee19369de64a52ee86961dd837b6459de6/STM32F1/system/libmaple/include/libmaple/dma.h
 * functions :
 * https://github.com/rogerclarkmelbourne/Arduino_STM32/blob/d4b3cd114ba567dbd917b7373f2160d14dd29fd4/STM32F1/system/libmaple/stm32f1/include/series/dma.h
 *
 * FCam1
 */

#include <Arduino.h>
#include <libmaple/dma.h>
#include <libmaple/usart.h>

#define LENGHT 2
#define dma_bufer_size2 LENGHT
#define dma_bufer_size 1

dma_tube_config tube_config;

uint16_t data = 0x3D;
uint16_t data2[LENGHT];

int p;

usart_reg_map *regs = USART1->regs;  // USART configuration registers

void setup() {
  Serial.begin(9600);   // USB
  Serial1.begin(9600);  // USART 1 TX: PA9 & RX: PA10

  regs->CR3 = USART_CR3_DMAT;  // enable DMA on UART
                               // regs->CR3 = USART_CR1_TXEIE;

  while (!Serial) {
  }
  delay(1000);
  Serial.println("--------------");
  Serial.setTimeout(10);

  data2[0] = 0xF3;
  data2[1] = 0xDE;

  // Setup of general flags
  tube_config.tube_src_size = DMA_SIZE_16BITS;
  tube_config.tube_dst = &(USART1->regs->DR);  // Destination of data
  tube_config.tube_dst_size = DMA_SIZE_8BITS;  // Size of the data register
  tube_config.target_data = 0;
  tube_config.tube_req_src = DMA_REQ_SRC_USART1_TX;  // DMA request source.

  // Setup of specifics flags depending on DMA mode whished
  DMA_Config_Buffer();
  // DMA_Config_OneByte();
  // DMA_Config_Continuous(); // Continuous transfer in background

  dma_init(DMA1);  // Initialization

  int error = dma_tube_cfg(DMA1, DMA_CH4, &tube_config);  // Setup of the DMA

  dma_set_priority(DMA1, DMA_CH4, DMA_PRIORITY_LOW);  // by default

  dma_enable(DMA1, DMA_CH4);  // Enable the channel and start the transfer.

  Serial.print("error:  ");
  Serial.println(error);  // return 0 if no error.
  Serial.print("dma_is_enabled:  ");
  Serial.println(dma_is_enabled(
      DMA1,
      DMA_CH4));  // return 0 if the tube is disabled, >0 if it is enabled.
}

void DMA_Config_Buffer()  // Send a buffer
{
  tube_config.tube_src = &data2;  // Source of the data
  tube_config.tube_nr_xfers = dma_bufer_size2;
  tube_config.tube_flags =
      (DMA_FROM_MEM | DMA_MINC_MODE);  // Read from memory to peripheral |
                                       // Auto-increment memory address
}

void DMA_Config_OneByte()  // Send a byte
{
  tube_config.tube_src = &data;  // Source of the data
  tube_config.tube_nr_xfers = dma_bufer_size;
  tube_config.tube_flags = (DMA_FROM_MEM);  // Read from memory to peripheral
}

void DMA_Config_Continuous()  // Send a byte
{
  tube_config.tube_src = &data;  // Source of the data
  tube_config.tube_nr_xfers = dma_bufer_size;
  tube_config.tube_flags =
      (DMA_FROM_MEM |
       DMA_CIRC_MODE);  // Read from memory to peripheral | Circular mode
}

void loop() {
  data2[0] = p++;
  data2[1] = 0xDE;

  // Repeted transfers and update
  // Useless with DMA_Config_Continuous()
  int error = dma_tube_cfg(DMA1, DMA_CH4, &tube_config);  // return the error
  dma_enable(DMA1, DMA_CH4);  // Enable the channel and start the transfer.

  delay(100);
}
```
