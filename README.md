# RC522 RFID Driver for STM32

A complete and robust RFID/NFC driver for the RC522 module using STM32 HAL library.

## Features

- ✅ Complete MIFARE Classic card support
- ✅ Card detection and UID reading
- ✅ Authentication (Key A/B)
- ✅ Read/Write operations
- ✅ Professional modular design
- ✅ Error handling and validation
- ✅ High-level convenience functions

## Hardware Requirements

- STM32 microcontroller (F4xx series tested)
- RC522 RFID module
- SPI interface connection
- MIFARE Classic cards

## Wiring

| RC522 Pin | STM32 Pin | Description |
|-----------|-----------|-------------|
| VCC       | 3.3V      | Power       |
| GND       | GND       | Ground      |
| SCK       | SPI_SCK   | SPI Clock   |
| MOSI      | SPI_MOSI  | SPI Data Out|
| MISO      | SPI_MISO  | SPI Data In |
| CS        | GPIO_PIN  | Chip Select |

## Quick Start

### 1. Include Files
```c
#include "RC522.h"
```

### 2. Initialize RC522
```c
RC522_InitTypeDef rc522;
SPI_HandleTypeDef hspi1; // Your SPI handle

// Initialize RC522
if (RC522_Init(&rc522, &hspi1, GPIOA, GPIO_PIN_4) == STATUS_OK) {
    printf("RC522 initialized successfully!\r\n");
}
```

### 3. Basic Card Operations
```c
uint8_t uid[5];
uint8_t key[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Default key
uint8_t data[16];

// Check for card and read UID
if (RC522_CheckForCard(&rc522, uid) == STATUS_OK) {
    printf("Card detected! UID: %02X %02X %02X %02X\r\n", 
           uid[0], uid[1], uid[2], uid[3]);
}

// Read from block 4
if (RC522_ReadCardBlock(&rc522, key, 'A', 4, data, NULL) == STATUS_OK) {
    printf("Data read successfully!\r\n");
}

// Write to block 4
uint8_t writeData[] = "Hello RC522!";
if (RC522_WriteCardBlock(&rc522, key, 'A', 4, writeData, NULL) == STATUS_OK) {
    printf("Data written successfully!\r\n");
}
```

## API Reference

### Core Functions

#### `RC522_Init()`
Initialize the RC522 module.
```c
RC522_STATUS_TypeDef RC522_Init(RC522_InitTypeDef * RC, 
                                SPI_HandleTypeDef * _SPI,
                                GPIO_TypeDef * _CSGPIOx, 
                                uint16_t _CSPIN);
```

#### `RC522_DeInit()`
Deinitialize and cleanup RC522 resources.
```c
RC522_STATUS_TypeDef RC522_DeInit(RC522_InitTypeDef * RC);
```

### Card Operations

#### `RC522_CheckForCard()`
Detect if a card is present and get its UID.
```c
RC522_STATUS_TypeDef RC522_CheckForCard(RC522_InitTypeDef * RC, uint8_t * uid);
```

#### `RC522_ReadCardBlock()`
Complete read operation (detect + authenticate + read).
```c
RC522_STATUS_TypeDef RC522_ReadCardBlock(RC522_InitTypeDef * RC,
                                         uint8_t * key,
                                         uint8_t keyType,    // 'A' or 'B'
                                         uint8_t blockAddr,
                                         uint8_t * data_out,
                                         uint8_t * uid_out); // Optional
```

#### `RC522_WriteCardBlock()`
Complete write operation (detect + authenticate + write).
```c
RC522_STATUS_TypeDef RC522_WriteCardBlock(RC522_InitTypeDef * RC,
                                          uint8_t * key,
                                          uint8_t keyType,    // 'A' or 'B'
                                          uint8_t blockAddr,
                                          uint8_t * data_in,
                                          uint8_t * uid_out); // Optional
```

### Low-Level Functions

#### `RC522_Auth()`
Authenticate with MIFARE card.
```c
RC522_STATUS_TypeDef RC522_Auth(RC522_InitTypeDef * RC,
                                uint8_t * uid,
                                uint8_t * key,
                                uint8_t keyType,
                                uint8_t blockAddr);
```

#### `RC522_Read_Card()` / `RC522_Write_Card()`
Direct read/write operations (requires prior authentication).

### Utility Functions

#### `RC522_HALT()`
Send HALT command to put card in sleep mode.
```c
void RC522_HALT(RC522_InitTypeDef * RC);
```

## Status Codes

| Status | Description |
|--------|-------------|
| `STATUS_OK` | Operation successful |
| `STATUS_ERROR` | Operation failed |

## Example Application

```c
#include "RC522.h"
#include "main.h"

int main(void) {
    RC522_InitTypeDef rc522;
    uint8_t uid[5];
    uint8_t key[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t data[16];
    
    // System initialization
    HAL_Init();
    SystemClock_Config();
    MX_SPI1_Init();
    
    // Initialize RC522
    if (RC522_Init(&rc522, &hspi1, GPIOA, GPIO_PIN_4) != STATUS_OK) {
        Error_Handler();
    }
    
    while (1) {
        // Check for cards every 100ms
        if (RC522_CheckForCard(&rc522, uid) == STATUS_OK) {
            printf("Card UID: %02X:%02X:%02X:%02X\r\n", 
                   uid[0], uid[1], uid[2], uid[3]);
            
            // Read block 4
            if (RC522_ReadCardBlock(&rc522, key, 'A', 4, data, NULL) == STATUS_OK) {
                printf("Block 4: %s\r\n", data);
            }
            
            HAL_Delay(1000); // Prevent rapid re-detection
        }
        
        HAL_Delay(100);
    }
}
```

## Block Structure

MIFARE Classic 1K card has 64 blocks (0-63):
- **Blocks 0-2, 4-6, 8-10, ...**: Data blocks
- **Blocks 3, 7, 11, 15, ...**: Sector trailer (keys + access bits)
- **Block 0**: Manufacturer data (read-only)

## Default Keys

Most MIFARE cards use default keys:
- Key A: `FF FF FF FF FF FF`
- Key B: `FF FF FF FF FF FF`

## Contributing

Feel free to submit issues and pull requests to improve this driver.

