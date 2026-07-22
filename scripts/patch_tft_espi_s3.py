"""
Corrige um bug conhecido da biblioteca TFT_eSPI 2.5.43 no ESP32-S3.

PROBLEMA
--------
Em Processors/TFT_eSPI_ESP32_S3.h, o ramo CONFIG_IDF_TARGET_ESP32S3 define
SPI_PORT como FSPI (o enum do Arduino core, que vale 0 no ESP32-S3/C3).
Mas o SDK (soc.h) usa REG_SPI_BASE(i), que so resolve endereco valido para
i>=2 (indice do periferico GPSPI2/GPSPI3), devolvendo 0 para qualquer valor
menor. Como SPI_USER_REG(i) = REG_SPI_BASE(i) + 0x10, com SPI_PORT=0 o
ponteiro _spi_user fica a apontar para o endereco 0x10 (quase nulo).
Escrever nele em tft->init() causa:
  Guru Meditation Error: Core 1 panic'ed (StoreProhibited)
  #0 TFT_eSPI::begin_tft_write() ... TFT_eSPI.cpp:81

O ramo do ESP32-S2 logo acima ja usa o valor literal correto (2);
este script aplica a mesma correcao ao ramo do ESP32-S3.

Como o ficheiro vive em .pio/libdeps (recriado sempre que a biblioteca e
descarregada de novo), este script corre automaticamente antes de cada
build (extra_scripts em platformio.ini) e e idempotente: se o ficheiro
ja estiver corrigido ou ainda nao existir (primeira instalacao da lib
ainda a decorrer), nao faz nada.
"""

import os

Import("env")  # noqa: F821  (injetado pelo SCons/PlatformIO)

TARGET_FILE = os.path.join(
    env.get("PROJECT_LIBDEPS_DIR"),  # noqa: F821
    env.get("PIOENV"),  # noqa: F821
    "TFT_eSPI", "Processors", "TFT_eSPI_ESP32_S3.h",
)

BUGGY = "  #elif CONFIG_IDF_TARGET_ESP32S3\n    #define SPI_PORT FSPI\n  #endif"
FIXED = (
    "  #elif CONFIG_IDF_TARGET_ESP32S3\n"
    "    // Corrigido por scripts/patch_tft_espi_s3.py: FSPI(=0) nao e um indice\n"
    "    // valido para REG_SPI_BASE() no ESP32-S3, causa crash StoreProhibited\n"
    "    // em tft->init(). Ver comentario completo no proprio script.\n"
    "    #define SPI_PORT 2\n"
    "  #endif"
)


def patch():
    if not os.path.isfile(TARGET_FILE):
        return  # biblioteca ainda nao foi descarregada, nada a fazer

    with open(TARGET_FILE, "r", encoding="utf-8") as f:
        content = f.read()

    if "Corrigido por scripts/patch_tft_espi_s3.py" in content:
        return  # ja aplicado

    if BUGGY not in content:
        print("[patch_tft_espi_s3] aviso: padrao esperado nao encontrado, "
              "a biblioteca pode ter mudado de versao, nada foi alterado.")
        return

    content = content.replace(BUGGY, FIXED)
    with open(TARGET_FILE, "w", encoding="utf-8") as f:
        f.write(content)
    print("[patch_tft_espi_s3] TFT_eSPI_ESP32_S3.h corrigido (SPI_PORT FSPI -> 2)")


patch()
