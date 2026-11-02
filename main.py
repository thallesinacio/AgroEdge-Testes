# =============================================================================
# main.py - Leitura de piranômetro SN-300AL-RA-N01 via RS-485 / Modbus-RTU
# Placa: BitDogLab (Raspberry Pi Pico W / RP2040) - MicroPython >= 1.20
#
# Ligações (fixas):
#   GPIO8 (UART1 TX) -> DI do MAX485
#   GPIO9 (UART1 RX) <- RO do MAX485 (via divisor 1k/2k)
#   GPIO4            -> DE + RE do MAX485 (1 = transmitir, 0 = receber)
#   GND comum entre placa, MAX485, sensor e fonte de 12 V
#
# Display OLED opcional: copie também o arquivo ssd1306.py para a placa.
# =============================================================================

from machine import UART, Pin, I2C
import time
import struct

# ----------------------------- CONFIGURAÇÃO ---------------------------------
SENSOR_ADDR = 0x01        # endereço Modbus do sensor
BAUD_RATE = 4800          # formato 8N1
REG_RADIACAO = 0x0000     # registrador de radiação (W/m²)

INTERVALO_MS = 2000       # período entre leituras
TIMEOUT_MS = 300          # tempo máximo de espera pela resposta

DEBUG = False             # True = imprime quadros em hexadecimal
SCAN_MODE = False         # True = executa scan() em vez do laço normal
USE_OLED = True           # True = mostra o valor no display SSD1306

# Pinos (não alterar - definidos pelo hardware)
PINO_TX = 8
PINO_RX = 9
PINO_DE_RE = 4

# Constantes do protocolo
FUNC_READ_HOLDING = 0x03

# Pino de direção do MAX485. Começa em 0 (modo recepção) para não ocupar
# o barramento RS-485 enquanto a placa não estiver transmitindo.
de_re = Pin(PINO_DE_RE, Pin.OUT, value=0)


class ModbusErro(Exception):
    """Falha de comunicação Modbus com mensagem descritiva."""
    pass


# ----------------------------- UTILITÁRIOS ----------------------------------
def criar_uart(baud):
    """Cria (ou recria) a UART1 com o baud rate indicado, formato 8N1."""
    return UART(1, baudrate=baud, bits=8, parity=None, stop=1,
                tx=Pin(PINO_TX), rx=Pin(PINO_RX))


def hexstr(dados):
    """Converte bytes em texto hexadecimal, ex.: '01 03 00 00'."""
    return " ".join("{:02X}".format(b) for b in dados)


def crc16(dados):
    """
    CRC-16 Modbus: valor inicial 0xFFFF, polinômio refletido 0xA001.
    Retorna um inteiro de 16 bits; no quadro, o byte baixo vai primeiro.
    """
    crc = 0xFFFF
    for b in dados:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def montar_requisicao(endereco, registrador, quantidade=1):
    """
    Monta o quadro de 8 bytes da função 0x03 (Read Holding Registers):
    [endereço][0x03][reg_hi][reg_lo][qtd_hi][qtd_lo][CRC_lo][CRC_hi]
    """
    corpo = struct.pack(">BBHH", endereco, FUNC_READ_HOLDING,
                        registrador, quantidade)
    # '<H' = little-endian: CRC_lo primeiro, depois CRC_hi
    return corpo + struct.pack("<H", crc16(corpo))


def aguardar_fim_tx(uart, n_bytes, baud):
    """
    Espera o último bit do último byte sair fisicamente pelo pino TX.

    uart.write() apenas copia os dados para o buffer/FIFO e retorna logo;
    se o DE/RE voltasse a 0 nesse momento, o MAX485 cortaria o final do
    quadro. Por isso usamos uart.flush(), que só retorna quando o
    transmissor terminou. Se flush() não existir nesta versão do
    firmware, aguardamos o tempo calculado de transmissão:
        n_bytes * 10 bits (1 start + 8 dados + 1 stop) / baud  + 1 ms
    """
    try:
        uart.flush()
    except (AttributeError, OSError):
        time.sleep_us((n_bytes * 10 * 1000000) // baud + 1000)


# ----------------------------- MODBUS ---------------------------------------
def transacao_modbus(uart, quadro, baud, n_esperado,
                     timeout_ms=TIMEOUT_MS, debug=DEBUG):
    """
    Envia um quadro pelo RS-485 e devolve os bytes recebidos (bytes),
    possivelmente vazio ou incompleto se o tempo esgotar.
    """
    # 1) Descarta bytes residuais no buffer RX (ruído, respostas atrasadas)
    while uart.any() > 0:
        uart.read()

    # 2) DE/RE = 1: habilita o driver do MAX485 (modo transmissão).
    #    Como RE está unido a DE, o receptor fica desligado e a placa
    #    não recebe o "eco" do próprio quadro.
    de_re.value(1)

    # 3) Envia o quadro
    uart.write(quadro)

    # 4) Espera o último byte sair fisicamente da UART
    aguardar_fim_tx(uart, len(quadro), baud)

    # 5) DE/RE = 0 imediatamente: libera o barramento e liga o receptor,
    #    pois o sensor começa a responder poucos milissegundos depois.
    de_re.value(0)

    if debug:
        print("  TX:", hexstr(quadro))

    # 6) Acumula a resposta até ter n_esperado bytes ou estourar o timeout
    resp = b""
    inicio = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), inicio) < timeout_ms:
        if uart.any() > 0:
            pedaco = uart.read()
            if pedaco:
                resp += pedaco
            # Bytes espúrios antes do endereço (comum na troca de direção
            # do MAX485, ex.: 0x00) são descartados
            while resp and resp[0] != quadro[0]:
                resp = resp[1:]
            # Resposta completa
            if len(resp) >= n_esperado:
                break
            # Resposta de exceção (função com bit 0x80) tem só 5 bytes
            if len(resp) >= 5 and (resp[1] & 0x80):
                break
        else:
            time.sleep_ms(1)

    if debug:
        print("  RX:", hexstr(resp) if resp else "(nada)")
    return resp


def ler_radiacao(uart, endereco=SENSOR_ADDR, baud=BAUD_RATE, debug=DEBUG):
    """
    Lê o registrador de radiação e retorna o valor em W/m² (int).
    Levanta ModbusErro com a causa específica em caso de falha.
    Resposta esperada (7 bytes):
    [endereço][0x03][0x02][dado_hi][dado_lo][CRC_lo][CRC_hi]
    """
    quadro = montar_requisicao(endereco, REG_RADIACAO, 1)
    resp = transacao_modbus(uart, quadro, baud, 7, debug=debug)

    if len(resp) == 0:
        raise ModbusErro("timeout (sem resposta)")

    # Resposta de exceção: [endereço][0x83][código][CRC_lo][CRC_hi]
    if len(resp) >= 5 and resp[1] == (FUNC_READ_HOLDING | 0x80):
        exc = resp[:5]
        if crc16(exc[:3]) != (exc[3] | (exc[4] << 8)):
            raise ModbusErro("CRC inválido")
        raise ModbusErro("exceção Modbus código {}".format(exc[2]))

    if len(resp) != 7:
        raise ModbusErro("resposta com tamanho errado ({} bytes)"
                         .format(len(resp)))

    if crc16(resp[:5]) != (resp[5] | (resp[6] << 8)):
        raise ModbusErro("CRC inválido")

    if resp[0] != endereco:
        raise ModbusErro("endereço inesperado (0x{:02X})".format(resp[0]))

    if resp[1] != FUNC_READ_HOLDING:
        raise ModbusErro("código de função inesperado (0x{:02X})"
                         .format(resp[1]))

    if resp[2] != 2:
        raise ModbusErro("byte count inválido ({})".format(resp[2]))

    return (resp[3] << 8) | resp[4]


# ----------------------------- DIAGNÓSTICO ----------------------------------
def scan():
    """
    Testa os baud rates 2400, 4800 e 9600 e os endereços 1 a 10, imprimindo
    toda combinação que responder. Use para descobrir a configuração real
    do sensor e depois ajuste SENSOR_ADDR e BAUD_RATE no topo do arquivo.
    """
    print("=== SCAN: procurando o sensor ===")
    encontrados = []
    for baud in (2400, 4800, 9600):
        print("Testando {} baud...".format(baud))
        uart = criar_uart(baud)        # recria a UART a cada baud rate
        time.sleep_ms(50)
        for endereco in range(1, 11):
            try:
                valor = ler_radiacao(uart, endereco, baud, debug=DEBUG)
                print("  RESPONDEU: baud={} endereço={} -> {} W/m²"
                      .format(baud, endereco, valor))
                encontrados.append((baud, endereco))
            except ModbusErro as e:
                msg = str(e)
                # Exceção Modbus ou CRC inválido também indicam que há
                # um dispositivo nessa combinação
                if not msg.startswith("timeout"):
                    print("  Resposta em baud={} endereço={}: {}"
                          .format(baud, endereco, msg))
                    encontrados.append((baud, endereco))
            # Pausa entre tentativas (silêncio mínimo do Modbus-RTU)
            time.sleep_ms(50)

    print("=== SCAN concluído ===")
    if encontrados:
        for baud, endereco in encontrados:
            print("  -> BAUD_RATE = {}   SENSOR_ADDR = {}".format(baud,
                                                                  endereco))
        print("Ajuste as constantes e defina SCAN_MODE = False.")
    else:
        print("Nenhuma resposta. Verifique alimentação de 12 V, GND comum,")
        print("fios A/B (tente invertê-los) e o divisor no RO.")


# ----------------------------- DISPLAY OLED ---------------------------------
def iniciar_oled():
    """Inicializa o SSD1306; retorna None se desativado ou não encontrado."""
    if not USE_OLED:
        return None
    try:
        import ssd1306
        i2c = I2C(1, sda=Pin(14), scl=Pin(15), freq=400000)
        if 0x3C not in i2c.scan():
            print("OLED não encontrado em 0x3C; seguindo só com print().")
            return None
        oled = ssd1306.SSD1306_I2C(128, 64, i2c, addr=0x3C)
        oled.fill(0)
        oled.text("Piranometro", 20, 0)
        oled.text("Iniciando...", 16, 28)
        oled.show()
        return oled
    except ImportError:
        print("ssd1306.py não encontrado na placa; seguindo só com print().")
    except Exception as e:
        print("Falha ao iniciar OLED ({}); seguindo só com print().".format(e))
    return None


def texto_grande(oled, texto, x, y, escala):
    """Desenha texto ampliado, replicando cada pixel da fonte 8x8."""
    import framebuf
    largura = len(texto) * 8
    buf = bytearray(((largura + 7) // 8) * 8)
    fb = framebuf.FrameBuffer(buf, largura, 8, framebuf.MONO_HLSB)
    fb.text(texto, 0, 0, 1)
    for py in range(8):
        for px in range(largura):
            if fb.pixel(px, py):
                oled.fill_rect(x + px * escala, y + py * escala,
                               escala, escala, 1)


def mostrar_oled(oled, valor, erro=None):
    """Atualiza o display com o valor lido ou com a mensagem de erro."""
    if oled is None:
        return
    try:
        oled.fill(0)
        oled.text("Radiacao solar", 8, 0)
        oled.hline(0, 10, 128, 1)
        if erro is None:
            txt = str(valor)
            escala = 3
            x = max(0, (128 - len(txt) * 8 * escala) // 2)
            texto_grande(oled, txt, x, 18, escala)
            oled.text("W/m2", 48, 50)
        else:
            oled.text("Falha:", 0, 22)
            # Quebra a mensagem em linhas de 16 caracteres
            for i in range(0, min(len(erro), 48), 16):
                oled.text(erro[i:i + 16], 0, 34 + (i // 16) * 10)
        oled.show()
    except Exception as e:
        print("Erro no OLED:", e)


# ----------------------------- PROGRAMA PRINCIPAL ---------------------------
def main():
    print("Piranômetro RS-485 / Modbus-RTU - BitDogLab")
    print("Endereço: {}  Baud: {}  Registrador: 0x{:04X}".format(
        SENSOR_ADDR, BAUD_RATE, REG_RADIACAO))

    if SCAN_MODE:
        scan()
        return

    uart = criar_uart(BAUD_RATE)
    oled = iniciar_oled()

    while True:
        inicio = time.ticks_ms()
        try:
            valor = ler_radiacao(uart)
            print("Radiação solar: {} W/m²".format(valor))
            mostrar_oled(oled, valor)
        except ModbusErro as e:
            print("Falha na leitura:", e)
            mostrar_oled(oled, None, str(e))
        except KeyboardInterrupt:
            raise
        except Exception as e:
            # Qualquer erro inesperado é reportado sem parar o programa
            print("Erro inesperado:", e)

        # Mantém o período de 2 s descontando o tempo gasto na leitura
        gasto = time.ticks_diff(time.ticks_ms(), inicio)
        if gasto < INTERVALO_MS:
            time.sleep_ms(INTERVALO_MS - gasto)


try:
    main()
except KeyboardInterrupt:
    de_re.value(0)
    print("Programa interrompido pelo usuário.")
